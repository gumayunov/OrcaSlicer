#include "FinGenerator.hpp"

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Tesselate.hpp"
#include "libslic3r/TriangleMeshSlicer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <optional>

namespace Slic3r {

namespace FinGeneratorInternal {

Transform3d plane_to_bed(const FinFeature& f)
{
    Transform3d m = Transform3d::Identity();
    Matrix3d    R;                                                        // columns: u, w, n
    R.col(0) = Vec3d(-std::sin(f.z_angle), std::cos(f.z_angle), 0.);      // u
    R.col(1) = Vec3d(0., 0., 1.);                                         // w (= Z)
    R.col(2) = Vec3d(std::cos(f.z_angle), std::sin(f.z_angle), 0.);       // n
    m.linear()      = R;
    m.translation() = Vec3d(f.anchor_xy.x(), f.anchor_xy.y(), 0.);
    return m;
}

// plane_to_bed is a rigid transform (orthonormal linear part), so the cheap
// transpose-based isometry inverse applies.
Transform3d bed_to_plane(const FinFeature& f) { return plane_to_bed(f).inverse(Eigen::Isometry); }

indexed_triangle_set object_solid_bed(const ModelObject& object, const Transform3d& to_bed)
{
    indexed_triangle_set out;
    for (const ModelVolume* v : object.volumes) {
        if (!v->is_model_part())
            continue;
        indexed_triangle_set its = v->mesh().its;      // volume-local
        its_transform(its, to_bed * v->get_matrix());  // -> bed frame
        its_merge(out, its);
    }
    return out;
}

int fin_connector_rows(const FinFeature& f)
{
    // +1e-9: fp rounding guard -- e.g. 15.0/0.3 == 49.9999... must not drop a row
    // (conn_step comes from GUI input in Phase 04, so "nice" ratios are common).
    return int(std::floor(f.height / f.conn_step + 1e-9)) + 1;
}

} // namespace FinGeneratorInternal

namespace {

// Builds an indexed_triangle_set with *shared* vertices: every distinct point
// gets one index, so coincident cap/side-wall corners weld into a manifold mesh
// instead of a triangle soup (a soup renders and slices fine, but reports every
// edge as open/non-manifold once attached as a ModelVolume, triggering GUI
// repair warnings). Exact-match keying (no epsilon) is sufficient here:
// coincident points are bit-identical because caps and side walls both derive
// their coordinates from the same unscale<double>() of the same scaled
// ExPolygon points (see Tesselate.cpp for the cap path), and keying on the
// stored float representation additionally tolerates any sub-float-precision
// double noise.
class VertexWelder
{
public:
    explicit VertexWelder(indexed_triangle_set& its) : m_its(its) {}

    void add_triangle(const Vec3d& a, const Vec3d& b, const Vec3d& c)
    {
        m_its.indices.emplace_back(index_of(a), index_of(b), index_of(c));
    }

private:
    int index_of(const Vec3d& p)
    {
        Vec3f pf = p.cast<float>();
        auto [it, inserted] = m_map.emplace(std::array<float, 3>{pf.x(), pf.y(), pf.z()},
                                            int(m_its.vertices.size()));
        if (inserted)
            m_its.vertices.emplace_back(pf);
        return it->second;
    }

    indexed_triangle_set&               m_its;
    std::map<std::array<float, 3>, int> m_map;
};

// Appends a triangle soup (flat list of Vec3d vertices, 3 per triangle, already
// in mm -- triangulate_expolygon(s)_3d() unscale internally, see Tesselate.cpp)
// through the welder, sharing vertices with previously added geometry.
void append_soup(VertexWelder& weld, const std::vector<Vec3d>& soup)
{
    for (size_t i = 0; i + 2 < soup.size(); i += 3)
        weld.add_triangle(soup[i], soup[i + 1], soup[i + 2]);
}

// One side-wall strip for a single ring (contour or hole) of `region`, in the
// (u, w, n) plane frame. Slic3r's ring convention (ExPolygon::contour is CCW,
// holes are CW, see ExPolygon.hpp) makes the filled region lie consistently to
// the *left* of the direction of travel for both ring kinds when walked in
// their stored point order -- so contour and hole rings need no special-casing
// here. For a ring edge p1->p2 = (du, dw), the outward (away from material) 2D
// direction is the -90 deg (clockwise) rotation (dw, -du); the quad
// [(p1,n0),(p2,n0),(p2,n1),(p1,n1)], split into triangles (p1,n0)-(p2,n0)-(p2,n1)
// and (p1,n0)-(p2,n1)-(p1,n1), has cross-product normal h*(dw,-du,0) with
// h = n1-n0 > 0 -- i.e. it already points outward, so no extra winding flip
// is needed for either ring kind.
void add_side_walls_ring(VertexWelder& weld, const Points& ring, double n0, double n1)
{
    size_t n = ring.size();
    if (n < 2)
        return;
    for (size_t i = 0; i < n; ++i) {
        Vec2d p1 = unscale(ring[i]);
        Vec2d p2 = unscale(ring[(i + 1) % n]);
        weld.add_triangle({p1.x(), p1.y(), n0}, {p2.x(), p2.y(), n0}, {p2.x(), p2.y(), n1});
        weld.add_triangle({p1.x(), p1.y(), n0}, {p2.x(), p2.y(), n1}, {p1.x(), p1.y(), n1});
    }
}

void add_side_walls(VertexWelder& weld, const ExPolygon& region, double n0, double n1)
{
    add_side_walls_ring(weld, region.contour.points, n0, n1);
    for (const Polygon& hole : region.holes)
        add_side_walls_ring(weld, hole.points, n0, n1);
}

// Extrudes `region` (u, w plane, scaled) into a closed prism spanning
// n in [n0, n1] (mm), appended to `out` in the (u, w, n) plane frame.
// Caps: bottom (n0) faces -n so its 2D triangulation is flipped; top (n1)
// faces +n so it keeps the natural (CCW) winding -- both already unscaled
// to mm by triangulate_expolygon_3d(), see Tesselate.cpp. All pieces share
// one welder, so the result is a closed shared-vertex mesh (0 open edges).
void add_prism(indexed_triangle_set& out, const ExPolygons& region, double n0, double n1)
{
    VertexWelder weld(out);
    for (const ExPolygon& ep : region) {
        append_soup(weld, triangulate_expolygon_3d(ep, n0, /*flip=*/true));
        append_soup(weld, triangulate_expolygon_3d(ep, n1, /*flip=*/false));
        add_side_walls(weld, ep, n0, n1);
    }
}

// Where the horizontal line y = w (scaled u,w plane) crosses `section`'s ring
// edges (contours and holes alike -- a ring is just a loop of edges, so both
// walk the same way), returning the extreme u among all crossings (max if
// max_side, else min), unscaled to mm. nullopt when no edge crosses the line
// -- i.e. w is strictly above the model's own top (or the model has no
// material at all), the caller's signal to skip a connector tab there
// (design doc §13.3: a row *at* the model's top still gets a tab, only rows
// *above* it don't -- see the "fin taller than the model" test).
//
// Edge-inclusion rule: a non-horizontal edge p1->p2 with y-span
// [y0, y1] = [min(y1,y2), max(y1,y2)] counts as crossing iff y0 <= w <= y1
// (closed on both ends). Horizontal edges (y0 == y1) are dropped outright --
// they would divide by zero computing an intersection u, and don't carry one
// anyway (an edge running *along* the query line has no single crossing
// point; the two adjacent non-horizontal edges already supply the real
// corner points at that height). A closed interval can report the same
// vertex twice when the query line passes exactly through it (both adjacent
// edges include it as an endpoint) -- harmless here since both give the
// identical u, and callers only want the extreme value, not a crossing
// count/parity. Being closed at *both* ends (not a half-open [y0, y1)) is
// what makes both boundary rows behave as intended without any epsilon
// nudge: y0 <= w covers w=0 at the bed (an object's cross-section never dips
// below w=0, so every edge touching the bed has 0 as its lower endpoint),
// and w <= y1 covers w exactly at the model's own top (its upper endpoint).
//
// Shares the multi-component/concave caveat documented at the fin2d rectangle
// in generate_one_fin(): the extreme-u scan can pick the far edge of the wrong
// disjoint piece of the section; same future centroid-filter fix applies.
std::optional<double> contour_u_at(const ExPolygons& section, coord_t w, bool max_side)
{
    std::optional<double> best;
    auto                  scan_ring = [&](const Points& ring) {
        size_t n = ring.size();
        for (size_t i = 0; i < n; ++i) {
            const Point& p1 = ring[i];
            const Point& p2 = ring[(i + 1) % n];
            if (p1.y() == p2.y())
                continue; // horizontal edge: no well-defined single crossing u
            coord_t y0 = std::min(p1.y(), p2.y());
            coord_t y1 = std::max(p1.y(), p2.y());
            if (w < y0 || w > y1)
                continue; // outside [y0, y1]
            double t = double(w - p1.y()) / double(p2.y() - p1.y());
            double u = double(p1.x()) + t * double(p2.x() - p1.x());
            if (!best || (max_side ? u > *best : u < *best))
                best = u;
        }
    };
    for (const ExPolygon& ep : section) {
        scan_ring(ep.contour.points);
        for (const Polygon& hole : ep.holes)
            scan_ring(hole.points);
    }
    return best ? std::make_optional(unscale<double>(*best)) : std::nullopt;
}

} // namespace

TriangleMesh generate_one_fin(const indexed_triangle_set& solid_bed_frame,
                               const FinFeature&           fin_bed_frame,
                               double                       layer_height,
                               double                       line_width)
{
    using namespace FinGeneratorInternal;

    // Slice the solid in the fin's plane frame, where the fin plane is Z' = 0
    // (local z = n, the plane normal), to get the cross-section contour in the
    // (u, w) plane. params.trafo applies the frame change during slicing itself
    // (single vertex pass, no mesh copy) -- the idiom used by PrintObject slicing.
    MeshSlicingParamsEx params;
    params.trafo = bed_to_plane(fin_bed_frame);
    // zs has exactly one entry, so slice_mesh_ex() always returns a vector of size 1
    // (with an empty ExPolygons inside when the plane misses the solid) -- .front() is safe.
    ExPolygons section = slice_mesh_ex(solid_bed_frame, std::vector<float>{0.f}, params).front(); // (u, w), scaled
    if (section.empty())
        return TriangleMesh(); // plane doesn't intersect the solid -> no fin body

    // 2D fin region: a rectangle on the +u side of the section (from the section's
    // u-midpoint out to `length` beyond its far edge, 0..`height` tall -- 0 = bed by
    // contract), truncated by the section contour offset outward by `conn_gap`. This
    // keeps the fin following the object's silhouette with a small standoff gap, and
    // the flip (z_angle += PI, tested in test_fin.cpp) swaps which side is +u, so this
    // single code path (always "+u side of the section bbox") handles both sides.
    // Section cross-sections seen so far are convex (single rectangle), so the bbox
    // midpoint split is sufficient. It can pick the wrong u on a concave section, or
    // on a disjoint multi-component one (e.g. two separate legs crossing the plane),
    // where the bbox midpoint may bisect the wrong side; the fix for both is the
    // same: keep the diff components whose centroid u exceeds the section's centroid
    // u (not needed by current tests).
    BoundingBox bb      = get_extents(section);
    coord_t     u_split = bb.center().x();
    coord_t     u_out   = bb.max.x() + scaled(fin_bed_frame.length);
    coord_t     w_top   = scaled(fin_bed_frame.height);
    Polygon     rect{{u_split, coord_t(0)}, {u_out, coord_t(0)}, {u_out, w_top}, {u_split, w_top}};

    ExPolygons contour_gap = offset_ex(section, scaled<float>(fin_bed_frame.conn_gap));
    ExPolygons fin2d       = diff_ex(Polygons{rect}, contour_gap);
    if (fin2d.empty())
        return TriangleMesh(); // gap swallowed the whole rectangle -> no fin body

    // Extrude fin2d into a solid prism along n (the plane normal), centered on the
    // fin plane: n in [-thickness/2, +thickness/2].
    indexed_triangle_set fin_local; // plane-frame coords (x=u, y=w, z=n), mm
    add_prism(fin_local, fin2d, -fin_bed_frame.thickness / 2., fin_bed_frame.thickness / 2.);

    // Connector tabs (T00-01-05): small boxes bridging conn_gap, one row every
    // conn_step mm along w, starting at the bed (w=0) up to and including
    // height. Each tab is centered on the fin plane (n in [-line_width,
    // +line_width], i.e. 2 perimeters wide) and reaches kTabOverlap past both
    // the model's own surface and the fin wall's near face, so it overlaps both
    // even though its_merge() below appends it as a topologically separate
    // shell (no shared vertices with the wall -- see the VertexWelder comment
    // above; that's fine here, tabs need not be welded to slice and print).
    // A row with no material at that height (w above the model's own top)
    // gets no tab -- contour_u_at() returns nullopt there (design doc §13.3).
    constexpr double kTabOverlap = 0.1; // mm past the model surface / wall face, each side
    int n_rows = fin_connector_rows(fin_bed_frame);
    for (int k = 0; k < n_rows; ++k) {
        double                w         = k * fin_bed_frame.conn_step;
        std::optional<double> u_surface = contour_u_at(section, scaled(w), /*max_side=*/true);
        if (!u_surface)
            continue;
        indexed_triangle_set tab = its_make_cube(fin_bed_frame.conn_gap + 2.0 * kTabOverlap, // u: span the gap + overlaps
                                                  layer_height,                              // w: exactly one layer tall
                                                  2.0 * line_width);                         // n: two perimeters thick
        its_translate(tab, Vec3f(float(*u_surface - kTabOverlap), float(w), float(-line_width)));
        its_merge(fin_local, tab);
    }

    its_transform(fin_local, plane_to_bed(fin_bed_frame)); // plane frame -> bed frame
    return TriangleMesh(fin_local);
}

std::vector<TriangleMesh> generate_fin_meshes(const ModelObject& o, const Transform3d& to_bed,
                                               double              layer_height,
                                               double              line_width)
{
    using namespace FinGeneratorInternal;

    std::vector<TriangleMesh> out;

    // Cheap early-out: an object with no enabled fins must do no work at all (no solid
    // assembly below) -- callers rely on this to make the disabled/no-fin path a true
    // no-op (see TransientFinGuard in PrintObjectSlice.cpp).
    if (std::none_of(o.fins.begin(), o.fins.end(), [](const FinFeature& f) { return f.enabled; }))
        return out;

    indexed_triangle_set solid = object_solid_bed(o, to_bed);

    for (const FinFeature& f : o.fins) {
        if (!f.enabled)
            continue;

        // object-local anchor/azimuth -> bed frame (design §13.1-13.2):
        // anchor: project the transformed anchor point to the bed plane.
        // azimuth: heading of the transformed u-axis projected onto the bed plane.
        Vec3d a_bed = to_bed * Vec3d(f.anchor_xy.x(), f.anchor_xy.y(), 0.);
        Vec3d u_obj(-std::sin(f.z_angle), std::cos(f.z_angle), 0.);
        Vec3d u_bed = to_bed.linear() * u_obj;
        u_bed.z()   = 0.;
        if (u_bed.norm() < EPSILON)
            continue; // degenerate (plane axis became vertical) -> skip fin

        FinFeature g = f;
        g.anchor_xy = Vec2d(a_bed.x(), a_bed.y());
        g.z_angle   = std::atan2(-u_bed.x(), u_bed.y()); // inverse of u = (-sin a, cos a)

        TriangleMesh mesh = generate_one_fin(solid, g, layer_height, line_width);
        if (!mesh.empty())
            out.push_back(std::move(mesh));
    }
    return out;
}

} // namespace Slic3r
