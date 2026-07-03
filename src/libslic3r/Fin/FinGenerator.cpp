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

int fin_connector_rows(const FinFeature& f, double layer_height)
{
    // Clamp the row spacing to at least one layer (phase 05 edge case -- see the doc comment in
    // FinGenerator.hpp): must match generate_one_fin()'s own effective_step exactly, or the row
    // count computed here and the row heights it actually emits tabs at would disagree.
    double effective_step = std::max(f.conn_step, layer_height);
    // +1e-9: fp rounding guard -- e.g. 15.0/0.3 == 49.9999... must not drop a row
    // (conn_step comes from GUI input in Phase 04, so "nice" ratios are common).
    return int(std::floor(f.height / effective_step + 1e-9)) + 1;
}

std::optional<FinFeature> fin_object_to_bed(const FinFeature& f, const Transform3d& to_bed)
{
    // object-local anchor/azimuth -> bed frame (design §13.1-13.2):
    // anchor: the transformed anchor point, XY only (z dropped, not projected along any
    // normal -- to_bed already carries the instance's full 3D transform).
    // azimuth: heading of the transformed u-axis, XY only (z dropped the same way).
    Vec3d a_bed = to_bed * Vec3d(f.anchor_xy.x(), f.anchor_xy.y(), 0.);
    Vec3d u_obj(-std::sin(f.z_angle), std::cos(f.z_angle), 0.);
    Vec3d u_bed = to_bed.linear() * u_obj;
    u_bed.z()   = 0.;
    if (u_bed.norm() < EPSILON)
        return std::nullopt; // degenerate (plane axis became vertical)

    FinFeature g = f;
    g.anchor_xy = Vec2d(a_bed.x(), a_bed.y());
    g.z_angle   = std::atan2(-u_bed.x(), u_bed.y()); // inverse of u = (-sin a, cos a)
    return g;
}

std::optional<FinFeature> fin_bed_to_object(const FinFeature& f_bed, const Transform3d& to_bed)
{
    // Both anchor and azimuth invert through the upper-left 2x2 block of to_bed's linear part:
    // fin_object_to_bed() maps an object-local (x, y[, implicit 0]) through to_bed and keeps
    // only the resulting XY, so inverting it for a *point* means solving
    //   A * (x, y) = a_bed.xy - translation.xy
    // and for a *direction* (the azimuth's u-axis, translation-invariant) means solving
    //   A * (x, y) ~ u_bed.xy  (proportional -- fin_object_to_bed() only reads u_bed's heading)
    // where A is that same 2x2 block. Guard on its determinant: a singular A means to_bed
    // squashes the object-local XY plane edge-on to the bed, and neither point can be solved.
    const Matrix3d linear = to_bed.linear();
    Matrix2d       A;
    A << linear(0, 0), linear(0, 1), linear(1, 0), linear(1, 1);
    const double det = A.determinant();
    if (std::abs(det) < EPSILON)
        return std::nullopt; // to_bed's XY block is (near) singular

    const Matrix2d A_inv = A.inverse();

    const Vec2d t_xy = to_bed.translation().head<2>();
    const Vec2d xy   = A_inv * (f_bed.anchor_xy - t_xy);

    const Vec2d u_bed_xy = Vec2d(-std::sin(f_bed.z_angle), std::cos(f_bed.z_angle));
    Vec2d       v        = A_inv * u_bed_xy;
    if (v.norm() < EPSILON)
        return std::nullopt; // shouldn't happen once A is invertible and u_bed_xy != 0, but stay defensive
    v.normalize();

    FinFeature g = f_bed;
    g.anchor_xy = xy;
    g.z_angle   = std::atan2(-v.x(), v.y()); // inverse of u = (-sin a, cos a), mirrors fin_object_to_bed()
    return g;
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
// Shares the multi-component/concave caveat documented at the shadow construction
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

// Sweeps every outer contour of `src` along the straight segment from (0,0) to `delta` (scaled mm,
// u/w plane) via Clipper's Minkowski sum (ClipperLib::MinkowskiSum, vendored in
// deps_src/clipper/clipper.hpp), unioning the per-contour results together. Used for both the +u
// "shadow" sweep and the downward "fill to bed" sweep of generate_one_fin() (design doc §15) --
// same primitive, different delta.
//
// Holes are intentionally dropped (only ep.contour is swept, never ep.holes): sweeping just the
// outer contour reproduces the *filled* polygon's own Minkowski sum. Verified empirically (see the
// scratch harness used while implementing this): MinkowskiSum(pattern, contour, closed=true) on a
// square with a central square hole -- feeding only the outer contour -- yields a solid swept
// rectangle, not a grooved one, because material swept in from the near side of where the hole used
// to be closes the gap for any sweep length reaching past the hole's own width in the sweep
// direction, which both call sites below guarantee (their `delta` always reaches past the window's
// far edge / the bed, plus a margin). Consequence: a hole can never punch a "window" into the shadow
// or the fill -- the outer silhouette alone determines both, matching the design's requirement that
// holes can't create fin material where none should be.
ExPolygons sweep_ex(const ExPolygons& src, const Point& delta)
{
    ClipperLib::Path pattern{Point(0, 0), delta};
    Polygons         out;
    for (const ExPolygon& ep : src) {
        ClipperLib::Paths solution;
        ClipperLib::MinkowskiSum(pattern, ep.contour.points, solution, /*pathIsClosed=*/true);
        for (const ClipperLib::Path& p : solution)
            out.emplace_back(p);
    }
    return union_ex(out);
}

} // namespace

TriangleMesh generate_one_fin(const indexed_triangle_set& solid_bed_frame,
                               const FinFeature&           fin_bed_frame,
                               double                       layer_height,
                               double                       line_width)
{
    using namespace FinGeneratorInternal;

    // Defensive hard guard (T00-01-03 review note + phase 05 plan): the GUI already clamps these
    // to sane minimums (GLGizmoFin.cpp's MinFinDimension/MinFinParamValue), but .3mf/API callers
    // can feed a FinFeature with arbitrary values -- e.g. a hand-edited or corrupted project file.
    // A non-positive length/height would otherwise flip the 2D rect's winding without crashing
    // Clipper, producing wrong-but-non-empty geometry instead of the "no fin" the caller should
    // get; conn_step <= 0 would divide by zero / infinite-loop fin_connector_rows(); thickness <=
    // 0 or conn_gap < 0 make no physical sense either. Bail out before slicing the solid at all.
    if (fin_bed_frame.length <= 0.0 || fin_bed_frame.height <= 0.0 || fin_bed_frame.thickness <= 0.0 ||
        fin_bed_frame.conn_gap < 0.0 || fin_bed_frame.conn_step <= 0.0)
        return TriangleMesh();

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

    // 2D fin region: gizmo window + part shadow (design doc §15, revoking the earlier
    // bbox-rectangle model in §6/§13). The gizmo rectangle window = [0,L] x [0,H] (anchor at
    // u=0, i.e. the plane frame's own origin -- w=0 is the bed by contract) is the literal bound
    // of where fin material may exist: WYSIWYG with the drawn gizmo rectangle. Within it,
    // material is restricted to the part's "+u shadow" (never on the near side of the part, never
    // above it), then filled straight down to the bed, then re-clipped to the gap clearance so
    // overhanging part regions still keep their standoff. The flip (z_angle += PI, tested in
    // test_fin.cpp) swaps which side is +u, so this single code path handles both sides.
    BoundingBox bb        = get_extents(section);
    coord_t     model_top = bb.max.y(); // part's own top at this plane (w, scaled)
    coord_t     w_top     = std::min(scaled(fin_bed_frame.height), model_top);
    if (w_top <= 0)
        return TriangleMesh(); // window has no usable height at this plane
    coord_t u_out = scaled(fin_bed_frame.length);
    Polygon window_rect{{coord_t(0), coord_t(0)}, {u_out, coord_t(0)}, {u_out, w_top}, {coord_t(0), w_top}};

    ExPolygons offset_sec = offset_ex(section, scaled<float>(fin_bed_frame.conn_gap));

    // Shadow: sweep the section's own silhouette along +u far enough to clear the window's far
    // edge from the section's own nearest point (bb.min.x()), plus a 1mm margin -- comfortably
    // covers the "sweep_len >= L + section width + margin" requirement regardless of how far the
    // section sits from the window. sweep_ex() sweeps outer contours only, so holes can't punch a
    // window into the shadow (see its doc comment).
    coord_t    sweep_len_u = std::max(scaled(1.0), u_out - bb.min.x() + scaled(1.0));
    ExPolygons far_shadow  = sweep_ex(section, Point(sweep_len_u, coord_t(0)));

    ExPolygons candidate = intersection_ex(Polygons{window_rect}, far_shadow);
    candidate            = diff_ex(candidate, offset_sec);
    if (candidate.empty())
        return TriangleMesh(); // window entirely inside the part+gap, or entirely on its near side

    // Stand on the bed: sweep straight down past w=0 (sweep_len_w = w_top + margin always clears
    // it), then re-clip to the window and re-carve the gap clearance -- otherwise an overhanging
    // part region would get paved over by material filled down from a taller, thinner region
    // directly above it (the two diff_ex() calls against offset_sec are deliberately not merged
    // into one: the first bounds the shadow before it's filled downward, the second re-applies
    // the same clearance to whatever the fill-down step produced).
    coord_t    sweep_len_w = w_top + scaled(1.0);
    ExPolygons filled      = sweep_ex(candidate, Point(coord_t(0), -sweep_len_w));

    ExPolygons fin2d = intersection_ex(Polygons{window_rect}, filled);
    fin2d            = diff_ex(fin2d, offset_sec);
    if (fin2d.empty())
        return TriangleMesh(); // gap re-carve consumed everything the fill-down step produced

    // Extrude fin2d into a solid prism along n (the plane normal), centered on the
    // fin plane: n in [-thickness/2, +thickness/2].
    indexed_triangle_set fin_local; // plane-frame coords (x=u, y=w, z=n), mm
    add_prism(fin_local, fin2d, -fin_bed_frame.thickness / 2., fin_bed_frame.thickness / 2.);

    // Connector tabs (design doc §15, revoking the earlier "always conn_gap wide" rule): small
    // boxes bridging the fin's inner edge to the part's surface, one row every effective_step mm
    // along w (effective_step = max(conn_step, layer_height), see fin_connector_rows()'s doc
    // comment -- a conn_step smaller than one layer would otherwise request duplicate,
    // vertically-overlapping tab rows), starting at the bed (w=0) up to and including height. A
    // row gets a tab only if (a) the section has material at that height (contour_u_at() returns
    // a value) and (b) the part's surface is within conn_gap + kTabReachSlack of the fin's own
    // inner edge -- the only way it isn't is the surface lying beyond the window's near edge
    // ("part further out than the gizmo allows"), which the user confirmed should get no tab.
    // kTabReachSlack tolerates the two independent computations (this closed-form d(w) vs. the
    // fin2d polygon boolean ops) landing a hair apart. A row can also have no fin material at all
    // (e.g. the window is fully consumed at that height) -- checked directly against fin2d rather
    // than re-derived, since deriving it from d(w) alone would miss multi-component sections (see
    // contour_u_at()'s own caveat -- also the reason tabs measure against the OUTERMOST lobe of a
    // multi-lobe section, not necessarily the nearest one; see the "two disjoint pillars" test).
    //
    // Penetration depths (2026-07-03 user report, progress.md): each side of the tab must reach
    // past its own "surface" by more than the old flat kTabOverlap=0.1mm, for two independent
    // reasons --
    //  - Model side: 0.1mm can land inside the model's own outer perimeter LINE (not yet solid
    //    material), so the tab's first extrusions have nothing under them and slice as an
    //    overhang ("in the air" in the G-code preview). `line_width` is the same representative
    //    wall width the caller already resolves for the tab's own thickness (2*line_width below;
    //    see PrintObjectSlice.cpp's TransientFinGuard) -- reusing it here guarantees model-side
    //    penetration is at least one full wall line deep, never less than the old kTabOverlap.
    //  - Fin side: on a sloped inner edge (a tilted part), the fin's own inner face recedes by
    //    roughly slope*layer_height over the tab's own (single-layer) height, so a fixed 0.1mm
    //    can undershoot and miss the fin body entirely at steep slopes. A conservative constant
    //    depth of layer_height + kTabOverlap covers any slope without computing the real per-row
    //    value; clamped to the window's own outer edge (u = length) so a thin fin (length close
    //    to conn_gap) never has its tab reach past material that was never generated.
    constexpr double kTabOverlap    = 0.1; // mm past the model surface / fin inner edge, floor value (each side)
    constexpr double kTabReachSlack = 0.2; // mm tolerance for the d(w) <= gap reach check
    // effective_conn_step (phase 05 edge case): a conn_step smaller than one layer would request
    // duplicate/vertically-overlapping tab rows -- must match fin_connector_rows()'s own clamp.
    double effective_conn_step = std::max(fin_bed_frame.conn_step, layer_height);
    int    n_rows              = fin_connector_rows(fin_bed_frame, layer_height);
    for (int k = 0; k < n_rows; ++k) {
        double                 w         = k * effective_conn_step;
        coord_t                w_scaled  = scaled(w);
        std::optional<double>  u_surface = contour_u_at(section, w_scaled, /*max_side=*/true);
        if (!u_surface)
            continue; // no material at this height -- nothing to bridge

        double inner = std::min(std::max(*u_surface + fin_bed_frame.conn_gap, 0.0), fin_bed_frame.length);
        double d     = inner - *u_surface;
        if (d > fin_bed_frame.conn_gap + kTabReachSlack)
            continue; // surface lies beyond the window's near edge -- gizmo doesn't reach that far
        if (!contour_u_at(fin2d, w_scaled, /*max_side=*/true))
            continue; // fin2d itself has no material at this row (e.g. window fully consumed)

        double model_overlap = std::max(kTabOverlap, line_width);                    // into the part
        double fin_overlap   = layer_height + kTabOverlap;                           // into the fin body
        double tab_start     = *u_surface - model_overlap;
        double tab_end       = std::min(inner + fin_overlap, fin_bed_frame.length);  // never past the window's own edge
        double tab_len       = tab_end - tab_start;
        if (tab_len <= 0.0)
            continue; // defensive: degenerate row (see contour_u_at()'s multi-component caveat)

        indexed_triangle_set tab = its_make_cube(tab_len,           // u: spans surface-model_overlap..inner+fin_overlap (clamped)
                                                  layer_height,      // w: exactly one layer tall
                                                  2.0 * line_width); // n: two perimeters thick
        its_translate(tab, Vec3f(float(tab_start), float(w), float(-line_width)));
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

        std::optional<FinFeature> g = fin_object_to_bed(f, to_bed);
        if (!g)
            continue; // degenerate (plane axis became vertical) -> skip fin

        TriangleMesh mesh = generate_one_fin(solid, *g, layer_height, line_width);
        if (!mesh.empty())
            out.push_back(std::move(mesh));
    }
    return out;
}

} // namespace Slic3r
