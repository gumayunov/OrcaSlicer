#include "FinGenerator.hpp"

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Tesselate.hpp"
#include "libslic3r/TriangleMeshSlicer.hpp"

#include <cmath>

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

} // namespace FinGeneratorInternal

namespace {

// Appends a triangle soup (flat list of Vec3d vertices, 3 per triangle, already
// in mm -- triangulate_expolygon(s)_3d() unscale internally, see Tesselate.cpp)
// to `out`, without deduplicating vertices shared between soups (side walls and
// caps meet only at edges/corners; welding is not needed for a valid mesh here).
void append_soup(indexed_triangle_set& out, const std::vector<Vec3d>& soup)
{
    out.vertices.reserve(out.vertices.size() + soup.size());
    out.indices.reserve(out.indices.size() + soup.size() / 3);
    for (size_t i = 0; i + 2 < soup.size(); i += 3) {
        int base = int(out.vertices.size());
        out.vertices.emplace_back(soup[i].cast<float>());
        out.vertices.emplace_back(soup[i + 1].cast<float>());
        out.vertices.emplace_back(soup[i + 2].cast<float>());
        out.indices.emplace_back(base, base + 1, base + 2);
    }
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
void add_side_walls_ring(indexed_triangle_set& out, const Points& ring, double n0, double n1)
{
    size_t n = ring.size();
    if (n < 2)
        return;
    std::vector<Vec3d> quad;
    quad.reserve(6);
    for (size_t i = 0; i < n; ++i) {
        Vec2d p1 = unscale(ring[i]);
        Vec2d p2 = unscale(ring[(i + 1) % n]);
        quad.clear();
        quad.emplace_back(p1.x(), p1.y(), n0);
        quad.emplace_back(p2.x(), p2.y(), n0);
        quad.emplace_back(p2.x(), p2.y(), n1);
        quad.emplace_back(p1.x(), p1.y(), n0);
        quad.emplace_back(p2.x(), p2.y(), n1);
        quad.emplace_back(p1.x(), p1.y(), n1);
        append_soup(out, quad);
    }
}

void add_side_walls(indexed_triangle_set& out, const ExPolygon& region, double n0, double n1)
{
    add_side_walls_ring(out, region.contour.points, n0, n1);
    for (const Polygon& hole : region.holes)
        add_side_walls_ring(out, hole.points, n0, n1);
}

// Extrudes `region` (u, w plane, scaled) into a closed prism spanning
// n in [n0, n1] (mm), appended to `out` in the (u, w, n) plane frame.
// Caps: bottom (n0) faces -n so its 2D triangulation is flipped; top (n1)
// faces +n so it keeps the natural (CCW) winding -- both already unscaled
// to mm by triangulate_expolygon_3d(), see Tesselate.cpp.
void add_prism(indexed_triangle_set& out, const ExPolygons& region, double n0, double n1)
{
    for (const ExPolygon& ep : region) {
        append_soup(out, triangulate_expolygon_3d(ep, n0, /*flip=*/true));
        append_soup(out, triangulate_expolygon_3d(ep, n1, /*flip=*/false));
        add_side_walls(out, ep, n0, n1);
    }
}

} // namespace

TriangleMesh generate_one_fin(const indexed_triangle_set& solid_bed_frame,
                               const FinFeature&           fin_bed_frame,
                               double                       layer_height,
                               double                       line_width)
{
    // layer_height/line_width drive the connector tabs (T00-01-05); unused until then.
    (void) layer_height;
    (void) line_width;

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
    // fin plane: n in [-thickness/2, +thickness/2]. Connector tabs are added later
    // (T00-01-05), before the transform to the bed frame.
    indexed_triangle_set fin_local; // plane-frame coords (x=u, y=w, z=n), mm
    add_prism(fin_local, fin2d, -fin_bed_frame.thickness / 2., fin_bed_frame.thickness / 2.);
    its_transform(fin_local, plane_to_bed(fin_bed_frame)); // plane frame -> bed frame
    return TriangleMesh(fin_local);
}

} // namespace Slic3r
