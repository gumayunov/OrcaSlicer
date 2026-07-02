#include "FinGenerator.hpp"

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

    // PLACEHOLDER, to be replaced by T00-01-03/04: the real fin body is a truncated
    // rectangle following the contour (rect - offset(contour)), extruded to `thickness`
    // along n and mapped back to the bed frame. For now, just prove the slicing step and
    // the empty/non-empty contract by triangulating the raw cross-section into a
    // zero-thickness slab lying on the fin plane (n = 0), then mapping it to the bed frame.
    indexed_triangle_set placeholder;
    std::vector<Vec3d>   triangles = triangulate_expolygons_3d(section, 0.); // (u, w, n=0), mm
    placeholder.vertices.reserve(triangles.size());
    placeholder.indices.reserve(triangles.size() / 3);
    for (size_t i = 0; i + 2 < triangles.size(); i += 3) {
        int base = int(placeholder.vertices.size());
        placeholder.vertices.emplace_back(triangles[i].cast<float>());
        placeholder.vertices.emplace_back(triangles[i + 1].cast<float>());
        placeholder.vertices.emplace_back(triangles[i + 2].cast<float>());
        placeholder.indices.emplace_back(base, base + 1, base + 2);
    }
    its_transform(placeholder, plane_to_bed(fin_bed_frame)); // plane frame -> bed frame
    return TriangleMesh(placeholder);
}

} // namespace Slic3r
