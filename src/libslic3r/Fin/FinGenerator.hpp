#pragma once

#include "libslic3r/Model.hpp"
#include "libslic3r/TriangleMesh.hpp"

namespace Slic3r {

// Build a single fin from an explicit solid given in the BED frame (z=0 = bed).
// Returns an empty mesh if the plane does not intersect the solid.
TriangleMesh generate_one_fin(const indexed_triangle_set& solid_bed_frame,
                               const FinFeature&           fin_bed_frame,
                               double                       layer_height,
                               double                       line_width);

// Internal helpers of FinGenerator, exposed (non-static) so unit tests can pin down
// the coordinate contract (design doc §13) and so later fin-generation phases
// (T00-01-02..05) can reuse them. Not part of the module's stable public API.
namespace FinGeneratorInternal {

// Plane-frame -> bed-frame transform for a fin feature.
// Local axes: x = u (horizontal, fin extends outward along +u), y = w (= Z, height),
// z = n (horizontal plane normal). At z_angle = 0: u = +Y world, n = +X world.
Transform3d plane_to_bed(const FinFeature& f);

// Bed-frame -> plane-frame transform (inverse of plane_to_bed).
Transform3d bed_to_plane(const FinFeature& f);

// Merge all MODEL_PART volume meshes of an object into one bed-frame solid.
// `to_bed` is the transform mapping object-local coordinates to the bed frame
// (pass Transform3d::Identity() when the object is already given in the bed frame).
indexed_triangle_set object_solid_bed(const ModelObject& object, const Transform3d& to_bed);

} // namespace FinGeneratorInternal

} // namespace Slic3r
