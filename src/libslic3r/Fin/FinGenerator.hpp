#pragma once

#include "libslic3r/Point.hpp"
// Complete type needed: generate_one_fin() returns TriangleMesh by value
// (also provides indexed_triangle_set).
#include "libslic3r/TriangleMesh.hpp"

#include <optional>
#include <vector>

namespace Slic3r {

class ModelObject;
struct FinFeature;

// Build a single fin from an explicit solid given in the BED frame (z=0 = bed).
// Returns an empty mesh if the plane does not intersect the solid.
TriangleMesh generate_one_fin(const indexed_triangle_set& solid_bed_frame,
                               const FinFeature&           fin_bed_frame,
                               double                       layer_height,
                               double                       line_width);

// Build meshes for all of `o`'s fins. `to_bed` is the transform mapping
// `o`'s object-local coordinates to the bed frame (z=0 = bed) -- e.g. a PrintObject's
// trafo_centered(). The returned meshes are themselves already in that BED frame.
// Fins whose plane misses the object's solid (empty result) are
// skipped, so the returned vector may be shorter than o.fins or empty.
std::vector<TriangleMesh> generate_fin_meshes(const ModelObject& o, const Transform3d& to_bed,
                                               double              layer_height,
                                               double              line_width);

// Internal helpers of FinGenerator, exposed (non-static) so unit tests can pin down
// the coordinate contract. Not part of the module's stable public API.
namespace FinGeneratorInternal {

// Plane-frame -> bed-frame transform for an explicit anchor/azimuth pair.
// Local axes: x = u (horizontal, fin extends outward along +u), y = w (= Z, height),
// z = n (horizontal plane normal). At z_angle = 0: u = +Y world, n = +X world. This is the
// primitive the FinFeature overload below builds on; it also lets callers that track the
// plane's anchor/azimuth outside of a FinFeature (GLGizmoFin::plane_matrix(), mid-drag)
// reuse the same formula instead of hand-mirroring it.
Transform3d plane_to_bed(const Vec2d& anchor_xy, double z_angle);

// Single home for the (-sin a, cos a) u-axis convention plane_to_bed() pins down:
// bed-frame azimuth <-> horizontal direction vector.
Vec3d  dir_from_z_angle(double z_angle);   // (-sin a, cos a, 0)
double z_angle_from_dir(const Vec3d& dir); // atan2(-dir.x, dir.y); ignores dir.z

// Plane-frame -> bed-frame transform for a fin feature. Valid for a BED-FRAME FinFeature
// only (anchor.z == 0, dir horizontal) -- exactly the form fin_object_to_bed() produces and
// generate_one_fin() consumes; every other FinFeature field is irrelevant to the plane's own
// frame.
Transform3d plane_to_bed(const FinFeature& f);

// Bed-frame -> plane-frame transform (inverse of plane_to_bed).
Transform3d bed_to_plane(const FinFeature& f);

// Merge all MODEL_PART volume meshes of an object into one bed-frame solid.
// `to_bed` is the transform mapping object-local coordinates to the bed frame
// (pass Transform3d::Identity() when the object is already given in the bed frame).
indexed_triangle_set object_solid_bed(const ModelObject& object, const Transform3d& to_bed);

// Number of connector-tab rows: one every effective_step mm along the fin's height, starting at
// w=0 (the bed) up to and including height, where effective_step = max(f.conn_step, layer_height).
// The clamp matters: each tab is one layer tall, so a conn_step smaller than one layer would
// request duplicate, vertically-overlapping tab rows at effectively the same height. Clamping the
// SPACING (not the row count) keeps the fin's own top row intact. A row with no material at that
// height (above the model's own top) is still counted here -- it is generate_one_fin() that skips
// emitting a tab for it.
int fin_connector_rows(const FinFeature& f, double layer_height);

// Object-local -> bed-frame conversion of `f`'s anchor/dir only; every other field passes
// through unchanged. This is generate_fin_meshes()'s per-fin conversion step, factored out here
// so it is unit-testable and shared with its inverse below (the gizmo uses both directions:
// this one to display an existing fin, the other to persist an edit). Returns nullopt when
// `to_bed` degenerates the object-local dir onto world Z (e.g. the instance is tipped so the
// fin's plane axis becomes vertical) -- the caller's signal to skip the fin. An instance
// rotated 90deg about X/Y is not degenerate by itself -- only a fin whose OWN u axis maps
// vertical is.
std::optional<FinFeature> fin_object_to_bed(const FinFeature& f, const Transform3d& to_bed);

// Bed-frame -> object-local conversion of `f_bed`'s anchor/dir (inverse of fin_object_to_bed).
// Both anchor and dir invert through the whole of `to_bed` (its full inverse for the point, its
// linear part's inverse for the direction), so that for any invertible `to_bed`:
//   fin_object_to_bed(*fin_bed_to_object(f_bed, to_bed), to_bed)
// reproduces f_bed's anchor/dir exactly -- including instances rotated ~90deg about X or Y,
// where a 2D XY-block solve would be singular and would silently drop every gizmo commit for
// an object stood on its side. Returns nullopt only when `to_bed`'s linear part is
// (near-)non-invertible or vanishingly scaled (the EPSILON determinant cutoff also rejects
// uniform scales below ~5%, |det| ~ s^3) -- the caller's signal to fall back to defaults.
std::optional<FinFeature> fin_bed_to_object(const FinFeature& f_bed, const Transform3d& to_bed);

// True when `delta` (a world-frame instance-matrix delta, M_now * M_cached^-1) preserves
// the object's bed-frame silhouette: a no-reflection isometry that rotates only about the
// vertical and does not lift off the bed. A cached bed-frame fin ghost stays exact under
// such a delta -- render it with `delta` as the model matrix instead of regenerating.
// `tol` bounds both the orthonormality check and the Z-translation (mm).
bool is_bed_rigid_delta(const Transform3d& delta, double tol = 1e-6);

} // namespace FinGeneratorInternal

} // namespace Slic3r
