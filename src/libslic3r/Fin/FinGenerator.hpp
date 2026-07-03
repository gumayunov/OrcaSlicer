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

// Build meshes for all of `o`'s enabled fins. `to_bed` is the transform mapping
// `o`'s object-local coordinates to the bed frame (z=0 = bed) -- e.g. a PrintObject's
// trafo_centered(). The returned meshes are themselves already in that BED frame.
// Disabled fins and fins whose plane misses the object's solid (empty result) are
// skipped, so the returned vector may be shorter than o.fins or empty.
std::vector<TriangleMesh> generate_fin_meshes(const ModelObject& o, const Transform3d& to_bed,
                                               double              layer_height,
                                               double              line_width);

// Internal helpers of FinGenerator, exposed (non-static) so unit tests can pin down
// the coordinate contract (design doc §13) and so later fin-generation phases
// (T00-01-02..05) can reuse them. Not part of the module's stable public API.
namespace FinGeneratorInternal {

// Plane-frame -> bed-frame transform for an explicit anchor/azimuth pair.
// Local axes: x = u (horizontal, fin extends outward along +u), y = w (= Z, height),
// z = n (horizontal plane normal). At z_angle = 0: u = +Y world, n = +X world. This is the
// primitive the FinFeature overload below builds on; it also lets callers that track the
// plane's anchor/azimuth outside of a FinFeature (GLGizmoFin::plane_matrix(), mid-drag)
// reuse the same formula instead of hand-mirroring it.
Transform3d plane_to_bed(const Vec2d& anchor_xy, double z_angle);

// Plane-frame -> bed-frame transform for a fin feature (uses anchor_xy/z_angle only --
// every other FinFeature field is irrelevant to the plane's own frame).
Transform3d plane_to_bed(const FinFeature& f);

// Bed-frame -> plane-frame transform (inverse of plane_to_bed).
Transform3d bed_to_plane(const FinFeature& f);

// Merge all MODEL_PART volume meshes of an object into one bed-frame solid.
// `to_bed` is the transform mapping object-local coordinates to the bed frame
// (pass Transform3d::Identity() when the object is already given in the bed frame).
indexed_triangle_set object_solid_bed(const ModelObject& object, const Transform3d& to_bed);

// Number of connector-tab rows: one every effective_step mm along the fin's height, starting at
// w=0 (the bed) up to and including height, where effective_step = max(f.conn_step, layer_height)
// (phase 05 edge case: a conn_step smaller than one layer would otherwise request more rows than
// the object even has layers, producing duplicate/vertically-overlapping tab geometry at
// effectively the same height -- clamping the row spacing to at least one layer avoids that
// without capping the row COUNT directly, which would silently shrink the fin's own top row).
// A row with no material at that height (above the model's own top) is still counted here -- it
// is generate_one_fin() that skips emitting a tab for it (design doc §13.3).
int fin_connector_rows(const FinFeature& f, double layer_height);

// Object-local -> bed-frame conversion of `f`'s anchor_xy/z_angle only; every other field
// passes through unchanged (design doc §13.1-13.2). This is generate_fin_meshes()'s per-fin
// conversion step, factored out here so it is unit-testable and shared with its inverse below
// (the gizmo, T00-04-05, uses both directions: this one to display an existing fin, the other
// to persist an edit). Returns nullopt when `to_bed` degenerates the object-local u axis onto
// world Z (e.g. the instance is tipped so the fin's plane axis becomes vertical) -- the
// caller's signal to skip the fin, exactly as generate_fin_meshes() did inline before this
// helper existed.
std::optional<FinFeature> fin_object_to_bed(const FinFeature& f, const Transform3d& to_bed);

// Bed-frame -> object-local conversion of `f_bed`'s anchor_xy/z_angle (inverse of
// fin_object_to_bed). Both anchor and azimuth invert through the same 2x2 linear system built
// from `to_bed`'s XY block (see the .cpp for the derivation), so that for any `to_bed` whose XY
// block is non-singular:
//   fin_object_to_bed(*fin_bed_to_object(f_bed, to_bed), to_bed)
// reproduces f_bed's anchor_xy/z_angle exactly. Returns nullopt when that XY block is (near)
// singular -- e.g. the instance is rotated so its local XY plane projects edge-on to the bed --
// the caller's signal to fall back to defaults.
std::optional<FinFeature> fin_bed_to_object(const FinFeature& f_bed, const Transform3d& to_bed);

} // namespace FinGeneratorInternal

} // namespace Slic3r
