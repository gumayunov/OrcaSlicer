#pragma once

// Shared fin ghost-preview machinery (design 01): FinGhost -- the primitives GLGizmoFin's
// live editing preview and the persistent scene ghosts have in common; FinGhostsRenderer
// (added in T01-00-03) -- the always-visible per-(object x instance) scene ghosts.

#include "slic3r/GUI/GLModel.hpp"
#include "libslic3r/Model.hpp" // FinFeature / FinFeatures, ModelObject
#include "libslic3r/ObjectID.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include <chrono>
#include <map>
#include <memory>
#include <utility>
#include <vector>

namespace Slic3r {

class Model;

namespace GUI {

namespace FinGhost {

// Layer height from the active print preset, `fallback` without one
// (moved verbatim from GLGizmoFin::preset_layer_height()).
double preset_layer_height(double fallback);

// generate_one_fin() inputs for preview meshes. Fidelity notes live in the .cpp
// (moved from GLGizmoFin::resolve_preview_layer_height()/resolve_preview_line_width()).
double preview_layer_height();
double preview_line_width();

// Fill `out` from generate_one_fin() over a bed-frame solid + bed-frame fin.
// Returns false (leaving `out` reset) when the fin misses the solid (empty mesh).
bool build_ghost(GLModel& out, const indexed_triangle_set& solid_bed, const FinFeature& fin_bed);

// Render a built ghost translucently (gouraud_light + blend, depth test on).
// `model_matrix` is an extra bed-frame transform: identity for a freshly generated
// ghost, the cached-instance delta for FinGhostsRenderer's rigid follow.
void render(GLModel& ghost, const Transform3d& model_matrix);

} // namespace FinGhost

} // namespace GUI
} // namespace Slic3r
