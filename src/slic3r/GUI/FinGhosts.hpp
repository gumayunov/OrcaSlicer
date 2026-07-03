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

// Always-visible scene ghosts (design 01): one translucent fin preview per
// (object-with-enabled-fins x instance), rendered by GLCanvas3D without the Fin gizmo.
// Cached ghost meshes follow the object rigidly while the instance delta keeps the
// bed-frame silhouette (XY move / Z spin -- FinGeneratorInternal::is_bed_rigid_delta());
// any other change regenerates, throttled to RegenInterval while a transform is in flux.
class FinGhostsRenderer
{
public:
    // Renders all ghosts. `skip_object` suppresses one object's ghosts entirely --
    // pass the Fin gizmo's target object while that gizmo is open (it draws its own
    // live, possibly-unsaved preview); ObjectID() (invalid) to skip nothing. Skipped
    // objects' entries are GC'd while the gizmo is open, so closing it costs one
    // immediate regen per instance -- intentional.
    void render(const Model& model, const ObjectID& skip_object);

private:
    // Everything a ghost was generated from, compared each frame to detect staleness.
    struct VolumeKey
    {
        ObjectID    id;
        Transform3d matrix;
        // Shared-mesh address: replacing a volume's mesh changes it. Address identity has
        // a theoretical ABA window (freed+realloc'd at the same address => transiently
        // stale ghost); never dereferenced, accepted.
        const void* mesh;
        bool operator==(const VolumeKey& rhs) const
        {
            return id == rhs.id && matrix.matrix() == rhs.matrix.matrix() && mesh == rhs.mesh;
        }
        bool operator!=(const VolumeKey& rhs) const { return !(*this == rhs); }
    };

    struct Entry
    {
        FinFeatures            fins;                                // snapshot of ModelObject::fins
        std::vector<VolumeKey> volumes;                             // MODEL_PART volumes only
        Transform3d            to_bed{ Transform3d::Identity() };   // instance matrix at generation time
        double                 layer_height{ 0.0 };                 // preset-derived generate_one_fin() inputs
        double                 line_width{ 0.0 };                   //   baked into the ghosts
        std::vector<std::unique_ptr<GLModel>> ghosts;               // one per enabled fin that produced geometry
        std::chrono::steady_clock::time_point last_regen{};         // epoch => "never" => first regen is immediate
        bool generated{ false };                                    // false until the first regeneration ran
        bool visited{ false };                                      // per-frame GC mark
    };

    void regenerate(Entry& entry, const ModelObject& object, const Transform3d& to_bed);

    // (ModelObject id, ModelInstance id) -> entry. Instance IDs (not indices) keep the
    // cache stable across instance list reordering; entries of deleted objects/instances
    // are garbage-collected via the `visited` mark on every render.
    std::map<std::pair<size_t, size_t>, Entry> m_entries;
};

} // namespace GUI
} // namespace Slic3r
