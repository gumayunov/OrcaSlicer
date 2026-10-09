#pragma once

// Shared fin ghost-preview machinery: FinGhost -- the primitives GLGizmoFin's
// live editing preview and the persistent scene ghosts have in common; FinGhostsRenderer
// -- the always-visible per-(object x instance) scene ghosts.

#include "slic3r/GUI/GLModel.hpp"
#include "slic3r/GUI/MeshUtils.hpp" // MeshRaycaster (complete type needed for unique_ptr member)
#include "libslic3r/Model.hpp" // FinFeature / FinFeatures, ModelObject
#include "libslic3r/ObjectID.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include <chrono>
#include <map>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace Slic3r {

class Model;
class GLVolumeCollection;

namespace GUI {

class Camera;

namespace FinGhost {

// Layer height from the active print preset, `fallback` without one.
double preset_layer_height(double fallback);

// generate_one_fin() inputs for preview meshes; fidelity notes live in the .cpp.
double preview_layer_height();
double preview_line_width();

// Fill `out` from generate_one_fin() over a bed-frame solid + bed-frame fin. Returns
// false (leaving `out` reset) when the fin misses the solid (empty mesh). When
// `mesh_out` is non-null it receives the generated mesh (for building a MeshRaycaster;
// GLModel won't keep the CPU geometry around).
bool build_ghost(GLModel& out, const indexed_triangle_set& solid_bed, const FinFeature& fin_bed,
                 TriangleMesh* mesh_out = nullptr);

// Render a built ghost translucently (gouraud_light + blend, depth test on).
// `model_matrix` is an extra bed-frame transform: identity for a freshly generated
// ghost, the cached-instance delta for FinGhostsRenderer's rigid follow.
void render(GLModel& ghost, const Transform3d& model_matrix);

} // namespace FinGhost

// Identifies one fin ghost in the scene. `object` + `fin_idx` (index into
// ModelObject::fins) identify the fin; `instance` is matched exactly by the hover
// path but IGNORED by selected-fin suppression, which is always object-wide (see
// render()). Default-constructed == "none".
struct FinGhostRef
{
    ObjectID object;
    ObjectID instance;
    int      fin_idx{ -1 };
    bool valid() const { return object.id != 0 && fin_idx >= 0; }
};

// pick() result: model indices for Selection::add_instance() plus the stable ids they
// were resolved from (callers re-validate against the live Model at use time).
struct FinPick
{
    int      object_idx{ -1 };
    int      instance_idx{ -1 };
    int      fin_idx{ -1 };
    ObjectID object_id;
    ObjectID instance_id;
};

// Always-visible scene ghosts: one translucent fin preview per
// (object-with-fins x instance), rendered by GLCanvas3D without the Fin gizmo.
// Cached ghost meshes follow the object rigidly while the instance delta keeps the
// bed-frame silhouette (XY move / Z spin -- FinGeneratorInternal::is_bed_rigid_delta());
// any other change regenerates, throttled to RegenInterval while a transform is in flux.
class FinGhostsRenderer
{
public:
    // Renders all ghosts. `volumes` supplies the LIVE per-instance world transforms:
    // during an interactive drag Selection mutates only GLVolume trafos, ModelInstance
    // catches up on mouse-up (GLCanvas3D::do_move()), so reading GLVolumes keeps ghosts
    // following mid-drag; entries fall back to ModelInstance::get_matrix() when no
    // GLVolume matches (they coincide outside a drag). The fin set via set_selected() is
    // suppressed on EVERY instance of its object -- the Fin gizmo draws that fin's live,
    // possibly-uncommitted ghost itself; everything else (other fins of the same object
    // included) renders from the committed model and is a valid pick target.
    void render(const Model& model, const GLVolumeCollection& volumes);

    // The fin the Fin gizmo currently edits (hidden + unpickable); default-constructed
    // ref = nothing selected. Pushed by GLCanvas3D::_render_fin_ghosts() every frame.
    void set_selected(const FinGhostRef& selected) { m_selected = selected; }
    // The ghost under the mouse (rendered highlighted); pushed by GLGizmoFin::
    // update_ghost_hover() on mouse moves, cleared by GLCanvas3D whenever the Fin
    // gizmo is not active.
    void set_hovered(const FinGhostRef& hovered) { m_hovered = hovered; }
    const FinGhostRef& hovered() const { return m_hovered; }

    // Closest ghost under the mouse ray, or nullopt. Skips the selected fin (not drawn =>
    // not pickable). Raycasts every cached ghost with the exact model matrix it was drawn
    // with this frame (Entry::last_delta), so picking always matches the screen -- even a
    // stale cache riding an affine delta under regeneration throttling.
    std::optional<FinPick> pick(const Vec2d& mouse_pos, const Camera& camera, const Model& model) const;

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
        struct Ghost
        {
            std::unique_ptr<GLModel>       model;
            // Owns a copy of the ghost mesh: GLModel frees its CPU-side geometry after the
            // first render uploads it to the GPU, so raycasting needs its own copy.
            std::unique_ptr<MeshRaycaster> raycaster;
            int                            fin_idx{ -1 }; // index into ModelObject::fins (fins
                                                          // with no geometry produce no Ghost)
        };
        std::vector<Ghost> ghosts;                        // one per fin that produced geometry
        // Model matrix the ghosts were last drawn with; pick() must use the same one.
        Transform3d last_delta{ Transform3d::Identity() };
        std::chrono::steady_clock::time_point last_regen{};         // epoch => "never" => first regen is immediate
        bool generated{ false };                                    // false until the first regeneration ran
        bool visited{ false };                                      // per-frame GC mark
    };

    void regenerate(Entry& entry, const ModelObject& object, const Transform3d& to_bed);

    // Regeneration cadence while a transform is in flux (mid-drag tilt/scale produces a
    // non-rigid delta every frame). An isolated change regenerates immediately: its
    // last_regen is at least one interval in the past by then. Also paces the preset
    // lookups below. Tune if the ghost feels too laggy or regenerates too hot.
    static constexpr auto RegenInterval = std::chrono::milliseconds(200);

    // (ModelObject id, ModelInstance id) -> entry. Instance IDs (not indices) keep the
    // cache stable across instance list reordering; entries of deleted objects/instances
    // are garbage-collected via the `visited` mark on every render.
    std::map<std::pair<size_t, size_t>, Entry> m_entries;

    // Cached preset-derived generate_one_fin() inputs; re-resolved at most once per
    // RegenInterval and only when a fin-bearing object exists, because
    // preview_line_width() builds the full preset config (expensive per frame). A preset
    // switch thus converges within ~2x RegenInterval worst case (one interval to notice
    // the change, one regen throttle) -- acceptable.
    double m_layer_height{ 0.0 };
    double m_line_width{ 0.0 };
    std::chrono::steady_clock::time_point m_params_checked{};

    FinGhostRef m_selected;
    FinGhostRef m_hovered;
};

} // namespace GUI
} // namespace Slic3r
