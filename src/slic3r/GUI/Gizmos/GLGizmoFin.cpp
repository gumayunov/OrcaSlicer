#include "GLGizmoFin.hpp"
#include "slic3r/GUI/GLCanvas3D.hpp"
#include "slic3r/GUI/GUI_App.hpp"
#include "slic3r/GUI/Plater.hpp"
#include "slic3r/GUI/Camera.hpp"
#include "slic3r/GUI/ImGuiWrapper.hpp"
#include "slic3r/GUI/Gizmos/GLGizmosManager.hpp"
#include "slic3r/Utils/UndoRedo.hpp"

#include "libslic3r/Geometry.hpp"
#include "libslic3r/Model.hpp" // FinFeature, ModelObject
#include "libslic3r/Fin/FinGenerator.hpp"
#include "libslic3r/Flow.hpp"

#include <glad/gl.h>
#include <wx/utils.h>

#include <boost/log/trivial.hpp>

#include <algorithm>
#include <cmath>

namespace Slic3r {
namespace GUI {

static constexpr double MinFinDimension = 0.5; // mm -- the plane may not collapse to zero size
static constexpr double MinFinParamValue = 0.1; // mm -- thickness / connector step / gap must stay positive and printable
static constexpr double MinRotationRingRadius = 7.5; // mm -- min ring diameter 15mm
static constexpr unsigned int RotationRingSegments = 48;

// Plane fill tint (T00-04-04): default vs. "no intersection with the object" (mirrors
// GLGizmoCut3D's CUT_PLANE_DEF_COLOR / CUT_PLANE_ERR_COLOR switch in render_cut_plane()).
static const ColorRGBA PlaneFillColor        = ColorRGBA(0.9f, 0.9f, 0.9f, 0.45f);
static const ColorRGBA PlaneFillInvalidColor = ColorRGBA(1.0f, 0.8f, 0.8f, 0.5f);
// Ghost preview mesh color: ColorRGBA::ORANGE() at preview alpha.
static const ColorRGBA GhostColor = ColorRGBA(0.923f, 0.504f, 0.264f, 0.45f);

GLGizmoFin::GLGizmoFin(GLCanvas3D& parent, const std::string& icon_filename, unsigned int sprite_id)
    : GLGizmoBase(parent, icon_filename, sprite_id)
{
}

bool GLGizmoFin::on_init()
{
    for (int i = 0; i < int(Count); ++i)
        m_grabbers.push_back(Grabber());
    return true;
}

std::string GLGizmoFin::on_get_name() const
{
    return _u8L("Fin");
}

void GLGizmoFin::on_set_state()
{
    if (get_state() == On) {
        init_from_selection();
        // T00-04-04: the ghost-preview solid cache is keyed off the selected object/instance
        // (see rebuild_object_solid()), but the object's geometry may have changed since this
        // gizmo was last open even if the same instance is re-selected -- always rebuild fresh
        // on activation rather than relying solely on the object/instance-idx comparison.
        rebuild_object_solid();
    }
}

// Called by GLGizmosManager::update_data() -- notably from update_after_undo_redo() with
// is_serializing == true, right after an undo/redo rolled the Model (incl. ModelObject::fins)
// back under this open gizmo. The gizmo's plane/params are a cached VIEW of fins[0]: without
// this reload the panel and ghost would keep showing the pre-undo state (undo looks like a
// no-op), and worse, the next commit gesture would write that stale state back -- silently
// re-doing what the user just undid. The model is the source of truth, so nothing is
// serialized into snapshots gizmo-side (on_save/on_load stay the base no-ops, unlike
// GLGizmoCut3D whose plane does NOT live in the model and therefore keeps m_ar_* archive
// copies to serialize); everything is re-derived from the restored model here instead.
// Ordering is safe: Plater::priv::update_after_undo_redo() restores the deserialized GUI
// selection BEFORE delegating to GLGizmosManager::update_after_undo_redo(), so the
// selection-derived lookups (selected_model_object() etc.) see valid indices.
// Non-serializing update_data() calls (open_gizmo, sidebar object manipulation, ...) are
// deliberately ignored: same-object geometry changes while the gizmo stays open are handled
// per frame by solid_cache_stale() and must NOT reload plane/params (see the comment in
// update_ghost_preview()).
void GLGizmoFin::data_changed(bool is_serializing)
{
    if (!is_serializing)
        return;
    BOOST_LOG_TRIVIAL(info) << "GLGizmoFin: undo/redo restore detected -- reloading gizmo state from ModelObject::fins";
    m_drag_ref_captured = false; // no in-flight gesture reference survives an undo/redo jump
    init_from_selection();
    rebuild_object_solid();
}

// T00-04-05: loads the selected object's existing fin (fins[0] -- single-fin UX for now) when
// there is one, converting object-local -> world via fin_object_to_bed() (the exact inverse of
// apply_to_model()'s world -> object-local conversion; see FinGenerator.hpp). Falls back to
// reset_to_defaults() for a fin-less object, or if the instance transform degenerates the
// stored fin's axes (fin_object_to_bed() returns nullopt -- e.g. the object was rotated
// elsewhere into an orientation where the fin's plane axis is no longer horizontal).
void GLGizmoFin::init_from_selection()
{
    const ModelObject* mo = selected_model_object();
    if (mo != nullptr && !mo->fins.empty()) {
        if (const std::optional<Transform3d> to_bed = selected_instance_matrix(); to_bed.has_value()) {
            if (const std::optional<FinFeature> fin_world = FinGeneratorInternal::fin_object_to_bed(mo->fins.front(), *to_bed);
                fin_world.has_value()) {
                m_plane_center = Vec3d(fin_world->anchor_xy.x(), fin_world->anchor_xy.y(), 0.0);
                m_z_angle      = fin_world->z_angle;
                m_height       = fin_world->height;
                m_length       = fin_world->length;
                m_thickness    = fin_world->thickness;
                m_conn_step    = fin_world->conn_step;
                m_conn_gap     = fin_world->conn_gap;
                m_enabled      = fin_world->enabled;
                m_ghost_dirty  = true; // T00-04-04: the plane/params above just changed
                return;
            }
            BOOST_LOG_TRIVIAL(warning) << "GLGizmoFin: stored fin's object-local axes are degenerate "
                                           "under the current instance transform -- falling back to defaults";
        }
    }
    reset_to_defaults();
}

// Factory-default plane/params for a new fin -- also the Reset button's target (unlike
// init_from_selection(), this never reads ModelObject::fins, so Reset always lands here
// regardless of whether a fin is already stored on the object).
void GLGizmoFin::reset_to_defaults()
{
    // Anchor at the instance's own +Y silhouette edge, not the bbox center (design doc §15): with
    // window semantics the gizmo rectangle is the literal bound of where fin material may exist,
    // so a center anchor would put the whole window inside the part for anything wider than 2*L,
    // yielding an empty (red) fin by default. At z_angle=0 (u == +Y, see plane_matrix()) this
    // puts the window's near edge (u=0) right at the part's surface, so the default fin hugs it
    // immediately instead of starting from the middle of the model.
    const BoundingBoxf3& bbox = m_parent.get_selection().get_bounding_box();
    const Vec3d center = bbox.center();
    m_plane_center = Vec3d(center.x(), bbox.max.y(), 0.0);
    m_z_angle      = 0.0;

    // Default H: the selected instance's world top above the bed (bbox is in world
    // coordinates and the instance sits on the bed), so the plane spans the whole
    // model instead of the (tiny) FinFeature 10mm default.
    m_height = std::max(MinFinDimension, bbox.max.z());

    const FinFeature defaults; // remaining fields just mirror FinFeature{} for now
    m_length    = defaults.length;
    m_thickness = defaults.thickness;
    m_conn_step = defaults.conn_step;
    m_conn_gap  = defaults.conn_gap;
    m_enabled   = defaults.enabled;

    m_ghost_dirty = true; // T00-04-04: the plane/params above just changed -- the cached ghost mesh is stale
}

bool GLGizmoFin::on_is_selectable() const
{
    return true;
}

bool GLGizmoFin::on_is_activable() const
{
    // Fin gizmo only makes sense for a single, fully selected object instance.
    return m_parent.get_selection().is_single_full_instance();
}

void GLGizmoFin::on_register_raycasters_for_picking()
{
    // the gizmo grabbers are rendered on top of the scene, so the raytraced picker should take it into account
    m_parent.set_raycaster_gizmos_on_top(true);
}

void GLGizmoFin::on_unregister_raycasters_for_picking()
{
    m_parent.set_raycaster_gizmos_on_top(false);
}

bool GLGizmoFin::on_mouse(const wxMouseEvent& mouse_event)
{
    if (use_grabbers(mouse_event))
        return true;

    // Consume left-clicks that did not interact with any grabber (but not the
    // release of a camera drag): otherwise a stray click on empty space reaches
    // GLCanvas3D's deselect path, which deselects the object, closes this gizmo
    // and throws away the not-yet-persisted fin parameters (persistence comes
    // with T00-04-05). Same mechanism as GLGizmoCut3D / GLGizmoPainterBase.
    // The gizmo is exited via its toolbar icon or Esc (GLGizmosManager::on_char
    // -> reset_all_states()).
    if (mouse_event.LeftUp() && !m_parent.is_mouse_dragging())
        return true;

    return false;
}

Transform3d GLGizmoFin::plane_matrix() const
{
    // The gizmo tracks anchor/azimuth as m_plane_center/m_z_angle (not a FinFeature -- mid-drag
    // there isn't a persisted one yet), so delegate to the (anchor_xy, z_angle) primitive rather
    // than hand-mirroring FinGeneratorInternal::plane_to_bed()'s (u, w, n) column formula here.
    // m_plane_center's Z is always kept at the bed (0), matching plane_to_bed()'s own Z=0 anchor.
    return FinGeneratorInternal::plane_to_bed(Vec2d(m_plane_center.x(), m_plane_center.y()), m_z_angle);
}

double GLGizmoFin::rotation_ring_radius() const
{
    // Ring diameter follows the fin's horizontal reach (m_length), never below 15mm.
    // Intentionally independent of m_height (which now defaults to the object's
    // height): the ring must stay compact, near the plane root, so it is always
    // in view when the user is looking at the fin base.
    return std::max(0.5 * m_length, MinRotationRingRadius);
}

// The edited print preset's layer height, or `fallback` when no usable print config is
// loaded (e.g. running headless). The two call sites below need different fallbacks: a UI
// drag-snap increment (m_snap_step) vs. a physically plausible layer height (0.2mm) to
// build preview geometry with.
double GLGizmoFin::preset_layer_height(double fallback) const
{
    if (wxGetApp().preset_bundle != nullptr) {
        const double layer_height = wxGetApp().preset_bundle->prints.get_edited_preset().config.opt_float("layer_height");
        if (layer_height > 0.0)
            return layer_height;
    }
    return fallback;
}

double GLGizmoFin::get_layer_height_snap_step() const
{
    // No print config available -- fall back to a plain mm snap.
    return preset_layer_height(m_snap_step);
}

// Layer height for the ghost-preview mesh (connector tab thickness, generate_one_fin()'s
// `layer_height` argument).
double GLGizmoFin::resolve_preview_layer_height() const
{
    return preset_layer_height(0.2); // mm -- generic default, only reached without a loaded print preset
}

// Extrusion line width for the ghost-preview mesh (fin wall / connector tabs).
// Preview fidelity is secondary (per the plan): this resolves the shared "line_width" print
// option via Flow::extrusion_width(), the same utility Flow::new_from_config_width() (and so
// PrintObjectSlice.cpp's TransientFinGuard, at slice time) ultimately falls back to for a
// zero-valued *_line_width option -- it already implements the "0 -> nozzle-diameter-based
// auto width" fallback (Flow::auto_extrusion_width(), 1.125x nozzle diameter for frPerimeter).
// Unlike TransientFinGuard, this does not walk the inner_wall_line_width -> line_width chain
// first, since no PrintRegion is available here; that only matters for exact width, not shape.
double GLGizmoFin::resolve_preview_line_width() const
{
    if (wxGetApp().preset_bundle != nullptr) {
        try {
            const DynamicPrintConfig full_config = wxGetApp().preset_bundle->full_config();
            const double             width        = Flow::extrusion_width("line_width", full_config, 0);
            if (width > 0.0)
                return width;
        } catch (const std::exception&) {
            // Missing nozzle_diameter or similar (e.g. no printer preset loaded) -- fall
            // through to the generic default below.
        }
    }
    return 0.4; // mm -- generic default, only reached without a usable print/printer config
}

// True when m_object_solid_bed no longer reflects the scene: a different object/instance is
// selected, the instance's world transform changed (move/rotate/scale -- also reachable while
// this gizmo stays open, e.g. via the sidebar object-manipulation fields), or MODEL_PART
// volumes were added/removed. Called every render frame -- everything here is O(1) (the matrix
// comparison is 16 doubles). Volume-LOCAL edits (a part's own transform/mesh changing without
// the volume count moving) are not detectable this way -- ModelObject carries no change
// timestamp (ObjectBase::timestamp() is a constant 0) -- but those require a volume
// sub-selection, which deactivates this gizmo (on_is_activable()), and reactivation rebuilds
// the solid unconditionally (on_set_state()).
bool GLGizmoFin::solid_cache_stale() const
{
    const Selection& selection = m_parent.get_selection();
    if (selection.get_object_idx() != m_solid_object_idx || selection.get_instance_idx() != m_solid_instance_idx)
        return true;
    if (m_solid_object_idx < 0 || m_solid_instance_idx < 0)
        return false; // nothing selected and nothing cached -- a rebuild would change nothing

    const ModelObjectPtrs& objects = wxGetApp().plater()->model().objects;
    if (size_t(m_solid_object_idx) >= objects.size())
        return false; // lookup would fail inside rebuild_object_solid() too -- don't rebuild-loop
    const ModelObject* mo = objects[m_solid_object_idx];
    if (mo == nullptr || size_t(m_solid_instance_idx) >= mo->instances.size())
        return false;

    return mo->volumes.size() != m_solid_volume_count
        || mo->instances[m_solid_instance_idx]->get_matrix().matrix() != m_solid_instance_matrix.matrix();
}

// Rebuilds m_object_solid_bed from the current selection: the selected instance's MODEL_PART
// volumes, merged into the world/bed frame (the gizmo's world frame IS the bed frame -- see
// the class doc comment). Snapshots the cache key (idxs, instance transform, volume count) so
// solid_cache_stale() can tell whether this actually needs to run again; the mesh assembly/
// merge itself is not free, so this must NOT run every render frame -- only on activation
// (on_set_state()) and when solid_cache_stale() fires in update_ghost_preview().
void GLGizmoFin::rebuild_object_solid()
{
    const Selection& selection  = m_parent.get_selection();
    m_solid_object_idx      = selection.get_object_idx();
    m_solid_instance_idx    = selection.get_instance_idx();
    m_solid_instance_matrix = Transform3d::Identity();
    m_solid_volume_count    = 0;
    m_object_solid_bed      = indexed_triangle_set();
    // The solid just changed (cleared above; possibly refilled below) -- any cached ghost mesh
    // is stale either way. Set before the early returns, so they cannot leave a ghost that was
    // built against a solid that no longer exists on screen (update_ghost_preview() against the
    // now-empty solid then correctly yields the invalid/red-plane state instead).
    m_ghost_dirty = true;

    if (m_solid_object_idx < 0 || m_solid_instance_idx < 0)
        return; // nothing selected -- shouldn't happen while on_is_activable() gates this gizmo, but stay defensive

    const ModelObjectPtrs& objects = wxGetApp().plater()->model().objects;
    if (size_t(m_solid_object_idx) >= objects.size())
        return;
    const ModelObject* mo = objects[m_solid_object_idx];
    if (mo == nullptr || size_t(m_solid_instance_idx) >= mo->instances.size())
        return;

    // Instance world transform only: FinGeneratorInternal::object_solid_bed() itself composes
    // each volume's own get_matrix() on top of `to_bed`, exactly mirroring how GLVolume/
    // Selection place an object's volumes in world space. No PrintObject::trafo_centered()
    // involved here, unlike the slice-time path (PrintObjectSlice.cpp's TransientFinGuard):
    // the gizmo has no PrintObject, and doesn't need one since its world frame already is the
    // bed frame.
    const Transform3d to_bed = mo->instances[m_solid_instance_idx]->get_matrix();
    m_solid_instance_matrix  = to_bed;
    m_solid_volume_count     = mo->volumes.size();
    m_object_solid_bed       = FinGeneratorInternal::object_solid_bed(*mo, to_bed);
}

// Rebuilds the cached ghost-preview mesh (m_ghost_model / m_ghost_valid) via generate_one_fin(),
// but only when actually dirty -- called every render frame, but a true no-op on most of them.
// Dirtied by: on_stop_dragging() (drag-end, not per drag frame), ImGui parameter edits
// (on_render_input_window()), and rebuild_object_solid() (the solid itself changed).
void GLGizmoFin::update_ghost_preview()
{
    // Selection changed under us without an on_set_state() cycle, or the model itself was
    // moved/rotated/scaled or got volumes added/removed while this gizmo stayed open (e.g.
    // via the sidebar fields) -- the cached bed-frame solid describes the OLD placement, so
    // re-derive it (which also marks the ghost dirty for regeneration below).
    if (solid_cache_stale()) {
        // Distinguish "a DIFFERENT object/instance got selected" (e.g. the user clicked another
        // object in the scene or the object list while this gizmo stayed open -- on_is_activable()
        // keeps it open as long as the new selection is still a single full instance) from "the
        // SAME object/instance moved/rotated/gained-or-lost a volume". Only the former must reload
        // the plane/params (T00-04-05): otherwise the previous object's plane position could
        // silently get applied onto the new object on its next commit. The latter must NOT touch
        // plane/params -- that would blow away in-progress edits every time the object is nudged.
        const Selection& selection       = m_parent.get_selection();
        const bool       identity_changed = selection.get_object_idx() != m_solid_object_idx
                                          || selection.get_instance_idx() != m_solid_instance_idx;
        if (identity_changed && !is_dragging()) {
            // Guard against is_dragging(): can't really happen via the mouse (dragging holds the
            // button down, so no other object can be clicked mid-gesture), but stay defensive --
            // reloading plane/params out from under an in-flight drag would invalidate its
            // m_drag_start_*/m_drag_ref_* reference state.
            m_drag_ref_captured = false; // no pending gesture reference should survive onto the new object
            init_from_selection();       // reload: the new object's own stored fin, or defaults
        }
        rebuild_object_solid();
    }

    if (!m_ghost_dirty)
        return;
    m_ghost_dirty = false;

    // Verbatim gizmo state -> bed-frame FinFeature (the plane already IS in the bed frame, so,
    // unlike generate_fin_meshes()'s object-local -> bed conversion, no further transform is
    // needed here). `enabled` is forced true: the preview always shows regardless of the
    // panel's Enabled toggle -- the toggle only affects what gets persisted/sliced (apply_to_model()
    // below), not this live preview.
    FinFeature fin_bed_frame;
    fin_bed_frame.anchor_xy = Vec2d(m_plane_center.x(), m_plane_center.y());
    fin_bed_frame.z_angle   = m_z_angle;
    fin_bed_frame.height    = m_height;
    fin_bed_frame.length    = m_length;
    fin_bed_frame.thickness = m_thickness;
    fin_bed_frame.conn_step = m_conn_step;
    fin_bed_frame.conn_gap  = m_conn_gap;
    fin_bed_frame.enabled   = true;

    const TriangleMesh mesh = generate_one_fin(m_object_solid_bed, fin_bed_frame, resolve_preview_layer_height(), resolve_preview_line_width());

    m_ghost_model.reset();
    m_ghost_valid = !mesh.empty();
    if (m_ghost_valid) {
        m_ghost_model.init_from(mesh);
        m_ghost_model.set_color(GhostColor);
    }
}

// --- Persistence (T00-04-05) -------------------------------------------------

ModelObject* GLGizmoFin::selected_model_object() const
{
    const Selection& selection  = m_parent.get_selection();
    const int         object_idx = selection.get_object_idx();
    if (object_idx < 0)
        return nullptr;
    const ModelObjectPtrs& objects = wxGetApp().plater()->model().objects;
    if (size_t(object_idx) >= objects.size())
        return nullptr;
    return objects[object_idx];
}

// The selected instance's current world transform (to_bed for this gizmo's purposes -- its
// world frame IS the bed frame, see the class doc comment). A fresh lookup, deliberately
// decoupled from the ghost-preview solid cache's m_solid_instance_matrix (that one is only
// refreshed once per render frame, in on_render(), which is not guaranteed to have already run
// before an apply triggered from the ImGui panel in the same frame).
std::optional<Transform3d> GLGizmoFin::selected_instance_matrix() const
{
    const Selection& selection    = m_parent.get_selection();
    const int         object_idx   = selection.get_object_idx();
    const int         instance_idx = selection.get_instance_idx();
    if (object_idx < 0 || instance_idx < 0)
        return std::nullopt;
    const ModelObject* mo = selected_model_object();
    if (mo == nullptr || size_t(instance_idx) >= mo->instances.size())
        return std::nullopt;
    return mo->instances[instance_idx]->get_matrix();
}

// Writes the current gizmo (world-frame) state into the selected object's fins[0] (creating it
// if the object had none yet -- single-fin UX for now), converting world -> object-local via
// fin_bed_to_object() (FinGenerator.hpp; the exact inverse of init_from_selection()'s
// fin_object_to_bed() call). Wrapped in one undo/redo snapshot per call -- callers are
// responsible for calling this only once per user gesture (a drag's on_stop_dragging(), or an
// ImGui field's commit/IsItemDeactivatedAfterEdit(), not every intermediate frame) so a single
// drag or edit does not spam the undo stack. Schedules a background reslice on success, mirroring
// how e.g. GLGizmoFdmSupports/GLGizmoBrimEars commit paint/point edits onto their ModelObject.
// No-op (nothing persisted, no snapshot taken) when there is no valid single-instance selection,
// or when the instance transform degenerates the fin's local axes (fin_bed_to_object() returns
// nullopt -- see its doc comment); the latter is logged since it silently drops the edit.
void GLGizmoFin::apply_to_model(const std::string& snapshot_name)
{
    ModelObject* mo = selected_model_object();
    if (mo == nullptr)
        return;
    const std::optional<Transform3d> to_bed = selected_instance_matrix();
    if (!to_bed.has_value())
        return;

    FinFeature fin_world;
    fin_world.anchor_xy = Vec2d(m_plane_center.x(), m_plane_center.y());
    fin_world.z_angle   = m_z_angle;
    fin_world.height    = m_height;
    fin_world.length    = m_length;
    fin_world.thickness = m_thickness;
    fin_world.conn_step = m_conn_step;
    fin_world.conn_gap  = m_conn_gap;
    fin_world.enabled   = m_enabled;

    const std::optional<FinFeature> fin_object = FinGeneratorInternal::fin_bed_to_object(fin_world, *to_bed);
    if (!fin_object.has_value()) {
        BOOST_LOG_TRIVIAL(warning) << "GLGizmoFin: instance transform makes the world -> object-local "
                                       "fin conversion degenerate -- not persisting this edit";
        return;
    }

    Plater::TakeSnapshot snapshot(wxGetApp().plater(), snapshot_name, UndoRedo::SnapshotType::GizmoAction);
    BOOST_LOG_TRIVIAL(info) << "GLGizmoFin: took undo snapshot '" << snapshot_name << "', persisting fin edit";
    if (mo->fins.empty())
        mo->fins.push_back(FinFeature{});
    mo->fins.front() = *fin_object;

    wxGetApp().plater()->set_plater_dirty(true);
    m_parent.post_event(SimpleEvent(EVT_GLCANVAS_SCHEDULE_BACKGROUND_PROCESS));
}

// "Remove fin" button (T00-04-05): clears the object's fins outright -- unlike the Enabled
// checkbox (which persists enabled=false and leaves the stored fin otherwise intact, see
// on_render_input_window()), this is how the user actually gets rid of one. No-op when the
// object has no fin to remove (nothing to snapshot).
void GLGizmoFin::remove_fin_from_model()
{
    ModelObject* mo = selected_model_object();
    if (mo == nullptr || mo->fins.empty())
        return;

    Plater::TakeSnapshot snapshot(wxGetApp().plater(), _u8L("Remove fin"), UndoRedo::SnapshotType::GizmoAction);
    BOOST_LOG_TRIVIAL(info) << "GLGizmoFin: took undo snapshot 'Remove fin', clearing fins";
    mo->fins.clear();

    wxGetApp().plater()->set_plater_dirty(true);
    m_parent.post_event(SimpleEvent(EVT_GLCANVAS_SCHEDULE_BACKGROUND_PROCESS));
}

void GLGizmoFin::init_rendering_items()
{
    if (m_rendering_items_ready)
        return;

    // Unit quad (0..1 x 0..1) in the local plane frame (x = u, y = w); scaled to
    // (m_length, m_height) at render time so the geometry never needs rebuilding
    // while H/L are being dragged.
    GLModel::Geometry fill_data;
    fill_data.format = { GLModel::Geometry::EPrimitiveType::Triangles, GLModel::Geometry::EVertexLayout::P3 };
    fill_data.reserve_vertices(4);
    fill_data.reserve_indices(6);
    fill_data.add_vertex(Vec3f(0.0f, 0.0f, 0.0f));
    fill_data.add_vertex(Vec3f(1.0f, 0.0f, 0.0f));
    fill_data.add_vertex(Vec3f(1.0f, 1.0f, 0.0f));
    fill_data.add_vertex(Vec3f(0.0f, 1.0f, 0.0f));
    fill_data.add_triangle(0, 1, 2);
    fill_data.add_triangle(0, 2, 3);
    fill_data.color = PlaneFillColor; // overridden every frame in render_plane() (T00-04-04: red tint when invalid)
    m_plane_fill.init_from(std::move(fill_data));

    GLModel::Geometry border_data;
    border_data.format = { GLModel::Geometry::EPrimitiveType::LineLoop, GLModel::Geometry::EVertexLayout::P3 };
    border_data.reserve_vertices(4);
    border_data.reserve_indices(4);
    border_data.add_vertex(Vec3f(0.0f, 0.0f, 0.0f));
    border_data.add_vertex(Vec3f(1.0f, 0.0f, 0.0f));
    border_data.add_vertex(Vec3f(1.0f, 1.0f, 0.0f));
    border_data.add_vertex(Vec3f(0.0f, 1.0f, 0.0f));
    for (unsigned int i = 0; i < 4; ++i)
        border_data.add_index(i);
    border_data.color = ColorRGBA::YELLOW();
    m_plane_border.init_from(std::move(border_data));

    GLModel::Geometry ring_data;
    ring_data.format = { GLModel::Geometry::EPrimitiveType::LineLoop, GLModel::Geometry::EVertexLayout::P3 };
    ring_data.reserve_vertices(RotationRingSegments);
    ring_data.reserve_indices(RotationRingSegments);
    for (unsigned int i = 0; i < RotationRingSegments; ++i) {
        const float angle = float(i) * 2.0f * float(PI) / float(RotationRingSegments);
        ring_data.add_vertex(Vec3f(std::cos(angle), std::sin(angle), 0.0f));
        ring_data.add_index(i);
    }
    ring_data.color = ColorRGBA::GRAY();
    m_rotation_ring.init_from(std::move(ring_data));

    m_rendering_items_ready = true;
}

void GLGizmoFin::render_plane(const Transform3d& plane_m)
{
    GLShaderProgram* shader = wxGetApp().get_shader("flat");
    if (shader == nullptr)
        return;

    glsafe(::glEnable(GL_DEPTH_TEST));
    glsafe(::glDisable(GL_CULL_FACE));
    glsafe(::glEnable(GL_BLEND));
    glsafe(::glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));

    shader->start_using();

    const Camera& camera = wxGetApp().plater()->get_camera();
    shader->set_uniform("projection_matrix", camera.get_projection_matrix());

    const Transform3d view_model_matrix = camera.get_view_matrix() * plane_m * Geometry::scale_transform(Vec3d(m_length, m_height, 1.0));
    shader->set_uniform("view_model_matrix", view_model_matrix);
    // T00-04-04: tint the fill red when the plane misses the object's solid (no ghost to
    // show), mirroring GLGizmoCut3D::render_cut_plane()'s CUT_PLANE_DEF_COLOR/ERR_COLOR switch.
    m_plane_fill.set_color(m_ghost_valid ? PlaneFillColor : PlaneFillInvalidColor);
    m_plane_fill.render();

    shader->set_uniform("width", 0.5f);
    m_plane_border.render();

    glsafe(::glEnable(GL_CULL_FACE));
    glsafe(::glDisable(GL_BLEND));

    shader->stop_using();
}

// T00-04-04: the fin mesh generate_one_fin() currently produces for m_object_solid_bed and
// the plane/params, rebuilt only by update_ghost_preview() (see its callers) -- never here.
// Drawn after the plane, semi-transparent, depth-tested against the rest of the scene.
void GLGizmoFin::render_ghost()
{
    // NOT is_empty(): GLModel frees its CPU-side geometry after the first render() uploads it
    // to the GPU (send_to_gpu()), which flips is_empty() to true -- gating on it would draw the
    // ghost exactly once and then skip it on every subsequent frame. is_initialized() keeps
    // reporting the (GPU-resident) vertex/index counts, so it stays true for the model's lifetime.
    if (!m_ghost_valid || !m_ghost_model.is_initialized())
        return;

    GLShaderProgram* shader = wxGetApp().get_shader("gouraud_light");
    if (shader == nullptr)
        return;

    // Depth test is already on (enabled at the top of on_render(), never disabled since:
    // render_plane() re-enables it too and restores everything else it touches).
    glsafe(::glEnable(GL_BLEND));
    glsafe(::glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));

    shader->start_using();

    const Camera& camera = wxGetApp().plater()->get_camera();
    shader->set_uniform("projection_matrix", camera.get_projection_matrix());
    // The mesh is already expressed in the bed/world frame (generate_one_fin()'s contract),
    // and the gizmo's world frame IS the bed frame, so no additional model transform is needed.
    shader->set_uniform("view_model_matrix", camera.get_view_matrix());
    shader->set_uniform("emission_factor", 0.2f);

    m_ghost_model.render();

    shader->stop_using();

    glsafe(::glDisable(GL_BLEND));
}

void GLGizmoFin::render_rotation_ring()
{
    GLShaderProgram* shader = wxGetApp().get_shader("flat");
    if (shader == nullptr)
        return;

    shader->start_using();

    const Camera& camera = wxGetApp().plater()->get_camera();
    shader->set_uniform("projection_matrix", camera.get_projection_matrix());

    const double r = rotation_ring_radius();
    const Transform3d view_model_matrix = camera.get_view_matrix() * Geometry::translation_transform(m_plane_center) * Geometry::scale_transform(Vec3d(r, r, r));
    shader->set_uniform("view_model_matrix", view_model_matrix);
    shader->set_uniform("width", 0.5f);
    m_rotation_ring.render();

    shader->stop_using();
}

void GLGizmoFin::update_grabbers(const Transform3d& plane_m)
{
    for (Grabber& g : m_grabbers)
        g.matrix = plane_m;

    // Grabber centers are expressed in the local plane frame (x = u, y = w, z = n);
    // Grabber::render() maps them to world space via g.matrix (== plane_m).
    m_grabbers[Move].center        = Vec3d::Zero();
    m_grabbers[ZRotation].center   = Vec3d(0.0, 0.0, rotation_ring_radius());
    m_grabbers[TopHeight].center   = Vec3d(0.5 * m_length, m_height, 0.0);
    m_grabbers[OuterLength].center = Vec3d(m_length, 0.5 * m_height, 0.0);
}

void GLGizmoFin::on_render()
{
    init_rendering_items();

    glsafe(::glClear(GL_DEPTH_BUFFER_BIT));
    glsafe(::glEnable(GL_DEPTH_TEST));

    // T00-04-04: cheap no-op unless something is actually dirty (drag-end / ImGui edit /
    // solid rebuild) -- see update_ghost_preview()'s doc comment. Must run before
    // render_plane() so the red/default tint below reflects this frame's result, not the
    // previous one.
    update_ghost_preview();

    const Transform3d plane_m = plane_matrix();

    render_plane(plane_m);
    render_ghost();
    render_rotation_ring();

    update_grabbers(plane_m);
    render_grabbers(Grabber::FixedGrabberSize);
}

void GLGizmoFin::on_start_dragging()
{
    m_drag_start_center  = m_plane_center;
    m_drag_start_z_angle = m_z_angle;
    m_drag_start_height  = m_height;
    m_drag_start_length  = m_length;
    m_drag_ref_captured  = false;

    if (m_hover_id >= 0 && size_t(m_hover_id) < m_grabbers.size())
        m_drag_start_world_pos = m_grabbers[m_hover_id].matrix * m_grabbers[m_hover_id].center;
}

void GLGizmoFin::on_stop_dragging()
{
    m_drag_ref_captured = false;
    // T00-04-04: rebuild the ghost preview once dragging ends -- not per drag frame. All four
    // grabbers (Move/ZRotation/TopHeight/OuterLength) feed generate_one_fin() one way or
    // another (anchor_xy, z_angle, height, length), so any drag can invalidate the cached mesh.
    m_ghost_dirty = true;

    // T00-04-05: persist on drag-end -- one snapshot per drag gesture (never per drag frame).
    // Skip a no-op drag (e.g. a click that starts and immediately ends a "drag" without actually
    // moving anything): only take a snapshot when the plane genuinely changed since
    // on_start_dragging(), mirroring GLGizmoCut3D::on_stop_dragging()'s m_was_cut_plane_dragged
    // guard around its own "Move cut plane" snapshot.
    const bool changed = m_plane_center != m_drag_start_center || m_z_angle != m_drag_start_z_angle
                       || m_height != m_drag_start_height || m_length != m_drag_start_length;
    if (changed)
        apply_to_model(_u8L("Edit fin"));
}

double GLGizmoFin::calc_axis_projection(const UpdateData& data, const Vec3d& ref_point, const Vec3d& axis_dir, double snap_step) const
{
    double projection = 0.0;
    const double axis_len = axis_dir.norm();
    if (axis_len > 1e-9) {
        const Vec3d axis_unit = axis_dir / axis_len;
        const Vec3d mouse_dir = data.mouse_ray.unit_vector();
        // Intersect the mouse ray with the plane parallel to the camera viewport and passing
        // through ref_point (same technique as GLGizmoMove3D::calc_projection), then project
        // the resulting offset onto axis_dir. Works for any axis direction, incl. world Z.
        const Vec3d inters = data.mouse_ray.a + (ref_point - data.mouse_ray.a).dot(mouse_dir) * mouse_dir;
        projection = (inters - ref_point).dot(axis_unit);
    }
    if (wxGetKeyState(WXK_SHIFT) && snap_step > 0.0)
        projection = snap_step * std::round(projection / snap_step);
    return projection;
}

void GLGizmoFin::on_dragging(const UpdateData& data)
{
    if (m_hover_id < 0)
        return;

    switch (m_hover_id) {
    case Move: {
        // Bed translation: literal ray / Z=0 plane intersection (the bed IS the plane's
        // root, so this is always well defined -- no viewport-parallel approximation needed).
        const Vec3d bed_hit = data.mouse_ray.intersect_plane(0.0);
        if (!m_drag_ref_captured) {
            m_drag_ref_bed_point = bed_hit;
            m_drag_ref_captured  = true;
        }
        Vec3d delta = bed_hit - m_drag_ref_bed_point;
        if (wxGetKeyState(WXK_SHIFT)) {
            delta.x() = m_snap_step * std::round(delta.x() / m_snap_step);
            delta.y() = m_snap_step * std::round(delta.y() / m_snap_step);
        }
        m_plane_center    = m_drag_start_center + Vec3d(delta.x(), delta.y(), 0.0);
        m_plane_center.z() = 0.0; // constraint: the plane's root never leaves the bed
        break;
    }
    case ZRotation: {
        // Angle in the bed plane around the (fixed, during this drag) plane center.
        const Vec3d  bed_hit = data.mouse_ray.intersect_plane(0.0);
        const double dx      = bed_hit.x() - m_plane_center.x();
        const double dy      = bed_hit.y() - m_plane_center.y();
        if (dx * dx + dy * dy < 1e-9)
            break; // degenerate: mouse projects (almost) onto the rotation axis itself
        const double angle = std::atan2(dy, dx);
        if (!m_drag_ref_captured) {
            m_drag_ref_angle    = angle;
            m_drag_ref_captured = true;
        }
        m_z_angle = m_drag_start_z_angle + (angle - m_drag_ref_angle);
        break;
    }
    case TopHeight: {
        const double projection = calc_axis_projection(data, m_drag_start_world_pos, Vec3d::UnitZ(), get_layer_height_snap_step());
        m_height = std::max(MinFinDimension, m_drag_start_height + projection);
        break;
    }
    case OuterLength: {
        const Vec3d  u          = plane_matrix().linear().col(0); // z_angle is constant during this drag
        const double projection = calc_axis_projection(data, m_drag_start_world_pos, u, m_snap_step);
        m_length = std::max(MinFinDimension, m_drag_start_length + projection);
        break;
    }
    default:
        break;
    }
}

void GLGizmoFin::init_input_window_layout()
{
    // Mirrors GLGizmoCut3D::init_input_window_data(): control width tracks the current font
    // size and is recomputed every frame (the UI scale can change at runtime), while label
    // width is the widest of the fixed field labels below, computed once and cached.
    m_control_width = m_imgui->get_font_size() * 9.f;

    if (m_label_width > 0.f)
        return;

    for (const std::string& label : { _u8L("Height (H)"), _u8L("Length (L)"), _u8L("Thickness (t)"),
                                       _u8L("Connector step (X)"), _u8L("Gap"), _u8L("Angle") }) {
        const float width = m_imgui->calc_text_size(label).x;
        if (m_label_width < width)
            m_label_width = width;
    }
    m_label_width += m_imgui->scaled(1.f);
    m_label_width += ImGui::GetStyle().WindowPadding.x;
}

// label: [input] row, mirroring GLGizmoCut3D::render_move_center_input()'s idiom (m_label_width
// for the caption column, m_control_width for the field, plain ImGuiInputTextFlags_CharsDecimal
// entry). Edits apply live (clamped to min_val) as soon as ImGui reports a change, so typing
// immediately drives the gizmo state and the live ghost preview. `committed` is OR-ed with
// ImGui::IsItemDeactivatedAfterEdit() (true once, the frame the field loses focus after having
// been edited) -- T00-04-05's signal for when to persist the field's value onto
// ModelObject::fins, as opposed to every keystroke.
bool GLGizmoFin::render_double_field(const std::string& label, const char* imgui_id, double& value, double min_val, const char* format, bool& committed)
{
    m_imgui->text(label);
    ImGui::SameLine(m_label_width);
    ImGui::PushItemWidth(m_control_width);
    double input = value;
    const bool changed = ImGui::InputDouble(imgui_id, &input, 0.0, 0.0, format, ImGuiInputTextFlags_CharsDecimal);
    committed |= ImGui::IsItemDeactivatedAfterEdit();
    if (changed)
        value = std::max(min_val, input);
    return changed;
}

void GLGizmoFin::on_render_input_window(float x, float y, float bottom_limit)
{
    GizmoImguiSetNextWIndowPos(x, y, ImGuiCond_Always, 0.0f, 0.0f);
    ImGuiWrapper::push_toolbar_style(m_parent.get_scale());
    GizmoImguiBegin(get_name(), ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar);

    init_input_window_layout();

    const float f_scale = m_parent.get_gizmos_manager().get_layout_scale();
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(6.0f, 4.0f * f_scale));

    // T00-04-04: any edit below that feeds generate_one_fin() invalidates the cached ghost
    // preview; collected into one flag and applied once at the end rather than at each call
    // site. (m_enabled is deliberately NOT included: the preview always uses enabled=true,
    // see update_ghost_preview(), so toggling it alone never changes the generated mesh.)
    bool params_changed = false;
    // T00-04-05: a field was "committed" (lost focus after being edited, or a single-shot button
    // like Flip was pressed) this frame -- persist once at the end of this function, one snapshot
    // per gesture, never per keystroke. Grabber drags persist separately, at drag-end
    // (on_stop_dragging()); this flag only covers the ImGui panel.
    bool field_committed = false;

    // H/L drive the plane rectangle drawn in on_render(); MinFinDimension mirrors the drag
    // clamp in on_dragging() so typing and dragging agree on the smallest allowed plane.
    params_changed |= render_double_field(_u8L("Height (H)"), "##fin_height", m_height, MinFinDimension, "%.2f mm", field_committed);
    params_changed |= render_double_field(_u8L("Length (L)"), "##fin_length", m_length, MinFinDimension, "%.2f mm", field_committed);
    params_changed |= render_double_field(_u8L("Thickness (t)"), "##fin_thickness", m_thickness, MinFinParamValue, "%.2f mm", field_committed);
    params_changed |= render_double_field(_u8L("Connector step (X)"), "##fin_conn_step", m_conn_step, MinFinParamValue, "%.2f mm", field_committed);

    ImGui::Separator();
    m_imgui->text_colored(ImGuiWrapper::COL_GREY_LIGHT, _u8L("Advanced"));
    params_changed |= render_double_field(_u8L("Gap"), "##fin_conn_gap", m_conn_gap, MinFinParamValue, "%.2f mm", field_committed);

    ImGui::Separator();

    // Angle: stored in radians (m_z_angle, consumed by plane_matrix()); displayed/edited in
    // degrees, matching how GLGizmoRotate and GLGizmoCut3D's groove angle show angles to users.
    m_imgui->text(_u8L("Angle"));
    ImGui::SameLine(m_label_width);
    ImGui::PushItemWidth(m_control_width);
    double angle_deg = Geometry::rad2deg(m_z_angle);
    if (ImGui::InputDouble("##fin_angle", &angle_deg, 0.0, 0.0, "%.1f°", ImGuiInputTextFlags_CharsDecimal)) {
        m_z_angle = Geometry::deg2rad(angle_deg);
        params_changed = true;
    }
    field_committed |= ImGui::IsItemDeactivatedAfterEdit();

    ImGui::SameLine();
    if (m_imgui->button(_L("Flip side (180°)"))) {
        // Keep the displayed angle tidy across repeated flips/edits instead of letting it
        // grow unbounded; std::remainder wraps into (-180°, 180°].
        m_z_angle = std::remainder(m_z_angle + PI, 2.0 * PI);
        params_changed  = true;
        field_committed = true; // a button click is its own complete gesture -- persist now
    }

    ImGui::Separator();

    if (m_imgui->button(_L("Reset")))
        reset_to_defaults(); // sets m_ghost_dirty itself; does NOT touch ModelObject::fins (see its doc comment)

    ImGui::SameLine();
    if (m_imgui->bbl_checkbox(_L("Enabled"), m_enabled))
        // A checkbox toggle is its own atomic gesture (there is nothing to "commit" later) --
        // persist immediately with its own snapshot, same as Flip/Remove fin below.
        apply_to_model(_u8L("Toggle fin"));

    // On its own row, separated from Reset/Enabled: unlike them, this is destructive (clears
    // ModelObject::fins outright, see remove_fin_from_model()'s doc comment).
    if (m_imgui->button(_L("Remove fin")))
        remove_fin_from_model();

    if (params_changed)
        m_ghost_dirty = true;
    if (field_committed)
        apply_to_model(_u8L("Edit fin"));

    // T00-05-01: mirrors the red-plane tint (render_plane(), T00-04-04) with a textual warning --
    // covers both empty-result causes generate_one_fin() can hit (plane misses the object
    // entirely, or the window is fully swallowed by the part+gap): one generic message is enough
    // to point the user at the red plane, matching how GLGizmoCut3D flags its own invalid states.
    if (!m_ghost_valid) {
        ImGui::Separator();
        m_imgui->warning_text(_u8L("Fin does not intersect the object").c_str());
    }

    ImGui::PopStyleVar(1); // ImGuiStyleVar_FramePadding

    GizmoImguiEnd();
    ImGuiWrapper::pop_toolbar_style();
}

} // namespace GUI
} // namespace Slic3r
