#include "GLGizmoFin.hpp"
#include "slic3r/GUI/GLCanvas3D.hpp"
#include "slic3r/GUI/GUI_App.hpp"
#include "slic3r/GUI/Plater.hpp"
#include "slic3r/GUI/Camera.hpp"
#include "slic3r/GUI/ImGuiWrapper.hpp"
#include "slic3r/GUI/format.hpp"
#include "slic3r/GUI/Gizmos/GLGizmosManager.hpp"
#include "slic3r/GUI/FinGhosts.hpp"
#include "slic3r/Utils/UndoRedo.hpp"

#include "libslic3r/Geometry.hpp"
#include "libslic3r/Model.hpp" // FinFeature, ModelObject
#include "libslic3r/Fin/FinGenerator.hpp"

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

// Plane fill tint: default vs. "no intersection with the object" (mirrors
// GLGizmoCut3D's CUT_PLANE_DEF_COLOR / CUT_PLANE_ERR_COLOR switch in render_cut_plane()).
static const ColorRGBA PlaneFillColor        = ColorRGBA(0.9f, 0.9f, 0.9f, 0.45f);
static const ColorRGBA PlaneFillInvalidColor = ColorRGBA(1.0f, 0.8f, 0.8f, 0.5f);

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
        // The ghost-preview solid cache is keyed off the selected object/instance
        // (see rebuild_object_solid()), but the object's geometry may have changed since this
        // gizmo was last open even if the same instance is re-selected -- always rebuild fresh
        // on activation rather than relying solely on the object/instance-idx comparison.
        rebuild_object_solid();
    } else {
        // WYSIWYG close: what the ghost showed is what gets kept -- commit the
        // panel state, including the untouched defaults of a freshly opened gizmo and a
        // typed-but-not-yet-committed field edit (its value already lives in the m_* state;
        // the per-gesture commit path would only pick it up on the NEXT ImGui frame, which
        // never comes once the gizmo closes). No-op when nothing changed (apply_to_model()
        // compares) or right after "Remove fin" (m_apply_on_close).
        //
        // Selection may already have moved on by the time this runs (e.g. the gizmo was
        // deactivated by clicking empty space or selecting a different object/sub-volume) --
        // only commit while the selection still points at the object/instance this panel
        // state belongs to; every completed gesture was already persisted individually via
        // apply_to_model(), so skipping here loses nothing.
        //
        // Never commit while GLGizmosManager is deserializing an undo/redo snapshot
        // (is_serializing()): load_snapshot() restores the Model first and only then closes
        // this gizmo via activate_gizmo() (when the target snapshot had no gizmo open), so at
        // that instant the m_* state still holds the just-undone values while ModelObject::fins
        // already holds the restored ones -- committing here would re-write the undone values
        // and take a rogue "Edit fin" snapshot INSIDE the undo load, making Ctrl+Z of a fin
        // edit appear to do nothing. data_changed(is_serializing=true) is the sanctioned
        // reload path instead; this mirrors how activate_gizmo() guards its own enter/leave
        // snapshots with !m_serializing.
        const Selection& selection = m_parent.get_selection();
        if (m_apply_on_close
            && !m_parent.get_gizmos_manager().is_serializing()
            && selection.get_object_idx() == m_solid_object_idx
            && selection.get_instance_idx() == m_solid_instance_idx)
            apply_to_model(_u8L("Edit fin"));
    }
}

// Called by GLGizmosManager::update_data() -- notably from update_after_undo_redo() with
// is_serializing == true, right after an undo/redo rolled the Model (incl. ModelObject::fins)
// back under this open gizmo. The gizmo's plane/params are a cached VIEW of fins[m_fin_idx]: without
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
    // Keep the user's fin selection across undo/redo when possible: re-select the same
    // index clamped to the restored vector (via the pending mechanism init_from_selection()
    // already consumes). An emptied vector falls through to defaults as before.
    if (const ModelObject* mo_before = selected_model_object();
        mo_before != nullptr && !mo_before->fins.empty())
        m_pending_selection = PendingFinSelection{ mo_before->id(),
                                                   std::min(m_fin_idx, int(mo_before->fins.size()) - 1) };
    init_from_selection();
    rebuild_object_solid();
    // After an undo/redo the restored model is the source of truth: if it says "no fin"
    // (e.g. a redone Remove fin), closing the gizmo must not resurrect one. The WYSIWYG
    // close-commit stays armed only when a fin actually survived the restore; any explicit
    // edit gesture afterwards re-arms it via apply_to_model().
    const ModelObject* mo = selected_model_object();
    m_apply_on_close = mo != nullptr && !mo->fins.empty();
}

// Loads the selected object's existing fin -- the pending fin (produced by
// data_changed()'s undo/redo re-selection or by handle_ghost_click()'s cross-object
// switch) or fins[0] -- when there is one, converting object-local -> world
// via fin_object_to_bed() (the exact inverse of apply_to_model()'s world -> object-local
// conversion; see FinGenerator.hpp). Falls back to reset_to_defaults() for a fin-less object,
// or if the instance transform degenerates the stored fin's axes (fin_object_to_bed() returns
// nullopt -- e.g. the object was rotated elsewhere into an orientation where the fin's plane
// axis is no longer horizontal).
void GLGizmoFin::init_from_selection()
{
    m_apply_on_close = true; // fresh per activation/object -- re-arm the WYSIWYG close-commit
    const ModelObject* mo = selected_model_object();

    int fin_idx = 0;
    if (mo != nullptr && m_pending_selection.has_value() && m_pending_selection->object == mo->id()
        && !mo->fins.empty())
        fin_idx = std::clamp(m_pending_selection->fin_idx, 0, int(mo->fins.size()) - 1);
    m_pending_selection.reset();

    if (!load_fin_into_panel(fin_idx)) {
        if (mo != nullptr && !mo->fins.empty())
            BOOST_LOG_TRIVIAL(warning) << "GLGizmoFin: could not load stored fin into the panel (no valid "
                                           "selection or degenerate instance transform) -- falling back to defaults";
        m_fin_idx = 0;
        reset_to_defaults();
    }
}

// fins[fin_idx] -> panel state (object-local -> bed via fin_object_to_bed(), the exact
// inverse of the persistence path). On success also makes fin_idx the selected fin.
// Returns false -- leaving the panel AND m_fin_idx untouched -- when the index is out of
// range, there is no valid selection, or the instance transform degenerates the stored
// fin's axes.
bool GLGizmoFin::load_fin_into_panel(int fin_idx)
{
    const ModelObject* mo = selected_model_object();
    if (mo == nullptr || fin_idx < 0 || size_t(fin_idx) >= mo->fins.size())
        return false;
    const std::optional<Transform3d> to_bed = selected_instance_matrix();
    if (!to_bed.has_value())
        return false;
    const std::optional<FinFeature> fin_world =
        FinGeneratorInternal::fin_object_to_bed(mo->fins[fin_idx], *to_bed);
    if (!fin_world.has_value())
        return false;

    m_fin_idx      = fin_idx;
    m_plane_center = Vec3d(fin_world->anchor.x(), fin_world->anchor.y(), 0.0);
    m_z_angle      = FinGeneratorInternal::z_angle_from_dir(fin_world->dir);
    m_height       = fin_world->height;
    m_length       = fin_world->length;
    m_thickness    = fin_world->thickness;
    m_conn_step    = fin_world->conn_step;
    m_conn_gap     = fin_world->conn_gap;
    m_ghost_dirty  = true;
    return true;
}

// Factory-default plane/params for a new fin -- also the Reset button's target (unlike
// init_from_selection(), this never reads ModelObject::fins, so Reset always lands here
// regardless of whether a fin is already stored on the object).
void GLGizmoFin::reset_to_defaults()
{
    // Anchor at the instance's own +Y silhouette edge, not the bbox center: the gizmo
    // rectangle is the literal window bounding where fin material may exist,
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

    m_ghost_dirty = true; // the plane/params above just changed -- the cached ghost mesh is stale
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

    if (mouse_event.Leaving()) {
        // The canvas only clears the hover state when this gizmo is NOT current
        // (_render_fin_ghosts()); leaving the canvas with the gizmo open would
        // otherwise freeze the last hover highlight until the pointer returns.
        m_parent.fin_ghosts().set_hovered(FinGhostRef{});
        return false;
    }

    // Ghost hover highlight: observe-only -- never consume moves.
    if (mouse_event.Moving()) {
        update_ghost_hover(mouse_event);
        return false;
    }

    // Ghost-click fin selection: a clean left press (no grabber hovered, no
    // camera drag in flight) on a scene ghost switches the edited fin -- possibly across
    // objects/instances. Consumed so the canvas never treats it as select/deselect.
    if (mouse_event.LeftDown() && !m_parent.is_mouse_dragging() && m_hover_id < 0
        && handle_ghost_click(mouse_event))
        return true;

    // Consume left-clicks that did not interact with any grabber (but not the
    // release of a camera drag): otherwise a stray click on empty space reaches
    // GLCanvas3D's deselect path, which deselects the object, closes this gizmo
    // and throws away the not-yet-persisted fin parameters. Same mechanism as
    // GLGizmoCut3D / GLGizmoPainterBase.
    // The gizmo is exited via its toolbar icon or Esc (GLGizmosManager::on_char
    // -> reset_all_states()).
    if (mouse_event.LeftUp() && !m_parent.is_mouse_dragging())
        return true;

    return false;
}

bool GLGizmoFin::render_follows_cursor() const
{
    return m_parent.fin_ghosts().hovered().valid();
}

// Raycast the scene ghosts under the cursor and push the hover descriptor into the
// renderer (it draws the hovered ghost highlighted). One raycast sweep per mouse move,
// only while this gizmo is active; each ghost is AABB-pruned inside MeshRaycaster, and
// fin meshes are small -- cheap enough. GLCanvas3D clears the hover state whenever the
// Fin gizmo is not the active one.
void GLGizmoFin::update_ghost_hover(const wxMouseEvent& mouse_event)
{
    const Vec2d   mouse_pos(double(mouse_event.GetX()), double(mouse_event.GetY()));
    const Camera& camera = wxGetApp().plater()->get_camera();
    const Model&  model  = wxGetApp().plater()->model();

    FinGhostRef hover;
    if (const std::optional<FinPick> hit = m_parent.fin_ghosts().pick(mouse_pos, camera, model);
        hit.has_value())
        hover = FinGhostRef{ hit->object_id, hit->instance_id, hit->fin_idx };
    m_parent.fin_ghosts().set_hovered(hover);
}

// A left press landed outside every grabber: if it hits a scene ghost, switch to that
// fin. Same object+instance -> in-place switch (select_fin() commits the current panel
// first, WYSIWYG). A ghost of another object/instance -> commit, arm the pending fin
// selection and move the Selection; update_ghost_preview()'s identity_changed path then
// runs init_from_selection(), which consumes the pending index instead of fins[0].
// Returns whether the click was consumed.
bool GLGizmoFin::handle_ghost_click(const wxMouseEvent& mouse_event)
{
    const Vec2d   mouse_pos(double(mouse_event.GetX()), double(mouse_event.GetY()));
    const Camera& camera = wxGetApp().plater()->get_camera();
    const Model&  model  = wxGetApp().plater()->model();

    const std::optional<FinPick> hit = m_parent.fin_ghosts().pick(mouse_pos, camera, model);
    if (!hit.has_value())
        return false;

    const Selection& selection = m_parent.get_selection();
    if (hit->object_idx == selection.get_object_idx() && hit->instance_idx == selection.get_instance_idx()) {
        select_fin(hit->fin_idx);
    } else {
        apply_to_model(_u8L("Edit fin")); // commit BEFORE the selection moves off this object
        m_pending_selection = PendingFinSelection{ hit->object_id, hit->fin_idx };
        m_parent.get_selection().add_instance(unsigned(hit->object_idx), unsigned(hit->instance_idx), true);
        m_parent.post_event(SimpleEvent(EVT_GLCANVAS_OBJECT_SELECT)); // sync the sidebar object list
    }
    return true;
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

double GLGizmoFin::get_layer_height_snap_step() const
{
    // No print config available -- fall back to a plain mm snap.
    return FinGhost::preset_layer_height(m_snap_step);
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
        // the plane/params: otherwise the previous object's plane position could
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
    // needed here).
    const FinFeature fin_bed_frame = panel_fin_world();

    m_ghost_valid = FinGhost::build_ghost(m_ghost_model, m_object_solid_bed, fin_bed_frame);
}

// --- Persistence ------------------------------------------------------------

// Approximate FinFeature equality for the "did the user actually change anything" check in
// prepare_commit(). Both operands are BED-FRAME canonical forms (anchor.z == 0, unit
// horizontal dir): the stored fin's fin_object_to_bed() image vs. the gizmo's own bed-frame
// state -- comparing raw object-local fields instead would be transform-dependent (see the
// comment at the call site) and the round trip through the conversions is exact only up to
// floating-point noise, so exact operator== would see phantom diffs either way.
static bool fins_approx_equal(const FinFeature& a, const FinFeature& b)
{
    constexpr double eps = 1e-9;
    // No angle field to compare: anchor and dir are both derived through a deterministic
    // conversion chain and dir stays normalized, so a plain componentwise comparison
    // suffices -- there is no +/-PI branch cut to fold away, and dir vs -dir are genuinely
    // different sides, not an artifact to treat as equal.
    return std::abs(a.anchor.x() - b.anchor.x()) < eps
        && std::abs(a.anchor.y() - b.anchor.y()) < eps
        && std::abs(a.anchor.z() - b.anchor.z()) < eps
        && std::abs(a.dir.x() - b.dir.x()) < eps
        && std::abs(a.dir.y() - b.dir.y()) < eps
        && std::abs(a.dir.z() - b.dir.z()) < eps
        && std::abs(a.height    - b.height)    < eps
        && std::abs(a.length    - b.length)    < eps
        && std::abs(a.thickness - b.thickness) < eps
        && std::abs(a.conn_step - b.conn_step) < eps
        && std::abs(a.conn_gap  - b.conn_gap)  < eps;
}

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

// The fin this gizmo currently edits, as a scene-ghost reference. Valid only when the
// panel edits a fin actually stored on the object (an uncommitted fresh default has no
// scene ghost to hide). Instance is left invalid deliberately: the fin is hidden on
// EVERY instance while being edited -- only the gizmo's own live ghost (drawn in the
// selected instance's frame) represents it.
FinGhostRef GLGizmoFin::selected_fin_ref() const
{
    const ModelObject* mo = selected_model_object();
    if (mo == nullptr || size_t(m_fin_idx) >= mo->fins.size())
        return {};
    return FinGhostRef{ mo->id(), ObjectID(), m_fin_idx };
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

// The panel's current plane/params as a bed-frame FinFeature (the gizmo's world frame IS
// the bed frame). Shared by the ghost preview and the persistence path so they can never
// drift apart.
FinFeature GLGizmoFin::panel_fin_world() const
{
    FinFeature f;
    f.anchor    = Vec3d(m_plane_center.x(), m_plane_center.y(), 0.0);
    f.dir       = FinGeneratorInternal::dir_from_z_angle(m_z_angle);
    f.height    = m_height;
    f.length    = m_length;
    f.thickness = m_thickness;
    f.conn_step = m_conn_step;
    f.conn_gap  = m_conn_gap;
    return f;
}

// Converts the panel state for a commit into fins[m_fin_idx]: bed-frame image, its
// object-local form, and whether storing it would actually change the model (the
// no-change check runs on the BED-FRAME images -- see the comment inside; raw
// object-local comparison would be transform-dependent). nullopt: no valid selection
// or the conversion is degenerate (logged -- it silently drops the edit otherwise).
std::optional<GLGizmoFin::PanelCommit> GLGizmoFin::prepare_commit() const
{
    const ModelObject* mo = selected_model_object();
    if (mo == nullptr)
        return std::nullopt;
    const std::optional<Transform3d> to_bed = selected_instance_matrix();
    if (!to_bed.has_value())
        return std::nullopt;

    PanelCommit commit;
    commit.fin_world = panel_fin_world();
    const std::optional<FinFeature> fin_object =
        FinGeneratorInternal::fin_bed_to_object(commit.fin_world, *to_bed);
    if (!fin_object.has_value()) {
        BOOST_LOG_TRIVIAL(warning) << "GLGizmoFin: instance transform makes the world -> object-local "
                                       "fin conversion degenerate -- not persisting this edit";
        return std::nullopt;
    }
    commit.fin_object    = *fin_object;
    commit.changes_model = true;

    if (size_t(m_fin_idx) < mo->fins.size()) {
        // No-change check on the BED-FRAME images, not the raw object-local fields: the stored
        // anchor was normalized to bed z=0 under the instance transform AT STORE TIME, so after
        // the user lifts/tilts the object a zero-edit open->close would produce object-local
        // fields differing by the transform delta -- a raw-field compare would then flag a
        // phantom edit and take a spurious snapshot + reslice for a geometrically identical
        // fin. A nullopt projection (stored fin degenerate) counts as "different".
        const std::optional<FinFeature> stored_bed =
            FinGeneratorInternal::fin_object_to_bed(mo->fins[m_fin_idx], *to_bed);
        if (stored_bed.has_value() && fins_approx_equal(*stored_bed, commit.fin_world))
            commit.changes_model = false;
    }
    return commit;
}

// The write half: fins[m_fin_idx] = commit (creating the slot when m_fin_idx points one
// past the end -- the "new fin" case of add_fin() and of an object with no fins yet).
// NO undo snapshot here -- callers own snapshotting, so add_fin() can wrap two stores
// into one "Add fin" snapshot.
void GLGizmoFin::store_commit(ModelObject& mo, const PanelCommit& commit)
{
    if (size_t(m_fin_idx) >= mo.fins.size()) {
        m_fin_idx = int(mo.fins.size());
        mo.fins.push_back(FinFeature{});
    }
    mo.fins[m_fin_idx] = commit.fin_object;
    m_apply_on_close   = true; // any explicit commit re-arms the WYSIWYG close-commit
}

// Writes the current gizmo (world-frame) state into the selected object's fins[m_fin_idx]
// (creating the slot when m_fin_idx points one past the end -- an object with no fins yet),
// converting world -> object-local via fin_bed_to_object() (FinGenerator.hpp; the exact inverse
// of init_from_selection()'s fin_object_to_bed() call -- see prepare_commit()/store_commit()
// above, which hold the snapshot-free halves of this path). Wrapped in one undo/redo snapshot
// per call -- callers are responsible for calling this only once per user gesture (a drag's
// on_stop_dragging(), or an ImGui field's commit/IsItemDeactivatedAfterEdit(), not every
// intermediate frame) so a single drag or edit does not spam the undo stack. Schedules a
// background reslice on success, mirroring how e.g. GLGizmoFdmSupports/GLGizmoBrimEars commit
// paint/point edits onto their ModelObject.
// No-op (nothing persisted, no snapshot taken) when there is no valid single-instance selection,
// or when the instance transform degenerates the fin's local axes (fin_bed_to_object() returns
// nullopt -- see its doc comment); the latter is logged since it silently drops the edit. Also a
// no-op when the converted fin is approximately equal to what is already stored (fins_approx_equal()
// above) -- this keeps a no-change WYSIWYG close-commit from polluting the undo stack. Note the
// approx-equal skip alone does NOT make the close-commit safe in every context: during undo/redo
// deserialization the m_* state genuinely differs from the freshly restored model, so
// on_set_state() additionally guards the call with !is_serializing() (see its comment).
//
// Full persistence contract for the Fin gizmo: every completed user gesture (drag-end,
// an ImGui field's IsItemDeactivatedAfterEdit(), Flip) calls this function
// directly. On top of that, closing the gizmo (toolbar re-click, Esc, switching to another gizmo)
// also calls this function once from on_set_state()'s Off branch, committing whatever the panel/ghost
// was showing at close time (WYSIWYG) -- including untouched defaults on a freshly opened gizmo and a
// typed-but-not-yet-committed field edit. The ONLY way to delete a stored fin is the "Remove fin"
// button (remove_fin_from_model()); the close-commit never deletes, and m_apply_on_close suppresses
// it from immediately resurrecting a just-removed fin.
void GLGizmoFin::apply_to_model(const std::string& snapshot_name)
{
    ModelObject* mo = selected_model_object();
    if (mo == nullptr)
        return;
    const std::optional<PanelCommit> commit = prepare_commit();
    if (!commit.has_value() || !commit->changes_model)
        return; // nothing to persist -- don't pollute the undo stack

    Plater::TakeSnapshot snapshot(wxGetApp().plater(), snapshot_name, UndoRedo::SnapshotType::GizmoAction);
    BOOST_LOG_TRIVIAL(debug) << "GLGizmoFin: took undo snapshot '" << snapshot_name << "', persisting fin edit";
    store_commit(*mo, *commit);

    wxGetApp().plater()->set_plater_dirty(true);
    m_parent.post_event(SimpleEvent(EVT_GLCANVAS_SCHEDULE_BACKGROUND_PROCESS));
}

// Switch the edited fin within the current object (arrow buttons and same-object ghost
// clicks -- handle_ghost_click()). WYSIWYG: the panel state is committed first, so
// the fin the user navigates away from keeps exactly what its ghost showed.
void GLGizmoFin::select_fin(int fin_idx)
{
    const ModelObject* mo = selected_model_object();
    if (mo == nullptr || fin_idx < 0 || size_t(fin_idx) >= mo->fins.size() || fin_idx == m_fin_idx)
        return;
    apply_to_model(_u8L("Edit fin"));
    if (!load_fin_into_panel(fin_idx))
        BOOST_LOG_TRIVIAL(warning) << "GLGizmoFin: fin " << fin_idx << " is degenerate under the "
                                       "current instance transform -- selection unchanged";
}

int GLGizmoFin::stored_fin_count() const
{
    const ModelObject* mo = selected_model_object();
    return mo == nullptr ? 0 : int(mo->fins.size());
}

// "+ Add fin": commit the current panel state (WYSIWYG), then create and select a new fin
// with the factory-default plane. ONE undo snapshot for the whole gesture, so Ctrl+Z rolls
// back both the pre-switch commit and the new fin in a single step.
void GLGizmoFin::add_fin()
{
    ModelObject* mo = selected_model_object();
    if (mo == nullptr || !selected_instance_matrix().has_value())
        return;

    Plater::TakeSnapshot snapshot(wxGetApp().plater(), _u8L("Add fin"), UndoRedo::SnapshotType::GizmoAction);
    BOOST_LOG_TRIVIAL(debug) << "GLGizmoFin: took undo snapshot 'Add fin'";

    if (const std::optional<PanelCommit> current = prepare_commit();
        current.has_value() && current->changes_model)
        store_commit(*mo, *current);

    reset_to_defaults();
    m_fin_idx = int(mo->fins.size()); // store_commit() creates fins[m_fin_idx] from the defaults
    if (const std::optional<PanelCommit> fresh = prepare_commit(); fresh.has_value())
        store_commit(*mo, *fresh);
    // A degenerate conversion (non-invertible instance matrix) leaves the vector as
    // committed above; the panel still shows the default plane and the next successful
    // gesture creates the fin. The snapshot is then near-empty -- accepted rare edge.

    wxGetApp().plater()->set_plater_dirty(true);
    m_parent.post_event(SimpleEvent(EVT_GLCANVAS_SCHEDULE_BACKGROUND_PROCESS));
}

// "Remove fin" button: erases ONLY the selected fin (fins[m_fin_idx]) -- the ONLY way to
// delete a stored fin. No-op when the object has no fin to remove (nothing to snapshot).
// When a fin survives, the nearest surviving neighbor is loaded into the panel (WYSIWYG
// re-arm: closing then commits THAT fin). Only when the vector just got emptied is the
// WYSIWYG close-commit suppressed (m_apply_on_close): the panel still holds the removed
// fin's parameters and the ghost stays visible, so without this, closing the gizmo right
// after Remove would immediately resurrect the fin the user just deleted.
void GLGizmoFin::remove_fin_from_model()
{
    ModelObject* mo = selected_model_object();
    if (mo == nullptr || mo->fins.empty())
        return;

    Plater::TakeSnapshot snapshot(wxGetApp().plater(), _u8L("Remove fin"), UndoRedo::SnapshotType::GizmoAction);
    const int removed = std::min(m_fin_idx, int(mo->fins.size()) - 1);
    BOOST_LOG_TRIVIAL(debug) << "GLGizmoFin: took undo snapshot 'Remove fin', erasing fin " << removed;
    mo->fins.erase(mo->fins.begin() + removed);

    if (mo->fins.empty()) {
        // Last fin gone: the panel keeps showing its parameters and the close-commit
        // is suppressed so closing does not resurrect it.
        m_fin_idx        = 0;
        m_apply_on_close = false;
    } else {
        const int next = std::min(removed, int(mo->fins.size()) - 1);
        if (!load_fin_into_panel(next)) {
            // Degenerate under the current transform: don't let a close-commit stamp the
            // REMOVED fin's parameters (still in the panel) onto a surviving fin.
            m_fin_idx        = next;
            m_apply_on_close = false;
        }
        // load_fin_into_panel() success re-selects `next`; closing then commits THAT fin,
        // which is correct WYSIWYG behavior.
    }

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
    fill_data.color = PlaneFillColor; // overridden every frame in render_plane() (red tint when invalid)
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
    // Tint the fill red when the plane misses the object's solid (no ghost to
    // show), mirroring GLGizmoCut3D::render_cut_plane()'s CUT_PLANE_DEF_COLOR/ERR_COLOR switch.
    m_plane_fill.set_color(m_ghost_valid ? PlaneFillColor : PlaneFillInvalidColor);
    m_plane_fill.render();

    shader->set_uniform("width", 0.5f);
    m_plane_border.render();

    glsafe(::glEnable(GL_CULL_FACE));
    glsafe(::glDisable(GL_BLEND));

    shader->stop_using();
}

// The fin mesh generate_one_fin() currently produces for m_object_solid_bed and
// the plane/params, rebuilt only by update_ghost_preview() (see its callers) -- never here.
// Drawn after the plane, semi-transparent, depth-tested against the rest of the scene.
void GLGizmoFin::render_ghost()
{
    if (!m_ghost_valid)
        return;
    // The gizmo's ghost mesh is already in the bed frame -- no extra model transform.
    FinGhost::render(m_ghost_model, Transform3d::Identity());
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

    // Cheap no-op unless something is actually dirty (drag-end / ImGui edit /
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
    // Rebuild the ghost preview once dragging ends -- not per drag frame. All four
    // grabbers (Move/ZRotation/TopHeight/OuterLength) feed generate_one_fin() one way or
    // another (anchor, dir, height, length), so any drag can invalidate the cached mesh.
    m_ghost_dirty = true;

    // Persist on drag-end -- one snapshot per drag gesture (never per drag frame).
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
// been edited) -- the signal for when to persist the field's value onto ModelObject::fins,
// as opposed to every keystroke.
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

    // Any edit below that feeds generate_one_fin() invalidates the cached ghost
    // preview; collected into one flag and applied once at the end rather than at each call
    // site.
    bool params_changed = false;
    // A field was "committed" (lost focus after being edited, or a single-shot button
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

    // Multi-fin navigation: the arrows cycle the selected fin -- the fallback
    // path for fins whose ghost is empty (plane misses the part) and therefore unclickable;
    // "+ Add fin" commits the current fin and creates-and-selects a new default one. The
    // count reflects the committed model; a fresh panel with no stored fin reads 1/1 with
    // the arrows disabled.
    const int fin_count = stored_fin_count();
    const int nav_count = std::max(fin_count, 1);
    m_imgui->disabled_begin(nav_count <= 1);
    if (m_imgui->button("<##fin_prev"))
        select_fin((m_fin_idx + nav_count - 1) % nav_count);
    ImGui::SameLine();
    if (m_imgui->button(">##fin_next"))
        select_fin((m_fin_idx + 1) % nav_count);
    m_imgui->disabled_end();
    ImGui::SameLine();
    m_imgui->text(GUI::format(_L("Fin %1%/%2%"), m_fin_idx + 1, nav_count));
    ImGui::SameLine();
    if (m_imgui->button(_L("+ Add fin")))
        add_fin();

    ImGui::Separator();

    if (m_imgui->button(_L("Reset")))
        reset_to_defaults(); // sets m_ghost_dirty itself; does NOT touch ModelObject::fins (see its doc comment)

    // On its own row, separated from Reset: unlike it, this is destructive (erases the
    // selected fin from ModelObject::fins, see remove_fin_from_model()'s doc comment).
    if (m_imgui->button(_L("Remove fin")))
        remove_fin_from_model();

    if (params_changed)
        m_ghost_dirty = true;
    if (field_committed)
        apply_to_model(_u8L("Edit fin"));

    // Mirrors the red-plane tint (render_plane()) with a textual warning --
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
