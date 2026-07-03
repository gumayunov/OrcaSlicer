#include "GLGizmoFin.hpp"
#include "slic3r/GUI/GLCanvas3D.hpp"
#include "slic3r/GUI/GUI_App.hpp"
#include "slic3r/GUI/Plater.hpp"
#include "slic3r/GUI/Camera.hpp"
#include "slic3r/GUI/ImGuiWrapper.hpp"
#include "slic3r/GUI/Gizmos/GLGizmosManager.hpp"

#include "libslic3r/Geometry.hpp"
#include "libslic3r/Model.hpp" // FinFeature
#include "libslic3r/Fin/FinGenerator.hpp"
#include "libslic3r/Flow.hpp"

#include <glad/gl.h>
#include <wx/utils.h>

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
    // T00-04-02: no FinFeature is read from the object yet, so every activation
    // (re-)seeds the plane from sensible defaults. A later task (T00-04-05) will
    // load an existing FinFeature here instead, when one is selected/present.
    if (get_state() == On) {
        init_from_selection();
        // T00-04-04: the ghost-preview solid cache is keyed off the selected object/instance
        // (see rebuild_object_solid()), but the object's geometry may have changed since this
        // gizmo was last open even if the same instance is re-selected -- always rebuild fresh
        // on activation rather than relying solely on the object/instance-idx comparison.
        rebuild_object_solid();
    }
}

void GLGizmoFin::init_from_selection()
{
    // Recomputed on every activation for now -- nothing persists yet. T00-04-05
    // will read the object's stored FinFeature first and only fall back to these
    // defaults for a new fin.
    const BoundingBoxf3& bbox = m_parent.get_selection().get_bounding_box();
    const Vec3d center = bbox.center();
    m_plane_center = Vec3d(center.x(), center.y(), 0.0);
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
    // Mirrors FinGeneratorInternal::plane_to_bed()'s (u, w, n) column convention,
    // restricted to the vertical-only case: rotation is about world Z only, and
    // the translation's Z is always 0 (the plane's root stays on the bed).
    Transform3d m = Transform3d::Identity();
    Matrix3d    R;
    R.col(0) = Vec3d(-std::sin(m_z_angle), std::cos(m_z_angle), 0.0); // u
    R.col(1) = Vec3d(0.0, 0.0, 1.0);                                  // w (== world Z)
    R.col(2) = Vec3d(std::cos(m_z_angle), std::sin(m_z_angle), 0.0);  // n
    m.linear()      = R;
    m.translation() = m_plane_center;
    return m;
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

// Rebuilds m_object_solid_bed from the current selection: the selected instance's MODEL_PART
// volumes, merged into the world/bed frame (the gizmo's world frame IS the bed frame -- see
// the class doc comment). Cheap object/instance-idx bookkeeping so update_ghost_preview() can
// tell whether this actually needs to run again; the mesh assembly/merge itself is not free,
// so this must NOT run every render frame -- only on activation (on_set_state()) and when the
// selected object/instance changes (the defensive check in update_ghost_preview()).
void GLGizmoFin::rebuild_object_solid()
{
    const Selection& selection = m_parent.get_selection();
    m_solid_object_idx   = selection.get_object_idx();
    m_solid_instance_idx = selection.get_instance_idx();
    m_object_solid_bed   = indexed_triangle_set();
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
    m_object_solid_bed       = FinGeneratorInternal::object_solid_bed(*mo, to_bed);
}

// Rebuilds the cached ghost-preview mesh (m_ghost_model / m_ghost_valid) via generate_one_fin(),
// but only when actually dirty -- called every render frame, but a true no-op on most of them.
// Dirtied by: on_stop_dragging() (drag-end, not per drag frame), ImGui parameter edits
// (on_render_input_window()), and rebuild_object_solid() (the solid itself changed).
void GLGizmoFin::update_ghost_preview()
{
    const Selection& selection = m_parent.get_selection();
    if (selection.get_object_idx() != m_solid_object_idx || selection.get_instance_idx() != m_solid_instance_idx)
        rebuild_object_solid(); // defensive: selection changed under us without an on_set_state() cycle

    if (!m_ghost_dirty)
        return;
    m_ghost_dirty = false;

    // Verbatim gizmo state -> bed-frame FinFeature (the plane already IS in the bed frame, so,
    // unlike generate_fin_meshes()'s object-local -> bed conversion, no further transform is
    // needed here). `enabled` is forced true: the preview always shows regardless of the
    // panel's Enabled toggle, whose actual slicing semantics land in T00-04-05.
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
    if (!m_ghost_valid || m_ghost_model.is_empty())
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
// immediately drives the gizmo state -- there is no separate "commit" step in this task.
bool GLGizmoFin::render_double_field(const std::string& label, const char* imgui_id, double& value, double min_val, const char* format)
{
    m_imgui->text(label);
    ImGui::SameLine(m_label_width);
    ImGui::PushItemWidth(m_control_width);
    double input = value;
    const bool changed = ImGui::InputDouble(imgui_id, &input, 0.0, 0.0, format, ImGuiInputTextFlags_CharsDecimal);
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

    // H/L drive the plane rectangle drawn in on_render(); MinFinDimension mirrors the drag
    // clamp in on_dragging() so typing and dragging agree on the smallest allowed plane.
    params_changed |= render_double_field(_u8L("Height (H)"), "##fin_height", m_height, MinFinDimension, "%.2f mm");
    params_changed |= render_double_field(_u8L("Length (L)"), "##fin_length", m_length, MinFinDimension, "%.2f mm");
    params_changed |= render_double_field(_u8L("Thickness (t)"), "##fin_thickness", m_thickness, MinFinParamValue, "%.2f mm");
    params_changed |= render_double_field(_u8L("Connector step (X)"), "##fin_conn_step", m_conn_step, MinFinParamValue, "%.2f mm");

    ImGui::Separator();
    m_imgui->text_colored(ImGuiWrapper::COL_GREY_LIGHT, _u8L("Advanced"));
    params_changed |= render_double_field(_u8L("Gap"), "##fin_conn_gap", m_conn_gap, MinFinParamValue, "%.2f mm");

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

    ImGui::SameLine();
    if (m_imgui->button(_L("Flip side (180°)"))) {
        // Keep the displayed angle tidy across repeated flips/edits instead of letting it
        // grow unbounded; std::remainder wraps into (-180°, 180°].
        m_z_angle = std::remainder(m_z_angle + PI, 2.0 * PI);
        params_changed = true;
    }

    ImGui::Separator();

    if (m_imgui->button(_L("Reset")))
        init_from_selection(); // sets m_ghost_dirty itself; no need to fold into params_changed

    ImGui::SameLine();
    // Toggles m_enabled only; T00-04-05 wires this to whether the fin is actually generated.
    m_imgui->bbl_checkbox(_L("Enabled"), m_enabled);

    if (params_changed)
        m_ghost_dirty = true;

    ImGui::PopStyleVar(1); // ImGuiStyleVar_FramePadding

    GizmoImguiEnd();
    ImGuiWrapper::pop_toolbar_style();
}

} // namespace GUI
} // namespace Slic3r
