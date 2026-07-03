#ifndef slic3r_GLGizmoFin_hpp_
#define slic3r_GLGizmoFin_hpp_

#include "GLGizmoBase.hpp"
#include "slic3r/GUI/GLModel.hpp"

namespace Slic3r {

namespace GUI {

// Vertical-only plane gizmo for the Fin feature: translation on the bed, azimuth
// (Z) rotation, and two linear extent handles (height H, outward length L).
//
// Constraint: the plane never tilts. Only m_plane_center (XY; z is always kept
// at the bed) and m_z_angle change -- there is no pitch/roll under any interaction.
//
// The local plane frame mirrors FinGeneratorInternal::plane_to_bed()'s convention
// (libslic3r/Fin/FinGenerator.hpp), restricted to the vertical-only case:
//   u = horizontal, in-plane, the fin extends outward along +u
//   w = vertical (== world Z)
//   n = horizontal plane normal
// At z_angle == 0: u == +Y world, n == +X world.
//
// This gizmo only owns transient UI state; later tasks read/write it from/to a
// FinFeature on the object (T00-04-05) and add an ImGui panel (T00-04-03) and a
// ghost mesh preview (T00-04-04).
class GLGizmoFin : public GLGizmoBase
{
    enum GrabberID {
        Move = 0,    // translate the plane on the bed (XY)
        ZRotation,   // rotate the plane's azimuth around its center
        TopHeight,   // drag along world Z: sets m_height
        OuterLength, // drag along local u: sets m_length
        Count
    };

    // Plane state.
    Vec3d  m_plane_center{ Vec3d::Zero() }; // world XY; z is always kept at the bed (0)
    double m_z_angle{ 0.0 };                // azimuth about world Z
    double m_height{ 10.0 };                // H: plane extent in Z, from the bed
    double m_length{ 8.0 };                 // L: plane extent along u, outward

    // Fin generation parameters not yet reflected in the plane geometry (T00-04-04 will use
    // them for the ghost preview, T00-04-05 will persist them to ModelObject::fins). Defaults
    // mirror FinFeature's; reset alongside the plane state in init_from_selection().
    double m_thickness{ 1.2 };  // t: fin wall thickness
    double m_conn_step{ 4.0 };  // X: vertical spacing of connectors
    double m_conn_gap{ 1.0 };   // gap: connector length / contour-follow distance
    bool   m_enabled{ true };   // toggled by the panel checkbox; semantics land in T00-04-05

    double m_snap_step{ 1.0 };              // mm, translation / length snap with Shift held

    // ImGui side panel layout (mirrors GLGizmoCut3D's m_label_width / m_control_width idiom).
    float m_label_width{ 0.f };
    float m_control_width{ 200.f };

    // Dragging scratch state, captured in on_start_dragging() / the first on_dragging() call.
    Vec3d  m_drag_start_center{ Vec3d::Zero() };
    double m_drag_start_z_angle{ 0.0 };
    double m_drag_start_height{ 0.0 };
    double m_drag_start_length{ 0.0 };
    Vec3d  m_drag_start_world_pos{ Vec3d::Zero() }; // grabber's world position at drag start (H/L axis reference)
    bool   m_drag_ref_captured{ false };            // Move/ZRotation: whether the bed-plane reference below was captured yet
    Vec3d  m_drag_ref_bed_point{ Vec3d::Zero() };   // Move: bed hit at the first dragging frame
    double m_drag_ref_angle{ 0.0 };                 // ZRotation: bed-plane angle at the first dragging frame

    GLModel m_plane_fill;    // unit quad (0..1 x 0..1), scaled to (m_length, m_height) at render time
    GLModel m_plane_border;
    GLModel m_rotation_ring; // unit circle, scaled to rotation_ring_radius() at render time
    bool    m_rendering_items_ready{ false };

public:
    GLGizmoFin(GLCanvas3D& parent, const std::string& icon_filename, unsigned int sprite_id);

    bool on_mouse(const wxMouseEvent& mouse_event) override;

protected:
    bool        on_init() override;
    std::string on_get_name() const override;
    void        on_set_state() override;
    void        on_render() override;
    void        on_render_input_window(float x, float y, float bottom_limit) override;
    void        on_start_dragging() override;
    void        on_dragging(const UpdateData& data) override;
    void        on_stop_dragging() override;
    bool        on_is_selectable() const override;
    bool        on_is_activable() const override;
    void        on_register_raycasters_for_picking() override;
    void        on_unregister_raycasters_for_picking() override;

private:
    void init_from_selection();
    void init_rendering_items();
    void update_grabbers(const Transform3d& plane_m);
    void render_plane(const Transform3d& plane_m);
    void render_rotation_ring();

    Transform3d plane_matrix() const;
    double      rotation_ring_radius() const;
    double      get_layer_height_snap_step() const;
    double      calc_axis_projection(const UpdateData& data, const Vec3d& ref_point, const Vec3d& axis_dir, double snap_step) const;

    // ImGui side panel (T00-04-03).
    void init_input_window_layout();
    bool render_double_field(const std::string& label, const char* imgui_id, double& value, double min_val, const char* format);
};

} // namespace GUI
} // namespace Slic3r

#endif // slic3r_GLGizmoFin_hpp_
