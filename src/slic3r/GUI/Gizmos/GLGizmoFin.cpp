#include "GLGizmoFin.hpp"
#include "slic3r/GUI/GLCanvas3D.hpp"

namespace Slic3r {
namespace GUI {

GLGizmoFin::GLGizmoFin(GLCanvas3D& parent, const std::string& icon_filename, unsigned int sprite_id)
    : GLGizmoBase(parent, icon_filename, sprite_id)
{
}

bool GLGizmoFin::on_init()
{
    return true;
}

std::string GLGizmoFin::on_get_name() const
{
    return _u8L("Fin");
}

void GLGizmoFin::on_render()
{
    // Intentionally empty: plane state and rendering are added in later tasks.
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

} // namespace GUI
} // namespace Slic3r
