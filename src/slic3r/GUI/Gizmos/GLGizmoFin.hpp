#ifndef slic3r_GLGizmoFin_hpp_
#define slic3r_GLGizmoFin_hpp_

#include "GLGizmoBase.hpp"

namespace Slic3r {

namespace GUI {

class GLGizmoFin : public GLGizmoBase
{
public:
    GLGizmoFin(GLCanvas3D& parent, const std::string& icon_filename, unsigned int sprite_id);

protected:
    bool        on_init() override;
    std::string on_get_name() const override;
    void        on_render() override;
    bool        on_is_selectable() const override;
    bool        on_is_activable() const override;
};

} // namespace GUI
} // namespace Slic3r

#endif // slic3r_GLGizmoFin_hpp_
