#include "FinGhosts.hpp"

#include "slic3r/GUI/GUI_App.hpp"
#include "slic3r/GUI/Plater.hpp"
#include "slic3r/GUI/Camera.hpp"
#include "slic3r/GUI/GLShader.hpp"
#include "slic3r/GUI/3DScene.hpp"

#include "libslic3r/Fin/FinGenerator.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/PresetBundle.hpp"

#include <glad/gl.h>

#include <cmath>

namespace Slic3r {
namespace GUI {

namespace FinGhost {

// Ghost preview mesh color: ColorRGBA::ORANGE() at preview alpha.
static const ColorRGBA GhostColor = ColorRGBA(0.923f, 0.504f, 0.264f, 0.45f);

// The edited print preset's layer height, or `fallback` when no usable print config is
// loaded (e.g. running headless). The two call sites below need different fallbacks: a UI
// drag-snap increment (m_snap_step) vs. a physically plausible layer height (0.2mm) to
// build preview geometry with.
double preset_layer_height(double fallback)
{
    if (wxGetApp().preset_bundle != nullptr) {
        const double layer_height = wxGetApp().preset_bundle->prints.get_edited_preset().config.opt_float("layer_height");
        if (layer_height > 0.0)
            return layer_height;
    }
    return fallback;
}

// Layer height for the ghost-preview mesh (connector tab thickness, generate_one_fin()'s
// `layer_height` argument).
double preview_layer_height()
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
double preview_line_width()
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

bool build_ghost(GLModel& out, const indexed_triangle_set& solid_bed, const FinFeature& fin_bed)
{
    const TriangleMesh mesh = generate_one_fin(solid_bed, fin_bed, preview_layer_height(), preview_line_width());

    out.reset();
    if (mesh.empty())
        return false;
    out.init_from(mesh);
    out.set_color(GhostColor);
    return true;
}

void render(GLModel& ghost, const Transform3d& model_matrix)
{
    // NOT is_empty(): GLModel frees its CPU-side geometry after the first render() uploads
    // it to the GPU, flipping is_empty() to true; is_initialized() stays true for the
    // model's lifetime (see the original comment in GLGizmoFin::render_ghost()).
    if (!ghost.is_initialized())
        return;

    GLShaderProgram* shader = wxGetApp().get_shader("gouraud_light");
    if (shader == nullptr)
        return;

    glsafe(::glEnable(GL_DEPTH_TEST));
    glsafe(::glEnable(GL_BLEND));
    glsafe(::glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));

    shader->start_using();

    const Camera&      camera      = wxGetApp().plater()->get_camera();
    const Transform3d& view_matrix = camera.get_view_matrix();
    shader->set_uniform("projection_matrix", camera.get_projection_matrix());
    shader->set_uniform("view_model_matrix", view_matrix * model_matrix);
    // Rotate lighting normals together with the mesh (gouraud_light.vs consumes
    // view_normal_matrix; the original gizmo-only render never set it and inherited
    // whatever the previous draw left behind -- correct for its identity model matrix,
    // wrong for a rotated delta). Standard formula: inverse-transpose of the linear part.
    const Matrix3d view_normal_matrix = view_matrix.matrix().block(0, 0, 3, 3)
                                      * model_matrix.matrix().block(0, 0, 3, 3).inverse().transpose();
    shader->set_uniform("view_normal_matrix", view_normal_matrix);
    shader->set_uniform("emission_factor", 0.2f);

    ghost.render();

    shader->stop_using();

    glsafe(::glDisable(GL_BLEND));
}

} // namespace FinGhost

} // namespace GUI
} // namespace Slic3r
