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

#include <algorithm>

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

void FinGhostsRenderer::render(const Model& model, const ObjectID& skip_object)
{
    for (auto& [key, entry] : m_entries)
        entry.visited = false;

    bool params_refreshed = false; // at most one preset refresh per render() call

    for (const ModelObject* mo : model.objects) {
        if (mo == nullptr || mo->id() == skip_object)
            continue;
        if (std::none_of(mo->fins.begin(), mo->fins.end(),
                         [](const FinFeature& f) { return f.enabled; }))
            continue;

        // Preset-derived inputs are baked into every ghost mesh: a print/printer preset
        // switch changes them, which must invalidate the cache like any other generation
        // input (exact compare in key_stale is fine -- the same value always comes from
        // the same preset read). Resolved lazily -- only when a fin-bearing object exists
        // and at most once per RegenInterval -- see m_layer_height's declaration.
        if (!params_refreshed) {
            params_refreshed = true;
            if (const auto now = std::chrono::steady_clock::now(); now - m_params_checked >= RegenInterval) {
                m_params_checked = now;
                m_layer_height   = FinGhost::preview_layer_height();
                m_line_width     = FinGhost::preview_line_width();
            }
        }

        // Shared by all instances of this object; the instance transform is the only
        // per-instance key component.
        std::vector<VolumeKey> volume_keys;
        for (const ModelVolume* v : mo->volumes)
            if (v->is_model_part())
                volume_keys.push_back({ v->id(), v->get_matrix(), static_cast<const void*>(v->mesh_ptr().get()) });

        for (const ModelInstance* instance : mo->instances) {
            Entry& entry  = m_entries[{ mo->id().id, instance->id().id }];
            entry.visited = true;

            const Transform3d to_bed = instance->get_matrix();
            // Order matters: before the first generation entry.to_bed is a meaningless
            // identity and the delta with it is garbage -- key_stale short-circuits that.
            const bool key_stale = !entry.generated || entry.fins != mo->fins
                                || entry.volumes != volume_keys
                                || entry.layer_height != m_layer_height
                                || entry.line_width != m_line_width;
            Transform3d delta = to_bed * entry.to_bed.inverse();
            if (key_stale || !FinGeneratorInternal::is_bed_rigid_delta(delta)) {
                const auto now = std::chrono::steady_clock::now();
                if (now - entry.last_regen >= RegenInterval) {
                    // Stamped before generating: if generation ever throws and the event
                    // loop survives, retries stay throttled instead of firing every frame.
                    entry.last_regen = now;
                    regenerate(entry, *mo, to_bed);
                    entry.fins         = mo->fins;
                    entry.volumes      = volume_keys;
                    entry.layer_height = m_layer_height;
                    entry.line_width   = m_line_width;
                    delta              = Transform3d::Identity();
                }
                // else: keep drawing the stale ghost with the current (affine) delta
                // below -- it visually sticks to the object; the exact shape catches
                // up at the next regeneration (design 01 §4).
            }
            if (!entry.generated)
                continue; // throttled away before the very first generation

            for (const auto& ghost : entry.ghosts)
                FinGhost::render(*ghost, delta);
        }
    }

    // GC entries whose object/instance disappeared (delete, undo, project close).
    for (auto it = m_entries.begin(); it != m_entries.end();)
        it = it->second.visited ? std::next(it) : m_entries.erase(it);
}

void FinGhostsRenderer::regenerate(Entry& entry, const ModelObject& object, const Transform3d& to_bed)
{
    entry.ghosts.clear();
    entry.to_bed    = to_bed;
    entry.generated = true;

    // Same pipeline as slicing/the gizmo: merged bed-frame solid -> per enabled fin,
    // object-local params -> bed frame -> mesh. Degenerate instance transforms
    // (fin_object_to_bed() -> nullopt) and misses (empty mesh) are skipped -- no ghost
    // is the honest "nothing will print here" signal, matching slice-time behavior.
    const indexed_triangle_set solid = FinGeneratorInternal::object_solid_bed(object, to_bed);
    if (solid.indices.empty())
        return;
    for (const FinFeature& fin : object.fins) {
        if (!fin.enabled)
            continue;
        const std::optional<FinFeature> fin_bed = FinGeneratorInternal::fin_object_to_bed(fin, to_bed);
        if (!fin_bed.has_value())
            continue;
        auto ghost = std::make_unique<GLModel>();
        if (FinGhost::build_ghost(*ghost, solid, *fin_bed))
            entry.ghosts.push_back(std::move(ghost));
    }
}

} // namespace GUI
} // namespace Slic3r
