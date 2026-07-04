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
#include <limits>

namespace Slic3r {
namespace GUI {

namespace FinGhost {

// Ghost preview mesh color: ColorRGBA::ORANGE() at preview alpha.
static const ColorRGBA GhostColor = ColorRGBA(0.923f, 0.504f, 0.264f, 0.45f);

// Hovered pick target (design 03): same hue, brighter and more opaque, so clickability
// is visible without stealing attention from the edited fin.
static const ColorRGBA GhostHoverColor = ColorRGBA(1.0f, 0.62f, 0.35f, 0.65f);

// The edited print preset's layer height, or `fallback` when no usable print config is
// loaded (e.g. running headless). The two call sites need different fallbacks: a UI
// drag-snap increment (m_snap_step, GLGizmoFin::get_layer_height_snap_step()) vs. a
// physically plausible layer height (0.2mm, preview_layer_height() below) to build
// preview geometry with.
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

bool build_ghost(GLModel& out, const indexed_triangle_set& solid_bed, const FinFeature& fin_bed,
                 TriangleMesh* mesh_out)
{
    TriangleMesh mesh = generate_one_fin(solid_bed, fin_bed, preview_layer_height(), preview_line_width());

    out.reset();
    if (mesh.empty()) {
        if (mesh_out != nullptr)
            *mesh_out = TriangleMesh();
        return false;
    }
    out.init_from(mesh);
    out.set_color(GhostColor);
    if (mesh_out != nullptr)
        *mesh_out = std::move(mesh);
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

void FinGhostsRenderer::render(const Model& model, const GLVolumeCollection& volumes)
{
    for (auto& [key, entry] : m_entries)
        entry.visited = false;

    // Early-out for fin-less scenes: skip the per-frame live_trafos build below. Entries
    // still need the GC (all unvisited by now) -- deleting an object's last fin must drop
    // its ghost this very frame.
    if (std::none_of(model.objects.begin(), model.objects.end(),
                     [](const ModelObject* mo) { return mo != nullptr && !mo->fins.empty(); })) {
        m_entries.clear();
        return;
    }

    // Live per-instance world transforms (see the header comment): first GLVolume of an
    // (object, instance) pair wins -- all of an instance's volumes share the instance trafo.
    std::map<std::pair<int, int>, Transform3d> live_trafos;
    for (const GLVolume* v : volumes.volumes)
        if (v != nullptr)
            live_trafos.emplace(std::make_pair(v->object_idx(), v->instance_idx()),
                                v->get_instance_transformation().get_matrix());

    bool params_refreshed = false; // at most one preset refresh per render() call

    for (int object_idx = 0; object_idx < int(model.objects.size()); ++object_idx) {
        const ModelObject* mo = model.objects[object_idx];
        if (mo == nullptr)
            continue;
        if (mo->fins.empty())
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

        for (int instance_idx = 0; instance_idx < int(mo->instances.size()); ++instance_idx) {
            const ModelInstance* instance = mo->instances[instance_idx];
            Entry& entry  = m_entries[{ mo->id().id, instance->id().id }];
            entry.visited = true;

            const auto        live   = live_trafos.find({ object_idx, instance_idx });
            const Transform3d to_bed = live != live_trafos.end() ? live->second : instance->get_matrix();
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

            entry.last_delta = delta;
            for (const auto& ghost : entry.ghosts) {
                if (m_selected.valid() && mo->id() == m_selected.object && ghost.fin_idx == m_selected.fin_idx)
                    continue; // the Fin gizmo draws this fin's live ghost itself (design 03)
                const bool hovered = m_hovered.valid() && mo->id() == m_hovered.object
                                  && instance->id() == m_hovered.instance
                                  && ghost.fin_idx == m_hovered.fin_idx;
                // set_color() is a plain stored-value write -- cheap to do per frame.
                ghost.model->set_color(hovered ? FinGhost::GhostHoverColor : FinGhost::GhostColor);
                FinGhost::render(*ghost.model, delta);
            }
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

    // Same pipeline as slicing/the gizmo: merged bed-frame solid -> per fin,
    // object-local params -> bed frame -> mesh. Degenerate instance transforms
    // (fin_object_to_bed() -> nullopt) and misses (empty mesh) are skipped -- no ghost
    // is the honest "nothing will print here" signal, matching slice-time behavior.
    const indexed_triangle_set solid = FinGeneratorInternal::object_solid_bed(object, to_bed);
    if (solid.indices.empty())
        return;
    for (int fin_idx = 0; fin_idx < int(object.fins.size()); ++fin_idx) {
        const std::optional<FinFeature> fin_bed =
            FinGeneratorInternal::fin_object_to_bed(object.fins[fin_idx], to_bed);
        if (!fin_bed.has_value())
            continue;
        Entry::Ghost ghost;
        ghost.model = std::make_unique<GLModel>();
        TriangleMesh mesh;
        if (!FinGhost::build_ghost(*ghost.model, solid, *fin_bed, &mesh))
            continue;
        ghost.raycaster = std::make_unique<MeshRaycaster>(std::make_shared<const TriangleMesh>(std::move(mesh)));
        ghost.fin_idx   = fin_idx;
        entry.ghosts.push_back(std::move(ghost));
    }
}

std::optional<FinPick> FinGhostsRenderer::pick(const Vec2d& mouse_pos, const Camera& camera, const Model& model) const
{
    // Entries are keyed by stable ObjectIDs; Selection wants indices -- resolve via the
    // live model. An entry whose object/instance vanished since the last render simply
    // fails the lookup (GC happens on render, not here).
    std::map<size_t, int>                    object_idx_by_id;
    std::map<std::pair<size_t, size_t>, int> instance_idx_by_id;
    for (int oi = 0; oi < int(model.objects.size()); ++oi) {
        const ModelObject* mo = model.objects[oi];
        if (mo == nullptr)
            continue;
        object_idx_by_id[mo->id().id] = oi;
        for (int ii = 0; ii < int(mo->instances.size()); ++ii)
            instance_idx_by_id[{ mo->id().id, mo->instances[ii]->id().id }] = ii;
    }

    std::optional<FinPick> best;
    double                 best_dist_sq = std::numeric_limits<double>::max();
    for (const auto& [key, entry] : m_entries) {
        if (!entry.generated)
            continue;
        const auto oi = object_idx_by_id.find(key.first);
        const auto ii = instance_idx_by_id.find(key);
        if (oi == object_idx_by_id.end() || ii == instance_idx_by_id.end())
            continue;
        for (const auto& ghost : entry.ghosts) {
            if (m_selected.valid() && ObjectID(key.first) == m_selected.object
                && ghost.fin_idx == m_selected.fin_idx)
                continue; // not drawn => not pickable
            if (ghost.raycaster == nullptr)
                continue;
            Vec3f position, normal;
            if (!ghost.raycaster->closest_hit(mouse_pos, entry.last_delta, camera, position, normal))
                continue;
            const Vec3d  world   = entry.last_delta * position.cast<double>();
            const double dist_sq = (world - camera.get_position()).squaredNorm();
            if (dist_sq < best_dist_sq) {
                best_dist_sq = dist_sq;
                best         = FinPick{ oi->second, ii->second, ghost.fin_idx,
                                        ObjectID(key.first), ObjectID(key.second) };
            }
        }
    }
    return best;
}

} // namespace GUI
} // namespace Slic3r
