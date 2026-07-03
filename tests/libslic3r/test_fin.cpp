#include <catch2/catch_all.hpp>

#include "libslic3r/Model.hpp"
#include "libslic3r/Fin/FinGenerator.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include "libslic3r/Utils.hpp"

#include <cereal/archives/binary.hpp>

#include <boost/filesystem/operations.hpp>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <optional>
#include <sstream>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

TEST_CASE("FinFeature defaults are sane", "[fin]")
{
    FinFeature f;
    CHECK(f.enabled);
    CHECK_THAT(f.conn_gap, WithinAbs(1.0, 1e-9)); // gap == connector length ~1mm
    CHECK(f.thickness > 0.0);
}

TEST_CASE("ModelObject carries fins", "[fin]")
{
    Model model;
    ModelObject* o = model.add_object();
    o->fins.push_back(FinFeature{});
    o->fins.back().height = 25.0;
    REQUIRE(o->fins.size() == 1);
    CHECK_THAT(o->fins.front().height, WithinAbs(25.0, 1e-9));
}

TEST_CASE("fins survive ModelObject copy", "[fin]")
{
    Model model;
    ModelObject* o = model.add_object();
    o->fins.push_back(FinFeature{});
    o->fins.back().height = 42.0;

    // Model::add_object(const ModelObject&) clones through
    // ModelObject::new_clone -> assign_clone -> assign_copy(const ModelObject&),
    // the same path Print::apply() uses to copy the model for background slicing.
    ModelObject* clone = model.add_object(*o);
    REQUIRE(clone->fins.size() == 1);
    CHECK_THAT(clone->fins.front().height, WithinAbs(42.0, 1e-9));
}

TEST_CASE("FinFeature cereal roundtrip preserves all fields", "[fin]")
{
    FinFeature src;
    src.anchor_xy = Vec2d(1.5, -2.5);
    src.z_angle   = 0.75;
    src.height    = 21.0;
    src.length    = 6.5;
    src.thickness = 0.9;
    src.conn_step = 3.0;
    src.conn_gap  = 1.25;
    src.enabled   = false;

    std::stringstream ss;
    {
        cereal::BinaryOutputArchive oarchive(ss);
        oarchive(src);
    }

    FinFeature dst;
    {
        cereal::BinaryInputArchive iarchive(ss);
        iarchive(dst);
    }

    CHECK_THAT(dst.anchor_xy.x(), WithinAbs(1.5, 1e-9));
    CHECK_THAT(dst.anchor_xy.y(), WithinAbs(-2.5, 1e-9));
    CHECK_THAT(dst.z_angle, WithinAbs(0.75, 1e-9));
    CHECK_THAT(dst.height, WithinAbs(21.0, 1e-9));
    CHECK_THAT(dst.length, WithinAbs(6.5, 1e-9));
    CHECK_THAT(dst.thickness, WithinAbs(0.9, 1e-9));
    CHECK_THAT(dst.conn_step, WithinAbs(3.0, 1e-9));
    CHECK_THAT(dst.conn_gap, WithinAbs(1.25, 1e-9));
    CHECK_FALSE(dst.enabled);
}

// --- FinFeature: .3mf persistence (T00-03-01) -------------------------------

namespace {

// Round-trips `src` through a real bbs_3mf save+load cycle (SaveStrategy::Silence
// skips the backup-path side effects a full app export would otherwise trigger).
// Returns true iff both the store and the load succeeded.
bool roundtrip_bbs_3mf(Model& src, Model& dst, const std::string& path)
{
    // store_bbs_3mf() stages a few scratch config files under a "backup" dir
    // rooted at temporary_dir(); in a fresh test process that global is unset
    // (it's normally set once by app startup), which resolves to the
    // filesystem root. Point it at the OS temp dir before saving.
    if (temporary_dir().empty())
        set_temporary_dir(std::filesystem::temp_directory_path().string());

    DynamicPrintConfig store_config;
    StoreParams        store_params;
    store_params.path     = path.c_str();
    store_params.model    = &src;
    store_params.config   = &store_config;
    store_params.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;
    if (!store_bbs_3mf(store_params))
        return false;

    DynamicPrintConfig        load_config;
    ConfigSubstitutionContext ctxt{ForwardCompatibilitySubstitutionRule::Disable};
    PlateDataPtrs             plate_data_list;
    std::vector<Preset*>      project_presets;
    bool                      is_bbl_3mf = false;
    Semver                    file_version;
    bool load_ok = load_bbs_3mf(path.c_str(), &load_config, &ctxt, &dst, &plate_data_list, &project_presets,
                                 &is_bbl_3mf, nullptr, &file_version, nullptr,
                                 LoadStrategy::LoadModel | LoadStrategy::LoadConfig);
    release_PlateData_list(plate_data_list);
    return load_ok;
}

} // namespace

TEST_CASE("fin: survives 3mf save/load roundtrip", "[fin]")
{
    Model        m;
    ModelObject* o = m.add_object();
    o->add_volume(TriangleMesh(its_make_cube(10, 10, 10)));
    o->add_instance();

    // Non-default values on every field, including the Vec2d and the bool, so
    // the roundtrip actually exercises all of FinFeature's persisted state.
    FinFeature f1;
    f1.anchor_xy = Vec2d(3.25, -1.75);
    f1.z_angle   = 1.23;
    f1.height    = 17.5;
    f1.length    = 9.25;
    f1.thickness = 1.4;
    f1.conn_step = 5.5;
    f1.conn_gap  = 0.9;
    f1.enabled   = true;
    o->fins.push_back(f1);

    FinFeature f2;
    f2.anchor_xy = Vec2d(-4.5, 6.125);
    f2.z_angle   = -2.5;
    f2.height    = 8.0;
    f2.length    = 3.0;
    f2.thickness = 0.8;
    f2.conn_step = 2.5;
    f2.conn_gap  = 1.75;
    f2.enabled   = false;
    o->fins.push_back(f2);

    std::string path = std::string(TEST_DATA_DIR) + "/test_3mf/fin_roundtrip.3mf";

    Model dst;
    bool  ok = roundtrip_bbs_3mf(m, dst, path);
    boost::filesystem::remove(path);
    REQUIRE(ok);

    REQUIRE(dst.objects.size() == 1);
    REQUIRE(dst.objects.front()->fins.size() == 2);

    const FinFeature& r1 = dst.objects.front()->fins[0];
    CHECK_THAT(r1.anchor_xy.x(), WithinAbs(f1.anchor_xy.x(), 1e-6));
    CHECK_THAT(r1.anchor_xy.y(), WithinAbs(f1.anchor_xy.y(), 1e-6));
    CHECK_THAT(r1.z_angle,       WithinAbs(f1.z_angle, 1e-6));
    CHECK_THAT(r1.height,        WithinAbs(f1.height, 1e-6));
    CHECK_THAT(r1.length,        WithinAbs(f1.length, 1e-6));
    CHECK_THAT(r1.thickness,     WithinAbs(f1.thickness, 1e-6));
    CHECK_THAT(r1.conn_step,     WithinAbs(f1.conn_step, 1e-6));
    CHECK_THAT(r1.conn_gap,      WithinAbs(f1.conn_gap, 1e-6));
    CHECK(r1.enabled == f1.enabled);

    const FinFeature& r2 = dst.objects.front()->fins[1];
    CHECK_THAT(r2.anchor_xy.x(), WithinAbs(f2.anchor_xy.x(), 1e-6));
    CHECK_THAT(r2.anchor_xy.y(), WithinAbs(f2.anchor_xy.y(), 1e-6));
    CHECK_THAT(r2.z_angle,       WithinAbs(f2.z_angle, 1e-6));
    CHECK_THAT(r2.height,        WithinAbs(f2.height, 1e-6));
    CHECK_THAT(r2.length,        WithinAbs(f2.length, 1e-6));
    CHECK_THAT(r2.thickness,     WithinAbs(f2.thickness, 1e-6));
    CHECK_THAT(r2.conn_step,     WithinAbs(f2.conn_step, 1e-6));
    CHECK_THAT(r2.conn_gap,      WithinAbs(f2.conn_gap, 1e-6));
    CHECK(r2.enabled == f2.enabled);
}

TEST_CASE("fin: loading a 3mf without fins is fine", "[fin]")
{
    // No fins on the object at all: the writer must skip fin_information.xml
    // entirely (mirrors an old project saved before this feature existed),
    // and the reader must not choke on its absence.
    Model        m;
    ModelObject* o = m.add_object();
    o->add_volume(TriangleMesh(its_make_cube(10, 10, 10)));
    o->add_instance();

    std::string path = std::string(TEST_DATA_DIR) + "/test_3mf/fin_none.3mf";

    Model dst;
    bool  ok = roundtrip_bbs_3mf(m, dst, path);
    boost::filesystem::remove(path);
    REQUIRE(ok);

    REQUIRE(dst.objects.size() == 1);
    CHECK(dst.objects.front()->fins.empty());
}

// --- FinGenerator: plane-frame <-> bed-frame -------------------------------

TEST_CASE("plane_to_bed maps the local origin to the anchor", "[fin]")
{
    FinFeature f;
    f.anchor_xy = Vec2d(3.0, -4.0);
    f.z_angle   = 0.37; // arbitrary, non-axis-aligned

    Transform3d plane_to_bed = FinGeneratorInternal::plane_to_bed(f);
    Vec3d       p            = plane_to_bed * Vec3d(0.0, 0.0, 0.0);

    CHECK_THAT(p.x(), WithinAbs(f.anchor_xy.x(), 1e-9));
    CHECK_THAT(p.y(), WithinAbs(f.anchor_xy.y(), 1e-9));
    CHECK_THAT(p.z(), WithinAbs(0.0, 1e-9));
}

TEST_CASE("plane_to_bed at z_angle=0 locks the coordinate contract", "[fin]")
{
    // Design contract (design doc §13): with z_angle = 0, u (local x, outward)
    // maps to world +Y, and n (local z, plane normal) maps to world +X.
    FinFeature f;
    f.anchor_xy = Vec2d::Zero();
    f.z_angle   = 0.0;

    Transform3d plane_to_bed = FinGeneratorInternal::plane_to_bed(f);
    Vec3d       u_world      = plane_to_bed.linear() * Vec3d(1.0, 0.0, 0.0); // local x = u
    Vec3d       n_world      = plane_to_bed.linear() * Vec3d(0.0, 0.0, 1.0); // local z = n

    CHECK_THAT(u_world.x(), WithinAbs(0.0, 1e-9));
    CHECK_THAT(u_world.y(), WithinAbs(1.0, 1e-9));
    CHECK_THAT(u_world.z(), WithinAbs(0.0, 1e-9));

    CHECK_THAT(n_world.x(), WithinAbs(1.0, 1e-9));
    CHECK_THAT(n_world.y(), WithinAbs(0.0, 1e-9));
    CHECK_THAT(n_world.z(), WithinAbs(0.0, 1e-9));
}

TEST_CASE("plane_to_bed at z_angle=PI/2 pins the rotation sign", "[fin]")
{
    // At z_angle = 0 the -sin term of u vanishes, so a sign flip there would be
    // invisible to the axis test above. Pin it at PI/2:
    // u = (-sin, cos, 0) -> (-1, 0, 0), n = (cos, sin, 0) -> (0, 1, 0), w stays Z.
    FinFeature f;
    f.anchor_xy = Vec2d::Zero();
    f.z_angle   = PI / 2.0;

    Transform3d plane_to_bed = FinGeneratorInternal::plane_to_bed(f);
    Vec3d       u_world      = plane_to_bed.linear() * Vec3d(1.0, 0.0, 0.0); // local x = u
    Vec3d       w_world      = plane_to_bed.linear() * Vec3d(0.0, 1.0, 0.0); // local y = w
    Vec3d       n_world      = plane_to_bed.linear() * Vec3d(0.0, 0.0, 1.0); // local z = n

    CHECK_THAT(u_world.x(), WithinAbs(-1.0, 1e-9));
    CHECK_THAT(u_world.y(), WithinAbs(0.0, 1e-9));
    CHECK_THAT(u_world.z(), WithinAbs(0.0, 1e-9));

    CHECK_THAT(w_world.x(), WithinAbs(0.0, 1e-9));
    CHECK_THAT(w_world.y(), WithinAbs(0.0, 1e-9));
    CHECK_THAT(w_world.z(), WithinAbs(1.0, 1e-9));

    CHECK_THAT(n_world.x(), WithinAbs(0.0, 1e-9));
    CHECK_THAT(n_world.y(), WithinAbs(1.0, 1e-9));
    CHECK_THAT(n_world.z(), WithinAbs(0.0, 1e-9));
}

TEST_CASE("bed_to_plane is the inverse of plane_to_bed", "[fin]")
{
    FinFeature f;
    f.anchor_xy = Vec2d(1.5, 2.5);
    f.z_angle   = 1.1;

    Transform3d roundtrip = FinGeneratorInternal::bed_to_plane(f) * FinGeneratorInternal::plane_to_bed(f);
    Vec3d       p         = roundtrip * Vec3d(7.0, -3.0, 2.0);

    CHECK_THAT(p.x(), WithinAbs(7.0, 1e-9));
    CHECK_THAT(p.y(), WithinAbs(-3.0, 1e-9));
    CHECK_THAT(p.z(), WithinAbs(2.0, 1e-9));
}

// --- FinGenerator: solid assembly ------------------------------------------

TEST_CASE("object_solid_bed merges MODEL_PART volumes and skips other types", "[fin]")
{
    Model model;
    ModelObject* o = model.add_object();

    ModelVolume* part = o->add_volume(make_cube(10.0, 10.0, 10.0), /*modify_to_center_geometry=*/false);
    REQUIRE(part->is_model_part());

    ModelVolume* modifier = o->add_volume(make_cube(1.0, 1.0, 1.0), /*modify_to_center_geometry=*/false);
    modifier->set_type(ModelVolumeType::PARAMETER_MODIFIER);

    indexed_triangle_set solid = FinGeneratorInternal::object_solid_bed(*o, Transform3d::Identity());

    CHECK(solid.indices.size() == part->mesh().its.indices.size());
    CHECK(solid.vertices.size() == part->mesh().its.vertices.size());
}

TEST_CASE("object_solid_bed returns an empty solid when there is no model part", "[fin]")
{
    Model model;
    ModelObject* o = model.add_object();

    indexed_triangle_set solid = FinGeneratorInternal::object_solid_bed(*o, Transform3d::Identity());
    CHECK(solid.indices.empty());
    CHECK(solid.vertices.empty());
}

// --- FinGenerator: generate_one_fin stub ------------------------------------

TEST_CASE("generate_one_fin returns an empty mesh for an empty solid", "[fin]")
{
    indexed_triangle_set empty_solid;
    FinFeature            f;

    TriangleMesh fin = generate_one_fin(empty_solid, f, /*layer_height=*/0.2, /*line_width=*/0.4);
    CHECK(fin.empty());
}

// --- FinGenerator: cross-section at the plane -------------------------------

TEST_CASE("fin: cross-section of a centered cube is non-empty", "[fin]")
{
    indexed_triangle_set cube = its_make_cube(20, 20, 20);       // 0..20 box
    its_translate(cube, Vec3f(-10.f, -10.f, 0.f));               // center XY on origin, base on bed

    FinFeature f;
    f.anchor_xy = Vec2d(0.0, 0.0);
    f.z_angle   = 0.0;
    f.height    = 15.0;
    // Window semantics (design doc §15): the anchor sits at the cube's own center along u, so the
    // section's surface is at local u = 10 (half-width) and the window's near edge (after the gap)
    // is at u = 11. length must clear that to leave any window open at all -- 16 (> 11) does, with
    // room to spare; length=8 (pre-§15) would land the whole window inside the part+gap -> empty.
    f.length    = 16.0;

    CHECK_FALSE(generate_one_fin(cube, f, /*layer_height=*/0.2, /*line_width=*/0.45).empty());
}

TEST_CASE("fin: plane missing the object yields empty mesh", "[fin]")
{
    indexed_triangle_set cube = its_make_cube(10, 10, 10);
    its_translate(cube, Vec3f(50.f, 50.f, 0.f)); // far from the plane

    FinFeature f;
    f.anchor_xy = Vec2d(0.0, 0.0);
    f.z_angle   = 0.0;

    CHECK(generate_one_fin(cube, f, /*layer_height=*/0.2, /*line_width=*/0.45).empty());
}

// --- FinGenerator: 2D fin region (truncated rectangle) ----------------------

TEST_CASE("fin: 2D region is a truncated rectangle on one side", "[fin]")
{
    indexed_triangle_set cube = its_make_cube(20, 20, 30);
    its_translate(cube, Vec3f(-10.f, -10.f, 0.f));

    FinFeature f;
    f.anchor_xy = Vec2d(0.0, 0.0);
    f.z_angle   = 0.0;
    f.height    = 20.0;
    // Window semantics (design doc §15): with the anchor at the cube's own center (u=0), the
    // section's far-side surface sits at local u = 10 and the window's near edge (past the gap) at
    // u = 11 -- length must exceed that for the window to have any room left, on EITHER side of the
    // flip below (center-anchoring is symmetric, so one length works for both directions, unlike
    // moving the anchor to a single face). 16 gives a clean 5mm-wide slice, same width the old
    // bbox-rectangle model produced with length=6 (kept here only as a memorable round number, not
    // because the exact width matters to this test).
    f.length    = 16.0;
    f.conn_gap  = 1.0;

    TriangleMesh m = generate_one_fin(cube, f, /*layer_height=*/0.2, /*line_width=*/0.45);
    REQUIRE_FALSE(m.empty());

    BoundingBoxf3 bb = m.bounding_box();
    CHECK_THAT(bb.size().z(), WithinAbs(20.0, 0.5)); // height H
    CHECK(bb.max.y() > 10.0);                        // fin extends along +u == +Y beyond the cube

    // flipped: z_angle += PI swaps which side is +u -> mirrored to -Y
    FinFeature g = f;
    g.z_angle    = PI;
    BoundingBoxf3 bbg = generate_one_fin(cube, g, /*layer_height=*/0.2, /*line_width=*/0.45).bounding_box();
    CHECK(bbg.min.y() < -10.0);
}

TEST_CASE("fin: gap offset swallowing the whole rectangle yields empty mesh", "[fin]")
{
    indexed_triangle_set cube = its_make_cube(10, 10, 10);
    its_translate(cube, Vec3f(-5.f, -5.f, 0.f));

    FinFeature f;
    f.anchor_xy = Vec2d(0.0, 0.0);
    f.z_angle   = 0.0;
    f.height    = 8.0;
    f.length    = 6.0;
    f.conn_gap  = 100.0; // huge vs. length/section: offset(contour) covers the whole rect

    CHECK(generate_one_fin(cube, f, /*layer_height=*/0.2, /*line_width=*/0.45).empty());
}

// --- FinGenerator: prism extrusion (real wall thickness) -------------------

TEST_CASE("fin: wall has the requested thickness", "[fin]")
{
    indexed_triangle_set cube = its_make_cube(20, 20, 30);
    its_translate(cube, Vec3f(-10, -10, 0));

    FinFeature f;
    f.anchor_xy  = Vec2d(0.0, 0.0);
    f.z_angle    = 0.0;
    f.height     = 15.0;
    // Window semantics (design doc §15): center anchor -> surface at local u=10, window's near edge
    // at u=11 (default conn_gap=1.0) -- length must exceed that or the fin is empty (length=6, the
    // pre-§15 value, would be). This test only cares about the wall's thickness, not its width.
    f.length     = 16.0;
    f.thickness  = 1.2;

    BoundingBoxf3 bb = generate_one_fin(cube, f, /*layer_height=*/0.2, /*line_width=*/0.45).bounding_box();
    CHECK_THAT(bb.size().x(), WithinAbs(1.2, 0.05)); // thickness along n == X at angle 0
}

TEST_CASE("fin: wall mesh is a closed, outward-facing solid (positive volume)", "[fin]")
{
    indexed_triangle_set cube = its_make_cube(20, 20, 30);
    its_translate(cube, Vec3f(-10, -10, 0));

    FinFeature f;
    f.anchor_xy = Vec2d(0.0, 0.0);
    f.z_angle   = 0.0;
    f.height    = 15.0;
    // Window semantics (design doc §15): center anchor -> section's far-side surface at local
    // u=10, window's near edge at u=11 (default conn_gap=1.0). length=16 (> 11) leaves a real
    // window open; the pre-§15 value (6) would land the whole window inside the part+gap -> empty.
    // 16 is also chosen so the *width* below (length - (surface+gap) = 16-11 = 5) matches exactly
    // what the old bbox-rectangle model produced with length=6, so every number below this point
    // (area/volume/tab math) is unchanged from before the §15 rework.
    f.length    = 16.0;
    f.thickness = 1.2;
    // f.conn_step (4.0) and f.conn_gap (1.0) are left at their defaults.

    double layer_height = 0.2, line_width = 0.45;
    TriangleMesh m = generate_one_fin(cube, f, layer_height, line_width);
    REQUIRE_FALSE(m.empty());
    CHECK(m.volume() > 0.0);

    // The prism must be a welded shared-vertex mesh, not a per-facet triangle
    // soup: a soup slices fine (position-based gap closing) but reports every
    // edge as open/non-manifold once attached as a ModelVolume, triggering GUI
    // repair warnings and disabling gizmos.
    CHECK(m.stats().open_edges == 0); // closed (manifold) surface: the wall shell
                                       // AND every tab cube (its own closed box) are.

    // Connector tabs (T00-01-05) are separate its_merge()'d shells -- no shared
    // vertices with the wall or with each other (its_merge doesn't weld; see
    // FinGenerator.cpp) -- so number_of_parts is exactly 1 (wall) + one per tab
    // row, not 1: rows are w = 0, 4, 8, 12 (floor(15/4)+1 = 4 rows, conn_step=4
    // default), all <= height=15 and all within the cube's full w in [0,30]
    // cross-section, so every row gets a tab.
    int n_tab_rows = FinGeneratorInternal::fin_connector_rows(f);
    REQUIRE(n_tab_rows == 4);
    CHECK(m.stats().number_of_parts == 1 + n_tab_rows); // one shell + one per tab

    // fin2d is exactly known here (design doc §15): the plane (bed X=0) cuts the cube into a full
    // u in [-10,10], w in [0,30] cross-section, so the section's far-side surface is at local
    // u=10 for every w in [0,30]. The window is u in [0,16] (length=16), w in [0,15] (=height,
    // capped at min(15, model top=30) = 15, no capping here). The shadow covers u >= 10 everywhere
    // within the window's own w-range, so candidate = window itself; the default 1mm conn_gap then
    // carves out u in [-11,11] (offset_sec), leaving fin2d = u in [11,16] x w in [0,15] ->
    // area 5*15 = 75 mm^2 -- the fill-to-bed step adds nothing new since candidate already reaches
    // w=0. volume = area * thickness confirms both the extrusion geometry and the winding (a
    // flipped/self-intersecting prism would not match this exactly).
    double wall_volume = 75.0 * f.thickness;
    // Each tab spans d(w) + 2*kTabOverlap along u, d(w) = inner(w) - surface_u(w) (generate_one_fin,
    // design doc §15). Here surface_u=10 and inner=min(surface_u+conn_gap, length)=11 (the window
    // doesn't clamp it, since length=16 > 11), so d=conn_gap=1.0 exactly like the pre-§15 formula --
    // same its_make_cube(conn_gap + 0.2, layer_height, 2*line_width) as before.
    double tab_volume      = (f.conn_gap + 0.2) * layer_height * (2.0 * line_width);
    double expected_volume = wall_volume + n_tab_rows * tab_volume;
    CHECK_THAT(m.volume(), WithinAbs(expected_volume, 0.05));
}

// --- FinGenerator: connector tabs (T00-01-05) -------------------------------

TEST_CASE("fin: fin_connector_rows == floor(H/X) + 1", "[fin]")
{
    FinFeature f;
    f.height    = 20.0;
    f.conn_step = 5.0;
    CHECK(FinGeneratorInternal::fin_connector_rows(f) == 5); // w = 0,5,10,15,20

    f.conn_step = 100.0;
    CHECK(FinGeneratorInternal::fin_connector_rows(f) == 1); // only w = 0
}

TEST_CASE("fin: connector count = floor(H/X)+1", "[fin]")
{
    indexed_triangle_set cube = its_make_cube(20, 20, 40);
    its_translate(cube, Vec3f(-10, -10, 0));

    FinFeature f;
    f.anchor_xy = Vec2d(0.0, 0.0);
    f.z_angle   = 0.0;
    f.height    = 20.0;
    // Window semantics (design doc §15): center anchor -> surface at local u=10, window's near
    // edge at u=11 -- length must exceed that (the pre-§15 value, 6, would leave the fin empty).
    // The exact width doesn't matter to this test (only tab counts/volume comparisons do).
    f.length    = 16.0;
    f.conn_step = 5.0;
    f.conn_gap  = 1.0;

    TriangleMesh with_tabs = generate_one_fin(cube, f, /*layer_height=*/0.2, /*line_width=*/0.45); // 5 tabs

    f.conn_step = 100.0; // only the z=0 tab
    TriangleMesh one_tab = generate_one_fin(cube, f, /*layer_height=*/0.2, /*line_width=*/0.45);

    REQUIRE_FALSE(with_tabs.empty());
    REQUIRE_FALSE(one_tab.empty());
    CHECK(with_tabs.volume() > one_tab.volume());

    // Exact part counts pin the tab geometry down further than the volume
    // comparison alone: 5 rows (w=0,5,...,20) vs. 1 row (w=0), all within the
    // cube's full w in [0,40] cross-section, so every row gets a tab.
    CHECK(with_tabs.stats().number_of_parts == 1 + 5);
    CHECK(one_tab.stats().number_of_parts == 1 + 1);
    CHECK(with_tabs.stats().open_edges == 0);
    CHECK(one_tab.stats().open_edges == 0);
}

TEST_CASE("fin taller than the model: no tabs above the model top", "[fin]")
{
    indexed_triangle_set cube = its_make_cube(20, 20, 10);
    its_translate(cube, Vec3f(-10, -10, 0));

    FinFeature f;
    f.anchor_xy = Vec2d(0.0, 0.0);
    f.z_angle   = 0.0;
    // f.height = 30 asks for a fin taller than the 10mm-tall model. Design doc §15 REVOKES the
    // earlier "H above the model is allowed" rule (§8/§13.3): above the model's own top there is no
    // section, hence no shadow, hence no fin material -- the window's own w-range is capped at
    // min(H, model top) = min(30, 10) = 10 (generate_one_fin's `w_top`), so the fin's bbox must come
    // out exactly 10mm tall, not 30.
    f.height    = 30.0;
    // Window semantics (design doc §15): center anchor -> surface at local u=10, window's near
    // edge at u=11 -- length must exceed that (the pre-§15 value, 6, would leave the fin empty).
    f.length    = 16.0;
    f.conn_step = 5.0;
    f.conn_gap  = 1.0;

    // Rows above the model's own top (w=10) -- w=15,20,25,30 -- must be skipped
    // by contour_u_at() (nullopt: no section contour up there), not crash.
    TriangleMesh m = generate_one_fin(cube, f, /*layer_height=*/0.2, /*line_width=*/0.45);
    REQUIRE_FALSE(m.empty());
    CHECK(m.stats().open_edges == 0);

    // Capped at the model's own top (10), not the requested height (30) -- see the comment above.
    CHECK_THAT(m.bounding_box().size().z(), WithinAbs(10.0, 0.5));

    // Rows w=0,5,10 are within the model's own height (10) -> 3 tabs.
    CHECK(m.stats().number_of_parts == 1 + 3);
}

// --- FinGenerator: gizmo window + part shadow (design doc §15) -------------

TEST_CASE("fin: tilted slab -- shadow follows the surface, never the near side or above the part", "[fin]")
{
    // A tall, thin slab tilted about the X axis so its (u,w) cross-section at the fin's plane is
    // a diagonal band, not an axis-aligned rectangle -- the shape that exposed the old bbox-
    // rectangle bug (design doc §15): with the old model the fin could be built behind a tilted
    // part, far from its surface, or above its actual top.
    //
    // Built via its_make_cube() + its_transform() with a REAL rotation (not a hand-rolled sheared
    // mesh), so winding/orientation come for free from its_make_cube()'s own correctness. Expected
    // geometry is derived programmatically below from that SAME rotation applied to the box's own
    // corners, rather than hand-computed trig, so the test can't silently drift from what the mesh
    // actually contains.
    const double hw = 2.0, hh = 20.0, hd = 5.0; // half-width/height/depth before rotation
    indexed_triangle_set slab = its_make_cube(2 * hd, 2 * hw, 2 * hh);
    its_translate(slab, Vec3f(float(-hd), float(-hw), float(-hh))); // center on the origin

    const double phi = 15.0 * PI / 180.0; // tilt angle
    Transform3d  rot(Eigen::AngleAxisd(-phi, Vec3d::UnitX()));
    its_transform(slab, rot);

    // The four Y,Z corners of the (pre-shift) cross-section, rotated the same way as the mesh --
    // X is irrelevant to a rotation about the X axis, so 0 is used as a placeholder.
    auto  corner = [&](double y, double z) { return rot * Vec3d(0.0, y, z); };
    Vec3d A = corner(-hw, -hh), B = corner(hw, -hh), C = corner(hw, hh), D = corner(-hw, hh);

    // Shift the whole slab up so its lowest point sits exactly on the bed (w=0), matching the
    // "object sits on the bed" contract generate_one_fin() relies on (contour_u_at()'s doc comment).
    double z_shift = -std::min({A.z(), B.z(), C.z(), D.z()});
    its_translate(slab, Vec3f(0.f, 0.f, float(z_shift)));
    for (Vec3d* p : {&A, &B, &C, &D})
        p->z() += z_shift;

    // B and C share the original "+Y" (right) side of the box, so edge B-C is the RIGHT boundary
    // (max u at a given w) of the tilted cross-section -- the "surface" the shadow/tabs measure
    // against -- from the bed up to C; edge C-D takes over from there up to the topmost corner D.
    // (A and the A-B/D-A edges bound the LEFT side and never matter here.)
    REQUIRE(B.z() < C.z());
    REQUIRE(C.z() < D.z());
    REQUIRE_THAT(B.z(), WithinAbs(0.0, 1e-6)); // B is the lowest corner -> exactly the bed after the shift above

    auto lerp_u = [](const Vec3d& p0, const Vec3d& p1, double w) {
        double t = (w - p0.z()) / (p1.z() - p0.z());
        return p0.y() + t * (p1.y() - p0.y());
    };
    // Piecewise right-boundary of the tilted slab, clamped into the slab's own height range -- the
    // same boundary contour_u_at(section, w, max_side=true) sees inside generate_one_fin(). Used
    // for the per-vertex near-side check below, where every mesh vertex is, by construction of the
    // solid, guaranteed to sit at a height within [B.z(), D.z()] (checked separately, not via
    // clamping) -- clamping here only protects against sub-epsilon float noise at the boundary.
    auto surface_u_at = [&](double w) {
        double wc = std::clamp(w, B.z(), D.z());
        return wc <= C.z() ? lerp_u(B, C, wc) : lerp_u(C, D, wc);
    };
    // Strict version for the tab-row loop below, where "no material at this height" (nullopt) must
    // stay distinguishable from "material, and close enough to reach with a tab".
    auto surface_u_strict = [&](double w) -> std::optional<double> {
        if (w < B.z() || w > D.z())
            return std::nullopt;
        return surface_u_at(w);
    };

    FinFeature f;
    f.anchor_xy = Vec2d(0.0, 0.0);
    f.z_angle   = 0.0;
    f.height    = D.z() + 5.0; // taller than the slab's own top -> exercises the model-top cap too
    f.length    = 10.0;        // comfortably beyond any inner(w) reached below -- never the binding edge
    f.thickness = 1.2;
    f.conn_step = 2.0;
    f.conn_gap  = 1.0;

    TriangleMesh m = generate_one_fin(slab, f, /*layer_height=*/0.2, /*line_width=*/0.45);
    REQUIRE_FALSE(m.empty());

    constexpr double kTabOverlap    = 0.1; // must match generate_one_fin()'s own constant
    constexpr double kTabReachSlack = 0.2; // must match generate_one_fin()'s own constant
    constexpr double kSlack         = 0.3; // generous epsilon: float32 mesh vertices + kTabReachSlack

    // (a) no fin vertex on the near side, and (b) none above the slab's own top: every vertex's u
    // (== bed-frame Y, since z_angle=0) must be within [surface_u(w) - kTabOverlap, +inf) of the
    // surface at that height (w == bed-frame Z, must itself fall within [0, D.z()] -- f.height asks
    // for more, see above, so a vertex above D.z() would be a real capping-bug regression, not a
    // legitimate tall fin).
    for (const Vec3f& v : m.its.vertices) {
        double w = double(v.z());
        CHECK(w >= -kSlack);
        CHECK(w <= D.z() + kSlack);
        CHECK(double(v.y()) >= surface_u_at(w) - kTabOverlap - kSlack);
    }

    // (d) stands on the bed.
    CHECK_THAT(m.bounding_box().min.z(), WithinAbs(0.0, 0.5));

    // (c) tabs only where the surface is within conn_gap + kTabReachSlack of the fin's inner edge
    // -- computed the same way generate_one_fin() does, from surface_u_strict() above.
    int n_rows         = FinGeneratorInternal::fin_connector_rows(f);
    int expected_tabs  = 0;
    for (int k = 0; k < n_rows; ++k) {
        std::optional<double> surface = surface_u_strict(k * f.conn_step);
        if (!surface)
            continue; // above the slab's own top -- no material, no tab
        double inner = std::min(std::max(*surface + f.conn_gap, 0.0), f.length);
        double d     = inner - *surface;
        if (d <= f.conn_gap + kTabReachSlack)
            ++expected_tabs;
    }
    REQUIRE(expected_tabs > 0);      // the test's own premise: some rows are close enough for a tab
    REQUIRE(expected_tabs < n_rows); // ... and some aren't (near the bed, the tilt has carried the
                                      // surface too far from the window's near edge) -- the tilt is
                                      // what makes this test exercise something the axis-aligned
                                      // ones don't.
    CHECK(m.stats().number_of_parts == 1 + expected_tabs);
    CHECK(m.stats().open_edges == 0);
}

TEST_CASE("fin: window fully inside the part -> empty mesh", "[fin]")
{
    // Window semantics (design doc §15): with the anchor at the cube's own center and a length
    // that never reaches the section's far-side surface (10, half-width) plus the gap (1), the
    // window [0,length] never overlaps the shadow (u >= surface_u(w) + gap) at all -- the
    // "window entirely within the part+gap" empty-result contract (requirement 4).
    indexed_triangle_set cube = its_make_cube(20, 20, 20);
    its_translate(cube, Vec3f(-10.f, -10.f, 0.f));

    FinFeature f;
    f.anchor_xy = Vec2d(0.0, 0.0);
    f.z_angle   = 0.0;
    f.height    = 15.0;
    f.length    = 5.0; // < surface(10) + gap(1) = 11
    f.conn_gap  = 1.0;

    CHECK(generate_one_fin(cube, f, /*layer_height=*/0.2, /*line_width=*/0.45).empty());
}

TEST_CASE("fin: part beyond the near edge farther than gap at every row -> full-width fin, zero tabs", "[fin]")
{
    // Anchor placed so the section's surface sits at local u = -10 for every row (well beyond
    // -(gap+kTabReachSlack) = -1.2) -- "the part is further out than the gizmo allows" (§15's
    // tab rule) at EVERY row, but the shadow condition (u >= surface_u(w) + gap = -9) still covers
    // the entire window (u in [0,10]), since -9 < 0. Expect: the wall fills the window edge to
    // edge (no near-side carve, since offset_sec doesn't reach anywhere near u=0), but not a
    // single tab (d(w) = -surface_u(w) = 10 for every row, way past conn_gap + kTabReachSlack).
    indexed_triangle_set cube = its_make_cube(20, 20, 20);
    its_translate(cube, Vec3f(-10.f, -10.f, 0.f));

    FinFeature f;
    f.anchor_xy = Vec2d(0.0, 20.0); // local u = world Y - 20 -> surface (world Y=10) at local u=-10
    f.z_angle   = 0.0;
    f.height    = 15.0;
    f.length    = 10.0;
    f.thickness = 1.2;
    f.conn_step = 5.0;
    f.conn_gap  = 1.0;

    TriangleMesh m = generate_one_fin(cube, f, /*layer_height=*/0.2, /*line_width=*/0.45);
    REQUIRE_FALSE(m.empty());

    CHECK(m.stats().number_of_parts == 1); // the wall, and nothing else -- zero tabs
    CHECK(m.stats().open_edges == 0);

    // Full-width columns: fin2d is the entire window, untouched by the gap carve -> area
    // length * height = 10 * 15 = 150 mm^2, volume = area * thickness.
    CHECK_THAT(m.volume(), WithinAbs(150.0 * f.thickness, 0.5));
}

// --- FinGenerator: generate_fin_meshes, object -> bed conversion ------------
//
// Pins down the anchor/azimuth conversion under a non-identity `to_bed`
// (design §13.1-13.2): the fin is defined object-locally, but must come out
// in the bed frame -- rotated instances only move the anchor / turn the
// azimuth, and the fin itself must ALWAYS stand vertical on the bed (z from
// 0). Expectations below are derived from the formulas in
// generate_fin_meshes(): u_obj = (-sin a, cos a, 0), u_bed = R * u_obj
// (z zeroed), bed z_angle = atan2(-u_bed.x, u_bed.y).
//
// Shared numbers (fin: height 10, length 16, gap 1, thickness 1.2, cube half-width 10 along the
// relevant axis, anchor at the cube's own center -- window semantics, design doc §15): the
// section's far-side surface sits at local u=10 (from the center anchor), the window's near edge
// (past the gap) at u=11, and length=16 keeps the window open past that (length=6, the pre-§15
// value, would land the whole window inside the part+gap -> empty, breaking every test below).
// fin body spans [11, 16] along +u beyond the section (u=16 is exactly the window's own right
// edge now, rather than "surface + length" under the old bbox model -- see generate_one_fin()),
// tabs bridge the gap starting at face-0.1 (kTabOverlap), so the mesh spans ~[9.9, 16] along the
// fin direction, [-0.6, 0.6] across (thickness), and [0, 10] in z. These numbers are unchanged
// from before the §15 rework (only `length` moved, from 6 to 16, to compensate for the window now
// being measured from the anchor instead of from the section's own edge).

namespace {

FinFeature conversion_test_fin()
{
    FinFeature f;
    f.anchor_xy = Vec2d(0.0, 0.0);
    f.z_angle   = 0.0;
    f.height    = 10.0;
    f.length    = 16.0;
    f.thickness = 1.2;
    f.conn_step = 4.0;
    f.conn_gap  = 1.0;
    return f;
}

} // namespace

TEST_CASE("generate_fin_meshes: instance Z rotation turns the fin azimuth", "[fin]")
{
    Model        model;
    ModelObject* o = model.add_object();
    TriangleMesh cube = make_cube(20.0, 20.0, 20.0);
    cube.translate(-10.f, -10.f, 0.f); // x,y in [-10,10], z in [0,20], base on bed
    o->add_volume(std::move(cube), /*modify_to_center_geometry=*/false);

    FinFeature f = conversion_test_fin();
    o->fins.push_back(f);

    SECTION("identity to_bed: fin extends along +Y (object-local == bed)")
    {
        std::vector<TriangleMesh> fins =
            generate_fin_meshes(*o, Transform3d::Identity(), /*layer_height=*/0.2, /*line_width=*/0.45);
        REQUIRE(fins.size() == 1);

        BoundingBoxf3 bb = fins.front().bounding_box();
        CHECK_THAT(bb.max.y(), WithinAbs(16.0, 1e-3)); // +u == +Y at z_angle = 0
        CHECK_THAT(bb.min.y(), WithinAbs(9.9, 1e-3));  // tabs reach kTabOverlap into the cube
        CHECK_THAT(bb.size().x(), WithinAbs(1.2, 1e-3)); // thickness across the plane
        CHECK_THAT(bb.min.z(), WithinAbs(0.0, 1e-3));
        CHECK_THAT(bb.max.z(), WithinAbs(10.0, 1e-3));
    }

    SECTION("to_bed = Rz(90deg): same fin comes out along bed -X")
    {
        // u_obj = (0,1,0) -> u_bed = Rz(90) * u_obj = (-1,0,0): the fin's outward
        // direction lands on bed -X, so the whole fin sits beyond the cube's -X face.
        Transform3d to_bed(Eigen::AngleAxisd(0.5 * PI, Vec3d::UnitZ()));
        std::vector<TriangleMesh> fins =
            generate_fin_meshes(*o, to_bed, /*layer_height=*/0.2, /*line_width=*/0.45);
        REQUIRE(fins.size() == 1);

        BoundingBoxf3 bb = fins.front().bounding_box();
        CHECK_THAT(bb.min.x(), WithinAbs(-16.0, 1e-3)); // +u == -X now
        CHECK_THAT(bb.max.x(), WithinAbs(-9.9, 1e-3));
        CHECK_THAT(bb.size().y(), WithinAbs(1.2, 1e-3)); // thickness across the plane
        CHECK_THAT(bb.min.z(), WithinAbs(0.0, 1e-3));    // still standing on the bed
        CHECK_THAT(bb.max.z(), WithinAbs(10.0, 1e-3));   // still exactly `height` tall
    }
}

TEST_CASE("generate_fin_meshes: instance tipped 90deg about X keeps the fin vertical", "[fin]")
{
    // Object-local cube fully centered on the origin (x,y,z in [-10,10]) so that
    // to_bed = Tz(10) * Rx(90) puts it back onto the bed (x,y in [-10,10],
    // z in [0,20]) AND the object's z=0 plane (which the fin anchor lives on)
    // maps to bed plane y=0 -- through the middle of the tipped cube, where the
    // fin plane can actually cut material.
    Model        model;
    ModelObject* o = model.add_object();
    TriangleMesh cube = make_cube(20.0, 20.0, 20.0);
    cube.translate(-10.f, -10.f, -10.f);
    o->add_volume(std::move(cube), /*modify_to_center_geometry=*/false);

    Transform3d to_bed = Transform3d::Identity();
    to_bed.translate(Vec3d(0.0, 0.0, 10.0));
    to_bed.rotate(Eigen::AngleAxisd(0.5 * PI, Vec3d::UnitX()));

    SECTION("fin whose u-axis stays horizontal is generated, vertical on the bed")
    {
        // z_angle = PI/2 -> u_obj = (-1,0,0); Rx(90) leaves the X axis in place, so
        // u_bed = (-1,0,0): fin along bed -X. Design §13.2's key promise: despite
        // the tipped instance the fin stands VERTICAL in the bed frame -- z from 0
        // (the bed) to exactly `height`, not tipped along with the object.
        FinFeature f = conversion_test_fin();
        f.z_angle    = 0.5 * PI;
        o->fins.push_back(f);

        std::vector<TriangleMesh> fins =
            generate_fin_meshes(*o, to_bed, /*layer_height=*/0.2, /*line_width=*/0.45);
        REQUIRE(fins.size() == 1);

        BoundingBoxf3 bb = fins.front().bounding_box();
        CHECK_THAT(bb.min.z(), WithinAbs(0.0, 1e-3));    // starts at the bed
        CHECK_THAT(bb.max.z(), WithinAbs(10.0, 1e-3));   // exactly `height` tall
        CHECK_THAT(bb.min.x(), WithinAbs(-16.0, 1e-3));  // beyond the cube's -X face
        CHECK_THAT(bb.max.x(), WithinAbs(-9.9, 1e-3));
        CHECK_THAT(bb.size().y(), WithinAbs(1.2, 1e-3)); // thickness across the plane
    }

    SECTION("fin whose u-axis becomes vertical is skipped as degenerate")
    {
        // z_angle = 0 -> u_obj = (0,1,0); Rx(90) maps it to (0,0,1) -- projecting
        // onto the bed plane leaves a zero vector, so no bed azimuth exists and
        // generate_fin_meshes() must skip the fin instead of emitting garbage.
        FinFeature f = conversion_test_fin();
        o->fins.push_back(f);

        std::vector<TriangleMesh> fins =
            generate_fin_meshes(*o, to_bed, /*layer_height=*/0.2, /*line_width=*/0.45);
        CHECK(fins.empty());
    }
}

// --- FinGenerator: visual checkpoint (T00-01-06) ----------------------------
//
// Not a real regression test: dumps binary STLs of the input cube and the
// generated fin to disk so a human can eyeball the result in a mesh viewer.
// Tagged "[.]" -- Catch2's hidden-test convention (any tag starting with '.'
// sets TestCaseProperties::IsHidden; catch_test_case_registry_impl.cpp only
// runs hidden cases when a filter explicitly names them). That means it is
// excluded from a bare `libslic3r_tests` run and from `ctest` (which invokes
// the binary with no filters), but runs via `libslic3r_tests "[fin-visual]"`.
//
// Set FIN_VISUAL_DIR to choose the output directory; otherwise it defaults to
// the system temp dir.
TEST_CASE("fin: visual STL dump for manual inspection", "[fin-visual][.]")
{
    indexed_triangle_set cube = its_make_cube(20, 20, 40); // 0..20 x 0..20 x 0..40
    its_translate(cube, Vec3f(-10.f, -10.f, 0.f));         // center XY on origin, base on bed

    FinFeature f;
    f.anchor_xy = Vec2d(0.0, 0.0);
    f.z_angle   = 0.0;
    // height=25 (taller than the 10mm default) so the fin reads clearly
    // against the 40mm-tall cube; length/thickness/conn_step/conn_gap stay
    // at their defaults (8, 1.2, 4, 1).
    f.height = 25.0;

    double layer_height = 0.2, line_width = 0.45;
    TriangleMesh fin = generate_one_fin(cube, f, layer_height, line_width);
    REQUIRE_FALSE(fin.empty());

    TriangleMesh cube_mesh(cube);
    REQUIRE_FALSE(cube_mesh.empty());

    std::filesystem::path out_dir;
    if (const char* env = std::getenv("FIN_VISUAL_DIR"); env != nullptr && *env != '\0')
        out_dir = env;
    else
        out_dir = std::filesystem::temp_directory_path();
    std::filesystem::create_directories(out_dir);

    std::filesystem::path cube_path = out_dir / "cube.stl";
    std::filesystem::path fin_path  = out_dir / "fin.stl";

    REQUIRE(cube_mesh.write_binary(cube_path.string().c_str()));
    REQUIRE(fin.write_binary(fin_path.string().c_str()));

    WARN("fin-visual: cube -> " << cube_path.string());
    WARN("fin-visual: fin  -> " << fin_path.string());

    CHECK(std::filesystem::exists(cube_path));
    CHECK(std::filesystem::exists(fin_path));
    CHECK(std::filesystem::file_size(cube_path) > 0);
    CHECK(std::filesystem::file_size(fin_path) > 0);
}
