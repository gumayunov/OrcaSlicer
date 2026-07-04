#include <catch2/catch_all.hpp>

#include "libslic3r/Model.hpp"
#include "libslic3r/Fin/FinGenerator.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include "libslic3r/Utils.hpp"
#include "libslic3r/miniz_extension.hpp"

#include <cereal/archives/binary.hpp>

#include <boost/filesystem/operations.hpp>

#include <miniz.h>

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
    // Non-default, non-zero on every component (incl. anchor/dir's Z) so a component dropped by
    // serialize() can't hide behind a default-initialized match.
    src.anchor    = Vec3d(1.5, -2.5, 3.5);
    src.dir       = Vec3d(0.6, -0.8, 0.25);
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

    CHECK_THAT(dst.anchor.x(), WithinAbs(1.5, 1e-9));
    CHECK_THAT(dst.anchor.y(), WithinAbs(-2.5, 1e-9));
    CHECK_THAT(dst.anchor.z(), WithinAbs(3.5, 1e-9));
    CHECK_THAT(dst.dir.x(), WithinAbs(0.6, 1e-9));
    CHECK_THAT(dst.dir.y(), WithinAbs(-0.8, 1e-9));
    CHECK_THAT(dst.dir.z(), WithinAbs(0.25, 1e-9));
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

// Saves `m` to a real 3mf, replaces its Metadata/fin_information.xml with `xml` verbatim
// (copying every other zip entry through untouched), loads the result into `dst` through the
// full load_bbs_3mf() path, and removes both temp archives. Returns load_bbs_3mf()'s success
// flag. This is the minimal way to feed hand-crafted fin XML (corrupt, legacy-format,
// non-finite values) through the REAL load path -- hand-building a whole project archive
// would just re-test store_bbs_3mf() instead of the tampered-input case.
bool load_3mf_with_fin_xml(Model& m, Model& dst, const std::string& stem, const std::string& xml)
{
    if (temporary_dir().empty())
        set_temporary_dir(std::filesystem::temp_directory_path().string());

    const std::string good_path = std::string(TEST_DATA_DIR) + "/test_3mf/" + stem + "_source.3mf";
    const std::string test_path = std::string(TEST_DATA_DIR) + "/test_3mf/" + stem + ".3mf";

    DynamicPrintConfig store_config;
    StoreParams        store_params;
    store_params.path     = good_path.c_str();
    store_params.model    = &m;
    store_params.config   = &store_config;
    store_params.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;
    REQUIRE(store_bbs_3mf(store_params));

    {
        mz_zip_archive reader{};
        REQUIRE(open_zip_reader(&reader, good_path));
        mz_zip_archive writer{};
        REQUIRE(open_zip_writer(&writer, test_path));

        const mz_uint n_entries = mz_zip_reader_get_num_files(&reader);
        for (mz_uint i = 0; i < n_entries; ++i) {
            mz_zip_archive_file_stat stat;
            REQUIRE(mz_zip_reader_file_stat(&reader, i, &stat));
            if (std::string(stat.m_filename) == "Metadata/fin_information.xml") {
                REQUIRE(mz_zip_writer_add_mem(&writer, stat.m_filename, xml.data(), xml.size(),
                                               MZ_DEFAULT_COMPRESSION));
            } else {
                REQUIRE(mz_zip_writer_add_from_zip_reader(&writer, &reader, i));
            }
        }
        REQUIRE(mz_zip_writer_finalize_archive(&writer));
        REQUIRE(close_zip_writer(&writer));
        REQUIRE(close_zip_reader(&reader));
    }
    boost::filesystem::remove(good_path);

    DynamicPrintConfig        load_config;
    ConfigSubstitutionContext ctxt{ForwardCompatibilitySubstitutionRule::Disable};
    PlateDataPtrs             plate_data_list;
    std::vector<Preset*>      project_presets;
    bool                      is_bbl_3mf = false;
    Semver                    file_version;
    bool load_ok = load_bbs_3mf(test_path.c_str(), &load_config, &ctxt, &dst, &plate_data_list, &project_presets,
                                 &is_bbl_3mf, nullptr, &file_version, nullptr,
                                 LoadStrategy::LoadModel | LoadStrategy::LoadConfig);
    release_PlateData_list(plate_data_list);
    boost::filesystem::remove(test_path);
    return load_ok;
}

// One-object model (10mm cube, one instance, one default fin) -- the shared fixture for the
// tampered-fin-XML tests above/below (the stored fin only exists so store_bbs_3mf() emits a
// fin_information.xml entry for load_3mf_with_fin_xml() to replace).
Model model_with_default_fin()
{
    Model        m;
    ModelObject* o = m.add_object();
    o->add_volume(TriangleMesh(its_make_cube(10, 10, 10)));
    o->add_instance();
    o->fins.push_back(FinFeature{});
    return m;
}

} // namespace

TEST_CASE("fin: survives 3mf save/load roundtrip", "[fin]")
{
    Model        m;
    ModelObject* o = m.add_object();
    o->add_volume(TriangleMesh(its_make_cube(10, 10, 10)));
    o->add_instance();

    // Non-default values on every field, including anchor/dir's Z components and the bool, so
    // the roundtrip actually exercises all of FinFeature's persisted state. dirs are unit-norm
    // (0.6^2+0.64^2+0.48^2 == 1) because the importer re-normalizes at the trust boundary --
    // a non-unit dir would come back changed by design, not by a roundtrip bug.
    FinFeature f1;
    f1.anchor    = Vec3d(3.25, -1.75, 2.5);
    f1.dir       = Vec3d(0.6, 0.64, 0.48);
    f1.height    = 17.5;
    f1.length    = 9.25;
    f1.thickness = 1.4;
    f1.conn_step = 5.5;
    f1.conn_gap  = 0.9;
    f1.enabled   = true;
    o->fins.push_back(f1);

    FinFeature f2;
    f2.anchor    = Vec3d(-4.5, 6.125, -3.0);
    f2.dir       = Vec3d(-0.8, 0.48, 0.36);
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
    CHECK_THAT(r1.anchor.x(),    WithinAbs(f1.anchor.x(), 1e-6));
    CHECK_THAT(r1.anchor.y(),    WithinAbs(f1.anchor.y(), 1e-6));
    CHECK_THAT(r1.anchor.z(),    WithinAbs(f1.anchor.z(), 1e-6));
    CHECK_THAT(r1.dir.x(),       WithinAbs(f1.dir.x(), 1e-6));
    CHECK_THAT(r1.dir.y(),       WithinAbs(f1.dir.y(), 1e-6));
    CHECK_THAT(r1.dir.z(),       WithinAbs(f1.dir.z(), 1e-6));
    CHECK_THAT(r1.height,        WithinAbs(f1.height, 1e-6));
    CHECK_THAT(r1.length,        WithinAbs(f1.length, 1e-6));
    CHECK_THAT(r1.thickness,     WithinAbs(f1.thickness, 1e-6));
    CHECK_THAT(r1.conn_step,     WithinAbs(f1.conn_step, 1e-6));
    CHECK_THAT(r1.conn_gap,      WithinAbs(f1.conn_gap, 1e-6));
    CHECK(r1.enabled == f1.enabled);

    const FinFeature& r2 = dst.objects.front()->fins[1];
    CHECK_THAT(r2.anchor.x(),    WithinAbs(f2.anchor.x(), 1e-6));
    CHECK_THAT(r2.anchor.y(),    WithinAbs(f2.anchor.y(), 1e-6));
    CHECK_THAT(r2.anchor.z(),    WithinAbs(f2.anchor.z(), 1e-6));
    CHECK_THAT(r2.dir.x(),       WithinAbs(f2.dir.x(), 1e-6));
    CHECK_THAT(r2.dir.y(),       WithinAbs(f2.dir.y(), 1e-6));
    CHECK_THAT(r2.dir.z(),       WithinAbs(f2.dir.z(), 1e-6));
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

TEST_CASE("fin: corrupt fin_information.xml in a 3mf does not abort project load", "[fin]")
{
    // Cross-cutting robustness note (T00-03-01 review, docs/designs/00-vertical-fin/progress.md):
    // bbs_3mf.cpp has 4 unguarded pt::read_xml() call sites (the cut/brim/config-info readers,
    // plus ours) -- ALL of them throw an uncaught xml_parser_error on truncated/corrupt XML,
    // which aborts the WHOLE project load, not just that one piece of metadata. This feature's
    // fix is scoped to only our own extractor (_extract_fin_information_from_archive): a
    // try/catch around its read_xml() logs a warning and skips fins gracefully, leaving the
    // other three sites' systemic risk for upstream (fixing those would change cut/brim
    // behavior, out of this feature's scope). This test pins the graceful-skip behavior
    // end-to-end through the real load_bbs_3mf() entry point.
    Model m = model_with_default_fin();
    Model dst;
    // Truncated/unterminated XML replaces fin_information.xml inside a real archive.
    bool load_ok = load_3mf_with_fin_xml(m, dst, "fin_corrupt",
                                          "<?xml version=\"1.0\"?><fins><object id=\"1\"><fin ");

    REQUIRE(load_ok); // must not throw / abort the whole load over one corrupt metadata file
    REQUIRE(dst.objects.size() == 1);
    CHECK(dst.objects.front()->fins.empty()); // fins skipped gracefully; rest of the project intact
}

TEST_CASE("fin: legacy (design 00) fin_information.xml (anchor_x/anchor_y/z_angle, no dir_x) migrates on load",
          "[fin]")
{
    // Design 02 §3/Goal 4: a .3mf saved by a pre-design-02 build only ever wrote
    // anchor_x/anchor_y/z_angle (2D object-local anchor + azimuth about the object Z axis), never
    // dir_x/dir_y/dir_z. _extract_fin_information_from_archive() must detect the absence of
    // dir_x and migrate on the fly: anchor = (anchor_x, anchor_y, 0), dir = dir_from_z_angle(z_angle)
    // -- built the same way the corrupt-xml test above reaches load_bbs_3mf() (hand-replacing just
    // fin_information.xml inside a real archive), so this exercises the real load path end to end.
    const double legacy_anchor_x = 5.5, legacy_anchor_y = -2.25, legacy_z_angle = 0.9;

    std::ostringstream legacy_xml; // hand-written legacy-format document (no dir_x/dir_y/dir_z at all)
    legacy_xml << "<?xml version=\"1.0\"?><fins><object id=\"1\"><fin "
               << "anchor_x=\"" << legacy_anchor_x << "\" "
               << "anchor_y=\"" << legacy_anchor_y << "\" "
               << "z_angle=\"" << legacy_z_angle << "\" "
               << "height=\"10\" length=\"8\" thickness=\"1.2\" conn_step=\"4\" conn_gap=\"1\" "
               << "enabled=\"1\"/></object></fins>";

    Model m = model_with_default_fin();
    Model dst;
    REQUIRE(load_3mf_with_fin_xml(m, dst, "fin_legacy", legacy_xml.str()));
    REQUIRE(dst.objects.size() == 1);
    REQUIRE(dst.objects.front()->fins.size() == 1);

    const FinFeature& f = dst.objects.front()->fins.front();
    CHECK_THAT(f.anchor.x(), WithinAbs(legacy_anchor_x, 1e-9));
    CHECK_THAT(f.anchor.y(), WithinAbs(legacy_anchor_y, 1e-9));
    CHECK_THAT(f.anchor.z(), WithinAbs(0.0, 1e-9));
    const Vec3d expected_dir = FinGeneratorInternal::dir_from_z_angle(legacy_z_angle);
    CHECK_THAT(f.dir.x(), WithinAbs(expected_dir.x(), 1e-9));
    CHECK_THAT(f.dir.y(), WithinAbs(expected_dir.y(), 1e-9));
    CHECK_THAT(f.dir.z(), WithinAbs(expected_dir.z(), 1e-9));
}

TEST_CASE("fin: degenerate (zero) fin direction in fin_information.xml is skipped at load", "[fin]")
{
    // Trust boundary (design 02 review): everything downstream assumes finite fields and a
    // normalized dir; the importer must drop a violating fin (with a warning), not load it.
    // A zero dir is the trigger that behaves identically on every platform: "nan"/"inf" also
    // hit the same guard where iostreams accept them (libstdc++), but on libc++/MSVC they fail
    // ptree's parse outright and never reach the sanitizer, so a NaN fixture would pin
    // platform-dependent behavior instead of the guard.
    Model m = model_with_default_fin();
    Model dst;
    REQUIRE(load_3mf_with_fin_xml(m, dst, "fin_zero_dir",
                                   "<?xml version=\"1.0\"?><fins><object id=\"1\"><fin "
                                   "anchor_x=\"1\" anchor_y=\"2\" anchor_z=\"0\" "
                                   "dir_x=\"0\" dir_y=\"0\" dir_z=\"0\" "
                                   "height=\"10\" length=\"8\" thickness=\"1.2\" "
                                   "conn_step=\"4\" conn_gap=\"1\" enabled=\"1\"/></object></fins>"));
    REQUIRE(dst.objects.size() == 1);
    CHECK(dst.objects.front()->fins.empty()); // the degenerate fin is skipped; the project still loads
}

// --- FinGenerator: plane-frame <-> bed-frame -------------------------------

TEST_CASE("plane_to_bed maps the local origin to the anchor", "[fin]")
{
    FinFeature f;
    f.anchor = Vec3d(3.0, -4.0, 0.0);
    f.dir    = FinGeneratorInternal::dir_from_z_angle(0.37); // arbitrary, non-axis-aligned

    Transform3d plane_to_bed = FinGeneratorInternal::plane_to_bed(f);
    Vec3d       p            = plane_to_bed * Vec3d(0.0, 0.0, 0.0);

    CHECK_THAT(p.x(), WithinAbs(f.anchor.x(), 1e-9));
    CHECK_THAT(p.y(), WithinAbs(f.anchor.y(), 1e-9));
    CHECK_THAT(p.z(), WithinAbs(0.0, 1e-9));
}

TEST_CASE("plane_to_bed at z_angle=0 locks the coordinate contract", "[fin]")
{
    // Design contract (design doc §13): with z_angle = 0, u (local x, outward)
    // maps to world +Y, and n (local z, plane normal) maps to world +X.
    FinFeature f;
    f.anchor = Vec3d::Zero();
    f.dir    = FinGeneratorInternal::dir_from_z_angle(0.0);

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
    f.anchor = Vec3d::Zero();
    f.dir    = FinGeneratorInternal::dir_from_z_angle(PI / 2.0);

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
    f.anchor = Vec3d(1.5, 2.5, 0.0);
    f.dir    = FinGeneratorInternal::dir_from_z_angle(1.1);

    Transform3d roundtrip = FinGeneratorInternal::bed_to_plane(f) * FinGeneratorInternal::plane_to_bed(f);
    Vec3d       p         = roundtrip * Vec3d(7.0, -3.0, 2.0);

    CHECK_THAT(p.x(), WithinAbs(7.0, 1e-9));
    CHECK_THAT(p.y(), WithinAbs(-3.0, 1e-9));
    CHECK_THAT(p.z(), WithinAbs(2.0, 1e-9));
}

// --- FinGenerator: object <-> bed (world) conversion (T00-04-05) -----------
//
// fin_object_to_bed()/fin_bed_to_object() are the gizmo's coordinate contract (design doc §13,
// the T00-04-04 review note; design 02 replaced the 2D anchor_xy/z_angle pair with a full 3D
// anchor/dir so the conversion survives an instance stood on its side): the gizmo edits a fin in
// the BED (world) frame, but ModelObject::fins stores it OBJECT-LOCAL. fin_object_to_bed() is
// generate_fin_meshes()'s own per-fin conversion (object -> bed, used to load an existing fin
// into the gizmo); fin_bed_to_object() is its inverse (bed -> object, used to persist a gizmo
// edit). Both touch only anchor/dir -- the other fields are asserted here too, to catch a field
// getting dropped by accident, even though the conversion itself never touches them.

namespace {

// Arbitrary, mutually distinct values for every field (incl. enabled=false, the non-default
// bool) so a field-dropping bug can't hide behind a default-initialized match.
FinFeature roundtrip_test_fin()
{
    FinFeature f;
    f.anchor    = Vec3d(3.25, -6.75, 0.0);
    f.dir       = FinGeneratorInternal::dir_from_z_angle(0.6); // arbitrary, non-axis-aligned heading
    f.height    = 17.5;
    f.length    = 9.25;
    f.thickness = 0.85;
    f.conn_step = 3.5;
    f.conn_gap  = 1.35;
    f.enabled   = false;
    return f;
}

void check_fins_match(const FinFeature& a, const FinFeature& b, double eps = 1e-9)
{
    CHECK_THAT(a.anchor.x(), WithinAbs(b.anchor.x(), eps));
    CHECK_THAT(a.anchor.y(), WithinAbs(b.anchor.y(), eps));
    CHECK_THAT(a.anchor.z(), WithinAbs(b.anchor.z(), eps));
    CHECK_THAT(a.dir.x(), WithinAbs(b.dir.x(), eps));
    CHECK_THAT(a.dir.y(), WithinAbs(b.dir.y(), eps));
    CHECK_THAT(a.dir.z(), WithinAbs(b.dir.z(), eps));
    CHECK_THAT(a.height, WithinAbs(b.height, eps));
    CHECK_THAT(a.length, WithinAbs(b.length, eps));
    CHECK_THAT(a.thickness, WithinAbs(b.thickness, eps));
    CHECK_THAT(a.conn_step, WithinAbs(b.conn_step, eps));
    CHECK_THAT(a.conn_gap, WithinAbs(b.conn_gap, eps));
    CHECK(a.enabled == b.enabled);
}

} // namespace

TEST_CASE("fin_object_to_bed / fin_bed_to_object round-trip: identity to_bed", "[fin]")
{
    using namespace FinGeneratorInternal;

    FinFeature f_obj = roundtrip_test_fin();

    std::optional<FinFeature> f_bed = fin_object_to_bed(f_obj, Transform3d::Identity());
    REQUIRE(f_bed.has_value());

    std::optional<FinFeature> f_obj2 = fin_bed_to_object(*f_bed, Transform3d::Identity());
    REQUIRE(f_obj2.has_value());

    check_fins_match(f_obj, *f_obj2);
}

TEST_CASE("fin_bed_to_object / fin_object_to_bed round-trip: translated + Z-rotated instance", "[fin]")
{
    // Design 02 Goal 2: bed->object->bed is the identity direction (the gizmo commit/load
    // cycle). object->bed->object deliberately is NOT exact when to_bed lifts the anchor off
    // z=0 (here: translation.z=3) -- fin_object_to_bed() re-anchors the plane at bed z=0, so
    // the object-local representative point normalizes on the first pass (design 02 §2).
    using namespace FinGeneratorInternal;

    Transform3d to_bed = Transform3d::Identity();
    to_bed.translate(Vec3d(12.0, -7.0, 3.0));
    to_bed.rotate(Eigen::AngleAxisd(1.1, Vec3d::UnitZ()));

    FinFeature f_bed = roundtrip_test_fin(); // anchor.z == 0, dir horizontal: valid bed-frame form

    std::optional<FinFeature> f_obj = fin_bed_to_object(f_bed, to_bed);
    REQUIRE(f_obj.has_value());

    std::optional<FinFeature> f_bed2 = fin_object_to_bed(*f_obj, to_bed);
    REQUIRE(f_bed2.has_value());

    check_fins_match(f_bed, *f_bed2);
}

TEST_CASE("fin_bed_to_object / fin_object_to_bed round-trip: instance tipped 30deg about X", "[fin]")
{
    // A partial (non-90deg) tip about a horizontal in-plane axis: the object-local frame no
    // longer aligns with the bed, but to_bed stays invertible, so the bed->object->bed cycle
    // (the gizmo's commit/load direction, design 02 Goal 2) is still exact. The opposite
    // direction would NOT be: the tip gives the stored dir's bed image a vertical component,
    // which fin_object_to_bed() drops when normalizing (design 02 §2).
    using namespace FinGeneratorInternal;

    Transform3d to_bed(Eigen::AngleAxisd(PI / 6.0, Vec3d::UnitX())); // 30deg

    FinFeature f_bed = roundtrip_test_fin(); // anchor.z == 0, dir horizontal: valid bed-frame form

    std::optional<FinFeature> f_obj = fin_bed_to_object(f_bed, to_bed);
    REQUIRE(f_obj.has_value());

    std::optional<FinFeature> f_bed2 = fin_object_to_bed(*f_obj, to_bed);
    REQUIRE(f_bed2.has_value());

    check_fins_match(f_bed, *f_bed2);
}

TEST_CASE("fin_object_to_bed returns nullopt when the object-local u axis becomes vertical", "[fin]")
{
    // Instance tipped exactly 90deg about X: object-local +Y (dir=(0,1,0), the old z_angle=0's u
    // axis) maps entirely onto world Z, so no bed-frame direction exists (mirrors the "instance
    // tipped 90deg about X" generate_fin_meshes test, but exercised directly against the helper
    // it now calls). Design 02 Goal 3: this geometric degeneracy is the only one left -- unlike
    // the old to_bed-XY-block singularity the regression test below fixes.
    using namespace FinGeneratorInternal;

    Transform3d to_bed(Eigen::AngleAxisd(0.5 * PI, Vec3d::UnitX()));
    FinFeature  f = roundtrip_test_fin();
    f.dir         = Vec3d(0.0, 1.0, 0.0);

    CHECK_FALSE(fin_object_to_bed(f, to_bed).has_value());
}

TEST_CASE("fin: bed->object->bed roundtrip survives an instance stood on its side", "[fin]")
{
    // Design-02 bug: Rx/Ry(90deg) instances made the old 2x2 XY-block solve singular,
    // so every gizmo commit was silently dropped for objects stood on their end.
    using namespace FinGeneratorInternal;
    const Transform3d cases[] = {
        Transform3d(Eigen::AngleAxisd(0.5 * PI, Vec3d::UnitX())),
        Transform3d(Eigen::AngleAxisd(-0.5 * PI, Vec3d::UnitY())),
        Transform3d(Eigen::Translation3d(3.0, -4.0, 0.0) * Eigen::AngleAxisd(0.5 * PI, Vec3d::UnitX())),
    };
    for (const Transform3d& to_bed : cases) {
        FinFeature f_bed;                         // bed-frame: anchor.z == 0, dir horizontal
        f_bed.anchor = Vec3d(12.5, -7.0, 0.0);
        f_bed.dir    = dir_from_z_angle(0.6);
        const std::optional<FinFeature> f_obj = fin_bed_to_object(f_bed, to_bed);
        REQUIRE(f_obj.has_value());
        const std::optional<FinFeature> back = fin_object_to_bed(*f_obj, to_bed);
        REQUIRE(back.has_value());
        CHECK_THAT(back->anchor.x(), WithinAbs(f_bed.anchor.x(), 1e-9));
        CHECK_THAT(back->anchor.y(), WithinAbs(f_bed.anchor.y(), 1e-9));
        CHECK_THAT(back->anchor.z(), WithinAbs(0.0, 1e-9));
        CHECK_THAT(back->dir.x(), WithinAbs(f_bed.dir.x(), 1e-9));
        CHECK_THAT(back->dir.y(), WithinAbs(f_bed.dir.y(), 1e-9));
        CHECK_THAT(back->dir.z(), WithinAbs(0.0, 1e-9));
    }
}

TEST_CASE("fin_bed_to_object returns nullopt for a non-invertible instance transform", "[fin]")
{
    // Design 02 Goal 3's other honest degeneracy: to_bed's linear part itself is (near) singular
    // (e.g. a near-zero scale on one axis), independent of any particular FinFeature -- nothing
    // can be solved, unlike the old to_bed-XY-block singularity above (fixed by the regression
    // test above; this one is a genuinely non-invertible transform, not just an inconvenient
    // orientation).
    using namespace FinGeneratorInternal;

    Transform3d to_bed = Geometry::scale_transform(Vec3d(1e-9, 1.0, 1.0));
    FinFeature  f_bed  = roundtrip_test_fin();

    CHECK_FALSE(fin_bed_to_object(f_bed, to_bed).has_value());
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
    f.anchor    = Vec3d(0.0, 0.0, 0.0);
    f.dir       = FinGeneratorInternal::dir_from_z_angle(0.0);
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
    f.anchor    = Vec3d(0.0, 0.0, 0.0);
    f.dir       = FinGeneratorInternal::dir_from_z_angle(0.0);

    CHECK(generate_one_fin(cube, f, /*layer_height=*/0.2, /*line_width=*/0.45).empty());
}

// --- FinGenerator: 2D fin region (truncated rectangle) ----------------------

TEST_CASE("fin: 2D region is a truncated rectangle on one side", "[fin]")
{
    indexed_triangle_set cube = its_make_cube(20, 20, 30);
    its_translate(cube, Vec3f(-10.f, -10.f, 0.f));

    FinFeature f;
    f.anchor    = Vec3d(0.0, 0.0, 0.0);
    f.dir       = FinGeneratorInternal::dir_from_z_angle(0.0);
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
    g.dir    = FinGeneratorInternal::dir_from_z_angle(PI);
    BoundingBoxf3 bbg = generate_one_fin(cube, g, /*layer_height=*/0.2, /*line_width=*/0.45).bounding_box();
    CHECK(bbg.min.y() < -10.0);
}

TEST_CASE("fin: gap offset swallowing the whole rectangle yields empty mesh", "[fin]")
{
    indexed_triangle_set cube = its_make_cube(10, 10, 10);
    its_translate(cube, Vec3f(-5.f, -5.f, 0.f));

    FinFeature f;
    f.anchor    = Vec3d(0.0, 0.0, 0.0);
    f.dir       = FinGeneratorInternal::dir_from_z_angle(0.0);
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
    f.anchor     = Vec3d(0.0, 0.0, 0.0);
    f.dir        = FinGeneratorInternal::dir_from_z_angle(0.0);
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
    f.anchor    = Vec3d(0.0, 0.0, 0.0);
    f.dir       = FinGeneratorInternal::dir_from_z_angle(0.0);
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
    int n_tab_rows = FinGeneratorInternal::fin_connector_rows(f, layer_height);
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
    // Each tab spans d(w) + model_overlap + fin_overlap along u, d(w) = inner(w) - surface_u(w)
    // (generate_one_fin, design doc §15 + the T00-05-01 penetration-depth fix). model_overlap =
    // max(kTabOverlap, line_width) is the reach INTO the model (deep enough to clear its own
    // outer wall line, not just its surface line); fin_overlap = layer_height + kTabOverlap is
    // the reach INTO the fin body (deep enough to survive a sloped fin inner face receding over
    // one tab's own height -- see the tilted-slab test for the sloped case itself). Here
    // surface_u=10, conn_gap=1.0, inner=min(surface_u+conn_gap, length)=11 (length=16 doesn't
    // clamp it), so d=1.0, model_overlap=max(0.1, 0.45)=0.45, fin_overlap=0.2+0.1=0.3, and
    // tab_len = 1.0 + 0.45 + 0.3 = 1.75 (up from the pre-fix 1.0 + 2*0.1 = 1.2).
    double model_overlap   = std::max(0.1, line_width); // must match generate_one_fin()'s kTabOverlap/line_width rule
    double fin_overlap     = layer_height + 0.1;         // must match generate_one_fin()'s layer_height + kTabOverlap rule
    double tab_volume      = (f.conn_gap + model_overlap + fin_overlap) * layer_height * (2.0 * line_width);
    double expected_volume = wall_volume + n_tab_rows * tab_volume;
    CHECK_THAT(m.volume(), WithinAbs(expected_volume, 0.05));
}

// --- FinGenerator: connector tabs (T00-01-05) -------------------------------

TEST_CASE("fin: fin_connector_rows == floor(H/X) + 1", "[fin]")
{
    FinFeature f;
    f.height    = 20.0;
    f.conn_step = 5.0;
    CHECK(FinGeneratorInternal::fin_connector_rows(f, 0.2) == 5); // w = 0,5,10,15,20

    f.conn_step = 100.0;
    CHECK(FinGeneratorInternal::fin_connector_rows(f, 0.2) == 1); // only w = 0
}

TEST_CASE("fin: fin_connector_rows clamps row spacing to at least one layer", "[fin]")
{
    // Phase 05 edge case: conn_step (0.1) smaller than layer_height (0.2) must not literally
    // floor(H/conn_step)+1 = 201 rows -- effective_step = max(conn_step, layer_height) = 0.2
    // clamps it to what the object can actually resolve into distinct, non-overlapping rows.
    FinFeature f;
    f.height    = 20.0;
    f.conn_step = 0.1;
    CHECK(FinGeneratorInternal::fin_connector_rows(f, 0.2) == 101); // floor(20/0.2)+1, not floor(20/0.1)+1=201

    // conn_step already >= layer_height: unaffected, same as the un-clamped formula.
    f.conn_step = 5.0;
    CHECK(FinGeneratorInternal::fin_connector_rows(f, 0.2) == 5);
}

TEST_CASE("fin: connector count = floor(H/X)+1", "[fin]")
{
    indexed_triangle_set cube = its_make_cube(20, 20, 40);
    its_translate(cube, Vec3f(-10, -10, 0));

    FinFeature f;
    f.anchor    = Vec3d(0.0, 0.0, 0.0);
    f.dir       = FinGeneratorInternal::dir_from_z_angle(0.0);
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
    f.anchor    = Vec3d(0.0, 0.0, 0.0);
    f.dir       = FinGeneratorInternal::dir_from_z_angle(0.0);
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
    f.anchor    = Vec3d(0.0, 0.0, 0.0);
    f.dir       = FinGeneratorInternal::dir_from_z_angle(0.0);
    f.height    = D.z() + 5.0; // taller than the slab's own top -> exercises the model-top cap too
    f.length    = 10.0;        // comfortably beyond any inner(w) reached below -- never the binding edge
    f.thickness = 1.2;
    f.conn_step = 2.0;
    f.conn_gap  = 1.0;

    double layer_height = 0.2, line_width = 0.45;
    TriangleMesh m = generate_one_fin(slab, f, layer_height, line_width);
    REQUIRE_FALSE(m.empty());

    constexpr double kTabOverlap    = 0.1; // must match generate_one_fin()'s own floor constant
    constexpr double kTabReachSlack = 0.2; // must match generate_one_fin()'s own constant
    constexpr double kSlack         = 0.3; // generous epsilon: float32 mesh vertices + kTabReachSlack
    // Model-side tab reach (T00-05-01): max(kTabOverlap, line_width), must match generate_one_fin()'s
    // own model_overlap -- line_width (0.45) dominates kTabOverlap (0.1) here.
    double model_overlap = std::max(kTabOverlap, line_width);

    // (a) no fin vertex on the near side, and (b) none above the slab's own top: every vertex's u
    // (== bed-frame Y, since z_angle=0) must be within [surface_u(w) - model_overlap, +inf) of the
    // surface at that height (w == bed-frame Z, must itself fall within [0, D.z()] -- f.height asks
    // for more, see above, so a vertex above D.z() would be a real capping-bug regression, not a
    // legitimate tall fin).
    for (const Vec3f& v : m.its.vertices) {
        double w = double(v.z());
        CHECK(w >= -kSlack);
        CHECK(w <= D.z() + kSlack);
        CHECK(double(v.y()) >= surface_u_at(w) - model_overlap - kSlack);
    }

    // (d) stands on the bed.
    CHECK_THAT(m.bounding_box().min.z(), WithinAbs(0.0, 0.5));

    // (c) tabs only where the surface is within conn_gap + kTabReachSlack of the fin's inner edge
    // -- computed the same way generate_one_fin() does, from surface_u_strict() above. Row height
    // uses effective_step = max(conn_step, layer_height) (phase 05 clamp), same as conn_step here
    // since conn_step(2.0) > layer_height(0.2).
    double effective_step = std::max(f.conn_step, layer_height);
    int n_rows         = FinGeneratorInternal::fin_connector_rows(f, layer_height);
    int expected_tabs  = 0;
    for (int k = 0; k < n_rows; ++k) {
        std::optional<double> surface = surface_u_strict(k * effective_step);
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
    f.anchor    = Vec3d(0.0, 0.0, 0.0);
    f.dir       = FinGeneratorInternal::dir_from_z_angle(0.0);
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
    f.anchor    = Vec3d(0.0, 20.0, 0.0); // local u = world Y - 20 -> surface (world Y=10) at local u=-10
    f.dir       = FinGeneratorInternal::dir_from_z_angle(0.0);
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

// --- FinGenerator: tab penetration depths (T00-05-01) -----------------------

TEST_CASE("fin: tab penetration follows max(kTabOverlap, line_width) into the model and "
          "layer_height + kTabOverlap into the fin, clamped to the window's outer edge",
          "[fin]")
{
    // 2026-07-03 user report (progress.md): a flat kTabOverlap=0.1mm on both sides under-reached
    // -- into the model it landed inside the outer wall LINE (tabs sliced as overhang "in the
    // air"), and into the fin it could miss a sloped inner face entirely (see the tilted-slab
    // test for that case). This axis-aligned fixture isolates the two depths and the far-edge
    // clamp with simple, exact numbers.
    indexed_triangle_set cube = its_make_cube(20, 20, 20);
    its_translate(cube, Vec3f(-10.f, -10.f, 0.f)); // surface_u = 10 for every row

    FinFeature f;
    f.anchor    = Vec3d(0.0, 0.0, 0.0);
    f.dir       = FinGeneratorInternal::dir_from_z_angle(0.0);
    f.height    = 5.0;
    f.length    = 16.0;   // comfortably beyond inner(w) -- the wall body's own far edge, not the tab's
    f.conn_step = 100.0;  // only the w=0 row -- isolates exactly one tab
    f.conn_gap  = 1.0;

    SECTION("line_width bigger than the old flat kTabOverlap dominates the model-side reach")
    {
        double layer_height = 0.25, line_width = 0.6; // line_width > kTabOverlap(0.1): must dominate
        TriangleMesh m = generate_one_fin(cube, f, layer_height, line_width);
        REQUIRE_FALSE(m.empty());
        REQUIRE(m.stats().number_of_parts == 2); // wall + exactly one tab
        CHECK(m.stats().open_edges == 0);

        // surface_u=10, inner=min(surface_u+conn_gap,length)=11 (length=16 doesn't clamp).
        double model_overlap = std::max(0.1, line_width); // == line_width (0.6) here
        double fin_overlap   = layer_height + 0.1;         // == 0.35
        BoundingBoxf3 bb = m.bounding_box();
        // The tab is the ONLY geometry reaching below u=11 (the wall body itself spans
        // [11, 16]), so bb.min.y() pins the model-side depth unambiguously.
        CHECK_THAT(bb.min.y(), WithinAbs(10.0 - model_overlap, 1e-3));

        double wall_area   = 5.0 * f.height; // fin2d = u in [11,16] x w in [0,5]
        double wall_volume = wall_area * f.thickness;
        double tab_len     = (f.conn_gap) + model_overlap + fin_overlap; // d + model_overlap + fin_overlap
        double tab_volume  = tab_len * layer_height * (2.0 * line_width);
        CHECK_THAT(m.volume(), WithinAbs(wall_volume + tab_volume, 0.01));
    }

    SECTION("fin-side reach is clamped to the window's own outer edge (u = length) on a thin fin")
    {
        // length picked so inner(=11) + fin_overlap(=0.4) would overshoot length -- the tab's far
        // end must clamp to length exactly, never past material the window never generated.
        FinFeature g = f;
        g.length     = 11.3; // in (11, 11.4): thin wall (0.3mm), but still enough room for one to exist
        g.height     = 3.0;

        double layer_height = 0.3, line_width = 0.2;
        TriangleMesh m = generate_one_fin(cube, g, layer_height, line_width);
        REQUIRE_FALSE(m.empty());
        REQUIRE(m.stats().number_of_parts == 2);
        CHECK(m.stats().open_edges == 0);

        double model_overlap = std::max(0.1, line_width); // == 0.2
        BoundingBoxf3 bb = m.bounding_box();
        CHECK_THAT(bb.min.y(), WithinAbs(10.0 - model_overlap, 1e-3)); // tab's near end
        // Unclamped this would be inner(11) + fin_overlap(0.4) = 11.4, past the window -- the
        // clamp caps it at g.length(11.3), which is also exactly the (razor-thin) wall's own far
        // edge, so bb.max.y() pins the clamp directly.
        CHECK_THAT(bb.max.y(), WithinAbs(g.length, 1e-3));
    }
}

// --- FinGenerator: multiple disjoint cross-sections (phase 05 edge case) ---

TEST_CASE("fin: two disjoint pillars in the section -- shadow/tabs measure the outermost lobe", "[fin]")
{
    // Two separate pillars straddling the fin's cutting plane (world X=0 at z_angle=0, anchor
    // (0,0)): a near one (world Y in [3,5]) and a far one (world Y in [6.5,8.5]), both spanning
    // the same Z range -- slicing the merged solid at X=0 yields TWO disjoint rectangles in the
    // (u,w) section, the "multiple disjoint cross-sections" edge case (phase 05 plan). The 1.5mm
    // gap between them (5 to 6.5) is narrower than 2*conn_gap(1.0), so their conn_gap offsets
    // overlap and diff_ex() carves a single clean shadow region -- keeps this fixture's own
    // expected geometry simple while still exercising a genuinely 2-component section.
    //
    // Documents the known limitation already called out in contour_u_at()'s doc comment
    // (FinGenerator.cpp): the extreme-u scan returns the OUTERMOST lobe's surface for tab
    // placement (the far pillar's u=8.5), regardless of a nearer lobe existing at the same
    // height (the near pillar's u=5.0) -- this test pins that current behavior, it does not
    // change it (see design.md's own future-work note on multi-fin/multi-lobe support).
    indexed_triangle_set near_pillar = its_make_cube(2, 2, 10); // -> world Y in [3,5], Z in [0,10]
    its_translate(near_pillar, Vec3f(-1.f, 3.f, 0.f));
    indexed_triangle_set far_pillar = its_make_cube(2, 2, 10); // -> world Y in [6.5,8.5], Z in [0,10]
    its_translate(far_pillar, Vec3f(-1.f, 6.5f, 0.f));

    indexed_triangle_set solid = near_pillar;
    its_merge(solid, far_pillar);

    FinFeature f;
    f.anchor    = Vec3d(0.0, 0.0, 0.0);
    f.dir       = FinGeneratorInternal::dir_from_z_angle(0.0);
    f.height    = 10.0;
    f.length    = 16.0; // clears the far lobe's surface (8.5) + gap (1.0) = 9.5, with room to spare
    f.conn_step = 5.0;
    f.conn_gap  = 1.0;

    double layer_height = 0.2, line_width = 0.45;
    TriangleMesh m = generate_one_fin(solid, f, layer_height, line_width);
    REQUIRE_FALSE(m.empty());
    CHECK(m.stats().open_edges == 0);
    CHECK_THAT(m.bounding_box().size().z(), WithinAbs(10.0, 0.5));

    // Rows w=0,5,10 (fin_connector_rows(f, layer_height) == 3) all fall within both pillars' own
    // height (0..10), so all three get a tab measured against the FAR lobe's surface (8.5), not
    // the near one's (5.0).
    REQUIRE(FinGeneratorInternal::fin_connector_rows(f, layer_height) == 3);
    CHECK(m.stats().number_of_parts == 1 + 3); // wall + 3 tabs

    // The far-lobe surface (8.5) is what the tab's model-side reach is measured from; had the
    // near lobe (5.0) been (incorrectly) used instead, this would come out around 4.55, not 8.05.
    double model_overlap = std::max(0.1, line_width);
    CHECK_THAT(m.bounding_box().min.y(), WithinAbs(8.5 - model_overlap, 1e-3));
}

// --- FinGenerator: very small conn_step vs. layer_height (phase 05 edge case) ---

TEST_CASE("fin: conn_step smaller than layer_height does not produce overlapping/duplicate tab rows", "[fin]")
{
    // Before the fin_connector_rows()/generate_one_fin() clamp (T00-05-01), conn_step=0.1 with a
    // 0.2mm layer_height would have asked for floor(2.0/0.1)+1 = 21 rows spaced 0.1mm apart -- each
    // tab is itself `layer_height` (0.2mm) tall, so consecutive rows would emit vertically
    // overlapping/duplicate tab slabs. effective_step = max(conn_step, layer_height) = 0.2 clamps
    // that to the object's actual layer resolution: 11 rows (w = 0, 0.2, ..., 2.0), each tab
    // touching but not overlapping its neighbor (row k's top == row k+1's bottom).
    indexed_triangle_set cube = its_make_cube(20, 20, 20);
    its_translate(cube, Vec3f(-10.f, -10.f, 0.f)); // surface_u = 10 for every row, full 20mm height

    FinFeature f;
    f.anchor    = Vec3d(0.0, 0.0, 0.0);
    f.dir       = FinGeneratorInternal::dir_from_z_angle(0.0);
    f.height    = 2.0;
    f.length    = 16.0;
    f.conn_step = 0.1; // smaller than layer_height below
    f.conn_gap  = 1.0;

    double layer_height = 0.2, line_width = 0.45;
    REQUIRE(FinGeneratorInternal::fin_connector_rows(f, layer_height) == 11); // floor(2/0.2)+1, not floor(2/0.1)+1=21

    TriangleMesh m = generate_one_fin(cube, f, layer_height, line_width);
    REQUIRE_FALSE(m.empty());
    CHECK(m.stats().open_edges == 0);
    // 1 wall + 11 tabs (every row has material within the cube's own 20mm height and reaches the
    // gap within kTabReachSlack, same as the axis-aligned tests above) -- NOT 1+21.
    CHECK(m.stats().number_of_parts == 1 + 11);
}

// --- FinGenerator: negative/zero parameters (T00-01-03 review note) --------

TEST_CASE("generate_one_fin: non-positive length/height/thickness/conn_step or negative conn_gap "
          "yields an empty mesh",
          "[fin]")
{
    // Defensive generator-side guard: the GUI already clamps these to sane minimums
    // (GLGizmoFin.cpp's MinFinDimension/MinFinParamValue), but .3mf/API callers can feed a
    // FinFeature with arbitrary values -- e.g. a hand-edited or corrupted project file. Negative
    // length/height in particular would otherwise flip the 2D rect's winding without crashing
    // Clipper, producing wrong-but-non-empty geometry rather than the "no fin" contract this
    // guard now enforces (T00-01-03 review note, docs/designs/00-vertical-fin/progress.md).
    indexed_triangle_set cube = its_make_cube(20, 20, 20);
    its_translate(cube, Vec3f(-10.f, -10.f, 0.f));

    FinFeature base;
    base.anchor    = Vec3d(0.0, 0.0, 0.0);
    base.dir       = FinGeneratorInternal::dir_from_z_angle(0.0);
    base.height    = 15.0;
    base.length    = 16.0;
    // thickness/conn_step/conn_gap stay at FinFeature{}'s own (positive) defaults.

    SECTION("length == 0") {
        FinFeature f = base; f.length = 0.0;
        CHECK(generate_one_fin(cube, f, 0.2, 0.45).empty());
    }
    SECTION("length < 0") {
        FinFeature f = base; f.length = -5.0;
        CHECK(generate_one_fin(cube, f, 0.2, 0.45).empty());
    }
    SECTION("height == 0") {
        FinFeature f = base; f.height = 0.0;
        CHECK(generate_one_fin(cube, f, 0.2, 0.45).empty());
    }
    SECTION("height < 0") {
        FinFeature f = base; f.height = -10.0;
        CHECK(generate_one_fin(cube, f, 0.2, 0.45).empty());
    }
    SECTION("thickness == 0") {
        FinFeature f = base; f.thickness = 0.0;
        CHECK(generate_one_fin(cube, f, 0.2, 0.45).empty());
    }
    SECTION("thickness < 0") {
        FinFeature f = base; f.thickness = -1.2;
        CHECK(generate_one_fin(cube, f, 0.2, 0.45).empty());
    }
    SECTION("conn_gap < 0") {
        FinFeature f = base; f.conn_gap = -1.0;
        CHECK(generate_one_fin(cube, f, 0.2, 0.45).empty());
    }
    SECTION("conn_step == 0") {
        FinFeature f = base; f.conn_step = 0.0;
        CHECK(generate_one_fin(cube, f, 0.2, 0.45).empty());
    }
    SECTION("conn_step < 0") {
        FinFeature f = base; f.conn_step = -4.0;
        CHECK(generate_one_fin(cube, f, 0.2, 0.45).empty());
    }
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
// tabs bridge the gap starting at face-model_overlap (T00-05-01: max(kTabOverlap, line_width) =
// max(0.1, 0.45) = 0.45 with the line_width used below), so the mesh spans ~[9.55, 16] along the
// fin direction, [-0.6, 0.6] across (thickness), and [0, 10] in z. These numbers are unchanged
// from before the §15 rework (only `length` moved, from 6 to 16, to compensate for the window now
// being measured from the anchor instead of from the section's own edge).

namespace {

FinFeature conversion_test_fin()
{
    FinFeature f;
    f.anchor    = Vec3d(0.0, 0.0, 0.0);
    f.dir       = FinGeneratorInternal::dir_from_z_angle(0.0);
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
        CHECK_THAT(bb.min.y(), WithinAbs(9.55, 1e-3)); // tabs reach max(kTabOverlap, line_width) into the cube (T00-05-01)
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
        CHECK_THAT(bb.max.x(), WithinAbs(-9.55, 1e-3)); // T00-05-01: max(kTabOverlap, line_width) reach
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
        f.dir    = FinGeneratorInternal::dir_from_z_angle(0.5 * PI);
        o->fins.push_back(f);

        std::vector<TriangleMesh> fins =
            generate_fin_meshes(*o, to_bed, /*layer_height=*/0.2, /*line_width=*/0.45);
        REQUIRE(fins.size() == 1);

        BoundingBoxf3 bb = fins.front().bounding_box();
        CHECK_THAT(bb.min.z(), WithinAbs(0.0, 1e-3));    // starts at the bed
        CHECK_THAT(bb.max.z(), WithinAbs(10.0, 1e-3));   // exactly `height` tall
        CHECK_THAT(bb.min.x(), WithinAbs(-16.0, 1e-3));  // beyond the cube's -X face
        CHECK_THAT(bb.max.x(), WithinAbs(-9.55, 1e-3)); // T00-05-01: max(kTabOverlap, line_width) reach
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
    f.anchor    = Vec3d(0.0, 0.0, 0.0);
    f.dir       = FinGeneratorInternal::dir_from_z_angle(0.0);
    // height=25 (taller than the 10mm default) so the fin reads clearly
    // against the 40mm-tall cube; thickness/conn_step/conn_gap stay at
    // their defaults (1.2, 4, 1).
    f.height = 25.0;
    // Window semantics (design doc §15): the anchor sits at the cube's own center along u
    // (like the other §15-updated tests, e.g. "fin: wall mesh is a closed, outward-facing
    // solid"), so the section's far-side surface is at local u=10 (half-width) and the
    // window's near edge (past the default 1mm conn_gap) is at u=11. The pre-§15 default
    // length (8) now lands the whole window inside the part+gap -> EMPTY mesh (this is the
    // T00-01-06 fixture the T00-05 review flagged as stale). Bumped to 16, same value used
    // by the other updated tests, for a real 5mm-wide wall.
    f.length = 16.0;

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

TEST_CASE("fin: bed-rigid delta classifier", "[fin]")
{
    using FinGeneratorInternal::is_bed_rigid_delta;

    // Shape-preserving deltas: identity, XY translation, Z rotation, their combo.
    CHECK(is_bed_rigid_delta(Transform3d::Identity()));
    CHECK(is_bed_rigid_delta(Geometry::translation_transform(Vec3d(12.5, -3.0, 0.0))));
    CHECK(is_bed_rigid_delta(Geometry::rotation_transform(Vec3d(0.0, 0.0, 0.7))));
    CHECK(is_bed_rigid_delta(Geometry::translation_transform(Vec3d(1.0, 2.0, 0.0))
                             * Geometry::rotation_transform(Vec3d(0.0, 0.0, M_PI))));
    // 180 deg about Z == diag(-1,-1,1): det=+1, must pass (it is NOT a mirror).
    CHECK(is_bed_rigid_delta(Geometry::scale_transform(Vec3d(-1.0, -1.0, 1.0))));

    // Silhouette-changing deltas: Z lift, tilt, scale, mirror, skew.
    CHECK_FALSE(is_bed_rigid_delta(Geometry::translation_transform(Vec3d(0.0, 0.0, 0.5))));
    CHECK_FALSE(is_bed_rigid_delta(Geometry::rotation_transform(Vec3d(0.3, 0.0, 0.0))));
    CHECK_FALSE(is_bed_rigid_delta(Geometry::rotation_transform(Vec3d(0.0, 0.2, 0.0))));
    CHECK_FALSE(is_bed_rigid_delta(Geometry::scale_transform(Vec3d(2.0, 2.0, 2.0))));
    CHECK_FALSE(is_bed_rigid_delta(Geometry::scale_transform(Vec3d(-1.0, 1.0, 1.0)))); // mirror, det=-1
    Transform3d skew = Transform3d::Identity();
    skew(0, 1) = 0.3;
    CHECK_FALSE(is_bed_rigid_delta(skew));

    // Tolerance: numeric noise well below tol must not flip the verdict.
    Transform3d noisy = Geometry::rotation_transform(Vec3d(0.0, 0.0, 1.1));
    noisy.translation() = Vec3d(5.0, 6.0, 1e-9);
    CHECK(is_bed_rigid_delta(noisy));
}
