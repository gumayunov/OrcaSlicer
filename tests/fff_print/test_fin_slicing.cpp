#include <catch2/catch_all.hpp>

#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include "test_data.hpp"

// T00-02-01 (design 00, phase 02): failing fff_print integration tests for slice-time fin
// injection. No injection code exists yet -- that's the next task (T00-02-02), which will
// inject transient fin volumes at the top of PrintObject::slice_volumes(). These tests are
// the TDD red for that task: "enabled fin adds geometry" is expected to FAIL right now
// (nothing adds the fin's geometry to the sliced object yet); "disabled/no-intersection is
// identical to no fin" is expected to pass trivially (there is no code path that could change
// the output either way).

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {

// A tall, thin box (10 x 10 x 40 mm), centered on the XY origin with its base on the bed
// (z in [0, 40]). its_make_cube()/make_cube() build an UNCENTERED 0..x/0..y/0..z box (see
// TriangleMesh.cpp), so it is re-centered here in XY only -- matching the pattern already
// used by the fin unit tests (tests/libslic3r/test_fin.cpp).
//
// Centering in XY matters for the fin placement below: PrintObject::trafo_centered()
// (used to slice, see PrintObjectSlice.cpp:1149-1174) is the instance transform with the
// object's own XY bounding-box center subtracted (Print.hpp / PrintObject.cpp:98-110). For a
// single, unrotated instance and a mesh whose own XY bbox is already centered on (0,0), that
// composition is (to floating-point precision) the identity -- so this mesh's own local
// coordinates are also the coordinates the slicer slices in, and a FinFeature's anchor_xy /
// z_angle (object-local, see Model.hpp) can be set directly against the geometry below
// without having to separately account for instance placement on the bed.
TriangleMesh tall_thin_box()
{
    TriangleMesh mesh = Slic3r::make_cube(10.0, 10.0, 40.0); // 0..10 x 0..10 x 0..40
    mesh.translate(-5.0f, -5.0f, 0.0f);                      // center XY on the origin; base stays on the bed
    return mesh;
}

// A FinFeature whose plane crosses tall_thin_box(): z_angle = 0 puts the plane's normal along
// +X (FinGenerator.cpp: plane_to_bed(), z_angle=0 => n = +X world), and anchor_xy = (0,0) puts
// the plane at world x=0 -- right through the middle of the box (which spans x in [-5,5]).
// height (20) stays inside the box's own Z range (0..40), so adding the fin cannot add new
// layers, only area within layers that already exist -- required for the "total sliced area"
// metric below to unambiguously measure the fin's own contribution.
FinFeature crossing_fin()
{
    FinFeature f;
    f.anchor_xy = Vec2d(0.0, 0.0);
    f.z_angle   = 0.0;
    f.height    = 20.0;
    f.length    = 6.0;
    f.thickness = 1.2;
    f.conn_step = 4.0;
    f.conn_gap  = 1.0;
    f.enabled   = true;
    return f;
}

struct SliceMetrics
{
    // Sum of ExPolygon::area() over every layer's lslices for the (single) sliced object.
    // Chosen as the one robust metric for both tests in this file: an intersecting, enabled
    // fin can only add solid cross-section somewhere in [0, fin.height], so it must strictly
    // increase this sum; a disabled or non-intersecting fin contributes no geometry anywhere,
    // so this sum -- and the layer count -- must come out exactly as if there were no fin at
    // all. This area-equality check is the phase's practical stand-in for "no injection when
    // disabled/non-intersecting"; full byte-identical G-code comparison is unnecessary here
    // because that code path is a no-op (nothing is added to the model) rather than something
    // that could plausibly produce identical-looking-but-different output.
    double area        = 0.0;
    size_t layer_count = 0;
};

// Builds a single-object Print around tall_thin_box(), slices it, and reports SliceMetrics.
// If `fin` is non-null, it is pushed onto the ModelObject's fins BEFORE Print::apply(), i.e.
// exactly as if a GUI-driven fin were already present on the model when slicing starts.
//
// Print/Model are built by hand here (not via Slic3r::Test::init_print) so that the
// FinFeature can be attached to the ModelObject before the single Print::apply() call --
// init_print()'s Model-reference overload already calls apply() internally, which would
// require a second apply() to pick up a fin added afterwards, an untested code path this
// phase doesn't need. The instance is placed well inside the default 200x200 bed
// (config `printable_area`); this offset cannot affect what gets sliced -- only where these
// tests place the object on the bed -- because trafo_centered() (see tall_thin_box() above)
// excludes the instance's on-bed translation entirely.
SliceMetrics slice_metrics(const FinFeature *fin)
{
    Slic3r::DynamicPrintConfig config = Slic3r::DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "layer_height",               2 },
        { "initial_layer_print_height", 2 },
        { "nozzle_diameter",            3 }, // must be >= layer_height, see test_printobject.cpp
        { "sparse_infill_density",      0 },
    });

    Slic3r::Model model;
    Slic3r::Print  print;

    ModelObject *object = model.add_object();
    object->name += "fin_box.stl";
    object->add_volume(tall_thin_box(), /*modify_to_center_geometry=*/false);
    ModelInstance *instance = object->add_instance();
    instance->set_offset(Vec3d(100.0, 100.0, 0.0)); // well inside the 200x200 default bed
    object->ensure_on_bed();

    if (fin != nullptr)
        object->fins.push_back(*fin);

    print.auto_assign_extruders(object);
    print.apply(model, config);
    // Return value intentionally ignored, matching Slic3r::Test::init_print() (test_data.cpp):
    // validate() also flags unrelated gcode-flavor/firmware advice (e.g. Marlin wanting
    // "G92 E0" in layer_gcode) that has no bearing on whether slicing itself produces correct
    // geometry, which is all these tests care about.
    print.validate();
    print.set_status_silent();
    print.process();

    REQUIRE(print.objects().size() == 1);
    const PrintObject &po = *print.objects().front();

    SliceMetrics m;
    m.layer_count = po.layers().size();
    for (const Layer *layer : po.layers())
        for (const ExPolygon &ex : layer->lslices)
            m.area += ex.area();
    return m;
}

} // namespace

TEST_CASE("fin: enabled fin adds geometry to the sliced object", "[fin]")
{
    SliceMetrics baseline = slice_metrics(nullptr);

    FinFeature   fin = crossing_fin();
    SliceMetrics with_fin = slice_metrics(&fin);

    // Fin height (20) is within the box's own Z range (40), so no layers are added or removed.
    CHECK(with_fin.layer_count == baseline.layer_count);
    // No injection exists yet (T00-02-02) -- this is expected to FAIL until that lands.
    CHECK(with_fin.area > baseline.area);
}

TEST_CASE("fin: disabled/no-intersection slice is identical to no fin", "[fin]")
{
    SliceMetrics baseline = slice_metrics(nullptr);

    SECTION("fin.enabled = false")
    {
        FinFeature fin = crossing_fin();
        fin.enabled    = false;
        SliceMetrics m = slice_metrics(&fin);

        CHECK(m.layer_count == baseline.layer_count);
        CHECK_THAT(m.area, WithinAbs(baseline.area, 1e-6));
    }

    SECTION("fin.enabled = true but the plane misses the object")
    {
        FinFeature fin  = crossing_fin();
        fin.anchor_xy   = Vec2d(1000.0, 1000.0); // far outside the box -> plane never crosses it
        SliceMetrics m  = slice_metrics(&fin);

        CHECK(m.layer_count == baseline.layer_count);
        CHECK_THAT(m.area, WithinAbs(baseline.area, 1e-6));
    }
}
