#include <catch2/catch_all.hpp>

#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Fin/FinGenerator.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/TriangleMesh.hpp"

// Integration tests for slice-time fin injection: transient fin geometry is merged into the
// object's mesh at the top of PrintObject::slice_volumes() and restored afterwards (see
// TransientFinGuard there). These tests pin the two ends of that contract: an intersecting
// fin adds sliced geometry; a non-intersecting fin leaves the slice identical to no fin.

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
// coordinates are also the coordinates the slicer slices in, and a FinFeature's anchor /
// dir (object-local, see Model.hpp) can be set directly against the geometry below
// without having to separately account for instance placement on the bed.
TriangleMesh tall_thin_box()
{
    TriangleMesh mesh = Slic3r::make_cube(10.0, 10.0, 40.0); // 0..10 x 0..10 x 0..40
    mesh.translate(-5.0f, -5.0f, 0.0f);                      // center XY on the origin; base stays on the bed
    return mesh;
}

// A FinFeature whose plane crosses tall_thin_box(): dir = +Y (the old z_angle = 0) puts the
// plane's normal along +X (FinGenerator.cpp: plane_to_bed(), z_angle=0 => n = +X world), and
// anchor = (0,0,0) puts the plane at world x=0 -- right through the middle of the box (which
// spans x in [-5,5]).
// height (20) stays inside the box's own Z range (0..40), so adding the fin cannot add new
// layers, only area within layers that already exist -- required for the "total sliced area"
// metric below to unambiguously measure the fin's own contribution.
FinFeature crossing_fin()
{
    FinFeature f;
    f.anchor    = Vec3d(0.0, 0.0, 0.0);
    f.dir       = FinGeneratorInternal::dir_from_z_angle(0.0);
    f.height    = 20.0;
    // The gizmo window spans u in [0, length] from the anchor, and fin material only exists past
    // the section's far surface plus the gap. With the anchor at the box's own center the surface
    // is at local u=5 (half-width) and the window's usable region starts at u=6 (past the 1mm
    // gap), so length=8 leaves a real (if narrow, u in [6,8]) window open -- enough to keep "adds
    // geometry" meaningfully true, the qualitative property this fixture's tests check.
    f.length    = 8.0;
    f.thickness = 1.2;
    f.conn_step = 4.0;
    f.conn_gap  = 1.0;
    return f;
}

struct SliceMetrics
{
    // Sum of ExPolygon::area() over every layer's lslices for the (single) sliced object.
    // Chosen as the one robust metric for both tests in this file: an intersecting
    // fin can only add solid cross-section somewhere in [0, fin.height], so it must strictly
    // increase this sum; a non-intersecting fin contributes no geometry anywhere,
    // so this sum -- and the layer count -- must come out exactly as if there were no fin at
    // all. This area-equality check is the phase's practical stand-in for "no injection when
    // non-intersecting"; full byte-identical G-code comparison is unnecessary here
    // because that code path is a no-op (nothing is added to the model) rather than something
    // that could plausibly produce identical-looking-but-different output.
    double area        = 0.0;
    size_t layer_count = 0;
};

// Sums SliceMetrics off an already-processed Print (single object expected). Factored out of
// slice_metrics() so the re-apply test below can measure the SAME Print instance twice.
SliceMetrics metrics_from(const Print &print)
{
    REQUIRE(print.objects().size() == 1);
    const PrintObject &po = *print.objects().front();

    SliceMetrics m;
    m.layer_count = po.layers().size();
    for (const Layer *layer : po.layers())
        for (const ExPolygon &ex : layer->lslices)
            m.area += ex.area();
    return m;
}

// Builds a single-object Print around tall_thin_box(), slices it, and reports SliceMetrics.
// Every FinFeature in `fins` is pushed onto the ModelObject's fins BEFORE Print::apply(), i.e.
// exactly as if GUI-driven fins were already present on the model when slicing starts.
//
// Print/Model are built by hand here (not via Slic3r::Test::init_print) so that the
// FinFeatures can be attached to the ModelObject before the single Print::apply() call --
// init_print() already calls apply() internally, which would require a second apply() to
// pick up a fin added afterwards, an extra code path these tests don't mean to exercise.
// The instance is placed well inside the default 200x200 bed
// (config `printable_area`); this offset cannot affect what gets sliced -- only where these
// tests place the object on the bed -- because trafo_centered() (see tall_thin_box() above)
// excludes the instance's on-bed translation entirely.
SliceMetrics slice_metrics(const std::vector<FinFeature> &fins)
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

    for (const FinFeature &f : fins)
        object->fins.push_back(f);

    print.auto_assign_extruders(object);
    print.apply(model, config);
    // validate()'s return value is intentionally ignored:
    // validate() also flags unrelated gcode-flavor/firmware advice (e.g. Marlin wanting
    // "G92 E0" in layer_gcode) that has no bearing on whether slicing itself produces correct
    // geometry, which is all these tests care about.
    print.validate();
    print.set_status_silent();
    print.process();

    return metrics_from(print);
}

// Single-fin convenience wrapper: null means "no fin at all" (the baseline), non-null a
// single fin.
SliceMetrics slice_metrics(const FinFeature *fin)
{
    std::vector<FinFeature> fins;
    if (fin != nullptr)
        fins.push_back(*fin);
    return slice_metrics(fins);
}

} // namespace

TEST_CASE("fin: fin adds geometry to the sliced object", "[fin]")
{
    SliceMetrics baseline = slice_metrics(nullptr);

    FinFeature   fin = crossing_fin();
    SliceMetrics with_fin = slice_metrics(&fin);

    // Fin height (20) is within the box's own Z range (40), so no layers are added or removed.
    CHECK(with_fin.layer_count == baseline.layer_count);
    CHECK(with_fin.area > baseline.area);
}

TEST_CASE("fin: every fin contributes geometry to the sliced object", "[fin]")
{
    // Slice-time injection is multi-fin: a second fin must ADD its own cross-section, not
    // replace or shadow the first. The second fin mirrors crossing_fin() to the box's -Y side
    // (dir flipped by PI):
    // center-anchoring is symmetric, so the same length keeps its window open there too, and
    // the two fins occupy disjoint regions -- their combined area is strictly larger.
    FinFeature first  = crossing_fin();
    FinFeature second = crossing_fin();
    second.dir = FinGeneratorInternal::dir_from_z_angle(PI); // opposite side of the box

    SliceMetrics with_one = slice_metrics({first});
    SliceMetrics with_two = slice_metrics({first, second});

    // Both fins share crossing_fin()'s height (20, inside the box's own 40mm Z range), so the
    // second fin cannot add layers -- only area within layers that already exist.
    CHECK(with_two.layer_count == with_one.layer_count);
    CHECK(with_two.area > with_one.area); // the second fin must add its own cross-section
}

TEST_CASE("fin: no-intersection slice is identical to no fin", "[fin]")
{
    SliceMetrics baseline = slice_metrics(nullptr);

    SECTION("the plane misses the object")
    {
        FinFeature fin  = crossing_fin();
        fin.anchor   = Vec3d(1000.0, 1000.0, 0.0); // far outside the box -> plane never crosses it
        SliceMetrics m  = slice_metrics(&fin);

        CHECK(m.layer_count == baseline.layer_count);
        CHECK_THAT(m.area, WithinAbs(baseline.area, 1e-6));
    }
}

TEST_CASE("fin: editing a fin re-slices the existing print", "[fin]")
{
    // Regression test for the incremental-apply path: every other test in this file builds a
    // FRESH Print per slice (see slice_metrics()), which is exactly why this bug escaped them.
    // In the GUI the SAME Print instance is re-applied after a fin edit; Print::apply()'s
    // fins_differ branch must invalidate the objects' own posSlice step, or process() reuses
    // the cached layers with the STALE fin baked in.
    SliceMetrics baseline = slice_metrics(nullptr); // fresh slice, no fin at all

    // Same setup as slice_metrics(), but Model/Print are kept alive across a second apply().
    Slic3r::DynamicPrintConfig config = Slic3r::DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "layer_height",               2 },
        { "initial_layer_print_height", 2 },
        { "nozzle_diameter",            3 }, // must be >= layer_height, see test_printobject.cpp
        { "sparse_infill_density",      0 },
    });

    Slic3r::Model model;
    Slic3r::Print print;

    ModelObject *object = model.add_object();
    object->name += "fin_box.stl";
    object->add_volume(tall_thin_box(), /*modify_to_center_geometry=*/false);
    ModelInstance *instance = object->add_instance();
    instance->set_offset(Vec3d(100.0, 100.0, 0.0)); // well inside the 200x200 default bed
    object->ensure_on_bed();
    object->fins.push_back(crossing_fin());

    print.auto_assign_extruders(object);
    print.apply(model, config);
    print.validate(); // return value intentionally ignored, see slice_metrics()
    print.set_status_silent();
    print.process();

    SliceMetrics with_fin = metrics_from(print);
    // Sanity: the crossing fin really contributed geometry to the first slice.
    CHECK(with_fin.layer_count == baseline.layer_count);
    REQUIRE(with_fin.area > baseline.area);

    // "Edit" the fin exactly as the GUI does: mutate the model, then re-apply to the SAME
    // Print. The new anchor puts the fin's plane far outside the box, so the fin no longer
    // intersects and must contribute nothing to the re-slice.
    model.objects.front()->fins.front().anchor = Vec3d(1000.0, 1000.0, 0.0);
    print.apply(model, config);
    print.process();

    SliceMetrics after_move = metrics_from(print);
    CHECK(after_move.layer_count == baseline.layer_count);
    // With the bug present this fails: posSlice stays done, the cached layers keep the old
    // fin's cross-section, and after_move.area sticks at with_fin.area.
    CHECK_THAT(after_move.area, WithinAbs(baseline.area, 1e-6));
}
