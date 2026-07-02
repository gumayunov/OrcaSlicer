#include <catch2/catch_all.hpp>

#include "libslic3r/Model.hpp"
#include "libslic3r/Fin/FinGenerator.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include <cereal/archives/binary.hpp>

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
    f.length    = 8.0;

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
    f.length    = 6.0;
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
    f.length     = 6.0;
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
    f.length    = 6.0;
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

    // fin2d is exactly known here: the plane (bed X=0) cuts the cube into a full
    // u in [-10,10], w in [0,30] cross-section; the fin rectangle is u in [0,16]
    // (u_split=0 .. bb.max+length=10+6), w in [0,15] (=height); the default 1mm
    // conn_gap offsets the section to u in [-11,11], eating u in [0,11] of the
    // rectangle, leaving fin2d = u in [11,16] x w in [0,15] -> area 5*15 = 75 mm^2.
    // volume = area * thickness confirms both the extrusion geometry and the
    // winding (a flipped/self-intersecting prism would not match this exactly).
    double wall_volume = 75.0 * f.thickness;
    // Each tab is its_make_cube(conn_gap + 0.2, layer_height, 2*line_width) (see
    // generate_one_fin): u spans the gap plus a 0.1mm overlap on each side, w is
    // exactly one layer tall, n is two perimeters thick.
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
    f.length    = 6.0;
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
    f.height    = 30.0; // fin itself may exceed the model -- allowed
    f.length    = 6.0;
    f.conn_step = 5.0;
    f.conn_gap  = 1.0;

    // Rows above the model's own top (w=10) -- w=15,20,25,30 -- must be skipped
    // by contour_u_at() (nullopt: no section contour up there), not crash.
    TriangleMesh m = generate_one_fin(cube, f, /*layer_height=*/0.2, /*line_width=*/0.45);
    REQUIRE_FALSE(m.empty());
    CHECK(m.stats().open_edges == 0);

    // Rows w=0,5,10 are within the model's own height (10) -> 3 tabs.
    CHECK(m.stats().number_of_parts == 1 + 3);
}
