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
