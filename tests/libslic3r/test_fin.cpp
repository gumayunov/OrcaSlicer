#include <catch2/catch_all.hpp>

#include "libslic3r/Model.hpp"

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
