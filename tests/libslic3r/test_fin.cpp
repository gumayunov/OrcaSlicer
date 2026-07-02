#include <catch2/catch_all.hpp>

#include "libslic3r/Model.hpp"

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
