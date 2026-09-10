// Spec §3.1 Rule U6 — the compile-time dimensional type system.
//
// The negative cases (what must NOT compile) live in
// tests/compile_fail/*.cpp and are checked by tools/check_compile_fail.py,
// because a test that must fail to build cannot live in a passing translation
// unit. This file covers the positive behaviour.
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "rcvd/models/bicycle.hpp"
#include "rcvd/units/quantity.hpp"
#include "support/fixtures.hpp"

using namespace rcvd;
using namespace rcvd::units;
using Catch::Matchers::WithinRel;

TEST_CASE("U6: quantities carry their dimension through arithmetic", "[units][u6]") {
    const Force f = of(1000.0, newton);
    const Length l = of(2.0, metre);
    const Angle a = of(90.0, degree);

    // Products and quotients derive the right type without being told.
    static_assert(std::is_same_v<decltype(f * l), Moment>);
    static_assert(std::is_same_v<decltype(f / a), CorneringStiffness>);
    static_assert(std::is_same_v<decltype(f * l / a), AngularStiffness>);
    static_assert(std::is_same_v<decltype(l / of(1.0, second)), Speed>);

    REQUIRE_THAT(in(f * l, newton_metre), WithinRel(2000.0, 1e-15));
    REQUIRE_THAT((f / a).si(), WithinRel(1000.0 / (PI / 2.0), 1e-15));
}

TEST_CASE("U6: Rule U5 composites convert exactly", "[units][u6]") {
    // 1 lbf/deg = 254.864324893 N/rad.
    const CorneringStiffness one = of(1.0, lbf_per_deg);
    REQUIRE_THAT(one.si(), WithinRel(254.864324893, 1e-11));
    REQUIRE_THAT(in(one, newton_per_rad), WithinRel(254.864324893, 1e-11));

    // 1 lbf*ft/deg = 77.6826 N*m/rad.
    const AngularStiffness k = of(1.0, lbf_ft_per_deg);
    REQUIRE_THAT(k.si(), WithinRel(77.6826, 1e-5));

    // The composites are formed from the primitives, so this holds exactly.
    REQUIRE(of(1.0, lbf_per_deg).si() == of(1.0, pound_force).si() / of(1.0, degree).si());
}

TEST_CASE("U6: round trips are lossless in both directions", "[units][u6]") {
    // Appendix D's < 1e-15 gate, now expressed on typed quantities.
    REQUIRE(in(of(3.7, foot), foot) == 3.7);
    REQUIRE(in(of(-1234.5, pound_force), pound_force) == -1234.5);
    REQUIRE(in(of(640.0, lbf_per_deg), lbf_per_deg) == 640.0);
    REQUIRE(in(of(0.5, gee), gee) == 0.5);
}

TEST_CASE("U6: the four derivative dimensions are distinct types", "[units][u6]") {
    // §8.3's six derivatives have four dimensions. That they are DIFFERENT TYPES
    // is the whole point: it is what makes the B-04 mistake unwriteable.
    static_assert(!std::is_same_v<CorneringStiffness, AngularStiffness>);
    static_assert(!std::is_same_v<CorneringStiffness, SideForceDamping>);
    static_assert(!std::is_same_v<AngularStiffness, YawDamping>);
    static_assert(!std::is_same_v<SideForceDamping, YawDamping>);

    // And they are exactly what the algebra of §8.3 produces.
    const Force F = of(1.0, newton);
    const Length L = of(1.0, metre);
    const Angle A = of(1.0, radian);
    const Duration T = of(1.0, second);
    static_assert(std::is_same_v<decltype(F / A), CorneringStiffness>);       // Y_beta
    static_assert(std::is_same_v<decltype(F * T / A), SideForceDamping>);     // Y_r
    static_assert(std::is_same_v<decltype(F * L / A), AngularStiffness>);     // N_beta
    static_assert(std::is_same_v<decltype(F * L * T / A), YawDamping>);       // N_r
}

TEST_CASE("U6: B-03 ... B-08 through the typed views", "[units][u6][acceptance]") {
    // The same acceptance vectors, now with the display unit chosen by TYPE rather
    // than by the author remembering which divisor goes with which derivative.
    const auto v = core::resolve(fixtures::ch5_input());
    REQUIRE(v.ok());
    const auto d = models::stability_derivatives(v.value(), fixtures::ch5_tires(),
                                                 44.0 * FOOT);

    REQUIRE_THAT(in(d.Y_beta_q(), lbf_per_deg), WithinRel(-1220.0, 1e-12));      // B-03
    REQUIRE_THAT(in(d.Y_r_q(), lbf_s_per_deg), WithinRel(10.965, 1e-4));        // B-04
    REQUIRE_THAT(in(d.Y_delta_q(), lbf_per_deg), WithinRel(640.0, 1e-12));      // B-05
    REQUIRE_THAT(in(d.N_beta_q(), lbf_ft_per_deg), WithinRel(482.44, 1e-4));    // B-06
    REQUIRE_THAT(in(d.N_r_q(), lbf_ft_s_per_deg), WithinRel(-528.74, 1e-4));    // B-07
    REQUIRE_THAT(in(d.N_delta_q(), lbf_ft_per_deg), WithinRel(2396.53, 1e-4));  // B-08
}

TEST_CASE("U6: g is admissible only as a presentation unit for acceleration",
          "[units][u6]") {
    // Rule U4: g is a field constant, not a unit. It appears here solely as the
    // §3.4 presentation unit for A_Y and A_X, and its dimension confines it there.
    static_assert(std::is_same_v<decltype(gee)::Dimension, Acceleration::Dimension>);
    const Acceleration ay = of(1.5173, gee);
    REQUIRE_THAT(ay.si(), WithinRel(1.5173 * G_STANDARD, 1e-15));
    REQUIRE_THAT(in(ay, gee), WithinRel(1.5173, 1e-15));

    // B-13 falls out: the front axle force divided by the front axle load.
    const Force Fy = of(3200.0, pound_force);
    const Force W_F = of(2109.0, pound_force);
    REQUIRE_THAT((Fy / W_F).si(), WithinRel(1.5173, 1e-4));
    // Force/Force is a genuine scalar, so it has no dimension left.
    static_assert(std::is_same_v<decltype(Fy / W_F), Scalar>);
}

TEST_CASE("U6: comparison and accumulation stay within a dimension", "[units][u6]") {
    Force total = of(0.0, newton);
    for (double x : {100.0, 250.5, -30.0}) total += of(x, pound_force);
    REQUIRE_THAT(in(total, pound_force), WithinRel(320.5, 1e-12));
    REQUIRE(of(2.0, foot) < of(1.0, metre));

    // Types do not repeal §13.1. FOOT and INCH are each exact by definition, but
    // 12 * 0.0254 and 0.3048 are DIFFERENT roundings in binary64, so 12 in and
    // 1 ft are not bit-equal. The type system prevents dimension errors; it does
    // not make floating point exact, and §13.1's "comparisons against zero SHALL
    // use explicit tolerances scaled to the quantity, never bare equality" still
    // governs.
    REQUIRE(of(12.0, inch).si() != of(1.0, foot).si());
    REQUIRE_THAT(in(of(12.0, inch), foot), WithinRel(1.0, 1e-15));
}
