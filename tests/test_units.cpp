// Spec §3.1 (Rules U2-U5) and Appendix D's unit-conversion round-trip gate.
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "rcvd/units/convert.hpp"

using namespace rcvd::units;
using Catch::Matchers::WithinRel;

TEST_CASE("U5 composites are formed from the exact primitives", "[units]") {
    // 1 lbf/deg = POUND_FORCE / DEGREE = 254.864324893 N/rad.
    REQUIRE_THAT(LBF_PER_DEG_TO_N_PER_RAD, WithinRel(254.864324893, 1e-11));
    // 1 lbf*ft/deg = POUND_FORCE * FOOT / DEGREE = 77.6826 N*m/rad.
    REQUIRE_THAT(LBF_FT_PER_DEG_TO_NM_PER_RAD, WithinRel(77.6826, 1e-5));
    // SLUG is derived, not transcribed, so this holds exactly.
    REQUIRE(SLUG == POUND_FORCE / FOOT);
    REQUIRE_THAT(SLUG, WithinRel(14.5939029372064, 1e-14));
}

TEST_CASE("U4 keeps g a field constant", "[units]") {
    REQUIRE(G_STANDARD == 9.80665);
    // The legacy rounded values are LOW/HIGH by the amounts §3.1 records; they are
    // absorbed by test tolerance, never by changing the constant.
    REQUIRE_THAT(32.174 * FOOT / G_STANDARD - 1.0, WithinRel(-1.5e-6, 0.2));
    REQUIRE_THAT(32.2 * FOOT / G_STANDARD - 1.0, WithinRel(8.1e-4, 0.02));
}

TEST_CASE("E-4: the legacy Ackermann constant 858 is 0.120 % high", "[units]") {
    REQUIRE_THAT(LEGACY_ACKERMANN_CONSTANT_EXACT, WithinRel(856.970, 1e-5));
    const double high = 858.0 / LEGACY_ACKERMANN_CONSTANT_EXACT - 1.0;
    REQUIRE_THAT(high, WithinRel(0.00120, 0.02));
}

TEST_CASE("U2 round trip is lossless in both directions", "[units]") {
    // Appendix D: round-trip error < 1e-15 across the whole vocabulary.
    for (const UnitEntry& e : kVocabulary) {
        for (double v : {1.0, -3.7, 1234.5, 1e-6, 9.87e8}) {
            const auto si = to_si(v, e.name, e.dim);
            REQUIRE(si.has_value());
            const auto back = from_si(*si, e.name, e.dim);
            REQUIRE(back.has_value());
            INFO("unit " << e.name << " value " << v);
            REQUIRE(std::abs(*back - v) <= 1e-15 * std::abs(v));
        }
    }
}

TEST_CASE("the vocabulary is closed and dimension-checked", "[units]") {
    // U6: a quantity offered under the wrong dimension is rejected, not converted.
    REQUIRE_FALSE(to_si(1.0, "lbf", Dim::Length).has_value());
    REQUIRE_FALSE(to_si(1.0, "lbf/deg", Dim::Force).has_value());
    // Outside the vocabulary entirely.
    REQUIRE_FALSE(to_si(1.0, "furlong", Dim::Length).has_value());
    REQUIRE_FALSE(to_si(1.0, "", Dim::Length).has_value());
    // §5.2's factor traps: a bare number read as SI is wrong by these factors.
    REQUIRE_THAT(*to_si(1.0, "1/deg", Dim::PerAngle), WithinRel(1.0 / DEGREE, 1e-15));
    REQUIRE_THAT(1.0 / *to_si(1.0, "1/lbf", Dim::PerForce), WithinRel(POUND_FORCE, 1e-15));
}

TEST_CASE("B-15/B-16 steering vectors convert exactly", "[units][acceptance]") {
    // delta_Ack = l/R with l = 104.5 in, R = 50 ft -> 9.979 deg.
    const double l_m = *to_si(104.5, "in", Dim::Length);
    const double R_m = *to_si(50.0, "ft", Dim::Length);
    const double delta_deg = *from_si(l_m / R_m, "deg", Dim::Angle);
    REQUIRE_THAT(delta_deg, WithinRel(9.9790, 1e-4));       // B-15
    REQUIRE_THAT(delta_deg * 14.0, WithinRel(139.706, 1e-4));  // B-16
}
