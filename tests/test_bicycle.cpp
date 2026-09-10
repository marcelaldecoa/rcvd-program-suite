// Spec §8 and the §17.2 acceptance vectors B-01 ... B-31.
//
// §17.1 tolerance policy: acceptance tolerances are dominated by the precision to
// which the source published the answer (typically 3-4 significant figures) and
// by its use of g = 32.2 ft/s^2. Default relative tolerance 3e-3 unless stated;
// invariant tests use 1e-12.
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

#include "rcvd/models/bicycle.hpp"
#include "rcvd/units/convert.hpp"
#include "support/fixtures.hpp"

using namespace rcvd;
using namespace rcvd::models;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {
constexpr double kDefaultTol = 3e-3;  // §17.1
constexpr double kLbfDeg = fixtures::kLbfDeg;

core::Vehicle ch5() {
    auto r = core::resolve(fixtures::ch5_input());
    REQUIRE(r.ok());
    return r.value();
}

// Display-side helpers, so the vectors are asserted in the units they were
// published in (Rule U2: conversion at the boundary, never inside).
double to_lbf_per_deg(double N_per_rad) { return N_per_rad / kLbfDeg; }
double to_lbf_ft_per_deg(double Nm_per_rad) {
    return Nm_per_rad / units::LBF_FT_PER_DEG_TO_NM_PER_RAD;
}
}  // namespace

// ------------------------------------------------------- §5.1 geometry resolution
TEST_CASE("B-01/B-02: mass and the a/b inversion", "[bicycle][acceptance]") {
    const core::Vehicle v = ch5();
    // B-01: m = 115 slug.
    REQUIRE_THAT(v.m / units::SLUG, WithinRel(115.0, 1e-5));
    // B-02: a = 3.74 ft, b = 4.96 ft. §5.1 demands a dedicated test for this
    // inversion — 57 % FRONT weight puts the CG closer to the front axle, so
    // a = 0.43*l. Getting it backwards is the single most common defect here.
    REQUIRE_THAT(v.a / units::FOOT, WithinRel(3.7446, 1e-4));
    REQUIRE_THAT(v.b / units::FOOT, WithinRel(4.9637, 1e-4));
    REQUIRE(v.a < v.b);
    REQUIRE_THAT(v.front_weight_fraction(), WithinRel(0.57, 1e-12));
    // The static axle loads follow, and cross over: W_F is set by b.
    REQUIRE_THAT(v.W_F() / units::POUND_FORCE, WithinRel(2109.0, 1e-9));
    REQUIRE_THAT(v.W_R() / units::POUND_FORCE, WithinRel(1591.0, 1e-9));
}

TEST_CASE("{a,b} and {wheelbase,fraction} forms agree", "[bicycle][validation]") {
    core::VehicleInput in = fixtures::ch5_input();
    in.wheelbase_m.reset();
    in.front_weight_fraction.reset();
    in.a_m = 0.43 * 104.5 * units::INCH;
    in.b_m = 0.57 * 104.5 * units::INCH;
    const auto r = core::resolve(in);
    REQUIRE(r.ok());
    REQUIRE_THAT(r.value().a, WithinRel(ch5().a, 1e-15));
    REQUIRE_THAT(r.value().b, WithinRel(ch5().b, 1e-15));
}

TEST_CASE("E-VEH-010: over-specification is rejected", "[bicycle][validation]") {
    // §5.1: the schema accepts exactly one form per group. Silent precedence is
    // forbidden, so supplying both forms must fail rather than pick one.
    core::VehicleInput both = fixtures::ch5_input();
    both.a_m = 1.0;
    both.b_m = 2.0;
    REQUIRE(core::resolve(both).error().code == "E-VEH-010");

    core::VehicleInput mass_and_weight = fixtures::ch5_input();
    mass_and_weight.mass_kg = 1500.0;
    REQUIRE(core::resolve(mass_and_weight).error().code == "E-VEH-010");

    core::VehicleInput neither = fixtures::ch5_input();
    neither.weight_N.reset();
    REQUIRE(core::resolve(neither).error().code == "E-VEH-010");
}

TEST_CASE("E-VEH-011: sprung/unsprung consistency (5.1a)-(5.1c)", "[bicycle][validation]") {
    // The §5.1 worked example: 3700 lbf total, 240/260 lbf unsprung per axle,
    // 11 in unsprung CG height => h_s = 21.406 in and a_s = 43.466 in.
    core::VehicleInput in = fixtures::ch5_input();
    in.unsprung_mass_front_kg = 240.0 * units::POUND_FORCE / units::G_STANDARD;
    in.unsprung_mass_rear_kg  = 260.0 * units::POUND_FORCE / units::G_STANDARD;
    in.unsprung_cg_height_front_m = 11.0 * units::INCH;
    in.unsprung_cg_height_rear_m  = 11.0 * units::INCH;

    const auto derived = core::resolve(in);
    REQUIRE(derived.ok());
    REQUIRE_THAT(derived.value().h_s / units::INCH, WithinRel(21.406, 1e-4));
    REQUIRE_THAT(derived.value().a_s / units::INCH, WithinRel(43.466, 1e-4));

    // Supplying the consistent values is accepted...
    in.sprung_cg_height_m = derived.value().h_s;
    in.sprung_cg_front_distance_m = derived.value().a_s;
    REQUIRE(core::resolve(in).ok());

    // ...and an inconsistent one is not. Leaving a_s free is a quiet source of
    // load-transfer error, so it is checked as strictly as h_s.
    in.sprung_cg_front_distance_m = derived.value().a_s * 1.01;
    REQUIRE(core::resolve(in).error().code == "E-VEH-011");

    in.sprung_cg_front_distance_m = derived.value().a_s;
    in.sprung_cg_height_m = derived.value().h_s * 1.01;
    REQUIRE(core::resolve(in).error().code == "E-VEH-011");
}

// ------------------------------------------------------ §8.3 stability derivatives
TEST_CASE("B-03 ... B-08: stability derivatives at 44 ft/s", "[bicycle][acceptance]") {
    const core::Vehicle v = ch5();
    const double V = 44.0 * units::FOOT;  // 30 mph
    const auto d = stability_derivatives(v, fixtures::ch5_tires(), V);

    // B-03: Y_beta = -1220 lbf/deg (exact: -640 - 580).
    REQUIRE_THAT(to_lbf_per_deg(d.Y_beta), WithinRel(-1220.0, 1e-12));
    // B-04: Y_r = +10.98 lbf/(deg/s) published; computed 10.965. Y_r is N*s/rad,
    // so lbf*s/deg is the same divisor as lbf/deg.
    REQUIRE_THAT(d.Y_r / kLbfDeg, WithinRel(10.965, 1e-4));
    REQUIRE_THAT(d.Y_r / kLbfDeg, WithinRel(10.98, kDefaultTol));
    // B-05: Y_delta = +640 lbf/deg. POSITIVE, because C_F < 0.
    REQUIRE_THAT(to_lbf_per_deg(d.Y_delta), WithinRel(640.0, 1e-12));
    REQUIRE(d.Y_delta > 0.0);
    // B-06: N_beta = +483.2 published, 482.44 computed.
    REQUIRE_THAT(to_lbf_ft_per_deg(d.N_beta), WithinRel(482.44, 1e-4));
    REQUIRE_THAT(to_lbf_ft_per_deg(d.N_beta), WithinRel(483.2, kDefaultTol));
    REQUIRE(d.understeering());
    // B-07: N_r = -527.7 published, -528.74 computed.
    REQUIRE_THAT(to_lbf_ft_per_deg(d.N_r), WithinRel(-528.74, 1e-4));
    REQUIRE_THAT(to_lbf_ft_per_deg(d.N_r), WithinRel(-527.7, kDefaultTol));
    // B-08: N_delta = +2393.6 published, 2396.53 computed.
    REQUIRE_THAT(to_lbf_ft_per_deg(d.N_delta), WithinRel(2396.53, 1e-4));
}

TEST_CASE("§8.4 second-column trap and B sign", "[bicycle][regression]") {
    const core::Vehicle v = ch5();
    const double V = 44.0 * units::FOOT;
    const auto d = stability_derivatives(v, fixtures::ch5_tires(), V);
    const auto ss = state_space(v, d);

    // A01 is Y_r/m - u, not (Y_r - m*u) and not Y_r/(m*u). The two wrong forms are
    // spelled out so the test fails loudly if either is ever substituted.
    REQUIRE_THAT(ss.A[0][1], WithinRel(d.Y_r / v.m - V, 1e-15));
    REQUIRE(std::abs(ss.A[0][1] - d.Y_r / (v.m * V)) > 1.0);
    REQUIRE(std::abs(ss.A[0][1] - (d.Y_r - v.m * V)) > 1.0);
    // A11 is N_r/I_z with no second division by u.
    REQUIRE_THAT(ss.A[1][1], WithinRel(d.N_r / v.I_z, 1e-15));
    // B is positive because C_F < 0.
    REQUIRE(ss.B[0] > 0.0);
    REQUIRE(ss.B[1] > 0.0);
}

// ------------------------------------------------------------ §8.5 steady state
TEST_CASE("B-09 ... B-12: handling metrics", "[bicycle][acceptance]") {
    const core::Vehicle v = ch5();
    const auto m = handling_metrics(v, fixtures::ch5_tires());

    // B-09: K = 3.45e-5 s^2/ft^2 published, 3.4399e-5 computed.
    const double K_per_ft2 = m.K * units::FOOT * units::FOOT;
    REQUIRE_THAT(K_per_ft2, WithinRel(3.4399e-5, 1e-4));
    REQUIRE_THAT(K_per_ft2, WithinRel(3.45e-5, kDefaultTol));
    // B-10: V_ch = 170.2 ft/s published, 170.50 computed.
    REQUIRE(m.V_ch.has_value());
    REQUIRE_FALSE(m.V_crit.has_value());
    REQUIRE_THAT(*m.V_ch / units::FOOT, WithinRel(170.50, 1e-4));
    // B-11: NSP = 0.475, SM = 0.045.
    REQUIRE_THAT(m.NSP, WithinRel(0.47541, 1e-4));
    REQUIRE_THAT(m.SM, WithinRel(0.04541, 1e-4));
    // B-12: neutral-steer a = 4.14 ft, i.e. where N_beta = 0.
    REQUIRE_THAT(m.NSP * v.l / units::FOOT, WithinRel(4.1400, 1e-4));
}

TEST_CASE("I-1 (B-26): derivative and axle forms of the understeer gradient",
          "[bicycle][invariant]") {
    // §8.5: a strong cross-check on the §8.3 sign conventions. Tolerance 1e-12.
    const core::Vehicle v = ch5();
    const auto C = fixtures::ch5_tires();
    const auto m = handling_metrics(v, C);

    const double lhs = m.K * v.l * v.g;                       // (8.14) x l x g
    const double rhs = understeer_gradient_axle_form(v, C);   // (8.24)
    REQUIRE_THAT(lhs, WithinRel(rhs, 1e-12));
    // 9.63787e-3 rad/g = 0.5522 deg/g.
    REQUIRE_THAT(lhs, WithinRel(9.63787e-3, 1e-5));
    REQUIRE_THAT(lhs / units::DEGREE, WithinRel(0.5522, 1e-4));
}

TEST_CASE("I-2 (B-25, B-27): three routes to the yaw-rate gain", "[bicycle][invariant]") {
    const core::Vehicle v = ch5();
    const auto C = fixtures::ch5_tires();
    const double V = 44.0 * units::FOOT;
    const auto m = handling_metrics(v, C);

    const double via_K    = yaw_rate_gain(v, m.K, V);                    // (8.15)
    const double via_derv = yaw_rate_gain_derivative_form(v, C, V);      // (8.25)
    const double via_tf   = yaw_rate_transfer_function(v, C, V).dc_gain();  // (8.28)

    REQUIRE_THAT(via_K, WithinRel(via_derv, 1e-14));
    REQUIRE_THAT(via_K, WithinRel(via_tf, 1e-14));
    // B-27: all three give 4.73716 s^-1 for CH5 at 44 ft/s (13.41 m/s).
    REQUIRE_THAT(via_K, WithinRel(4.73716, 1e-5));

    // The steady-state solve must agree with the closed form too.
    const double delta = 0.02;
    const auto ss = steady_state(v, C, V, delta);
    REQUIRE(ss.ok());
    REQUIRE_THAT(ss.value().r_rad_per_s / delta, WithinRel(via_K, 1e-10));
}

TEST_CASE("B-15/B-16: Ackermann steer and steering-wheel angle", "[bicycle][acceptance]") {
    const core::Vehicle v = ch5();
    const double R = 50.0 * units::FOOT;
    const double delta_ack = v.l / R;                       // (8.17)
    REQUIRE_THAT(delta_ack / units::DEGREE, WithinRel(9.9790, 1e-4));
    REQUIRE_THAT(delta_ack / units::DEGREE * v.steering_ratio, WithinRel(139.706, 1e-4));
}

TEST_CASE("B-30: the yaw-rate coefficient r = 84*V/R", "[bicycle][acceptance]") {
    // deg/s, mph, ft. 57.2958 * 1.466667 = 84.034; the source quotes 84.
    const double coeff = (1.0 / units::DEGREE) * (units::MPH / units::FOOT);
    REQUIRE_THAT(coeff, WithinRel(84.034, 1e-4));
    REQUIRE_THAT(coeff, WithinRel(84.0, 1e-3));
}

TEST_CASE("B-17/B-29: crowned road is speed-independent", "[bicycle][acceptance]") {
    const core::Vehicle v = ch5();
    const auto C = fixtures::ch5_tires();
    const auto t = tilted_road_trim(v, C, 5.0 * units::DEGREE);

    // B-17: beta = 0.239 deg, delta = -0.048 deg.
    REQUIRE_THAT(t.beta_rad / units::DEGREE, WithinRel(0.2392, 1e-3));
    REQUIRE_THAT(t.delta_rad / units::DEGREE, WithinRel(-0.0480, 3e-3));
    // The axle forces carry no net yaw moment.
    REQUIRE_THAT(v.a * t.Fy_F_N - v.b * t.Fy_R_N, WithinAbs(0.0, 1e-9));

    // B-29: the answer does not depend on speed at all — there is no speed in it.
    // Asserted structurally rather than by sampling: tilted_road_trim takes no V.
    REQUIRE_THAT(t.Fy_F_N + t.Fy_R_N,
                 WithinRel(-v.m * v.g * std::sin(5.0 * units::DEGREE), 1e-12));
}

TEST_CASE("B-18: crosswind at 1.5 ft ahead of the CG", "[bicycle][acceptance]") {
    const core::Vehicle v = ch5();
    const double F = 100.0 * units::POUND_FORCE;
    const double d = 1.5 * units::FOOT;
    const auto c = crosswind_trim(v, F, d);

    // Corrected in this repo's spec commit: -74.2249 / -25.7751, not -74.29/-25.71.
    REQUIRE_THAT(c.Fy_F_N / units::POUND_FORCE, WithinRel(-74.2249, 1e-5));
    REQUIRE_THAT(c.Fy_R_N / units::POUND_FORCE, WithinRel(-25.7751, 1e-5));
    // The two identities the vector exists to enforce.
    REQUIRE_THAT(c.Fy_F_N + c.Fy_R_N, WithinRel(-F, 1e-12));
    REQUIRE_THAT(v.a * c.Fy_F_N - v.b * c.Fy_R_N, WithinRel(-F * d, 1e-12));
}

TEST_CASE("B-19: reversed weight distribution flips N_beta", "[bicycle][acceptance]") {
    const auto r = core::resolve(fixtures::ch5_os_input());
    REQUIRE(r.ok());
    const auto d = stability_derivatives(r.value(), fixtures::ch5_tires(), 44.0 * units::FOOT);
    REQUIRE_THAT(to_lbf_ft_per_deg(d.N_beta), WithinRel(-1004.94, 1e-5));
    REQUIRE_FALSE(d.understeering());  // oversteer
}

// -------------------------------------------------- §8.6 transient, First Example
TEST_CASE("B-20 ... B-24: First Example eigenstructure", "[bicycle][acceptance]") {
    const auto rv = core::resolve(fixtures::first_example_input());
    REQUIRE(rv.ok());
    const core::Vehicle v = rv.value();
    const auto C = fixtures::first_example_tires();
    const double V = 40.0 * units::MPH;

    const auto e = eigenstructure(v, C, V);
    // B-20: s^2 + 16.188 s + 87.698 published; 16.190 / 87.813 computed.
    REQUIRE_THAT(-e.trace, WithinRel(16.190, 1e-4));
    REQUIRE_THAT(e.det, WithinRel(87.813, 1e-4));
    // B-21: poles -8.094 +/- 4.710i published.
    REQUIRE(e.oscillatory());
    REQUIRE_THAT(e.re1, WithinRel(-8.0951, 1e-4));
    REQUIRE_THAT(std::abs(e.im1), WithinRel(4.7204, 1e-3));
    // B-22 / B-23.
    REQUIRE_THAT(e.omega_n, WithinRel(9.3709, 1e-4));
    REQUIRE_THAT(e.zeta, WithinRel(0.86386, 1e-4));
    REQUIRE(e.stable());

    // §8.6: eigenvalues cross-checked against trace and determinant to 1e-12.
    REQUIRE_THAT(e.re1 + e.re2, WithinRel(e.trace, 1e-12));
    REQUIRE_THAT(e.re1 * e.re2 - e.im1 * e.im2, WithinRel(e.det, 1e-12));

    // B-24: r/delta numerator 0.84 s + 7.57 per deg published.
    const auto tf = yaw_rate_transfer_function(v, C, V);
    REQUIRE_THAT(tf.n1 * units::DEGREE, WithinRel(0.8410, 1e-3));
    REQUIRE_THAT(tf.n0 * units::DEGREE, WithinRel(7.5504, 1e-3));
    // B-25: DC gain of (8.28) equals (8.15). Verified: both 4.92645 s^-1.
    const auto m = handling_metrics(v, C);
    REQUIRE_THAT(tf.dc_gain(), WithinRel(yaw_rate_gain(v, m.K, V), 1e-14));
    REQUIRE_THAT(tf.dc_gain(), WithinRel(4.92645, 1e-5));
}

// ------------------------------------------------------------- §8.5 tangent speed
TEST_CASE("B-28: sideslip changes sign at the tangent speed", "[bicycle][acceptance]") {
    const core::Vehicle v = ch5();
    const auto C = fixtures::ch5_tires();
    const auto m = handling_metrics(v, C);
    REQUIRE(m.V_tan > 0.0);

    // §3.3: for a right-hand turn, beta is POSITIVE below the tangent speed and
    // NEGATIVE above it. The source's unconditional "v positive" is correct only
    // below V_tan, which is why the sign table carries the qualifier.
    const double delta = 0.02;  // positive steer => right-hand turn
    const auto below = steady_state(v, C, 0.5 * m.V_tan, delta);
    const auto above = steady_state(v, C, 1.5 * m.V_tan, delta);
    REQUIRE(below.ok());
    REQUIRE(above.ok());
    REQUIRE(below.value().beta_rad > 0.0);
    REQUIRE(above.value().beta_rad < 0.0);
    // v and beta always share a sign, since beta = atan(v/u) with u > 0.
    REQUIRE(below.value().v_mps > 0.0);
    REQUIRE(above.value().v_mps < 0.0);
    // The rest of the §3.3 table for the same run.
    REQUIRE(below.value().r_rad_per_s > 0.0);
    REQUIRE(below.value().A_Y_mps2 > 0.0);

    // At the tangent speed itself beta passes through zero.
    const auto at = steady_state(v, C, m.V_tan, delta);
    REQUIRE(at.ok());
    REQUIRE_THAT(at.value().beta_rad, WithinAbs(0.0, 1e-12));
}

// ----------------------------------------------------- §8.6 stability equivalence
TEST_CASE("B-31: det A and 1 + K V^2 are sign-equivalent", "[bicycle][invariant]") {
    // §8.6: they differ only by a positive speed-dependent factor, so their sign
    // change brackets V_crit. This links the transient and steady-state code paths,
    // which is why it is normative.
    const auto rv = core::resolve(fixtures::ch5_os_input());
    REQUIRE(rv.ok());
    const core::Vehicle v = rv.value();
    const auto C = fixtures::ch5_tires();
    const auto m = handling_metrics(v, C);

    REQUIRE(m.K < 0.0);           // oversteering
    REQUIRE(m.V_crit.has_value());

    for (double V_fts : {100.0, 170.0}) {
        const double V = V_fts * units::FOOT;
        const auto e = eigenstructure(v, C, V);
        const double one_plus_KV2 = 1.0 + m.K * V * V;
        INFO("V = " << V_fts << " ft/s");
        REQUIRE(std::signbit(e.det) == std::signbit(one_plus_KV2));
    }
    // 100 ft/s is below V_crit and 170 ft/s above it, so the pair brackets it.
    REQUIRE(*m.V_crit / units::FOOT > 100.0);
    REQUIRE(*m.V_crit / units::FOOT < 170.0);

    // Sweeping the sign change must land on V_crit itself.
    const auto e_lo = eigenstructure(v, C, *m.V_crit * 0.999);
    const auto e_hi = eigenstructure(v, C, *m.V_crit * 1.001);
    REQUIRE(e_lo.det > 0.0);
    REQUIRE(e_hi.det < 0.0);
    REQUIRE(e_lo.stable());
    REQUIRE_FALSE(e_hi.stable());
}

TEST_CASE("E-SS-030: at or above the critical speed", "[bicycle][validation]") {
    // §8.5: the solver returns a structured error rather than a huge number.
    const auto rv = core::resolve(fixtures::ch5_os_input());
    REQUIRE(rv.ok());
    const auto m = handling_metrics(rv.value(), fixtures::ch5_tires());
    const auto r = steady_state(rv.value(), fixtures::ch5_tires(), *m.V_crit, 0.02);
    REQUIRE_FALSE(r.ok());
    REQUIRE(r.error().code == "E-SS-030");
}
