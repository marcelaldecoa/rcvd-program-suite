// Spec §10 and the §17.6 acceptance vectors.
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

#include "rcvd/analysis/moment_method.hpp"
#include "rcvd/models/bicycle.hpp"
#include "rcvd/tire/linear.hpp"
#include "support/fixtures.hpp"

using namespace rcvd;
using namespace rcvd::analysis;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {
constexpr double LBF = units::POUND_FORCE;
constexpr double FT  = units::FOOT;
constexpr double G   = units::G_STANDARD;
const double kLbfDeg = units::LBF_PER_DEG_TO_N_PER_RAD;

// §17.6: 60-slug vehicle, l = 10 ft, linear-with-breakaway tires, alpha_bk = 6 deg,
// V = 30 mph. C_F and C_R are AXLE stiffnesses.
core::Vehicle mmm_vehicle(double a_ft, double b_ft) {
    core::VehicleInput in;
    in.name = "MMM";
    in.mass_kg = 60.0 * units::SLUG;
    in.a_m = a_ft * FT;
    in.b_m = b_ft * FT;
    in.cg_height_m = 18.0 * units::INCH;
    in.track_front_m = 60.0 * units::INCH;
    in.track_rear_m = 60.0 * units::INCH;
    in.steering_ratio = 20.0;
    auto r = core::resolve(in);
    REQUIRE(r.ok());
    return r.value();
}

// A per-WHEEL L2 tire whose axle pair sums to the given axle stiffness.
tire::LinearTire axle_tire(double axle_C_lbf_per_deg, double alpha_bk_deg = 6.0) {
    tire::LinearTireConfig c;
    c.id = "l2";
    c.C_alpha_N_per_rad = -0.5 * axle_C_lbf_per_deg * kLbfDeg;
    c.breakaway_slip_rad = alpha_bk_deg * units::DEGREE;
    c.valid_alpha_half_range_rad = 0.6;
    auto r = tire::LinearTire::create(c);
    REQUIRE(r.ok());
    return std::move(r).value();
}
}  // namespace

// ------------------------------------------------------- §17.6 construction lines
TEST_CASE("17.6: all twenty construction-line coordinates", "[mmm][acceptance]") {
    // §17.6 sets this block's tolerance to 5e-3, with the reason recorded: the
    // residual is dominated by the source's rounding to three significant figures,
    // not by the gravity constant.
    constexpr double kTol = 5e-3;

    struct Case {
        const char* name;
        double a, b;          // ft
        double C_F, C_R;      // lbf/deg, axle, magnitudes
        double front_AY, front_CN, rear_AY, rear_CN;
    };
    const Case cases[] = {
        {"a", 5, 5, 420, 420, 1.30, 0.65, 1.30, -0.65},
        {"b", 6, 4, 420, 420, 1.30, 0.78, 1.30, -0.52},
        {"c", 4, 6, 420, 420, 1.30, 0.52, 1.30, -0.78},
        {"d", 5, 5, 504, 336, 1.56, 0.78, 1.04, -0.52},
        {"e", 5, 5, 336, 504, 1.04, 0.52, 1.56, -0.78},
    };

    for (const Case& c : cases) {
        INFO("case " << c.name);
        const core::Vehicle v = mmm_vehicle(c.a, c.b);
        const auto lim = construction_limits(v, c.C_F * kLbfDeg, 6.0 * units::DEGREE,
                                             c.C_R * kLbfDeg, 6.0 * units::DEGREE);
        REQUIRE_THAT(lim.front_A_Y_bar, WithinRel(c.front_AY, kTol));
        REQUIRE_THAT(lim.front_C_N, WithinRel(c.front_CN, kTol));
        REQUIRE_THAT(lim.rear_A_Y_bar, WithinRel(c.rear_AY, kTol));
        REQUIRE_THAT(lim.rear_C_N, WithinRel(c.rear_CN, kTol));
    }

    // §17.6's worst case, quoted exactly: case a gives A_Y_bar = 1.3054 against a
    // published 1.30 with the mandated g = 9.80665.
    const core::Vehicle va = mmm_vehicle(5, 5);
    const auto la = construction_limits(va, 420.0 * kLbfDeg, 6.0 * units::DEGREE,
                                        420.0 * kLbfDeg, 6.0 * units::DEGREE);
    REQUIRE_THAT(la.front_A_Y_bar, WithinRel(1.3054, 1e-4));
    REQUIRE_THAT(la.front_A_Y_bar / 1.30 - 1.0, WithinAbs(4.2e-3, 2e-4));
}

TEST_CASE("10.4: the numerical sweep reproduces the analytic (10.5) limits",
          "[mmm][acceptance]") {
    // "The front and rear construction lines are obtained by the same sweep with
    // the other axle's force set to zero, which reduces to (10.5) for L2."
    const core::Vehicle v = mmm_vehicle(5, 5);
    auto tF = axle_tire(420.0);
    auto tR = axle_tire(420.0);

    MmmSweep s;
    s.V_mps = 30.0 * units::MPH;
    s.beta_min = -20.0 * units::DEGREE;
    s.beta_max = 20.0 * units::DEGREE;
    s.delta_min = -20.0 * units::DEGREE;
    s.delta_max = 20.0 * units::DEGREE;
    const MomentMethod mmm(v, tF, tR, s);

    const auto lim = construction_limits(v, 420.0 * kLbfDeg, 6.0 * units::DEGREE,
                                         420.0 * kLbfDeg, 6.0 * units::DEGREE);

    double best_front = 0.0, cn_front = 0.0;
    for (const MmmPoint& p : mmm.construction_line(true)) {
        if (std::abs(p.A_Y_bar) > std::abs(best_front)) {
            best_front = p.A_Y_bar;
            cn_front = p.C_N;
        }
    }
    REQUIRE_THAT(std::abs(best_front), WithinRel(lim.front_A_Y_bar, 1e-9));
    REQUIRE_THAT(std::abs(cn_front), WithinRel(std::abs(lim.front_C_N), 1e-9));
}

// ----------------------------------------------------------- §10.5 classification
TEST_CASE("17.6: limit character from the diagram", "[mmm][acceptance]") {
    // Character is decided by the sign of C_N at the diagram's extreme |A_Y| point,
    // relative to the trim line (C_N = 0): above => spin, on => drift, below => plow.
    struct Case { const char* name; double a, b, C_F, C_R; LimitCharacter expect; };
    const Case cases[] = {
        {"a", 5, 5, 420, 420, LimitCharacter::terminal_drift},
        {"b", 6, 4, 420, 420, LimitCharacter::terminal_spin},
        {"c", 4, 6, 420, 420, LimitCharacter::terminal_plow},
        {"d", 5, 5, 504, 336, LimitCharacter::terminal_spin},
        {"e", 5, 5, 336, 504, LimitCharacter::terminal_plow},
    };

    for (const Case& c : cases) {
        INFO("case " << c.name);
        const core::Vehicle v = mmm_vehicle(c.a, c.b);
        auto tF = axle_tire(c.C_F);
        auto tR = axle_tire(c.C_R);
        MmmSweep s;
        s.V_mps = 30.0 * units::MPH;
        s.beta_min = -25.0 * units::DEGREE;
        s.beta_max = 25.0 * units::DEGREE;
        s.delta_min = -25.0 * units::DEGREE;
        s.delta_max = 25.0 * units::DEGREE;
        s.beta_n = 121;
        s.delta_n = 121;
        const MomentMethod mmm(v, tF, tR, s);
        const auto m = metrics(mmm);
        INFO("limit A_Y_bar " << m.limit_A_Y_bar << ", C_N " << m.limit_C_N
                              << " => " << to_string(m.character));
        REQUIRE(m.character == c.expect);
    }
}

TEST_CASE("limit character agrees with the 8.5 understeer gradient", "[mmm][invariant]") {
    // Two independent routes to the same answer: the sign of K from (8.14) and the
    // sign of C_N at the diagram's limit point. They must agree for every case.
    struct Case { const char* name; double a, b, C_F, C_R; };
    const Case cases[] = {
        {"a", 5, 5, 420, 420}, {"b", 6, 4, 420, 420}, {"c", 4, 6, 420, 420},
        {"d", 5, 5, 504, 336}, {"e", 5, 5, 336, 504},
    };
    for (const Case& c : cases) {
        INFO("case " << c.name);
        const core::Vehicle v = mmm_vehicle(c.a, c.b);
        const models::AxleStiffness C{-c.C_F * kLbfDeg, -c.C_R * kLbfDeg};
        const double K = models::handling_metrics(v, C).K;

        auto tF = axle_tire(c.C_F);
        auto tR = axle_tire(c.C_R);
        MmmSweep s;
        s.V_mps = 30.0 * units::MPH;
        s.beta_min = -25.0 * units::DEGREE;
        s.beta_max = 25.0 * units::DEGREE;
        s.delta_min = -25.0 * units::DEGREE;
        s.delta_max = 25.0 * units::DEGREE;
        s.beta_n = 121;
        s.delta_n = 121;
        const auto m = metrics(MomentMethod(v, tF, tR, s));

        // K > 0 understeer -> plow; K < 0 oversteer -> spin; K == 0 -> drift.
        if (std::abs(K) < 1e-12) {
            REQUIRE(m.character == LimitCharacter::terminal_drift);
        } else if (K > 0.0) {
            REQUIRE(m.character == LimitCharacter::terminal_plow);
        } else {
            REQUIRE(m.character == LimitCharacter::terminal_spin);
        }
    }
}

// ------------------------------------------------------------- §10.5 stability index
TEST_CASE("(10.6) vs (10.7): stability index against its analytic value", "[mmm][invariant]") {
    // §10.5 requires the central-difference SI to be cross-checked against the
    // linear-range analytic value.
    const core::Vehicle v = mmm_vehicle(4, 6);   // understeering
    auto tF = axle_tire(420.0);
    auto tR = axle_tire(420.0);
    MmmSweep s;
    s.V_mps = 30.0 * units::MPH;
    s.beta_min = -6.0 * units::DEGREE;   // stay well inside the linear range
    s.beta_max = 6.0 * units::DEGREE;
    s.delta_min = -6.0 * units::DEGREE;
    s.delta_max = 6.0 * units::DEGREE;
    const auto m = metrics(MomentMethod(v, tF, tR, s));

    const double analytic = stability_index_linear(v, -420.0 * kLbfDeg, -420.0 * kLbfDeg);
    REQUIRE_THAT(m.stability_index, WithinRel(analytic, 1e-9));
    REQUIRE(m.stability_index < 0.0);   // §10.5: SI < 0 is stable

    // Control moment, analytic in the linear range: -a*C_F/(m g l).
    const double cm = -v.a * (-420.0 * kLbfDeg) / (v.m * v.g * v.l);
    REQUIRE_THAT(m.control_moment, WithinRel(cm, 1e-6));
}

TEST_CASE("10.3: the sweep uses r/u, not r/V", "[mmm]") {
    // (3.1) is normative on the body-frame longitudinal velocity, and the sweep
    // reaches +/-10 deg or more where cos(beta) differs from 1 by 1.5 %.
    const core::Vehicle v = mmm_vehicle(5, 5);
    auto tF = axle_tire(420.0);
    auto tR = axle_tire(420.0);
    MmmSweep s;
    s.V_mps = 30.0 * units::MPH;
    s.radius_m = 100.0 * FT;   // a constrained turn, so r != 0
    const MomentMethod mmm(v, tF, tR, s);

    const double beta = 12.0 * units::DEGREE;
    const auto p = mmm.evaluate(beta, 0.0);
    const double u = s.V_mps * std::cos(beta);
    const double r = s.V_mps / s.radius_m;
    REQUIRE_THAT(p.alpha_F, WithinRel(beta + v.a * r / u, 1e-12));
    REQUIRE_THAT(p.alpha_R, WithinRel(beta - v.b * r / u, 1e-12));
    // Using r/V instead would be wrong by cos(beta) — 2.2 % at 12 deg.
    REQUIRE(std::abs(v.a * r / u - v.a * r / s.V_mps) / std::abs(v.a * r / u) > 0.02);
}

// ---------------------------------------------------------------- §10.4 boundary
TEST_CASE("10.4: the boundary is the image of the rectangle's edge, not a hull",
          "[mmm]") {
    // "The true boundary is generally NON-CONVEX — the re-entrant corner is
    // precisely what distinguishes terminal plow from terminal spin — so any
    // support-function or convex-hull construction is forbidden."
    const core::Vehicle v = mmm_vehicle(4, 6);
    auto tF = axle_tire(420.0);
    auto tR = axle_tire(420.0);
    MmmSweep s;
    s.V_mps = 30.0 * units::MPH;
    s.beta_min = -25.0 * units::DEGREE;
    s.beta_max = 25.0 * units::DEGREE;
    s.delta_min = -25.0 * units::DEGREE;
    s.delta_max = 25.0 * units::DEGREE;
    const MomentMethod mmm(v, tF, tR, s);

    const auto poly = mmm.boundary(60);
    REQUIRE(poly.size() == 240);
    // It is a closed loop: the walk returns to where it started.
    REQUIRE_THAT(poly.front().beta_rad, WithinAbs(s.beta_min, 1e-12));

    // Demonstrate non-convexity: some boundary vertex lies strictly INSIDE the
    // convex hull of the others, so a support-function sweep would miss it. Test
    // it directly — a point is a hull vertex iff some direction maximises at it.
    int interior = 0;
    for (std::size_t i = 0; i < poly.size(); ++i) {
        bool is_extreme = false;
        for (int k = 0; k < 720 && !is_extreme; ++k) {
            const double th = k * units::PI / 360.0;
            const double cx = std::cos(th), cy = std::sin(th);
            const double mine = cx * poly[i].A_Y_bar + cy * poly[i].C_N;
            bool best = true;
            for (std::size_t j = 0; j < poly.size(); ++j) {
                if (cx * poly[j].A_Y_bar + cy * poly[j].C_N > mine + 1e-12) {
                    best = false;
                    break;
                }
            }
            if (best) is_extreme = true;
        }
        if (!is_extreme) ++interior;
    }
    INFO(interior << " of " << poly.size()
                  << " boundary points are NOT convex-hull extreme points");
    REQUIRE(interior > 0);
}

// ---------------------------------------------------------------- §10.5 trim line
TEST_CASE("10.5: the trim line is the C_N = 0 locus", "[mmm]") {
    const core::Vehicle v = mmm_vehicle(4, 6);
    auto tF = axle_tire(420.0);
    auto tR = axle_tire(420.0);
    MmmSweep s;
    s.V_mps = 30.0 * units::MPH;
    s.beta_min = -20.0 * units::DEGREE;
    s.beta_max = 20.0 * units::DEGREE;
    s.delta_min = -20.0 * units::DEGREE;
    s.delta_max = 20.0 * units::DEGREE;
    const MomentMethod mmm(v, tF, tR, s);

    const auto trim = mmm.trim_line();
    REQUIRE(trim.size() > 10);
    for (const MmmPoint& p : trim) {
        REQUIRE_THAT(p.C_N, WithinAbs(0.0, 1e-10));   // §10.5 rooting tolerance
    }
    // Trimmed lateral acceleration rises monotonically with steer until the tires
    // saturate, so the maximum is attained and reported with its (beta, delta).
    const auto m = metrics(mmm);
    REQUIRE(m.max_trimmed_A_Y_bar > 0.5);
    REQUIRE(std::abs(m.max_trimmed_delta) > 0.0);
}

// ------------------------------------------------------------------ §10.7 I-4/C-01
TEST_CASE("I-4 / C-01: P3 max trimmed A_Y matches P1's steady-state limit",
          "[mmm][crosscheck]") {
    // §10.7: the two SHALL agree to within 2 %. Larger disagreement indicates an
    // inconsistency between the two force paths.
    const core::Vehicle v = mmm_vehicle(4, 6);          // understeering
    const double C_F = 420.0, C_R = 420.0;
    auto tF = axle_tire(C_F);
    auto tR = axle_tire(C_R);

    MmmSweep s;
    s.V_mps = 30.0 * units::MPH;
    s.beta_min = -25.0 * units::DEGREE;
    s.beta_max = 25.0 * units::DEGREE;
    s.delta_min = -25.0 * units::DEGREE;
    s.delta_max = 25.0 * units::DEGREE;
    s.beta_n = 241;
    s.delta_n = 241;
    const auto m = metrics(MomentMethod(v, tF, tR, s));

    // P1's steady-state limit: the understeering front axle saturates first, so
    // A_Y = |C_F| * alpha_bk / W_F.
    const double Fy_front_max = C_F * LBF / units::DEGREE * (6.0 * units::DEGREE);
    const double p1_limit = Fy_front_max / v.W_F();

    INFO("P3 max trimmed " << m.max_trimmed_A_Y_bar << " g, P1 limit " << p1_limit << " g");
    REQUIRE_THAT(m.max_trimmed_A_Y_bar, WithinRel(p1_limit, 2e-2));
}

// -------------------------------------------------------------- §10.5 (10.10), M-04
TEST_CASE("M-04: understeer gradient from steering sensitivity", "[mmm][acceptance]") {
    // 60 mph, l = 8.5 ft, G_s = 20:1. §17.6 erratum E-3: only the 1.18 / 0.37
    // steering-sensitivity pair is consistent with the published understeer
    // gradients, so the acceptance vectors use those, not the quoted 1.30 / 0.33.
    const double l = 8.5 * FT, V = 60.0 * units::MPH, G_s = 20.0;

    struct Case { double SS, expect_deg_per_g; };
    const Case cases[] = {{1.18, 2.214}, {0.37, 11.490}};
    for (const Case& c : cases) {
        // SS is g per 100 deg SW, so d(delta_SW)/d(A_Y_bar) = 100/SS deg per g.
        const double d_sw_per_g = (100.0 / c.SS) * units::DEGREE;
        const double ug = understeer_gradient_per_g(d_sw_per_g, G_s, l, V, G);
        INFO("SS = " << c.SS);
        REQUIRE_THAT(ug / units::DEGREE, WithinRel(c.expect_deg_per_g, 1e-3));
    }

    // The E-3 alternative pair reproduces neither published gradient, which is the
    // evidence for choosing 1.18 / 0.37.
    for (double bad_ss : {1.30, 0.33}) {
        const double ug = understeer_gradient_per_g((100.0 / bad_ss) * units::DEGREE, G_s, l, V, G);
        REQUIRE(std::abs(ug / units::DEGREE - 2.214) > 0.1);
        REQUIRE(std::abs(ug / units::DEGREE - 11.490) > 0.1);
    }

    // E-4: the Ackermann gradient uses l*g/V^2, never the hard-coded 858.
    const double ackermann_deg_per_g = l * G / (V * V) / units::DEGREE;
    REQUIRE_THAT(ackermann_deg_per_g, WithinRel(2.0234, 1e-3));
    const double legacy = 858.0 * 8.5 / (60.0 * 60.0);
    REQUIRE_THAT(legacy / ackermann_deg_per_g - 1.0, WithinAbs(0.00120, 2e-4));
}

TEST_CASE("(10.8)/(10.9): sensitivity conventions", "[mmm]") {
    // SS is per 100 deg of STEERING WHEEL, in g.
    REQUIRE_THAT(steering_sensitivity(0.2, 20.0 * units::DEGREE), WithinRel(1.0, 1e-12));
    // SSl is deg of sideslip per g.
    REQUIRE_THAT(sideslip_sensitivity(-2.0 * units::DEGREE, 1.0), WithinRel(-2.0, 1e-12));
}
