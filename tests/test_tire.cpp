// Spec §6.1-6.3 and §6.7 — the TireModel contract, models L1 and L2.
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>

#include "rcvd/tire/conformance.hpp"
#include "rcvd/tire/linear.hpp"
#include "rcvd/units/convert.hpp"
#include "support/defective_tire.hpp"

using namespace rcvd;
using namespace rcvd::tire;
using Catch::Matchers::WithinRel;

namespace {
constexpr double kLbfDeg = units::LBF_PER_DEG_TO_N_PER_RAD;

LinearTireConfig ch5_front() {
    LinearTireConfig c;
    c.id = "linear_breakaway";
    c.C_alpha_N_per_rad = -640.0 * kLbfDeg;     // CH5 front axle
    c.pneumatic_trail_m = 1.1 * units::INCH;
    c.breakaway_slip_rad = 5.0 * units::DEGREE;  // [R280] Ch. 5 Prob. 5
    return c;
}
}  // namespace

// ------------------------------------------------------- construction (§14.1 D3)
TEST_CASE("E-TIRE-005: exactly one source of cornering stiffness", "[tire][validation]") {
    // §5.2: these are alternatives, not a base value plus a modifier. There is no
    // precedence rule, deliberately.
    LinearTireConfig both;
    both.C_alpha_N_per_rad = -1000.0;
    both.load_sensitivity = QuadraticStiffness{-0.4, 8e-5};
    auto r = LinearTire::create(both);
    REQUIRE_FALSE(r.ok());
    REQUIRE(r.error().code == "E-TIRE-005");

    LinearTireConfig neither;
    auto r2 = LinearTire::create(neither);
    REQUIRE_FALSE(r2.ok());
    REQUIRE(r2.error().code == "E-TIRE-005");
}

TEST_CASE("E-TIRE-003: cornering stiffness must be negative", "[tire][validation]") {
    LinearTireConfig c;
    c.C_alpha_N_per_rad = +640.0 * kLbfDeg;
    auto r = LinearTire::create(c);
    REQUIRE_FALSE(r.ok());
    REQUIRE(r.error().code == "E-TIRE-003");
}

TEST_CASE("E-TIRE-004: quadratic C_alpha must not change sign in range", "[tire][validation]") {
    // §5.2's coefficients in SI: c0 = -0.4 /deg, c1 = 0.8e-4 /(lbf*deg). The sign
    // change is at Fz = -c0/c1 = 5000 lbf.
    const double c0 = -0.4 / units::DEGREE;
    const double c1 = 0.8e-4 / (units::POUND_FORCE * units::DEGREE);

    LinearTireConfig c;
    c.load_sensitivity = QuadraticStiffness{c0, c1};
    c.Fz_max_operating_N = 30000.0;   // above the 22241 N sign change
    auto r = LinearTire::create(c);
    REQUIRE_FALSE(r.ok());
    REQUIRE(r.error().code == "E-TIRE-004");

    c.Fz_max_operating_N = 15000.0;   // below it
    auto ok = LinearTire::create(c);
    REQUIRE(ok.ok());
    // §6.7: |C_alpha| peaks at Fz* = -c0/(2 c1) = 2500 lbf; beyond it W-TIRE-101.
    const auto peak = ok.value().stiffness_peak_load_N();
    REQUIRE(peak.has_value());
    REQUIRE_THAT(*peak, WithinRel(2500.0 * units::POUND_FORCE, 1e-12));

    // E-TIRE-003 validates the INPUT coefficients, not the evaluated C_alpha(Fz).
    LinearTireConfig wrong;
    wrong.load_sensitivity = QuadraticStiffness{+0.4, 8e-5};
    REQUIRE(LinearTire::create(wrong).error().code == "E-TIRE-003");
}

TEST_CASE("E-TIRE-006: no friction block on a pure-stiffness model", "[tire][validation]") {
    LinearTireConfig c = ch5_front();
    c.document_has_friction_block = true;
    auto r = LinearTire::create(c);
    REQUIRE_FALSE(r.ok());
    REQUIRE(r.error().code == "E-TIRE-006");
}

// ------------------------------------------------------------------ model L1
TEST_CASE("L1 is (6.1) with (6.2) aligning moment", "[tire][L1]") {
    LinearTireConfig c;
    c.id = "linear";
    c.C_alpha_N_per_rad = -640.0 * kLbfDeg;
    c.pneumatic_trail_m = 1.1 * units::INCH;
    auto r = LinearTire::create(c);
    REQUIRE(r.ok());
    const LinearTire& t = r.value();

    REQUIRE_FALSE(t.has_mu());          // T5: pure stiffness
    REQUIRE(t.is_C1());
    REQUIRE(t.slope_break_points().empty());

    const auto o = t.evaluate({0.02, 5000.0});
    REQUIRE_THAT(o.Fy_N, WithinRel(-640.0 * kLbfDeg * 0.02, 1e-15));
    REQUIRE_THAT(o.Mz_Nm, WithinRel(-o.trail_m * o.Fy_N, 1e-15));
    REQUIRE_THAT(o.C_alpha_N_per_rad, WithinRel(-640.0 * kLbfDeg, 1e-15));
    REQUIRE_FALSE(o.mu_peak.has_value());
}

TEST_CASE("L1 honours T4 even though (6.1) never mentions Fz", "[tire][L1]") {
    // The contract binds the model, not the formula. This is the case a literal
    // transcription of (6.1) gets wrong.
    LinearTireConfig c;
    c.C_alpha_N_per_rad = -640.0 * kLbfDeg;
    c.pneumatic_trail_m = 1.1 * units::INCH;
    const auto t = LinearTire::create(c);
    REQUIRE(t.ok());

    const auto z = t.value().evaluate({0.02, 0.0});
    REQUIRE(z.Fy_N == 0.0);
    REQUIRE(z.Mz_Nm == 0.0);
    REQUIRE(std::isfinite(z.trail_m));   // T8
}

// ------------------------------------------------------------------ model L2
TEST_CASE("L2 saturates at the breakaway slip angle", "[tire][L2]") {
    const auto r = LinearTire::create(ch5_front());
    REQUIRE(r.ok());
    const LinearTire& t = r.value();

    REQUIRE_FALSE(t.has_mu());
    REQUIRE_FALSE(t.is_C1());
    REQUIRE(t.slope_break_points().size() == 2);
    REQUIRE(std::is_sorted(t.slope_break_points().begin(), t.slope_break_points().end()));

    const double abk = 5.0 * units::DEGREE;
    const auto sat = t.evaluate({2.0 * abk, 5000.0});
    REQUIRE(sat.saturated);
    // |C_alpha| * alpha_bk = 640 lbf/deg * 5 deg = 3200 lbf exactly.
    REQUIRE_THAT(std::abs(sat.Fy_N) / units::POUND_FORCE, WithinRel(3200.0, 1e-12));
    REQUIRE(sat.C_alpha_N_per_rad == 0.0);
}

TEST_CASE("L2 returns the one-sided slope from below at the kink", "[tire][L2]") {
    // §6.3 makes this a documented choice, and requires it to be tested.
    const auto r = LinearTire::create(ch5_front());
    REQUIRE(r.ok());
    const double abk = 5.0 * units::DEGREE;
    REQUIRE_THAT(r.value().dFy_dalpha({abk, 5000.0}), WithinRel(-640.0 * kLbfDeg, 1e-15));
    REQUIRE(r.value().dFy_dalpha({abk * 1.0001, 5000.0}) == 0.0);
}

// ------------------------------------------------------- the shared T1-T8 suite
TEST_CASE("L1 and L2 satisfy the §6.1 contract", "[tire][conformance]") {
    LinearTireConfig l1;
    l1.id = "linear";
    l1.C_alpha_N_per_rad = -640.0 * kLbfDeg;
    l1.pneumatic_trail_m = 1.1 * units::INCH;

    for (const LinearTireConfig& cfg : {l1, ch5_front()}) {
        const auto r = LinearTire::create(cfg);
        REQUIRE(r.ok());
        const auto rep = check_contract(r.value());
        for (const auto& f : rep.failures) {
            INFO(f.check << ": " << f.detail);
            REQUIRE(f.check.empty());  // reports which check failed, and why
        }
        REQUIRE(rep.ok());
    }
}

TEST_CASE("the conformance suite catches every injected defect", "[tire][conformance]") {
    // Appendix D fault injection. A conformance suite that cannot fail is
    // worthless, so each violation must be shown to fire its intended check.
    const double Ca = -640.0 * kLbfDeg;
    for (int i = static_cast<int>(Defect::T1_not_odd);
         i <= static_cast<int>(Defect::T8_nan_trail); ++i) {
        const Defect d = static_cast<Defect>(i);
        const DefectiveTire bad(Ca, d);
        const auto rep = check_contract(bad);
        INFO("defect " << defect_name(d) << ", expecting " << expected_check(d));
        REQUIRE_FALSE(rep.ok());
        const bool fired = std::any_of(
            rep.failures.begin(), rep.failures.end(),
            [&](const ConformanceReport::Failure& f) { return f.check == expected_check(d); });
        REQUIRE(fired);
    }
    // The control must pass, so the above is not merely "everything fails".
    const DefectiveTire clean(Ca, Defect::none);
    REQUIRE(check_contract(clean).ok());
}

// --------------------------------------------------------- acceptance linkage
TEST_CASE("B-13: CH5 skidpad limit through the tire model", "[tire][acceptance]") {
    // The understeering front axle saturates first, so the limit is
    // |C_F|*alpha_bk / W_F with W_F = 3700 * 0.57 = 2109 lbf.
    const auto t = LinearTire::create(ch5_front());
    REQUIRE(t.ok());
    const double Fy_lbf = std::abs(t.value().evaluate({0.2, 5000.0}).Fy_N) / units::POUND_FORCE;
    REQUIRE_THAT(Fy_lbf / (3700.0 * 0.57), WithinRel(1.5173, 1e-4));
}

TEST_CASE("§17.6 case a: MMM construction lines", "[tire][acceptance]") {
    // 60-slug vehicle, l = 10 ft, a = b = 5 ft, C = 420 lbf/deg, alpha_bk = 6 deg.
    LinearTireConfig c;
    c.id = "mmm";
    c.C_alpha_N_per_rad = -420.0 * kLbfDeg;
    c.breakaway_slip_rad = 6.0 * units::DEGREE;
    const auto t = LinearTire::create(c);
    REQUIRE(t.ok());

    const double Fy = std::abs(t.value().evaluate({0.5, 5000.0}).Fy_N);
    const double mg = 60.0 * units::SLUG * units::G_STANDARD;
    // Published +/-1.30 / +/-0.65; §17.6 sets this block's tolerance to 5e-3 with
    // the reason (source rounding to three significant figures) recorded.
    REQUIRE_THAT(Fy / mg, WithinRel(1.3054, 1e-4));
    REQUIRE_THAT((5.0 * units::FOOT) * Fy / (mg * 10.0 * units::FOOT), WithinRel(0.6527, 1e-4));
}
