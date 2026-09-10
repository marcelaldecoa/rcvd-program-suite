// Spec §6.4-6.9 (models L3, L4, L5, friction, ellipse, relaxation) and the
// §6.10 validation vectors TV-1 ... TV-8.
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <vector>

#include "rcvd/tire/conformance.hpp"
#include "rcvd/tire/friction.hpp"
#include "rcvd/tire/magic_formula.hpp"
#include "rcvd/tire/radt.hpp"
#include "rcvd/tire/relaxation.hpp"
#include "rcvd/tire/table.hpp"
#include "rcvd/units/convert.hpp"

using namespace rcvd;
using namespace rcvd::tire;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {
const double LBF = units::POUND_FORCE;
const double DEG = units::DEGREE;
const double kLbfDeg = units::LBF_PER_DEG_TO_N_PER_RAD;

// §6.10 DATASET A: the tabulated master curve of [R280] Ch. 14 Problem 1, whose
// peak is F_bar = 1.000 at alpha_bar = 3.0. §6.10 is emphatic that datasets A and
// B are DIFFERENT TIRES and must live in separate fixtures — a single fixture
// cannot satisfy both TV-4 and TV-5, and a "fix" that made it appear to would be
// a data error.
RadtTireConfig dataset_a() {
    RadtTireConfig c;
    c.id = "dataset_A";
    // A saturating master curve rising to 1.000 at alpha_bar = 3.
    c.alpha_bar = {0.0, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 2.0, 2.5, 3.0};
    c.F_bar = {0.0, 0.2474, 0.4650, 0.6293, 0.7631, 0.8529, 0.9130, 0.9756, 0.9950, 1.0000};
    c.Fz_N = {500.0 * LBF, 1000.0 * LBF, 1500.0 * LBF};
    c.C_alpha_N_per_rad = {-140.0 * kLbfDeg, -240.0 * kLbfDeg, -310.0 * kLbfDeg};
    c.mu = {1.05, 0.965, 0.90};
    c.trail_m = {1.2 * units::INCH, 1.1 * units::INCH, 1.0 * units::INCH};
    return c;
}

// A well-behaved tabular tire for the L3 checks: odd by mirroring, saturating.
TableTireConfig table_fixture() {
    TableTireConfig c;
    c.id = "table";
    c.alpha_rad = {0.0, 1.0 * DEG, 2.0 * DEG, 4.0 * DEG, 6.0 * DEG, 10.0 * DEG};
    c.Fz_N = {500.0 * LBF, 1000.0 * LBF, 1500.0 * LBF};
    // Fy negative for positive alpha (T3), saturating, load-sensitive.
    const std::vector<std::vector<double>> fy = {
        {0.0, 0.0, 0.0},
        {-140.0, -240.0, -310.0},
        {-270.0, -465.0, -600.0},
        {-455.0, -790.0, -1030.0},
        {-520.0, -915.0, -1210.0},
        {-540.0, -960.0, -1290.0},
    };
    for (const auto& row : fy) {
        std::vector<double> si;
        for (double v : row) si.push_back(v * LBF);
        c.Fy_N.push_back(si);
    }
    c.pneumatic_trail_m = 1.1 * units::INCH;
    return c;
}
}  // namespace

// ---------------------------------------------------------------- §6.4 model L3
TEST_CASE("L3: tabular tire interpolates its own data", "[tire][L3]") {
    const auto r = TableTire::create(table_fixture());
    REQUIRE(r.ok());
    const TableTire& t = r.value();

    // Nodes reproduce exactly.
    REQUIRE_THAT(t.evaluate({4.0 * DEG, 1000.0 * LBF}).Fy_N / LBF, WithinRel(-790.0, 1e-9));
    REQUIRE_THAT(t.evaluate({2.0 * DEG, 500.0 * LBF}).Fy_N / LBF, WithinRel(-270.0, 1e-9));
    // T2 at the origin, exactly.
    REQUIRE(t.evaluate({0.0, 1000.0 * LBF}).Fy_N == 0.0);
    REQUIRE(t.evaluate({0.0, 1000.0 * LBF}).Mz_Nm == 0.0);
    // §6.4: constant extrapolation past the tabulated alpha, with saturated set.
    const auto beyond = t.evaluate({20.0 * DEG, 1000.0 * LBF});
    REQUIRE(beyond.saturated);
    REQUIRE_THAT(beyond.Fy_N / LBF, WithinRel(-960.0, 1e-9));
    // D-2: PCHIP is C1, so unlike a linear-interpolated table there are no slope
    // breaks to declare.
    REQUIRE(t.is_C1());
    REQUIRE(t.slope_break_points().empty());
    REQUIRE_FALSE(t.interpolation_knots().empty());
}

TEST_CASE("L3: shape-preserving, so no overshoot near the force peak", "[tire][L3]") {
    // The reason §13.5 mandates Fritsch-Carlson over a natural cubic spline: a
    // saturating tire's data would make a natural spline overshoot the peak.
    const auto r = TableTire::create(table_fixture());
    REQUIRE(r.ok());
    double worst = 0.0;
    for (double a = 0.0; a <= 10.0 * DEG; a += 0.01 * DEG) {
        worst = std::max(worst, std::abs(r.value().evaluate({a, 1000.0 * LBF}).Fy_N));
    }
    // The tabulated maximum at this load is 960 lbf; the interpolant must not
    // exceed it.
    REQUIRE(worst / LBF <= 960.0 + 1e-6);
}

TEST_CASE("L3: E-TIRE-020 territory is reported, not silently extrapolated",
          "[tire][L3][validation]") {
    const auto r = TableTire::create(table_fixture());
    REQUIRE(r.ok());
    // §6.4 permits linear extrapolation up to 25 % beyond the last Fz node.
    REQUIRE_FALSE(r.value().beyond_Fz_range(1500.0 * LBF * 1.2));
    REQUIRE(r.value().beyond_Fz_range(1500.0 * LBF * 1.3));
    REQUIRE(r.value().valid_Fz_N().hi > 1500.0 * LBF);
}

TEST_CASE("L3: a half-table that is not zero at alpha = 0 is rejected",
          "[tire][L3][validation]") {
    TableTireConfig bad = table_fixture();
    bad.Fy_N[0][1] = 5.0 * LBF;   // T2 violated in the data
    REQUIRE(TableTire::create(bad).error().code == "E-TIRE-003");

    // And a mirrored table must not also span negative alpha.
    TableTireConfig both = table_fixture();
    both.alpha_rad.insert(both.alpha_rad.begin(), -1.0 * DEG);
    both.Fy_N.insert(both.Fy_N.begin(), {240.0 * LBF, 240.0 * LBF, 240.0 * LBF});
    REQUIRE(TableTire::create(both).error().code == "E-IO-003");
}

// ---------------------------------------------------------------- §6.5 model L4
TEST_CASE("TV-3: Radt normalisation on dataset A", "[tire][L4][acceptance]") {
    // C_alpha = -240 lb/deg, Fz = 1000 lb, mu = 0.965, alpha = +4 deg gives
    // alpha_bar = -0.996 (the source quotes the magnitude 0.996).
    const double Ca = -240.0 * kLbfDeg;
    const double Fz = 1000.0 * LBF;
    const double mu = 0.965;
    const double abar = Ca * std::tan(4.0 * DEG) / (mu * Fz);
    REQUIRE_THAT(abar, WithinRel(-0.996, 1e-3));
    REQUIRE_THAT(std::abs(abar), WithinRel(0.996, 1e-3));
}

TEST_CASE("TV-4: Radt expansion on dataset A", "[tire][L4][acceptance]") {
    const auto r = RadtTire::create(dataset_a());
    REQUIRE(r.ok());
    const RadtTire& t = r.value();
    const double Fz = 1000.0 * LBF;

    // F_y = -736.4 lb at the TV-3 operating point. The master curve is tabulated,
    // so this is the interpolant's answer, checked to the precision the data
    // supports.
    const double Fy = t.evaluate({4.0 * DEG, Fz}).Fy_N;
    REQUIRE_THAT(Fy / LBF, WithinRel(-736.4, 5e-3));

    // Master-curve peak at alpha_bar = 3.0 gives alpha = 11.89 deg, F_y = -965.0 lb.
    REQUIRE_THAT(t.peak_alpha_rad(Fz) / DEG, WithinRel(11.89, 2e-3));
    REQUIRE_THAT(std::abs(t.peak_Fy_N(Fz)) / LBF, WithinRel(965.0, 2e-3));
    // mu*Fz IS the peak, by the definition of the normalisation.
    REQUIRE_THAT(std::abs(t.peak_Fy_N(Fz)) / LBF, WithinRel(0.965 * 1000.0, 1e-9));
}

TEST_CASE("L4: the Fz guard is applied before dividing, not after", "[tire][L4]") {
    // §6.5: mu*Fz is the denominator of (6.4). Below the floor the model returns
    // zero force per T4 WITHOUT evaluating (6.4).
    const auto r = RadtTire::create(dataset_a());
    REQUIRE(r.ok());
    const auto zero = r.value().evaluate({0.05, 0.0});
    REQUIRE(zero.Fy_N == 0.0);
    REQUIRE(zero.Mz_Nm == 0.0);
    REQUIRE(std::isfinite(zero.trail_m));
    // A tiny but non-zero load must not produce an enormous alpha_bar either.
    const auto tiny = r.value().evaluate({0.05, 0.5});
    REQUIRE(std::isfinite(tiny.Fy_N));
    REQUIRE(tiny.Fy_N == 0.0);
}

TEST_CASE("L4: T5's friction bound holds at every load", "[tire][L4]") {
    const auto r = RadtTire::create(dataset_a());
    REQUIRE(r.ok());
    REQUIRE(r.value().has_mu());
    for (double Fz : {500.0 * LBF, 1000.0 * LBF, 1500.0 * LBF}) {
        for (double a = -0.2; a <= 0.2; a += 0.005) {
            const auto o = r.value().evaluate({a, Fz});
            REQUIRE(o.mu_peak.has_value());
            REQUIRE(std::abs(o.Fy_N) <= *o.mu_peak * Fz * (1.0 + 1e-9));
        }
    }
}

// ---------------------------------------------------------------- §6.6 model L5
TEST_CASE("TV-5: Magic Formula fit of the normalised curve of dataset B",
          "[tire][L5][acceptance]") {
    // B = 0.714, C = 1.4, D = 1.0, E = -0.45, BCD = 0.9996.
    //
    // §6.6: the fitter SHALL impose B*C*D = 1 as a CONSTRAINT with a tolerance of
    // 1e-3 rather than exact equality, because published coefficient sets are
    // rounded — re-solving B from C and D to make the product exactly 1 changes B
    // in its fourth significant figure.
    MagicFormulaCoefficients c{0.714, 1.4, 1.0, -0.45, 0.0, 0.0};
    REQUIRE_THAT(c.BCD(), WithinRel(0.9996, 1e-6));
    REQUIRE_THAT(c.BCD(), WithinAbs(1.0, 1e-3));

    MagicFormulaConfig cfg;
    cfg.id = "dataset_B_normalised";
    cfg.normalised = true;
    cfg.Fz_N = {1.0};
    cfg.coefficients = {c};
    const auto t = MagicFormulaTire::create(cfg);
    REQUIRE(t.ok());

    // The normalised curve passes through the origin with unit slope, which is
    // what BCD = 1 means.
    REQUIRE(c.evaluate(0.0) == 0.0);
    REQUIRE_THAT(c.slope(0.0), WithinRel(c.BCD(), 1e-9));

    // §6.10: dataset B's peak is near alpha_bar = 2.46, NOT dataset A's 3.0 — they
    // are different tires and the suite must not cross-apply them.
    double peak_abar = 0.0, peak = 0.0;
    for (double x = 0.0; x < 6.0; x += 0.001) {
        if (c.evaluate(x) > peak) {
            peak = c.evaluate(x);
            peak_abar = x;
        }
    }
    REQUIRE_THAT(peak_abar, WithinRel(2.46, 5e-2));
    REQUIRE_THAT(peak, WithinRel(1.0, 1e-3));

    // Evaluating dataset B at dataset A's TV-3 operating point gives F_bar = 0.786,
    // which does NOT match TV-4 — correctly, because it is a different tire.
    REQUIRE_THAT(c.evaluate(0.996), WithinRel(0.786, 2e-2));
}

TEST_CASE("§5.2: the MF sign check runs in both directions", "[tire][L5][validation]") {
    // A dimensional block must have B*C*D < 0 (D is the peak lateral force in the
    // SAE tire frame, hence negative); a normalised one must have B*C*D > 0.
    MagicFormulaConfig dim;
    dim.id = "dimensional";
    dim.normalised = false;
    dim.Fz_N = {1000.0 * LBF};
    dim.coefficients = {{0.25, 1.4, +900.0 * LBF, -0.45, 0.0, 0.0}};   // D positive: wrong
    REQUIRE(MagicFormulaTire::create(dim).error().code == "E-TIRE-003");

    dim.coefficients = {{0.25, 1.4, -900.0 * LBF, -0.45, 0.0, 0.0}};   // D negative: right
    REQUIRE(MagicFormulaTire::create(dim).ok());

    MagicFormulaConfig norm;
    norm.id = "normalised";
    norm.normalised = true;
    norm.Fz_N = {1.0};
    norm.coefficients = {{0.714, 1.4, -1.0, -0.45, 0.0, 0.0}};   // D negative: wrong here
    REQUIRE(MagicFormulaTire::create(norm).error().code == "E-TIRE-003");

    // §6.6 box constraints.
    norm.coefficients = {{-0.714, 1.4, 1.0, -0.45, 0.0, 0.0}};
    REQUIRE(MagicFormulaTire::create(norm).error().code == "E-TIRE-003");   // B <= 0
    norm.coefficients = {{0.714, 3.5, 1.0, -0.45, 0.0, 0.0}};
    REQUIRE(MagicFormulaTire::create(norm).error().code == "E-TIRE-003");   // C out of [1,3]
    norm.coefficients = {{0.714, 1.4, 1.0, 1.5, 0.0, 0.0}};
    REQUIRE(MagicFormulaTire::create(norm).error().code == "E-TIRE-003");   // E > 1
}

TEST_CASE("L5: BCD is the cornering stiffness, and load interpolation is PCHIP",
          "[tire][L5]") {
    MagicFormulaConfig cfg;
    cfg.id = "mf_loads";
    cfg.normalised = false;
    cfg.Fz_N = {700.0 * LBF, 1300.0 * LBF, 1900.0 * LBF, 2500.0 * LBF};
    cfg.coefficients = {
        {0.30, 1.35, -700.0 * LBF, -0.4, 0.0, 0.0},
        {0.26, 1.38, -1250.0 * LBF, -0.45, 0.0, 0.0},
        {0.23, 1.40, -1730.0 * LBF, -0.5, 0.0, 0.0},
        {0.21, 1.42, -2150.0 * LBF, -0.55, 0.0, 0.0},
    };
    const auto r = MagicFormulaTire::create(cfg);
    REQUIRE(r.ok());
    const MagicFormulaTire& t = r.value();

    // At a tabulated load the coefficients are the tabulated ones.
    REQUIRE_THAT(t.at_load(1300.0 * LBF).B, WithinRel(0.26, 1e-9));
    // BCD is the slope at the origin, i.e. the cornering stiffness.
    const auto c = t.at_load(1300.0 * LBF);
    REQUIRE_THAT(t.dFy_dalpha({0.0, 1300.0 * LBF}), WithinRel(c.BCD(), 1e-9));
    REQUIRE(c.BCD() < 0.0);
    // Between loads the interpolation is monotone, so no spurious peak appears.
    double prev = 0.0;
    for (double Fz = 700.0 * LBF; Fz <= 2500.0 * LBF; Fz += 10.0 * LBF) {
        const double peak = std::abs(t.at_load(Fz).D);
        REQUIRE(peak >= prev - 1e-6);
        prev = peak;
    }
}

// ------------------------------------------------------- §6.1 shared conformance
TEST_CASE("L3, L4 and L5 all satisfy the §6.1 contract", "[tire][conformance]") {
    // The point of a SHARED conformance test: a new model earns T1-T8 by
    // construction rather than by someone remembering to write its checks.
    const auto l3 = TableTire::create(table_fixture());
    REQUIRE(l3.ok());
    const auto l4 = RadtTire::create(dataset_a());
    REQUIRE(l4.ok());

    MagicFormulaConfig mf;
    mf.id = "mf";
    mf.normalised = false;
    mf.Fz_N = {700.0 * LBF, 1300.0 * LBF, 1900.0 * LBF};
    mf.coefficients = {{0.30, 1.35, -700.0 * LBF, -0.4, 0.0, 0.0},
                       {0.26, 1.38, -1250.0 * LBF, -0.45, 0.0, 0.0},
                       {0.23, 1.40, -1730.0 * LBF, -0.5, 0.0, 0.0}};
    const auto l5 = MagicFormulaTire::create(mf);
    REQUIRE(l5.ok());

    struct Case { const char* name; const TireModel* m; ConformanceConfig cfg; };
    ConformanceConfig table_cfg;
    table_cfg.loads_N = {500.0 * LBF, 1000.0 * LBF, 1500.0 * LBF};
    ConformanceConfig mf_cfg;
    mf_cfg.loads_N = {700.0 * LBF, 1300.0 * LBF, 1900.0 * LBF};

    for (const Case& c : {Case{"L3 table", &l3.value(), table_cfg},
                          Case{"L4 radt", &l4.value(), table_cfg},
                          Case{"L5 magic formula", &l5.value(), mf_cfg}}) {
        INFO("model " << c.name);
        const auto rep = check_contract(*c.m, c.cfg);
        for (const auto& f : rep.failures) {
            INFO(f.check << ": " << f.detail);
            REQUIRE(f.check.empty());
        }
        REQUIRE(rep.ok());
    }
}

// ------------------------------------------------------------ §6.7 / §6.8
TEST_CASE("(6.9): friction falls with load, and the clamp is reported", "[tire][friction]") {
    // TV-2: mu = Fy_peak/Fz decreases with load. This is what makes lateral load
    // transfer reduce an axle's capability, and therefore what makes roll-stiffness
    // distribution a tuning parameter at all.
    FrictionModel f{1.22, 2.4e-4 / LBF, 0.1};
    REQUIRE_THAT(f.mu(1000.0 * LBF), WithinRel(1.22 - 0.24, 1e-9));
    REQUIRE(f.mu(2000.0 * LBF) < f.mu(1000.0 * LBF));
    REQUIRE_FALSE(f.clamped_at(1000.0 * LBF));

    // Rule D1: the clamp is reported rather than silent.
    REQUIRE(f.mu(1e6 * LBF) == 0.1);
    REQUIRE(f.clamped_at(1e6 * LBF));
    REQUIRE(f.dmu_dFz(1e6 * LBF) == 0.0);
}

TEST_CASE("TV-8: the friction ellipse, with its mandatory guard", "[tire][friction]") {
    FrictionEllipse e{1.0, 1.0};
    const double Fz = 1000.0 * LBF;
    // F_x = sqrt((mu Fz)^2 - Fy^2), zero at the lateral peak.
    REQUIRE_THAT(e.longitudinal_capacity(0.0, Fz), WithinRel(Fz, 1e-12));
    REQUIRE_THAT(e.longitudinal_capacity(0.6 * Fz, Fz), WithinRel(0.8 * Fz, 1e-12));
    REQUIRE_THAT(e.longitudinal_capacity(Fz, Fz), WithinAbs(0.0, 1e-9));

    // §6.8: the max(0, .) guard is MANDATORY. Rounding can drive the radicand
    // slightly negative at the lateral limit, and an unguarded sqrt produces a NaN
    // that propagates silently through an entire time history.
    bool clamped = false;
    const double past = e.longitudinal_capacity(1.05 * Fz, Fz, &clamped);
    REQUIRE(std::isfinite(past));    // NOT NaN
    REQUIRE(past == 0.0);
    REQUIRE(clamped);                // EV-CLAMP-ELLIPSE

    REQUIRE(e.within(0.5 * Fz, 0.5 * Fz, Fz));
    REQUIRE_FALSE(e.within(0.9 * Fz, 0.9 * Fz, Fz));
    // A lifted wheel carries no force at all.
    REQUIRE(e.longitudinal_capacity(0.0, 0.0) == 0.0);
}

// ---------------------------------------------------------------- §6.9
TEST_CASE("TV-6: the relaxation time constant is sigma/V", "[tire][relaxation]") {
    // sigma = 6 ft, V = 10 mph => tau = 0.409 s.
    const double sigma = 6.0 * units::FOOT;
    const double V = 10.0 * units::MPH;
    REQUIRE_THAT(relaxation_tau_s(sigma, V), WithinRel(0.409, 1e-3));

    // §6.9's own illustration: at 100 m/s with sigma = 0.3 m, tau = 3 ms.
    REQUIRE_THAT(relaxation_tau_s(0.3, 100.0), WithinRel(0.003, 1e-9));
    // And the ratio the manifest reports: negligible at 30 m/s, dominant at 5.
    REQUIRE(tau_ratio(0.5, 30.0, 0.1) < 0.2);
    REQUIRE(tau_ratio(0.5, 5.0, 0.1) >= 1.0);
}

TEST_CASE("§6.9: step and ramp responses, and the integrator agrees with them",
          "[tire][relaxation]") {
    const double tau = 0.409;
    // 63.2 % at tau, 98.2 % at 4 tau.
    REQUIRE_THAT(step_response(1.0, tau, tau), WithinRel(0.632, 1e-3));
    REQUIRE_THAT(step_response(1.0, 4.0 * tau, tau), WithinRel(0.982, 1e-3));

    // (6.16): the ramp response lags the quasi-static line C_alpha*A*t by exactly
    // A*tau*C_alpha at large t — a pure time delay of tau.
    //
    // The lag is SIGNED and follows C_alpha, which is negative (§3.3): as
    // exp(-t/tau) dies, Fy -> A*C_alpha*t - A*tau*C_alpha, so the difference
    // quasi_static - lagged is +A*tau*C_alpha, itself negative. The lagged force
    // is smaller in MAGNITUDE than the quasi-static line, which is what lagging
    // means for a negative-stiffness tire.
    const double A = 0.1, C = -1000.0, t = 20.0 * tau;
    const double lagged = ramp_response(A, C, t, tau);
    const double quasi_static = C * A * t;
    REQUIRE_THAT(quasi_static - lagged, WithinRel(A * tau * C, 1e-6));
    REQUIRE(std::abs(lagged) < std::abs(quasi_static));
    // Equivalently: the lag is a pure time delay of tau.
    REQUIRE_THAT(lagged, WithinRel(C * A * (t - tau), 1e-6));

    // The filter reproduces the closed-form step response.
    RelaxationConfig cfg;
    cfg.enabled = true;
    cfg.sigma_m = 6.0 * units::FOOT;
    RelaxationFilter f(cfg);
    const double V = 10.0 * units::MPH, dt = 1e-4;
    double time = 0.0;
    while (time < tau) {
        f.step(1.0, V, dt);
        time += dt;
    }
    REQUIRE_THAT(f.value(), WithinRel(0.632, 2e-3));
}

TEST_CASE("§6.9: the speed guard bypasses and re-initialises", "[tire][relaxation]") {
    // tau = sigma/u is singular at u = 0. Below the floor the filter is bypassed
    // AND the state re-initialised to Fy_ss, so no transient is injected when the
    // vehicle re-accelerates — a filter left holding a stale value would produce
    // exactly that.
    RelaxationConfig cfg;
    cfg.enabled = true;
    cfg.sigma_m = 0.5;
    cfg.speed_floor_mps = 0.5;
    RelaxationFilter f(cfg);

    f.step(1000.0, 30.0, 0.05);
    REQUIRE(f.value() > 0.0);
    REQUIRE_FALSE(f.bypassed(30.0));

    // Drop below the floor: the output is the steady-state force immediately.
    REQUIRE(f.bypassed(0.1));
    REQUIRE(f.step(-500.0, 0.1, 0.05) == -500.0);
    REQUIRE(f.value() == -500.0);
    // Re-accelerating starts from there, with no injected transient.
    REQUIRE_THAT(f.step(-500.0, 30.0, 0.05), WithinRel(-500.0, 1e-12));
}

TEST_CASE("§6.9: disabled by default", "[tire][relaxation]") {
    // The default SHALL be disabled for every program; P2/P4/P5 make it
    // user-selectable.
    RelaxationFilter f;
    REQUIRE(f.bypassed(30.0));
    REQUIRE(f.step(1234.0, 30.0, 0.01) == 1234.0);
}
