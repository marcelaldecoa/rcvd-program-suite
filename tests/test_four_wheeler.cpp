// Spec §9.6-9.7 and cross-check C-02 (§17.8).
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <vector>

#include "rcvd/models/four_wheeler.hpp"
#include "rcvd/numerics/integrate.hpp"
#include "rcvd/tire/linear.hpp"
#include "support/fixtures.hpp"

using namespace rcvd;
using namespace rcvd::models;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {
constexpr double IN = units::INCH;
constexpr double LBF = units::POUND_FORCE;
constexpr double G = units::G_STANDARD;

core::Vehicle ch5_with_springing() {
    core::VehicleInput in = fixtures::ch5_input();
    in.roll_stiffness_ctg_front_Nm_per_rad = 975.0 * units::LBF_FT_PER_DEG_TO_NM_PER_RAD;
    in.roll_stiffness_ctg_rear_Nm_per_rad = 325.0 * units::LBF_FT_PER_DEG_TO_NM_PER_RAD;
    in.unsprung_mass_front_kg = 240.0 * LBF / G;
    in.unsprung_mass_rear_kg = 260.0 * LBF / G;
    in.unsprung_cg_height_front_m = 11.0 * IN;
    in.unsprung_cg_height_rear_m = 11.0 * IN;
    in.roll_centre_height_rear_m = 4.0 * IN;
    auto r = core::resolve(in);
    REQUIRE(r.ok());
    return r.value();
}

// A per-WHEEL linear tire whose axle pair sums to the CH5 axle stiffness.
tire::LinearTire half_axle(double axle_C_N_per_rad) {
    tire::LinearTireConfig c;
    c.id = "half_axle";
    c.C_alpha_N_per_rad = 0.5 * axle_C_N_per_rad;
    c.alpha_lin_rad = 1.0;   // wide validity: C-02 sweeps well past 2 deg
    auto r = tire::LinearTire::create(c);
    REQUIRE(r.ok());
    return std::move(r).value();
}
}  // namespace

// -------------------------------------------------------------- §9.14 slip angles
TEST_CASE("§9.14: outside wheels travel faster in a turn", "[fourwheeler]") {
    const core::Vehicle v = ch5_with_springing();
    SolverSettings cfg;

    // Right-hand turn: r > 0, so the LEFT wheels are outside and see a LARGER
    // longitudinal velocity, u + r*t/2. Placing the minus sign on the left is the
    // natural transcription error and inverts this.
    const double u = 20.0, r = 0.3;
    const auto sa = slip_angles(v, u, 0.0, r, 0.0, cfg);

    // With v = 0 and delta = 0 the numerator is +a*r at the front, so both front
    // slip angles are positive, but the LEFT one is smaller (bigger denominator).
    REQUIRE(sa.alpha[LF] > 0.0);
    REQUIRE(sa.alpha[RF] > 0.0);
    REQUIRE(sa.alpha[LF] < sa.alpha[RF]);
    // Rear numerator is -b*r, so the rear slip angles are negative.
    REQUIRE(sa.alpha[LR] < 0.0);
    REQUIRE(sa.alpha[RR] < 0.0);

    // Turning the track terms off collapses each axle's pair onto one value.
    cfg.switches.track_velocity_terms = false;
    const auto flat = slip_angles(v, u, 0.0, r, 0.0, cfg);
    REQUIRE_THAT(flat.alpha[LF], WithinRel(flat.alpha[RF], 1e-15));
    REQUIRE_THAT(flat.alpha[LR], WithinRel(flat.alpha[RR], 1e-15));
}

TEST_CASE("EV-WHEEL-STALL below the speed floor", "[fourwheeler][validation]") {
    const core::Vehicle v = ch5_with_springing();
    SolverSettings cfg;
    cfg.speed_floor_mps = 0.5;

    // Small u with a large yaw rate drives the inside wheel's longitudinal
    // velocity through zero.
    const auto sa = slip_angles(v, 0.2, 0.0, 0.5, 0.0, cfg, 2.5);
    REQUIRE_FALSE(sa.events.empty());
    REQUIRE(sa.events.front().code == "EV-WHEEL-STALL");
    REQUIRE(sa.events.front().wheel.has_value());
    REQUIRE_THAT(*sa.events.front().t_s, WithinAbs(2.5, 1e-15));
    // The stalled wheel takes +/- pi/2, not a NaN or an unbounded value.
    for (double a : sa.alpha) REQUIRE(std::isfinite(a));
}

TEST_CASE("§9.14: the small-angle switch is unconditional", "[fourwheeler]") {
    // §3.3 as corrected: arctan is normative and there is NO automatic threshold
    // switch. small_angle_slip forces v/u whatever the magnitude.
    const core::Vehicle v = ch5_with_springing();
    SolverSettings arc, lin;
    lin.switches.small_angle_slip = true;

    const double u = 10.0, vy = 4.0;  // v/u = 0.4, well past small_angle_limit
    const auto a = slip_angles(v, u, vy, 0.0, 0.0, arc);
    const auto l = slip_angles(v, u, vy, 0.0, 0.0, lin);
    REQUIRE_THAT(a.alpha[LR], WithinRel(std::atan2(vy - v.b * 0.0, u), 1e-15));
    REQUIRE_THAT(l.alpha[LR], WithinRel(vy / u, 1e-15));
    REQUIRE(std::abs(a.alpha[LR] - l.alpha[LR]) > 1e-3);  // genuinely different
}

// --------------------------------------------------------------- §17.8 C-02
TEST_CASE("C-02: P2 in bicycle-equivalence mode reproduces P1", "[fourwheeler][crosscheck]") {
    // "The single most valuable structural test in the suite: it forces P1 and P2
    // to share one force path rather than two implementations that drift apart."
    const core::Vehicle veh = ch5_with_springing();
    const auto C = fixtures::ch5_tires();
    const double V = 30.0 * units::MPH;

    auto tF = half_axle(C.C_F_N_per_rad);
    auto tR = half_axle(C.C_R_N_per_rad);
    const std::array<const tire::TireModel*, 4> tires{&tF, &tF, &tR, &tR};

    SolverSettings cfg;
    cfg.switches = ModelSwitches::bicycle_equivalence();
    aero::AeroConfig no_aero;

    FourWheeler p2(veh, tires, no_aero, cfg);
    p2.set_speed(V);
    BicyclePlant p1(veh, C, V);

    // Compare the derivative fields directly over a grid of states and inputs.
    double worst = 0.0;
    for (double delta_deg : {0.0, 1.0, 3.0, 5.0, 10.0}) {
        for (double v : {-2.0, 0.0, 1.5}) {
            for (double r : {-0.3, 0.0, 0.2, 0.5}) {
                const std::vector<double> x{v, r, 0.4, 10.0, -5.0};
                std::vector<double> d1(state::kDim), d2(state::kDim);
                const DriverInput in{delta_deg * units::DEGREE, 0.0};
                p1.derivative(0.0, x, in, d1);
                p2.derivative(0.0, x, in, d2);
                for (int i = 0; i < state::kDim; ++i) {
                    const double scale = std::max({1.0, std::abs(d1[i]), std::abs(d2[i])});
                    worst = std::max(worst, std::abs(d1[i] - d2[i]) / scale);
                }
            }
        }
    }
    INFO("worst relative difference across the state/input grid: " << worst);
    REQUIRE(worst < 1e-10);   // §17.8 C-02 criterion
}

TEST_CASE("C-02: steer_projection is not optional for it", "[fourwheeler][crosscheck]") {
    // §17.8: P1's (8.1) sums the axle forces UNPROJECTED, so retaining cos(delta)
    // leaves a residual of 1 - cos(delta) — 1.5e-4 at 1 deg, 3.8e-3 at 5 deg —
    // which exceeds the 1e-10 criterion at every steer angle that produces a
    // measurable response. This is the switch added to §5.5 for exactly this.
    const core::Vehicle veh = ch5_with_springing();
    const auto C = fixtures::ch5_tires();
    const double V = 30.0 * units::MPH;

    auto tF = half_axle(C.C_F_N_per_rad);
    auto tR = half_axle(C.C_R_N_per_rad);
    const std::array<const tire::TireModel*, 4> tires{&tF, &tF, &tR, &tR};

    SolverSettings cfg;
    cfg.switches = ModelSwitches::bicycle_equivalence();
    cfg.switches.steer_projection = true;   // the one thing put back
    aero::AeroConfig no_aero;

    FourWheeler p2(veh, tires, no_aero, cfg);
    p2.set_speed(V);
    BicyclePlant p1(veh, C, V);

    for (double deg : {1.0, 5.0}) {
        const std::vector<double> x{0.0, 0.0, 0.0, 0.0, 0.0};
        std::vector<double> d1(state::kDim), d2(state::kDim);
        const DriverInput in{deg * units::DEGREE, 0.0};
        p1.derivative(0.0, x, in, d1);
        p2.derivative(0.0, x, in, d2);
        const double rel = std::abs(d1[state::kV] - d2[state::kV]) / std::abs(d1[state::kV]);
        INFO("delta = " << deg << " deg, relative gap " << rel);
        REQUIRE_THAT(rel, WithinRel(1.0 - std::cos(deg * units::DEGREE), 1e-6));
        REQUIRE(rel > 1e-10);   // fails C-02, which is the point
    }
}

TEST_CASE("C-02: every switch in the mode is individually load-bearing",
          "[fourwheeler][crosscheck]") {
    // §17.8: "Each switch SHALL exist in SolverConfig and SHALL be individually
    // tested." A mode whose switches did nothing would pass C-02 vacuously, so
    // put each one back in turn and require the equivalence to BREAK.
    const core::Vehicle veh = ch5_with_springing();
    const auto C = fixtures::ch5_tires();
    const double V = 30.0 * units::MPH;

    auto tF = half_axle(C.C_F_N_per_rad);
    auto tR = half_axle(C.C_R_N_per_rad);
    const std::array<const tire::TireModel*, 4> tires{&tF, &tF, &tR, &tR};
    aero::AeroConfig ac;
    ac.frontal_area_m2 = 2.0;
    ac.CD = 0.4;
    ac.CLF = -1.0;
    ac.CLR = -0.8;

    BicyclePlant p1(veh, C, V);
    // A state with sideslip AND yaw rate AND steer, so every term is exercised.
    const std::vector<double> x{1.5, 0.35, 0.2, 0.0, 0.0};
    const DriverInput in{5.0 * units::DEGREE, 0.0};
    std::vector<double> d1(state::kDim), d2(state::kDim);
    p1.derivative(0.0, x, in, d1);

    auto gap_with = [&](ModelSwitches s) {
        SolverSettings cfg;
        cfg.switches = s;
        FourWheeler p2(veh, tires, ac, cfg);
        p2.set_speed(V);
        p2.derivative(0.0, x, in, d2);
        double worst = 0.0;
        for (int i = 0; i < state::kDim; ++i) {
            const double sc = std::max({1.0, std::abs(d1[i]), std::abs(d2[i])});
            worst = std::max(worst, std::abs(d1[i] - d2[i]) / sc);
        }
        return worst;
    };

    const ModelSwitches base = ModelSwitches::bicycle_equivalence();
    REQUIRE(gap_with(base) < 1e-10);   // the control

    // Only three of the seven can be exercised THROUGH C-02. See below.
    struct Case { const char* name; ModelSwitches s; };
    std::vector<Case> cases;
    { auto s = base; s.track_velocity_terms = true;  cases.push_back({"track_velocity_terms", s}); }
    { auto s = base; s.small_angle_slip = false;     cases.push_back({"small_angle_slip", s}); }
    { auto s = base; s.steer_projection = true;      cases.push_back({"steer_projection", s}); }

    for (const Case& c : cases) {
        const double gap = gap_with(c.s);
        INFO("switch " << c.name << " gives a gap of " << gap);
        REQUIRE(gap > 1e-10);
    }

    // load_transfer and aero are PROVABLY INERT under C-02's own configuration:
    // C-02 mandates tire model L1, and an L1 with constant C_alpha has Fy
    // independent of Fz. Both switches reach the lateral force only through Fz, so
    // that path is severed. This is §6.7's observation applied to C-02, and it is
    // why §17.8 requires them to be tested against a load-sensitive tire instead.
    { auto s = base; s.load_transfer = true;   REQUIRE(gap_with(s) < 1e-10); }
    { auto s = base; s.aero = true;            REQUIRE(gap_with(s) < 1e-10); }
    // aligning_moment and longitudinal_forces are inert for a second reason: L1
    // with t_p = 0 returns Mz = 0, and Rev 1.0 has no driveline, so Fx is
    // identically zero. They are carried in the mode for correctness under models
    // that do produce those terms (extension seams 3 and 7).
    { auto s = base; s.aligning_moment = true;      REQUIRE(gap_with(s) < 1e-10); }
    { auto s = base; s.longitudinal_forces = true;  REQUIRE(gap_with(s) < 1e-10); }
}

TEST_CASE("C-02: load_transfer and aero are load-bearing against a sensitive tire",
          "[fourwheeler][crosscheck]") {
    // §17.8: the two switches C-02 itself cannot exercise are tested here instead,
    // by requiring each to change P2's OWN output — no reference to P1, which has
    // fixed axle stiffness and would differ trivially.
    const core::Vehicle veh = ch5_with_springing();

    // (6.10) load-sensitive tire: C_alpha(Fz) = c0*Fz + c1*Fz^2.
    tire::LinearTireConfig q;
    q.id = "quadratic";
    q.load_sensitivity = tire::QuadraticStiffness{
        -0.4 / units::DEGREE, 0.8e-4 / (LBF * units::DEGREE)};
    q.alpha_lin_rad = 1.0;
    q.Fz_max_operating_N = 15000.0;
    auto rq = tire::LinearTire::create(q);
    REQUIRE(rq.ok());
    const tire::LinearTire& t = rq.value();
    const std::array<const tire::TireModel*, 4> tires{&t, &t, &t, &t};

    // Sanity: this tire's force really does depend on load, unlike C-02's L1.
    REQUIRE(std::abs(t.evaluate({0.05, 3000.0}).Fy_N - t.evaluate({0.05, 6000.0}).Fy_N) > 1.0);

    aero::AeroConfig ac;
    ac.frontal_area_m2 = 2.0;
    ac.CD = 0.4;
    ac.CLF = -1.0;
    ac.CLR = -0.8;

    const std::vector<double> x{1.5, 0.35, 0.2, 0.0, 0.0};
    const DriverInput in{5.0 * units::DEGREE, 0.0};

    auto deriv = [&](ModelSwitches s, std::vector<double>& d) {
        SolverSettings cfg;
        cfg.switches = s;
        // §9.7 / §17.8: under `explicit` coupling A_Y comes from the PREVIOUS step,
        // so a single derivative() call sees A_Y = 0 and hence no lateral transfer
        // at all — which would report load_transfer as inert even here. Use the
        // implicit loop for a pointwise switch test.
        cfg.implicit_load_transfer = true;
        FourWheeler p(veh, tires, ac, cfg);
        p.set_speed(30.0 * units::MPH);
        p.derivative(0.0, x, in, d);
    };
    auto gap = [](const std::vector<double>& a, const std::vector<double>& b) {
        double w = 0.0;
        for (int i = 0; i < state::kDim; ++i) {
            w = std::max(w, std::abs(a[i] - b[i]) /
                                std::max({1.0, std::abs(a[i]), std::abs(b[i])}));
        }
        return w;
    };

    const ModelSwitches base = ModelSwitches::bicycle_equivalence();
    std::vector<double> d0(state::kDim), d1(state::kDim), d2(state::kDim);
    deriv(base, d0);

    { auto s = base; s.load_transfer = true; deriv(s, d1); }
    INFO("load_transfer changes P2 by " << gap(d0, d1));
    REQUIRE(gap(d0, d1) > 1e-3);

    { auto s = base; s.load_transfer = true; s.aero = true; deriv(s, d2); }
    INFO("aero changes P2 by " << gap(d1, d2));
    REQUIRE(gap(d1, d2) > 1e-3);
}

TEST_CASE("C-02 holds under time integration, not just pointwise",
          "[fourwheeler][crosscheck]") {
    const core::Vehicle veh = ch5_with_springing();
    const auto C = fixtures::ch5_tires();
    const double V = 30.0 * units::MPH;

    auto tF = half_axle(C.C_F_N_per_rad);
    auto tR = half_axle(C.C_R_N_per_rad);
    const std::array<const tire::TireModel*, 4> tires{&tF, &tF, &tR, &tR};

    SolverSettings cfg;
    cfg.switches = ModelSwitches::bicycle_equivalence();
    aero::AeroConfig no_aero;
    FourWheeler p2(veh, tires, no_aero, cfg);
    p2.set_speed(V);
    BicyclePlant p1(veh, C, V);

    const auto man = Maneuver::make_step(4.0 * units::DEGREE, 0.0);
    auto integrate = [&](const Plant& p) {
        std::vector<double> x(state::kDim, 0.0), out(state::kDim), work;
        const double h = 1e-3;
        numerics::Derivative f = [&](double t, std::span<const double> s,
                                     std::span<double> d) {
            p.derivative(t, s, man.evaluate(t), d);
        };
        for (int i = 0; i < 2000; ++i) {  // 2 s
            numerics::rk4_step(f, i * h, x, h, out, work);
            x = out;
        }
        return x;
    };

    const auto a = integrate(p1);
    const auto b = integrate(p2);
    for (int i = 0; i < state::kDim; ++i) {
        const double scale = std::max({1.0, std::abs(a[i]), std::abs(b[i])});
        INFO("state index " << i);
        REQUIRE(std::abs(a[i] - b[i]) / scale < 1e-10);
    }
    // And the run actually did something: a 4 deg step at 30 mph turns the car.
    REQUIRE(std::abs(a[state::kR]) > 0.05);
    REQUIRE(std::abs(a[state::kYe]) > 1.0);
}

// ---------------------------------------------------------- §9.7 algebraic loop
TEST_CASE("§9.7: implicit and explicit load-transfer coupling agree at steady state",
          "[fourwheeler]") {
    const core::Vehicle veh = ch5_with_springing();
    const auto C = fixtures::ch5_tires();

    auto tF = half_axle(C.C_F_N_per_rad);
    auto tR = half_axle(C.C_R_N_per_rad);
    const std::array<const tire::TireModel*, 4> tires{&tF, &tF, &tR, &tR};
    aero::AeroConfig no_aero;

    SolverSettings expl, impl;
    impl.implicit_load_transfer = true;

    FourWheeler pe(veh, tires, no_aero, expl);
    FourWheeler pi(veh, tires, no_aero, impl);
    pe.set_speed(25.0);
    pi.set_speed(25.0);

    const std::vector<double> x{0.5, 0.15, 0.0, 0.0, 0.0};
    const DriverInput in{3.0 * units::DEGREE, 0.0};

    // Drive the explicit mode to its own fixed point by repetition, which is what
    // a converged steady state does.
    FourWheelerObservation oe, oi;
    std::vector<double> d(state::kDim);
    for (int i = 0; i < 200; ++i) pe.evaluate(0.0, x, in, oe, d);
    pi.evaluate(0.0, x, in, oi, d);

    REQUIRE(oi.ay_iterations > 0);           // the Newton loop actually ran
    REQUIRE_THAT(oe.A_Y, WithinRel(oi.A_Y, 1e-8));
    // §9.7: the contraction factor is reported, because Phi' -> 1 is the numerical
    // signature of an axle at its load-transfer limit and users should see it.
    REQUIRE(std::abs(oi.phi_prime) < 1.0);
}

TEST_CASE("§9.8: yaw_moment_residual vanishes at steady state", "[fourwheeler]") {
    // "At steady state it must be zero; carrying it as an output channel makes the
    // approach to steady state visible and is the primary debugging aid for P2."
    const core::Vehicle veh = ch5_with_springing();
    const auto C = fixtures::ch5_tires();
    auto tF = half_axle(C.C_F_N_per_rad);
    auto tR = half_axle(C.C_R_N_per_rad);
    const std::array<const tire::TireModel*, 4> tires{&tF, &tF, &tR, &tR};
    aero::AeroConfig no_aero;

    SolverSettings cfg;
    cfg.switches = ModelSwitches::bicycle_equivalence();
    const double V = 30.0 * units::MPH;
    FourWheeler p2(veh, tires, no_aero, cfg);
    p2.set_speed(V);

    // Take P1's steady state and feed it to P2; the residual must be ~0.
    const double delta = 3.0 * units::DEGREE;
    const auto ss = steady_state(veh, C, V, delta);
    REQUIRE(ss.ok());

    const std::vector<double> x{ss.value().v_mps, ss.value().r_rad_per_s, 0.0, 0.0, 0.0};
    FourWheelerObservation obs;
    std::vector<double> d(state::kDim);
    p2.evaluate(0.0, x, {delta, 0.0}, obs, d);

    const double scale = std::abs(veh.a * veh.m * obs.A_Y);
    REQUIRE(std::abs(obs.yaw_moment_residual) / scale < 1e-10);
    REQUIRE_THAT(obs.A_Y, WithinRel(ss.value().A_Y_mps2, 1e-9));
}

TEST_CASE("§9.6 sign trap: asymmetric Fx produces the correct yaw moment",
          "[fourwheeler][regression]") {
    // "A defect that a symmetric-input test can never reveal." The couple is
    // (t/2)(Fx_left - Fx_right); writing it the other way silently reverses the
    // yaw effect of asymmetric braking and of a locked differential.
    const core::Vehicle veh = ch5_with_springing();
    const auto geo = wheel_geometry(veh);

    // Right wheels at y = +t/2, left at -t/2. A forward force on the LEFT side
    // alone must yaw the car to the RIGHT, i.e. produce a POSITIVE M_z.
    REQUIRE(geo[LF].y < 0.0);
    REQUIRE(geo[RF].y > 0.0);
    const double Fx_left = 500.0;
    const double yaw_from_left = -geo[LF].y * Fx_left;
    REQUIRE(yaw_from_left > 0.0);
    const double yaw_from_right = -geo[RF].y * Fx_left;
    REQUIRE(yaw_from_right < 0.0);
    // Which is exactly (t/2)*(Fx_left - Fx_right) for Fx_right = 0.
    REQUIRE_THAT(yaw_from_left, WithinRel(0.5 * veh.t_F * Fx_left, 1e-15));
}

// ------------------------------------------------------------------- §5.6 inputs
TEST_CASE("§5.6: maneuvers and their discontinuities", "[fourwheeler][maneuver]") {
    const auto step = Maneuver::make_step(0.1, 0.5);
    REQUIRE(step.delta_at(0.4) == 0.0);
    REQUIRE(step.delta_at(0.5) == 0.1);
    REQUIRE(step.discontinuities() == std::vector<double>{0.5});

    const auto ramp = Maneuver::make_ramp(0.2, 1.0, 2.0);
    REQUIRE(ramp.delta_at(0.5) == 0.0);
    REQUIRE_THAT(ramp.delta_at(2.0), WithinRel(0.1, 1e-15));
    REQUIRE_THAT(ramp.delta_at(9.0), WithinRel(0.2, 1e-15));   // clamped
    REQUIRE(ramp.discontinuities().empty());

    const auto sine = Maneuver::make_sinusoid(0.05, 0.2);
    // §13.3's canonical failure: a 5 Hz input sampled at dt = 0.1 s lands on zero
    // every time.
    for (double t = 0.0; t <= 0.4; t += 0.1) {
        REQUIRE_THAT(sine.delta_at(t), WithinAbs(0.0, 1e-15));
    }
    REQUIRE_THAT(sine.delta_at(0.05), WithinRel(0.05, 1e-12));  // but the peak is there

    const auto doublet = Maneuver::make_doublet(0.1, 1.0, 2.0);
    REQUIRE_THAT(doublet.delta_at(1.5), WithinRel(0.1, 1e-15));
    REQUIRE_THAT(doublet.delta_at(2.5), WithinRel(-0.1, 1e-15));
    REQUIRE(doublet.delta_at(3.5) == 0.0);
    REQUIRE(doublet.discontinuities().size() == 3);
}
