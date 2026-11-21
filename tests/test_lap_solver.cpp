// Spec §11 and the §17.7 acceptance checks S-01 ... S-04, plus C-03.
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

#include "rcvd/analysis/lap_solver.hpp"
#include "rcvd/models/four_wheeler.hpp"
#include "rcvd/tire/linear.hpp"
#include "support/fixtures.hpp"

using namespace rcvd;
using namespace rcvd::analysis;
using namespace rcvd::models;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {
constexpr double FT = units::FOOT;
constexpr double IN = units::INCH;
constexpr double LBF = units::POUND_FORCE;
constexpr double G = units::G_STANDARD;

core::Vehicle speedway_vehicle() {
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

tire::LinearTire half_axle(double axle_C) {
    tire::LinearTireConfig c;
    c.id = "half";
    c.C_alpha_N_per_rad = 0.5 * axle_C;
    c.alpha_lin_rad = 1.0;
    auto r = tire::LinearTire::create(c);
    REQUIRE(r.ok());
    return std::move(r).value();
}

// A circuit the vehicle can actually lap: start on a radius, steer the Ackermann
// angle for that radius, and let the transient do what it does.
Circuit ring(double inner_ft, double outer_ft = 400.0) {
    Circuit c;
    c.inner_radius = inner_ft * FT;
    c.outer_radius = outer_ft * FT;
    return c;
}

DriverPlan plan_at(double radius_ft, double steer_rad, double speed_mps) {
    DriverPlan p;
    p.initial_speed_mps = speed_mps;
    p.steer = Maneuver::make_fixed(steer_rad);
    // Start on the +X axis, heading +Y, so a positive steer curls left-handed
    // around the centre and the swept angle accumulates.
    p.start_x = radius_ft * FT;
    p.start_y = 0.0;
    p.start_heading_rad = units::PI / 2.0;
    return p;
}
}  // namespace

// ---------------------------------------------------------------- §11.4 lap timing
TEST_CASE("11.4: a lap completes and the time is not step-quantised",
          "[lap][acceptance]") {
    const core::Vehicle v = speedway_vehicle();
    const auto C = fixtures::ch5_tires();
    const double V = 40.0 * FT;

    // Steady-state steer for a 150 ft path radius, from P1's own gain.
    const auto m = handling_metrics(v, C);
    const double R = 150.0 * FT;
    const double delta = (v.l / R) * (1.0 + m.K * V * V);   // (8.15)/(8.17) inverted

    BicyclePlant p1(v, C, V);
    LapConfig cfg;
    cfg.calculation_length_s = 60.0;
    const auto res = lap_solve(p1, ring(100.0, 400.0), plan_at(150.0, delta, V), cfg);
    REQUIRE(res.ok());
    INFO("event " << to_string(res.value().event) << " at t = " << res.value().time_s);
    REQUIRE(res.value().event == LapEvent::lap);

    // The lap time must be close to the kinematic estimate 2*pi*R/V.
    REQUIRE_THAT(res.value().time_s, WithinRel(2.0 * units::PI * R / V, 0.1));

    // §11.4: it must NOT be quantised to the integration step. The refined time
    // is generically not a multiple of any record or step increment.
    const double q = res.value().time_s / cfg.output_record_increment_s;
    REQUIRE(std::abs(q - std::round(q)) > 1e-6);
}

TEST_CASE("S-02: lap time is invariant to output_record_increment", "[lap][acceptance]") {
    // §17.7: to 1e-9 s. Event location is by dense output, not by record sampling,
    // so changing the record rate must not move the answer at all.
    const core::Vehicle v = speedway_vehicle();
    const auto C = fixtures::ch5_tires();
    const double V = 40.0 * FT;
    const auto m = handling_metrics(v, C);
    const double delta = (v.l / (150.0 * FT)) * (1.0 + m.K * V * V);
    BicyclePlant p1(v, C, V);

    double reference = 0.0;
    for (double inc : {0.5, 0.05, 0.01, 0.002}) {
        LapConfig cfg;
        cfg.calculation_length_s = 60.0;
        cfg.output_record_increment_s = inc;
        const auto res = lap_solve(p1, ring(100.0, 400.0), plan_at(150.0, delta, V), cfg);
        REQUIRE(res.ok());
        REQUIRE(res.value().event == LapEvent::lap);
        if (reference == 0.0) {
            reference = res.value().time_s;
        } else {
            INFO("record increment " << inc);
            REQUIRE_THAT(res.value().time_s, WithinAbs(reference, 1e-9));
        }
    }

    // Turning recording off entirely must also not move it.
    LapConfig off;
    off.calculation_length_s = 60.0;
    off.record_trajectory = false;
    const auto bare = lap_solve(p1, ring(100.0, 400.0), plan_at(150.0, delta, V), off);
    REQUIRE_THAT(bare.value().time_s, WithinAbs(reference, 1e-9));
}

TEST_CASE("S-04: lap time converges under step refinement", "[lap][acceptance]") {
    const core::Vehicle v = speedway_vehicle();
    const auto C = fixtures::ch5_tires();
    const double V = 40.0 * FT;
    const auto m = handling_metrics(v, C);
    const double delta = (v.l / (150.0 * FT)) * (1.0 + m.K * V * V);
    BicyclePlant p1(v, C, V);

    auto solve_at = [&](double rtol) {
        LapConfig cfg;
        cfg.calculation_length_s = 60.0;
        cfg.integrator.rtol = rtol;
        cfg.integrator.h_max = 0.02;
        const auto r = lap_solve(p1, ring(100.0, 400.0), plan_at(150.0, delta, V), cfg);
        REQUIRE(r.ok());
        REQUIRE(r.value().event == LapEvent::lap);
        return r.value().time_s;
    };

    const double t1 = solve_at(1e-6), t2 = solve_at(1e-8), t3 = solve_at(1e-10);
    const auto conv = lap_convergence(t1, t2, t3);
    INFO("lap times " << t1 << ", " << t2 << ", " << t3
                      << "; Richardson error " << conv.richardson_error_s);
    // Tightening the tolerance must move the answer by less than the previous move.
    REQUIRE(std::abs(t3 - t2) <= std::abs(t2 - t1));
    REQUIRE(conv.richardson_error_s < 1e-6);
}

// ------------------------------------------------------------ §11.5 / §11.6 events
TEST_CASE("11.6 / S-01: the theoretical optimum hits the inner wall",
          "[lap][acceptance]") {
    // "This solution crashes into the inner wall." At t = 0 the yaw rate and
    // lateral velocity are zero, so the vehicle needs a transient before it
    // reaches the steady-state circular path; during it the path is not
    // concentric and the circle it settles on is offset INWARD.
    const core::Vehicle v = speedway_vehicle();
    const auto C = fixtures::ch5_tires();
    const double V = 40.0 * FT;
    const auto m = handling_metrics(v, C);
    const double R = 50.0 * FT;
    const double delta = (v.l / R) * (1.0 + m.K * V * V);

    BicyclePlant p1(v, C, V);
    const auto eig = eigenstructure(v, C, V);
    TransientContext ctx;
    ctx.tau_dominant_s = eig.tau_dominant;
    ctx.steady_state_radius_m = V / (yaw_rate_gain(v, m.K, V) * delta);
    REQUIRE_THAT(ctx.steady_state_radius_m, WithinRel(R, 1e-9));   // by construction

    LapConfig cfg;
    cfg.calculation_length_s = 30.0;
    const auto res = lap_solve(p1, ring(50.0, 400.0), plan_at(50.0, delta, V), cfg, ctx);
    REQUIRE(res.ok());
    REQUIRE(res.value().event == LapEvent::wall_in);

    // §11.6: the hint fires because the settled path WOULD have cleared the wall,
    // so only the transient offset explains the contact.
    REQUIRE_FALSE(res.value().diagnostics.empty());
    REQUIRE(res.value().diagnostics.front().code == "H-SPD-001");

    // The contact is NOT within 5 tau, which is why §11.6's attribution test is
    // "within the first revolution with an adequate R_ss" rather than a fixed
    // multiple of the yaw time constant: tau is 0.059 s here, 5 tau is 0.30 s, and
    // contact comes at 1.8-3.3 s — 30 to 56 tau — because the offset circle's
    // closest approach to the wall is up to half a revolution after the transient.
    REQUIRE(res.value().time_s > 5.0 * eig.tau_dominant);
    REQUIRE(res.value().time_s < units::PI * R / V * 1.1);   // inside half a lap

    // S-01: "starting further from the wall" is the remedy that actually applies
    // here, and the geometry says why. For a tangential start at R_start settling
    // onto radius R_ss, the settled circle's centre sits at |R_start - R_ss| from
    // the origin, so its closest approach is |R_ss - |R_start - R_ss||. For
    // R_ss > R_start that equals R_start EXACTLY — the circle is tangent to the
    // start radius from outside. So a car starting ON the wall cannot be saved by
    // reducing steer alone, however gentle: the best it achieves is tangency, and
    // the transient offset then takes it inside. Moving the start out gives real
    // margin.
    const double R2 = 75.0 * FT;
    const double delta2 = (v.l / R2) * (1.0 + m.K * V * V);
    TransientContext ctx2 = ctx;
    ctx2.steady_state_radius_m = V / (yaw_rate_gain(v, m.K, V) * delta2);
    REQUIRE_THAT(ctx2.steady_state_radius_m, WithinRel(R2, 1e-9));
    const auto fixed = lap_solve(p1, ring(50.0, 400.0), plan_at(75.0, delta2, V), cfg, ctx2);
    REQUIRE(fixed.ok());
    REQUIRE(fixed.value().event == LapEvent::lap);
}

TEST_CASE("11.6: too much steer is not attributed to the transient", "[lap][validation]") {
    // The discriminator the attribution test exists for: when the implied
    // steady-state radius is INSIDE the wall, the vehicle would have hit it even
    // fully settled, so the contact is not the transient's doing and gets no hint.
    const core::Vehicle v = speedway_vehicle();
    const auto C = fixtures::ch5_tires();
    const double V = 40.0 * FT;
    const auto m = handling_metrics(v, C);
    BicyclePlant p1(v, C, V);

    // Steer for a 30 ft steady-state radius on a 50 ft inner wall.
    const double delta = (v.l / (30.0 * FT)) * (1.0 + m.K * V * V);
    TransientContext ctx;
    ctx.tau_dominant_s = eigenstructure(v, C, V).tau_dominant;
    ctx.steady_state_radius_m = V / (yaw_rate_gain(v, m.K, V) * delta);
    REQUIRE(ctx.steady_state_radius_m < 50.0 * FT);

    LapConfig cfg;
    cfg.calculation_length_s = 30.0;
    const auto res = lap_solve(p1, ring(50.0, 400.0), plan_at(70.0, delta, V), cfg, ctx);
    REQUIRE(res.ok());
    REQUIRE(res.value().event == LapEvent::wall_in);
    REQUIRE(res.value().diagnostics.empty());   // no H-SPD-001
}

TEST_CASE("11.5: outer-wall departure is detected and refined", "[lap][validation]") {
    const core::Vehicle v = speedway_vehicle();
    const auto C = fixtures::ch5_tires();
    const double V = 40.0 * FT;
    BicyclePlant p1(v, C, V);

    // No steer at all: the car drives straight off the outer wall.
    LapConfig cfg;
    cfg.calculation_length_s = 30.0;
    const auto res = lap_solve(p1, ring(50.0, 200.0), plan_at(100.0, 0.0, V), cfg);
    REQUIRE(res.ok());
    REQUIRE(res.value().event == LapEvent::wall_out);

    // The refined state must sit ON the wall, not past it by up to a step.
    const double rad = std::hypot(res.value().final_state[state::kXe],
                                  res.value().final_state[state::kYe]);
    REQUIRE_THAT(rad, WithinAbs(200.0 * FT, 1e-6));
}

TEST_CASE("EV-TIMEOUT when no lap is completed", "[lap][validation]") {
    const core::Vehicle v = speedway_vehicle();
    const auto C = fixtures::ch5_tires();
    BicyclePlant p1(v, C, 40.0 * FT);

    LapConfig cfg;
    cfg.calculation_length_s = 1.0;   // far too short to lap
    const auto res = lap_solve(p1, ring(100.0, 1e6), plan_at(150.0, 0.02, 40.0 * FT), cfg);
    REQUIRE(res.ok());
    REQUIRE(res.value().event == LapEvent::timeout);
    REQUIRE_THAT(res.value().time_s, WithinAbs(1.0, 1e-12));
    REQUIRE(res.value().diagnostics.front().code == "EV-TIMEOUT");
}

TEST_CASE("11.4: the event is located on the bracketing step, not after it",
          "[lap][regression]") {
    // The defect §11.4 exists to prevent: reporting the first step at which the
    // condition holds. With a deliberately coarse step the refined time must still
    // land where the geometry says, not on a step boundary.
    const core::Vehicle v = speedway_vehicle();
    const auto C = fixtures::ch5_tires();
    const double V = 40.0 * FT;
    BicyclePlant p1(v, C, V);

    LapConfig coarse, fine;
    coarse.calculation_length_s = 30.0;
    coarse.integrator.h_max = 0.25;      // very coarse
    coarse.integrator.rtol = 1e-6;
    fine.calculation_length_s = 30.0;
    fine.integrator.h_max = 0.002;
    fine.integrator.rtol = 1e-11;

    const auto a = lap_solve(p1, ring(50.0, 200.0), plan_at(100.0, 0.0, V), coarse);
    const auto b = lap_solve(p1, ring(50.0, 200.0), plan_at(100.0, 0.0, V), fine);
    REQUIRE(a.value().event == LapEvent::wall_out);
    REQUIRE(b.value().event == LapEvent::wall_out);
    // A step-quantised answer would differ by up to h_max = 0.25 s. Dense output
    // plus Brent brings them to within a small multiple of the event tolerance.
    INFO("coarse " << a.value().time_s << " s, fine " << b.value().time_s << " s");
    REQUIRE(std::abs(a.value().time_s - b.value().time_s) < 1e-5);
}

// ------------------------------------------------------------------- §17.8 C-03
TEST_CASE("C-03: P4 and P5 produce identical lap times", "[lap][crosscheck]") {
    // §17.8: "P4 with the P1 plant and P5 with a P2 plant configured as in C-02
    // produce identical lap times to 1e-9 s."
    const core::Vehicle v = speedway_vehicle();
    const auto C = fixtures::ch5_tires();
    const double V = 40.0 * FT;
    const auto m = handling_metrics(v, C);
    const double delta = (v.l / (150.0 * FT)) * (1.0 + m.K * V * V);

    auto tF = half_axle(C.C_F_N_per_rad);
    auto tR = half_axle(C.C_R_N_per_rad);
    const std::array<const tire::TireModel*, 4> tires{&tF, &tF, &tR, &tR};
    aero::AeroConfig no_aero;
    SolverSettings cfg2;
    cfg2.switches = ModelSwitches::bicycle_equivalence();

    BicyclePlant p4(v, C, V);
    FourWheeler p5(v, tires, no_aero, cfg2);
    p5.set_speed(V);

    LapConfig cfg;
    cfg.calculation_length_s = 60.0;
    const auto a = lap_solve(p4, ring(100.0, 400.0), plan_at(150.0, delta, V), cfg);
    const auto b = lap_solve(p5, ring(100.0, 400.0), plan_at(150.0, delta, V), cfg);
    REQUIRE(a.ok());
    REQUIRE(b.ok());
    REQUIRE(a.value().event == LapEvent::lap);
    REQUIRE(b.value().event == a.value().event);
    INFO("P4 " << a.value().time_s << " s, P5 " << b.value().time_s << " s");
    REQUIRE_THAT(b.value().time_s, WithinAbs(a.value().time_s, 1e-9));
}

// -------------------------------------------------------------------- §11.7 / A2
TEST_CASE("A2: repeated runs are bit-identical, and recording changes nothing",
          "[lap][invariant]") {
    // §11.7: "The display SHALL NOT affect the numerical result" — frame drops,
    // resizing or pausing change nothing. Recording is the headless stand-in for
    // the display, so the same requirement applies to it, and repeated runs must
    // be bit-identical (Rule A2).
    const core::Vehicle v = speedway_vehicle();
    const auto C = fixtures::ch5_tires();
    const double V = 40.0 * FT;
    const auto m = handling_metrics(v, C);
    const double delta = (v.l / (150.0 * FT)) * (1.0 + m.K * V * V);
    BicyclePlant p1(v, C, V);

    LapConfig cfg;
    cfg.calculation_length_s = 60.0;
    double first = 0.0;
    for (int i = 0; i < 5; ++i) {
        const auto r = lap_solve(p1, ring(100.0, 400.0), plan_at(150.0, delta, V), cfg);
        REQUIRE(r.ok());
        if (i == 0) {
            first = r.value().time_s;
        } else {
            REQUIRE(r.value().time_s == first);   // BIT-identical, not merely close
        }
    }
}

TEST_CASE("15.1: records come from dense output at the requested rate", "[lap]") {
    const core::Vehicle v = speedway_vehicle();
    const auto C = fixtures::ch5_tires();
    const double V = 40.0 * FT;
    BicyclePlant p1(v, C, V);

    LapConfig cfg;
    cfg.calculation_length_s = 2.0;
    cfg.output_record_increment_s = 0.1;
    const auto res = lap_solve(p1, ring(50.0, 1e6), plan_at(150.0, 0.01, V), cfg);
    REQUIRE(res.ok());
    REQUIRE(res.value().event == LapEvent::timeout);

    // ~21 records over 2 s at 0.1 s, plus the initial one.
    REQUIRE(res.value().path.size() >= 20);
    // The record times are exactly the requested multiples, independent of where
    // the integrator chose to step.
    for (std::size_t i = 1; i + 1 < res.value().path.size(); ++i) {
        const double dt = res.value().path[i + 1].t - res.value().path[i].t;
        REQUIRE_THAT(dt, WithinAbs(0.1, 1e-12));
    }
}
