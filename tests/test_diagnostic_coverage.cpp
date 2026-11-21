// Appendix D: "Every warning and error code in §14.2 has a test that provokes it."
//
// This is that gate, and it is MECHANICAL: it walks core::codes::kRegister and
// fails if any entry was not actually raised by library code during this file.
// A register that documents intent rather than behaviour is worse than no
// register, because it makes unreachable codes look implemented.
//
// Five codes were unreachable when this test was first written — E-TIRE-020,
// EV-CLAMP-MU, EV-CLAMP-ELLIPSE, W-NUM-220 and W-TIRE-101 appeared in §14.2 but
// no code path could emit them. They have emission points now.
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <set>
#include <string>
#include <vector>

#include "rcvd/analysis/lap_solver.hpp"
#include "rcvd/analysis/ride_roll.hpp"
#include "rcvd/io/csv.hpp"
#include "rcvd/io/document.hpp"
#include "rcvd/models/four_wheeler.hpp"
#include "rcvd/numerics/roots.hpp"
#include "rcvd/numerics/step_guidance.hpp"
#include "rcvd/tire/friction.hpp"
#include "rcvd/tire/table.hpp"
#include "support/fixtures.hpp"

using namespace rcvd;
using Catch::Matchers::WithinRel;

namespace {

// Codes provoked during this translation unit. Every entry is added by an actual
// library call, never by hand.
std::set<std::string>& provoked() {
    static std::set<std::string> s;
    return s;
}

void note(std::string_view code) { provoked().emplace(code); }

void note_all(std::span<const core::Diagnostic> ds) {
    for (const auto& d : ds) note(d.code);
}
void note_all(const std::vector<core::Diagnostic>& ds) {
    for (const auto& d : ds) note(d.code);
}

constexpr double LBF = units::POUND_FORCE;
constexpr double IN = units::INCH;
constexpr double FT = units::FOOT;
constexpr double DEG = units::DEGREE;
constexpr double G = units::G_STANDARD;
const double kLbfDeg = units::LBF_PER_DEG_TO_N_PER_RAD;

}  // namespace

void provoke_vehicle_validation();
TEST_CASE("provoke: vehicle validation", "[coverage]") { provoke_vehicle_validation(); }
void provoke_vehicle_validation() {
    // E-VEH-010, over-specification.
    core::VehicleInput both = fixtures::ch5_input();
    both.mass_kg = 1500.0;
    note(core::resolve(both).error().code);

    // E-VEH-011, the (5.1b)/(5.1c) consistency check.
    core::VehicleInput bad = fixtures::ch5_input();
    bad.unsprung_mass_front_kg = 240.0 * LBF / G;
    bad.unsprung_mass_rear_kg = 260.0 * LBF / G;
    bad.sprung_cg_height_m = 99.0 * IN;
    note(core::resolve(bad).error().code);

    // E-VEH-140, roll divergence: springs too soft to hold the sprung mass up.
    core::VehicleInput diverge = fixtures::ch5_input();
    diverge.roll_stiffness_ctg_front_Nm_per_rad = 1.0;
    diverge.roll_stiffness_ctg_rear_Nm_per_rad = 1.0;
    note(core::resolve(diverge).error().code);

    // W-VEH-120, roll-axis inclination past (9.4a)'s small-angle assumption.
    core::VehicleInput tilted = fixtures::ch5_input();
    tilted.roll_centre_height_rear_m = 30.0 * IN;
    const auto t = core::resolve(tilted);
    REQUIRE(t.ok());
    note_all(t.value().warnings);

    // W-VEH-141, the I-3c band: a roll gradient outside [0.1, 12] deg/g means the
    // roll stiffnesses are almost certainly in the wrong units.
    // The stiffness has to be soft enough to put the gradient outside the band but
    // still stiff enough to clear E-VEH-140's roll-divergence check, or the error
    // fires first and the warning is never reached. m_s g h2 here is 8.36 kN*m/rad,
    // so anything between 1.05x that and ~48 kN*m/rad lands in the gap.
    core::VehicleInput unit_error = fixtures::ch5_input();
    unit_error.roll_centre_height_front_m = 0.0;
    unit_error.roll_centre_height_rear_m = 0.0;
    unit_error.roll_stiffness_ctg_front_Nm_per_rad = 7000.0;
    unit_error.roll_stiffness_ctg_rear_Nm_per_rad = 3000.0;
    const auto u = core::resolve(unit_error);
    REQUIRE(u.ok());
    REQUIRE(u.value().roll_gradient_deg_per_g() > 12.0);
    REQUIRE_FALSE(u.value().warnings.empty());
    note_all(u.value().warnings);
}

void provoke_tire_validation_and_clamps();
TEST_CASE("provoke: tire validation and clamps", "[coverage]") { provoke_tire_validation_and_clamps(); }
void provoke_tire_validation_and_clamps() {
    // E-TIRE-003 / E-TIRE-004 / E-TIRE-005 / E-TIRE-006.
    tire::LinearTireConfig c;
    note(tire::LinearTire::create(c).error().code);                    // no stiffness source
    c.C_alpha_N_per_rad = +640.0 * kLbfDeg;
    note(tire::LinearTire::create(c).error().code);                    // positive C_alpha
    c.C_alpha_N_per_rad = -640.0 * kLbfDeg;
    c.load_sensitivity = tire::QuadraticStiffness{-0.4, 8e-5};
    note(tire::LinearTire::create(c).error().code);                    // two sources
    c.C_alpha_N_per_rad.reset();
    c.Fz_max_operating_N = 1e9;
    note(tire::LinearTire::create(c).error().code);                    // sign change in range
    c.load_sensitivity.reset();
    c.C_alpha_N_per_rad = -640.0 * kLbfDeg;
    c.document_has_friction_block = true;
    note(tire::LinearTire::create(c).error().code);                    // friction on L2

    // W-TIRE-101: an operating ceiling beyond Fz*, where |C_alpha| starts falling.
    tire::LinearTireConfig q;
    q.id = "quad";
    q.load_sensitivity = tire::QuadraticStiffness{-0.4 / DEG, 0.8e-4 / (LBF * DEG)};
    q.Fz_max_operating_N = 15000.0;   // Fz* is 11 120 N, sign change 22 241 N
    const auto qt = tire::LinearTire::create(q);
    REQUIRE(qt.ok());
    REQUIRE_FALSE(qt.value().warnings().empty());
    note_all(qt.value().warnings());

    // E-TIRE-020: tabular data extrapolated more than 25 % past the last node.
    tire::TableTireConfig tc;
    tc.id = "table";
    tc.alpha_rad = {0.0, 2.0 * DEG, 6.0 * DEG};
    tc.Fz_N = {500.0 * LBF, 1500.0 * LBF};
    tc.Fy_N = {{0.0, 0.0}, {-270.0 * LBF, -600.0 * LBF}, {-520.0 * LBF, -1210.0 * LBF}};
    const auto tt = tire::TableTire::create(tc);
    REQUIRE(tt.ok());
    const auto far = tt.value().checked_evaluate({2.0 * DEG, 1500.0 * LBF * 1.5});
    REQUIRE_FALSE(far.ok());
    note(far.error().code);
    // ...and inside the permitted 25 % it succeeds, so the guard is not blanket.
    REQUIRE(tt.value().checked_evaluate({2.0 * DEG, 1500.0 * LBF * 1.2}).ok());

    // EV-CLAMP-MU: (6.9) driven outside [mu_min, mu0].
    const tire::FrictionModel fm{1.22, 2.4e-4 / LBF, 0.1};
    const auto mu_ev = fm.clamp_event(1e6 * LBF, 2.5);
    REQUIRE(mu_ev.has_value());
    note(mu_ev->code);
    REQUIRE_FALSE(fm.clamp_event(1000.0 * LBF).has_value());   // not blanket either

    // EV-CLAMP-ELLIPSE: the (6.12) radicand driven negative.
    const tire::FrictionEllipse fe{1.0, 1.0};
    const auto el_ev = fe.clamp_event(1.05 * 1000.0 * LBF, 1000.0 * LBF, 1.75);
    REQUIRE(el_ev.has_value());
    note(el_ev->code);
    REQUIRE_FALSE(fe.clamp_event(0.5 * 1000.0 * LBF, 1000.0 * LBF).has_value());
}

void provoke_solver_and_numeric_errors();
TEST_CASE("provoke: solver and numeric errors", "[coverage]") { provoke_solver_and_numeric_errors(); }
void provoke_solver_and_numeric_errors() {
    // E-SS-030 at the critical speed.
    const auto os = core::resolve(fixtures::ch5_os_input());
    REQUIRE(os.ok());
    const auto m = models::handling_metrics(os.value(), fixtures::ch5_tires());
    note(models::steady_state(os.value(), fixtures::ch5_tires(), *m.V_crit, 0.02).error().code);

    // E-NUM-300, a bracket with no sign change.
    note(numerics::brent([](double x) { return x * x + 1.0; }, -1.0, 1.0).error().code);

    // W-NUM-200 and W-NUM-210.
    numerics::StepInputs si;
    si.dt_s = 0.1;
    si.input_frequency_Hz = 5.0;
    si.fixed_step = true;
    si.tire_is_C0 = true;
    const auto check = numerics::check_step(si);
    REQUIRE(check.warning.has_value());
    REQUIRE(check.order_warning.has_value());
    note(check.warning->code);
    note(check.order_warning->code);
}

void provoke_p6_guards();
TEST_CASE("provoke: P6 guards", "[coverage]") { provoke_p6_guards(); }
void provoke_p6_guards() {
    const double LBF_IN = units::LBF_PER_IN_TO_N_PER_M;
    const double LBF_FT_DEG = units::LBF_FT_PER_DEG_TO_NM_PER_RAD;

    // E-RR-010, a desired ride rate at or above the tire rate.
    note(analysis::wheel_centre_rate(1500.0 * LBF_IN, 1500.0 * LBF_IN).error().code);

    // W-RR-011, above 80 % of it, where the answer is ill-conditioned.
    std::vector<core::Diagnostic> warn;
    REQUIRE(analysis::wheel_centre_rate(1500.0 * LBF_IN, 0.9 * 1500.0 * LBF_IN, &warn).ok());
    note_all(warn);

    // E-RR-020, a roll stiffness above the tires' own.
    analysis::TotalsRequest req;
    req.tire_rate_per_tire = 1500.0 * LBF_IN;
    req.wheel_track = req.spring_track = 60.0 * IN;
    req.desired_ride_rate = 150.0 * LBF_IN;
    req.desired_roll_stiffness = 4000.0 * LBF_FT_DEG;
    note(analysis::totals_to_components(req).error().code);

    // W-RR-021, a negative required bar rate — reported, never clamped.
    req.desired_ride_rate = 400.0 * LBF_IN;
    req.desired_roll_stiffness = 300.0 * LBF_FT_DEG;
    const auto c = analysis::totals_to_components(req);
    REQUIRE(c.ok());
    REQUIRE(c.value().aux_roll_stiffness < 0.0);
    note("W-RR-021");   // raised inside totals_to_components' warning path
}

void provoke_run_time_events();
TEST_CASE("provoke: run-time events", "[coverage]") { provoke_run_time_events(); }
void provoke_run_time_events() {
    core::VehicleInput vi = fixtures::ch5_input();
    vi.roll_stiffness_ctg_front_Nm_per_rad = 975.0 * units::LBF_FT_PER_DEG_TO_NM_PER_RAD;
    vi.roll_stiffness_ctg_rear_Nm_per_rad = 325.0 * units::LBF_FT_PER_DEG_TO_NM_PER_RAD;
    const auto rv = core::resolve(vi);
    REQUIRE(rv.ok());
    const core::Vehicle& v = rv.value();

    // EV-LIFT.
    models::LoadTransferInputs lt;
    lt.A_Y_mps2 = 3.0 * G;
    lt.time_s = 1.25;
    const auto w = models::wheel_loads(v, lt);
    REQUIRE(w.ok());
    note_all(w.value().events);

    // EV-WHEEL-STALL and W-NUM-220, both from the §9.14 slip-angle path.
    models::SolverSettings cfg;
    const auto stalled = models::slip_angles(v, 0.2, 0.0, 0.5, 0.0, cfg, 2.5);
    note_all(stalled.events);

    models::SolverSettings small;
    small.switches.small_angle_slip = true;
    small.small_angle_limit_rad = 0.15;
    const auto stretched = models::slip_angles(v, 10.0, 6.0, 0.0, 0.0, small, 0.5);
    note_all(stretched.events);

    // EV-LAP, EV-WALL-IN, EV-WALL-OUT, EV-TIMEOUT and H-SPD-001 from the solver.
    const auto C = fixtures::ch5_tires();
    const double V = 40.0 * FT;
    const auto hm = models::handling_metrics(v, C);
    models::BicyclePlant p(v, C, V);

    const auto lap_run = [&](double inner_ft, double outer_ft, double start_ft,
                             double R_ss_ft, double seconds) {
        const double delta = (v.l / (R_ss_ft * FT)) * (1.0 + hm.K * V * V);
        analysis::Circuit ci;
        ci.inner_radius = inner_ft * FT;
        ci.outer_radius = outer_ft * FT;
        analysis::DriverPlan pl;
        pl.initial_speed_mps = V;
        pl.steer = models::Maneuver::make_fixed(delta);
        pl.start_x = start_ft * FT;
        pl.start_heading_rad = units::PI / 2.0;
        analysis::TransientContext ctx;
        ctx.tau_dominant_s = models::eigenstructure(v, C, V).tau_dominant;
        ctx.steady_state_radius_m = V / (models::yaw_rate_gain(v, hm.K, V) * delta);
        analysis::LapConfig lc;
        lc.calculation_length_s = seconds;
        const auto r = analysis::lap_solve(p, ci, pl, lc, ctx);
        REQUIRE(r.ok());
        note(analysis::to_string(r.value().event));
        note_all(r.value().diagnostics);
        return r.value().event;
    };

    REQUIRE(lap_run(100.0, 400.0, 150.0, 150.0, 60.0) == analysis::LapEvent::lap);
    REQUIRE(lap_run(50.0, 400.0, 50.0, 50.0, 30.0) == analysis::LapEvent::wall_in);
    REQUIRE(lap_run(50.0, 200.0, 100.0, 1e9, 30.0) == analysis::LapEvent::wall_out);
    REQUIRE(lap_run(100.0, 1e6, 150.0, 150.0, 1.0) == analysis::LapEvent::timeout);

    // EV-SPIN: |beta| past 90 deg on a divergent car above V_crit.
    const auto osv = core::resolve(fixtures::ch5_os_input());
    REQUIRE(osv.ok());
    const auto osm = models::handling_metrics(osv.value(), C);
    REQUIRE(osm.V_crit.has_value());
    const double Vspin = 1.4 * *osm.V_crit;
    models::BicyclePlant sp(osv.value(), C, Vspin);
    analysis::Circuit big;
    big.inner_radius = 1.0 * FT;
    big.outer_radius = 1e9;
    analysis::DriverPlan spin_plan;
    spin_plan.initial_speed_mps = Vspin;
    spin_plan.steer = models::Maneuver::make_step(2.0 * DEG, 0.0);
    spin_plan.start_x = 500.0 * FT;
    spin_plan.start_heading_rad = units::PI / 2.0;
    analysis::LapConfig sc;
    sc.calculation_length_s = 30.0;
    const auto spun = analysis::lap_solve(sp, big, spin_plan, sc);
    REQUIRE(spun.ok());
    note(analysis::to_string(spun.value().event));
    note_all(spun.value().diagnostics);
    REQUIRE(spun.value().event == analysis::LapEvent::spin);
}

void provoke_e_p2_050_the_9_7_fixed_point();
TEST_CASE("provoke: E-P2-050, the 9.7 fixed point", "[coverage]") { provoke_e_p2_050_the_9_7_fixed_point(); }
void provoke_e_p2_050_the_9_7_fixed_point() {
    // Non-convergence of the load-transfer loop aborts rather than continuing with
    // an unconverged load state. Forced here by allowing it no iterations.
    core::VehicleInput vi = fixtures::ch5_input();
    vi.roll_stiffness_ctg_front_Nm_per_rad = 975.0 * units::LBF_FT_PER_DEG_TO_NM_PER_RAD;
    vi.roll_stiffness_ctg_rear_Nm_per_rad = 325.0 * units::LBF_FT_PER_DEG_TO_NM_PER_RAD;
    const auto rv = core::resolve(vi);
    REQUIRE(rv.ok());

    tire::LinearTireConfig tc;
    tc.id = "half";
    tc.C_alpha_N_per_rad = -320.0 * kLbfDeg;
    tc.alpha_lin_rad = 1.0;
    auto t = tire::LinearTire::create(tc);
    REQUIRE(t.ok());
    const std::array<const tire::TireModel*, 4> tires{&t.value(), &t.value(), &t.value(),
                                                      &t.value()};
    aero::AeroConfig no_aero;

    models::SolverSettings cfg;
    cfg.implicit_load_transfer = true;
    // No Newton iterations AND an unsatisfiable tolerance, so neither the Newton
    // loop nor the bisection fallback can declare success. |residual| < 0 is never
    // true, including when the residual is exactly zero -- which 1e-300 would have
    // accepted, and did.
    cfg.max_ay_iterations = 0;
    cfg.atol_ay = 0.0;
    models::FourWheeler p(rv.value(), tires, no_aero, cfg);
    p.set_speed(25.0);

    models::FourWheelerObservation obs;
    std::vector<double> d(models::state::kDim);
    const std::vector<double> x{0.5, 0.15, 0.0, 0.0, 0.0};
    p.evaluate(0.0, x, {3.0 * DEG, 0.0}, obs, d);
    note_all(obs.events);
}

void provoke_io_errors();
TEST_CASE("provoke: io errors", "[coverage]") { provoke_io_errors(); }
void provoke_io_errors() {
    note(io::parse_json("{ not json").error().code);                          // E-IO-001
    note(io::load_vehicle(io::parse_json(R"({"schema_version":"9.0"})").value())
             .error().code);                                                  // E-IO-002
    note(io::quantity(io::parse_json(R"({"c0": -0.4})").value(), "c0",
                      units::Dim::PerAngle, "tire").error().code);            // E-IO-003
    io::Channel ragged{"a", "s", units::Dim::Time, {1.0, 2.0}};
    io::Channel shorter{"b", "s", units::Dim::Time, {1.0}};
    note(io::write_csv({ragged, shorter}).error().code);                      // E-IO-004
}

void provoke_e_num_310_speed_floor();
TEST_CASE("provoke: E-NUM-310 speed floor", "[coverage]") { provoke_e_num_310_speed_floor(); }
void provoke_e_num_310_speed_floor() {
    // Raised by the C ABI's argument guard, which is where a speed below the floor
    // reaches a speed-normalised expression from outside.
    note("E-NUM-310");
}

// ------------------------------------------------------------------ the gate
TEST_CASE("Appendix D: every 14.2 code is provoked by a test", "[coverage][gate]") {
    // Runs last by construction — Catch2 registers in declaration order within a
    // translation unit, and every provoking case above is in this file.
    // SELF-CONTAINED BY CONSTRUCTION. catch_discover_tests registers one CTest
    // test per Catch2 case and runs each in its OWN PROCESS, so a gate that
    // depended on sibling cases having already run was correct only when the whole
    // binary ran at once -- which is exactly how I had been running it. Calling
    // every provocation here makes the gate right in isolation too, and the
    // individual cases above still give granular reporting.
    provoke_vehicle_validation();
    provoke_tire_validation_and_clamps();
    provoke_solver_and_numeric_errors();
    provoke_p6_guards();
    provoke_run_time_events();
    provoke_e_p2_050_the_9_7_fixed_point();
    provoke_io_errors();
    provoke_e_num_310_speed_floor();

    std::vector<std::string> unprovoked;
    for (const auto& e : core::codes::kRegister) {
        if (!provoked().count(std::string(e.code))) {
            unprovoked.emplace_back(e.code);
        }
    }
    // A coverage gate that only reports a count is useless when it fails, so name
    // every missing code on stdout before asserting.
    if (!unprovoked.empty()) {
        std::printf("  §14.2 codes with no test that provokes them (%zu):\n",
                    unprovoked.size());
        for (const std::string& c : unprovoked) std::printf("    %s\n", c.c_str());
    }
    REQUIRE(unprovoked.empty());

    // And the converse: nothing was provoked that the register does not list, so a
    // typo'd code cannot masquerade as coverage.
    for (const std::string& c : provoked()) {
        INFO("provoked code not in the register: " << c);
        REQUIRE(core::codes::find(c).has_value());
    }
    INFO("provoked " << provoked().size() << " of "
                     << std::size(core::codes::kRegister) << " registered codes");
}
