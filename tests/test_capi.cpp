// Spec §4.1 (the C ABI), §4.6 (Rule A3) and §18.3 (interface equivalence).
//
// §18.3: "For every acceptance vector, the GUI, rcvd-cli and the C API SHALL be
// exercised and SHALL agree bit-for-bit. This is what makes Rule A1 testable
// rather than aspirational."
//
// The GUI does not exist yet, so this closes the C++/C half of that gate. Every
// comparison below is `==` on doubles, deliberately: "bit-for-bit" is the
// requirement, and a tolerance here would silently permit the C layer to do
// arithmetic of its own.
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <cstring>
#include <string>

#include "rcvd/analysis/lap_solver.hpp"
#include "rcvd/analysis/ride_roll.hpp"
#include "rcvd/capi/rcvd.h"
#include "rcvd/models/bicycle.hpp"
#include "rcvd/models/four_wheeler.hpp"
#include "support/fixtures.hpp"

using namespace rcvd;
using Catch::Matchers::WithinRel;

namespace {

const char* kCh5Json = R"JSON({
  "schema_version": "1.0",
  "name": "CH5",
  "inertia": {
    "weight":      {"value": 3700, "unit": "lbf"},
    "yaw_inertia": {"value": 1800, "unit": "slug*ft^2"}
  },
  "geometry": {
    "wheelbase":   {"value": 104.5, "unit": "in"},
    "front_weight_fraction": 0.57,
    "track_front": {"value": 60, "unit": "in"},
    "track_rear":  {"value": 60, "unit": "in"},
    "cg_height":   {"value": 20, "unit": "in"}
  },
  "steering": {"ratio": 14.0}
})JSON";

// RAII so a failing REQUIRE cannot leak the handle.
struct Handle {
    rcvd_vehicle* p = nullptr;
    ~Handle() { rcvd_vehicle_free(p); }
};

const double kCF = -640.0 * units::LBF_PER_DEG_TO_N_PER_RAD;
const double kCR = -580.0 * units::LBF_PER_DEG_TO_N_PER_RAD;

}  // namespace

TEST_CASE("C ABI: version and provenance", "[capi]") {
    int major = -1, minor = -1, patch = -1;
    rcvd_version(&major, &minor, &patch);
    REQUIRE(major >= 0);
    REQUIRE(std::string(rcvd_spec_revision()) == "1.0");
    REQUIRE(rcvd_abi_version() == 1);
    // Null arguments are tolerated, since a binding may want only one field.
    rcvd_version(nullptr, nullptr, nullptr);
}

TEST_CASE("C ABI: Rule U2 conversion, with the dimension check intact", "[capi]") {
    double out = 0.0;
    REQUIRE(rcvd_convert(1.0, "lbf/deg", "N/rad", &out) == RCVD_OK);
    REQUIRE_THAT(out, WithinRel(254.864324893, 1e-11));

    REQUIRE(rcvd_convert(104.5, "in", "ft", &out) == RCVD_OK);
    REQUIRE_THAT(out, WithinRel(104.5 / 12.0, 1e-15));

    // Different dimensions must fail, not silently scale.
    REQUIRE(rcvd_convert(1.0, "lbf", "ft", &out) == RCVD_ERROR);
    REQUIRE(std::string(rcvd_last_error_code()) == "E-IO-003");
    REQUIRE(std::string(rcvd_last_error_message()).find("different dimensions") !=
            std::string::npos);

    // Unknown units too — §5's vocabulary is closed.
    REQUIRE(rcvd_convert(1.0, "furlong", "ft", &out) == RCVD_ERROR);
    // And a successful call clears the previous error.
    REQUIRE(rcvd_convert(1.0, "ft", "m", &out) == RCVD_OK);
    REQUIRE(std::string(rcvd_last_error_code()).empty());
}

TEST_CASE("C ABI: a bad document reports the same diagnostic the core does",
          "[capi][validation]") {
    Handle h;
    REQUIRE(rcvd_vehicle_load_json("{ not json", &h.p) == RCVD_ERROR);
    REQUIRE(std::string(rcvd_last_error_code()) == "E-IO-001");
    REQUIRE(h.p == nullptr);

    Handle h2;
    REQUIRE(rcvd_vehicle_load_json(R"JSON({"schema_version":"2.0"})JSON", &h2.p) == RCVD_ERROR);
    REQUIRE(std::string(rcvd_last_error_code()) == "E-IO-002");

    // §3.3: a positive cornering stiffness is rejected at the boundary too.
    Handle good;
    REQUIRE(rcvd_vehicle_load_json(kCh5Json, &good.p) == RCVD_OK);
    rcvd_derivatives d{};
    REQUIRE(kCF < 0.0);   // the fixture is already negative; negate to make it invalid
    REQUIRE(rcvd_stability_derivatives(good.p, -kCF, kCR, 13.0, &d) == RCVD_ERROR);
    REQUIRE(std::string(rcvd_last_error_code()) == "E-TIRE-003");
    // ...and the valid pair still works, so the guard is not simply rejecting all.
    REQUIRE(rcvd_stability_derivatives(good.p, kCF, kCR, 13.0, &d) == RCVD_OK);
}

// -------------------------------------------------------------------- §18.3
TEST_CASE("18.3: the C API and the C++ API agree bit-for-bit", "[capi][crosscheck]") {
    Handle h;
    REQUIRE(rcvd_vehicle_load_json(kCh5Json, &h.p) == RCVD_OK);

    // The same vehicle through the C++ path.
    const auto cpp = core::resolve(fixtures::ch5_input());
    REQUIRE(cpp.ok());
    const core::Vehicle& v = cpp.value();
    const models::AxleStiffness C{kCF, kCR};

    // Resolved geometry. `==`, not a tolerance.
    struct Field { const char* name; double expected; };
    for (const Field& f : {Field{"m", v.m}, {"I_z", v.I_z}, {"a", v.a}, {"b", v.b},
                           {"l", v.l}, {"h", v.h}, {"W_F", v.W_F()}, {"W_R", v.W_R()},
                           {"g", v.g}, {"steering_ratio", v.steering_ratio},
                           {"front_weight_fraction", v.front_weight_fraction()}}) {
        double got = 0.0;
        INFO("field " << f.name);
        REQUIRE(rcvd_vehicle_get(h.p, f.name, &got) == RCVD_OK);
        REQUIRE(got == f.expected);
    }

    for (double V : {13.0, 20.0, 30.0, 44.0 * units::FOOT}) {
        INFO("speed " << V);

        rcvd_derivatives cd{};
        REQUIRE(rcvd_stability_derivatives(h.p, kCF, kCR, V, &cd) == RCVD_OK);
        const auto pd = models::stability_derivatives(v, C, V);
        REQUIRE(cd.Y_beta == pd.Y_beta);
        REQUIRE(cd.Y_r == pd.Y_r);
        REQUIRE(cd.Y_delta == pd.Y_delta);
        REQUIRE(cd.N_beta == pd.N_beta);
        REQUIRE(cd.N_r == pd.N_r);
        REQUIRE(cd.N_delta == pd.N_delta);

        rcvd_eigen ce{};
        REQUIRE(rcvd_eigenstructure(h.p, kCF, kCR, V, &ce) == RCVD_OK);
        const auto pe = models::eigenstructure(v, C, V);
        REQUIRE(ce.omega_n == pe.omega_n);
        REQUIRE(ce.zeta == pe.zeta);
        REQUIRE(ce.re1 == pe.re1);
        REQUIRE(ce.im1 == pe.im1);
        REQUIRE(ce.trace == pe.trace);
        REQUIRE(ce.det == pe.det);
        REQUIRE((ce.stable != 0) == pe.stable());

        double cg = 0.0;
        REQUIRE(rcvd_yaw_rate_gain(h.p, kCF, kCR, V, &cg) == RCVD_OK);
        const auto pm = models::handling_metrics(v, C);
        REQUIRE(cg == models::yaw_rate_gain(v, pm.K, V));

        for (double delta : {0.01, 0.05, -0.03}) {
            rcvd_steady cs{};
            REQUIRE(rcvd_steady_state(h.p, kCF, kCR, V, delta, &cs) == RCVD_OK);
            const auto ps = models::steady_state(v, C, V, delta);
            REQUIRE(ps.ok());
            REQUIRE(cs.v == ps.value().v_mps);
            REQUIRE(cs.r == ps.value().r_rad_per_s);
            REQUIRE(cs.beta == ps.value().beta_rad);
            REQUIRE(cs.A_Y == ps.value().A_Y_mps2);
            REQUIRE(cs.rcond == ps.value().rcond);
        }
    }

    rcvd_handling ch{};
    REQUIRE(rcvd_handling_metrics(h.p, kCF, kCR, &ch) == RCVD_OK);
    const auto pm = models::handling_metrics(v, C);
    REQUIRE(ch.K == pm.K);
    REQUIRE(ch.K_us == pm.K_us);
    REQUIRE(ch.NSP == pm.NSP);
    REQUIRE(ch.SM == pm.SM);
    REQUIRE(ch.V_tan == pm.V_tan);
    REQUIRE((ch.has_V_ch != 0) == pm.V_ch.has_value());
    REQUIRE(ch.V_ch == pm.V_ch.value_or(0.0));
}

TEST_CASE("18.3: the B-vectors reproduce through the C API", "[capi][acceptance]") {
    Handle h;
    REQUIRE(rcvd_vehicle_load_json(kCh5Json, &h.p) == RCVD_OK);

    rcvd_derivatives d{};
    REQUIRE(rcvd_stability_derivatives(h.p, kCF, kCR, 30.0 * units::MPH, &d) == RCVD_OK);

    // Converted to the published units through the C API's own converter, so the
    // whole path — document, solver, conversion — is exercised in C.
    const auto in_unit = [](double si, const char* from, const char* to) {
        double out = 0.0;
        REQUIRE(rcvd_convert(si, from, to, &out) == RCVD_OK);
        return out;
    };
    REQUIRE_THAT(in_unit(d.Y_beta, "N/rad", "lbf/deg"), WithinRel(-1220.0, 1e-12));
    REQUIRE_THAT(in_unit(d.Y_r, "N*s/rad", "lbf*s/deg"), WithinRel(10.965, 1e-4));
    REQUIRE_THAT(in_unit(d.Y_delta, "N/rad", "lbf/deg"), WithinRel(640.0, 1e-12));
    REQUIRE_THAT(in_unit(d.N_beta, "N*m/rad", "lbf*ft/deg"), WithinRel(482.44, 1e-4));
    REQUIRE_THAT(in_unit(d.N_r, "N*m*s/rad", "lbf*ft*s/deg"), WithinRel(-528.74, 1e-4));
    REQUIRE_THAT(in_unit(d.N_delta, "N*m/rad", "lbf*ft/deg"), WithinRel(2396.53, 1e-4));

    rcvd_handling m{};
    REQUIRE(rcvd_handling_metrics(h.p, kCF, kCR, &m) == RCVD_OK);
    REQUIRE(m.has_V_ch == 1);
    REQUIRE(m.has_V_crit == 0);
    REQUIRE_THAT(in_unit(m.V_ch, "m/s", "ft/s"), WithinRel(170.50, 1e-4));   // B-10
    REQUIRE_THAT(m.NSP, WithinRel(0.47541, 1e-4));                           // B-11
    REQUIRE_THAT(m.SM, WithinRel(0.04541, 1e-4));
}

TEST_CASE("18.3: P6 agrees bit-for-bit, and I-5 holds through C", "[capi][crosscheck]") {
    const double LBF_IN = units::LBF_PER_IN_TO_N_PER_M;
    const double LBF_FT_DEG = units::LBF_FT_PER_DEG_TO_NM_PER_RAD;

    rcvd_ride_roll_request req{};
    req.desired_ride_rate = 163.6 * LBF_IN;
    req.desired_roll_stiffness = 975.0 * LBF_FT_DEG;
    req.tire_rate_per_tire = 1500.0 * LBF_IN;
    req.wheel_track = 60.0 * units::INCH;
    req.spring_track = 60.0 * units::INCH;

    rcvd_ride_roll_components comp{};
    REQUIRE(rcvd_ride_roll_totals_to_components(&req, &comp) == RCVD_OK);
    // R-03 / R-04.
    REQUIRE_THAT(comp.wheel_centre_rate / LBF_IN, WithinRel(183.628, 1e-5));
    REQUIRE_THAT(comp.aux_roll_stiffness / LBF_FT_DEG, WithinRel(816.292, 1e-5));

    // The same call through C++, required to match exactly.
    analysis::TotalsRequest cpp_req;
    cpp_req.desired_ride_rate = req.desired_ride_rate;
    cpp_req.desired_roll_stiffness = req.desired_roll_stiffness;
    cpp_req.tire_rate_per_tire = req.tire_rate_per_tire;
    cpp_req.wheel_track = req.wheel_track;
    cpp_req.spring_track = req.spring_track;
    const auto cpp = analysis::totals_to_components(cpp_req);
    REQUIRE(cpp.ok());
    REQUIRE(comp.wheel_centre_rate == cpp.value().wheel_centre_rate_left);
    REQUIRE(comp.aux_roll_stiffness == cpp.value().aux_roll_stiffness);

    // I-5: direction B then A returns the totals, through the C boundary.
    rcvd_ride_roll_totals back{};
    REQUIRE(rcvd_ride_roll_components_to_totals(&comp, &back) == RCVD_OK);
    REQUIRE_THAT(back.ride_rate, WithinRel(req.desired_ride_rate, 1e-12));
    REQUIRE_THAT(back.roll_stiffness_total, WithinRel(req.desired_roll_stiffness, 1e-12));

    // R-10: the rigid-tire convention (a negative rate means k_T -> infinity).
    rcvd_ride_roll_request rigid{};
    rigid.desired_ride_rate = 150.0 * LBF_IN;
    rigid.desired_roll_stiffness = 1000.0 * LBF_FT_DEG;
    rigid.tire_rate_per_tire = -1.0;
    rigid.wheel_track = rigid.spring_track = 60.0 * units::INCH;
    rcvd_ride_roll_components rc{};
    REQUIRE(rcvd_ride_roll_totals_to_components(&rigid, &rc) == RCVD_OK);
    REQUIRE_THAT(rc.aux_roll_stiffness / LBF_FT_DEG, WithinRel(607.301, 1e-5));

    // E-RR-020 crosses the boundary intact.
    rcvd_ride_roll_request bad = req;
    bad.desired_roll_stiffness = 4000.0 * LBF_FT_DEG;
    REQUIRE(rcvd_ride_roll_totals_to_components(&bad, &rc) == RCVD_ERROR);
    REQUIRE(std::string(rcvd_last_error_code()) == "E-RR-020");
}

TEST_CASE("18.3: the lap solver agrees bit-for-bit", "[capi][crosscheck]") {
    Handle h;
    REQUIRE(rcvd_vehicle_load_json(kCh5Json, &h.p) == RCVD_OK);

    const auto cpp = core::resolve(fixtures::ch5_input());
    REQUIRE(cpp.ok());
    const core::Vehicle& v = cpp.value();
    const models::AxleStiffness C{kCF, kCR};
    const double V = 40.0 * units::FOOT;
    const auto m = models::handling_metrics(v, C);
    const double delta = (v.l / (150.0 * units::FOOT)) * (1.0 + m.K * V * V);

    rcvd_lap cl{};
    REQUIRE(rcvd_lap_solve_bicycle(h.p, kCF, kCR, V, delta, 100.0 * units::FOOT,
                                   400.0 * units::FOOT, 150.0 * units::FOOT, &cl) == RCVD_OK);

    analysis::Circuit circuit;
    circuit.inner_radius = 100.0 * units::FOOT;
    circuit.outer_radius = 400.0 * units::FOOT;
    analysis::DriverPlan plan;
    plan.initial_speed_mps = V;
    plan.steer = models::Maneuver::make_fixed(delta);
    plan.start_x = 150.0 * units::FOOT;
    plan.start_heading_rad = units::PI / 2.0;
    analysis::TransientContext ctx;
    ctx.tau_dominant_s = models::eigenstructure(v, C, V).tau_dominant;
    ctx.steady_state_radius_m = V / (models::yaw_rate_gain(v, m.K, V) * delta);
    analysis::LapConfig cfg;
    cfg.calculation_length_s = 120.0;
    const models::BicyclePlant plant(v, C, V);
    const auto pr = analysis::lap_solve(plant, circuit, plan, cfg, ctx);
    REQUIRE(pr.ok());

    REQUIRE(cl.event == static_cast<int>(pr.value().event));
    REQUIRE(cl.time == pr.value().time_s);          // bit-for-bit, per §18.3
    REQUIRE(cl.steps_accepted == pr.value().stats.accepted);
    REQUIRE(cl.steps_rejected == pr.value().stats.rejected);
}

TEST_CASE("C ABI: warnings survive the boundary", "[capi]") {
    // W-VEH-120: roll-axis inclination beyond (9.4a)'s small-angle assumption.
    const char* tilted = R"JSON({
      "schema_version": "1.0", "name": "tilted",
      "inertia": {"weight": {"value": 3000, "unit": "lbf"}},
      "geometry": {"wheelbase": {"value": 100, "unit": "in"},
                   "front_weight_fraction": 0.5,
                   "cg_height": {"value": 20, "unit": "in"},
                   "track_front": {"value": 60, "unit": "in"},
                   "track_rear": {"value": 60, "unit": "in"},
                   "roll_centre_height_front": {"value": 0, "unit": "in"},
                   "roll_centre_height_rear": {"value": 20, "unit": "in"}}
    })JSON";
    Handle h;
    REQUIRE(rcvd_vehicle_load_json(tilted, &h.p) == RCVD_OK);
    REQUIRE(rcvd_vehicle_warning_count(h.p) >= 1);
    REQUIRE(std::string(rcvd_vehicle_warning_code(h.p, 0)) == "W-VEH-120");
    REQUIRE_FALSE(std::string(rcvd_vehicle_warning_message(h.p, 0)).empty());
    // Out-of-range indices are safe rather than UB.
    REQUIRE(std::string(rcvd_vehicle_warning_code(h.p, 99)).empty());
    REQUIRE(std::string(rcvd_vehicle_warning_code(nullptr, 0)).empty());
}
