// src/capi/rcvd_capi.cpp — the extern "C" ABI implementation, spec §4.1, §4.6.
//
// Rule A3: the core never throws, so nothing here catches. Every entry point is
// noexcept, which makes an unrecoverable defect (bad_alloc, a failed assertion)
// terminate at the boundary rather than unwind through C — the honest behaviour,
// and what §4.6 means by "not recoverable".
//
// Rule A1 / §18.3: this must produce results IDENTICAL to the C++ API, bit for
// bit. It therefore does no arithmetic of its own; every function marshals and
// delegates.
#include "rcvd/capi/rcvd.h"

#include <cmath>
#include <cstring>
#include <limits>
#include <new>
#include <string>
#include <vector>

#include "rcvd/analysis/lap_solver.hpp"
#include "rcvd/analysis/ride_roll.hpp"
#include "rcvd/core/vehicle.hpp"
#include "rcvd/io/document.hpp"
#include "rcvd/io/json.hpp"
#include "rcvd/models/bicycle.hpp"
#include "rcvd/models/four_wheeler.hpp"
#include "rcvd/units/convert.hpp"

namespace {

// Thread-local, so a failing call on one thread cannot clobber another's
// diagnosis. This is the C spelling of Result<T>'s error half.
thread_local std::string g_error_code;
thread_local std::string g_error_message;

void clear_error() noexcept {
    g_error_code.clear();
    g_error_message.clear();
}

rcvd_status set_error(const rcvd::core::Diagnostic& d) noexcept {
    g_error_code.assign(d.code.begin(), d.code.end());
    g_error_message = d.message;
    return RCVD_ERROR;
}

rcvd_status set_error(const char* code, const char* message) noexcept {
    g_error_code = code;
    g_error_message = message;
    return RCVD_ERROR;
}

// The C header's rigid-tire convention: a negative rate means k_T -> infinity.
double tire_rate(double v) noexcept {
    return v < 0.0 ? std::numeric_limits<double>::infinity() : v;
}

}  // namespace

struct rcvd_vehicle {
    rcvd::core::Vehicle v;
};

// ---------------------------------------------------------------- version
void rcvd_version(int* major, int* minor, int* patch) noexcept {
    if (major) *major = 0;
    if (minor) *minor = 1;
    if (patch) *patch = 0;
}

const char* rcvd_spec_revision(void) noexcept { return "1.0"; }

int rcvd_abi_version(void) noexcept { return 1; }

// ---------------------------------------------------------------- errors
const char* rcvd_last_error_code(void) noexcept { return g_error_code.c_str(); }
const char* rcvd_last_error_message(void) noexcept { return g_error_message.c_str(); }

// ---------------------------------------------------------------- units
rcvd_status rcvd_convert(double value, const char* from_unit, const char* to_unit,
                         double* out) noexcept {
    clear_error();
    if (!from_unit || !to_unit || !out) {
        return set_error("E-IO-003", "null argument to rcvd_convert");
    }
    const auto from = rcvd::units::lookup(from_unit);
    const auto to = rcvd::units::lookup(to_unit);
    if (!from) {
        return set_error("E-IO-003",
                         (std::string("unit '") + from_unit +
                          "' is not in the closed vocabulary").c_str());
    }
    if (!to) {
        return set_error("E-IO-003",
                         (std::string("unit '") + to_unit +
                          "' is not in the closed vocabulary").c_str());
    }
    if (from->dim != to->dim) {
        return set_error("E-IO-003",
                         (std::string("cannot convert '") + from_unit + "' to '" + to_unit +
                          "': different dimensions").c_str());
    }
    *out = value * from->to_si / to->to_si;
    return RCVD_OK;
}

// ---------------------------------------------------------------- vehicle
rcvd_status rcvd_vehicle_load_json(const char* json, rcvd_vehicle** out) noexcept {
    clear_error();
    if (!json || !out) return set_error("E-IO-003", "null argument");
    *out = nullptr;

    const auto doc = rcvd::io::parse_json(json);
    if (!doc.ok()) return set_error(doc.error());
    const auto in = rcvd::io::load_vehicle(doc.value());
    if (!in.ok()) return set_error(in.error());
    const auto resolved = rcvd::core::resolve(in.value());
    if (!resolved.ok()) return set_error(resolved.error());

    auto* h = new rcvd_vehicle{resolved.value()};
    *out = h;
    return RCVD_OK;
}

void rcvd_vehicle_free(rcvd_vehicle* v) noexcept { delete v; }

rcvd_status rcvd_vehicle_get(const rcvd_vehicle* v, const char* field, double* out) noexcept {
    clear_error();
    if (!v || !field || !out) return set_error("E-IO-003", "null argument");
    const rcvd::core::Vehicle& q = v->v;
    const std::string f = field;

    if (f == "m") *out = q.m;
    else if (f == "I_z") *out = q.I_z;
    else if (f == "a") *out = q.a;
    else if (f == "b") *out = q.b;
    else if (f == "l") *out = q.l;
    else if (f == "h") *out = q.h;
    else if (f == "m_s") *out = q.m_s;
    else if (f == "h_s") *out = q.h_s;
    else if (f == "a_s") *out = q.a_s;
    else if (f == "t_F") *out = q.t_F;
    else if (f == "t_R") *out = q.t_R;
    else if (f == "W") *out = q.W();
    else if (f == "W_F") *out = q.W_F();
    else if (f == "W_R") *out = q.W_R();
    else if (f == "g") *out = q.g;
    else if (f == "steering_ratio") *out = q.steering_ratio;
    else if (f == "front_weight_fraction") *out = q.front_weight_fraction();
    else if (f == "roll_gradient_deg_per_g") {
        if (!q.has_springing()) {
            return set_error("E-VEH-010", "vehicle has no springing, so no roll gradient");
        }
        *out = q.roll_gradient_deg_per_g();
    } else {
        return set_error("E-IO-003", (std::string("unknown field '") + field + "'").c_str());
    }
    return RCVD_OK;
}

int rcvd_vehicle_warning_count(const rcvd_vehicle* v) noexcept {
    return v ? static_cast<int>(v->v.warnings.size()) : 0;
}

const char* rcvd_vehicle_warning_code(const rcvd_vehicle* v, int index) noexcept {
    if (!v || index < 0 || index >= static_cast<int>(v->v.warnings.size())) return "";
    static thread_local std::string buf;
    const auto& c = v->v.warnings[static_cast<std::size_t>(index)].code;
    buf.assign(c.begin(), c.end());
    return buf.c_str();
}

const char* rcvd_vehicle_warning_message(const rcvd_vehicle* v, int index) noexcept {
    if (!v || index < 0 || index >= static_cast<int>(v->v.warnings.size())) return "";
    return v->v.warnings[static_cast<std::size_t>(index)].message.c_str();
}

// ---------------------------------------------------------------- P1
rcvd_status rcvd_stability_derivatives(const rcvd_vehicle* v, double CF, double CR, double V,
                                       rcvd_derivatives* out) noexcept {
    clear_error();
    if (!v || !out) return set_error("E-IO-003", "null argument");
    if (CF >= 0.0 || CR >= 0.0) {
        return set_error("E-TIRE-003", "axle cornering stiffness must be negative (§3.3)");
    }
    if (!(V > 0.0)) return set_error("E-NUM-310", "speed must be positive");

    const auto d = rcvd::models::stability_derivatives(v->v, {CF, CR}, V);
    out->Y_beta = d.Y_beta;
    out->Y_r = d.Y_r;
    out->Y_delta = d.Y_delta;
    out->N_beta = d.N_beta;
    out->N_r = d.N_r;
    out->N_delta = d.N_delta;
    return RCVD_OK;
}

rcvd_status rcvd_handling_metrics(const rcvd_vehicle* v, double CF, double CR,
                                  rcvd_handling* out) noexcept {
    clear_error();
    if (!v || !out) return set_error("E-IO-003", "null argument");
    if (CF >= 0.0 || CR >= 0.0) {
        return set_error("E-TIRE-003", "axle cornering stiffness must be negative (§3.3)");
    }
    const auto m = rcvd::models::handling_metrics(v->v, {CF, CR});
    out->K = m.K;
    out->K_us = m.K_us;
    out->NSP = m.NSP;
    out->SM = m.SM;
    out->V_tan = m.V_tan;
    out->has_V_ch = m.V_ch.has_value() ? 1 : 0;
    out->has_V_crit = m.V_crit.has_value() ? 1 : 0;
    out->V_ch = m.V_ch.value_or(0.0);
    out->V_crit = m.V_crit.value_or(0.0);
    return RCVD_OK;
}

rcvd_status rcvd_steady_state(const rcvd_vehicle* v, double CF, double CR, double V,
                              double delta, rcvd_steady* out) noexcept {
    clear_error();
    if (!v || !out) return set_error("E-IO-003", "null argument");
    const auto r = rcvd::models::steady_state(v->v, {CF, CR}, V, delta);
    if (!r.ok()) return set_error(r.error());
    out->v = r.value().v_mps;
    out->r = r.value().r_rad_per_s;
    out->beta = r.value().beta_rad;
    out->A_Y = r.value().A_Y_mps2;
    out->rcond = r.value().rcond;
    return RCVD_OK;
}

rcvd_status rcvd_eigenstructure(const rcvd_vehicle* v, double CF, double CR, double V,
                                rcvd_eigen* out) noexcept {
    clear_error();
    if (!v || !out) return set_error("E-IO-003", "null argument");
    if (!(V > 0.0)) return set_error("E-NUM-310", "speed must be positive");
    const auto e = rcvd::models::eigenstructure(v->v, {CF, CR}, V);
    out->omega_n = e.omega_n;
    out->zeta = e.zeta;
    out->re1 = e.re1;
    out->im1 = e.im1;
    out->re2 = e.re2;
    out->im2 = e.im2;
    out->tau_dominant = e.tau_dominant;
    out->trace = e.trace;
    out->det = e.det;
    out->stable = e.stable() ? 1 : 0;
    out->oscillatory = e.oscillatory() ? 1 : 0;
    return RCVD_OK;
}

rcvd_status rcvd_yaw_rate_gain(const rcvd_vehicle* v, double CF, double CR, double V,
                               double* out) noexcept {
    clear_error();
    if (!v || !out) return set_error("E-IO-003", "null argument");
    if (!(V > 0.0)) return set_error("E-NUM-310", "speed must be positive");
    const auto m = rcvd::models::handling_metrics(v->v, {CF, CR});
    *out = rcvd::models::yaw_rate_gain(v->v, m.K, V);
    return RCVD_OK;
}

// ---------------------------------------------------------------- P6
rcvd_status rcvd_ride_roll_components_to_totals(const rcvd_ride_roll_components* in,
                                                rcvd_ride_roll_totals* out) noexcept {
    clear_error();
    if (!in || !out) return set_error("E-IO-003", "null argument");
    rcvd::analysis::AxleSpringing s;
    s.tire_rate_per_tire = tire_rate(in->tire_rate_per_tire);
    s.wheel_track = in->wheel_track;
    s.spring_track = in->spring_track;
    s.wheel_centre_rate_left = s.wheel_centre_rate_right = in->wheel_centre_rate;
    s.aux_roll_stiffness = in->aux_roll_stiffness;
    s.axle_weight_N = in->axle_weight;

    const auto t = rcvd::analysis::components_to_totals(s);
    if (!t.ok()) return set_error(t.error());
    out->ride_rate = t.value().ride_rate;
    out->roll_stiffness_tire = t.value().roll_stiffness_tire;
    out->roll_stiffness_springs = t.value().roll_stiffness_springs;
    out->roll_stiffness_chassis_to_wc = t.value().roll_stiffness_chassis_to_wc;
    out->roll_stiffness_total = t.value().roll_stiffness_total;
    out->ride_frequency_Hz = t.value().ride_frequency_Hz;
    return RCVD_OK;
}

rcvd_status rcvd_ride_roll_totals_to_components(const rcvd_ride_roll_request* in,
                                                rcvd_ride_roll_components* out) noexcept {
    clear_error();
    if (!in || !out) return set_error("E-IO-003", "null argument");
    rcvd::analysis::TotalsRequest req;
    req.desired_ride_rate = in->desired_ride_rate;
    req.desired_roll_stiffness = in->desired_roll_stiffness;
    req.tire_rate_per_tire = tire_rate(in->tire_rate_per_tire);
    req.wheel_track = in->wheel_track;
    req.spring_track = in->spring_track;
    req.axle_weight_N = in->axle_weight;

    const auto c = rcvd::analysis::totals_to_components(req);
    if (!c.ok()) return set_error(c.error());
    out->tire_rate_per_tire = in->tire_rate_per_tire;
    out->wheel_track = c.value().wheel_track;
    out->spring_track = c.value().spring_track;
    out->wheel_centre_rate = c.value().wheel_centre_rate_left;
    out->aux_roll_stiffness = c.value().aux_roll_stiffness;
    out->axle_weight = c.value().axle_weight_N;
    return RCVD_OK;
}

// ---------------------------------------------------------------- P4
rcvd_status rcvd_lap_solve_bicycle(const rcvd_vehicle* v, double CF, double CR, double V,
                                   double delta, double inner_radius, double outer_radius,
                                   double start_radius, rcvd_lap* out) noexcept {
    clear_error();
    if (!v || !out) return set_error("E-IO-003", "null argument");
    if (!(V > 0.0)) return set_error("E-NUM-310", "speed must be positive");

    rcvd::analysis::Circuit circuit;
    circuit.inner_radius = inner_radius;
    circuit.outer_radius = outer_radius > 0.0 ? outer_radius
                                              : std::numeric_limits<double>::infinity();

    rcvd::analysis::DriverPlan plan;
    plan.initial_speed_mps = V;
    plan.steer = rcvd::models::Maneuver::make_fixed(delta);
    plan.start_x = start_radius;
    plan.start_heading_rad = rcvd::units::PI / 2.0;

    const rcvd::models::AxleStiffness C{CF, CR};
    const rcvd::models::BicyclePlant plant(v->v, C, V);
    const auto m = rcvd::models::handling_metrics(v->v, C);

    rcvd::analysis::TransientContext ctx;
    ctx.tau_dominant_s = rcvd::models::eigenstructure(v->v, C, V).tau_dominant;
    ctx.steady_state_radius_m =
        delta != 0.0 ? V / (rcvd::models::yaw_rate_gain(v->v, m.K, V) * delta)
                     : std::numeric_limits<double>::infinity();

    rcvd::analysis::LapConfig cfg;
    cfg.calculation_length_s = 120.0;
    const auto r = rcvd::analysis::lap_solve(plant, circuit, plan, cfg, ctx);
    if (!r.ok()) return set_error(r.error());

    out->event = static_cast<int>(r.value().event);
    out->time = r.value().time_s;
    out->steps_accepted = r.value().stats.accepted;
    out->steps_rejected = r.value().stats.rejected;
    out->hint_fired = 0;
    for (const auto& d : r.value().diagnostics) {
        if (d.code == "H-SPD-001") out->hint_fired = 1;
    }
    return RCVD_OK;
}
