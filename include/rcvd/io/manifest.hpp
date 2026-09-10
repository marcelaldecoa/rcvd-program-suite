// rcvd/io/manifest.hpp — §4.5 the run manifest.
//
// "Every run SHALL emit a manifest ... Results without a manifest are not
// reportable." The fully-resolved input document it records is the POST-DEFAULTING,
// POST-UNIT-CONVERSION SI form, which is what makes a result reproducible from the
// manifest alone.
#ifndef RCVD_IO_MANIFEST_HPP
#define RCVD_IO_MANIFEST_HPP

#include <string>
#include <vector>

#include "rcvd/core/vehicle.hpp"
#include "rcvd/io/json.hpp"
#include "rcvd/numerics/integrate.hpp"

namespace rcvd::io {

struct BuildInfo {
    std::string spec_revision = "1.0";
    std::string library_version = "0.1.0";
    std::string git_sha;
    std::string compiler;
    std::string compile_flags;
    std::string platform;
};

struct RunManifest {
    BuildInfo build;
    Json resolved_input;             // §4.5: fully resolved, in SI
    Json solver_config;
    double wall_time_s = 0.0;
    numerics::StepStats integrator{};
    std::string output_sha256;
    std::vector<core::Diagnostic> diagnostics;
    // §9.6 / §9.7: choices the spec requires the manifest to record.
    std::string load_transfer_coupling = "explicit";
    double observed_phi_prime = 0.0;
    double tau_ratio = 0.0;          // §6.9 tau_tire / tau_vehicle
    double gravity_mps2 = units::G_STANDARD;
};

// The resolved SI vehicle, as the manifest records it. Everything here is already
// converted, so there is not a unit string in sight.
[[nodiscard]] inline Json resolved_vehicle_json(const core::Vehicle& v) {
    JsonObject o;
    o.emplace("name", Json(v.name));
    o.emplace("units", Json(std::string("SI throughout: m, kg, s, N, rad")));
    JsonObject inertia;
    inertia.emplace("m_kg", Json(v.m));
    inertia.emplace("I_z_kg_m2", Json(v.I_z));
    inertia.emplace("m_s_kg", Json(v.m_s));
    inertia.emplace("m_uF_kg", Json(v.m_uF));
    inertia.emplace("m_uR_kg", Json(v.m_uR));
    o.emplace("inertia", Json(std::move(inertia)));
    JsonObject geom;
    geom.emplace("a_m", Json(v.a));
    geom.emplace("b_m", Json(v.b));
    geom.emplace("l_m", Json(v.l));
    geom.emplace("h_m", Json(v.h));
    geom.emplace("h_s_m", Json(v.h_s));
    geom.emplace("a_s_m", Json(v.a_s));
    geom.emplace("t_F_m", Json(v.t_F));
    geom.emplace("t_R_m", Json(v.t_R));
    geom.emplace("t_sF_m", Json(v.t_sF));
    geom.emplace("t_sR_m", Json(v.t_sR));
    geom.emplace("z_RF_m", Json(v.z_RF));
    geom.emplace("z_RR_m", Json(v.z_RR));
    geom.emplace("z_WF_m", Json(v.z_WF));
    geom.emplace("z_WR_m", Json(v.z_WR));
    o.emplace("geometry", Json(std::move(geom)));
    if (v.has_springing()) {
        JsonObject spr;
        spr.emplace("K_phi_F_Nm_per_rad", Json(*v.K_phi_F));
        spr.emplace("K_phi_R_Nm_per_rad", Json(*v.K_phi_R));
        spr.emplace("roll_gradient_deg_per_g", Json(v.roll_gradient_deg_per_g()));
        o.emplace("springing", Json(std::move(spr)));
    }
    o.emplace("steering_ratio", Json(v.steering_ratio));
    o.emplace("gravity_mps2", Json(v.g));
    return Json(std::move(o));
}

[[nodiscard]] inline Json manifest_json(const RunManifest& m) {
    JsonObject o;
    JsonObject build;
    build.emplace("spec_revision", Json(m.build.spec_revision));
    build.emplace("library_version", Json(m.build.library_version));
    build.emplace("git_sha", Json(m.build.git_sha));
    build.emplace("compiler", Json(m.build.compiler));
    build.emplace("compile_flags", Json(m.build.compile_flags));
    build.emplace("platform", Json(m.build.platform));
    o.emplace("build", Json(std::move(build)));

    o.emplace("resolved_input", m.resolved_input);
    o.emplace("solver_config", m.solver_config);
    o.emplace("wall_time_s", Json(m.wall_time_s));

    JsonObject stats;
    stats.emplace("steps_accepted", Json(static_cast<double>(m.integrator.accepted)));
    stats.emplace("steps_rejected", Json(static_cast<double>(m.integrator.rejected)));
    stats.emplace("h_min_s", Json(m.integrator.h_min));
    stats.emplace("h_max_s", Json(m.integrator.h_max));
    stats.emplace("error_norm_max", Json(m.integrator.err_max));
    o.emplace("integrator", Json(std::move(stats)));

    o.emplace("output_sha256", Json(m.output_sha256));
    o.emplace("load_transfer_coupling", Json(m.load_transfer_coupling));
    o.emplace("observed_phi_prime", Json(m.observed_phi_prime));
    o.emplace("tau_tire_over_tau_vehicle", Json(m.tau_ratio));
    // Rule U4: a non-default gravity SHALL appear in the manifest and on every
    // plot caption, so it can never be used quietly to absorb a discrepancy.
    o.emplace("gravity_mps2", Json(m.gravity_mps2));
    o.emplace("gravity_is_standard", Json(m.gravity_mps2 == units::G_STANDARD));

    JsonArray diags;
    for (const core::Diagnostic& d : m.diagnostics) {
        JsonObject e;
        e.emplace("code", Json(std::string(d.code)));
        e.emplace("message", Json(d.message));
        if (d.t_s) e.emplace("t_s", Json(*d.t_s));
        if (d.wheel) e.emplace("wheel", Json(std::string(core::to_string(*d.wheel))));
        diags.push_back(Json(std::move(e)));
    }
    o.emplace("diagnostics", Json(std::move(diags)));
    return Json(std::move(o));
}

}  // namespace rcvd::io

#endif  // RCVD_IO_MANIFEST_HPP
