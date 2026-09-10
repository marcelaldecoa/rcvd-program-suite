// rcvd/core/vehicle.hpp — the Vehicle value type and its validation, spec §5.1.
//
// This is the FULLY RESOLVED, SI form: post-defaulting, post-unit-conversion,
// exactly what §4.5 requires the run manifest to record. JSON deserialisation
// sits above it and is not yet implemented.
//
// §14.1 Rule D3: validation happens once, here, at construction. Solvers do not
// re-validate per step; they assert.
#ifndef RCVD_CORE_VEHICLE_HPP
#define RCVD_CORE_VEHICLE_HPP

#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include "rcvd/core/result.hpp"
#include "rcvd/units/constants.hpp"

namespace rcvd::core {

// As supplied. Several quantities can be given more than one way; the redundancy
// rule of §5.1 is that the schema accepts EXACTLY ONE form per group and rejects
// documents that over-specify (E-VEH-010). Silent precedence is forbidden.
struct VehicleInput {
    std::string name;

    // ---- inertia. Exactly one of mass / weight. -----------------------------
    std::optional<double> mass_kg;
    std::optional<double> weight_N;
    double yaw_inertia_kg_m2 = 0.0;

    // Per AXLE, not per wheel — §5.1 is explicit, and the field names say so.
    double unsprung_mass_front_kg = 0.0;
    double unsprung_mass_rear_kg  = 0.0;
    double unsprung_cg_height_front_m = 0.0;
    double unsprung_cg_height_rear_m  = 0.0;

    // ---- geometry. Exactly one of {wheelbase, front_weight_fraction} / {a, b}.
    std::optional<double> wheelbase_m;
    std::optional<double> front_weight_fraction;
    std::optional<double> a_m;  // CG to front axle
    std::optional<double> b_m;  // CG to rear axle

    double track_front_m = 0.0;
    double track_rear_m  = 0.0;
    double spring_track_front_m = 0.0;
    double spring_track_rear_m  = 0.0;
    double cg_height_m = 0.0;                 // TOTAL vehicle CG

    // Optional; derived from (5.1b)/(5.1c) when omitted, checked when supplied.
    std::optional<double> sprung_cg_height_m;          // h_s
    std::optional<double> sprung_cg_front_distance_m;  // a_s

    double roll_centre_height_front_m = 0.0;
    double roll_centre_height_rear_m  = 0.0;

    // ---- springing. Chassis-to-ground (§9.3 decision D-5). Optional: P1 has no
    // roll degree of freedom and must not require them.
    std::optional<double> roll_stiffness_ctg_front_Nm_per_rad;
    std::optional<double> roll_stiffness_ctg_rear_Nm_per_rad;

    double steering_ratio = 1.0;              // G_s = delta_SW / delta
    double gravity_mps2 = units::G_STANDARD;  // §5.4; Rule U4 forbids abusing this
};

struct Vehicle {
    std::string name;

    double m{};      // total mass
    double I_z{};    // yaw inertia about the CG
    double a{};      // CG to front axle
    double b{};      // CG to rear axle
    double l{};      // wheelbase
    double h{};      // total CG height

    double m_s{};    // sprung mass
    double h_s{};    // sprung CG height
    double a_s{};    // front axle to sprung CG
    double m_uF{}, m_uR{};
    double z_WF{}, z_WR{};

    double t_F{}, t_R{};
    double t_sF{}, t_sR{};
    double z_RF{}, z_RR{};

    std::optional<double> K_phi_F, K_phi_R;  // chassis-to-ground roll stiffness

    double steering_ratio{};
    double g{};

    // Warnings raised during resolution. §14.1: a W- diagnostic does not fail the
    // Result; it travels with a successful one and callers SHALL surface it.
    std::vector<Diagnostic> warnings;

    [[nodiscard]] double W() const { return m * g; }
    // Static axle loads, (9.1). Note the cross-over: the FRONT load is set by b.
    [[nodiscard]] double W_F() const { return m * g * b / l; }
    [[nodiscard]] double W_R() const { return m * g * a / l; }
    [[nodiscard]] double front_weight_fraction() const { return b / l; }

    // (9.4a): roll-axis height beneath the sprung CG.
    [[nodiscard]] double z_R_eff() const {
        return (1.0 - a_s / l) * z_RF + (a_s / l) * z_RR;
    }
    // (9.4b): perpendicular distance, sprung CG to roll axis.
    [[nodiscard]] double h2() const { return h_s - z_R_eff(); }

    [[nodiscard]] bool has_springing() const { return K_phi_F && K_phi_R; }
    [[nodiscard]] double sum_K_phi() const { return *K_phi_F + *K_phi_R; }

    // (12.10) / (9.5): roll gradient, rad per (m/s^2), defined on the outward-lean
    // magnitude-with-sign phi_tilde. One definition, used in both places.
    [[nodiscard]] double roll_gradient_rad_per_mps2() const {
        return m_s * h2() / (sum_K_phi() - m_s * g * h2());
    }
    [[nodiscard]] double roll_gradient_deg_per_g() const {
        return roll_gradient_rad_per_mps2() * g / units::DEGREE;
    }
};

// Resolve and validate. Returns E-VEH-010 / E-VEH-011 / E-VEH-140 on failure.
[[nodiscard]] inline Result<Vehicle> resolve(const VehicleInput& in) {
    const std::string who = in.name.empty() ? std::string("<unnamed>") : in.name;
    Vehicle v;
    v.name = who;
    v.g = in.gravity_mps2;

    // ---- mass / weight ------------------------------------------------------
    if (in.mass_kg.has_value() == in.weight_N.has_value()) {
        return err<Vehicle>("E-VEH-010",
                            "vehicle '" + who + "': supply exactly one of mass or weight");
    }
    v.m = in.mass_kg ? *in.mass_kg : *in.weight_N / v.g;
    if (!(v.m > 0.0)) {
        return err<Vehicle>("E-VEH-010", "vehicle '" + who + "': mass must be positive");
    }
    v.I_z = in.yaw_inertia_kg_m2;

    // ---- geometry -----------------------------------------------------------
    const bool wb_form = in.wheelbase_m.has_value() && in.front_weight_fraction.has_value();
    const bool ab_form = in.a_m.has_value() && in.b_m.has_value();
    if (wb_form == ab_form) {
        return err<Vehicle>(
            "E-VEH-010",
            "vehicle '" + who + "': supply exactly one of {wheelbase, front_weight_fraction} "
            "or {a, b}");
    }
    if (wb_form) {
        // §5.1's derived-quantity rule, and the single most frequently inverted
        // relation in vehicle-dynamics code: a car with 57 % front weight has its
        // CG CLOSER to the front axle, so a = 0.43*l, not 0.57*l.
        const double f = *in.front_weight_fraction;
        if (!(f > 0.0 && f < 1.0)) {
            return err<Vehicle>("E-VEH-010",
                                "vehicle '" + who + "': front_weight_fraction must be in (0,1)");
        }
        v.l = *in.wheelbase_m;
        v.a = v.l * (1.0 - f);
        v.b = v.l * f;
    } else {
        v.a = *in.a_m;
        v.b = *in.b_m;
        v.l = v.a + v.b;
    }
    if (!(v.l > 0.0 && v.a > 0.0 && v.b > 0.0)) {
        return err<Vehicle>("E-VEH-010", "vehicle '" + who + "': wheelbase and both CG "
                                         "distances must be positive");
    }

    v.h    = in.cg_height_m;
    v.t_F  = in.track_front_m;
    v.t_R  = in.track_rear_m;
    v.t_sF = in.spring_track_front_m > 0.0 ? in.spring_track_front_m : in.track_front_m;
    v.t_sR = in.spring_track_rear_m  > 0.0 ? in.spring_track_rear_m  : in.track_rear_m;
    v.z_RF = in.roll_centre_height_front_m;
    v.z_RR = in.roll_centre_height_rear_m;

    // ---- sprung / unsprung consistency, (5.1a)-(5.1c) ----------------------
    v.m_uF = in.unsprung_mass_front_kg;
    v.m_uR = in.unsprung_mass_rear_kg;
    v.z_WF = in.unsprung_cg_height_front_m;
    v.z_WR = in.unsprung_cg_height_rear_m;

    v.m_s = v.m - v.m_uF - v.m_uR;  // (5.1a). There is no sprung_mass_fraction field.
    if (!(v.m_s > 0.0)) {
        return err<Vehicle>("E-VEH-011",
                            "vehicle '" + who + "': unsprung masses (" +
                                std::to_string(v.m_uF + v.m_uR) +
                                " kg) equal or exceed the total mass");
    }

    // (5.1b) vertical moment: m*h = m_s*h_s + m_uF*z_WF + m_uR*z_WR
    const double h_s_derived = (v.m * v.h - v.m_uF * v.z_WF - v.m_uR * v.z_WR) / v.m_s;
    // (5.1c) longitudinal moment: m*a = m_s*a_s + m_uR*l, measured from the FRONT
    // axle, so the front unsprung mass sits at x = 0 and contributes nothing.
    const double a_s_derived = (v.m * v.a - v.m_uR * v.l) / v.m_s;

    constexpr double kConsistencyRelTol = 1e-9;  // §5.1
    const auto check_supplied = [&](const char* field, double supplied,
                                    double derived) -> std::optional<Diagnostic> {
        const double scale = std::max(1.0, std::abs(derived));
        if (std::abs(supplied - derived) > kConsistencyRelTol * scale) {
            return make_diagnostic(
                "E-VEH-011",
                "vehicle '" + who + "': supplied " + field + " = " + std::to_string(supplied) +
                    " m contradicts the value " + std::to_string(derived) +
                    " m required by the sprung/unsprung moment balance");
        }
        return std::nullopt;
    };

    if (in.sprung_cg_height_m) {
        if (auto d = check_supplied("sprung_cg_height", *in.sprung_cg_height_m, h_s_derived)) {
            return Result<Vehicle>(*d);
        }
        v.h_s = *in.sprung_cg_height_m;
    } else {
        v.h_s = h_s_derived;
    }

    if (in.sprung_cg_front_distance_m) {
        // Leaving a_s free is a common and quiet source of load-transfer error:
        // it sets the front/rear split of the sprung weight in (9.6a)-(9.6b).
        if (auto d = check_supplied("sprung_cg_front_distance", *in.sprung_cg_front_distance_m,
                                    a_s_derived)) {
            return Result<Vehicle>(*d);
        }
        v.a_s = *in.sprung_cg_front_distance_m;
    } else {
        v.a_s = a_s_derived;
    }

    // ---- springing ----------------------------------------------------------
    const bool kf = in.roll_stiffness_ctg_front_Nm_per_rad.has_value();
    const bool kr = in.roll_stiffness_ctg_rear_Nm_per_rad.has_value();
    if (kf != kr) {
        return err<Vehicle>("E-VEH-010",
                            "vehicle '" + who + "': roll stiffness must be given for both axles "
                            "or neither");
    }
    if (kf) {
        v.K_phi_F = *in.roll_stiffness_ctg_front_Nm_per_rad;
        v.K_phi_R = *in.roll_stiffness_ctg_rear_Nm_per_rad;

        // E-VEH-140: the effective roll stiffness is SumK - m_s g h2 (9.5). As it
        // approaches zero the vehicle is roll-divergent. Both terms must be in
        // N*m/rad — this is where a lbf*ft/deg conversion error announces itself.
        const double destabilising = v.m_s * v.g * v.h2();
        if (v.sum_K_phi() <= 1.05 * destabilising) {
            return err<Vehicle>(
                "E-VEH-140",
                "vehicle '" + who + "': roll divergence — total roll stiffness " +
                    std::to_string(v.sum_K_phi()) + " N*m/rad is not above 1.05 * m_s g h2 = " +
                    std::to_string(1.05 * destabilising) + " N*m/rad");
        }

        // W-VEH-141 / I-3c: an absolute scale check. Neither I-3 nor I-3b detects a
        // CONSISTENT unit error in the roll stiffnesses, because both read K_phi
        // from the same resolved document the model does. This band does.
        const double rg = v.roll_gradient_deg_per_g();
        if (rg < 0.1 || rg > 12.0) {
            v.warnings.push_back(make_diagnostic(
                "W-VEH-141",
                "vehicle '" + who + "': roll gradient " + std::to_string(rg) +
                    " deg/g is outside the physical band [0.1, 12.0] of (9.10c); suspect a "
                    "lbf*ft/deg <-> N*m/rad error in the roll stiffnesses"));
        }
    }

    // W-VEH-120: (9.4a) small-angle roll-axis approximation starts to bite.
    if (v.l > 0.0 && std::abs(v.z_RR - v.z_RF) / v.l > 0.05) {
        v.warnings.push_back(make_diagnostic(
            "W-VEH-120",
            "vehicle '" + who + "': roll-axis inclination |z_RR - z_RF|/l = " +
                std::to_string(std::abs(v.z_RR - v.z_RF) / v.l) +
                " exceeds 0.05, beyond the small-angle assumption of (9.4a)"));
    }

    v.steering_ratio = in.steering_ratio;
    return v;
}

}  // namespace rcvd::core

#endif  // RCVD_CORE_VEHICLE_HPP
