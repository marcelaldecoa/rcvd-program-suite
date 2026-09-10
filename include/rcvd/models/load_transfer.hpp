// rcvd/models/load_transfer.hpp — spec §9.3-9.5.
//
// "This is the numerically delicate part of P2 and is specified in full."
//
// The roll angle used throughout is phi_tilde: the OUTWARD lean, magnitude with
// sign, positive for positive A_Y and ODD in A_Y. The SAE roll angle of §3.3 is
// phi = -phi_tilde. Using |phi| would make the transfer EVEN in A_Y, so a
// symmetric vehicle would transfer different loads turning left and right
// (deviation D-4).
#ifndef RCVD_MODELS_LOAD_TRANSFER_HPP
#define RCVD_MODELS_LOAD_TRANSFER_HPP

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "rcvd/core/result.hpp"
#include "rcvd/core/vehicle.hpp"

namespace rcvd::models {

// §19 compatibility switches. Every deviation is deliberate, justified, tested
// and flagged; nothing is a silent "improvement".
struct LegacyLoadTransfer {
    // E-1a: the published gravity term SUBTRACTS from the transfer. As printed it
    // violates I-3 — the sprung CG displaces toward the outside wheels as the body
    // rolls, which must INCREASE the load there.
    bool gravity_sign = false;
    // E-1b: the published treatment distributes the gravity moment by the
    // SPRUNG-WEIGHT fraction while distributing the spring moment by K_phi/SumK.
    // Those two ratios are not equal in general.
    bool gravity_distribution = false;

    [[nodiscard]] bool any() const { return gravity_sign || gravity_distribution; }
};

struct AxleTransfer {
    double transfer_N{};   // (9.8) added to the outside wheel, subtracted from inside
    double flt{};          // (9.9) = 2*transfer/F_z,axle; 1.0 is exactly wheel lift
};

struct WheelLoads {
    double LF{}, RF{}, LR{}, RR{};
    double phi_tilde_rad{};      // (9.5) outward lean, odd in A_Y
    double roll_angle_rad{};     // SAE phi = -phi_tilde (§3.3)
    AxleTransfer front{}, rear{};
    double static_front_N{}, static_rear_N{};
    double aero_front_N{}, aero_rear_N{};
    double longitudinal_transfer_N{};   // (9.3)
    std::vector<core::Diagnostic> events;  // EV-LIFT, per Rule D1
};

struct LoadTransferInputs {
    double A_Y_mps2 = 0.0;
    double A_X_mps2 = 0.0;             // (9.12a); the achieved value, not commanded
    double aero_downforce_front_N = 0.0;
    double aero_downforce_rear_N = 0.0;
    double time_s = 0.0;               // for event stamping
};

// (9.5). Also (12.10) — one definition, two places, no sign discrepancy.
[[nodiscard]] inline double roll_angle_tilde(const core::Vehicle& v, double A_Y_mps2) {
    return v.m_s * v.h2() * A_Y_mps2 / (v.sum_K_phi() - v.m_s * v.g * v.h2());
}

// (9.6a)/(9.6b), plus the §19 legacy variants.
//
// WHY THIS FORM. The published treatment splits the sprung-mass contribution into
// a spring term distributed by K_phi/SumK plus a separate gravity term distributed
// by the SPRUNG-WEIGHT fraction. The split is inconsistent: the load transferred
// through an axle's springs is K_phi,axle*phi_tilde/t BY DEFINITION of a roll
// spring, whatever the origin of the moment that produced phi_tilde. The single
// term below is that quantity exactly, and it carries the gravity effect
// automatically through the reduced denominator of (9.5).
[[nodiscard]] inline double axle_transfer_N(const core::Vehicle& v, double A_Y_mps2,
                                            double phi_tilde, bool front,
                                            LegacyLoadTransfer legacy = {}) {
    const double K_phi   = front ? *v.K_phi_F : *v.K_phi_R;
    const double t       = front ? v.t_F : v.t_R;
    const double z_R     = front ? v.z_RF : v.z_RR;
    const double m_u     = front ? v.m_uF : v.m_uR;
    const double z_W     = front ? v.z_WF : v.z_WR;
    // Sprung-weight fraction on this axle: (1 - a_s/l) at the front.
    const double sprung_fraction = front ? (1.0 - v.a_s / v.l) : (v.a_s / v.l);

    // The geometric ("jacking") term reacted through the suspension links at the
    // roll centre, and the unsprung mass acting at its own CG height. Common to
    // both the correct and the legacy forms.
    const double geometric = sprung_fraction * v.m_s * z_R * A_Y_mps2;
    const double unsprung  = m_u * z_W * A_Y_mps2;

    double sprung_moment;
    if (!legacy.any()) {
        // (9.6a): the whole sprung contribution in one term.
        sprung_moment = K_phi * phi_tilde;
    } else {
        // Expanding K_phi*phi_tilde exposes the published decomposition:
        //   K_phi*phi_tilde = m_s*h2*(K_phi/SumK)*A_Y + m_s*h2*(K_phi/SumK)*g*phi_tilde
        // so the gravity contribution is distributed by K_phi/SumK, NOT by the
        // sprung-weight fraction, and it ADDS rather than subtracts.
        const double stiffness_ratio = K_phi / v.sum_K_phi();
        const double spring_term = v.m_s * v.h2() * stiffness_ratio * A_Y_mps2;
        const double gravity_ratio =
            legacy.gravity_distribution ? sprung_fraction : stiffness_ratio;
        const double gravity_sign = legacy.gravity_sign ? -1.0 : +1.0;
        const double gravity_term =
            gravity_sign * v.m_s * v.h2() * gravity_ratio * v.g * phi_tilde;
        sprung_moment = spring_term + gravity_term;
    }

    return (sprung_moment + geometric + unsprung) / t;
}

// (9.1)-(9.7). Steps 1-6 of §9.3.
[[nodiscard]] inline core::Result<WheelLoads> wheel_loads(const core::Vehicle& v,
                                                          const LoadTransferInputs& in,
                                                          LegacyLoadTransfer legacy = {}) {
    if (!v.has_springing()) {
        return core::err<WheelLoads>(
            "E-VEH-010",
            "vehicle '" + v.name + "': the load-transfer model requires chassis-to-ground "
            "roll stiffnesses for both axles");
    }

    WheelLoads w;
    // Step 1 — static loads (9.1). Note the cross-over: front is set by b.
    w.static_front_N = v.m * v.g * v.b / v.l;
    w.static_rear_N  = v.m * v.g * v.a / v.l;

    // Step 2 — aerodynamic vertical load (9.2), per axle, positive for downforce.
    w.aero_front_N = in.aero_downforce_front_N;
    w.aero_rear_N  = in.aero_downforce_rear_N;

    // Step 3 — longitudinal transfer (9.3): off the front, onto the rear, under
    // acceleration. Aerodynamic drag acts at the CG height in Rev 1.0.
    w.longitudinal_transfer_N = v.m * in.A_X_mps2 * v.h / v.l;

    // Steps 4 — roll angle (9.5) and the axle transfers (9.6a)/(9.6b).
    w.phi_tilde_rad  = roll_angle_tilde(v, in.A_Y_mps2);
    w.roll_angle_rad = -w.phi_tilde_rad;  // SAE §3.3

    w.front.transfer_N = axle_transfer_N(v, in.A_Y_mps2, w.phi_tilde_rad, true, legacy);
    w.rear.transfer_N  = axle_transfer_N(v, in.A_Y_mps2, w.phi_tilde_rad, false, legacy);

    // Step 5 — assembly (9.7). Signs shown for a right-hand turn (A_Y > 0), in
    // which the LEFT wheels are outside.
    const double front_axle = w.static_front_N + w.aero_front_N - w.longitudinal_transfer_N;
    const double rear_axle  = w.static_rear_N + w.aero_rear_N + w.longitudinal_transfer_N;

    w.LF = 0.5 * front_axle + w.front.transfer_N;
    w.RF = 0.5 * front_axle - w.front.transfer_N;
    w.LR = 0.5 * rear_axle  + w.rear.transfer_N;
    w.RR = 0.5 * rear_axle  - w.rear.transfer_N;

    // (9.9). FLT = 1 is exactly the wheel-lift condition. Computed BEFORE the
    // clamp, so it can legitimately exceed 1 and say by how much.
    w.front.flt = (front_axle != 0.0) ? 2.0 * w.front.transfer_N / front_axle : 0.0;
    w.rear.flt  = (rear_axle  != 0.0) ? 2.0 * w.rear.transfer_N  / rear_axle  : 0.0;

    // Step 6 — wheel lift. Rule D1: no silent clamping; every clamp raises an
    // event the first time it fires, with the wheel identity and the time.
    const auto clamp_wheel = [&](double& load, core::Wheel which) {
        if (load < 0.0) {
            w.events.push_back(core::make_diagnostic(
                "EV-LIFT",
                "wheel " + std::string(core::to_string(which)) + " load reached zero (" +
                    std::to_string(load) + " N before clamping); the quasi-static roll model's "
                    "premises no longer hold and Rev 1.0 does not redistribute the lost moment",
                in.time_s, which));
            load = 0.0;
        }
    };
    clamp_wheel(w.LF, core::Wheel::LF);
    clamp_wheel(w.RF, core::Wheel::RF);
    clamp_wheel(w.LR, core::Wheel::LR);
    clamp_wheel(w.RR, core::Wheel::RR);

    return w;
}

// ---------------------------------------------------------------- invariants
//
// §9.5 requires all three. Each is blind to defects the others catch, and the
// spec's own defect-injection table records exactly which.

// I-3 (9.10): whole-vehicle roll-moment balance. Catches a wrong phi_tilde
// denominator, a flipped gravity sign, an inverted z_R_eff. BLIND to a K_phiF /
// K_phiR swap and to a consistent unit error in the roll stiffnesses.
[[nodiscard]] inline double invariant_I3_residual(const core::Vehicle& v, double A_Y,
                                                  const WheelLoads& w) {
    const double lhs = w.front.transfer_N * v.t_F + w.rear.transfer_N * v.t_R;
    // The gravity term carries NO A_Y/g factor: phi_tilde is already first-order
    // in A_Y through (9.5).
    const double rhs = v.m_s * A_Y * v.h_s
                     + v.m_s * v.g * v.h2() * w.phi_tilde_rad
                     + v.m_uF * A_Y * v.z_WF
                     + v.m_uR * A_Y * v.z_WR;
    const double scale = std::max({1.0, std::abs(lhs), std::abs(rhs)});
    return (lhs - rhs) / scale;
}

// I-3b (9.10b): per-axle allocation. Against (9.6a) this is an algebraic
// tautology and carries no independent physics; its value is as a STRUCTURAL
// check, pinning the implementation to the specified split. It catches the
// K_phiF/K_phiR swap that I-3 is blind to.
[[nodiscard]] inline double invariant_I3b_residual(const core::Vehicle& v, double A_Y,
                                                   const WheelLoads& w, bool front) {
    const double t   = front ? v.t_F : v.t_R;
    const double K   = front ? *v.K_phi_F : *v.K_phi_R;
    const double z_R = front ? v.z_RF : v.z_RR;
    const double m_u = front ? v.m_uF : v.m_uR;
    const double z_W = front ? v.z_WF : v.z_WR;
    const double frac = front ? (1.0 - v.a_s / v.l) : (v.a_s / v.l);
    const double transfer = front ? w.front.transfer_N : w.rear.transfer_N;

    const double residual = transfer * t - K * w.phi_tilde_rad - frac * v.m_s * z_R * A_Y
                          - m_u * z_W * A_Y;
    const double scale = std::max(1.0, std::abs(transfer * t));
    return residual / scale;
}

// I-3c (9.10c): an ABSOLUTE scale check with an independent dimensional
// reference. The one the other two cannot do: neither I-3 nor I-3b detects a
// CONSISTENT unit error in the roll stiffnesses, because (9.10) is an identity in
// SumK and (9.10b) reads K_phi from the same resolved document the model does.
struct RollGradientCheck {
    double deg_per_g{};
    bool   in_band{};        // [0.1, 12.0]
    double equality_residual{};  // against (12.10), computed independently
};

[[nodiscard]] inline RollGradientCheck invariant_I3c(const core::Vehicle& v, double A_Y,
                                                     const WheelLoads& w) {
    RollGradientCheck c;
    c.deg_per_g = v.roll_gradient_deg_per_g();
    c.in_band = (c.deg_per_g >= 0.1 && c.deg_per_g <= 12.0);
    // (9.5) and (12.10) must agree exactly — they are the same expression.
    if (A_Y != 0.0) {
        const double from_model = w.phi_tilde_rad / A_Y;
        const double from_12_10 = v.roll_gradient_rad_per_mps2();
        c.equality_residual =
            (from_model - from_12_10) / std::max(1e-12, std::abs(from_12_10));
    }
    return c;
}

// §9.4: for a RIGID vehicle with equal tracks. F-03 checks this against the
// per-axle model's premises rather than through (9.6).
[[nodiscard]] inline double rigid_lateral_transfer_N(const core::Vehicle& v, double A_Y,
                                                     double track_m) {
    return v.m * A_Y * v.h / track_m;
}

// (9.3) as a standalone, for F-02.
[[nodiscard]] inline double longitudinal_transfer_N(const core::Vehicle& v, double A_X) {
    return v.m * A_X * v.h / v.l;
}

}  // namespace rcvd::models

#endif  // RCVD_MODELS_LOAD_TRANSFER_HPP
