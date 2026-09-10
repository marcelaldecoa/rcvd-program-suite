// rcvd/models/bicycle.hpp — Program P1, spec §8.
//
// Linear 2-DOF (yaw + sideslip). Constant forward speed, small angles, linear
// tires, no load transfer, no roll DOF, no aerodynamics. The CG height is
// irrelevant to this model and is ignored (§8.1).
#ifndef RCVD_MODELS_BICYCLE_HPP
#define RCVD_MODELS_BICYCLE_HPP

#include <array>
#include <cmath>
#include <optional>
#include <string>

#include "rcvd/core/result.hpp"
#include "rcvd/core/vehicle.hpp"
#include "rcvd/numerics/linalg.hpp"
#include "rcvd/units/quantity.hpp"

namespace rcvd::models {

// Axle cornering stiffnesses, both negative (§3.3). These are AXLE values: for a
// four-wheel vehicle each is the sum over that axle's two tires.
struct AxleStiffness {
    double C_F_N_per_rad{};
    double C_R_N_per_rad{};
};

// §8.3. Y_r and N_r are speed-dependent BY CONSTRUCTION, which is why the API
// requires V and why derivatives are never cached on the Vehicle.
struct StabilityDerivatives {
    double V_mps{};
    double Y_beta{};   // N/rad
    double Y_r{};      // N*s/rad
    double Y_delta{};  // N/rad
    double N_beta{};   // N*m/rad
    double N_r{};      // N*m*s/rad
    double N_delta{};  // N*m/rad

    // §8.3: N_beta is the weathervane term, and its sign decides steady-state
    // under/oversteer. N_beta > 0 understeer, = 0 neutral, < 0 oversteer.
    [[nodiscard]] bool understeering() const { return N_beta > 0.0; }

    // Rule U6 typed views. The six derivatives have FOUR distinct dimensions, and
    // mixing them is the error this exists to make impossible:
    //   Y_beta, Y_delta   N/rad        CorneringStiffness
    //   Y_r               N*s/rad      SideForceDamping
    //   N_beta, N_delta   N*m/rad      AngularStiffness
    //   N_r               N*m*s/rad    YawDamping
    // Displaying Y_r with the lbf*ft/deg factor — which is what I did when first
    // writing the B-04 vector — is now a compile error rather than a wrong number.
    [[nodiscard]] units::CorneringStiffness Y_beta_q() const {
        return units::CorneringStiffness{Y_beta};
    }
    [[nodiscard]] units::SideForceDamping Y_r_q() const {
        return units::SideForceDamping{Y_r};
    }
    [[nodiscard]] units::CorneringStiffness Y_delta_q() const {
        return units::CorneringStiffness{Y_delta};
    }
    [[nodiscard]] units::AngularStiffness N_beta_q() const {
        return units::AngularStiffness{N_beta};
    }
    [[nodiscard]] units::YawDamping N_r_q() const { return units::YawDamping{N_r}; }
    [[nodiscard]] units::AngularStiffness N_delta_q() const {
        return units::AngularStiffness{N_delta};
    }
};

[[nodiscard]] inline StabilityDerivatives stability_derivatives(const core::Vehicle& v,
                                                                const AxleStiffness& C,
                                                                double V_mps) {
    const double CF = C.C_F_N_per_rad, CR = C.C_R_N_per_rad;
    StabilityDerivatives d;
    d.V_mps   = V_mps;
    d.Y_beta  = CF + CR;                                        // (8.6)
    d.Y_r     = (v.a * CF - v.b * CR) / V_mps;                  // (8.7)
    d.Y_delta = -CF;                                            // (8.8)
    d.N_beta  = v.a * CF - v.b * CR;                            // (8.9)
    d.N_r     = (v.a * v.a * CF + v.b * v.b * CR) / V_mps;      // (8.10)
    d.N_delta = -v.a * CF;                                      // (8.11)
    return d;
}

// §8.4 (8.12). State x = [v, r], input delta.
struct StateSpace {
    std::array<std::array<double, 2>, 2> A{};
    std::array<double, 2> B{};
};

[[nodiscard]] inline StateSpace state_space(const core::Vehicle& veh,
                                            const StabilityDerivatives& d) {
    const double u = d.V_mps;
    StateSpace ss;
    // SECOND-COLUMN TRAP (§8.4): Y_r and N_r already carry the 1/u of (8.7) and
    // (8.10). The r-column entries are Y_r/m - u and N_r/I_z, NOT Y_r/(m u) and
    // N_r/(I_z u). Dividing by u a second time leaves the eigenvalues plausible
    // while making the yaw-rate gain wrong by a factor of order u.
    ss.A[0][0] = d.Y_beta / (veh.m * u);
    ss.A[0][1] = d.Y_r / veh.m - u;
    ss.A[1][0] = d.N_beta / (veh.I_z * u);
    ss.A[1][1] = d.N_r / veh.I_z;
    // SIGN NOTE (§8.4): Y_delta = -C_F and C_F < 0, so these are POSITIVE. Writing
    // B with a leading minus double-negates the input and yields a steady-state
    // yaw rate of the wrong sign and magnitude. Identity I-2 catches it.
    ss.B[0] = d.Y_delta / veh.m;
    ss.B[1] = d.N_delta / veh.I_z;
    return ss;
}

// §8.5.
struct SteadyStateResult {
    double v_mps{};        // lateral velocity
    double r_rad_per_s{};  // yaw rate
    double beta_rad{};
    double A_Y_mps2{};
    double rcond{};        // §13.8: reported with every solve
};

[[nodiscard]] inline core::Result<SteadyStateResult> steady_state(
    const core::Vehicle& veh, const AxleStiffness& C, double V_mps, double delta_rad) {
    const auto d = stability_derivatives(veh, C, V_mps);
    const auto ss = state_space(veh, d);

    // (8.13) x_ss = -A^-1 B delta, solved by LU with partial pivoting — never by
    // an explicit inverse (§8.5).
    const std::array<double, 2> rhs{-ss.B[0] * delta_rad, -ss.B[1] * delta_rad};
    const auto sol = numerics::lu_solve<2>(ss.A, rhs);

    if (sol.singular || sol.rcond < 1e-12) {
        return core::err<SteadyStateResult>(
            "E-SS-030",
            "steady-state system is singular at V = " + std::to_string(V_mps) +
                " m/s (rcond = " + std::to_string(sol.rcond) +
                "); the vehicle is at or above its critical speed");
    }

    SteadyStateResult out;
    out.v_mps       = sol.x[0];
    out.r_rad_per_s = sol.x[1];
    out.beta_rad    = std::atan2(out.v_mps, V_mps);
    out.A_Y_mps2    = V_mps * out.r_rad_per_s;  // steady state: vdot = 0, so A_Y = u*r
    out.rcond       = sol.rcond;
    return out;
}

// §8.5 closed forms, "retained because they are the cheapest cross-check
// available".
struct HandlingMetrics {
    double K{};             // (8.14) stability factor, s^2/m^2
    double K_us{};          // (8.18) understeer gradient, rad per (m/s^2)
    double NSP{};           // (8.22) fraction of l aft of the front axle
    double SM{};            // (8.23) static margin
    double V_tan{};         // (8.21) tangent speed
    std::optional<double> V_ch;    // (8.19), understeer only
    std::optional<double> V_crit;  // (8.20), oversteer only
};

[[nodiscard]] inline HandlingMetrics handling_metrics(const core::Vehicle& veh,
                                                      const AxleStiffness& C) {
    const double CF = C.C_F_N_per_rad, CR = C.C_R_N_per_rad;
    // Formed at unit speed: K, NSP, SM and V_tan are all speed-independent, since
    // the 1/u in Y_r and N_r cancels out of (8.14).
    const auto d = stability_derivatives(veh, C, 1.0);

    HandlingMetrics m;
    // (8.14) K = m*N_beta / (l*(N_beta*Y_delta - Y_beta*N_delta)). The denominator
    // reduces to l*C_F*C_R*l, but it is written in derivative form here because
    // that is the form identity I-1 cross-checks.
    m.K = veh.m * d.N_beta / (veh.l * (d.N_beta * d.Y_delta - d.Y_beta * d.N_delta));
    m.K_us  = m.K * veh.l;                                       // (8.18)
    m.NSP   = CR / (CF + CR);                                    // (8.22)
    m.SM    = m.NSP - veh.a / veh.l;                             // (8.23)
    // (8.21) V_tan = sqrt(-b*l*C_R/(m*a)); C_R < 0, so the radicand is positive.
    m.V_tan = std::sqrt(-veh.b * veh.l * CR / (veh.m * veh.a));

    if (m.K > 0.0) m.V_ch = std::sqrt(1.0 / m.K);                // (8.19)
    if (m.K < 0.0) m.V_crit = std::sqrt(-1.0 / m.K);             // (8.20)
    return m;
}

// (8.15) yaw-rate gain via the stability factor.
[[nodiscard]] inline double yaw_rate_gain(const core::Vehicle& veh, double K, double V_mps) {
    return (V_mps / veh.l) / (1.0 + K * V_mps * V_mps);
}

// (8.25) the direct derivative expression. Identity I-2 requires this to equal
// (8.15) to machine precision — a strong check on the §8.3 sign conventions.
[[nodiscard]] inline double yaw_rate_gain_derivative_form(const core::Vehicle& veh,
                                                          const AxleStiffness& C, double V_mps) {
    const auto d = stability_derivatives(veh, C, V_mps);
    const double num = d.Y_beta * d.N_delta - d.N_beta * d.Y_delta;
    const double den = d.N_beta * d.Y_r - d.N_beta * veh.m * V_mps - d.Y_beta * d.N_r;
    return num / den;
}

// I-1 (8.24): the classical axle form of the understeer gradient, rad per g.
// Cross-checks the derivative form (8.14) and the §8.3 sign conventions.
[[nodiscard]] inline double understeer_gradient_axle_form(const core::Vehicle& veh,
                                                          const AxleStiffness& C) {
    return veh.W_F() / std::abs(C.C_F_N_per_rad) - veh.W_R() / std::abs(C.C_R_N_per_rad);
}

// §8.6.
struct Eigenstructure {
    double omega_n{};
    double zeta{};
    double re1{}, im1{}, re2{}, im2{};
    double tau_dominant{};  // 1/|Re(s_dominant)|
    double trace{}, det{};
    [[nodiscard]] bool stable() const { return trace < 0.0 && det > 0.0; }
    [[nodiscard]] bool oscillatory() const { return im1 != 0.0; }
};

[[nodiscard]] inline Eigenstructure eigenstructure(const core::Vehicle& veh,
                                                   const AxleStiffness& C, double V_mps) {
    const auto d = stability_derivatives(veh, C, V_mps);
    const auto ss = state_space(veh, d);
    const auto e = numerics::eig2x2(ss.A);

    Eigenstructure out;
    out.trace = ss.A[0][0] + ss.A[1][1];
    out.det   = ss.A[0][0] * ss.A[1][1] - ss.A[0][1] * ss.A[1][0];
    // (8.26)-(8.27): 2*zeta*omega_n = -tr A, omega_n^2 = det A.
    out.omega_n = std::sqrt(std::abs(out.det));
    out.zeta    = (out.omega_n > 0.0) ? -out.trace / (2.0 * out.omega_n) : 0.0;
    out.re1 = e.re1; out.im1 = e.im1;
    out.re2 = e.re2; out.im2 = e.im2;

    const double dom = std::max(e.re1, e.re2);  // least negative == slowest
    out.tau_dominant = (dom != 0.0) ? 1.0 / std::abs(dom) : INFINITY;
    return out;
}

// (8.28) r(s)/delta(s) = (n1 s + n0) / (s^2 + 2 zeta omega_n s + omega_n^2).
struct YawRateTransferFunction {
    double n1{}, n0{};
    double two_zeta_omega_n{}, omega_n_sq{};
    [[nodiscard]] double dc_gain() const { return n0 / omega_n_sq; }
};

[[nodiscard]] inline YawRateTransferFunction yaw_rate_transfer_function(
    const core::Vehicle& veh, const AxleStiffness& C, double V_mps) {
    const auto d = stability_derivatives(veh, C, V_mps);
    const auto ss = state_space(veh, d);
    YawRateTransferFunction tf;
    tf.n1 = ss.B[1];                                   // C = [0, 1]
    tf.n0 = -ss.B[1] * ss.A[0][0] + ss.B[0] * ss.A[1][0];
    tf.two_zeta_omega_n = -(ss.A[0][0] + ss.A[1][1]);
    tf.omega_n_sq = ss.A[0][0] * ss.A[1][1] - ss.A[0][1] * ss.A[1][0];
    return tf;
}

// §8.8 (8.30). A constant road tilt introduces a gravity component in the lateral
// equation. The straight-run answer is SPEED-INDEPENDENT, which is a useful
// invariant test (B-29).
struct TiltedRoadTrim {
    double Fy_F_N{}, Fy_R_N{};
    double alpha_F_rad{}, alpha_R_rad{};
    double beta_rad{}, delta_rad{};
};

[[nodiscard]] inline TiltedRoadTrim tilted_road_trim(const core::Vehicle& veh,
                                                     const AxleStiffness& C, double tilt_rad) {
    const double lateral = veh.m * veh.g * std::sin(tilt_rad);
    TiltedRoadTrim t;
    // Setting r = 0 in (8.30): the axle forces share -m g sin(theta) by the
    // opposite lever arm, so the yaw moment a*Fy_F - b*Fy_R vanishes identically.
    t.Fy_F_N = -lateral * (veh.b / veh.l);
    t.Fy_R_N = -lateral * (veh.a / veh.l);
    t.alpha_F_rad = t.Fy_F_N / C.C_F_N_per_rad;
    t.alpha_R_rad = t.Fy_R_N / C.C_R_N_per_rad;
    t.beta_rad  = t.alpha_R_rad;
    t.delta_rad = t.beta_rad - t.alpha_F_rad;
    return t;
}

// B-18: a steady side force F applied `d` ahead of the CG. Pure statics — no g,
// no tire model, no speed. The pair must sum to -F and satisfy
// a*Fy_F - b*Fy_R = -F*d to machine precision.
struct CrosswindTrim {
    double Fy_F_N{}, Fy_R_N{};
};

[[nodiscard]] inline CrosswindTrim crosswind_trim(const core::Vehicle& veh, double force_N,
                                                  double ahead_of_cg_m) {
    CrosswindTrim c;
    c.Fy_F_N = -force_N * (veh.b + ahead_of_cg_m) / veh.l;
    c.Fy_R_N = -force_N * (veh.a - ahead_of_cg_m) / veh.l;
    return c;
}

}  // namespace rcvd::models

#endif  // RCVD_MODELS_BICYCLE_HPP
