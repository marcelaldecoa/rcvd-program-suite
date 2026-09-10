// rcvd/tire/relaxation.hpp — §6.9 relaxation-length dynamics.
//
// The tire does not generate lateral force instantaneously. The build-up is a
// first-order lag in DISTANCE TRAVELLED, not time:
//
//   sigma * dFy/ds + Fy = Fy_ss(alpha, Fz)                                (6.13)
//
// With ds = u dt that becomes a time-domain lag with a SPEED-DEPENDENT time
// constant tau = sigma/u (6.14). At 30 m/s with sigma = 0.5 m, tau = 17 ms
// against a vehicle yaw time constant of order 100 ms — negligible. At 5 m/s,
// tau = 100 ms — dominant. Which is why it is a switch, not a default.
#ifndef RCVD_TIRE_RELAXATION_HPP
#define RCVD_TIRE_RELAXATION_HPP

#include <cmath>

namespace rcvd::tire {

struct RelaxationConfig {
    double sigma_m = 0.0;        // relaxation length
    double speed_floor_mps = 0.5;  // §5.5 speed_floor
    bool enabled = false;        // §6.9: default disabled for every program
};

// (6.14) tau = sigma / u.
[[nodiscard]] inline double relaxation_tau_s(double sigma_m, double u_mps) {
    return sigma_m / u_mps;
}

// The lagged state for one wheel.
class RelaxationFilter {
public:
    explicit RelaxationFilter(RelaxationConfig cfg = {}) : cfg_(cfg) {}

    // Advance one step of dt and return the lagged force.
    //
    // SPEED GUARD (normative). tau = sigma/u is singular at u = 0. Below the speed
    // floor the filter is BYPASSED and the steady-state force used, with the state
    // RE-INITIALISED to Fy_ss so that no transient is injected when the vehicle
    // re-accelerates — a filter left holding a stale value would produce one.
    double step(double Fy_ss_N, double u_mps, double dt_s) {
        if (!cfg_.enabled || cfg_.sigma_m <= 0.0 || u_mps < cfg_.speed_floor_mps) {
            Fy_ = Fy_ss_N;
            return Fy_;
        }
        const double tau = relaxation_tau_s(cfg_.sigma_m, u_mps);
        // Exact discrete solution of (6.15) over dt for a held Fy_ss, which is
        // unconditionally stable and matches the step response below exactly.
        const double a = std::exp(-dt_s / tau);
        Fy_ = a * Fy_ + (1.0 - a) * Fy_ss_N;
        return Fy_;
    }

    [[nodiscard]] double value() const { return Fy_; }
    void reset(double Fy_N = 0.0) { Fy_ = Fy_N; }
    [[nodiscard]] bool bypassed(double u_mps) const {
        return !cfg_.enabled || cfg_.sigma_m <= 0.0 || u_mps < cfg_.speed_floor_mps;
    }

private:
    RelaxationConfig cfg_;
    double Fy_ = 0.0;
};

// §6.9 closed forms, retained because they are what the acceptance vectors check.

// Step response: Fy(t) = Fy_ss (1 - exp(-t/tau)), 63.2 % at tau, 98.2 % at 4 tau.
[[nodiscard]] inline double step_response(double Fy_ss_N, double t_s, double tau_s) {
    return Fy_ss_N * (1.0 - std::exp(-t_s / tau_s));
}

// (6.16) ramp response at slope A, from Laplace inversion of (tau s + 1) Fy = C A/s^2:
//   Fy(t) = A tau C_alpha (exp(-t/tau) + t/tau - 1)
// which lags the quasi-static line C_alpha A t by exactly A tau C_alpha at large
// t — i.e. a pure time delay of tau.
[[nodiscard]] inline double ramp_response(double A_rad_per_s, double C_alpha_N_per_rad,
                                          double t_s, double tau_s) {
    return A_rad_per_s * tau_s * C_alpha_N_per_rad *
           (std::exp(-t_s / tau_s) + t_s / tau_s - 1.0);
}

// §6.9 / §13.4: the ratio the run manifest reports, so the user can see when the
// choice matters. An explicit fixed-step integrator needs dt <~ tau/5.
[[nodiscard]] inline double tau_ratio(double sigma_m, double u_mps, double tau_vehicle_s) {
    return tau_vehicle_s > 0.0 ? relaxation_tau_s(sigma_m, u_mps) / tau_vehicle_s : 0.0;
}

}  // namespace rcvd::tire

#endif  // RCVD_TIRE_RELAXATION_HPP
