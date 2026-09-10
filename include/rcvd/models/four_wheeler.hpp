// rcvd/models/four_wheeler.hpp — Program P2, spec §9.
//
// Four individual wheels with individual vertical loads; lateral and
// longitudinal load transfer; roll-stiffness distribution; roll-centre heights;
// aerodynamic drag and downforce; nonlinear load-sensitive tires. Roll remains
// QUASI-STATIC (algebraic), not a dynamic degree of freedom — the defining
// simplification of P2 (§9.1).
#ifndef RCVD_MODELS_FOUR_WHEELER_HPP
#define RCVD_MODELS_FOUR_WHEELER_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "rcvd/aero/aero_model.hpp"
#include "rcvd/core/result.hpp"
#include "rcvd/core/vehicle.hpp"
#include "rcvd/models/bicycle.hpp"  // BicyclePlant wraps §8 for the C-02 comparison
#include "rcvd/models/load_transfer.hpp"
#include "rcvd/models/plant.hpp"
#include "rcvd/tire/tire_model.hpp"

namespace rcvd::models {

// Wheel ordering is fixed everywhere: LF, RF, LR, RR.
enum Corner { LF = 0, RF = 1, LR = 2, RR = 3 };

struct WheelGeometry {
    double x{}, y{};  // position in F2; y positive to the driver's RIGHT
};

// §9.6: (x_i, y_i) = LF(+a, -t_F/2)  RF(+a, +t_F/2)  LR(-b, -t_R/2)  RR(-b, +t_R/2)
[[nodiscard]] inline std::array<WheelGeometry, 4> wheel_geometry(const core::Vehicle& v) {
    return {{{v.a, -0.5 * v.t_F}, {v.a, +0.5 * v.t_F},
             {-v.b, -0.5 * v.t_R}, {-v.b, +0.5 * v.t_R}}};
}

// §9.14 slip angles.
//
// DERIVATION AND SIGN. The velocity of a point at body coordinates (x_i, y_i) is
// (u - r*y_i, v + r*x_i). The LEFT wheels are at y = -t/2, so their longitudinal
// velocity is u + r*t/2: in a right-hand turn (r > 0) the left wheels are on the
// outside and travel FASTER, as they must. Placing the minus sign on the left
// wheels is the natural transcription error and inverts this.
struct SlipAngles {
    std::array<double, 4> alpha{};
    std::vector<core::Diagnostic> events;  // EV-WHEEL-STALL
};

[[nodiscard]] inline SlipAngles slip_angles(const core::Vehicle& veh, double u, double v,
                                            double r, double delta,
                                            const SolverSettings& cfg, double t_s = 0.0) {
    SlipAngles out;
    const auto geo = wheel_geometry(veh);

    for (int i = 0; i < 4; ++i) {
        const double lateral = v + r * geo[i].x;
        // The +/- r*t/2 terms are second-order in most manoeuvres but become
        // significant at small radius — exactly the regime of P5's 50 ft circuit.
        const double longitudinal =
            cfg.switches.track_velocity_terms ? (u - r * geo[i].y) : u;

        double a;
        if (std::abs(longitudinal) < cfg.speed_floor_mps) {
            // §9.14 guard. Distinct from EV-SPIN, which is reserved for |beta| > 90 deg.
            a = 0.5 * units::PI * (lateral >= 0.0 ? 1.0 : -1.0);
            out.events.push_back(core::make_diagnostic(
                "EV-WHEEL-STALL",
                "wheel longitudinal velocity " + std::to_string(longitudinal) +
                    " m/s fell below the speed floor",
                t_s, static_cast<core::Wheel>(i)));
        } else if (cfg.switches.small_angle_slip) {
            a = lateral / longitudinal;
            // §3.3 as corrected: small_angle_limit is a DIAGNOSTIC threshold, never
            // a branch condition. The force path does not change here — that would
            // break Rule A2 by switching mid-manoeuvre — but the excursion is
            // reported so the user knows the approximation is being stretched.
            if (std::abs(a) > cfg.small_angle_limit_rad) {
                out.events.push_back(core::make_diagnostic(
                    "W-NUM-220",
                    "small_angle_slip is forced on and |v/u| reached " + std::to_string(std::abs(a)) +
                        " rad, beyond small_angle_limit = " +
                        std::to_string(cfg.small_angle_limit_rad) +
                        " rad; the force path is unchanged, but arctan and v/u differ by " +
                        std::to_string(std::abs(a - std::atan(a))) + " rad here",
                    t_s, static_cast<core::Wheel>(i)));
            }
        } else {
            a = std::atan2(lateral, longitudinal);
        }
        // Front wheels steer; Rev 1.0 applies the same delta to both (no Ackermann,
        // no toe), but the per-wheel form is retained so geometry can be added
        // without changing the equation structure.
        if (i == LF || i == RF) a -= delta;
        out.alpha[i] = a;
    }
    return out;
}

struct WheelForces {
    std::array<double, 4> Fy{}, Fx{}, Mz{}, Fz{};
};

struct FourWheelerObservation {
    double u{}, v{}, r{}, psi{}, X_E{}, Y_E{};
    double beta{}, A_X{}, A_Y{};
    std::array<double, 4> alpha{}, Fz{}, Fy{};
    double roll_angle{};
    double yaw_moment_residual{};   // §9.8: zero at steady state, the primary debug aid
    double path_curvature{};
    double flt_front{}, flt_rear{};
    double phi_prime{};             // §9.7 observed contraction factor
    int ay_iterations{};
    std::vector<core::Diagnostic> events;
};

class FourWheeler final : public Plant {
public:
    // Four tire models, one per corner, in LF/RF/LR/RR order. Non-owning: the
    // caller keeps them alive for the run.
    FourWheeler(const core::Vehicle& veh, std::array<const tire::TireModel*, 4> tires,
                aero::AeroConfig aero_cfg, SolverSettings cfg)
        : veh_(veh), tires_(tires), aero_(std::move(aero_cfg)), cfg_(std::move(cfg)) {}

    [[nodiscard]] int state_dim() const override { return state::kDim; }

    void derivative(double t, std::span<const double> x, const DriverInput& in,
                    std::span<double> xdot) const override {
        FourWheelerObservation obs;
        evaluate(t, x, in, obs, xdot);
    }

    // The full evaluation, exposed so the output layer gets the §9.8 channels
    // without re-integrating.
    void evaluate(double t, std::span<const double> x, const DriverInput& in,
                  FourWheelerObservation& obs, std::span<double> xdot) const {
        const double u = speed_;
        const double v = x[state::kV];
        const double r = x[state::kR];
        const double delta = in.delta_rad;

        // §9.7: solve the algebraic loop for A_Y, then take the forces it implies.
        AySolution sol = solve_A_Y(t, u, v, r, delta);

        obs.u = u; obs.v = v; obs.r = r;
        obs.psi = x[state::kPsi]; obs.X_E = x[state::kXe]; obs.Y_E = x[state::kYe];
        obs.beta = cfg_.switches.small_angle_slip ? v / u : std::atan2(v, u);
        obs.alpha = sol.alpha;
        obs.Fz = sol.forces.Fz;
        obs.Fy = sol.forces.Fy;
        obs.A_Y = sol.A_Y;
        obs.A_X = sol.A_X;
        obs.roll_angle = sol.roll_angle;
        obs.flt_front = sol.flt_front;
        obs.flt_rear = sol.flt_rear;
        obs.phi_prime = sol.phi_prime;
        obs.ay_iterations = sol.iterations;
        obs.events = std::move(sol.events);
        obs.yaw_moment_residual = sol.yaw_moment;
        obs.path_curvature = (u != 0.0) ? r / std::hypot(u, v) : 0.0;

        if (!xdot.empty()) {
            // (9.11): m(vdot + u r) = sum Fy cos d + sum Fx sin d + m g sin(tilt)
            xdot[state::kV] = sol.A_Y - u * r;
            // (9.13a)
            xdot[state::kR] = sol.yaw_moment / veh_.I_z;
            path_kinematics(u, v, r, x[state::kPsi], xdot);
        }
    }

    void set_speed(double u_mps) { speed_ = u_mps; }
    [[nodiscard]] double speed() const { return speed_; }

private:
    struct AySolution {
        double A_Y{}, A_X{}, yaw_moment{}, roll_angle{};
        double flt_front{}, flt_rear{}, phi_prime{};
        int iterations{};
        std::array<double, 4> alpha{};
        WheelForces forces{};
        std::vector<core::Diagnostic> events;
    };

    // Phi(A_Y) of (9.15): the lateral acceleration the tires produce when the
    // loads are those A_Y itself implies.
    [[nodiscard]] AySolution evaluate_at(double t, double u, double v, double r,
                                         double delta, double A_Y_assumed) const {
        AySolution s;
        const auto sa = slip_angles(veh_, u, v, r, delta, cfg_, t);
        s.alpha = sa.alpha;
        s.events = sa.events;

        // Aerodynamics (§7), evaluated on the speed MAGNITUDE.
        double drag = 0.0, df_front = 0.0, df_rear = 0.0;
        if (cfg_.switches.aero) {
            const auto af = aero::evaluate(aero_, aero::speed_magnitude(u, v));
            drag = af.drag_N;
            df_front = af.downforce_front_N;
            df_rear = af.downforce_rear_N;
        }

        // Wheel loads. With load_transfer off these reduce to the static split,
        // which is what bicycle-equivalence needs.
        if (cfg_.switches.load_transfer) {
            LoadTransferInputs lt;
            lt.A_Y_mps2 = A_Y_assumed;
            lt.A_X_mps2 = last_A_X_;
            lt.aero_downforce_front_N = df_front;
            lt.aero_downforce_rear_N = df_rear;
            lt.time_s = t;
            const auto w = wheel_loads(veh_, lt, LegacyLoadTransfer{});
            if (w.ok()) {
                s.forces.Fz = {w.value().LF, w.value().RF, w.value().LR, w.value().RR};
                s.roll_angle = w.value().roll_angle_rad;
                s.flt_front = w.value().front.flt;
                s.flt_rear = w.value().rear.flt;
                for (const auto& e : w.value().events) s.events.push_back(e);
            }
        } else {
            const double f = 0.5 * (veh_.m * veh_.g * veh_.b / veh_.l + df_front);
            const double rr = 0.5 * (veh_.m * veh_.g * veh_.a / veh_.l + df_rear);
            s.forces.Fz = {f, f, rr, rr};
        }

        // Tire forces.
        for (int i = 0; i < 4; ++i) {
            tire::TireState ts;
            ts.alpha_rad = s.alpha[i];
            ts.Fz_N = std::max(0.0, s.forces.Fz[i]);
            ts.u_mps = u;
            const auto o = tires_[i]->evaluate(ts);
            s.forces.Fy[i] = o.Fy_N;
            s.forces.Mz[i] = cfg_.switches.aligning_moment ? o.Mz_Nm : 0.0;
            s.forces.Fx[i] = 0.0;  // Rev 1.0 has no driveline; longitudinal_forces
                                   // gates only whether they enter the equations.
        }

        const double cd = cfg_.switches.steer_projection ? std::cos(delta) : 1.0;
        const double sd = cfg_.switches.steer_projection ? std::sin(delta) : 0.0;
        const bool fx = cfg_.switches.longitudinal_forces;

        // (9.11)
        double sumFy = 0.0, sumFx = 0.0;
        for (int i = 0; i < 4; ++i) {
            sumFy += s.forces.Fy[i];
            sumFx += fx ? s.forces.Fx[i] : 0.0;
        }
        const double lateral_force =
            (s.forces.Fy[LF] + s.forces.Fy[RF]) * cd + (s.forces.Fy[LR] + s.forces.Fy[RR])
            + ((s.forces.Fx[LF] + s.forces.Fx[RF]) * sd) * (fx ? 1.0 : 0.0)
            + veh_.m * veh_.g * std::sin(cfg_.road_tilt_rad);
        s.A_Y = lateral_force / veh_.m;

        // (9.12a): the SPECIFIC LONGITUDINAL FORCE, not udot - v*r. Under
        // speed_mode = constant the latter evaluates to -v*r and would discard
        // exactly the drag-induced transfer this definition exists to preserve.
        s.A_X = ((fx ? (s.forces.Fx[LF] + s.forces.Fx[RF]) * cd : 0.0)
                 - (s.forces.Fy[LF] + s.forces.Fy[RF]) * sd
                 + (fx ? (s.forces.Fx[LR] + s.forces.Fx[RR]) : 0.0)
                 - drag) / veh_.m;

        // (9.13a). SIGN TRAP: the right-hand wheels sit at y = +t/2, so their
        // longitudinal force produces a NEGATIVE yaw moment. The couple is
        // (t/2)(Fx_left - Fx_right), not the other way round.
        double N = veh_.a * ((s.forces.Fy[LF] + s.forces.Fy[RF]) * cd
                             + (fx ? (s.forces.Fx[LF] + s.forces.Fx[RF]) * sd : 0.0))
                 - veh_.b * (s.forces.Fy[LR] + s.forces.Fy[RR]);
        if (fx || cfg_.switches.steer_projection) {
            N += 0.5 * veh_.t_F * ((fx ? (s.forces.Fx[LF] - s.forces.Fx[RF]) * cd : 0.0)
                                   - (s.forces.Fy[LF] - s.forces.Fy[RF]) * sd);
            N += 0.5 * veh_.t_R * (fx ? (s.forces.Fx[LR] - s.forces.Fx[RR]) : 0.0);
        }
        if (cfg_.switches.aligning_moment) {
            for (int i = 0; i < 4; ++i) N += s.forces.Mz[i];
        }
        s.yaw_moment = N;
        (void)sumFy;
        (void)sumFx;
        return s;
    }

    // §9.7.
    [[nodiscard]] AySolution solve_A_Y(double t, double u, double v, double r,
                                       double delta) const {
        // Mode `explicit` (default, matches the source programs): use A_Y from the
        // previous accepted evaluation. Cheap, O(dt) lag in the algebraic coupling.
        if (!cfg_.implicit_load_transfer || !cfg_.switches.load_transfer) {
            AySolution s = evaluate_at(t, u, v, r, delta, last_A_Y_);
            last_A_Y_ = s.A_Y;
            last_A_X_ = s.A_X;
            s.iterations = 0;
            return s;
        }

        // Mode `implicit`: damped Newton on R(A_Y) = Phi(A_Y) - A_Y.
        double A_Y = last_A_Y_;
        AySolution s = evaluate_at(t, u, v, r, delta, A_Y);
        for (int it = 1; it <= cfg_.max_ay_iterations; ++it) {
            const double R = s.A_Y - A_Y;
            if (std::abs(R) < cfg_.atol_ay) {
                s.iterations = it;
                last_A_Y_ = s.A_Y;
                last_A_X_ = s.A_X;
                return s;
            }
            // Phi' by a finite difference on the assumed A_Y. The map is smooth
            // (dFz/dA_Y is constant), so this converges quadratically in practice.
            const double h = std::max(1e-6, 1e-6 * std::abs(A_Y));
            const AySolution sh = evaluate_at(t, u, v, r, delta, A_Y + h);
            const double phi_prime = (sh.A_Y - s.A_Y) / h;
            s.phi_prime = phi_prime;

            const double denom = phi_prime - 1.0;
            if (std::abs(denom) < 1e-12) break;  // stalled; fall through to bisection
            const double step = -R / denom;
            // Damping keeps a bad Jacobian from throwing the iterate out of range.
            A_Y += std::clamp(step, -3.0 * veh_.g, 3.0 * veh_.g);
            s = evaluate_at(t, u, v, r, delta, A_Y);
            s.phi_prime = phi_prime;
        }

        // Fall back to bisection on [-3g, +3g].
        double lo = -3.0 * veh_.g, hi = 3.0 * veh_.g;
        auto residual = [&](double a) { return evaluate_at(t, u, v, r, delta, a).A_Y - a; };
        double flo = residual(lo), fhi = residual(hi);
        if (flo * fhi <= 0.0) {
            for (int i = 0; i < 200; ++i) {
                const double mid = 0.5 * (lo + hi);
                const double fm = residual(mid);
                if (std::abs(fm) < cfg_.atol_ay) {
                    AySolution out = evaluate_at(t, u, v, r, delta, mid);
                    out.iterations = cfg_.max_ay_iterations + i;
                    last_A_Y_ = out.A_Y;
                    last_A_X_ = out.A_X;
                    return out;
                }
                if (flo * fm <= 0.0) { hi = mid; fhi = fm; } else { lo = mid; flo = fm; }
            }
        }

        // §9.7: non-convergence aborts rather than continuing with an unconverged
        // load state. The Plant interface cannot return a Result, so the condition
        // is surfaced as a fatal diagnostic on the observation and the caller
        // (the run driver) turns it into E-P2-050.
        s.events.push_back(core::make_diagnostic(
            "E-P2-050",
            "load-transfer fixed point failed to converge at t = " + std::to_string(t) +
                " s; last residual " + std::to_string(s.A_Y - A_Y) + " m/s^2, observed Phi' = " +
                std::to_string(s.phi_prime),
            t));
        return s;
    }

    core::Vehicle veh_;
    std::array<const tire::TireModel*, 4> tires_;
    aero::AeroConfig aero_;
    SolverSettings cfg_;
    double speed_ = 0.0;
    mutable double last_A_Y_ = 0.0;
    mutable double last_A_X_ = 0.0;
};

// ---------------------------------------------------------------- P1 as a Plant
//
// §8 with the (8.29) kinematics, sharing P2's state layout so C-02 can compare
// the two element by element.
class BicyclePlant final : public Plant {
public:
    BicyclePlant(const core::Vehicle& veh, AxleStiffness C, double V_mps)
        : veh_(veh), C_(C), V_(V_mps) {}

    [[nodiscard]] int state_dim() const override { return state::kDim; }

    void derivative(double, std::span<const double> x, const DriverInput& in,
                    std::span<double> xdot) const override {
        const double u = V_;
        const double v = x[state::kV];
        const double r = x[state::kR];
        const auto d = stability_derivatives(veh_, C_, u);
        const auto ss = state_space(veh_, d);

        xdot[state::kV] = ss.A[0][0] * v + ss.A[0][1] * r + ss.B[0] * in.delta_rad;
        xdot[state::kR] = ss.A[1][0] * v + ss.A[1][1] * r + ss.B[1] * in.delta_rad;
        path_kinematics(u, v, r, x[state::kPsi], xdot);
    }

private:
    core::Vehicle veh_;
    AxleStiffness C_;
    double V_;
};

}  // namespace rcvd::models

#endif  // RCVD_MODELS_FOUR_WHEELER_HPP
