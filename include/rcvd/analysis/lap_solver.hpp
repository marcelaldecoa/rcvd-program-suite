// rcvd/analysis/lap_solver.hpp — Programs P4 and P5, spec §11.
//
// A minimal closed-circuit exercise: circumnavigate a circular obstacle of known
// radius in minimum time. P4 uses the P1 plant, P5 uses the P2 plant. The value is
// pedagogical, so responsiveness and clarity of failure modes matter more than
// fidelity — but the EVENT LOCATION is not negotiable: §11.4 makes a lap time
// quantised to the integration step a defect, not a tolerance issue.
#ifndef RCVD_ANALYSIS_LAP_SOLVER_HPP
#define RCVD_ANALYSIS_LAP_SOLVER_HPP

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "rcvd/core/result.hpp"
#include "rcvd/models/plant.hpp"
#include "rcvd/numerics/integrate.hpp"
#include "rcvd/numerics/roots.hpp"

namespace rcvd::analysis {

// §11.2. Rev 1.0 implements constant_radius only; a compound circuit is an
// extension seam.
struct Circuit {
    double centre_x = 0.0, centre_y = 0.0;
    double inner_radius = 0.0;
    double outer_radius = std::numeric_limits<double>::infinity();  // optional wall
};

// §11.3. Open loop: there is NO closed-loop driver in Rev 1.0. The absence of
// feedback is what makes the exercise instructive, and what makes the initial
// transient decisive (§11.6).
struct DriverPlan {
    double initial_speed_mps = 0.0;
    models::Maneuver steer = models::Maneuver::make_fixed(0.0);
    double start_x = 0.0, start_y = 0.0, start_heading_rad = 0.0;
};

// §11.5.
enum class LapEvent { none, lap, wall_in, wall_out, spin, timeout };

[[nodiscard]] inline std::string to_string(LapEvent e) {
    switch (e) {
        case LapEvent::none:     return "none";
        case LapEvent::lap:      return "EV-LAP";
        case LapEvent::wall_in:  return "EV-WALL-IN";
        case LapEvent::wall_out: return "EV-WALL-OUT";
        case LapEvent::spin:     return "EV-SPIN";
        case LapEvent::timeout:  return "EV-TIMEOUT";
    }
    return "?";
}

struct TrajectorySample {
    double t = 0.0;
    double X_E = 0.0, Y_E = 0.0, psi = 0.0, v = 0.0, r = 0.0;
};

struct LapConfig {
    double calculation_length_s = 30.0;
    double initial_step_s = 1e-3;
    double output_record_increment_s = 0.01;
    numerics::AdaptiveConfig integrator{};
    double event_tolerance_s = 1e-9;   // §11.4
    bool record_trajectory = true;
};

struct LapResult {
    LapEvent event = LapEvent::none;
    double time_s = 0.0;               // lap time, or time to the terminating event
    std::vector<double> final_state;
    std::vector<TrajectorySample> path;
    std::vector<core::Diagnostic> diagnostics;
    numerics::StepStats stats{};
    double tau_dominant_s = 0.0;       // for the §11.6 hint
};

namespace detail {

// Signed angle swept from p0 to p1 about the centre, in (-pi, pi]. Correct as
// long as one step spans less than half a turn, which any usable step does.
inline double swept_angle(double x0, double y0, double x1, double y1, double cx, double cy) {
    const double ax = x0 - cx, ay = y0 - cy;
    const double bx = x1 - cx, by = y1 - cy;
    return std::atan2(ax * by - ay * bx, ax * bx + ay * by);
}

inline double radius_from(double x, double y, double cx, double cy) {
    return std::hypot(x - cx, y - cy);
}

}  // namespace detail

// §11.4 / §11.5. Every terminating condition is located by DENSE OUTPUT plus
// Brent refinement on the bracketing step — never by taking the first step at
// which the condition holds.
// §11.6 attribution context. Both are needed to decide whether inner-wall contact
// is the initial transient's doing or simply too much steer.
struct TransientContext {
    double tau_dominant_s = 0.0;
    // R_ss = V / (yaw-rate gain * delta), from (8.15). Infinity for zero steer.
    double steady_state_radius_m = std::numeric_limits<double>::infinity();
};

[[nodiscard]] inline core::Result<LapResult> lap_solve(const models::Plant& plant,
                                                       const Circuit& circuit,
                                                       const DriverPlan& driver,
                                                       const LapConfig& cfg,
                                                       TransientContext ctx = {}) {
    using namespace models;
    LapResult out;
    out.tau_dominant_s = ctx.tau_dominant_s;

    const int n = plant.state_dim();
    std::vector<double> x(static_cast<std::size_t>(n), 0.0);
    x[state::kPsi] = driver.start_heading_rad;
    x[state::kXe] = driver.start_x;
    x[state::kYe] = driver.start_y;

    numerics::Derivative f = [&](double t, std::span<const double> s, std::span<double> d) {
        plant.derivative(t, s, driver.steer.evaluate(t), d);
    };

    numerics::AdaptiveConfig icfg = cfg.integrator;
    if (static_cast<int>(icfg.atol.size()) < n) icfg.atol.resize(static_cast<std::size_t>(n), icfg.atol.empty() ? 1e-9 : icfg.atol.back());

    numerics::DenseSegment seg;
    double t = 0.0, h = cfg.initial_step_s;
    double theta_acc = 0.0;             // accumulated path angle about the centre
    double next_record = 0.0;
    std::vector<double> probe(static_cast<std::size_t>(n));

    // §5.6 discontinuity rule: an adaptive integrator SHALL restart at each
    // discontinuity. Integrating across one destroys the method order.
    std::vector<double> stops = driver.steer.discontinuities();
    stops.push_back(cfg.calculation_length_s);
    std::sort(stops.begin(), stops.end());
    std::size_t next_stop = 0;
    while (next_stop < stops.size() && stops[next_stop] <= 0.0) ++next_stop;

    const auto record = [&](double ts, std::span<const double> s) {
        if (!cfg.record_trajectory) return;
        out.path.push_back({ts, s[state::kXe], s[state::kYe], s[state::kPsi],
                            s[state::kV], s[state::kR]});
    };
    record(0.0, x);

    const double r0 = detail::radius_from(x[state::kXe], x[state::kYe],
                                          circuit.centre_x, circuit.centre_y);
    if (r0 < circuit.inner_radius) {
        return core::err<LapResult>("EV-WALL-IN",
                                    "the start position is already inside the inner radius");
    }

    while (t < cfg.calculation_length_s) {
        // Clip the step so it lands exactly on the next discontinuity.
        double h_max_here = cfg.calculation_length_s - t;
        if (next_stop < stops.size()) h_max_here = std::min(h_max_here, stops[next_stop] - t);
        if (h_max_here <= 0.0) { ++next_stop; continue; }
        h = std::min(h, h_max_here);

        const double h_try = h;
        if (!numerics::dopri54_step(f, t, x, h, icfg, seg, out.stats)) continue;  // rejected

        const double t0 = t, t1 = t + h_try;

        // ---- event functions on the accepted step, via dense output ---------
        // g(s) evaluated at local parameter s in [0,1].
        const auto state_at = [&](double s) -> std::span<const double> {
            seg.eval(s, probe);
            return probe;
        };
        const double dtheta_full = detail::swept_angle(seg.y0[state::kXe], seg.y0[state::kYe],
                                                       seg.y1[state::kXe], seg.y1[state::kYe],
                                                       circuit.centre_x, circuit.centre_y);

        // (11.1) lap completion: |theta(t) - theta(0)| >= 2 pi, unwrapped.
        const auto g_lap = [&](double s) {
            const auto st = state_at(s);
            const double dth = detail::swept_angle(seg.y0[state::kXe], seg.y0[state::kYe],
                                                   st[state::kXe], st[state::kYe],
                                                   circuit.centre_x, circuit.centre_y);
            return std::abs(theta_acc + dth) - 2.0 * units::PI;
        };
        const auto g_in = [&](double s) {
            const auto st = state_at(s);
            return detail::radius_from(st[state::kXe], st[state::kYe], circuit.centre_x,
                                       circuit.centre_y) - circuit.inner_radius;
        };
        const auto g_out = [&](double s) {
            const auto st = state_at(s);
            return circuit.outer_radius -
                   detail::radius_from(st[state::kXe], st[state::kYe], circuit.centre_x,
                                       circuit.centre_y);
        };
        const auto g_spin = [&](double s) {
            const auto st = state_at(s);
            // EV-SPIN is |beta| > 90 deg. With u > 0 that is |v| exceeding the
            // longitudinal speed; using the plant's own speed keeps it exact.
            return std::abs(st[state::kV]) - driver.initial_speed_mps;
        };

        struct Candidate { LapEvent ev; double s; };
        std::vector<Candidate> hits;
        const auto try_event = [&](LapEvent ev, auto&& g) {
            const double a = g(0.0), b = g(1.0);
            if (a > 0.0 && b > 0.0) return;
            if (a <= 0.0 && b <= 0.0) return;      // already inside at step start
            const auto root = numerics::brent(
                g, 0.0, 1.0,
                numerics::RootConfig{cfg.event_tolerance_s / std::max(1e-12, h_try), 1e-14, 200});
            if (root.ok()) hits.push_back({ev, root.value().root});
        };
        try_event(LapEvent::lap, g_lap);
        try_event(LapEvent::wall_in, g_in);
        if (std::isfinite(circuit.outer_radius)) try_event(LapEvent::wall_out, g_out);
        try_event(LapEvent::spin, g_spin);

        // ---- output records inside this step, by dense output ---------------
        // §15.1: records come from interpolation, NOT by forcing the step to land
        // on record times, so record frequency and accuracy stay independent.
        if (cfg.record_trajectory) {
            while (next_record + cfg.output_record_increment_s <= t1 + 1e-15) {
                next_record += cfg.output_record_increment_s;
                if (next_record < t0) continue;
                const double s = (next_record - t0) / h_try;
                seg.eval(std::clamp(s, 0.0, 1.0), probe);
                record(next_record, probe);
            }
        }

        if (!hits.empty()) {
            // The EARLIEST event terminates the run.
            const Candidate* first = &hits.front();
            for (const Candidate& c : hits) {
                if (c.s < first->s) first = &c;
            }
            seg.eval(first->s, probe);
            out.event = first->ev;
            out.time_s = t0 + first->s * h_try;
            out.final_state.assign(probe.begin(), probe.end());
            record(out.time_s, probe);

            // §11.6 attribution test: within the first revolution, AND the
            // steady-state path radius the inputs imply would have cleared the
            // wall. If the steer is simply too tight (R_ss < inner radius) the
            // contact is not the transient's doing and gets no hint.
            const double swept_at_event =
                std::abs(theta_acc + detail::swept_angle(seg.y0[state::kXe], seg.y0[state::kYe],
                                                         probe[state::kXe], probe[state::kYe],
                                                         circuit.centre_x, circuit.centre_y));
            if (out.event == LapEvent::wall_in && swept_at_event < 2.0 * units::PI &&
                ctx.steady_state_radius_m >= circuit.inner_radius) {
                out.diagnostics.push_back(core::make_diagnostic(
                    "H-SPD-001",
                    "inner-wall contact at t = " + std::to_string(out.time_s) +
                        " s, within the first revolution, with an implied steady-state path "
                        "radius of " + std::to_string(ctx.steady_state_radius_m) +
                        " m against an inner radius of " +
                        std::to_string(circuit.inner_radius) + " m (tau = " +
                        std::to_string(ctx.tau_dominant_s) +
                        " s). The settled path would have cleared the wall, so the contact is "
                        "the initial transient's doing. At t = 0 the yaw rate and lateral "
                        "velocity are zero, so the "
                        "vehicle needs a transient of a few time constants before it reaches "
                        "the steady-state circular path; during that transient the path is "
                        "not concentric with the intended circle and the circle it settles "
                        "on is offset inward. Remedies: reduce the steer angle slightly, or "
                        "start further from the wall.",
                    out.time_s));
            }
            return out;
        }

        // Commit the step.
        theta_acc += dtheta_full;
        t = t1;
        x = seg.y1;
        if (next_stop < stops.size() && t >= stops[next_stop] - 1e-15) ++next_stop;
    }

    out.event = LapEvent::timeout;
    out.time_s = cfg.calculation_length_s;
    out.final_state = x;
    out.diagnostics.push_back(core::make_diagnostic(
        "EV-TIMEOUT", "calculation length reached without lap completion", out.time_s));
    return out;
}

// §17.7 S-04: a Richardson estimate of the discretisation error in the lap time.
struct LapConvergence {
    double coarse_s = 0.0, fine_s = 0.0;
    double richardson_error_s = 0.0;
    double observed_order = 0.0;
};

[[nodiscard]] inline LapConvergence lap_convergence(double t_h, double t_h2, double t_h4,
                                                    int order = 5) {
    LapConvergence c;
    c.coarse_s = t_h;
    c.fine_s = t_h4;
    const double d1 = t_h2 - t_h, d2 = t_h4 - t_h2;
    if (std::abs(d2) > 0.0) c.observed_order = std::log2(std::abs(d1 / d2));
    // Richardson: the remaining error in the finest estimate.
    const double p = std::pow(2.0, static_cast<double>(order));
    c.richardson_error_s = std::abs(d2) / (p - 1.0);
    return c;
}

}  // namespace rcvd::analysis

#endif  // RCVD_ANALYSIS_LAP_SOLVER_HPP
