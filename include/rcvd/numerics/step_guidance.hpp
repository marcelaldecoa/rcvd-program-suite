// rcvd/numerics/step_guidance.hpp — §13.3 step-size guidance.
//
// "The step must resolve the fastest mode present, not merely the slowest."
//
//   Vehicle yaw/sideslip   period 2*pi/omega_n     dt <= 2*pi/(10 omega_n)
//   Steering input at f    period 1/f              dt <= 1/(10 f)
//   Tire relaxation        tau = sigma/u           dt <= tau/5
//
// The binding constraint is normally the relaxation lag when it is enabled, and
// the steering input frequency otherwise.
//
// These are RESOLUTION limits, not accuracy limits: a 4th-order method at a given
// tolerance may demand a smaller step still, which is what the adaptive controller
// of (13.1) is for. A step that passes this check can still be too large for the
// tolerance; a step that fails it is producing a spurious answer regardless.
#ifndef RCVD_NUMERICS_STEP_GUIDANCE_HPP
#define RCVD_NUMERICS_STEP_GUIDANCE_HPP

#include <algorithm>
#include <limits>
#include <optional>
#include <string>

#include "rcvd/core/diagnostic.hpp"
#include "rcvd/units/constants.hpp"

namespace rcvd::numerics {

struct StepInputs {
    double dt_s = 0.0;
    double omega_n_rad_per_s = 0.0;   // from the linearised system at the initial condition
    double input_frequency_Hz = 0.0;  // 0 when the steer input has no frequency
    double tau_tire_s = 0.0;          // 0 when relaxation is disabled
    bool fixed_step = false;          // §13.3's W-NUM-210 condition
    bool tire_is_C0 = false;
};

struct StepCheck {
    bool adequate = true;
    std::string limiting_mode;    // which term bound the step
    double suggested_dt_s = std::numeric_limits<double>::infinity();
    std::optional<core::Diagnostic> warning;        // W-NUM-200
    std::optional<core::Diagnostic> order_warning;  // W-NUM-210
};

[[nodiscard]] inline StepCheck check_step(const StepInputs& in) {
    StepCheck out;

    struct Limit { const char* name; double dt; };
    Limit limits[3] = {{"", std::numeric_limits<double>::infinity()},
                       {"", std::numeric_limits<double>::infinity()},
                       {"", std::numeric_limits<double>::infinity()}};

    if (in.omega_n_rad_per_s > 0.0) {
        limits[0] = {"vehicle yaw/sideslip mode",
                     2.0 * units::PI / (10.0 * in.omega_n_rad_per_s)};
    }
    if (in.input_frequency_Hz > 0.0) {
        limits[1] = {"steering input frequency", 1.0 / (10.0 * in.input_frequency_Hz)};
    }
    if (in.tau_tire_s > 0.0) {
        limits[2] = {"tire relaxation lag", in.tau_tire_s / 5.0};
    }

    for (const Limit& l : limits) {
        if (l.dt < out.suggested_dt_s) {
            out.suggested_dt_s = l.dt;
            out.limiting_mode = l.name;
        }
    }

    out.adequate = in.dt_s <= out.suggested_dt_s;
    if (!out.adequate) {
        out.warning = core::make_diagnostic(
            "W-NUM-200",
            "integration step " + std::to_string(in.dt_s) + " s is too large for the " +
                out.limiting_mode + ", which needs " + std::to_string(out.suggested_dt_s) +
                " s or less (at least ten points per cycle). Beating and aliasing are the "
                "same defect in other clothing: a 5 Hz input sampled at 0.1 s lands on zero "
                "every time and produces a completely spurious result.");
    }

    // §13.3: for C0 tire models the classical order is lost whenever a step
    // brackets a slope break, so dopri54 is recommended in that configuration.
    if (in.fixed_step && in.tire_is_C0) {
        out.order_warning = core::make_diagnostic(
            "W-NUM-210",
            "fixed-step integrator with a C0 tire model: the classical order is lost "
            "whenever a step brackets a declared slope break. Prefer dopri54, which can "
            "restart at the kink.");
    }
    return out;
}

// §13.3's canonical failure, as a predicate the tests can name directly.
//
// A sinusoid of period T sampled at dt lands on a ZERO CROSSING every step when
// 2*dt/T is an integer — NOT when dt/T is. The spec's own example is the case
// 2*dt/T = 1: a 5 Hz input (T = 0.2 s) at dt = 0.1 s is sampled only where it is
// zero, "both landing where the input is zero", and the run is completely
// spurious.
[[nodiscard]] inline bool samples_land_on_zeros(double period_s, double dt_s,
                                                double tol = 1e-9) {
    if (period_s <= 0.0 || dt_s <= 0.0) return false;
    const double half_periods = 2.0 * dt_s / period_s;
    return std::abs(half_periods - std::round(half_periods)) < tol;
}

// The apparent (aliased) frequency a sampler at dt reports for a true frequency f.
// Above the Nyquist limit fs/2 the spectrum folds, so a fast input reappears as a
// slow one — §13.3's 0.051 s case, where the apparent period is far LONGER than
// the input.
[[nodiscard]] inline double aliased_frequency_Hz(double true_f_Hz, double dt_s) {
    const double fs = 1.0 / dt_s;
    const double folded = std::fmod(true_f_Hz, fs);
    return folded > fs / 2.0 ? fs - folded : folded;
}

// The beat a sampler at dt produces for a frequency just BELOW Nyquist — §13.3's
// 0.205 s case. Writing f = fs/2 - e, the sampled sequence is
// (-1)^k * sin(2 pi e k dt), i.e. an alternating carrier under an envelope at e.
// So the beat is |fs/2 - f|, and it is the SAME defect as aliasing: both are the
// spectrum folding about fs/2, seen from opposite sides.
[[nodiscard]] inline double beat_frequency_Hz(double true_f_Hz, double dt_s) {
    return std::abs(1.0 / (2.0 * dt_s) - true_f_Hz);
}

}  // namespace rcvd::numerics

#endif  // RCVD_NUMERICS_STEP_GUIDANCE_HPP
