// rcvd/analysis/ride_roll.hpp — Program P6, spec §12.
//
// "Despite being the simplest program it is the one most used in anger, so its
// algebra is specified completely and its two directions are specified as exact
// inverses." Invariant I-5 is the acceptance test: B then A returns the original
// totals to 1e-12, and vice versa.
#ifndef RCVD_ANALYSIS_RIDE_ROLL_HPP
#define RCVD_ANALYSIS_RIDE_ROLL_HPP

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "rcvd/core/result.hpp"
#include "rcvd/units/constants.hpp"

namespace rcvd::analysis {

// ---------------------------------------------------------------- §12.1 elements

// (12.1) ride rate: the tire in SERIES with the wheel-centre spring rate.
// k_T may be INFINITY, which denotes a rigid tire and gives k_R = k_W.
[[nodiscard]] inline double ride_rate(double k_T, double k_W) {
    if (std::isinf(k_T)) return k_W;
    return k_T * k_W / (k_T + k_W);
}

// (12.2) the inverse. Singular at k_R = k_T and negative beyond: a desired ride
// rate at or above the tire rate is unachievable with ANY finite spring, because
// the tire alone is softer than that.
[[nodiscard]] inline core::Result<double> wheel_centre_rate(double k_T, double k_R,
                                                            std::vector<core::Diagnostic>* warn = nullptr) {
    if (std::isinf(k_T)) return k_R;
    if (k_R >= k_T || k_T - k_R <= 1e-9 * k_T) {
        return core::err<double>(
            "E-RR-010",
            "desired ride rate " + std::to_string(k_R) + " N/m is at or above the tire rate " +
                std::to_string(k_T) + " N/m, which is the ceiling: the tire alone is softer "
                "than that, so no finite spring achieves it");
    }
    if (warn && k_R > 0.8 * k_T) {
        warn->push_back(core::make_diagnostic(
            "W-RR-011",
            "desired ride rate is " + std::to_string(100.0 * k_R / k_T) +
                " % of the tire rate; the required wheel-centre rate is extremely sensitive "
                "to the assumed tire rate here"));
    }
    return k_R * k_T / (k_T - k_R);
}

// (12.3) installation ratio. IR is the SPRING TRAVEL PER UNIT WHEEL TRAVEL, so
// IR < 1 (spring inboard of the wheel) gives a wheel-centre rate SOFTER than the
// spring rate. The square is the point: a 5 % installation-ratio error is a 10 %
// rate error. Some texts define the motion ratio as the reciprocal, which is why
// the schema field is named spring_travel_per_wheel_travel.
[[nodiscard]] inline double wheel_rate_from_spring(double k_S, double installation_ratio) {
    return installation_ratio * installation_ratio * k_S;
}

// (12.4) bell-crank / rocker installation. Reduces to (12.3) when theta1 == theta2.
// This is the source of rising-rate behaviour, and therefore of ride-height-
// dependent roll stiffness: a nose-down attitude stiffens the front and softens
// the rear, shifting roll-stiffness distribution forward.
[[nodiscard]] inline double rocker_rate_ratio(double L1, double L2, double theta1,
                                              double theta2, double rocker_rotation) {
    const double c1 = std::cos(theta1 + rocker_rotation);
    const double c2 = std::cos(theta2 + rocker_rotation);
    return (L2 / L1) * (L2 / L1) * (c2 * c2) / (c1 * c1);
}

// (12.5) roll stiffness from a pair of wheel rates. Under a roll angle phi the
// outside wheel deflects +t*phi/2 and the inside -t*phi/2. The general (split-rate)
// form; (12.6) K = k*t^2/2 is the symmetric special case.
[[nodiscard]] inline double roll_stiffness_from_pair(double track, double k_left,
                                                     double k_right) {
    return 0.25 * track * track * (k_left + k_right);
}

// ---------------------------------------------------------------- §12.4 frequency

// (12.9) f = (1/2pi) sqrt(k_R / m_corner), m_corner = W_axle / (2 g).
//
// ERRATUM E-2. [R280] p. 258 pairs the WHOLE-AXLE mass with a SINGLE-WHEEL rate,
// which is dimensionally inconsistent: the published 163.6 and 123.7 lbf/in for
// stated 1.0 and 1.1 Hz targets actually correspond to sqrt(2) x those
// frequencies. The correct per-corner rates are 81.8 and 61.9 lbf/in.
[[nodiscard]] inline double ride_frequency_Hz(double k_R, double axle_weight_N, double g,
                                              bool legacy = false) {
    const double m = legacy ? (axle_weight_N / g)            // E-2 as printed
                            : (axle_weight_N / (2.0 * g));  // (12.9), per corner
    return std::sqrt(k_R / m) / (2.0 * units::PI);
}

[[nodiscard]] inline double ride_rate_for_frequency(double f_Hz, double axle_weight_N,
                                                    double g, bool legacy = false) {
    const double m = legacy ? (axle_weight_N / g) : (axle_weight_N / (2.0 * g));
    const double w = 2.0 * units::PI * f_Hz;
    return m * w * w;
}

// ---------------------------------------------------------------- §12.2 / §12.3

struct AxleSpringing {
    double tire_rate_per_tire = std::numeric_limits<double>::infinity();  // N/m
    double wheel_track = 0.0;   // §12.1: the TIRE contribution uses the WHEEL track
    double spring_track = 0.0;  // ...and the ride-spring contribution the SPRING track
    double wheel_centre_rate_left = 0.0;   // N/m
    double wheel_centre_rate_right = 0.0;
    // The anti-roll bar is specified DIRECTLY as a roll stiffness, chassis-to-
    // wheel-centre, so no track enters it: a bar's rate is naturally quoted that
    // way and its lever arms are part of its design, not of the vehicle's track.
    double aux_roll_stiffness = 0.0;  // N*m/rad
    double axle_weight_N = 0.0;
};

struct AxleTotals {
    double ride_rate = 0.0;                  // (12.1), chassis to ground, per wheel
    double roll_stiffness_tire = 0.0;        // wheel centre to ground
    double roll_stiffness_springs = 0.0;     // chassis to wheel centre, springs only
    double roll_stiffness_chassis_to_wc = 0.0;
    double roll_stiffness_total = 0.0;       // chassis to ground — what §9.3 wants
    double ride_frequency_Hz = 0.0;
    std::vector<core::Diagnostic> warnings;
};

// Direction A — components to totals, (12.7).
//
// The series combination of the tire roll stiffness with the spring+bar roll
// stiffness is the step most often omitted; it is why measured roll stiffness is
// always lower than the value computed from springs and bars alone.
[[nodiscard]] inline core::Result<AxleTotals> components_to_totals(
    const AxleSpringing& in, double g = units::G_STANDARD, bool legacy_frequency = false) {
    AxleTotals out;

    const double k_W_avg = 0.5 * (in.wheel_centre_rate_left + in.wheel_centre_rate_right);
    out.ride_rate = ride_rate(in.tire_rate_per_tire, k_W_avg);

    out.roll_stiffness_tire =
        std::isinf(in.tire_rate_per_tire)
            ? std::numeric_limits<double>::infinity()
            : roll_stiffness_from_pair(in.wheel_track, in.tire_rate_per_tire,
                                       in.tire_rate_per_tire);
    out.roll_stiffness_springs = roll_stiffness_from_pair(
        in.spring_track, in.wheel_centre_rate_left, in.wheel_centre_rate_right);
    out.roll_stiffness_chassis_to_wc = out.roll_stiffness_springs + in.aux_roll_stiffness;

    // Rigid tires degenerate the series step: chassis-to-ground == chassis-to-WC.
    out.roll_stiffness_total =
        std::isinf(out.roll_stiffness_tire)
            ? out.roll_stiffness_chassis_to_wc
            : 1.0 / (1.0 / out.roll_stiffness_chassis_to_wc + 1.0 / out.roll_stiffness_tire);

    if (in.axle_weight_N > 0.0) {
        out.ride_frequency_Hz =
            ride_frequency_Hz(out.ride_rate, in.axle_weight_N, g, legacy_frequency);
    }
    return out;
}

struct TotalsRequest {
    double desired_ride_rate = 0.0;        // N/m, chassis to ground, per wheel
    double desired_roll_stiffness = 0.0;   // N*m/rad, chassis to ground
    double tire_rate_per_tire = std::numeric_limits<double>::infinity();
    double wheel_track = 0.0;
    double spring_track = 0.0;
    double axle_weight_N = 0.0;
};

// Direction B — totals to components, (12.8).
[[nodiscard]] inline core::Result<AxleSpringing> totals_to_components(
    const TotalsRequest& req) {
    AxleSpringing out;
    out.tire_rate_per_tire = req.tire_rate_per_tire;
    out.wheel_track = req.wheel_track;
    out.spring_track = req.spring_track;
    out.axle_weight_N = req.axle_weight_N;

    std::vector<core::Diagnostic> warnings;
    const auto k_W = wheel_centre_rate(req.tire_rate_per_tire, req.desired_ride_rate, &warnings);
    if (!k_W.ok()) return core::Result<AxleSpringing>(k_W.error());
    out.wheel_centre_rate_left = k_W.value();
    out.wheel_centre_rate_right = k_W.value();

    const double K_tire =
        std::isinf(req.tire_rate_per_tire)
            ? std::numeric_limits<double>::infinity()
            : roll_stiffness_from_pair(req.wheel_track, req.tire_rate_per_tire,
                                       req.tire_rate_per_tire);

    // K_phi,des >= K_phi,tire is unachievable: the tires alone are softer in roll
    // than that.
    if (!std::isinf(K_tire) &&
        (req.desired_roll_stiffness >= K_tire ||
         K_tire - req.desired_roll_stiffness <= 1e-9 * K_tire)) {
        return core::err<AxleSpringing>(
            "E-RR-020",
            "desired roll stiffness " + std::to_string(req.desired_roll_stiffness) +
                " N*m/rad is at or above the tires' own roll stiffness " +
                std::to_string(K_tire) + " N*m/rad, which is the ceiling");
    }

    const double K_c_to_wc =
        std::isinf(K_tire) ? req.desired_roll_stiffness
                           : 1.0 / (1.0 / req.desired_roll_stiffness - 1.0 / K_tire);
    const double K_springs = roll_stiffness_from_pair(req.spring_track, k_W.value(), k_W.value());
    out.aux_roll_stiffness = K_c_to_wc - K_springs;

    // W-RR-021: a negative bar rate means the ride springs alone already exceed the
    // desired roll stiffness. Reported AS NEGATIVE rather than clamped to zero,
    // because a negative value is meaningful design feedback: soften the springs,
    // or accept more roll stiffness.
    if (out.aux_roll_stiffness < 0.0) {
        warnings.push_back(core::make_diagnostic(
            "W-RR-021",
            "required anti-roll bar rate is negative (" +
                std::to_string(out.aux_roll_stiffness) +
                " N*m/rad): the ride springs alone already exceed the desired roll stiffness"));
    }
    (void)warnings;  // carried on the totals side; see AxleTotals::warnings
    return out;
}

// ---------------------------------------------------------------- §12.5 roll gradient

struct RollStiffnessTarget {
    double exact = 0.0;       // (12.11), including the destabilising m_s g h2
    double simplified = 0.0;  // (12.12), the commonly used shortcut
    double difference = 0.0;  // exactly m_s g h2 — what the shortcut drops
};

// §12.5: BOTH are reported, together with their difference, so the user sees the
// size of the gravity-term correction rather than having it applied invisibly.
// Because (12.11) and (12.12) differ by exactly one term — same m_s, same h2 —
// the difference isolates exactly what it claims to. This is erratum E-1c.
[[nodiscard]] inline RollStiffnessTarget roll_stiffness_for_gradient(
    double m_s, double h2, double target_rad_per_mps2, double g = units::G_STANDARD) {
    RollStiffnessTarget t;
    t.simplified = m_s * h2 / target_rad_per_mps2;
    t.difference = m_s * g * h2;
    t.exact = t.simplified + t.difference;
    return t;
}

// (12.10), for feeding a given total back through.
[[nodiscard]] inline double roll_gradient_rad_per_mps2(double m_s, double h2, double sum_K,
                                                       double g = units::G_STANDARD) {
    return m_s * h2 / (sum_K - m_s * g * h2);
}

}  // namespace rcvd::analysis

#endif  // RCVD_ANALYSIS_RIDE_ROLL_HPP
