// rcvd/aero/aero_model.hpp — spec §7.
//
// Sign convention: C_L < 0 denotes DOWNFORCE. The resulting vertical force
// INCREASES the tire vertical-load magnitude F_z of §6 (decision D-1), so the
// adapter adds -q*C_L*A_f to each axle's load.
#ifndef RCVD_AERO_AERO_MODEL_HPP
#define RCVD_AERO_AERO_MODEL_HPP

#include <cmath>
#include <optional>
#include <string>

#include "rcvd/units/constants.hpp"

namespace rcvd::aero {

// ISO standard atmosphere, 15 C, 101.325 kPa.
inline constexpr double kRhoStd = 1.2250;      // kg/m^3
inline constexpr double kPStd   = 101325.0;    // Pa
inline constexpr double kTStd   = 288.15;      // K

struct AeroConfig {
    std::string id;
    double rho_kg_m3 = kRhoStd;
    double frontal_area_m2 = 0.0;
    double CD  = 0.0;
    double CLF = 0.0;   // negative => front downforce
    double CLR = 0.0;
    // §5.3. Below this speed the aerodynamic forces are taken as zero: q ~ V^2 is
    // negligible there, and it keeps a stationary vehicle from carrying a
    // meaningless balance figure.
    double reference_speed_floor_mps = 1.0;
};

struct AeroForces {
    double q_Pa{};              // (7.1) dynamic pressure
    double drag_N{};            // (7.2) along -x
    double lift_front_N{};      // (7.3) signed, per axle
    double lift_rear_N{};       // (7.4)
    // The D-1 adapter output: what to ADD to each axle's F_z magnitude. Positive
    // for downforce.
    double downforce_front_N{};
    double downforce_rear_N{};
};

// (7.5) altitude / temperature correction.
[[nodiscard]] inline double air_density(double pressure_Pa, double temperature_K) {
    return kRhoStd * (pressure_Pa / kPStd) * (kTStd / temperature_K);
}

// §7: V is the speed MAGNITUDE sqrt(u^2 + v^2), not u. At the sideslip angles
// P2/P5 reach the difference is small, but the choice is explicit and documented
// rather than incidental.
[[nodiscard]] inline double speed_magnitude(double u_mps, double v_mps) {
    return std::hypot(u_mps, v_mps);
}

[[nodiscard]] inline AeroForces evaluate(const AeroConfig& c, double V_mps) {
    AeroForces f;
    if (V_mps < c.reference_speed_floor_mps) return f;  // all zero

    f.q_Pa = 0.5 * c.rho_kg_m3 * V_mps * V_mps;         // (7.1)
    const double qA = f.q_Pa * c.frontal_area_m2;
    f.drag_N       = qA * c.CD;                          // (7.2)
    f.lift_front_N = qA * c.CLF;                         // (7.3)
    f.lift_rear_N  = qA * c.CLR;                         // (7.4)
    // §9.2: added per axle, split evenly left/right by the caller.
    f.downforce_front_N = -f.lift_front_N;
    f.downforce_rear_N  = -f.lift_rear_N;
    return f;
}

// §7 aerodynamic balance. Both are UNDEFINED when CLF + CLR == 0; the spec
// requires reporting n/a rather than a division result, which is what the
// disengaged optional means here.
struct AeroBalance {
    std::optional<double> percent_front;
    std::optional<double> x_cp_aft_of_front_axle_m;
};

// MSVC C4723, "potential divide by 0", is a FALSE POSITIVE on this function: the
// guard returns before either division whenever `sum` could be zero. It appears
// only in an OPTIMISED build, and only when the n/a path is exercised with
// exactly-cancelling constants through Catch2's expression-decomposition
// templates — constant propagation folds `sum` to 0.0 and the analyser sees the
// division in a branch it has not yet eliminated. Neither the function alone nor
// the same call site outside Catch2 reproduces it.
//
// The pragma must enclose the WHOLE FUNCTION, not just the division statements:
// C4723 is emitted by the optimiser back end, and push/disable inside a function
// body has no effect on back-end warnings. §4.2's /WX stays in force everywhere
// else, and this is deliberately NOT "fixed" by adding `sum == 0.0` to the guard
// — §13.1 forbids bare equality against zero and the tolerance test subsumes it.
#if defined(_MSC_VER)
#  pragma warning(push)
#  pragma warning(disable : 4723)
#endif
[[nodiscard]] inline AeroBalance balance(const AeroConfig& c, double wheelbase_m) {
    AeroBalance b;
    const double sum = c.CLF + c.CLR;
    if (std::abs(sum) < 1e-9) return b;  // n/a
    b.percent_front = c.CLF / sum;
    b.x_cp_aft_of_front_axle_m = wheelbase_m * c.CLR / sum;
    return b;
}
#if defined(_MSC_VER)
#  pragma warning(pop)
#endif

}  // namespace rcvd::aero

#endif  // RCVD_AERO_AERO_MODEL_HPP
