// rcvd/tire/tire_model.hpp — the TireModel contract, spec §4.3 and §6.1.
//
// "The tire is the dominant model-form uncertainty in every result this suite
// produces. Accordingly the tire interface is the most tightly specified
// component." (§6) The invariants T1-T8 are enforced by the shared conformance
// test in conformance.hpp, applied to every concrete model.
#ifndef RCVD_TIRE_TIRE_MODEL_HPP
#define RCVD_TIRE_TIRE_MODEL_HPP

#include <optional>
#include <span>
#include <string_view>

namespace rcvd::tire {

// §5.7: a closed real interval.
struct Interval {
    double lo{};
    double hi{};
    [[nodiscard]] constexpr bool contains(double v) const noexcept { return v >= lo && v <= hi; }
};

struct TireState {
    double alpha_rad{};      // slip angle
    double Fz_N{};           // vertical load MAGNITUDE, >= 0 (decision D-1)
    double gamma_rad = 0.0;  // inclination angle (reserved, §20 seam 4)
    double kappa     = 0.0;  // slip ratio (reserved, §20 seam 3)
    double u_mps     = 0.0;  // rolling speed, for the §6.9 relaxation filter
};

struct TireOutput {
    double Fy_N{};                 // lateral force, SAE tire frame
    double Mz_Nm{};                // aligning moment
    double trail_m{};              // pneumatic trail; see T8 — never NaN
    // §6.1 T5: engaged iff has_mu(). std::optional and NOT a NaN sentinel,
    // because §13.1 makes a NaN a fatal defect, so it cannot double as "absent".
    std::optional<double> mu_peak{};
    double C_alpha_N_per_rad{};    // LOCAL slope dFy/dalpha at this state
    bool   saturated = false;
};

// §6.1 T8 / (6.2a): below this |Fy| the trail is taken as the model's t_p(Fz)
// limit rather than evaluated as -Mz/Fy, which is 0/0 at alpha = 0.
inline constexpr double kFyFloorN = 1e-6;

// §6.5 guard / §13.9: below this load the tire produces no force (T4).
inline constexpr double kFzFloorN = 1.0;

class TireModel {
public:
    virtual ~TireModel() = default;

    // Copy is deleted: copying through a base reference slices, and a sliced tire
    // is a different tire. Move is allowed but protected, so only a derived class
    // can invoke it — which is what lets a concrete model be returned by value in
    // a Result<T> without opening the slicing hole.
    TireModel(const TireModel&) = delete;
    TireModel& operator=(const TireModel&) = delete;

    [[nodiscard]] virtual TireOutput evaluate(const TireState&) const = 0;

    // Analytic where available; otherwise a central difference with the step of
    // §13.6. T6 checks the two against each other.
    [[nodiscard]] virtual double dFy_dalpha(const TireState&) const = 0;
    [[nodiscard]] virtual double dFy_dFz(const TireState&) const = 0;

    [[nodiscard]] virtual Interval valid_alpha_rad() const = 0;
    [[nodiscard]] virtual Interval valid_Fz_N() const = 0;
    [[nodiscard]] virtual std::string_view id() const = 0;

    // Model-capability queries used by the §6.1 conformance suite and the solvers.
    [[nodiscard]] virtual bool has_mu() const = 0;  // false for L1, L2
    [[nodiscard]] virtual bool is_C1() const = 0;   // false for L2, linear-interp table
    [[nodiscard]] virtual std::span<const double> slope_break_points() const = 0;

    // Interpolation knots, for the T6 knot exclusion. These are NOT slope breaks
    // — a PCHIP interpolant is C1 at a knot, so slope_break_points() correctly
    // omits them — but (13.2)'s Richardson stencil still degrades across the
    // curvature jump. Empty for models with no tabulated abscissa.
    [[nodiscard]] virtual std::span<const double> interpolation_knots() const {
        return {};
    }

    // §14.1 Rule D2 is the caller's to honour: a solver checks this and records
    // the excursion. Deliberately not tracked inside the model — mutable state in
    // evaluate() would race under the §18.1 parallel sweep and would make the
    // model something other than a pure function of its state.
    [[nodiscard]] bool in_valid_range(const TireState& s) const {
        return valid_alpha_rad().contains(s.alpha_rad) && valid_Fz_N().contains(s.Fz_N);
    }

protected:
    TireModel() = default;
    TireModel(TireModel&&) = default;
    TireModel& operator=(TireModel&&) = default;
};

}  // namespace rcvd::tire

#endif  // RCVD_TIRE_TIRE_MODEL_HPP
