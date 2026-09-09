// tests/support/defective_tire.hpp — fault injection for the §6.1 conformance
// suite, per Appendix D ("Every clamp and every extrapolation raises an event —
// verified by fault-injection tests").
//
// A conformance suite that cannot fail is worthless. This model is a correct
// linear tire into which exactly one contract violation can be injected, so the
// suite can be shown to catch each of T1-T8 rather than merely to pass.
#ifndef RCVD_TESTS_SUPPORT_DEFECTIVE_TIRE_HPP
#define RCVD_TESTS_SUPPORT_DEFECTIVE_TIRE_HPP

#include <cmath>
#include <limits>
#include <string_view>

#include "rcvd/tire/tire_model.hpp"

namespace rcvd::tire {

enum class Defect {
    none,
    T1_not_odd,        // asymmetric in alpha
    T2_nonzero_at_0,   // Fy(0) != 0
    T3_wrong_sign,     // alpha*Fy > 0
    T4_force_at_zero,  // produces force on a lifted wheel
    T5_mu_engaged,     // has_mu() false but mu_peak set
    T6_wrong_slope,    // analytic derivative disagrees with the curve
    T7_discontinuous,  // a step in Fy
    T8_nan_trail,      // trail_m is NaN — the 0/0 T8 exists to prevent
};

[[nodiscard]] inline std::string_view defect_name(Defect d) {
    switch (d) {
        case Defect::none:             return "none";
        case Defect::T1_not_odd:       return "T1_not_odd";
        case Defect::T2_nonzero_at_0:  return "T2_nonzero_at_0";
        case Defect::T3_wrong_sign:    return "T3_wrong_sign";
        case Defect::T4_force_at_zero: return "T4_force_at_zero";
        case Defect::T5_mu_engaged:    return "T5_mu_engaged";
        case Defect::T6_wrong_slope:   return "T6_wrong_slope";
        case Defect::T7_discontinuous: return "T7_discontinuous";
        case Defect::T8_nan_trail:     return "T8_nan_trail";
    }
    return "?";
}

[[nodiscard]] inline std::string_view expected_check(Defect d) {
    switch (d) {
        case Defect::none:             return "";
        case Defect::T1_not_odd:       return "T1";
        case Defect::T2_nonzero_at_0:  return "T2";
        case Defect::T3_wrong_sign:    return "T3";
        case Defect::T4_force_at_zero: return "T4";
        case Defect::T5_mu_engaged:    return "T5";
        case Defect::T6_wrong_slope:   return "T6";
        case Defect::T7_discontinuous: return "T7";
        case Defect::T8_nan_trail:     return "T8";
    }
    return "";
}

class DefectiveTire final : public TireModel {
public:
    DefectiveTire(double C_alpha_N_per_rad, Defect d)
        : Ca_(C_alpha_N_per_rad), d_(d) {}

    [[nodiscard]] TireOutput evaluate(const TireState& s) const override {
        TireOutput o;
        o.trail_m = (d_ == Defect::T8_nan_trail) ? std::numeric_limits<double>::quiet_NaN()
                                                 : 0.03;

        if (s.Fz_N < kFzFloorN && d_ != Defect::T4_force_at_zero) {
            o.Fy_N = 0.0;
            o.Mz_Nm = 0.0;
            o.C_alpha_N_per_rad = 0.0;
            if (d_ == Defect::T5_mu_engaged) o.mu_peak = 1.0;
            return o;
        }

        double a = s.alpha_rad;
        double fy = Ca_ * a;

        switch (d_) {
            case Defect::T1_not_odd:
                if (a > 0.0) fy *= 1.05;                  // one side stiffer
                break;
            case Defect::T2_nonzero_at_0:
                fy += 1.0;                                 // constant offset
                break;
            case Defect::T3_wrong_sign:
                fy = -fy;                                  // positive C_alpha in effect
                break;
            case Defect::T7_discontinuous:
                if (a > 0.01) fy += 50.0;                  // a step, not a kink
                break;
            default:
                break;
        }

        o.Fy_N = fy;
        o.C_alpha_N_per_rad = (d_ == Defect::T6_wrong_slope) ? Ca_ * 1.5 : Ca_;
        o.Mz_Nm = -o.trail_m * o.Fy_N;
        if (o.Fy_N == 0.0) o.Mz_Nm = 0.0;
        if (d_ == Defect::T5_mu_engaged) o.mu_peak = 1.0;  // has_mu() is false
        return o;
    }

    [[nodiscard]] double dFy_dalpha(const TireState&) const override {
        return (d_ == Defect::T6_wrong_slope) ? Ca_ * 1.5 : Ca_;
    }
    [[nodiscard]] double dFy_dFz(const TireState&) const override { return 0.0; }
    [[nodiscard]] Interval valid_alpha_rad() const override { return {-0.05, 0.05}; }
    [[nodiscard]] Interval valid_Fz_N() const override { return {0.0, 20000.0}; }
    [[nodiscard]] std::string_view id() const override { return "defective"; }
    [[nodiscard]] bool has_mu() const override { return false; }
    // The injected T7 defect is a genuine discontinuity, not a declared kink, so
    // the model still claims C1 — which is exactly the lie the check must catch.
    [[nodiscard]] bool is_C1() const override { return true; }
    [[nodiscard]] std::span<const double> slope_break_points() const override { return {}; }

private:
    double Ca_;
    Defect d_;
};

}  // namespace rcvd::tire

#endif  // RCVD_TESTS_SUPPORT_DEFECTIVE_TIRE_HPP
