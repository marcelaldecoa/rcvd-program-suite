// rcvd/tire/magic_formula.hpp — model L5, Pacejka Magic Formula (pure lateral
// slip), spec §6.6.
//
//   Fy = D sin( C arctan( Bx - E(Bx - arctan(Bx)) ) ) + S_V,   x = alpha + S_H
//
// with D = peak and BCD = dFy/dx at x = 0, i.e. the cornering stiffness.
//
// SIGN CONVENTION (§5.2). §6.6's box constraints (B > 0, 1 <= C <= 3, D > 0) apply
// to the fit of the NORMALISED curve of §6.5, where alpha_bar and F_bar share a
// sign and B*C*D = +1. In a DIMENSIONAL per-load set the tire must satisfy T3
// (alpha*Fy <= 0), so D is stored NEGATIVE — the peak lateral force in the SAE
// tire frame — and B*C*D = C_alpha < 0. The loader checks B*C*D < 0 for a
// dimensional block and > 0 for a normalised one.
#ifndef RCVD_TIRE_MAGIC_FORMULA_HPP
#define RCVD_TIRE_MAGIC_FORMULA_HPP

#include <cmath>
#include <string>
#include <vector>

#include "rcvd/core/result.hpp"
#include "rcvd/numerics/pchip.hpp"
#include "rcvd/tire/tire_model.hpp"

namespace rcvd::tire {

struct MagicFormulaCoefficients {
    double B = 0.0, C = 0.0, D = 0.0, E = 0.0;
    double S_H = 0.0, S_V = 0.0;

    // (6.8). Written so that x = 0 gives exactly S_V, which keeps T2 exact when
    // the shifts are zero.
    [[nodiscard]] double evaluate(double alpha) const {
        const double x = alpha + S_H;
        const double Bx = B * x;
        const double inner = Bx - E * (Bx - std::atan(Bx));
        return D * std::sin(C * std::atan(inner)) + S_V;
    }
    [[nodiscard]] double slope(double alpha) const {
        const double x = alpha + S_H;
        const double Bx = B * x;
        const double inner = Bx - E * (Bx - std::atan(Bx));
        // d(inner)/dx = B(1 - E) + E*B/(1 + (Bx)^2)
        const double d_inner = B * (1.0 - E) + E * B / (1.0 + Bx * Bx);
        return D * std::cos(C * std::atan(inner)) * C / (1.0 + inner * inner) * d_inner;
    }
    // BCD is the cornering stiffness by construction (§6.6).
    [[nodiscard]] double BCD() const { return B * C * D; }
};

struct MagicFormulaConfig {
    std::string id = "mf";
    std::vector<double> Fz_N;                          // per-load coefficient sets
    std::vector<MagicFormulaCoefficients> coefficients;
    double pneumatic_trail_m = 0.0;
    // A normalised block (the §6.5 master curve) has B*C*D = +1; a dimensional one
    // has B*C*D = C_alpha < 0.
    bool normalised = false;
    double valid_alpha_half_range_rad = 0.35;
};

class MagicFormulaTire final : public TireModel {
public:
    [[nodiscard]] static core::Result<MagicFormulaTire> create(MagicFormulaConfig cfg) {
        const std::string who = "tire '" + cfg.id + "'";
        if (cfg.Fz_N.size() != cfg.coefficients.size() || cfg.Fz_N.empty()) {
            return core::err<MagicFormulaTire>(
                "E-IO-003", who + ": one coefficient set is required per load");
        }
        for (std::size_t i = 0; i < cfg.coefficients.size(); ++i) {
            const auto& c = cfg.coefficients[i];
            const std::string at = " (load index " + std::to_string(i) + ")";
            // §6.6 box constraints. B > 0 and 1 <= C <= 3 hold in both spellings;
            // only D's sign differs.
            if (!(c.B > 0.0)) {
                return core::err<MagicFormulaTire>("E-TIRE-003", who + ": B must be > 0" + at);
            }
            if (c.C < 1.0 || c.C > 3.0) {
                return core::err<MagicFormulaTire>("E-TIRE-003",
                                                   who + ": C must be in [1, 3]" + at);
            }
            if (c.E > 1.0) {
                return core::err<MagicFormulaTire>("E-TIRE-003", who + ": E must be <= 1" + at);
            }
            // §5.2's sign check, in both directions.
            if (cfg.normalised && !(c.BCD() > 0.0)) {
                return core::err<MagicFormulaTire>(
                    "E-TIRE-003",
                    who + ": a normalised block must have B*C*D > 0, got " +
                        std::to_string(c.BCD()) + at);
            }
            if (!cfg.normalised && !(c.BCD() < 0.0)) {
                return core::err<MagicFormulaTire>(
                    "E-TIRE-003",
                    who + ": a dimensional block must have B*C*D < 0 (D is the peak lateral "
                          "force in the SAE tire frame, so it is negative), got " +
                        std::to_string(c.BCD()) + at);
            }
        }
        return MagicFormulaTire(std::move(cfg));
    }

    [[nodiscard]] TireOutput evaluate(const TireState& s) const override {
        TireOutput out;
        out.trail_m = cfg_.pneumatic_trail_m;
        if (s.Fz_N < kFzFloorN) {   // T4
            out.C_alpha_N_per_rad = 0.0;
            out.mu_peak = peak_mu(kFzFloorN);
            return out;
        }
        const MagicFormulaCoefficients c = at_load(s.Fz_N);
        out.Fy_N = c.evaluate(s.alpha_rad);
        out.C_alpha_N_per_rad = c.slope(s.alpha_rad);
        out.mu_peak = peak_mu(s.Fz_N);
        // Saturated once past the peak, where the slope has changed sign.
        out.saturated = out.C_alpha_N_per_rad * c.BCD() < 0.0;
        out.Mz_Nm = -out.trail_m * out.Fy_N;
        if (out.Fy_N == 0.0) out.Mz_Nm = 0.0;
        return out;
    }

    [[nodiscard]] double dFy_dalpha(const TireState& s) const override {
        if (s.Fz_N < kFzFloorN) return 0.0;
        return at_load(s.Fz_N).slope(s.alpha_rad);
    }

    [[nodiscard]] double dFy_dFz(const TireState& s) const override {
        if (s.Fz_N < kFzFloorN || cfg_.Fz_N.size() < 2) return 0.0;
        const double h = std::max(1.0, 1e-6 * s.Fz_N);
        return (evaluate({s.alpha_rad, s.Fz_N + h}).Fy_N -
                evaluate({s.alpha_rad, std::max(kFzFloorN, s.Fz_N - h)}).Fy_N) /
               (s.Fz_N + h - std::max(kFzFloorN, s.Fz_N - h));
    }

    [[nodiscard]] Interval valid_alpha_rad() const override {
        return {-cfg_.valid_alpha_half_range_rad, cfg_.valid_alpha_half_range_rad};
    }
    [[nodiscard]] Interval valid_Fz_N() const override { return {0.0, cfg_.Fz_N.back()}; }
    [[nodiscard]] std::string_view id() const override { return cfg_.id; }
    [[nodiscard]] bool has_mu() const override { return !cfg_.normalised; }
    [[nodiscard]] bool is_C1() const override { return true; }
    [[nodiscard]] std::span<const double> slope_break_points() const override { return {}; }
    [[nodiscard]] std::span<const double> interpolation_knots() const override {
        return cfg_.Fz_N.size() > 1 ? std::span<const double>{} : std::span<const double>{};
    }

    [[nodiscard]] MagicFormulaCoefficients at_load(double Fz_N) const {
        if (cfg_.Fz_N.size() == 1) return cfg_.coefficients.front();
        MagicFormulaCoefficients c;
        c.B = B_(Fz_N);
        c.C = C_(Fz_N);
        c.D = D_(Fz_N);
        c.E = E_(Fz_N);
        c.S_H = SH_(Fz_N);
        c.S_V = SV_(Fz_N);
        return c;
    }

private:
    explicit MagicFormulaTire(MagicFormulaConfig cfg) : cfg_(std::move(cfg)) {
        if (cfg_.Fz_N.size() < 2) return;
        // §6.6: load dependence supplied as explicit per-load sets with PCHIP
        // interpolation in Fz. The MF load-scaling coefficients are an extension
        // seam; Rev 1.0 requires the former.
        const auto column = [&](double MagicFormulaCoefficients::*field) {
            std::vector<double> v;
            for (const auto& c : cfg_.coefficients) v.push_back(c.*field);
            return numerics::Pchip(cfg_.Fz_N, v);
        };
        B_ = column(&MagicFormulaCoefficients::B);
        C_ = column(&MagicFormulaCoefficients::C);
        D_ = column(&MagicFormulaCoefficients::D);
        E_ = column(&MagicFormulaCoefficients::E);
        SH_ = column(&MagicFormulaCoefficients::S_H);
        SV_ = column(&MagicFormulaCoefficients::S_V);
    }

    [[nodiscard]] double peak_mu(double Fz_N) const {
        return std::abs(at_load(Fz_N).D) / std::max(kFzFloorN, Fz_N);
    }

    MagicFormulaConfig cfg_;
    numerics::Pchip B_, C_, D_, E_, SH_, SV_;
};

}  // namespace rcvd::tire

#endif  // RCVD_TIRE_MAGIC_FORMULA_HPP
