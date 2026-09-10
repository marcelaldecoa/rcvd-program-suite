// rcvd/tire/radt.hpp — model L4, Radt non-dimensional, spec §6.5.
//
// Three normalising groups collapse every load onto ONE master curve:
//
//   alpha_bar = C_alpha * tan(alpha) / (mu * Fz)                          (6.4)
//   F_bar     = Fy / (mu * Fz)                                            (6.5)
//   M_bar     = Mz / (mu * Fz * t_p)                                      (6.6)
//
// Expansion to an untested load runs (6.7) in the other direction. The collapse
// residual is reported as a data-quality metric: above 3 % means the supplied
// C_alpha, mu or t_p estimates are inconsistent with the raw table.
#ifndef RCVD_TIRE_RADT_HPP
#define RCVD_TIRE_RADT_HPP

#include <cmath>
#include <string>
#include <vector>

#include "rcvd/core/result.hpp"
#include "rcvd/numerics/pchip.hpp"
#include "rcvd/tire/tire_model.hpp"

namespace rcvd::tire {

struct RadtTireConfig {
    std::string id = "radt";
    // The master curve F_bar(alpha_bar), tabulated for alpha_bar >= 0 and mirrored
    // (the normalised curve is odd, like the dimensional one).
    std::vector<double> alpha_bar;
    std::vector<double> F_bar;
    std::vector<double> M_bar;              // optional
    // Per-load coefficients, PCHIP-interpolated in Fz.
    std::vector<double> Fz_N;
    std::vector<double> C_alpha_N_per_rad;  // negative
    std::vector<double> mu;
    std::vector<double> trail_m;
    double Fz_min_N = 1.0;                  // §6.5 guard, default 1 N
};

class RadtTire final : public TireModel {
public:
    [[nodiscard]] static core::Result<RadtTire> create(RadtTireConfig cfg) {
        const std::string who = "tire '" + cfg.id + "'";
        if (cfg.alpha_bar.size() < 2 || cfg.alpha_bar.size() != cfg.F_bar.size()) {
            return core::err<RadtTire>("E-IO-003", who + ": master curve is malformed");
        }
        if (cfg.alpha_bar.front() != 0.0 || cfg.F_bar.front() != 0.0) {
            return core::err<RadtTire>(
                "E-TIRE-003", who + ": the master curve must start at the origin (contract T2)");
        }
        const std::size_t n = cfg.Fz_N.size();
        if (n < 2 || cfg.C_alpha_N_per_rad.size() != n || cfg.mu.size() != n ||
            cfg.trail_m.size() != n) {
            return core::err<RadtTire>("E-IO-003",
                                       who + ": per-load coefficient arrays must all match Fz");
        }
        for (double c : cfg.C_alpha_N_per_rad) {
            if (c >= 0.0) {
                return core::err<RadtTire>("E-TIRE-003",
                                           who + ": every per-load C_alpha must be negative");
            }
        }
        for (double m : cfg.mu) {
            if (!(m > 0.0)) {
                return core::err<RadtTire>("E-TIRE-003", who + ": every per-load mu must be > 0");
            }
        }
        return RadtTire(std::move(cfg));
    }

    [[nodiscard]] TireOutput evaluate(const TireState& s) const override {
        TireOutput out;
        const double Fz = s.Fz_N;
        out.trail_m = trail_(std::max(cfg_.Fz_min_N, Fz));

        // §6.5 guard: mu*Fz is the denominator of (6.4). When Fz is below the
        // floor the model returns zero force per T4 WITHOUT evaluating (6.4) —
        // guarded before dividing, not after.
        if (Fz < cfg_.Fz_min_N || Fz < kFzFloorN) {
            out.C_alpha_N_per_rad = 0.0;
            out.mu_peak = mu_(std::max(cfg_.Fz_min_N, Fz));
            return out;
        }

        const double Ca = C_(Fz), mu = mu_(Fz);
        const double denom = mu * Fz;
        out.mu_peak = mu;

        // (6.4) then (6.7). tan(), not the small-angle form: the normalisation is
        // defined on the tangent and the curve reaches well past small angles.
        const double abar = Ca * std::tan(s.alpha_rad) / denom;
        const double fbar = master(abar);
        out.Fy_N = fbar * denom;
        out.saturated = std::abs(abar) > cfg_.alpha_bar.back();
        out.C_alpha_N_per_rad = dFy_dalpha(s);

        if (!M_master_.empty()) {
            out.Mz_Nm = master_M(abar) * denom * out.trail_m;
        } else {
            out.Mz_Nm = -out.trail_m * out.Fy_N;
        }
        if (out.Fy_N == 0.0) out.Mz_Nm = 0.0;
        return out;
    }

    [[nodiscard]] double dFy_dalpha(const TireState& s) const override {
        if (s.Fz_N < cfg_.Fz_min_N || s.Fz_N < kFzFloorN) return 0.0;
        const double Ca = C_(s.Fz_N), mu = mu_(s.Fz_N);
        const double denom = mu * s.Fz_N;
        const double abar = Ca * std::tan(s.alpha_rad) / denom;
        // Chain rule: dFy/dalpha = denom * dFbar/dabar * dabar/dalpha, with
        // dabar/dalpha = Ca * sec^2(alpha) / denom.
        const double sec2 = 1.0 / (std::cos(s.alpha_rad) * std::cos(s.alpha_rad));
        return denom * master_slope(abar) * (Ca * sec2 / denom);
    }

    [[nodiscard]] double dFy_dFz(const TireState& s) const override {
        if (s.Fz_N < cfg_.Fz_min_N) return 0.0;
        const double h = std::max(1.0, 1e-6 * s.Fz_N);
        return (evaluate({s.alpha_rad, s.Fz_N + h}).Fy_N -
                evaluate({s.alpha_rad, std::max(cfg_.Fz_min_N, s.Fz_N - h)}).Fy_N) /
               (s.Fz_N + h - std::max(cfg_.Fz_min_N, s.Fz_N - h));
    }

    [[nodiscard]] Interval valid_alpha_rad() const override {
        // The alpha at which the master curve's tabulated range is exhausted, at
        // the lightest load — the tightest of the loads.
        double widest = 0.0;
        for (std::size_t i = 0; i < cfg_.Fz_N.size(); ++i) {
            const double a = std::atan(cfg_.alpha_bar.back() * cfg_.mu[i] * cfg_.Fz_N[i] /
                                       std::abs(cfg_.C_alpha_N_per_rad[i]));
            widest = std::max(widest, a);
        }
        return {-widest, widest};
    }
    [[nodiscard]] Interval valid_Fz_N() const override { return {0.0, cfg_.Fz_N.back()}; }
    [[nodiscard]] std::string_view id() const override { return cfg_.id; }
    [[nodiscard]] bool has_mu() const override { return true; }
    [[nodiscard]] bool is_C1() const override { return true; }
    [[nodiscard]] std::span<const double> slope_break_points() const override { return {}; }
    [[nodiscard]] std::span<const double> interpolation_knots() const override {
        return knots_;
    }

    // §6.5 expansion: the peak location at a given load, from the master peak.
    [[nodiscard]] double peak_alpha_rad(double Fz_N) const {
        const double abar_peak = peak_abar();
        return std::atan(abar_peak * mu_(Fz_N) * Fz_N / std::abs(C_(Fz_N)));
    }
    [[nodiscard]] double peak_Fy_N(double Fz_N) const {
        return master(peak_abar()) * mu_(Fz_N) * Fz_N;
    }

    // §6.5: the collapse residual, reported as a data-quality metric. Above 3 %
    // the supplied C_alpha, mu or t_p are inconsistent with the raw data.
    [[nodiscard]] double collapse_residual(
        const std::vector<double>& alpha_rad,
        const std::vector<std::vector<double>>& Fy_by_load) const {
        double worst = 0.0;
        for (std::size_t j = 0; j < cfg_.Fz_N.size() && j < Fy_by_load.size(); ++j) {
            const double denom = cfg_.mu[j] * cfg_.Fz_N[j];
            for (std::size_t i = 0; i < alpha_rad.size() && i < Fy_by_load[j].size(); ++i) {
                const double abar =
                    cfg_.C_alpha_N_per_rad[j] * std::tan(alpha_rad[i]) / denom;
                worst = std::max(worst, std::abs(Fy_by_load[j][i] / denom - master(abar)));
            }
        }
        return worst;
    }

private:
    explicit RadtTire(RadtTireConfig cfg) : cfg_(std::move(cfg)) {
        master_ = numerics::Pchip(cfg_.alpha_bar, cfg_.F_bar);
        if (!cfg_.M_bar.empty()) M_master_ = numerics::Pchip(cfg_.alpha_bar, cfg_.M_bar);
        C_pchip_ = numerics::Pchip(cfg_.Fz_N, cfg_.C_alpha_N_per_rad);
        mu_pchip_ = numerics::Pchip(cfg_.Fz_N, cfg_.mu);
        trail_pchip_ = numerics::Pchip(cfg_.Fz_N, cfg_.trail_m);
        for (double a : cfg_.alpha_bar) {
            knots_.push_back(a);
            if (a > 0.0) knots_.push_back(-a);
        }
        std::sort(knots_.begin(), knots_.end());
    }

    // The master curve is odd; tabulate the positive half and mirror, which makes
    // T1 exact by construction.
    [[nodiscard]] double master(double abar) const {
        return abar < 0.0 ? -master_(-abar) : master_(abar);
    }
    [[nodiscard]] double master_slope(double abar) const {
        return master_.derivative(std::abs(abar));   // slope is even
    }
    [[nodiscard]] double master_M(double abar) const {
        return abar < 0.0 ? -M_master_(-abar) : M_master_(abar);
    }
    [[nodiscard]] double peak_abar() const {
        double best = 0.0, best_v = 0.0;
        for (std::size_t i = 0; i < cfg_.alpha_bar.size(); ++i) {
            if (std::abs(cfg_.F_bar[i]) > best_v) {
                best_v = std::abs(cfg_.F_bar[i]);
                best = cfg_.alpha_bar[i];
            }
        }
        return best;
    }
    [[nodiscard]] double C_(double Fz) const { return C_pchip_(Fz); }
    [[nodiscard]] double mu_(double Fz) const { return mu_pchip_(Fz); }
    [[nodiscard]] double trail_(double Fz) const { return trail_pchip_(Fz); }

    RadtTireConfig cfg_;
    numerics::Pchip master_, M_master_, C_pchip_, mu_pchip_, trail_pchip_;
    std::vector<double> knots_;
};

}  // namespace rcvd::tire

#endif  // RCVD_TIRE_RADT_HPP
