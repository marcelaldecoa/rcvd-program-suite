// rcvd/tire/table.hpp — model L3, tabular, spec §6.4.
//
// INTERPOLATION IS NORMATIVE. Bilinear interpolation SHALL NOT be used: the
// resulting slope discontinuities across cell boundaries produce the visible
// "kinks" in steady-state-response plots noted at [R280] p. 255, and corrupt the
// numerical Jacobians used by the steady-state, MMM and implicit load-transfer
// solvers. Fritsch-Carlson PCHIP in both directions, tensor product, evaluated
// alpha-first then Fz (deviation D-2).
#ifndef RCVD_TIRE_TABLE_HPP
#define RCVD_TIRE_TABLE_HPP

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "rcvd/core/result.hpp"
#include "rcvd/numerics/pchip.hpp"
#include "rcvd/tire/friction.hpp"
#include "rcvd/tire/tire_model.hpp"

namespace rcvd::tire {

struct TableTireConfig {
    std::string id = "table";
    std::vector<double> alpha_rad;   // strictly increasing
    std::vector<double> Fz_N;        // strictly increasing
    // Fy[i][j] = Fy(alpha_i, Fz_j), rows indexed by alpha (§5.2 "rows_are").
    std::vector<std::vector<double>> Fy_N;
    std::vector<std::vector<double>> Mz_Nm;   // optional
    // §6.4: if the table covers only alpha >= 0, the model mirrors per T1 rather
    // than requiring the user to duplicate rows.
    bool mirror_negative_alpha = true;
    double pneumatic_trail_m = 0.0;
    // §6.4: linear extrapolation in Fz is permitted up to 25 % beyond the last
    // node, beyond which the model raises E-TIRE-020.
    double Fz_extrapolation_fraction = 0.25;
};

class TableTire final : public TireModel {
public:
    [[nodiscard]] static core::Result<TableTire> create(TableTireConfig cfg) {
        const std::string who = "tire '" + cfg.id + "'";
        if (cfg.alpha_rad.size() < 2 || cfg.Fz_N.size() < 2) {
            return core::err<TableTire>("E-IO-003",
                                        who + ": need at least two alpha and two Fz nodes");
        }
        if (cfg.Fy_N.size() != cfg.alpha_rad.size()) {
            return core::err<TableTire>("E-IO-003", who + ": Fy row count must match alpha");
        }
        for (const auto& row : cfg.Fy_N) {
            if (row.size() != cfg.Fz_N.size()) {
                return core::err<TableTire>("E-IO-003", who + ": Fy column count must match Fz");
            }
        }
        if (!std::is_sorted(cfg.alpha_rad.begin(), cfg.alpha_rad.end()) ||
            !std::is_sorted(cfg.Fz_N.begin(), cfg.Fz_N.end())) {
            return core::err<TableTire>("E-IO-003",
                                        who + ": alpha and Fz must be strictly increasing");
        }
        if (cfg.mirror_negative_alpha && cfg.alpha_rad.front() < 0.0) {
            return core::err<TableTire>(
                "E-IO-003",
                who + ": mirror_negative_alpha is set but the table already spans negative "
                      "alpha; supply one or the other");
        }
        // T2 is exact only if the data says so.
        if (cfg.mirror_negative_alpha && cfg.alpha_rad.front() == 0.0) {
            for (double v : cfg.Fy_N.front()) {
                if (v != 0.0) {
                    return core::err<TableTire>(
                        "E-TIRE-003",
                        who + ": Fy at alpha = 0 must be exactly zero (contract T2)");
                }
            }
        }
        return TableTire(std::move(cfg));
    }

    [[nodiscard]] TireOutput evaluate(const TireState& s) const override {
        TireOutput out;
        out.trail_m = cfg_.pneumatic_trail_m;   // T8

        if (s.Fz_N < kFzFloorN) {   // T4, guarded before any use of Fz
            out.C_alpha_N_per_rad = 0.0;
            return out;
        }

        // T1 by construction when mirroring: evaluate at |alpha| and carry the sign.
        const double a = cfg_.mirror_negative_alpha ? std::abs(s.alpha_rad) : s.alpha_rad;
        const double sign = (cfg_.mirror_negative_alpha && s.alpha_rad < 0.0) ? -1.0 : 1.0;

        out.Fy_N = sign * interpolate(a, s.Fz_N, Fy_);
        out.C_alpha_N_per_rad = dFy_dalpha(s);
        out.saturated = a < cfg_.alpha_rad.front() || a > cfg_.alpha_rad.back();

        if (!Mz_.empty()) {
            out.Mz_Nm = sign * interpolate(a, s.Fz_N, Mz_);
        } else {
            out.Mz_Nm = -out.trail_m * out.Fy_N;   // (6.2)
        }
        if (out.Fy_N == 0.0) out.Mz_Nm = 0.0;
        return out;
    }

    [[nodiscard]] double dFy_dalpha(const TireState& s) const override {
        if (s.Fz_N < kFzFloorN) return 0.0;
        const double a = cfg_.mirror_negative_alpha ? std::abs(s.alpha_rad) : s.alpha_rad;
        // The mirrored curve's slope is EVEN in alpha: d/dalpha[sgn*f(|alpha|)] is
        // f'(|alpha|) on both sides.
        std::vector<double> at_nodes(cfg_.Fz_N.size());
        for (std::size_t j = 0; j < cfg_.Fz_N.size(); ++j) {
            at_nodes[j] = Fy_[j].derivative(a);
        }
        return interpolate_in_Fz(s.Fz_N, at_nodes);
    }

    [[nodiscard]] double dFy_dFz(const TireState& s) const override {
        if (s.Fz_N < kFzFloorN) return 0.0;
        const double h = std::max(1.0, 1e-6 * s.Fz_N);
        const TireState lo{s.alpha_rad, std::max(kFzFloorN, s.Fz_N - h)};
        const TireState hi{s.alpha_rad, s.Fz_N + h};
        return (evaluate(hi).Fy_N - evaluate(lo).Fy_N) / (hi.Fz_N - lo.Fz_N);
    }

    [[nodiscard]] Interval valid_alpha_rad() const override {
        const double hi = cfg_.alpha_rad.back();
        return cfg_.mirror_negative_alpha ? Interval{-hi, hi}
                                          : Interval{cfg_.alpha_rad.front(), hi};
    }
    [[nodiscard]] Interval valid_Fz_N() const override {
        return {0.0, cfg_.Fz_N.back() * (1.0 + cfg_.Fz_extrapolation_fraction)};
    }
    [[nodiscard]] std::string_view id() const override { return cfg_.id; }

    // §6.4 gives no friction limit; the table IS the force. A separate friction
    // model may be layered on, but this model does not claim one.
    [[nodiscard]] bool has_mu() const override { return false; }
    // PCHIP is C1, so unlike a linear-interpolated table this model does declare
    // C1 and has no slope breaks (deviation D-2's whole point).
    [[nodiscard]] bool is_C1() const override { return true; }
    [[nodiscard]] std::span<const double> slope_break_points() const override { return {}; }
    // NOT slope breaks — the interpolant is C1 at a knot — but a curvature jump
    // that (13.2)'s Richardson stencil cannot straddle (contract T6).
    [[nodiscard]] std::span<const double> interpolation_knots() const override {
        return knots_;
    }

    // §6.4: silent extrapolation of tire data is a correctness hazard, not a
    // convenience.
    [[nodiscard]] bool beyond_Fz_range(double Fz_N) const {
        return Fz_N > cfg_.Fz_N.back() * (1.0 + cfg_.Fz_extrapolation_fraction);
    }

    // The checked entry point. evaluate() cannot return a Result -- the TireModel
    // contract is a pure function called once per wheel per integrator stage, and
    // §14.1 Rule D3 says solvers assert rather than re-validate -- so the model
    // offers this for the loading and setup paths, where E-TIRE-020 belongs.
    [[nodiscard]] core::Result<TireOutput> checked_evaluate(const TireState& s) const {
        if (beyond_Fz_range(s.Fz_N)) {
            return core::err<TireOutput>(
                "E-TIRE-020",
                "tire '" + cfg_.id + "': load " + std::to_string(s.Fz_N) +
                    " N is more than " +
                    std::to_string(100.0 * cfg_.Fz_extrapolation_fraction) +
                    " % beyond the last tabulated node (" +
                    std::to_string(cfg_.Fz_N.back()) + " N); extrapolating tire data "
                    "further is a correctness hazard, not a convenience");
        }
        return evaluate(s);
    }

private:
    explicit TableTire(TableTireConfig cfg) : cfg_(std::move(cfg)) {
        // One alpha-PCHIP per load column.
        for (std::size_t j = 0; j < cfg_.Fz_N.size(); ++j) {
            std::vector<double> column(cfg_.alpha_rad.size());
            for (std::size_t i = 0; i < cfg_.alpha_rad.size(); ++i) column[i] = cfg_.Fy_N[i][j];
            Fy_.emplace_back(cfg_.alpha_rad, column);
        }
        if (!cfg_.Mz_Nm.empty()) {
            for (std::size_t j = 0; j < cfg_.Fz_N.size(); ++j) {
                std::vector<double> column(cfg_.alpha_rad.size());
                for (std::size_t i = 0; i < cfg_.alpha_rad.size(); ++i) {
                    column[i] = cfg_.Mz_Nm[i][j];
                }
                Mz_.emplace_back(cfg_.alpha_rad, column);
            }
        }
        // Knots, mirrored if the model mirrors, so T6 can exclude both sides.
        for (double a : cfg_.alpha_rad) {
            knots_.push_back(a);
            if (cfg_.mirror_negative_alpha && a > 0.0) knots_.push_back(-a);
        }
        std::sort(knots_.begin(), knots_.end());
    }

    // §6.4: tensor product, evaluated ALPHA-FIRST then Fz.
    [[nodiscard]] double interpolate(double alpha, double Fz,
                                     const std::vector<numerics::Pchip>& curves) const {
        std::vector<double> at_nodes(curves.size());
        for (std::size_t j = 0; j < curves.size(); ++j) at_nodes[j] = curves[j](alpha);
        return interpolate_in_Fz(Fz, at_nodes);
    }

    [[nodiscard]] double interpolate_in_Fz(double Fz, const std::vector<double>& values) const {
        const numerics::Pchip in_load(cfg_.Fz_N, values);
        if (Fz >= cfg_.Fz_N.front() && Fz <= cfg_.Fz_N.back()) return in_load(Fz);
        // §6.4: linear extrapolation in Fz, up to the declared fraction. Pchip
        // holds the boundary value, so extend along the end slope by hand.
        if (Fz < cfg_.Fz_N.front()) {
            const double x0 = cfg_.Fz_N.front();
            return in_load(x0) + in_load.derivative(x0 + 1e-9) * (Fz - x0);
        }
        const double x1 = cfg_.Fz_N.back();
        return in_load(x1) + in_load.derivative(x1 - 1e-9) * (Fz - x1);
    }

    TableTireConfig cfg_;
    std::vector<numerics::Pchip> Fy_, Mz_;
    std::vector<double> knots_;
};

}  // namespace rcvd::tire

#endif  // RCVD_TIRE_TABLE_HPP
