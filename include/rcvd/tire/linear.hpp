// rcvd/tire/linear.hpp — models L1 and L2, spec §6.2-6.3, with the §6.7
// cornering-stiffness load sensitivity (6.10).
//
// Both are PURE STIFFNESS models: they have no friction limit, so has_mu() is
// false and mu_peak is disengaged (§6.1 T5). An L2 tire can therefore be asked to
// produce a force exceeding any physical mu*Fz — as the §17.6 MMM cases do — and
// that is a property of the model, not a defect.
#ifndef RCVD_TIRE_LINEAR_HPP
#define RCVD_TIRE_LINEAR_HPP

#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include "rcvd/core/result.hpp"
#include "rcvd/tire/tire_model.hpp"

namespace rcvd::tire {

// §6.7 (6.10): C_alpha(Fz) = c0*Fz + c1*Fz^2, with c0 < 0 and c1 > 0.
struct QuadraticStiffness {
    double c0{};  // 1/rad      (dimension 1/angle)
    double c1{};  // 1/(N*rad)  (dimension 1/(force*angle))
};

struct LinearTireConfig {
    std::string id = "linear";

    // §5.2 redundancy rule: EXACTLY ONE of these. Supplying both is E-TIRE-005;
    // supplying neither leaves the tire undefined. There is no precedence rule,
    // deliberately — silent precedence between two sources of C_alpha is how a
    // tire silently becomes a different tire.
    std::optional<double>             C_alpha_N_per_rad;  // constant
    std::optional<QuadraticStiffness> load_sensitivity;   // (6.10)

    double pneumatic_trail_m = 0.0;  // t_p of (6.2)

    // Present => model L2 (linear with breakaway saturation, §6.3).
    // Absent  => model L1 (pure linear, §6.2).
    std::optional<double> breakaway_slip_rad;

    // §6.2: L1 is valid for |alpha| <= alpha_lin, default 0.035 rad = 2 deg.
    // Used as the L1 validity half-range; L2 uses valid_alpha_half_range_rad.
    double alpha_lin_rad = 0.035;
    double valid_alpha_half_range_rad = 0.35;  // L2, ~20 deg

    // Operating load ceiling, against which §6.7's two breakpoints are checked at
    // construction.
    double Fz_max_operating_N = 20000.0;

    // §5.2 / E-TIRE-006: a friction block is inadmissible for a model that
    // declares has_mu() == false. Set by the loader when the document carries one.
    bool document_has_friction_block = false;
};

class LinearTire final : public TireModel {
public:
    // §14.1 Rule D3: validation happens once, at construction, on a fully-resolved
    // SI document. Solvers do not re-validate per step; they assert.
    [[nodiscard]] static core::Result<LinearTire> create(LinearTireConfig cfg) {
        const bool has_const = cfg.C_alpha_N_per_rad.has_value();
        const bool has_quad  = cfg.load_sensitivity.has_value();

        if (has_const && has_quad) {
            return core::err<LinearTire>(
                "E-TIRE-005",
                "tire '" + cfg.id + "' supplies both a constant cornering stiffness and a "
                "load_sensitivity quadratic; these are alternatives, not a base value plus "
                "a modifier");
        }
        if (!has_const && !has_quad) {
            return core::err<LinearTire>(
                "E-TIRE-005",
                "tire '" + cfg.id + "' supplies no source of cornering stiffness");
        }

        // E-TIRE-006: L1/L2 have no friction limit, so a friction block is
        // meaningless for them and would otherwise be silently ignored.
        if (cfg.document_has_friction_block) {
            return core::err<LinearTire>(
                "E-TIRE-006",
                "tire '" + cfg.id + "' is a pure-stiffness model (has_mu() == false) but the "
                "document supplies a friction block");
        }

        // E-TIRE-003 validates the INPUT COEFFICIENTS, not the evaluated
        // C_alpha(Fz) — §6.7 is explicit about this, to avoid a spurious failure
        // at the (6.10) sign change, which E-TIRE-004 handles instead.
        if (has_const && cfg.C_alpha_N_per_rad.value() >= 0.0) {
            return core::err<LinearTire>(
                "E-TIRE-003",
                "tire '" + cfg.id + "' has cornering stiffness " +
                    std::to_string(*cfg.C_alpha_N_per_rad) +
                    " N/rad; C_alpha = dFy/dalpha|_0 must be strictly negative");
        }
        if (has_quad) {
            const QuadraticStiffness q = *cfg.load_sensitivity;
            if (q.c0 >= 0.0 || q.c1 <= 0.0) {
                return core::err<LinearTire>(
                    "E-TIRE-003",
                    "tire '" + cfg.id + "' quadratic C_alpha(Fz) requires c0 < 0 and c1 > 0; "
                    "got c0 = " + std::to_string(q.c0) + ", c1 = " + std::to_string(q.c1));
            }
            // §6.7: at Fz = -c0/c1 the expression changes sign, which is
            // unphysical. Reaching it anywhere in the operating range is fatal.
            const double Fz_sign_change = -q.c0 / q.c1;
            if (cfg.Fz_max_operating_N >= Fz_sign_change) {
                return core::err<LinearTire>(
                    "E-TIRE-004",
                    "tire '" + cfg.id + "': quadratic C_alpha(Fz) changes sign at Fz = " +
                        std::to_string(Fz_sign_change) + " N, within the operating range "
                        "(ceiling " + std::to_string(cfg.Fz_max_operating_N) + " N)");
            }
        }

        if (cfg.pneumatic_trail_m < 0.0) {
            return core::err<LinearTire>(
                "E-TIRE-003", "tire '" + cfg.id + "' has a negative pneumatic trail");
        }
        if (cfg.breakaway_slip_rad && *cfg.breakaway_slip_rad <= 0.0) {
            return core::err<LinearTire>(
                "E-TIRE-003", "tire '" + cfg.id + "' has a non-positive breakaway slip angle");
        }

        LinearTire t(std::move(cfg));
        // §6.7: emit W-TIRE-101 if any load in the run exceeds
        // Fz* = -c0/(2 c1), beyond which |C_alpha| FALLS with load. Not fatal --
        // E-TIRE-004 already rejected the sign change -- but the user needs to know
        // the tire is on the far side of its stiffness peak.
        if (const auto peak = t.stiffness_peak_load_N()) {
            if (t.cfg_.Fz_max_operating_N > *peak) {
                t.warnings_.push_back(core::make_diagnostic(
                    "W-TIRE-101",
                    "tire '" + t.cfg_.id + "': the operating load ceiling " +
                        std::to_string(t.cfg_.Fz_max_operating_N) +
                        " N is beyond Fz* = " + std::to_string(*peak) +
                        " N, where the quadratic C_alpha(Fz) peaks; above it the "
                        "cornering-stiffness MAGNITUDE falls with load"));
            }
        }
        return t;
    }

    // §14.1: a W- diagnostic travels with a SUCCESSFUL Result and callers SHALL
    // surface it.
    [[nodiscard]] std::span<const core::Diagnostic> warnings() const { return warnings_; }

    // §6.7: W-TIRE-101 if any load in the run exceeds Fz* = -c0/(2 c1), beyond
    // which |C_alpha| falls with load. Not fatal — reported by the caller.
    [[nodiscard]] std::optional<double> stiffness_peak_load_N() const {
        if (!cfg_.load_sensitivity) return std::nullopt;
        return -cfg_.load_sensitivity->c0 / (2.0 * cfg_.load_sensitivity->c1);
    }

    [[nodiscard]] TireOutput evaluate(const TireState& s) const override {
        TireOutput out;
        out.trail_m = trail_at(s.Fz_N);

        // T4: a lifted wheel produces no force. Guarded BEFORE any use of Fz, not
        // after. Note this matters even for L1, whose (6.1) does not otherwise
        // reference Fz at all — the contract is on the model, not on the formula.
        if (s.Fz_N < kFzFloorN) {
            out.Fy_N = 0.0;
            out.Mz_Nm = 0.0;
            out.C_alpha_N_per_rad = 0.0;
            out.saturated = false;
            return out;  // mu_peak stays disengaged (T5)
        }

        const double Ca = stiffness_at(s.Fz_N);
        const double a  = s.alpha_rad;

        if (cfg_.breakaway_slip_rad) {  // ---- L2, (6.3)
            const double abk = *cfg_.breakaway_slip_rad;
            if (std::abs(a) <= abk) {
                out.Fy_N = Ca * a;
                out.C_alpha_N_per_rad = Ca;
                out.saturated = false;
            } else {
                out.Fy_N = Ca * abk * (a > 0.0 ? 1.0 : -1.0);
                out.C_alpha_N_per_rad = 0.0;
                out.saturated = true;
            }
        } else {                        // ---- L1, (6.1)
            out.Fy_N = Ca * a;
            out.C_alpha_N_per_rad = Ca;
            out.saturated = false;
        }

        // T2 is exact at alpha = 0 because Ca * 0.0 == 0.0 in IEEE 754 for any
        // finite Ca, and (6.2) then gives Mz == -0.0 * t_p == 0.0.
        out.Mz_Nm = -out.trail_m * out.Fy_N;
        if (out.Fy_N == 0.0) out.Mz_Nm = 0.0;  // normalise -0.0 to +0.0

        return out;
    }

    [[nodiscard]] double dFy_dalpha(const TireState& s) const override {
        if (s.Fz_N < kFzFloorN) return 0.0;
        const double Ca = stiffness_at(s.Fz_N);
        if (!cfg_.breakaway_slip_rad) return Ca;
        // §6.3: C_alpha below breakaway, 0 above. At EXACTLY |alpha| = alpha_bk the
        // one-sided derivative FROM BELOW is returned — a documented choice, and
        // the reason slope_break_points() exposes the kink locations.
        return (std::abs(s.alpha_rad) <= *cfg_.breakaway_slip_rad) ? Ca : 0.0;
    }

    [[nodiscard]] double dFy_dFz(const TireState& s) const override {
        if (!cfg_.load_sensitivity) return 0.0;  // constant stiffness
        if (s.Fz_N < kFzFloorN) return 0.0;
        const QuadraticStiffness q = *cfg_.load_sensitivity;
        const double dCa_dFz = q.c0 + 2.0 * q.c1 * s.Fz_N;
        if (cfg_.breakaway_slip_rad) {
            const double abk = *cfg_.breakaway_slip_rad;
            if (std::abs(s.alpha_rad) > abk) {
                return dCa_dFz * abk * (s.alpha_rad > 0.0 ? 1.0 : -1.0);
            }
        }
        return dCa_dFz * s.alpha_rad;
    }

    [[nodiscard]] Interval valid_alpha_rad() const override {
        const double h = cfg_.breakaway_slip_rad ? cfg_.valid_alpha_half_range_rad
                                                 : cfg_.alpha_lin_rad;
        return {-h, h};
    }
    [[nodiscard]] Interval valid_Fz_N() const override { return {0.0, cfg_.Fz_max_operating_N}; }
    [[nodiscard]] std::string_view id() const override { return cfg_.id; }

    [[nodiscard]] bool has_mu() const override { return false; }  // T5: pure stiffness
    [[nodiscard]] bool is_C1() const override { return !cfg_.breakaway_slip_rad.has_value(); }

    [[nodiscard]] std::span<const double> slope_break_points() const override {
        return breaks_;  // empty for L1; {-alpha_bk, +alpha_bk} for L2, sorted
    }

private:
    explicit LinearTire(LinearTireConfig cfg) : cfg_(std::move(cfg)) {
        if (cfg_.breakaway_slip_rad) {
            breaks_ = {-*cfg_.breakaway_slip_rad, *cfg_.breakaway_slip_rad};
        }
    }

    [[nodiscard]] double stiffness_at(double Fz_N) const {
        if (cfg_.C_alpha_N_per_rad) return *cfg_.C_alpha_N_per_rad;
        const QuadraticStiffness q = *cfg_.load_sensitivity;
        return q.c0 * Fz_N + q.c1 * Fz_N * Fz_N;
    }

    // T8 / (6.2a). At Fz = 0 the trail is a geometric property of a tire carrying
    // no load and is undefined; returning t_p(0+) rather than zero avoids making
    // the physical claim that the force acts at the contact centre.
    [[nodiscard]] double trail_at(double /*Fz_N*/) const { return cfg_.pneumatic_trail_m; }

    LinearTireConfig            cfg_;
    std::vector<double>         breaks_;
    std::vector<core::Diagnostic> warnings_;
};

}  // namespace rcvd::tire

#endif  // RCVD_TIRE_LINEAR_HPP
