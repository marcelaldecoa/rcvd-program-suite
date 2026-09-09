// rcvd/tire/conformance.hpp — the shared T1-T8 contract check of spec §6.1.
//
// "The following invariants are normative and SHALL be enforced by a shared
// conformance test applied to every model." This is that test. It takes only the
// abstract TireModel, so a new model earns its conformance by construction rather
// than by someone remembering to write its checks.
#ifndef RCVD_TIRE_CONFORMANCE_HPP
#define RCVD_TIRE_CONFORMANCE_HPP

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "rcvd/numerics/roots.hpp"
#include "rcvd/tire/tire_model.hpp"

namespace rcvd::tire {

struct ConformanceReport {
    struct Failure {
        std::string check;   // "T3"
        std::string detail;
    };
    std::vector<Failure> failures;
    [[nodiscard]] bool ok() const noexcept { return failures.empty(); }
};

struct ConformanceConfig {
    int    alpha_samples = 41;
    int    T6_samples    = 20;   // §6.1 T6 says 20 sample points
    double rel_tol       = 1e-12;
    double slope_tol     = 1e-6;  // T6
    std::vector<double> loads_N{1000.0, 3000.0, 6000.0};
};

namespace detail {

inline bool near_any(double x, std::span<const double> pts, double window) {
    return std::any_of(pts.begin(), pts.end(),
                       [&](double p) { return std::abs(x - p) < window; });
}

inline void add(ConformanceReport& r, std::string check, std::string detail) {
    r.failures.push_back({std::move(check), std::move(detail)});
}

inline std::string at(double alpha, double Fz) {
    return " at alpha = " + std::to_string(alpha) + " rad, Fz = " + std::to_string(Fz) + " N";
}

}  // namespace detail

[[nodiscard]] inline ConformanceReport check_contract(const TireModel& m,
                                                      ConformanceConfig cfg = {}) {
    using detail::add;
    using detail::at;
    ConformanceReport r;

    const Interval A = m.valid_alpha_rad();
    const Interval Z = m.valid_Fz_N();

    // Keep the sampled loads inside the model's declared validity.
    std::vector<double> loads;
    for (double Fz : cfg.loads_N) {
        if (Z.contains(Fz)) loads.push_back(Fz);
    }
    if (loads.empty()) loads.push_back(std::clamp(1000.0, Z.lo, Z.hi));

    const auto alpha_at = [&](int i) {
        const double t = static_cast<double>(i) / (cfg.alpha_samples - 1);
        return A.lo + t * (A.hi - A.lo);
    };

    for (double Fz : loads) {
        // ---- T2: exact zero crossing -------------------------------------
        {
            const TireOutput o = m.evaluate({0.0, Fz});
            if (o.Fy_N != 0.0) add(r, "T2", "Fy(0, Fz) = " + std::to_string(o.Fy_N) +
                                                 ", must be exactly 0" + at(0.0, Fz));
            if (o.Mz_Nm != 0.0) add(r, "T2", "Mz(0, Fz) = " + std::to_string(o.Mz_Nm) +
                                                  ", must be exactly 0" + at(0.0, Fz));
        }

        for (int i = 0; i < cfg.alpha_samples; ++i) {
            const double a = alpha_at(i);
            const TireOutput o = m.evaluate({a, Fz});

            // ---- T1: odd symmetry ----------------------------------------
            const TireOutput on = m.evaluate({-a, Fz});
            const double sym_den = std::max(1.0, std::abs(o.Fy_N));
            if (std::abs(on.Fy_N + o.Fy_N) > cfg.rel_tol * sym_den) {
                add(r, "T1", "Fy(-a) != -Fy(a): " + std::to_string(on.Fy_N) + " vs " +
                                 std::to_string(-o.Fy_N) + at(a, Fz));
            }

            // ---- T3: correct sign ----------------------------------------
            if (a * o.Fy_N > 0.0) {
                add(r, "T3", "alpha*Fy = " + std::to_string(a * o.Fy_N) + " > 0" + at(a, Fz));
            }

            // ---- T5: friction bound --------------------------------------
            if (m.has_mu()) {
                if (!o.mu_peak.has_value()) {
                    add(r, "T5", "has_mu() is true but mu_peak is disengaged" + at(a, Fz));
                } else if (std::abs(o.Fy_N) > *o.mu_peak * Fz * (1.0 + 1e-9)) {
                    add(r, "T5", "|Fy| = " + std::to_string(std::abs(o.Fy_N)) +
                                     " exceeds mu*Fz = " + std::to_string(*o.mu_peak * Fz) +
                                     at(a, Fz));
                }
            } else if (o.mu_peak.has_value()) {
                // A pure-stiffness model must leave it DISENGAGED, not set to a
                // sentinel: §13.1 makes NaN fatal, so NaN cannot mean "absent".
                add(r, "T5", "has_mu() is false but mu_peak is engaged" + at(a, Fz));
            }

            // ---- T8: trail is finite, never 0/0 --------------------------
            if (!std::isfinite(o.trail_m)) {
                add(r, "T8", "trail_m is not finite (" + std::to_string(o.trail_m) + ")" +
                                 at(a, Fz));
            }
        }

        // ---- T6: analytic slope vs Richardson central difference ----------
        // Excluding declared slope discontinuities AND interpolation knots: a
        // stencil straddling either loses its order, and at a knot that is a
        // property of (13.2), not of the model's derivative.
        {
            const auto breaks = m.slope_break_points();
            const auto knots  = m.interpolation_knots();
            for (int i = 0; i < cfg.T6_samples; ++i) {
                const double t = (i + 0.5) / cfg.T6_samples;
                const double a = A.lo + t * (A.hi - A.lo);
                const double h = numerics::diff_step(a, A.hi - A.lo);
                if (detail::near_any(a, breaks, 10.0 * h)) continue;
                if (detail::near_any(a, knots, 10.0 * h)) continue;
                // Keep the whole +/-2h stencil inside the valid range.
                if (a - 2.0 * h < A.lo || a + 2.0 * h > A.hi) continue;

                const double analytic = m.dFy_dalpha({a, Fz});
                const double fd = numerics::central_difference(
                    [&](double x) { return m.evaluate({x, Fz}).Fy_N; }, a, A.hi - A.lo);
                const double den = std::max(1.0, std::abs(fd));
                if (std::abs(analytic - fd) > cfg.slope_tol * den) {
                    add(r, "T6", "analytic dFy/dalpha = " + std::to_string(analytic) +
                                     " vs central difference " + std::to_string(fd) + at(a, Fz));
                }
            }
        }

        // ---- T7: continuity ----------------------------------------------
        // Every model SHALL be continuous (C0 minimum). Checked across the declared
        // slope breaks in particular, since that is where a C0 model is at risk.
        //
        // The test is a SCALE-FREE RATIO, not an absolute tolerance on
        // |Fy(p+e) - Fy(p-e)|. That difference is dominated by the legitimate
        // slope: with C_alpha ~ -1.6e5 N/rad the smooth change over e = 1e-9 rad
        // is already 3e-4 N, so any fixed tolerance either rejects continuous
        // models or is too loose to catch a real jump. Instead: for a continuous
        // function the gap shrinks LINEARLY with e, so gap(e) ~ gap(10e)/10; at a
        // genuine discontinuity gap(e) ~ gap(10e) ~ the jump itself.
        {
            // Force scale for the floor, so an identically-zero region passes.
            double F_scale = 0.0;
            for (int i = 0; i < cfg.alpha_samples; ++i) {
                F_scale = std::max(F_scale, std::abs(m.evaluate({alpha_at(i), Fz}).Fy_N));
            }
            const double floor = 1e-12 * std::max(1.0, F_scale);

            std::vector<double> probes;
            for (int i = 1; i < cfg.alpha_samples; ++i) probes.push_back(alpha_at(i));
            for (double b : m.slope_break_points()) probes.push_back(b);

            const auto gap_ratio_ok = [&](double p, double e, auto&& fn, double& g1, double& g10) {
                g1  = std::abs(fn(p + e) - fn(p - e));
                g10 = std::abs(fn(p + 10.0 * e) - fn(p - 10.0 * e));
                return g1 <= 0.6 * g10 + floor;
            };

            for (double p : probes) {
                const double eps = 1e-9 * std::max(1.0, std::abs(p));
                if (!A.contains(p - 10.0 * eps) || !A.contains(p + 10.0 * eps)) continue;
                double g1 = 0.0, g10 = 0.0;
                const auto Fy = [&](double x) { return m.evaluate({x, Fz}).Fy_N; };
                if (!gap_ratio_ok(p, eps, Fy, g1, g10)) {
                    add(r, "T7", "Fy is discontinuous: gap " + std::to_string(g1) +
                                     " at e, " + std::to_string(g10) + " at 10e (a continuous "
                                     "Fy would give ~1/10 of the latter)" + at(p, Fz));
                }
            }

            // A model claiming C1 must have a continuous SLOPE by the same test.
            if (m.is_C1()) {
                for (double p : probes) {
                    const double eps = 1e-7 * std::max(1.0, std::abs(p));
                    if (!A.contains(p - 10.0 * eps) || !A.contains(p + 10.0 * eps)) continue;
                    double g1 = 0.0, g10 = 0.0;
                    const auto slope = [&](double x) { return m.dFy_dalpha({x, Fz}); };
                    if (!gap_ratio_ok(p, eps, slope, g1, g10)) {
                        add(r, "T7", "is_C1() is true but dFy/dalpha is discontinuous: gap " +
                                         std::to_string(g1) + " at e, " + std::to_string(g10) +
                                         " at 10e" + at(p, Fz));
                    }
                }
                if (!m.slope_break_points().empty()) {
                    add(r, "T7", "is_C1() is true but slope_break_points() is non-empty");
                }
            }
        }
    }

    // ---- T4: zero load, checked once ------------------------------------
    for (int i = 0; i < cfg.alpha_samples; ++i) {
        const double a = alpha_at(i);
        const TireOutput o = m.evaluate({a, 0.0});
        if (o.Fy_N != 0.0) {
            add(r, "T4", "Fz = 0 must give Fy = 0 exactly, got " + std::to_string(o.Fy_N) +
                             at(a, 0.0));
        }
        if (o.Mz_Nm != 0.0) {
            add(r, "T4", "Fz = 0 must give Mz = 0 exactly, got " + std::to_string(o.Mz_Nm) +
                             at(a, 0.0));
        }
        if (!std::isfinite(o.trail_m)) {
            add(r, "T8", "trail_m is not finite at zero load" + at(a, 0.0));
        }
    }

    // §6.1 T7: slope_break_points() is documented as sorted, and the MMM sweep and
    // the integrator rely on that to subdivide correctly.
    {
        const auto b = m.slope_break_points();
        if (!std::is_sorted(b.begin(), b.end())) {
            add(r, "T7", "slope_break_points() is not sorted");
        }
    }

    return r;
}

}  // namespace rcvd::tire

#endif  // RCVD_TIRE_CONFORMANCE_HPP
