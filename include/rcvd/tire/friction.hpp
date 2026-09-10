// rcvd/tire/friction.hpp — §6.7 load sensitivity and §6.8 the friction ellipse.
#ifndef RCVD_TIRE_FRICTION_HPP
#define RCVD_TIRE_FRICTION_HPP

#include <algorithm>
#include <cmath>
#include <optional>

#include "rcvd/core/diagnostic.hpp"

namespace rcvd::tire {

// (6.9) mu(Fz) = mu0 - mu1*Fz, clamped to [mu_min, mu0].
//
// Load sensitivity is what makes lateral load transfer reduce an axle's total
// lateral force capability, and therefore what makes roll-stiffness distribution
// a handling tuning parameter at all (§6.7).
struct FrictionModel {
    double mu0 = 1.0;
    double mu1_per_N = 0.0;   // dimensional: 1/force
    double mu_min = 0.1;      // §6.7 default

    [[nodiscard]] double mu(double Fz_N) const {
        return std::clamp(mu0 - mu1_per_N * Fz_N, mu_min, mu0);
    }
    // Rule D1: no silent clamping. The caller raises EV-CLAMP-MU the first time
    // this reports true.
    [[nodiscard]] bool clamped_at(double Fz_N) const {
        const double raw = mu0 - mu1_per_N * Fz_N;
        return raw < mu_min || raw > mu0;
    }
    [[nodiscard]] double dmu_dFz(double Fz_N) const {
        return clamped_at(Fz_N) ? 0.0 : -mu1_per_N;
    }

    // Rule D1's other half for this model. A tire held at mu_min is not obeying
    // (6.9) any more, and the run needs to say so rather than quietly flatten the
    // load sensitivity that makes roll-stiffness distribution a tuning parameter.
    [[nodiscard]] std::optional<core::Diagnostic> clamp_event(double Fz_N,
                                                              double t_s = 0.0) const {
        if (!clamped_at(Fz_N)) return std::nullopt;
        const double raw = mu0 - mu1_per_N * Fz_N;
        return core::make_diagnostic(
            "EV-CLAMP-MU",
            "friction coefficient clamped: (6.9) gives mu = " + std::to_string(raw) +
                " at Fz = " + std::to_string(Fz_N) + " N, outside [" +
                std::to_string(mu_min) + ", " + std::to_string(mu0) + "]",
            t_s);
    }
};

// §6.8 (6.11)-(6.12). Rev 1.0 models combined slip only as a CAPACITY LIMIT, not
// as a full combined-slip force model.
struct FrictionEllipse {
    double mu_x = 1.0, mu_y = 1.0;

    // (6.12) F_x,max = mu_x Fz sqrt(max(0, 1 - (Fy/(mu_y Fz))^2)).
    //
    // The max(0, .) guard is MANDATORY: rounding can drive the radicand slightly
    // negative when the tire is at the lateral limit, and an unguarded sqrt
    // produces a NaN that propagates silently through an entire time history —
    // which §13.1 makes a fatal defect.
    [[nodiscard]] double longitudinal_capacity(double Fy_N, double Fz_N,
                                               bool* clamped = nullptr) const {
        if (Fz_N <= 0.0) {
            if (clamped) *clamped = false;
            return 0.0;
        }
        const double ratio = Fy_N / (mu_y * Fz_N);
        const double radicand = 1.0 - ratio * ratio;
        if (clamped) *clamped = radicand < 0.0;   // EV-CLAMP-ELLIPSE
        return mu_x * Fz_N * std::sqrt(std::max(0.0, radicand));
    }

    // Rule D1: every clamp raises an event the first time it fires. EV-CLAMP-ELLIPSE
    // exists because the max(0, .) guard above is a real physical clamp, not a
    // rounding convenience: without the event, a tire silently past its friction
    // limit looks identical to one exactly at it.
    [[nodiscard]] std::optional<core::Diagnostic> clamp_event(double Fy_N, double Fz_N,
                                                              double t_s = 0.0) const {
        bool clamped = false;
        (void)longitudinal_capacity(Fy_N, Fz_N, &clamped);
        if (!clamped) return std::nullopt;
        return core::make_diagnostic(
            "EV-CLAMP-ELLIPSE",
            "friction-ellipse radicand clamped to zero: |Fy| = " + std::to_string(Fy_N) +
                " N already exceeds mu_y*Fz = " + std::to_string(mu_y * Fz_N) +
                " N, so no longitudinal capacity remains",
            t_s);
    }

    // (6.11) as a predicate: is this force pair inside the ellipse?
    [[nodiscard]] bool within(double Fx_N, double Fy_N, double Fz_N) const {
        if (Fz_N <= 0.0) return Fx_N == 0.0 && Fy_N == 0.0;
        const double x = Fx_N / (mu_x * Fz_N);
        const double y = Fy_N / (mu_y * Fz_N);
        return x * x + y * y <= 1.0 + 1e-9;
    }
};

}  // namespace rcvd::tire

#endif  // RCVD_TIRE_FRICTION_HPP
