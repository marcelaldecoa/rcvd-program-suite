// rcvd/numerics/roots.hpp — Brent's method and the differentiation step, §13.6-13.7.
#ifndef RCVD_NUMERICS_ROOTS_HPP
#define RCVD_NUMERICS_ROOTS_HPP

#include <cmath>
#include <limits>
#include <string>

#include "rcvd/core/result.hpp"

namespace rcvd::numerics {

struct RootConfig {
    double xtol      = 1e-12;  // §13.7 absolute, on the independent variable
    double ftol      = 1e-10;  // §13.7 on the residual; whichever is reached first
    int    max_iter  = 100;
};

struct RootResult {
    double root;
    double residual;
    int    iterations;
};

// Brent (bisection + secant + inverse quadratic) on a VERIFIED bracket.
// §13.7: if no sign change exists on [a,b] the solver reports E-NUM-300 with the
// interval and endpoint values, rather than returning an endpoint.
template <class F>
[[nodiscard]] core::Result<RootResult> brent(F&& f, double a, double b,
                                             RootConfig cfg = {}) {
    double fa = f(a);
    double fb = f(b);

    if (fa == 0.0) return RootResult{a, 0.0, 0};
    if (fb == 0.0) return RootResult{b, 0.0, 0};

    if (fa * fb > 0.0) {
        return core::err<RootResult>(
            "E-NUM-300",
            "no sign change on the supplied bracket: f(" + std::to_string(a) + ") = " +
                std::to_string(fa) + ", f(" + std::to_string(b) + ") = " + std::to_string(fb));
    }

    double c = a, fc = fa, d = b - a, e = d;

    for (int iter = 1; iter <= cfg.max_iter; ++iter) {
        if (fb * fc > 0.0) {  // keep c as the counterpoint to b
            c = a; fc = fa; d = b - a; e = d;
        }
        if (std::abs(fc) < std::abs(fb)) {  // b is always the best estimate
            a = b; b = c; c = a;
            fa = fb; fb = fc; fc = fa;
        }

        const double tol = 2.0 * std::numeric_limits<double>::epsilon() * std::abs(b) + 0.5 * cfg.xtol;
        const double m   = 0.5 * (c - b);

        if (std::abs(m) <= tol || std::abs(fb) <= cfg.ftol || fb == 0.0) {
            return RootResult{b, fb, iter};
        }

        if (std::abs(e) < tol || std::abs(fa) <= std::abs(fb)) {
            d = m; e = m;                       // bisect
        } else {
            double p, q;
            const double s = fb / fa;
            if (a == c) {                       // secant
                p = 2.0 * m * s;
                q = 1.0 - s;
            } else {                            // inverse quadratic
                const double qq = fa / fc;
                const double r  = fb / fc;
                p = s * (2.0 * m * qq * (qq - r) - (b - a) * (r - 1.0));
                q = (qq - 1.0) * (r - 1.0) * (s - 1.0);
            }
            if (p > 0.0) q = -q; else p = -p;

            if (2.0 * p < std::min(3.0 * m * q - std::abs(tol * q), std::abs(e * q))) {
                e = d; d = p / q;               // accept the interpolation
            } else {
                d = m; e = m;                   // fall back to bisection
            }
        }

        a = b; fa = fb;
        b += (std::abs(d) > tol) ? d : (m > 0.0 ? tol : -tol);
        fb = f(b);
    }

    return core::err<RootResult>("E-NUM-300",
                                 "Brent did not converge in " + std::to_string(cfg.max_iter) +
                                     " iterations; last residual " + std::to_string(fb));
}

// §13.6 (13.2): the truncation/round-off optimum step for a central difference.
// eps^(1/3) ~= 6.06e-6.
[[nodiscard]] inline double diff_step(double x, double x_typ = 1.0) noexcept {
    constexpr double kCbrtEps = 6.055454452393343e-6;  // cbrt(2.220446049250313e-16)
    return std::max(std::abs(x), std::abs(x_typ)) * kCbrtEps;
}

// Central difference with one Richardson extrapolation, §13.6. Fourth-order
// accurate; this is the reference the tire contract T6 checks analytic slopes
// against to 1e-6 relative.
template <class F>
[[nodiscard]] double central_difference(F&& f, double x, double x_typ = 1.0) {
    const double h  = diff_step(x, x_typ);
    const double d1 = (f(x + h) - f(x - h)) / (2.0 * h);
    const double h2 = 2.0 * h;
    const double d2 = (f(x + h2) - f(x - h2)) / (2.0 * h2);
    return (4.0 * d1 - d2) / 3.0;  // Richardson: cancels the h^2 term
}

}  // namespace rcvd::numerics

#endif  // RCVD_NUMERICS_ROOTS_HPP
