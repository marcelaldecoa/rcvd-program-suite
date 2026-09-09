// rcvd/numerics/pchip.hpp — Fritsch-Carlson monotone cubic Hermite, spec §13.5.
//
// Normative everywhere table data is interpolated (§6.4 tire data, §5.6 driver
// tables, aero maps). Chosen over bilinear/spline because it is C1 — so the
// numerical Jacobians of §9.7 and the MMM contouring of §10.5 behave — and
// shape-preserving, so it introduces no overshoot near a tire's force peak.
#ifndef RCVD_NUMERICS_PCHIP_HPP
#define RCVD_NUMERICS_PCHIP_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <span>
#include <vector>

namespace rcvd::numerics {

class Pchip {
public:
    Pchip() = default;

    // x must be strictly increasing and the same length as y, with >= 2 nodes.
    // Validation is the caller's (§14.1 Rule D3: validate once, at construction).
    Pchip(std::span<const double> x, std::span<const double> y)
        : x_(x.begin(), x.end()), y_(y.begin(), y.end()), d_(x.size(), 0.0) {
        compute_slopes();
    }

    [[nodiscard]] bool empty() const noexcept { return x_.size() < 2; }
    [[nodiscard]] double x_min() const noexcept { return x_.front(); }
    [[nodiscard]] double x_max() const noexcept { return x_.back(); }

    // Constant extrapolation outside the range (§6.4). The caller is responsible
    // for raising the extrapolation event of Rule D2; this returns the held value.
    [[nodiscard]] double operator()(double x) const noexcept {
        if (x <= x_.front()) return y_.front();
        if (x >= x_.back()) return y_.back();
        const std::size_t i = interval(x);
        const double h = x_[i + 1] - x_[i];
        const double t = (x - x_[i]) / h;
        return hermite(y_[i], y_[i + 1], d_[i], d_[i + 1], h, t);
    }

    // Closed-form derivative. Zero in the held-value regions, which is the
    // correct one-sided slope for a saturated tire.
    [[nodiscard]] double derivative(double x) const noexcept {
        if (empty() || x <= x_.front() || x >= x_.back()) return 0.0;
        const std::size_t i = interval(x);
        const double h = x_[i + 1] - x_[i];
        const double t = (x - x_[i]) / h;
        const double t2 = t * t;
        // d/dx of the Hermite basis, divided through by h. The first term is
        // (6t^2-6t)(y0-y1)/h; writing it as (y1-y0) flips the interpolant's slope
        // wherever the data is not flat.
        return (6.0 * t - 6.0 * t2) * (y_[i + 1] - y_[i]) / h
             + (3.0 * t2 - 4.0 * t + 1.0) * d_[i]
             + (3.0 * t2 - 2.0 * t) * d_[i + 1];
    }

private:
    [[nodiscard]] std::size_t interval(double x) const noexcept {
        const auto it = std::upper_bound(x_.begin(), x_.end(), x);
        return static_cast<std::size_t>(it - x_.begin()) - 1;
    }

    static double hermite(double y0, double y1, double d0, double d1, double h, double t) noexcept {
        const double t2 = t * t;
        const double t3 = t2 * t;
        const double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
        const double h10 = t3 - 2.0 * t2 + t;
        const double h01 = -2.0 * t3 + 3.0 * t2;
        const double h11 = t3 - t2;
        return h00 * y0 + h10 * h * d0 + h01 * y1 + h11 * h * d1;
    }

    // Fritsch-Carlson (1980). Slopes are limited so that every interval stays
    // monotone; this is what kills the overshoot a natural cubic spline shows at
    // a tire's force peak.
    void compute_slopes() {
        const std::size_t n = x_.size();
        if (n < 2) return;
        std::vector<double> h(n - 1), delta(n - 1);
        for (std::size_t i = 0; i + 1 < n; ++i) {
            h[i] = x_[i + 1] - x_[i];
            delta[i] = (y_[i + 1] - y_[i]) / h[i];
        }
        if (n == 2) {
            d_[0] = d_[1] = delta[0];
            return;
        }
        // Interior: weighted harmonic mean, zeroed at a sign change or a flat.
        for (std::size_t i = 1; i + 1 < n; ++i) {
            if (delta[i - 1] * delta[i] <= 0.0) {
                d_[i] = 0.0;
            } else {
                const double w1 = 2.0 * h[i] + h[i - 1];
                const double w2 = h[i] + 2.0 * h[i - 1];
                d_[i] = (w1 + w2) / (w1 / delta[i - 1] + w2 / delta[i]);
            }
        }
        d_[0]     = endpoint_slope(h[0], h[1], delta[0], delta[1]);
        d_[n - 1] = endpoint_slope(h[n - 2], h[n - 3], delta[n - 2], delta[n - 3]);
    }

    // Three-point one-sided formula, clamped to preserve monotonicity.
    static double endpoint_slope(double h0, double h1, double del0, double del1) noexcept {
        double d = ((2.0 * h0 + h1) * del0 - h0 * del1) / (h0 + h1);
        if (d * del0 <= 0.0) {
            d = 0.0;
        } else if (del0 * del1 <= 0.0 && std::abs(d) > std::abs(3.0 * del0)) {
            d = 3.0 * del0;
        }
        return d;
    }

    std::vector<double> x_, y_, d_;
};

}  // namespace rcvd::numerics

#endif  // RCVD_NUMERICS_PCHIP_HPP
