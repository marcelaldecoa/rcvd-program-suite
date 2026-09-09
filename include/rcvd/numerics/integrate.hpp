// rcvd/numerics/integrate.hpp — RK4 fixed and DOPRI5(4) with dense output, §13.2.
//
// Rule A2 (§4.4): no unordered iteration affecting accumulation order, no parallel
// reduction over floating point, no -ffast-math. Stage accumulation below is in a
// fixed order, deliberately.
#ifndef RCVD_NUMERICS_INTEGRATE_HPP
#define RCVD_NUMERICS_INTEGRATE_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <span>
#include <vector>

namespace rcvd::numerics {

// xdot = f(t, x) -> writes into the final span.
using Derivative = std::function<void(double, std::span<const double>, std::span<double>)>;

struct StepStats {
    long   accepted   = 0;
    long   rejected   = 0;
    double h_min      = 0.0;
    double h_max      = 0.0;
    double err_max    = 0.0;
};

// ---- classical RK4, fixed step (§13.2 default, matches source behaviour) -----
inline void rk4_step(const Derivative& f, double t, std::span<const double> x, double h,
                     std::span<double> x_out, std::vector<double>& work) {
    const std::size_t n = x.size();
    work.assign(5 * n, 0.0);
    std::span<double> k1{work.data() + 0 * n, n};
    std::span<double> k2{work.data() + 1 * n, n};
    std::span<double> k3{work.data() + 2 * n, n};
    std::span<double> k4{work.data() + 3 * n, n};
    std::span<double> tmp{work.data() + 4 * n, n};

    f(t, x, k1);
    for (std::size_t i = 0; i < n; ++i) tmp[i] = x[i] + 0.5 * h * k1[i];
    f(t + 0.5 * h, tmp, k2);
    for (std::size_t i = 0; i < n; ++i) tmp[i] = x[i] + 0.5 * h * k2[i];
    f(t + 0.5 * h, tmp, k3);
    for (std::size_t i = 0; i < n; ++i) tmp[i] = x[i] + h * k3[i];
    f(t + h, tmp, k4);

    for (std::size_t i = 0; i < n; ++i) {
        x_out[i] = x[i] + (h / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }
}

// ---- Dormand-Prince 5(4) ----------------------------------------------------
namespace dopri {
// Nodes
inline constexpr double c2 = 1.0 / 5.0, c3 = 3.0 / 10.0, c4 = 4.0 / 5.0,
                        c5 = 8.0 / 9.0, c6 = 1.0, c7 = 1.0;
// Runge-Kutta matrix
inline constexpr double a21 = 1.0 / 5.0;
inline constexpr double a31 = 3.0 / 40.0,       a32 = 9.0 / 40.0;
inline constexpr double a41 = 44.0 / 45.0,      a42 = -56.0 / 15.0,     a43 = 32.0 / 9.0;
inline constexpr double a51 = 19372.0 / 6561.0, a52 = -25360.0 / 2187.0,
                        a53 = 64448.0 / 6561.0, a54 = -212.0 / 729.0;
inline constexpr double a61 = 9017.0 / 3168.0,  a62 = -355.0 / 33.0,
                        a63 = 46732.0 / 5247.0, a64 = 49.0 / 176.0, a65 = -5103.0 / 18656.0;
// 5th-order solution weights (== a7j, the FSAL row)
inline constexpr double b1 = 35.0 / 384.0, b2 = 0.0, b3 = 500.0 / 1113.0,
                        b4 = 125.0 / 192.0, b5 = -2187.0 / 6784.0, b6 = 11.0 / 84.0;
// 4th-order embedded weights
inline constexpr double e1 = 5179.0 / 57600.0, e2 = 0.0, e3 = 7571.0 / 16695.0,
                        e4 = 393.0 / 640.0, e5 = -92097.0 / 339200.0,
                        e6 = 187.0 / 2100.0, e7 = 1.0 / 40.0;
}  // namespace dopri

// One accepted step's worth of state, retained so that §11.4 can locate a lap or
// a wall contact by dense output plus Brent refinement rather than by taking the
// first step at which the condition holds.
struct DenseSegment {
    double t0{}, h{};
    std::vector<double> y0, y1, k1, k3, k4, k5, k6, k7;

    // Standard Dormand-Prince 4th-order interpolant, valid for theta in [0,1].
    // Horner form of Hairer, Norsett & Wanner Vol. I (the dopri5.f "rcont" nesting)
    // rather than an expanded per-stage polynomial: it satisfies eval(0) == y0 and
    // eval(1) == y1 by construction, which an expanded form only does if every
    // coefficient is transcribed correctly.
    void eval(double theta, std::span<double> out) const {
        const std::size_t n = y0.size();
        const double th = theta;
        const double th1 = 1.0 - th;
        for (std::size_t i = 0; i < n; ++i) {
            const double c1 = y0[i];
            const double c2 = y1[i] - y0[i];
            const double c3 = h * k1[i] - c2;
            const double c4 = c2 - h * k7[i] - c3;
            const double c5 = h * (kD1 * k1[i] + kD3 * k3[i] + kD4 * k4[i] +
                                   kD5 * k5[i] + kD6 * k6[i] + kD7 * k7[i]);
            out[i] = c1 + th * (c2 + th1 * (c3 + th * (c4 + th1 * c5)));
        }
    }

    // Dense-output stage weights.
    static constexpr double kD1 = -12715105075.0 / 11282082432.0;
    static constexpr double kD3 =  87487479700.0 / 32700410799.0;
    static constexpr double kD4 = -10690763975.0 / 1880347072.0;
    static constexpr double kD5 = 701980252875.0 / 199316789632.0;
    static constexpr double kD6 =  -1453857185.0 / 822651844.0;
    static constexpr double kD7 =     69997945.0 / 29380423.0;
};

struct AdaptiveConfig {
    double rtol      = 1e-8;
    double h_max     = 0.01;
    double h_min     = 1e-10;
    double safety    = 0.9;
    double min_scale = 0.2;
    double max_scale = 5.0;
    // §13.2: per-state atol, applied BEFORE the norm. An unscaled norm silently
    // lets the position states dominate the error control. Sized to the state
    // vector by the caller; if shorter, the last entry is reused.
    std::vector<double> atol{1e-8, 1e-9, 1e-9, 1e-6, 1e-6};

    [[nodiscard]] double atol_for(std::size_t i) const noexcept {
        if (atol.empty()) return 1e-9;
        return i < atol.size() ? atol[i] : atol.back();
    }
};

// One adaptive step. Returns true if accepted; h is updated in place per (13.1).
inline bool dopri54_step(const Derivative& f, double t, std::span<const double> y,
                         double& h, const AdaptiveConfig& cfg, DenseSegment& seg,
                         StepStats& stats) {
    using namespace dopri;
    const std::size_t n = y.size();

    auto& k1 = seg.k1; auto& k3 = seg.k3; auto& k4 = seg.k4;
    auto& k5 = seg.k5; auto& k6 = seg.k6; auto& k7 = seg.k7;
    std::vector<double> k2(n), tmp(n), y5(n);
    k1.assign(n, 0.0); k3.assign(n, 0.0); k4.assign(n, 0.0);
    k5.assign(n, 0.0); k6.assign(n, 0.0); k7.assign(n, 0.0);

    f(t, y, k1);
    for (std::size_t i = 0; i < n; ++i) tmp[i] = y[i] + h * a21 * k1[i];
    f(t + c2 * h, tmp, k2);
    for (std::size_t i = 0; i < n; ++i) tmp[i] = y[i] + h * (a31 * k1[i] + a32 * k2[i]);
    f(t + c3 * h, tmp, k3);
    for (std::size_t i = 0; i < n; ++i) tmp[i] = y[i] + h * (a41 * k1[i] + a42 * k2[i] + a43 * k3[i]);
    f(t + c4 * h, tmp, k4);
    for (std::size_t i = 0; i < n; ++i)
        tmp[i] = y[i] + h * (a51 * k1[i] + a52 * k2[i] + a53 * k3[i] + a54 * k4[i]);
    f(t + c5 * h, tmp, k5);
    for (std::size_t i = 0; i < n; ++i)
        tmp[i] = y[i] + h * (a61 * k1[i] + a62 * k2[i] + a63 * k3[i] + a64 * k4[i] + a65 * k5[i]);
    f(t + c6 * h, tmp, k6);

    for (std::size_t i = 0; i < n; ++i) {
        y5[i] = y[i] + h * (b1 * k1[i] + b3 * k3[i] + b4 * k4[i] + b5 * k5[i] + b6 * k6[i]);
    }
    f(t + h, y5, k7);  // FSAL

    // (13.1) scaled infinity norm.
    double err = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const double y4 = y[i] + h * (e1 * k1[i] + e3 * k3[i] + e4 * k4[i] +
                                      e5 * k5[i] + e6 * k6[i] + e7 * k7[i]);
        const double sc = cfg.atol_for(i) + cfg.rtol * std::max(std::abs(y[i]), std::abs(y5[i]));
        err = std::max(err, std::abs(y5[i] - y4) / sc);
    }
    stats.err_max = std::max(stats.err_max, err);

    const bool accept = (err <= 1.0);
    if (accept) {
        seg.t0 = t;
        seg.h  = h;
        seg.y0.assign(y.begin(), y.end());
        seg.y1 = y5;
        ++stats.accepted;
        stats.h_min = (stats.h_min == 0.0) ? h : std::min(stats.h_min, h);
        stats.h_max = std::max(stats.h_max, h);
    } else {
        ++stats.rejected;
    }

    const double scale = (err > 0.0)
        ? std::clamp(cfg.safety * std::pow(err, -0.2), cfg.min_scale, cfg.max_scale)
        : cfg.max_scale;
    h = std::clamp(h * scale, cfg.h_min, cfg.h_max);
    return accept;
}

}  // namespace rcvd::numerics

#endif  // RCVD_NUMERICS_INTEGRATE_HPP
