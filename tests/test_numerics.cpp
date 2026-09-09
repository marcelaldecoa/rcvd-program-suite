// Spec §13. Appendix C step 3 requires each routine to be checked against a
// closed-form test problem before any vehicle physics exists.
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <vector>

#include "rcvd/numerics/integrate.hpp"
#include "rcvd/numerics/linalg.hpp"
#include "rcvd/numerics/pchip.hpp"
#include "rcvd/numerics/roots.hpp"

using namespace rcvd::numerics;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

// ---------------------------------------------------------------- §13.5 PCHIP
TEST_CASE("PCHIP interpolates its nodes and preserves shape", "[numerics][pchip]") {
    // Step-like data, as a saturating tire produces. A natural cubic spline would
    // overshoot here; §13.5 chooses Fritsch-Carlson precisely to avoid that.
    const std::vector<double> x{0, 1, 2, 3, 4, 5};
    const std::vector<double> y{0, 0, 0, 1, 1, 1};
    const Pchip p(x, y);

    for (std::size_t i = 0; i < x.size(); ++i) {
        REQUIRE_THAT(p(x[i]), WithinAbs(y[i], 1e-14));
    }
    double lo = 1e30, hi = -1e30;
    for (double t = 0.0; t <= 5.0; t += 0.001) { lo = std::min(lo, p(t)); hi = std::max(hi, p(t)); }
    REQUIRE(hi <= 1.0 + 1e-12);   // no overshoot above the data
    REQUIRE(lo >= -1e-12);        // no undershoot below it
}

TEST_CASE("PCHIP is C1 away from its knots", "[numerics][pchip]") {
    const std::vector<double> x{0, 1, 2, 3, 4};
    const std::vector<double> y{0, 2, 3, 3.5, 3.6};
    const Pchip q(x, y);

    // Tire contract T6, with the knot exclusion T6 now states: PCHIP is C1 but not
    // C2, so a Richardson stencil straddling a knot degrades from O(h^4) to O(h)
    // and reports ~1.3e-6 for an entirely correct analytic derivative. Away from
    // the knots the agreement is ~7e-11.
    double worst = 0.0;
    for (double t = 0.3; t < 3.9; t += 0.017) {
        if (std::abs(t - std::round(t)) < 10.0 * diff_step(t)) continue;
        const double fd = central_difference([&](double u) { return q(u); }, t);
        worst = std::max(worst, std::abs(q.derivative(t) - fd) / std::max(1.0, std::abs(fd)));
    }
    INFO("worst relative slope error " << worst);
    REQUIRE(worst < 1e-6);
}

TEST_CASE("PCHIP holds the boundary value outside its range", "[numerics][pchip]") {
    // §6.4 constant extrapolation; the caller raises the Rule D2 event.
    const std::vector<double> x{0, 1, 2}, y{0, 1, 1.5};
    const Pchip p(x, y);
    REQUIRE(p(-10.0) == 0.0);
    REQUIRE(p(10.0) == 1.5);
    REQUIRE(p.derivative(-10.0) == 0.0);
    REQUIRE(p.derivative(10.0) == 0.0);
}

TEST_CASE("PCHIP keeps monotone data monotone", "[numerics][pchip]") {
    const std::vector<double> x{0, 1, 2, 3, 4};
    const std::vector<double> y{0, 2, 3, 3.5, 3.6};
    const Pchip q(x, y);
    double prev = -1e30;
    for (double t = 0.0; t <= 4.0; t += 0.001) {
        REQUIRE(q(t) >= prev - 1e-12);
        prev = q(t);
    }
}

// ---------------------------------------------------------------- §13.7 Brent
TEST_CASE("Brent finds a root on a verified bracket", "[numerics][roots]") {
    auto r = brent([](double v) { return v * v - 2.0; }, 0.0, 2.0, RootConfig{1e-15, 1e-15, 100});
    REQUIRE(r.ok());
    REQUIRE_THAT(r.value().root, WithinRel(std::sqrt(2.0), 1e-14));

    auto r2 = brent([](double v) { return std::cos(v) - v; }, 0.0, 2.0, RootConfig{1e-15, 1e-15, 100});
    REQUIRE(r2.ok());
    REQUIRE_THAT(r2.value().root, WithinRel(0.7390851332151607, 1e-14));
}

TEST_CASE("Brent reports E-NUM-300 rather than returning an endpoint", "[numerics][roots]") {
    // §13.7 is explicit: no sign change must be an error, not a plausible answer.
    auto bad = brent([](double v) { return v * v + 1.0; }, -1.0, 1.0);
    REQUIRE_FALSE(bad.ok());
    REQUIRE(bad.error().code == "E-NUM-300");
}

TEST_CASE("(13.2) differentiation step is the central-difference optimum", "[numerics][roots]") {
    REQUIRE_THAT(diff_step(1.0), WithinRel(6.055454452393343e-6, 1e-12));
    // Scales with the magnitude of x, floored by x_typ.
    REQUIRE_THAT(diff_step(1000.0), WithinRel(1000.0 * 6.055454452393343e-6, 1e-12));
    REQUIRE_THAT(diff_step(0.0, 2.0), WithinRel(2.0 * 6.055454452393343e-6, 1e-12));
    // Roundoff-limited near 1e-11; T6 asks for 1e-6, so there is ample margin.
    REQUIRE_THAT(central_difference([](double v) { return std::sin(v); }, 0.7),
                 WithinRel(std::cos(0.7), 1e-10));
}

// --------------------------------------------------------------- §13.8 linalg
TEST_CASE("LU solves and reports rcond", "[numerics][linalg]") {
    const std::array<std::array<double, 2>, 2> A{{{4.0, 1.0}, {2.0, 3.0}}};
    const auto s = lu_solve<2>(A, {1.0, 2.0});
    REQUIRE_FALSE(s.singular);
    REQUIRE_THAT(s.x[0], WithinRel(0.1, 1e-14));
    REQUIRE_THAT(s.x[1], WithinRel(0.6, 1e-14));
    REQUIRE(s.rcond > 0.1);
}

TEST_CASE("LU detects singular and near-singular systems", "[numerics][linalg]") {
    const std::array<std::array<double, 2>, 2> S{{{1.0, 2.0}, {2.0, 4.0}}};
    const auto ss = lu_solve<2>(S, {1.0, 2.0});
    REQUIRE(ss.singular);
    REQUIRE(ss.rcond == 0.0);

    // §8.5 reaches E-SS-030 through this path when 1 + K V^2 -> 0 at V_crit.
    const std::array<std::array<double, 2>, 2> N{{{1.0, 1.0}, {1.0, 1.0 + 1e-13}}};
    REQUIRE(lu_solve<2>(N, {1.0, 1.0}).rcond < 1e-12);
}

TEST_CASE("2x2 eigenvalues satisfy the trace/determinant cross-check", "[numerics][linalg]") {
    // §8.6 requires the closed-form solve to be cross-checked against tr A and
    // det A to 1e-12. Matrix is A(u) of (8.12) for the "First Example" at 40 mph.
    // A01 is Y_r/m - u, NOT Y_r - m*u: §8.4's second-column trap.
    const std::array<std::array<double, 2>, 2> A{{{-6.9128, -52.6467}, {0.44978, -9.2778}}};
    const auto e = eig2x2(A);
    const double tr  = A[0][0] + A[1][1];
    const double det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
    REQUIRE_THAT(e.re1 + e.re2, WithinRel(tr, 1e-14));
    REQUIRE_THAT(e.re1 * e.re2 - e.im1 * e.im2, WithinRel(det, 1e-13));
    REQUIRE(e.complex_pair());

    // B-22 / B-23: omega_n and zeta follow from det and trace.
    const double wn = std::sqrt(det);
    REQUIRE_THAT(wn, WithinRel(9.3709, 1e-4));
    REQUIRE_THAT(-tr / (2.0 * wn), WithinRel(0.86386, 1e-4));

    // Overdamped case stays real.
    const std::array<std::array<double, 2>, 2> R{{{-1.0, 0.0}, {0.0, -4.0}}};
    REQUIRE_FALSE(eig2x2(R).complex_pair());
}

// ----------------------------------------------------------- §13.2 integrators
namespace {
// y' = -2y, y(0) = 1  =>  y = exp(-2t)
void decay(double, std::span<const double> y, std::span<double> d) { d[0] = -2.0 * y[0]; }
// y'' = -y as a first-order pair.
void oscillator(double, std::span<const double> y, std::span<double> d) {
    d[0] = y[1];
    d[1] = -y[0];
}
}  // namespace

TEST_CASE("RK4 fixed step converges at fourth order", "[numerics][integrate]") {
    const Derivative f = decay;
    auto err_at = [&](double h) {
        std::vector<double> y{1.0}, out(1), work;
        for (double t = 0.0; t < 1.0 - 1e-12; t += h) {
            rk4_step(f, t, y, h, out, work);
            y = out;
        }
        return std::abs(y[0] - std::exp(-2.0));
    };
    const double e1 = err_at(0.01);
    const double e2 = err_at(0.005);
    REQUIRE(e2 < e1);
    REQUIRE_THAT(std::log2(e1 / e2), WithinAbs(4.0, 0.06));
}

TEST_CASE("DOPRI5(4) meets its tolerance and its dense output is accurate",
          "[numerics][integrate]") {
    const Derivative f = decay;
    AdaptiveConfig cfg;
    cfg.rtol = 1e-10;
    cfg.atol = {1e-12};
    cfg.h_max = 0.1;

    StepStats stats;
    DenseSegment seg;
    std::vector<double> y{1.0};
    double t = 0.0, h = 0.01, worst_dense = 0.0;

    while (t < 1.0 - 1e-14) {
        h = std::min(h, 1.0 - t);
        const double h_try = h;
        if (dopri54_step(f, t, y, h, cfg, seg, stats)) {
            // §11.4 locates a lap or a wall contact by this interpolant, so its
            // accuracy is a correctness requirement, not a convenience.
            for (double th : {0.13, 0.5, 0.87}) {
                std::vector<double> yi(1);
                seg.eval(th, yi);
                worst_dense = std::max(worst_dense,
                                       std::abs(yi[0] - std::exp(-2.0 * (t + th * h_try))));
            }
            t += h_try;
            y = seg.y1;
        }
    }
    REQUIRE_THAT(y[0], WithinRel(std::exp(-2.0), 1e-10));
    REQUIRE(worst_dense < 1e-8);
    REQUIRE(stats.accepted > 0);
}

TEST_CASE("dense output is continuous with the step it spans", "[numerics][integrate]") {
    const Derivative f = decay;
    AdaptiveConfig cfg;
    cfg.atol = {1e-12};
    StepStats stats;
    DenseSegment seg;
    const std::vector<double> y{1.0};
    double h = 0.01;
    REQUIRE(dopri54_step(f, 0.0, y, h, cfg, seg, stats));

    std::vector<double> a(1), b(1);
    seg.eval(0.0, a);
    seg.eval(1.0, b);
    // The rcont nesting gives these by construction; an expanded per-stage
    // polynomial only does so if every coefficient is transcribed correctly.
    REQUIRE_THAT(a[0], WithinRel(seg.y0[0], 1e-14));
    REQUIRE_THAT(b[0], WithinRel(seg.y1[0], 1e-12));
}

TEST_CASE("DOPRI5(4) tracks an oscillator over many radians", "[numerics][integrate]") {
    const Derivative f = oscillator;
    AdaptiveConfig cfg;
    cfg.rtol = 1e-11;
    cfg.atol = {1e-13, 1e-13};
    cfg.h_max = 0.5;

    StepStats stats;
    DenseSegment seg;
    std::vector<double> y{1.0, 0.0};
    double t = 0.0, h = 0.1;
    while (t < 10.0 - 1e-14) {
        h = std::min(h, 10.0 - t);
        const double h_try = h;
        if (dopri54_step(f, t, y, h, cfg, seg, stats)) {
            t += h_try;
            y = seg.y1;
        }
    }
    REQUIRE_THAT(y[0], WithinAbs(std::cos(10.0), 1e-9));
    REQUIRE_THAT(y[1], WithinAbs(-std::sin(10.0), 1e-9));
}
