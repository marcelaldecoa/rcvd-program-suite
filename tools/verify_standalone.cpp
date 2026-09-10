// tools/verify_standalone.cpp - dependency-free mirror of the Catch2 suite in
// tests/, for checking numerics on a machine without CMake or Catch2. The suite
// in tests/ is authoritative (spec 4.2 mandates Catch2 v3); this exists so the
// algorithms can be exercised with nothing but a C++20 compiler. See README.
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>
#include <algorithm>
#include <limits>
#include <random>

#include "rcvd/core/result.hpp"
#include "rcvd/numerics/integrate.hpp"
#include "rcvd/numerics/linalg.hpp"
#include "rcvd/numerics/pchip.hpp"
#include "rcvd/numerics/roots.hpp"
#include "rcvd/aero/aero_model.hpp"
#include "rcvd/analysis/pair_analysis.hpp"
#include "rcvd/analysis/ride_roll.hpp"
#include "rcvd/models/load_transfer.hpp"
#include "rcvd/tire/conformance.hpp"
#include "rcvd/tire/linear.hpp"

#include "support/defective_tire.hpp"
#include "support/fixtures.hpp"
#include "rcvd/units/convert.hpp"

static int g_fail = 0, g_pass = 0;

static void check(bool cond, const std::string& what) {
    if (cond) { ++g_pass; }
    else { ++g_fail; std::printf("  FAIL  %s\n", what.c_str()); }
}
static void close_to(double got, double want, double rtol, const std::string& what) {
    const double denom = std::abs(want) > 0 ? std::abs(want) : 1.0;
    const double rel = std::abs(got - want) / denom;
    if (rel <= rtol) { ++g_pass; }
    else {
        ++g_fail;
        std::printf("  FAIL  %s: got %.17g want %.17g (rel %.3e > %.3e)\n",
                    what.c_str(), got, want, rel, rtol);
    }
}

using namespace rcvd;

int main() {
    std::printf("== units (spec 3.1) ==\n");
    // Rule U5 composites, formed from the exact primitives.
    close_to(units::LBF_PER_DEG_TO_N_PER_RAD, 254.864324893, 1e-11, "1 lbf/deg -> N/rad");
    close_to(units::LBF_FT_PER_DEG_TO_NM_PER_RAD, 77.6826, 1e-5, "1 lbf*ft/deg -> N*m/rad");
    close_to(units::SLUG, units::POUND_FORCE / units::FOOT, 0.0, "SLUG == lbf/ft exactly");
    close_to(units::SLUG, 14.5939029372064, 1e-14, "SLUG == published decimal");
    // E-4: the exact English Ackermann constant; 858 is 0.120 % high.
    close_to(units::LEGACY_ACKERMANN_CONSTANT_EXACT, 856.970, 1e-5, "E-4 exact constant");
    close_to(858.0 / units::LEGACY_ACKERMANN_CONSTANT_EXACT - 1.0, 0.00120, 2e-2, "858 is 0.12% high");

    // Rule U2 round trip, both directions, < 1e-15 (Appendix D).
    {
        double worst = 0.0;
        for (const auto& e : units::kVocabulary) {
            for (double v : {1.0, -3.7, 1234.5, 1e-6}) {
                const auto si = units::to_si(v, e.name, e.dim);
                const auto back = units::from_si(*si, e.name, e.dim);
                worst = std::max(worst, std::abs(*back - v) / (std::abs(v)));
            }
        }
        check(worst < 1e-15, "round trip < 1e-15 (worst " + std::to_string(worst) + ")");
    }
    // Dimension mismatch is rejected, not silently converted.
    check(!units::to_si(1.0, "lbf", units::Dim::Length).has_value(), "lbf as Length rejected");
    check(!units::to_si(1.0, "furlong", units::Dim::Length).has_value(), "unknown unit rejected");
    // B-16: 9.979 deg road wheel at 14:1 -> 139.7 deg steering wheel.
    close_to(*units::from_si(*units::to_si(9.9790, "deg", units::Dim::Angle), "deg",
                             units::Dim::Angle) * 14.0, 139.706, 1e-4, "B-16 steering wheel");

    std::printf("== core Result (spec 4.6) ==\n");
    {
        core::Result<double> good(2.5);
        check(good.ok() && good.value() == 2.5, "successful Result carries the value");
        auto bad = core::err<double>("E-SS-030", "singular");
        check(!bad.ok(), "failed Result reports !ok()");
        check(bad.error().code == "E-SS-030", "code preserved");
        // Severity comes from the register, so a caller cannot downgrade an error.
        check(bad.error().severity == core::Severity::Error, "severity from register");
        auto warn = core::make_diagnostic("W-NUM-220", "beyond small_angle_limit");
        check(warn.severity == core::Severity::Warning, "W- code is a Warning");
        check(bad.value_or(-1.0) == -1.0, "value_or on failure");
    }

    std::printf("== pchip (spec 13.5) ==\n");
    {
        // Shape preservation: the step-like data a saturating tire produces.
        std::vector<double> x{0, 1, 2, 3, 4, 5};
        std::vector<double> y{0, 0, 0, 1, 1, 1};
        numerics::Pchip p(x, y);
        double maxv = 0.0, minv = 1.0;
        for (double t = 0; t <= 5.0; t += 0.001) { maxv = std::max(maxv, p(t)); minv = std::min(minv, p(t)); }
        check(maxv <= 1.0 + 1e-12, "no overshoot above the data (max " + std::to_string(maxv) + ")");
        check(minv >= -1e-12, "no undershoot below the data");
        // Interpolates the nodes exactly.
        for (size_t i = 0; i < x.size(); ++i) close_to(p(x[i]), y[i], 1e-14, "node " + std::to_string(i));
        // C1: analytic derivative agrees with a central difference (contract T6 style).
        std::vector<double> xs{0, 1, 2, 3, 4}, ys{0, 2, 3, 3.5, 3.6};
        numerics::Pchip q(xs, ys);
        // T6, with the knot exclusion: PCHIP is C1 but not C2, so a Richardson
        // stencil straddling a knot degrades to O(h) and reports ~1.3e-6 for an
        // entirely correct derivative. Sample at least 10h clear of each knot.
        double worst = 0.0;
        for (double t = 0.3; t < 3.9; t += 0.017) {
            if (std::abs(t - std::round(t)) < 10.0 * numerics::diff_step(t)) continue;
            const double fd = numerics::central_difference([&](double u) { return q(u); }, t);
            worst = std::max(worst, std::abs(q.derivative(t) - fd) / std::max(1.0, std::abs(fd)));
        }
        check(worst < 1e-6, "derivative vs central difference (worst " + std::to_string(worst) + ")");
        // Monotone data stays monotone.
        double prev = -1e30; bool mono = true;
        for (double t = 0; t <= 4.0; t += 0.001) { if (q(t) < prev - 1e-12) mono = false; prev = q(t); }
        check(mono, "monotone data interpolates monotonically");
        // Constant extrapolation outside the range (6.4).
        check(q(-5.0) == ys.front() && q(99.0) == ys.back(), "constant extrapolation");
    }

    std::printf("== brent (spec 13.7) ==\n");
    {
        auto r = numerics::brent([](double v) { return v * v - 2.0; }, 0.0, 2.0);
        check(r.ok(), "root found");
        close_to(r.value().root, std::sqrt(2.0), 1e-12, "sqrt(2)");
        // Transcendental. 13.7 stops on xtol OR ftol, whichever comes first, so the
        // default ftol=1e-10 limits the root to ~1e-11 relative; tighten both to
        // show the method itself is not the limit.
        auto r2 = numerics::brent([](double v) { return std::cos(v) - v; }, 0.0, 2.0,
                                  numerics::RootConfig{1e-15, 1e-15, 100});
        close_to(r2.value().root, 0.7390851332151607, 1e-14, "cos(x) = x");
        // 13.7: no sign change must report E-NUM-300, not return an endpoint.
        auto bad = numerics::brent([](double v) { return v * v + 1.0; }, -1.0, 1.0);
        check(!bad.ok() && bad.error().code == "E-NUM-300", "no bracket -> E-NUM-300");
        // 13.6 (13.2) differentiation step.
        close_to(numerics::diff_step(1.0), 6.055454452393343e-6, 1e-12, "diff_step = eps^(1/3)");
        // Roundoff-limited at ~eps/h with h = eps^(1/3): ~1e-11 is the floor, and
        // T6 asks for 1e-6, so this has ample margin.
        close_to(numerics::central_difference([](double v) { return std::sin(v); }, 0.7),
                 std::cos(0.7), 1e-10, "d/dx sin at 0.7");
    }

    std::printf("== linalg (spec 13.8) ==\n");
    {
        std::array<std::array<double, 2>, 2> A{{{4.0, 1.0}, {2.0, 3.0}}};
        auto s = numerics::lu_solve<2>(A, {1.0, 2.0});
        // A x = b  =>  x = (0.1, 0.6)
        close_to(s.x[0], 0.1, 1e-14, "lu x0");
        close_to(s.x[1], 0.6, 1e-14, "lu x1");
        check(!s.singular && s.rcond > 0.1, "well-conditioned rcond");
        // Singular matrix is detected, not solved.
        std::array<std::array<double, 2>, 2> S{{{1.0, 2.0}, {2.0, 4.0}}};
        auto ss = numerics::lu_solve<2>(S, {1.0, 2.0});
        check(ss.singular && ss.rcond == 0.0, "singular detected");
        // Near-singular gives a small rcond, which is how 8.5 reaches E-SS-030.
        std::array<std::array<double, 2>, 2> N{{{1.0, 1.0}, {1.0, 1.0 + 1e-13}}};
        auto ns = numerics::lu_solve<2>(N, {1.0, 1.0});
        check(ns.rcond < 1e-12, "near-singular rcond < 1e-12 (" + std::to_string(ns.rcond) + ")");
        // 8.6: eigenvalues cross-checked against trace and determinant, on the
        // "First Example" A(u) of (8.12) at 40 mph. Note A01 is Y_r/m - u
        // = 374.22/62.163 - 58.6667, NOT (Y_r - m*u): that is exactly the
        // second-column trap 8.4 documents, and it was in this test's data.
        std::array<std::array<double, 2>, 2> B{{{-6.9128, -52.6467}, {0.44978, -9.2778}}};
        auto e = numerics::eig2x2(B);
        const double tr = B[0][0] + B[1][1];
        const double det = B[0][0] * B[1][1] - B[0][1] * B[1][0];
        close_to(e.re1 + e.re2, tr, 1e-14, "eig sum == trace");
        close_to(e.re1 * e.re2 - e.im1 * e.im2, det, 1e-13, "eig product == det");
        check(e.complex_pair(), "underdamped pair is complex");
        // The First Example (B-20..B-23): omega_n and zeta from trace/det.
        const double wn = std::sqrt(det), zeta = -tr / (2.0 * wn);
        close_to(wn, 9.3709, 1e-4, "B-22 omega_n");
        close_to(zeta, 0.86386, 1e-4, "B-23 zeta");
    }

    std::printf("== integrators (spec 13.2) ==\n");
    {
        // Closed-form problem: y' = -2y, y(0) = 1  =>  y = exp(-2t).
        numerics::Derivative f = [](double, std::span<const double> y, std::span<double> d) {
            d[0] = -2.0 * y[0];
        };
        // RK4 fixed: 4th-order convergence under step halving.
        auto rk4_to = [&](double h) {
            std::vector<double> y{1.0}, out(1), work;
            for (double t = 0; t < 1.0 - 1e-12; t += h) {
                numerics::rk4_step(f, t, y, h, out, work);
                y = out;
            }
            return std::abs(y[0] - std::exp(-2.0));
        };
        const double e1 = rk4_to(0.01), e2 = rk4_to(0.005);
        check(e2 < e1, "RK4 error falls with step");
        const double order = std::log2(e1 / e2);
        close_to(order, 4.0, 0.06, "RK4 observed order (" + std::to_string(order) + ")");

        // DOPRI5(4): adaptive to tolerance, plus dense output between nodes.
        numerics::AdaptiveConfig cfg;
        cfg.rtol = 1e-10; cfg.atol = {1e-12}; cfg.h_max = 0.1;
        numerics::StepStats st;
        numerics::DenseSegment seg;
        std::vector<double> y{1.0};
        double t = 0.0, h = 0.01;
        double worst_dense = 0.0;
        while (t < 1.0 - 1e-14) {
            h = std::min(h, 1.0 - t);
            const double h_try = h;
            if (numerics::dopri54_step(f, t, y, h, cfg, seg, st)) {
                // Dense output must reproduce the analytic solution mid-step.
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
        close_to(y[0], std::exp(-2.0), 1e-10, "DOPRI5 endpoint");
        check(worst_dense < 1e-8, "dense output accurate (worst " + std::to_string(worst_dense) + ")");
        check(st.accepted > 0, "steps accepted");
        std::printf("  (dopri: %ld accepted, %ld rejected, dense err %.2e)\n",
                    st.accepted, st.rejected, worst_dense);

        // Dense output continuity at the segment ends: theta=0 -> y0, theta=1 -> y1.
        std::vector<double> a(1), b(1);
        seg.eval(0.0, a); seg.eval(1.0, b);
        close_to(a[0], seg.y0[0], 1e-14, "dense theta=0 == y0");
        close_to(b[0], seg.y1[0], 1e-12, "dense theta=1 == y1");

        // Oscillator y'' = -y, checks the vector path and phase accuracy over 10 rad.
        numerics::Derivative osc = [](double, std::span<const double> s, std::span<double> d) {
            d[0] = s[1]; d[1] = -s[0];
        };
        std::vector<double> z{1.0, 0.0};
        numerics::AdaptiveConfig c2; c2.rtol = 1e-11; c2.atol = {1e-13, 1e-13}; c2.h_max = 0.5;
        numerics::StepStats s2; numerics::DenseSegment g2;
        double tt = 0.0, hh = 0.1;
        while (tt < 10.0 - 1e-14) {
            hh = std::min(hh, 10.0 - tt);
            const double ht = hh;
            if (numerics::dopri54_step(osc, tt, z, hh, c2, g2, s2)) { tt += ht; z = g2.y1; }
        }
        close_to(z[0], std::cos(10.0), 1e-9, "oscillator cos(10)");
        close_to(z[1], -std::sin(10.0), 1e-9, "oscillator -sin(10)");
    }

    std::printf("== tire L1/L2 (spec 6.1-6.3, 6.7) ==\n");
    {
        using namespace rcvd::tire;
        const double kLbfDeg = units::LBF_PER_DEG_TO_N_PER_RAD;

        // --- construction validation ---------------------------------------
        {
            LinearTireConfig c; c.id = "both"; c.C_alpha_N_per_rad = -1000.0;
            c.load_sensitivity = QuadraticStiffness{-0.4, 8e-5};
            auto r = LinearTire::create(c);
            check(!r.ok() && r.error().code == "E-TIRE-005", "two stiffness sources -> E-TIRE-005");
        }
        {
            LinearTireConfig c; c.id = "none";
            auto r = LinearTire::create(c);
            check(!r.ok() && r.error().code == "E-TIRE-005", "no stiffness source -> E-TIRE-005");
        }
        {
            LinearTireConfig c; c.id = "pos"; c.C_alpha_N_per_rad = +1000.0;
            auto r = LinearTire::create(c);
            check(!r.ok() && r.error().code == "E-TIRE-003", "positive C_alpha -> E-TIRE-003");
        }
        {
            // c0 = -0.4 1/deg, c1 = 0.8e-4 1/(lbf*deg) converted to SI; sign change
            // at Fz = -c0/c1 = 5000 lbf = 22241 N. An operating ceiling above that
            // is fatal per 6.7.
            const double c0 = -0.4 / units::DEGREE;
            const double c1 = 0.8e-4 / (units::POUND_FORCE * units::DEGREE);
            LinearTireConfig c; c.id = "quad"; c.load_sensitivity = QuadraticStiffness{c0, c1};
            c.Fz_max_operating_N = 30000.0;
            auto r = LinearTire::create(c);
            check(!r.ok() && r.error().code == "E-TIRE-004", "C_alpha sign change in range -> E-TIRE-004");

            c.Fz_max_operating_N = 15000.0;   // below the 22241 N sign change
            auto r2 = LinearTire::create(c);
            check(r2.ok(), "quadratic accepted below the sign change");
            if (r2.ok()) {
                // 6.7: |C_alpha| peaks at Fz* = -c0/(2 c1) = 2500 lbf = 11120.6 N.
                const auto pk = r2.value().stiffness_peak_load_N();
                check(pk.has_value(), "stiffness peak load reported");
                close_to(*pk, 2500.0 * units::POUND_FORCE, 1e-12, "W-TIRE-101 breakpoint Fz*");
            }
        }
        {
            LinearTireConfig c; c.id = "fric"; c.C_alpha_N_per_rad = -1000.0;
            c.document_has_friction_block = true;
            auto r = LinearTire::create(c);
            check(!r.ok() && r.error().code == "E-TIRE-006", "friction block on L2 -> E-TIRE-006");
        }

        // --- L1, model 6.2 --------------------------------------------------
        LinearTireConfig l1;
        l1.id = "linear";
        l1.C_alpha_N_per_rad = -640.0 * kLbfDeg;   // CH5 front axle
        l1.pneumatic_trail_m = 1.1 * units::INCH;
        auto r_l1 = LinearTire::create(l1);
        check(r_l1.ok(), "L1 constructs");
        if (r_l1.ok()) {
            const LinearTire& t = r_l1.value();
            check(!t.has_mu() && t.is_C1(), "L1: has_mu() false, is_C1() true");
            check(t.slope_break_points().empty(), "L1 has no slope breaks");

            const auto o = t.evaluate({0.02, 5000.0});
            close_to(o.Fy_N, -640.0 * kLbfDeg * 0.02, 1e-15, "L1 Fy = C_alpha*alpha");
            close_to(o.Mz_Nm, -o.trail_m * o.Fy_N, 1e-15, "L1 (6.2) Mz = -t_p*Fy");
            close_to(o.C_alpha_N_per_rad, -640.0 * kLbfDeg, 1e-15, "L1 local slope");
            check(!o.mu_peak.has_value(), "L1 mu_peak disengaged (T5)");

            // T4 matters here precisely BECAUSE (6.1) never mentions Fz.
            const auto z = t.evaluate({0.02, 0.0});
            check(z.Fy_N == 0.0 && z.Mz_Nm == 0.0, "L1 T4: lifted wheel makes no force");
            check(std::isfinite(z.trail_m), "L1 T8: trail finite at zero load");

            // T8 at alpha = 0, where -Mz/Fy would be 0/0.
            const auto zero = t.evaluate({0.0, 5000.0});
            check(zero.Fy_N == 0.0 && zero.Mz_Nm == 0.0, "L1 T2: exact zero crossing");
            close_to(zero.trail_m, 1.1 * units::INCH, 1e-15, "L1 T8: trail is t_p at alpha=0");

            const auto rep = check_contract(t);
            check(rep.ok(), "L1 passes T1-T8 conformance");
            for (const auto& f : rep.failures) std::printf("    %s: %s\n", f.check.c_str(), f.detail.c_str());
        }

        // --- L2, model 6.3 --------------------------------------------------
        LinearTireConfig l2;
        l2.id = "linear_breakaway";
        l2.C_alpha_N_per_rad = -640.0 * kLbfDeg;
        l2.pneumatic_trail_m = 1.1 * units::INCH;
        l2.breakaway_slip_rad = 5.0 * units::DEGREE;   // Ch. 5 Prob. 5
        auto r_l2 = LinearTire::create(l2);
        check(r_l2.ok(), "L2 constructs");
        if (r_l2.ok()) {
            const LinearTire& t = r_l2.value();
            check(!t.has_mu(), "L2: has_mu() false (pure stiffness, T5)");
            check(!t.is_C1(), "L2: is_C1() false");
            check(t.slope_break_points().size() == 2, "L2 exposes both kink locations");

            const double abk = 5.0 * units::DEGREE;
            // Saturated magnitude is |C_alpha|*alpha_bk = 3200 lbf, which is what
            // makes B-13's skidpad limit 3200/2109 = 1.5173 g.
            const auto sat = t.evaluate({2.0 * abk, 5000.0});
            close_to(std::abs(sat.Fy_N) / units::POUND_FORCE, 3200.0, 1e-12, "L2 saturated force [lbf]");
            check(sat.saturated, "L2 reports saturation");
            close_to(sat.C_alpha_N_per_rad, 0.0, 0.0, "L2 local slope is 0 above breakaway");

            // 6.3: at EXACTLY |alpha| = alpha_bk the derivative from BELOW is returned.
            close_to(t.dFy_dalpha({abk, 5000.0}), -640.0 * kLbfDeg, 1e-15,
                     "L2 slope at exactly alpha_bk is the one from below");
            close_to(t.dFy_dalpha({abk * 1.0001, 5000.0}), 0.0, 0.0, "L2 slope just above is 0");

            const auto rep = check_contract(t);
            check(rep.ok(), "L2 passes T1-T8 conformance");
            for (const auto& f : rep.failures) std::printf("    %s: %s\n", f.check.c_str(), f.detail.c_str());
        }

        // --- acceptance-linked values through the model + unit layer ---------
        {
            // B-13: CH5 skidpad limit. The understeering front axle saturates
            // first, so A_Y = |C_F|*alpha_bk / W_F with W_F = 3700*0.57 = 2109 lbf.
            LinearTireConfig f = l2;
            auto tf = LinearTire::create(f);
            const double Fy_lbf = std::abs(tf.value().evaluate({0.2, 5000.0}).Fy_N) / units::POUND_FORCE;
            const double W_F = 3700.0 * 0.57;
            close_to(Fy_lbf / W_F, 1.5173, 1e-4, "B-13 skidpad limit A_Y [g]");

            // 17.6 case a: 60-slug vehicle, l = 10 ft, C = 420 lbf/deg, alpha_bk = 6 deg.
            LinearTireConfig m;
            m.id = "mmm";
            m.C_alpha_N_per_rad = -420.0 * kLbfDeg;
            m.breakaway_slip_rad = 6.0 * units::DEGREE;
            auto tm = LinearTire::create(m);
            const double Fy = std::abs(tm.value().evaluate({0.5, 5000.0}).Fy_N);
            const double mg = 60.0 * units::SLUG * units::G_STANDARD;
            close_to(Fy / mg, 1.3054, 1e-4, "17.6 case a front construction line A_Y [g]");
            const double a_ft = 5.0 * units::FOOT, l_ft = 10.0 * units::FOOT;
            close_to(a_ft * Fy / (mg * l_ft), 0.6527, 1e-4, "17.6 case a front line C_N");
        }
    }

    std::printf("== tire conformance fault injection (Appendix D) ==\n");
    {
        // A conformance suite that cannot fail is worthless. Inject one defect at a
        // time and assert the intended check -- and only that check -- fires.
        using namespace rcvd::tire;
        const double Ca = -640.0 * units::LBF_PER_DEG_TO_N_PER_RAD;

        for (int defect = static_cast<int>(Defect::T1_not_odd);
             defect <= static_cast<int>(Defect::T8_nan_trail); ++defect) {
            const Defect d = static_cast<Defect>(defect);
            DefectiveTire bad(Ca, d);
            const auto rep = check_contract(bad);
            const bool fired = std::any_of(
                rep.failures.begin(), rep.failures.end(),
                [&](const ConformanceReport::Failure& f) { return f.check == expected_check(d); });
            check(fired, "conformance catches " + std::string(defect_name(d)) +
                             " (expected " + std::string(expected_check(d)) + ")");
        }

        // And the same model with no defect injected must pass cleanly, so the
        // above is not just "everything fails".
        DefectiveTire clean(Ca, Defect::none);
        const auto rep = check_contract(clean);
        check(rep.ok(), "the un-injected control model passes");
        for (const auto& f : rep.failures) std::printf("    %s: %s\n", f.check.c_str(), f.detail.c_str());
    }

    std::printf("== P1 bicycle model, B-01 ... B-31 (spec 8, 17.2) ==\n");
    {
        using namespace rcvd::models;
        const auto rv = core::resolve(fixtures::ch5_input());
        check(rv.ok(), "CH5 resolves");
        const core::Vehicle v = rv.value();
        const auto C = fixtures::ch5_tires();
        const double V44 = 44.0 * units::FOOT;

        close_to(v.m / units::SLUG, 115.0, 1e-5, "B-01 mass [slug]");
        // B-02: the a/b inversion. 57 % FRONT weight puts the CG closer to the
        // front axle, so a = 0.43*l. Getting this backwards is defect number one.
        close_to(v.a / units::FOOT, 3.7446, 1e-4, "B-02 a [ft]");
        close_to(v.b / units::FOOT, 4.9637, 1e-4, "B-02 b [ft]");
        check(v.a < v.b, "B-02 a < b for a front-heavy car");
        close_to(v.W_F() / units::POUND_FORCE, 2109.0, 1e-9, "static front load [lbf]");

        const auto d = stability_derivatives(v, C, V44);
        close_to(d.Y_beta / fixtures::kLbfDeg, -1220.0, 1e-12, "B-03 Y_beta [lbf/deg]");
        close_to(d.Y_r / fixtures::kLbfDeg, 10.965, 1e-4, "B-04 Y_r [lbf*s/deg]");
        close_to(d.Y_delta / fixtures::kLbfDeg, 640.0, 1e-12, "B-05 Y_delta [lbf/deg]");
        const double kLbfFtDeg = units::LBF_FT_PER_DEG_TO_NM_PER_RAD;
        close_to(d.N_beta / kLbfFtDeg, 482.44, 1e-4, "B-06 N_beta [lbf*ft/deg]");
        close_to(d.N_r / kLbfFtDeg, -528.74, 1e-4, "B-07 N_r [lbf*ft*s/deg]");
        close_to(d.N_delta / kLbfFtDeg, 2396.53, 1e-4, "B-08 N_delta [lbf*ft/deg]");
        check(d.understeering(), "CH5 understeers (N_beta > 0)");

        const auto m = handling_metrics(v, C);
        close_to(m.K * units::FOOT * units::FOOT, 3.4399e-5, 1e-4, "B-09 K [s^2/ft^2]");
        check(m.V_ch.has_value(), "B-10 V_ch defined for an understeering car");
        close_to(*m.V_ch / units::FOOT, 170.50, 1e-4, "B-10 V_ch [ft/s]");
        close_to(m.NSP, 0.47541, 1e-4, "B-11 NSP");
        close_to(m.SM, 0.04541, 1e-4, "B-11 SM");
        close_to(m.NSP * v.l / units::FOOT, 4.1400, 1e-4, "B-12 neutral-steer a [ft]");
        close_to(v.l / (50.0 * units::FOOT) / units::DEGREE, 9.9790, 1e-4, "B-15 delta_Ack [deg]");

        // I-1: the derivative form (8.14) against the classical axle form (8.24).
        close_to(m.K * v.l * v.g, understeer_gradient_axle_form(v, C), 1e-12, "B-26 identity I-1");
        close_to(m.K * v.l * v.g / units::DEGREE, 0.5522, 1e-4, "B-26 K_us [deg/g]");

        // I-2: three independent routes to the same yaw-rate gain.
        const double gK  = yaw_rate_gain(v, m.K, V44);
        const double gD  = yaw_rate_gain_derivative_form(v, C, V44);
        const double gTF = yaw_rate_transfer_function(v, C, V44).dc_gain();
        close_to(gK, gD, 1e-14, "B-27 identity I-2, (8.15) vs (8.25)");
        close_to(gK, gTF, 1e-14, "B-25 identity I-2, (8.15) vs (8.28) DC gain");
        close_to(gK, 4.73716, 1e-5, "B-27 yaw-rate gain [1/s]");

        // B-17: crowned road, speed-independent by construction.
        const auto tilt = tilted_road_trim(v, C, 5.0 * units::DEGREE);
        close_to(tilt.beta_rad / units::DEGREE, 0.2392, 1e-3, "B-17 beta [deg]");
        close_to(tilt.delta_rad / units::DEGREE, -0.0480, 3e-3, "B-17 delta [deg]");

        // B-18: pure statics. The corrected values, and the two identities.
        const double F = 100.0 * units::POUND_FORCE, dd = 1.5 * units::FOOT;
        const auto cw = crosswind_trim(v, F, dd);
        close_to(cw.Fy_F_N / units::POUND_FORCE, -74.2249, 1e-5, "B-18 Fy_F [lbf]");
        close_to(cw.Fy_R_N / units::POUND_FORCE, -25.7751, 1e-5, "B-18 Fy_R [lbf]");
        close_to(cw.Fy_F_N + cw.Fy_R_N, -F, 1e-12, "B-18 forces sum to -F");
        close_to(v.a * cw.Fy_F_N - v.b * cw.Fy_R_N, -F * dd, 1e-12, "B-18 moment arm is 1.5 ft");

        // B-19 / B-31 on the oversteering variant.
        const auto ros = core::resolve(fixtures::ch5_os_input());
        check(ros.ok(), "CH5-OS resolves");
        const auto dos = stability_derivatives(ros.value(), C, V44);
        close_to(dos.N_beta / kLbfFtDeg, -1004.94, 1e-5, "B-19 reversed N_beta [lbf*ft/deg]");
        check(!dos.understeering(), "B-19 CH5-OS oversteers");

        const auto mos = handling_metrics(ros.value(), C);
        check(mos.V_crit.has_value(), "B-31 V_crit defined for an oversteering car");
        for (double V_fts : {100.0, 170.0}) {
            const double V = V_fts * units::FOOT;
            const auto e = eigenstructure(ros.value(), C, V);
            check(std::signbit(e.det) == std::signbit(1.0 + mos.K * V * V),
                  "B-31 det A and 1 + K V^2 agree in sign at " + std::to_string(V_fts) + " ft/s");
        }
        check(*mos.V_crit / units::FOOT > 100.0 && *mos.V_crit / units::FOOT < 170.0,
              "B-31 the pair brackets V_crit");
        check(!steady_state(ros.value(), C, *mos.V_crit, 0.02).ok(),
              "E-SS-030 at the critical speed");

        // B-20 ... B-25 on the First Example.
        const auto rfe = core::resolve(fixtures::first_example_input());
        check(rfe.ok(), "First Example resolves");
        const auto Cfe = fixtures::first_example_tires();
        const double V40 = 40.0 * units::MPH;
        const auto e = eigenstructure(rfe.value(), Cfe, V40);
        close_to(-e.trace, 16.190, 1e-4, "B-20 char. eq. s-coefficient");
        close_to(e.det, 87.813, 1e-4, "B-20 char. eq. constant");
        close_to(e.re1, -8.0951, 1e-4, "B-21 pole real part");
        close_to(std::abs(e.im1), 4.7204, 1e-3, "B-21 pole imaginary part");
        close_to(e.omega_n, 9.3709, 1e-4, "B-22 omega_n [rad/s]");
        close_to(e.zeta, 0.86386, 1e-4, "B-23 zeta");
        check(e.stable() && e.oscillatory(), "First Example is stable and underdamped");
        close_to(e.re1 + e.re2, e.trace, 1e-12, "8.6 eigen sum == trace");
        close_to(e.re1 * e.re2 - e.im1 * e.im2, e.det, 1e-12, "8.6 eigen product == det");

        const auto tf = yaw_rate_transfer_function(rfe.value(), Cfe, V40);
        close_to(tf.n1 * units::DEGREE, 0.8410, 1e-3, "B-24 numerator n1 [per deg]");
        close_to(tf.n0 * units::DEGREE, 7.5504, 1e-3, "B-24 numerator n0 [per deg]");
        close_to(tf.dc_gain(), 4.92645, 1e-5, "B-25 DC gain [1/s]");
    }

    std::printf("== aero + load transfer, F-01 ... F-12 (spec 7, 9.3-9.5, 17.5) ==\n");
    {
        using namespace rcvd::models;
        const double LBF = units::POUND_FORCE, IN = units::INCH, FT = units::FOOT;
        const double G = units::G_STANDARD;

        // --- 7 aerodynamics -------------------------------------------------
        aero::AeroConfig ac;
        ac.frontal_area_m2 = 2.0; ac.CD = 0.5; ac.CLF = -2.0; ac.CLR = -1.5;
        const auto af = aero::evaluate(ac, 50.0);
        check(af.lift_front_N < 0.0 && af.downforce_front_N > 0.0,
              "7 sign test: C_LF = -2.0 increases the D-1 front load");
        close_to(af.q_Pa, 0.5 * aero::kRhoStd * 2500.0, 1e-15, "7.1 dynamic pressure");
        check(aero::evaluate(ac, 0.5).q_Pa == 0.0, "below the speed floor aero is zero");
        ac.CLR = +2.0;  // cancels CLF
        check(!aero::balance(ac, 3.0).percent_front.has_value(),
              "7 balance reports n/a when C_LF + C_LR = 0");

        // --- F-12 Case 1 vehicle --------------------------------------------
        auto case1 = [&](double zRF_in, double ratio_f, double as_over_l) {
            core::VehicleInput vi;
            vi.name = "F-12 Case 1";
            vi.weight_N = 2000.0 * LBF;
            vi.wheelbase_m = 100.0 * IN;
            vi.front_weight_fraction = 1.0 - as_over_l;
            vi.cg_height_m = 20.0 * IN;
            vi.track_front_m = 60.0 * IN;
            vi.track_rear_m = 60.0 * IN;
            vi.roll_centre_height_front_m = zRF_in * IN;
            vi.roll_centre_height_rear_m = 4.0 * IN;
            const double sumK = 1200.0 * units::LBF_FT_PER_DEG_TO_NM_PER_RAD;
            vi.roll_stiffness_ctg_front_Nm_per_rad = sumK * ratio_f;
            vi.roll_stiffness_ctg_rear_Nm_per_rad = sumK * (1.0 - ratio_f);
            return core::resolve(vi).value();
        };
        auto transfer_lbf = [&](const core::Vehicle& v, bool front,
                                LegacyLoadTransfer lg = {}) {
            LoadTransferInputs lt;
            lt.A_Y_mps2 = G;
            const auto w = wheel_loads(v, lt, lg);
            return (front ? w.value().front.transfer_N : w.value().rear.transfer_N) / LBF;
        };

        // F-12: the published sweep table.
        struct Row { double zRF, h2_in, rg, f, r; };
        const Row rows[] = {{0, 18.0, 2.614, 313.7, 380.4}, {4, 16.0, 2.312, 344.1, 344.1},
                            {8, 14.0, 2.013, 374.9, 308.2}, {16, 10.0, 1.423, 437.5, 237.5}};
        for (const Row& row : rows) {
            const core::Vehicle v = case1(row.zRF, 0.5, 0.5);
            const std::string at = " at z_RF = " + std::to_string(int(row.zRF)) + " in";
            close_to(v.h2() / IN, row.h2_in, 1e-6, "F-12 h2" + at);
            close_to(v.roll_gradient_deg_per_g(), row.rg, 1e-3, "F-12 roll gradient" + at);
            close_to(transfer_lbf(v, true), row.f, 1e-3, "F-12 front transfer" + at);
            close_to(transfer_lbf(v, false), row.r, 1e-3, "F-12 rear transfer" + at);
        }

        // F-12 errata: aggregate fit, NOT a pointwise ordering. The front pair at
        // z_RF = 0 ties and the z_RF = 16 rear favours the published form; both
        // gaps are within read-off error of the plotted figure.
        const LegacyLoadTransfer published{true, true};
        const core::Vehicle c0 = case1(0, 0.5, 0.5), c16 = case1(16, 0.5, 0.5);
        struct Corner { const core::Vehicle* v; bool front; double plotted; };
        const Corner corners[] = {{&c0, true, 300.0}, {&c0, false, 370.0},
                                  {&c16, true, 440.0}, {&c16, false, 232.0}};
        double ec = 0.0, ep = 0.0;
        for (const Corner& c : corners) {
            ec += std::abs(transfer_lbf(*c.v, c.front) - c.plotted);
            ep += std::abs(transfer_lbf(*c.v, c.front, published) - c.plotted);
        }
        close_to(ec, 32.0, 2e-2, "F-12 summed deviation, corrected form");
        close_to(ep, 44.3, 2e-2, "F-12 summed deviation, published form");
        check(ec < ep, "F-12 corrected form fits better in aggregate");

        // E-1b: isolating the distribution defect from the sign defect.
        const core::Vehicle e1b = case1(0, 2.0 / 3.0, 0.47);
        close_to(e1b.roll_gradient_deg_per_g(), 2.63, 2e-3, "E-1b roll gradient [deg/g]");
        close_to(transfer_lbf(e1b, true), 421.17, 1e-4, "E-1b correct front [lbf/g]");
        close_to(transfer_lbf(e1b, false), 273.25, 1e-4, "E-1b correct rear [lbf/g]");
        close_to(transfer_lbf(e1b, true, {false, true}), 417.37, 1e-4,
                 "E-1b wrong distribution alone, front");
        close_to(transfer_lbf(e1b, false, {false, true}), 277.04, 1e-4,
                 "E-1b wrong distribution alone, rear");
        close_to(transfer_lbf(e1b, true, {true, true}), 387.96, 1e-4,
                 "E-1a+E-1b as printed, front");

        // --- F-01 ... F-10 ---------------------------------------------------
        const core::Vehicle ch5 = core::resolve(fixtures::ch5_input()).value();
        close_to(ch5.W_F() / LBF / 2.0, 1054.5, 1e-9, "F-01 static front wheel load [lbf]");
        close_to(ch5.W_R() / LBF / 2.0, 795.5, 1e-9, "F-01 static rear wheel load [lbf]");
        close_to(longitudinal_transfer_N(ch5, 0.4 * G) / LBF, 283.25, 1e-4,
                 "F-02 longitudinal transfer [lbf]");
        close_to(rigid_lateral_transfer_N(ch5, 0.4 * G, 60.0 * IN) / LBF, 493.33, 1e-4,
                 "F-03 rigid lateral transfer [lbf]");

        const auto bc = analysis::banked_corner(3500.0 * LBF, 150.0 * units::MPH,
                                                600.0 * FT, 20.0 * units::DEGREE, G);
        close_to(bc.A_Y_over_g, 2.5072, 1e-4, "F-04 banked A_Y [g]");
        close_to(bc.normal_load_N / LBF, 6290.2, 1e-4, "F-05 normal load [lbf]");
        close_to(bc.lateral_force_N / LBF, 7048.9, 1e-4, "F-06 lateral force [lbf]");

        close_to(analysis::flt_from_effective_height(0.35, 16.5, 61.5), 0.1878, 1e-3,
                 "F-07 FLT front at 0.35 g");
        close_to(analysis::lift_A_Y_over_g(23.5, 57.8), 1.2298, 1e-4,
                 "F-08 inside-rear lift [g]");
        close_to(analysis::sustainable_A_Y_over_g({16.5, 61.5, 2109.0 * LBF, 1.22,
                                                   2.4e-4 / LBF}).value(),
                 0.90698, 1e-4, "F-09 sustainable A_Y, front [g]");
        close_to(analysis::sustainable_A_Y_over_g({23.5, 57.8, 1591.0 * LBF, 1.22,
                                                   2.4e-4 / LBF}).value(),
                 0.92181, 1e-4, "F-10 sustainable A_Y, rear [g]");

        // --- F-11/F-11b invariants, and the blindness table -------------------
        {
            const core::Vehicle v = case1(0, 2.0 / 3.0, 0.47);
            const double A_Y = 0.6 * G;
            LoadTransferInputs lt;
            lt.A_Y_mps2 = A_Y;
            const auto w = wheel_loads(v, lt);
            close_to(invariant_I3_residual(v, A_Y, w.value()), 0.0, 1e-14, "I-3 residual");
            close_to(invariant_I3b_residual(v, A_Y, w.value(), true), 0.0, 1e-14,
                     "I-3b residual, front");
            const auto c = invariant_I3c(v, A_Y, w.value());
            check(c.in_band, "I-3c roll gradient inside the physical band");
            close_to(c.equality_residual, 0.0, 1e-14, "I-3c (9.5) equals (12.10)");

            // Transfers are exactly ODD in A_Y (deviation D-4).
            LoadTransferInputs n;
            n.A_Y_mps2 = -A_Y;
            const auto wn = wheel_loads(v, n);
            close_to(w.value().front.transfer_N + wn.value().front.transfer_N, 0.0, 1e-14,
                     "transfers are odd in A_Y");

            // K_phiF <-> K_phiR swapped: I-3 blind, I-3b catches.
            core::Vehicle sw = v;
            std::swap(sw.K_phi_F, sw.K_phi_R);
            const auto ws = wheel_loads(sw, lt);
            check(std::abs(invariant_I3_residual(sw, A_Y, ws.value())) < 1e-14,
                  "I-3 is blind to a K_phi swap");
            check(std::abs(invariant_I3b_residual(v, A_Y, ws.value(), true)) > 1e-3,
                  "I-3b catches a K_phi swap");

            // E-1a legacy gravity sign: I-3 catches it.
            const auto wl = wheel_loads(v, lt, LegacyLoadTransfer{true, false});
            check(std::abs(invariant_I3_residual(v, A_Y, wl.value())) > 1e-3,
                  "I-3 catches the E-1a gravity sign");
        }

        // --- Rule D1: wheel lift is clamped but never silent -----------------
        {
            LoadTransferInputs lt;
            lt.A_Y_mps2 = 3.0 * G;
            lt.time_s = 1.25;
            const auto w = wheel_loads(case1(0, 0.5, 0.5), lt);
            check(!w.value().events.empty() && w.value().events.front().code == "EV-LIFT",
                  "EV-LIFT raised on wheel lift");
            check(w.value().RF >= 0.0 && w.value().RR >= 0.0, "no load left negative");
            check(w.value().front.flt > 1.0, "FLT computed before the clamp, so it exceeds 1");
        }
    }

    std::printf("== P6 ride & roll, R-01 ... R-14 (spec 12, 17.4) ==\n");
    {
        using namespace rcvd::analysis;
        const double LBF = units::POUND_FORCE, IN = units::INCH, G = units::G_STANDARD;
        const double LBF_IN = units::LBF_PER_IN_TO_N_PER_M;
        const double LBF_FT_DEG = units::LBF_FT_PER_DEG_TO_NM_PER_RAD;

        // R-01 / R-01b: 2600 lbf sprung, h2 = 18 in, target 3.0 deg/g.
        const double m_s = 2600.0 * LBF / G, h2 = 18.0 * IN;
        const auto tgt = roll_stiffness_for_gradient(m_s, h2, 3.0 * units::DEGREE / G, G);
        close_to(tgt.simplified / LBF_FT_DEG, 1300.0, 1e-9, "R-01 simplified (12.12) [lbf*ft/deg]");
        close_to(tgt.difference / LBF_FT_DEG, 68.07, 1e-3, "R-01b gravity term m_s g h2");
        close_to(tgt.exact / LBF_FT_DEG, 1368.07, 1e-5, "R-01b exact (12.11)");
        // E-1c: the source's 1300 delivers 3.166 deg/g, a 5.5 % miss.
        close_to(roll_gradient_rad_per_mps2(m_s, h2, 1300.0 * LBF_FT_DEG, G) * G / units::DEGREE,
                 3.166, 1e-3, "E-1c delivered gradient from the simplified form");

        // R-02 ... R-09: the two calculator screens.
        close_to(roll_stiffness_from_pair(60.0 * IN, 1500.0 * LBF_IN, 1500.0 * LBF_IN) / LBF_FT_DEG,
                 3926.98, 1e-5, "R-05 roll stiffness due to tires");
        struct Axle { double total, ride, spring_track, kW, aux, springs; const char* name; };
        const Axle axles[] = {{975.0, 163.6, 60.0, 183.628, 816.292, 480.736, "front"},
                              {325.0, 123.7, 50.0, 134.818, 109.218, 245.106, "rear"}};
        for (const Axle& a : axles) {
            TotalsRequest req;
            req.desired_ride_rate = a.ride * LBF_IN;
            req.desired_roll_stiffness = a.total * LBF_FT_DEG;
            req.tire_rate_per_tire = 1500.0 * LBF_IN;
            req.wheel_track = 60.0 * IN;
            req.spring_track = a.spring_track * IN;
            const auto c = totals_to_components(req);
            check(c.ok(), std::string("P6 direction B solves, ") + a.name);
            close_to(c.value().wheel_centre_rate_left / LBF_IN, a.kW, 1e-5,
                     std::string("R-03/R-07 wheel-centre rate, ") + a.name);
            close_to(c.value().aux_roll_stiffness / LBF_FT_DEG, a.aux, 1e-5,
                     std::string("R-04/R-08 aux roll stiffness, ") + a.name);
            const auto t = components_to_totals(c.value(), G);
            close_to(t.value().roll_stiffness_springs / LBF_FT_DEG, a.springs, 1e-5,
                     std::string("R-06/R-09 roll stiffness due to springs, ") + a.name);
            close_to(t.value().roll_stiffness_total / LBF_FT_DEG, a.total, 1e-12,
                     std::string("A o B returns the total, ") + a.name);
        }

        // E-2: the published rates are sqrt(2) x their stated frequencies.
        close_to(ride_rate_for_frequency(1.0, 1600.0 * LBF, G) / LBF_IN, 81.8, 1e-3,
                 "E-2 correct front rate for 1.0 Hz [lbf/in]");
        close_to(ride_rate_for_frequency(1.1, 1000.0 * LBF, G) / LBF_IN, 61.9, 1e-3,
                 "E-2 correct rear rate for 1.1 Hz [lbf/in]");
        close_to(ride_frequency_Hz(163.6 * LBF_IN, 1600.0 * LBF, G), 1.41421, 1e-3,
                 "E-2 published 163.6 gives 1.4142 Hz");
        close_to(ride_frequency_Hz(163.6 * LBF_IN, 1600.0 * LBF, G, true), 1.0, 1e-3,
                 "--legacy-ride-frequency reproduces the source");

        // R-10 ... R-13.
        {
            TotalsRequest req;
            req.desired_ride_rate = 150.0 * LBF_IN;
            req.desired_roll_stiffness = 1000.0 * LBF_FT_DEG;
            req.tire_rate_per_tire = std::numeric_limits<double>::infinity();
            req.wheel_track = 60.0 * IN;
            req.spring_track = 60.0 * IN;
            close_to(totals_to_components(req).value().aux_roll_stiffness / LBF_FT_DEG,
                     607.301, 1e-5, "R-10 aux with rigid tires");
        }
        close_to(ride_rate(1500.0 * LBF_IN, 150.0 * LBF_IN) / LBF_IN, 136.364, 1e-5, "R-11");
        close_to(ride_rate(1550.0 * LBF_IN, 150.0 * LBF_IN) / LBF_IN, 136.765, 1e-5, "R-12");
        close_to(wheel_centre_rate(1500.0 * LBF_IN, 136.76 * LBF_IN).value() / LBF_IN,
                 150.49, 1e-4, "R-13");

        // Guards.
        check(wheel_centre_rate(1500.0 * LBF_IN, 1500.0 * LBF_IN).error().code == "E-RR-010",
              "E-RR-010 when the desired ride rate reaches the tire rate");
        {
            TotalsRequest req;
            req.tire_rate_per_tire = 1500.0 * LBF_IN;
            req.wheel_track = 60.0 * IN;
            req.spring_track = 60.0 * IN;
            req.desired_ride_rate = 150.0 * LBF_IN;
            req.desired_roll_stiffness = 4000.0 * LBF_FT_DEG;
            check(totals_to_components(req).error().code == "E-RR-020",
                  "E-RR-020 above the tires' own roll stiffness");
        }

        // R-14 / I-5: the two directions are exact inverses.
        {
            std::mt19937_64 rng(20260909);
            std::uniform_real_distribution<double> U(0.0, 1.0);
            auto pick = [&](double lo, double hi) { return lo + U(rng) * (hi - lo); };
            double worst = 0.0;
            int n = 0;
            for (int i = 0; i < 10000; ++i) {
                TotalsRequest req;
                req.tire_rate_per_tire = pick(800.0, 2500.0) * LBF_IN;
                req.wheel_track = pick(45.0, 75.0) * IN;
                req.spring_track = pick(35.0, 75.0) * IN;
                req.desired_ride_rate = pick(0.05, 0.55) * req.tire_rate_per_tire;
                const double K_tire = roll_stiffness_from_pair(
                    req.wheel_track, req.tire_rate_per_tire, req.tire_rate_per_tire);
                req.desired_roll_stiffness = pick(0.05, 0.7) * K_tire;
                const auto c = totals_to_components(req);
                if (!c.ok()) continue;
                const auto t = components_to_totals(c.value(), G);
                if (!t.ok()) continue;
                ++n;
                worst = std::max(worst,
                                 std::abs(t.value().ride_rate - req.desired_ride_rate) /
                                     req.desired_ride_rate);
                worst = std::max(worst,
                                 std::abs(t.value().roll_stiffness_total -
                                          req.desired_roll_stiffness) /
                                     req.desired_roll_stiffness);
            }
            check(n > 9000, "I-5 ran " + std::to_string(n) + " admissible cases");
            check(worst < 1e-12, "R-14 round trip worst error " + std::to_string(worst));
        }

        // C-04: P6 into P2 reproduces the target roll gradient, against R-01b.
        {
            struct Sp { double share, ride, track; };
            const Sp sps[] = {{0.75, 163.6, 60.0}, {0.25, 123.7, 50.0}};
            double sumK = 0.0;
            for (const Sp& s : sps) {
                TotalsRequest req;
                req.desired_ride_rate = s.ride * LBF_IN;
                req.desired_roll_stiffness = s.share * tgt.exact;
                req.tire_rate_per_tire = 1500.0 * LBF_IN;
                req.wheel_track = 60.0 * IN;
                req.spring_track = s.track * IN;
                sumK += components_to_totals(totals_to_components(req).value(), G)
                            .value().roll_stiffness_total;
            }
            close_to(roll_gradient_rad_per_mps2(m_s, h2, sumK, G) * G / units::DEGREE, 3.0,
                     1e-9, "C-04 delivered roll gradient [deg/g]");
        }
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
