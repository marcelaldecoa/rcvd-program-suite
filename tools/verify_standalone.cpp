// tools/verify_standalone.cpp - dependency-free mirror of the Catch2 suite in
// tests/, for checking numerics on a machine without CMake or Catch2. The suite
// in tests/ is authoritative (spec 4.2 mandates Catch2 v3); this exists so the
// algorithms can be exercised with nothing but a C++20 compiler. See README.
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>
#include <algorithm>

#include "rcvd/core/result.hpp"
#include "rcvd/numerics/integrate.hpp"
#include "rcvd/numerics/linalg.hpp"
#include "rcvd/numerics/pchip.hpp"
#include "rcvd/numerics/roots.hpp"
#include "rcvd/tire/conformance.hpp"
#include "rcvd/tire/linear.hpp"

#include "support/defective_tire.hpp"
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

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
