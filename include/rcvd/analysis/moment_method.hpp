// rcvd/analysis/moment_method.hpp — Program P3, spec §10.
//
// The Moment Method is a CONSTRAINED TEST. The vehicle is held at a prescribed
// speed and path curvature while body slip angle beta and steer angle delta are
// swept independently. At each (beta, delta) the tire forces are evaluated and the
// resulting lateral force and RESIDUAL YAW MOMENT are recorded. The vehicle is
// generally not in yaw equilibrium — that is the point: the residual moment
// measures the vehicle's ability to change its yaw state, which is invisible to
// any steady-state-only analysis.
#ifndef RCVD_ANALYSIS_MOMENT_METHOD_HPP
#define RCVD_ANALYSIS_MOMENT_METHOD_HPP

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "rcvd/core/result.hpp"
#include "rcvd/core/vehicle.hpp"
#include "rcvd/numerics/roots.hpp"
#include "rcvd/tire/tire_model.hpp"
#include "rcvd/units/constants.hpp"

namespace rcvd::analysis {

struct MmmSweep {
    double V_mps = 0.0;
    // §10.3: default infinity, i.e. r = 0 — the "straight-line constrained test".
    double radius_m = std::numeric_limits<double>::infinity();
    double beta_min = -15.0 * units::DEGREE, beta_max = 15.0 * units::DEGREE;
    double delta_min = -15.0 * units::DEGREE, delta_max = 15.0 * units::DEGREE;
    int beta_n = 41, delta_n = 41;   // §10.3 default grid
    bool include_aligning_moment = false;
};

struct MmmPoint {
    double beta_rad = 0.0, delta_rad = 0.0;
    double A_Y_mps2 = 0.0;   // (10.1) solver quantity
    double N_Nm = 0.0;       // (10.2) solver quantity
    double A_Y_bar = 0.0;    // (10.3a) diagram abscissa, dimensionless
    double C_N = 0.0;        // (10.3b) diagram ordinate
    double alpha_F = 0.0, alpha_R = 0.0;
    bool saturated_F = false, saturated_R = false;
};

enum class AxleMask { both, front_only, rear_only };

// The lazily-evaluated map (beta, delta) -> (A_Y, C_N), so the contouring and
// root-finding stages refine adaptively rather than being limited to the display
// grid (§10.3).
class MomentMethod {
public:
    MomentMethod(core::Vehicle veh, const tire::TireModel& front,
                 const tire::TireModel& rear, MmmSweep sweep)
        : veh_(std::move(veh)), front_(&front), rear_(&rear), sweep_(std::move(sweep)) {}

    [[nodiscard]] const core::Vehicle& vehicle() const { return veh_; }
    [[nodiscard]] const MmmSweep& sweep() const { return sweep_; }

    [[nodiscard]] MmmPoint evaluate(double beta, double delta,
                                    AxleMask mask = AxleMask::both) const {
        MmmPoint p;
        p.beta_rad = beta;
        p.delta_rad = delta;

        // §10.3. Note r/u, NOT r/V: (3.1) is normative on the body-frame
        // longitudinal velocity, and the sweep reaches +/-10 deg or more, where
        // cos(beta) differs from 1 by 1.5 %.
        const double u = sweep_.V_mps * std::cos(beta);
        const double r = std::isinf(sweep_.radius_m) ? 0.0 : sweep_.V_mps / sweep_.radius_m;

        p.alpha_F = beta + veh_.a * r / u - delta;
        p.alpha_R = beta - veh_.b * r / u;

        // The LINEARISED MMM specified here uses the STATIC axle loads; the
        // four-wheel variant with load transfer is extension seam 9 and changes
        // only the load computation, not the sweep.
        const double FzF_per_wheel = 0.5 * veh_.m * veh_.g * veh_.b / veh_.l;
        const double FzR_per_wheel = 0.5 * veh_.m * veh_.g * veh_.a / veh_.l;

        double FyF = 0.0, FyR = 0.0, Mz = 0.0;
        if (mask != AxleMask::rear_only) {
            const auto o = front_->evaluate({p.alpha_F, FzF_per_wheel});
            FyF = 2.0 * o.Fy_N;                 // two tires per axle
            p.saturated_F = o.saturated;
            if (sweep_.include_aligning_moment) Mz += 2.0 * o.Mz_Nm;
        }
        if (mask != AxleMask::front_only) {
            const auto o = rear_->evaluate({p.alpha_R, FzR_per_wheel});
            FyR = 2.0 * o.Fy_N;
            p.saturated_R = o.saturated;
            if (sweep_.include_aligning_moment) Mz += 2.0 * o.Mz_Nm;
        }

        p.A_Y_mps2 = (FyF + FyR) / veh_.m;                       // (10.1)
        p.N_Nm = veh_.a * FyF - veh_.b * FyR + Mz;               // (10.2)

        // (10.3a)/(10.3b): normalised PRESENTATION quantities, formed at the
        // boundary. Dividing the moment by m*g*l rather than m*g alone makes C_N
        // comparable across vehicles of different wheelbase.
        p.A_Y_bar = p.A_Y_mps2 / veh_.g;
        p.C_N = p.N_Nm / (veh_.m * veh_.g * veh_.l);
        return p;
    }

    // §10.3 display grid.
    [[nodiscard]] std::vector<MmmPoint> grid() const {
        std::vector<MmmPoint> out;
        out.reserve(static_cast<std::size_t>(sweep_.beta_n) * sweep_.delta_n);
        for (int i = 0; i < sweep_.beta_n; ++i) {
            const double b = lerp(sweep_.beta_min, sweep_.beta_max, i, sweep_.beta_n);
            for (int j = 0; j < sweep_.delta_n; ++j) {
                out.push_back(evaluate(b, lerp(sweep_.delta_min, sweep_.delta_max, j,
                                               sweep_.delta_n)));
            }
        }
        return out;
    }

    // §10.4 boundary.
    //
    // The true boundary is generally NON-CONVEX — the re-entrant corner is
    // precisely what distinguishes terminal plow from terminal spin — so any
    // support-function or convex-hull construction is FORBIDDEN: sweeping 360
    // directions and taking the point of maximum projection returns exactly the
    // convex hull's extreme points and provably cannot see a re-entrant notch.
    //
    // The correct construction exploits the fact that the sweep is a parameterised
    // map: the diagram is the image of the (beta, delta) rectangle, and its outer
    // edge is the image of the rectangle's BOUNDARY, traced in order.
    [[nodiscard]] std::vector<MmmPoint> boundary(int per_edge = 0) const {
        const int n = per_edge > 0 ? per_edge : std::max(sweep_.beta_n, sweep_.delta_n);
        std::vector<MmmPoint> poly;
        poly.reserve(static_cast<std::size_t>(4 * n));
        // Walk the four edges of the rectangle in order, so the image is a closed
        // polyline rather than a point cloud.
        for (int j = 0; j < n; ++j)
            poly.push_back(evaluate(sweep_.beta_min, lerp(sweep_.delta_min, sweep_.delta_max, j, n)));
        for (int i = 0; i < n; ++i)
            poly.push_back(evaluate(lerp(sweep_.beta_min, sweep_.beta_max, i, n), sweep_.delta_max));
        for (int j = n - 1; j >= 0; --j)
            poly.push_back(evaluate(sweep_.beta_max, lerp(sweep_.delta_min, sweep_.delta_max, j, n)));
        for (int i = n - 1; i >= 0; --i)
            poly.push_back(evaluate(lerp(sweep_.beta_min, sweep_.beta_max, i, n), sweep_.delta_min));
        return poly;
    }

    // §10.4: the construction lines are the same sweep with the OTHER axle's force
    // set to zero, which reduces to (10.5) for L2 and generalises correctly.
    [[nodiscard]] std::vector<MmmPoint> construction_line(bool front) const {
        std::vector<MmmPoint> out;
        const int n = std::max(sweep_.beta_n, sweep_.delta_n);
        for (int i = 0; i < n; ++i) {
            const double b = lerp(sweep_.beta_min, sweep_.beta_max, i, n);
            out.push_back(evaluate(b, front ? 0.0 : b,
                                   front ? AxleMask::front_only : AxleMask::rear_only));
        }
        return out;
    }

    // §10.5 trim line: the locus C_N = 0, found per delta by Brent root-finding in
    // beta. Where the tire model is only C0 (§6.1 T7) the bracket is subdivided at
    // the declared slope breaks before rooting.
    [[nodiscard]] std::vector<MmmPoint> trim_line() const {
        std::vector<MmmPoint> out;
        for (int j = 0; j < sweep_.delta_n; ++j) {
            const double d = lerp(sweep_.delta_min, sweep_.delta_max, j, sweep_.delta_n);
            if (const auto b = trim_beta(d)) out.push_back(evaluate(*b, d));
        }
        return out;
    }

    [[nodiscard]] std::optional<double> trim_beta(double delta) const {
        auto f = [&](double b) { return evaluate(b, delta).C_N; };
        // Subdivide at the declared slope-break images before bracketing, so a C0
        // tire's kink cannot hide a sign change inside one interval.
        std::vector<double> nodes{sweep_.beta_min};
        for (double br : front_->slope_break_points()) {
            const double b = br + delta;   // alpha_F = beta - delta at r = 0
            if (b > sweep_.beta_min && b < sweep_.beta_max) nodes.push_back(b);
        }
        for (double br : rear_->slope_break_points()) {
            if (br > sweep_.beta_min && br < sweep_.beta_max) nodes.push_back(br);
        }
        nodes.push_back(sweep_.beta_max);
        std::sort(nodes.begin(), nodes.end());

        for (std::size_t k = 0; k + 1 < nodes.size(); ++k) {
            const double lo = nodes[k], hi = nodes[k + 1];
            if (hi - lo < 1e-12) continue;
            if (f(lo) * f(hi) > 0.0) continue;
            const auto r = numerics::brent(f, lo, hi, numerics::RootConfig{1e-12, 1e-10, 100});
            if (r.ok()) return r.value().root;
        }
        return std::nullopt;
    }

private:
    static double lerp(double lo, double hi, int i, int n) {
        return n <= 1 ? lo : lo + (hi - lo) * static_cast<double>(i) / (n - 1);
    }

    core::Vehicle veh_;
    const tire::TireModel* front_;
    const tire::TireModel* rear_;
    MmmSweep sweep_;
};

// ---------------------------------------------------------------- §10.4 (10.5)
// Analytic construction-line limits for the linear-with-breakaway tire. Computed
// in closed form and drawn as reference lines.
struct ConstructionLimits {
    double front_A_Y_bar = 0.0, front_C_N = 0.0;
    double rear_A_Y_bar = 0.0, rear_C_N = 0.0;
};

[[nodiscard]] inline ConstructionLimits construction_limits(const core::Vehicle& v,
                                                            double C_F_axle, double alpha_bk_F,
                                                            double C_R_axle, double alpha_bk_R) {
    ConstructionLimits c;
    const double mg = v.m * v.g;
    // Magnitudes: the "+/-" of (10.5) is the pair of turn directions.
    c.front_A_Y_bar = std::abs(C_F_axle) * alpha_bk_F / mg;
    c.front_C_N = v.a * std::abs(C_F_axle) * alpha_bk_F / (mg * v.l);
    c.rear_A_Y_bar = std::abs(C_R_axle) * alpha_bk_R / mg;
    // Sign of C_N opposite for the rear: rear lateral force produces a RESTORING
    // yaw moment.
    c.rear_C_N = -v.b * std::abs(C_R_axle) * alpha_bk_R / (mg * v.l);
    return c;
}

// ---------------------------------------------------------------- §10.5 metrics
enum class LimitCharacter { terminal_spin, terminal_drift, terminal_plow };

[[nodiscard]] inline std::string to_string(LimitCharacter c) {
    switch (c) {
        case LimitCharacter::terminal_spin:  return "terminal spin";
        case LimitCharacter::terminal_drift: return "terminal drift";
        case LimitCharacter::terminal_plow:  return "terminal plow";
    }
    return "?";
}

struct MmmMetrics {
    double stability_index = 0.0;        // (10.6) dC_N/dA_Y_bar on the trim line
    // (10.7)'s analytic linear-range value is NOT a member: metrics() works from a
    // TireModel and cannot recover an axle cornering stiffness from it, so a field
    // here could only ever read back as zero. A caller comparing (10.6) against a
    // silent zero would conclude the cross-check had passed. Call
    // stability_index_linear(vehicle, C_F_axle, C_R_axle) with the stiffnesses you
    // supplied instead.
    double control_moment = 0.0;         // dC_N/ddelta at constant beta
    // §10.5: "the largest |A_Y_bar| on the trim line", so this is a MAGNITUDE and
    // is never negative. The (beta, delta) at which it occurs carry the direction;
    // for a symmetric vehicle the diagram is antisymmetric and both turn
    // directions give the same magnitude.
    double max_trimmed_A_Y_bar = 0.0;
    double max_trimmed_beta = 0.0, max_trimmed_delta = 0.0;
    // A saturating tire model makes the maximum FLAT in delta rather than a
    // single point. With L2, once the front axle is past breakaway its force is
    // constant, the trim condition a*Fy_F = b*Fy_R then fixes Fy_R and so beta
    // independently of delta, and A_Y stops depending on delta altogether — the
    // maximum is attained at EVERY delta past breakaway. These bound that
    // plateau; lo == hi means the maximum is a genuine single point.
    double max_trimmed_delta_lo = 0.0, max_trimmed_delta_hi = 0.0;
    bool max_trimmed_is_plateau = false;

    // How much of the steer sweep the trim line actually spans. trim_beta()
    // searches beta within the sweep's own beta window, so a steer station with
    // no trim solution in that window is dropped — and the trim line stops short
    // of delta_max without saying so. The reported maximum is then a property of
    // the BETA window, and widening that window is what moves it.
    //
    // This is easy to miss, because the truncated end need not sit exactly on the
    // beta boundary: for CH5 with non-saturating tires the trim line requires
    // beta = -4.968 * delta, so it dies at |delta| = 3.02 deg with beta at
    // -15 deg, and the last surviving grid station reports beta = -14.9 deg,
    // which no equality test against beta_max would catch.
    int trim_points = 0;           // steer stations with a trim solution
    int trim_points_requested = 0; // steer stations swept
    [[nodiscard]] bool trim_line_truncated() const {
        return trim_points < trim_points_requested;
    }
    double limit_A_Y_bar = 0.0;          // the diagram's extreme |A_Y| point
    double limit_C_N = 0.0;
    LimitCharacter character = LimitCharacter::terminal_drift;
    double trim_to_limit_distance = 0.0; // §10.5 scalar measure
};

// (10.8) steering sensitivity, industry convention: g per 100 deg of steering wheel.
[[nodiscard]] inline double steering_sensitivity(double delta_A_Y_bar,
                                                 double delta_delta_SW_rad) {
    return 100.0 * delta_A_Y_bar / (delta_delta_SW_rad / units::DEGREE);
}

// (10.9) sideslip sensitivity, deg/g.
[[nodiscard]] inline double sideslip_sensitivity(double delta_beta_rad,
                                                 double delta_A_Y_bar) {
    return (delta_beta_rad / units::DEGREE) / delta_A_Y_bar;
}

// (10.10) understeer gradient from the diagram, in rad per (m/s^2).
//
// The legacy English form UG[deg/g] = (ddelta_SW/dA_Y)/G_s - 858*l[ft]/V[mph]^2
// is recovered exactly by the unit layer; 858 is 0.120 % high (erratum E-4), and
// the implementation SHALL NOT hard-code it.
[[nodiscard]] inline double understeer_gradient(double d_delta_SW_per_A_Y, double G_s,
                                                double wheelbase_m, double V_mps) {
    return d_delta_SW_per_A_Y / G_s - wheelbase_m / (V_mps * V_mps);
}

// Same, expressed per g — which is how it is always quoted.
[[nodiscard]] inline double understeer_gradient_per_g(double d_delta_SW_per_g_rad, double G_s,
                                                      double wheelbase_m, double V_mps,
                                                      double g = units::G_STANDARD) {
    return d_delta_SW_per_g_rad / G_s - wheelbase_m * g / (V_mps * V_mps);
}

[[nodiscard]] inline MmmMetrics metrics(const MomentMethod& mmm) {
    MmmMetrics m;
    const core::Vehicle& v = mmm.vehicle();

    // Max trimmed lateral acceleration: the largest |A_Y_bar| on the trim line,
    // reported with the (beta, delta) at which it occurs.
    //
    // Two passes, not one. A single pass keeping the first strictly-greater point
    // makes the reported steer angle an artefact of the sweep WINDOW whenever the
    // maximum is flat: trim_line() walks delta upwards from delta_min, so the
    // first point on a plateau is delta_min itself, and widening the sweep merely
    // moves the answer to the new corner. (Verified for CH5 with L2 tires: the
    // reported delta tracked the window edge at every half-width from 10 to 90
    // degrees while A_Y_bar stayed at 1.517306780.)
    //
    // So: find the maximum first, then report the SMALLEST |delta| that attains
    // it — the least steer that reaches maximum trimmed lateral acceleration,
    // which is a property of the vehicle rather than of the window, and which
    // lies in the interior whenever the plateau does.
    const std::vector<MmmPoint> trim = mmm.trim_line();
    m.trim_points = static_cast<int>(trim.size());
    m.trim_points_requested = mmm.sweep().delta_n;
    for (const MmmPoint& p : trim) {
        m.max_trimmed_A_Y_bar = std::max(m.max_trimmed_A_Y_bar, std::abs(p.A_Y_bar));
    }
    if (m.max_trimmed_A_Y_bar > 0.0) {
        // The tied values are equal in exact arithmetic but are formed by
        // different sums, so they must be compared with a tolerance.
        const double tol = 1e-12 * m.max_trimmed_A_Y_bar;
        bool have = false;
        for (const MmmPoint& p : trim) {
            if (std::abs(p.A_Y_bar) < m.max_trimmed_A_Y_bar - tol) continue;
            if (!have) {
                m.max_trimmed_delta_lo = m.max_trimmed_delta_hi = p.delta_rad;
                m.max_trimmed_delta = p.delta_rad;
                m.max_trimmed_beta = p.beta_rad;
                have = true;
                continue;
            }
            m.max_trimmed_delta_lo = std::min(m.max_trimmed_delta_lo, p.delta_rad);
            m.max_trimmed_delta_hi = std::max(m.max_trimmed_delta_hi, p.delta_rad);

            // The trim line is ODD in delta, so +d and -d attain the maximum
            // together and their |delta| are equal — but only in exact
            // arithmetic. lerp() forms them as lo + (hi-lo)*i/(n-1), which
            // rounds the two products differently, so the two magnitudes can
            // differ by an ulp. A bare `<` then decides the reported sign on
            // that ulp, and the answer flips with the sweep width for no
            // physical reason. §13.1's no-bare-equality rule applies here as
            // much as anywhere: compare with a tolerance, and break the
            // remaining tie on a stated convention rather than on rounding.
            const double d_p = std::abs(p.delta_rad);
            const double d_best = std::abs(m.max_trimmed_delta);
            const double d_tol = 1e-9 * std::max({1.0, d_p, d_best});
            const bool closer = d_p < d_best - d_tol;
            // Convention: among mirrored solutions report the positive steer,
            // which pairs with a negative sideslip and a positive A_Y.
            const bool mirrored_positive =
                std::abs(d_p - d_best) <= d_tol && p.delta_rad > m.max_trimmed_delta;
            if (closer || mirrored_positive) {
                m.max_trimmed_delta = p.delta_rad;
                m.max_trimmed_beta = p.beta_rad;
            }
        }
        // A plateau is a RANGE of delta, not two mirrored points: the trim line is
        // odd in delta, so +d and -d both attain the maximum even when it is
        // otherwise unique. Require more than two tied points before saying so.
        int ties = 0;
        for (const MmmPoint& p : trim) {
            if (std::abs(p.A_Y_bar) >= m.max_trimmed_A_Y_bar - tol) ++ties;
        }
        m.max_trimmed_is_plateau = ties > 2;
    }

    // The diagram's extreme limit point, from the boundary walk.
    const auto poly = mmm.boundary();
    const MmmPoint* extreme = nullptr;
    for (const MmmPoint& p : poly) {
        if (!extreme || p.A_Y_bar > extreme->A_Y_bar) extreme = &p;
    }
    if (extreme) {
        m.limit_A_Y_bar = extreme->A_Y_bar;
        m.limit_C_N = extreme->C_N;
        // §10.5: relative to the trim line (C_N = 0) at the limit point —
        // above => terminal spin, on => terminal drift, below => terminal plow.
        constexpr double kDriftBand = 1e-9;
        m.character = (extreme->C_N > kDriftBand)    ? LimitCharacter::terminal_spin
                      : (extreme->C_N < -kDriftBand) ? LimitCharacter::terminal_plow
                                                     : LimitCharacter::terminal_drift;
        m.trim_to_limit_distance = std::abs(extreme->C_N);
    }

    // (10.6) stability index by central difference along the trim line, and
    // (10.7) the linear-range analytic cross-check.
    const double d0 = 0.0;
    if (const auto b0 = mmm.trim_beta(d0)) {
        const double h = 1e-4;
        const auto lo = mmm.evaluate(*b0 - h, d0);
        const auto hi = mmm.evaluate(*b0 + h, d0);
        const double dAY = hi.A_Y_bar - lo.A_Y_bar;
        if (std::abs(dAY) > 1e-12) m.stability_index = (hi.C_N - lo.C_N) / dAY;

        // Control moment: dC_N/ddelta at constant beta.
        const auto dl = mmm.evaluate(*b0, d0 - h);
        const auto dh = mmm.evaluate(*b0, d0 + h);
        m.control_moment = (dh.C_N - dl.C_N) / (2.0 * h);
    }
    (void)v;
    return m;
}

// (10.7) SI_linear = N_beta / (l * (C_F + C_R)), with both stiffnesses negative.
[[nodiscard]] inline double stability_index_linear(const core::Vehicle& v, double C_F_axle,
                                                   double C_R_axle) {
    const double N_beta = v.a * C_F_axle - v.b * C_R_axle;
    return N_beta / (v.l * (C_F_axle + C_R_axle));
}

}  // namespace rcvd::analysis

#endif  // RCVD_ANALYSIS_MOMENT_METHOD_HPP
