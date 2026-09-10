// rcvd/analysis/pair_analysis.hpp — the closed-form axle-pair relations of
// spec §17.5, provided alongside P2 as analytic limiting checks.
//
// These are NOT the §9.3 load model. (17.1) is the load-sensitive pair equation
// the numerical solver must reproduce in the limit; (17.2) covers the banked
// corner, which Rev 1.0's flat-surface §9.3 chain cannot produce at all (see
// extension seam 11).
#ifndef RCVD_ANALYSIS_PAIR_ANALYSIS_HPP
#define RCVD_ANALYSIS_PAIR_ANALYSIS_HPP

#include <cmath>
#include <string>

#include "rcvd/core/result.hpp"

namespace rcvd::analysis {

// §9.4: FLT_axle = 2*(A_Y/g)*(h_e/t), with h_e that axle's EFFECTIVE CG height.
// FLT = 1 is exactly the inside-wheel lift condition.
[[nodiscard]] inline double flt_from_effective_height(double A_Y_over_g, double h_e_m,
                                                      double track_m) {
    return 2.0 * A_Y_over_g * (h_e_m / track_m);
}

// Inverting the above: the lateral acceleration at which the inside wheel lifts.
[[nodiscard]] inline double lift_A_Y_over_g(double h_e_m, double track_m) {
    return track_m / (2.0 * h_e_m);
}

// (17.1) the load-sensitive pair equation:
//
//   (2 mu1 eta^2 pW) Abar_Y^2 + Abar_Y + (mu1 pW / 2 - mu0) = 0,
//   eta = h_e/t,  pW = axle weight
//
// mu0, mu1 are the FRICTION coefficients of (6.9) — not the cornering-stiffness
// coefficients c0, c1 of (6.10), which have different dimensions. Units must be
// consistent: mu1 in 1/N with pW in N.
struct PairAnalysisInput {
    double h_e_m{};
    double track_m{};
    double axle_weight_N{};
    double mu0{};
    double mu1_per_N{};
};

[[nodiscard]] inline core::Result<double> sustainable_A_Y_over_g(
    const PairAnalysisInput& in) {
    const double eta = in.h_e_m / in.track_m;
    const double pW  = in.axle_weight_N;

    // (17.1) has a unit coefficient on the linear term by construction, so the
    // quadratic can only degenerate through A.
    const double A = 2.0 * in.mu1_per_N * eta * eta * pW;
    const double C = in.mu1_per_N * pW / 2.0 - in.mu0;

    if (std::abs(A) < 1e-30) {  // no load sensitivity: (17.1) is linear, Abar_Y = mu0
        return -C;
    }

    const double disc = 1.0 - 4.0 * A * C;
    if (disc < 0.0) {
        return core::err<double>(
            "E-NUM-300",
            "pair-analysis equation has no real solution (discriminant " +
                std::to_string(disc) + "); the axle cannot sustain any lateral acceleration "
                "with the supplied friction model");
    }
    // The physically meaningful root is the positive one. With mu1 > 0 and
    // C < 0 (mu0 above the load-sensitivity offset), that is (-1 + sqrt)/2A.
    return (-1.0 + std::sqrt(disc)) / (2.0 * A);
}

// (17.2) the banked corner. F-04 ... F-06 verify these; §9.3 is flat-surface
// throughout in Rev 1.0, so there is no path by which P2 produces them.
struct BankedCorner {
    double A_Y_over_g{};        // V^2/(g R)
    double normal_load_N{};     // W*(cos theta + Abar_Y sin theta)
    double lateral_force_N{};   // W*(Abar_Y cos theta - sin theta)
};

[[nodiscard]] inline BankedCorner banked_corner(double weight_N, double V_mps,
                                                double radius_m, double bank_rad,
                                                double g_mps2) {
    BankedCorner b;
    b.A_Y_over_g = V_mps * V_mps / (g_mps2 * radius_m);
    const double c = std::cos(bank_rad), s = std::sin(bank_rad);
    b.normal_load_N   = weight_N * (c + b.A_Y_over_g * s);
    b.lateral_force_N = weight_N * (b.A_Y_over_g * c - s);
    return b;
}

}  // namespace rcvd::analysis

#endif  // RCVD_ANALYSIS_PAIR_ANALYSIS_HPP
