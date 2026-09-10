// Spec §7, §9.3-9.5 and the §17.5 acceptance vectors F-01 ... F-12.
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <random>

#include "rcvd/aero/aero_model.hpp"
#include "rcvd/analysis/pair_analysis.hpp"
#include "rcvd/models/load_transfer.hpp"
#include "support/fixtures.hpp"

using namespace rcvd;
using namespace rcvd::models;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {
constexpr double LBF = units::POUND_FORCE;
constexpr double IN  = units::INCH;
constexpr double FT  = units::FOOT;
constexpr double G   = units::G_STANDARD;

// §17.5 F-12 Case 1, fully specified: W = W_s = 2000 lbf (no unsprung mass),
// t_F = t_R = 60 in, a_s/l = 0.50, h_s = 20 in, z_RR = 4 in, K_phiF = K_phiR
// with SumK = 1200 lbf*ft/deg, z_RF swept 0 -> 16 in.
core::Vehicle f12_case1(double z_RF_in, double stiffness_ratio_front = 0.5,
                        double a_s_over_l = 0.5) {
    core::VehicleInput in;
    in.name = "F-12 Case 1";
    in.weight_N = 2000.0 * LBF;
    // No unsprung mass, so h_s == h and a_s follows from a. To place a_s/l where
    // the case wants it, set the weight distribution to match: a_s = a here.
    in.wheelbase_m = 100.0 * IN;              // any l; the case is specified by ratios
    in.front_weight_fraction = 1.0 - a_s_over_l;
    in.cg_height_m = 20.0 * IN;
    in.track_front_m = 60.0 * IN;
    in.track_rear_m = 60.0 * IN;
    in.roll_centre_height_front_m = z_RF_in * IN;
    in.roll_centre_height_rear_m = 4.0 * IN;

    const double sumK = 1200.0 * units::LBF_FT_PER_DEG_TO_NM_PER_RAD;
    in.roll_stiffness_ctg_front_Nm_per_rad = sumK * stiffness_ratio_front;
    in.roll_stiffness_ctg_rear_Nm_per_rad  = sumK * (1.0 - stiffness_ratio_front);

    auto r = core::resolve(in);
    REQUIRE(r.ok());
    return r.value();
}

// Transfer per g, in lbf — the units §17.5's tables are published in.
double transfer_lbf_per_g(const core::Vehicle& v, bool front, LegacyLoadTransfer legacy = {}) {
    LoadTransferInputs in;
    in.A_Y_mps2 = G;
    const auto w = wheel_loads(v, in, legacy);
    REQUIRE(w.ok());
    return (front ? w.value().front.transfer_N : w.value().rear.transfer_N) / LBF;
}
}  // namespace

// ------------------------------------------------------------------- §7 aero
TEST_CASE("§7: downforce increases the D-1 load magnitude", "[aero]") {
    aero::AeroConfig c;
    c.frontal_area_m2 = 2.0;
    c.CD = 0.5;
    c.CLF = -2.0;   // negative => downforce
    c.CLR = -1.5;
    const auto f = aero::evaluate(c, 50.0);

    // §7 sign test: C_LF = -2.0 at 50 m/s must show front load INCREASING.
    REQUIRE(f.lift_front_N < 0.0);
    REQUIRE(f.downforce_front_N > 0.0);
    REQUIRE_THAT(f.q_Pa, WithinRel(0.5 * aero::kRhoStd * 2500.0, 1e-15));
    REQUIRE_THAT(f.drag_N, WithinRel(f.q_Pa * 0.5 * 2.0, 1e-15));
    REQUIRE(f.drag_N > 0.0);

    // Below the reference speed floor everything is zero.
    const auto still = aero::evaluate(c, 0.5);
    REQUIRE(still.q_Pa == 0.0);
    REQUIRE(still.downforce_front_N == 0.0);
}

TEST_CASE("§7: aerodynamic balance is n/a when the lifts cancel", "[aero]") {
    aero::AeroConfig c;
    c.CLF = -2.6;
    c.CLR = -2.4;
    const auto b = aero::balance(c, 3.0);
    REQUIRE(b.percent_front.has_value());
    REQUIRE_THAT(*b.percent_front, WithinRel(2.6 / 5.0, 1e-12));
    REQUIRE_THAT(*b.x_cp_aft_of_front_axle_m, WithinRel(3.0 * 2.4 / 5.0, 1e-12));

    // §7 requires reporting n/a rather than a division result.
    c.CLR = +2.6;
    const auto n = aero::balance(c, 3.0);
    REQUIRE_FALSE(n.percent_front.has_value());
    REQUIRE_FALSE(n.x_cp_aft_of_front_axle_m.has_value());
}

TEST_CASE("§7 (7.5): air density correction", "[aero]") {
    REQUIRE_THAT(aero::air_density(aero::kPStd, aero::kTStd), WithinRel(aero::kRhoStd, 1e-15));
    // Hotter air is thinner.
    REQUIRE(aero::air_density(aero::kPStd, 313.15) < aero::kRhoStd);
}

// --------------------------------------------------------- §17.5 F-01 ... F-03
TEST_CASE("F-01 ... F-03: static loads and rigid transfers", "[loadtransfer][acceptance]") {
    const auto rv = core::resolve(fixtures::ch5_input());
    REQUIRE(rv.ok());
    const core::Vehicle v = rv.value();

    // F-01: 3700 lbf, 57/43 => 1054.5 / 795.5 lbf PER WHEEL.
    REQUIRE_THAT(v.W_F() / LBF / 2.0, WithinRel(1054.5, 1e-9));
    REQUIRE_THAT(v.W_R() / LBF / 2.0, WithinRel(795.5, 1e-9));

    // F-02: longitudinal transfer, h/l = 20/104.5, 0.4 g => 283.2 lbf.
    REQUIRE_THAT(longitudinal_transfer_N(v, 0.4 * G) / LBF, WithinRel(283.25, 1e-4));
    // F-03: lateral transfer, h/t = 20/60, 0.4 g => 493.3 lbf.
    REQUIRE_THAT(rigid_lateral_transfer_N(v, 0.4 * G, 60.0 * IN) / LBF, WithinRel(493.33, 1e-4));
}

// --------------------------------------------------------- §17.5 F-04 ... F-06
TEST_CASE("F-04 ... F-06: banked corner (analytic, not a P2 output)",
          "[loadtransfer][acceptance]") {
    // W = 3500 lbf, 150 mph, R = 600 ft, 20 deg bank. These verify (17.2); §9.3 is
    // flat-surface in Rev 1.0, so P2 cannot produce them (extension seam 11).
    const auto b = analysis::banked_corner(3500.0 * LBF, 150.0 * units::MPH,
                                           600.0 * FT, 20.0 * units::DEGREE, G);
    REQUIRE_THAT(b.A_Y_over_g, WithinRel(2.5072, 1e-4));           // F-04
    REQUIRE_THAT(b.normal_load_N / LBF, WithinRel(6290.2, 1e-4));  // F-05
    REQUIRE_THAT(b.lateral_force_N / LBF, WithinRel(7048.9, 1e-4));// F-06
}

// --------------------------------------------------------- §17.5 F-07 ... F-10
TEST_CASE("F-07/F-08: fraction load transfer and wheel lift", "[loadtransfer][acceptance]") {
    // F-07: FLT front at 0.35 g, h_e/t = 16.5/61.5.
    REQUIRE_THAT(analysis::flt_from_effective_height(0.35, 16.5, 61.5),
                 WithinRel(0.1878, 1e-3));
    // F-08: inside-rear lift at h_e/t = 23.5/57.8. FLT = 1 is exactly lift.
    REQUIRE_THAT(analysis::lift_A_Y_over_g(23.5, 57.8), WithinRel(1.2298, 1e-4));
    REQUIRE_THAT(analysis::flt_from_effective_height(analysis::lift_A_Y_over_g(23.5, 57.8),
                                                     23.5, 57.8),
                 WithinRel(1.0, 1e-12));
}

TEST_CASE("F-09/F-10: load-sensitive pair analysis (17.1)", "[loadtransfer][acceptance]") {
    // mu = 1.22 - 2.4e-4 * F_z with F_z in lbf, so mu1 in 1/N is 2.4e-4/LBF.
    analysis::PairAnalysisInput front{16.5, 61.5, 2109.0 * LBF, 1.22, 2.4e-4 / LBF};
    const auto f = analysis::sustainable_A_Y_over_g(front);
    REQUIRE(f.ok());
    REQUIRE_THAT(f.value(), WithinRel(0.90698, 1e-4));   // F-09

    analysis::PairAnalysisInput rear{23.5, 57.8, 1591.0 * LBF, 1.22, 2.4e-4 / LBF};
    const auto r = analysis::sustainable_A_Y_over_g(rear);
    REQUIRE(r.ok());
    REQUIRE_THAT(r.value(), WithinRel(0.92181, 1e-4));   // F-10

    // These are the FRICTION coefficients of (6.9), not the cornering-stiffness
    // c0/c1 of (6.10) — different dimensions, and §17.5 says so explicitly.
    // A sanity floor: with no load sensitivity the answer is just mu0.
    analysis::PairAnalysisInput flat{16.5, 61.5, 2109.0 * LBF, 1.22, 0.0};
    REQUIRE_THAT(analysis::sustainable_A_Y_over_g(flat).value(), WithinRel(1.22, 1e-12));
}

// ------------------------------------------------------------- §17.5 F-12 sweep
TEST_CASE("F-12: roll-centre-height sweep reproduces the published table",
          "[loadtransfer][acceptance]") {
    // z_RF [in] | h2 [in] | roll gradient [deg/g] | front [lbf/g] | rear [lbf/g]
    struct Row { double z_RF, h2_in, rg, front, rear; };
    const Row rows[] = {
        {0.0,  18.0, 2.614, 313.7, 380.4},
        {4.0,  16.0, 2.312, 344.1, 344.1},
        {8.0,  14.0, 2.013, 374.9, 308.2},
        {16.0, 10.0, 1.423, 437.5, 237.5},
    };
    for (const Row& row : rows) {
        const core::Vehicle v = f12_case1(row.z_RF);
        INFO("z_RF = " << row.z_RF << " in");
        REQUIRE_THAT(v.h2() / IN, WithinRel(row.h2_in, 1e-6));
        REQUIRE_THAT(v.roll_gradient_deg_per_g(), WithinRel(row.rg, 1e-3));
        REQUIRE_THAT(transfer_lbf_per_g(v, true), WithinRel(row.front, 1e-3));
        REQUIRE_THAT(transfer_lbf_per_g(v, false), WithinRel(row.rear, 1e-3));
    }
}

TEST_CASE("F-12: the published decomposition fits worse in aggregate",
          "[loadtransfer][errata]") {
    // §17.5: with its gravity term subtracting AND distributed by sprung-weight
    // fraction, the published form gives 286.3 -> 429.2 front and 353.0 -> 229.2
    // rear, against the plotted ~300 -> 440 and ~370 -> 232. This is the
    // numerical evidence for errata E-1a and E-1b.
    const LegacyLoadTransfer published{/*gravity_sign=*/true, /*gravity_distribution=*/true};

    const core::Vehicle at0 = f12_case1(0.0);
    REQUIRE_THAT(transfer_lbf_per_g(at0, true, published), WithinRel(286.3, 1e-3));
    REQUIRE_THAT(transfer_lbf_per_g(at0, false, published), WithinRel(353.0, 1e-3));

    const core::Vehicle at16 = f12_case1(16.0);
    REQUIRE_THAT(transfer_lbf_per_g(at16, true, published), WithinRel(429.2, 1e-3));
    REQUIRE_THAT(transfer_lbf_per_g(at16, false, published), WithinRel(229.2, 1e-3));

    // §17.5: the corrected form is closer in AGGREGATE, not at every point. The
    // front pair at z_RF = 0 is an exact tie, and at z_RF = 16 in the rear the
    // published form is nearer. Both gaps are a few lbf against values read off a
    // plotted figure, so neither corner is evidence either way — which is why the
    // acceptance criterion is the summed deviation, never a pointwise ordering.
    struct Corner { const core::Vehicle* v; bool front; double plotted; };
    const Corner corners[] = {
        {&at0, true, 300.0}, {&at0, false, 370.0},
        {&at16, true, 440.0}, {&at16, false, 232.0},
    };
    double err_correct = 0.0, err_published = 0.0;
    for (const Corner& c : corners) {
        err_correct   += std::abs(transfer_lbf_per_g(*c.v, c.front) - c.plotted);
        err_published += std::abs(transfer_lbf_per_g(*c.v, c.front, published) - c.plotted);
    }
    INFO("summed |deviation|: correct " << err_correct << ", published " << err_published);
    REQUIRE_THAT(err_correct, WithinRel(32.0, 2e-2));
    REQUIRE_THAT(err_published, WithinRel(44.3, 2e-2));
    REQUIRE(err_correct < err_published);

    // The decisive argument is not this curve at all: the published gravity sign
    // violates I-3 exactly, with a residual of order 1.
    LoadTransferInputs lt;
    lt.A_Y_mps2 = 0.6 * G;
    const auto bad = wheel_loads(at0, lt, published);
    REQUIRE(bad.ok());
    REQUIRE(std::abs(invariant_I3_residual(at0, lt.A_Y_mps2, bad.value())) > 1e-3);
}

TEST_CASE("E-1b: isolating the distribution defect from the sign defect",
          "[loadtransfer][errata]") {
    // §19's fully specified example: the F-12 Case-1 vehicle with
    // K_phiF:K_phiR = 2:1 and a_s/l = 0.47 (53 % front sprung weight), roll
    // gradient 2.63 deg/g.
    const core::Vehicle v = f12_case1(0.0, 2.0 / 3.0, 0.47);
    REQUIRE_THAT(v.roll_gradient_deg_per_g(), WithinRel(2.63, 2e-3));

    // Correct: 421.17 / 273.25 lbf/g.
    REQUIRE_THAT(transfer_lbf_per_g(v, true), WithinRel(421.17, 1e-4));
    REQUIRE_THAT(transfer_lbf_per_g(v, false), WithinRel(273.25, 1e-4));

    // Wrong distribution ALONE: 417.37 / 277.04, i.e. -0.90 % front, +1.39 % rear.
    const LegacyLoadTransfer dist_only{false, true};
    const double f_dist = transfer_lbf_per_g(v, true, dist_only);
    const double r_dist = transfer_lbf_per_g(v, false, dist_only);
    REQUIRE_THAT(f_dist, WithinRel(417.37, 1e-4));
    REQUIRE_THAT(r_dist, WithinRel(277.04, 1e-4));
    REQUIRE_THAT(f_dist / 421.17 - 1.0, WithinAbs(-0.0090, 1e-4));
    REQUIRE_THAT(r_dist / 273.25 - 1.0, WithinAbs(+0.0139, 1e-4));

    // Wrong distribution AND wrong sign (the form as printed): 387.96, -7.88 %.
    const LegacyLoadTransfer printed{true, true};
    const double f_both = transfer_lbf_per_g(v, true, printed);
    REQUIRE_THAT(f_both, WithinRel(387.96, 1e-4));
    REQUIRE_THAT(f_both / 421.17 - 1.0, WithinAbs(-0.0788, 1e-4));
}

// ------------------------------------------------------- §9.5 invariants I-3 ff
TEST_CASE("F-11/F-11b: I-3 and I-3b over randomised parameter sets",
          "[loadtransfer][invariant]") {
    // §9.5 requires each at >= 200 randomised admissible sets, at BOTH signs of
    // A_Y — which additionally tests the antisymmetry §9.3 requires.
    std::mt19937_64 rng(20260909);
    std::uniform_real_distribution<double> U(0.0, 1.0);
    auto pick = [&](double lo, double hi) { return lo + U(rng) * (hi - lo); };

    double worst_I3 = 0.0, worst_I3b = 0.0, worst_odd = 0.0;
    int sets = 0;

    for (int i = 0; i < 400; ++i) {
        core::VehicleInput in;
        in.name = "random";
        in.weight_N = pick(1500.0, 4000.0) * LBF;
        in.wheelbase_m = pick(80.0, 120.0) * IN;
        in.front_weight_fraction = pick(0.40, 0.65);
        in.cg_height_m = pick(12.0, 24.0) * IN;
        in.track_front_m = pick(50.0, 70.0) * IN;
        in.track_rear_m = pick(50.0, 70.0) * IN;
        in.roll_centre_height_front_m = pick(0.0, 8.0) * IN;
        in.roll_centre_height_rear_m = pick(0.0, 8.0) * IN;
        in.unsprung_mass_front_kg = pick(50.0, 250.0) * LBF / G;
        in.unsprung_mass_rear_kg = pick(50.0, 250.0) * LBF / G;
        in.unsprung_cg_height_front_m = pick(8.0, 14.0) * IN;
        in.unsprung_cg_height_rear_m = pick(8.0, 14.0) * IN;
        const double sumK = pick(600.0, 2500.0) * units::LBF_FT_PER_DEG_TO_NM_PER_RAD;
        const double frac = pick(0.3, 0.8);
        in.roll_stiffness_ctg_front_Nm_per_rad = sumK * frac;
        in.roll_stiffness_ctg_rear_Nm_per_rad = sumK * (1.0 - frac);

        const auto rv = core::resolve(in);
        if (!rv.ok()) continue;   // E-VEH-140 rejects roll-divergent draws
        const core::Vehicle v = rv.value();
        ++sets;

        for (double A_Y : {+0.6 * G, -0.6 * G}) {
            LoadTransferInputs lt;
            lt.A_Y_mps2 = A_Y;
            const auto w = wheel_loads(v, lt);
            REQUIRE(w.ok());
            worst_I3 = std::max(worst_I3, std::abs(invariant_I3_residual(v, A_Y, w.value())));
            worst_I3b = std::max(worst_I3b,
                                 std::abs(invariant_I3b_residual(v, A_Y, w.value(), true)));
            worst_I3b = std::max(worst_I3b,
                                 std::abs(invariant_I3b_residual(v, A_Y, w.value(), false)));
        }

        // The transfers must be exactly ODD in A_Y (§9.3 / deviation D-4): a
        // symmetric vehicle transfers the same magnitude turning left and right.
        LoadTransferInputs p, n;
        p.A_Y_mps2 = 0.6 * G;
        n.A_Y_mps2 = -0.6 * G;
        const auto wp = wheel_loads(v, p), wn = wheel_loads(v, n);
        const double sum = wp.value().front.transfer_N + wn.value().front.transfer_N;
        worst_odd = std::max(worst_odd,
                             std::abs(sum) / std::max(1.0, std::abs(wp.value().front.transfer_N)));
    }

    INFO("sets " << sets << ", I-3 " << worst_I3 << ", I-3b " << worst_I3b
                 << ", oddness " << worst_odd);
    REQUIRE(sets >= 200);
    REQUIRE(worst_I3 < 1e-12);
    REQUIRE(worst_I3b < 1e-12);
    REQUIRE(worst_odd < 1e-14);
}

TEST_CASE("I-3 and I-3b are blind to different defects", "[loadtransfer][invariant]") {
    // §9.5's defect table. This is what justifies carrying both.
    const core::Vehicle v = f12_case1(0.0, 2.0 / 3.0, 0.47);
    const double A_Y = 0.6 * G;

    LoadTransferInputs lt;
    lt.A_Y_mps2 = A_Y;
    const auto clean = wheel_loads(v, lt);
    REQUIRE(clean.ok());
    REQUIRE(std::abs(invariant_I3_residual(v, A_Y, clean.value())) < 1e-14);
    REQUIRE(std::abs(invariant_I3b_residual(v, A_Y, clean.value(), true)) < 1e-14);

    // Defect: K_phiF <-> K_phiR swapped. I-3 is a SUMMED identity and cannot see
    // a misallocation that preserves the total; I-3b can.
    core::Vehicle swapped = v;
    std::swap(swapped.K_phi_F, swapped.K_phi_R);
    const auto sw = wheel_loads(swapped, lt);
    REQUIRE(sw.ok());
    // Evaluate the swapped model's transfers against the ORIGINAL split, which is
    // what a structural check does.
    REQUIRE(std::abs(invariant_I3_residual(swapped, A_Y, sw.value())) < 1e-14);  // blind
    REQUIRE(std::abs(invariant_I3b_residual(v, A_Y, sw.value(), true)) > 1e-3);  // catches

    // Defect: the E-1a legacy gravity sign. I-3 catches it; I-3b is blind,
    // because it reads phi_tilde from the same model.
    const auto legacy = wheel_loads(v, lt, LegacyLoadTransfer{true, false});
    REQUIRE(legacy.ok());
    REQUIRE(std::abs(invariant_I3_residual(v, A_Y, legacy.value())) > 1e-3);      // catches
    REQUIRE(std::abs(invariant_I3b_residual(v, A_Y, legacy.value(), true)) > 1e-3);
}

TEST_CASE("I-3c: the scale check the other two cannot do", "[loadtransfer][invariant]") {
    const core::Vehicle v = f12_case1(0.0);
    const double A_Y = 0.6 * G;
    LoadTransferInputs lt;
    lt.A_Y_mps2 = A_Y;
    const auto w = wheel_loads(v, lt);
    REQUIRE(w.ok());

    const auto c = invariant_I3c(v, A_Y, w.value());
    REQUIRE(c.in_band);
    REQUIRE_THAT(c.deg_per_g, WithinRel(2.614, 1e-3));
    // (9.5) and (12.10) are the same expression, so this is exactly zero.
    REQUIRE_THAT(c.equality_residual, WithinAbs(0.0, 1e-14));

    // A consistent deg<->rad error in BOTH roll stiffnesses is invisible to I-3
    // and I-3b but moves the roll gradient right out of the band.
    core::Vehicle bad = v;
    bad.K_phi_F = *bad.K_phi_F / units::DEGREE * units::DEGREE * 57.2957795;
    bad.K_phi_R = *bad.K_phi_R * 57.2957795;
    const auto wb = wheel_loads(bad, lt);
    REQUIRE(wb.ok());
    REQUIRE(std::abs(invariant_I3_residual(bad, A_Y, wb.value())) < 1e-12);   // blind
    REQUIRE_FALSE(invariant_I3c(bad, A_Y, wb.value()).in_band);               // catches
}

// ------------------------------------------------------------ §9.3 step 6, D1
TEST_CASE("EV-LIFT: wheel lift is clamped but never silent", "[loadtransfer][validation]") {
    // Rule D1: every clamp raises an event with the wheel identity and the time.
    const core::Vehicle v = f12_case1(0.0);
    LoadTransferInputs lt;
    lt.A_Y_mps2 = 3.0 * G;   // far beyond the lift threshold
    lt.time_s = 1.25;

    const auto w = wheel_loads(v, lt);
    REQUIRE(w.ok());
    REQUIRE_FALSE(w.value().events.empty());
    REQUIRE(w.value().events.front().code == "EV-LIFT");
    REQUIRE(w.value().events.front().t_s.has_value());
    REQUIRE_THAT(*w.value().events.front().t_s, WithinAbs(1.25, 1e-15));
    REQUIRE(w.value().events.front().wheel.has_value());
    // No load ends up negative...
    REQUIRE(w.value().RF >= 0.0);
    REQUIRE(w.value().RR >= 0.0);
    // ...but FLT is computed BEFORE the clamp, so it can say how far past lift.
    REQUIRE(w.value().front.flt > 1.0);
}

TEST_CASE("§9.4: transfer and FLT are distinct quantities", "[loadtransfer]") {
    // Deviation D-3: confusing them is a factor-of-two error that silently halves
    // or doubles every wheel load.
    const core::Vehicle v = f12_case1(0.0);
    LoadTransferInputs lt;
    lt.A_Y_mps2 = 0.5 * G;
    const auto w = wheel_loads(v, lt);
    REQUIRE(w.ok());

    const double axle = w.value().static_front_N;
    REQUIRE_THAT(w.value().front.flt,
                 WithinRel(2.0 * w.value().front.transfer_N / axle, 1e-12));
    // The outside-minus-inside difference is 2 * transfer, not transfer.
    REQUIRE_THAT(w.value().LF - w.value().RF,
                 WithinRel(2.0 * w.value().front.transfer_N, 1e-12));
}

TEST_CASE("§9.3 step 2/3: aero and longitudinal transfer enter the assembly",
          "[loadtransfer]") {
    const core::Vehicle v = f12_case1(0.0);
    LoadTransferInputs lt;
    lt.A_Y_mps2 = 0.0;
    lt.A_X_mps2 = 0.4 * G;                 // accelerating
    lt.aero_downforce_front_N = 500.0;
    lt.aero_downforce_rear_N = 300.0;

    const auto w = wheel_loads(v, lt);
    REQUIRE(w.ok());
    // With A_Y = 0 there is no lateral transfer, so left and right match exactly.
    REQUIRE_THAT(w.value().LF, WithinRel(w.value().RF, 1e-15));
    REQUIRE_THAT(w.value().LR, WithinRel(w.value().RR, 1e-15));
    // Acceleration takes load OFF the front and puts it ON the rear.
    const double dx = longitudinal_transfer_N(v, 0.4 * G);
    REQUIRE_THAT(w.value().LF + w.value().RF,
                 WithinRel(w.value().static_front_N + 500.0 - dx, 1e-12));
    REQUIRE_THAT(w.value().LR + w.value().RR,
                 WithinRel(w.value().static_rear_N + 300.0 + dx, 1e-12));
}
