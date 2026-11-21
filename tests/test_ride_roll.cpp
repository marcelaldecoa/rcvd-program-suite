// Spec §12 and the §17.4 acceptance vectors R-01 ... R-14, plus invariant I-5.
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <limits>
#include <random>

#include "rcvd/analysis/ride_roll.hpp"

using namespace rcvd;
using namespace rcvd::analysis;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {
constexpr double LBF = units::POUND_FORCE;
constexpr double IN  = units::INCH;
constexpr double FT  = units::FOOT;
constexpr double G   = units::G_STANDARD;
const double LBF_IN  = units::LBF_PER_IN_TO_N_PER_M;        // lbf/in  -> N/m
const double LBF_FT_DEG = units::LBF_FT_PER_DEG_TO_NM_PER_RAD;  // lbf*ft/deg -> N*m/rad

// §17.4 source case: tracks 60 in front and rear, independent front, beam rear
// with 50 in spring track, 1600 lbf front axle, 1000 lbf rear axle, tire rate
// 1500 lbf/in per tire, CG height 18 in, roll axis on the ground, target roll
// gradient 3.0 deg/g with 75 % of roll stiffness at the front.
constexpr double kTireRate = 1500.0;   // lbf/in per tire
constexpr double kWheelTrack = 60.0;   // in
}  // namespace

// ------------------------------------------------------------ §12.5 / R-01, R-01b
TEST_CASE("R-01/R-01b: roll stiffness for a 3.0 deg/g target", "[rideroll][acceptance]") {
    // W = W_s = 2600 lbf (1600 + 1000), h2 = 18 in = 1.5 ft (roll axis on ground).
    const double m_s = 2600.0 * LBF / G;
    const double h2 = 18.0 * IN;
    const double target = 3.0 * units::DEGREE / G;   // rad per (m/s^2)

    const auto t = roll_stiffness_for_gradient(m_s, h2, target, G);

    // R-01: the SIMPLIFIED (12.12) form the source uses => 1300 lbf*ft/deg.
    REQUIRE_THAT(t.simplified / LBF_FT_DEG, WithinRel(1300.0, 1e-9));
    // R-01b: the EXACT (12.11) form, including the m_s g h2 = 68.07 lbf*ft/deg
    // gravity term => 1368.07. Not published.
    REQUIRE_THAT(t.difference / LBF_FT_DEG, WithinRel(68.07, 1e-3));
    REQUIRE_THAT(t.exact / LBF_FT_DEG, WithinRel(1368.07, 1e-5));

    // §17.4: feeding the source's 1300 back through the exact roll-gradient
    // relation (12.10) gives 3.166 deg/g, not the 3.0 asked for — a 5.5 % miss on
    // the design target. This is erratum E-1c, and it is why C-04 runs against
    // R-01b rather than R-01.
    const double delivered = roll_gradient_rad_per_mps2(m_s, h2, 1300.0 * LBF_FT_DEG, G);
    REQUIRE_THAT(delivered * G / units::DEGREE, WithinRel(3.166, 1e-3));
    REQUIRE_THAT(delivered * G / units::DEGREE / 3.0 - 1.0, WithinAbs(0.055, 2e-3));

    // The exact value delivers the target it was asked for.
    const double from_exact = roll_gradient_rad_per_mps2(m_s, h2, t.exact, G);
    REQUIRE_THAT(from_exact * G / units::DEGREE, WithinRel(3.0, 1e-12));
}

// ---------------------------------------------------- §12.2 direction A, R-02 ... R-09
TEST_CASE("R-02 ... R-09: the two calculator screens reproduce", "[rideroll][acceptance]") {
    // R-02: 75/25 split of 1300 lbf*ft/deg.
    REQUIRE_THAT(0.75 * 1300.0, WithinRel(975.0, 1e-12));
    REQUIRE_THAT(0.25 * 1300.0, WithinRel(325.0, 1e-12));

    // R-05: roll stiffness due to the tires, using the 60 in WHEEL track at both
    // ends: (t^2/4)(k_T,L + k_T,R).
    const double K_tire = roll_stiffness_from_pair(kWheelTrack * IN, kTireRate * LBF_IN,
                                                   kTireRate * LBF_IN);
    REQUIRE_THAT(K_tire / LBF_FT_DEG, WithinRel(3926.98, 1e-5));

    struct Axle {
        double total_lbf_ft_deg;   // chassis-to-ground target for this axle
        double ride_rate_lbf_in;   // the §17.4 given
        double spring_track_in;
        double expect_kW;          // R-03 / R-07
        double expect_aux;         // R-04 / R-08
        double expect_springs;     // R-06 / R-09
    };
    // Front is independent (spring track == wheel track); rear is a beam axle with
    // inboard springs at 50 in. Using the wrong track is a (t_wheel/t_spring)^2
    // error — 44 % for this 60/50 case.
    const Axle axles[] = {
        {975.0, 163.6, 60.0, 183.628, 816.292, 480.736},
        {325.0, 123.7, 50.0, 134.818, 109.218, 245.106},
    };

    for (const Axle& ax : axles) {
        INFO("axle with spring track " << ax.spring_track_in << " in");

        TotalsRequest req;
        req.desired_ride_rate = ax.ride_rate_lbf_in * LBF_IN;
        req.desired_roll_stiffness = ax.total_lbf_ft_deg * LBF_FT_DEG;
        req.tire_rate_per_tire = kTireRate * LBF_IN;
        req.wheel_track = kWheelTrack * IN;
        req.spring_track = ax.spring_track_in * IN;

        const auto comp = totals_to_components(req);
        REQUIRE(comp.ok());
        // R-03 / R-07: wheel-centre rate, from (12.2) on the given ride rate.
        REQUIRE_THAT(comp.value().wheel_centre_rate_left / LBF_IN,
                     WithinRel(ax.expect_kW, 1e-5));
        // R-04 / R-08: auxiliary (anti-roll bar) roll stiffness.
        REQUIRE_THAT(comp.value().aux_roll_stiffness / LBF_FT_DEG,
                     WithinRel(ax.expect_aux, 1e-5));

        // R-06 / R-09: roll stiffness due to the ride springs alone.
        const auto tot = components_to_totals(comp.value(), G);
        REQUIRE(tot.ok());
        REQUIRE_THAT(tot.value().roll_stiffness_springs / LBF_FT_DEG,
                     WithinRel(ax.expect_springs, 1e-5));
        // And direction A returns the totals direction B was given.
        REQUIRE_THAT(tot.value().ride_rate / LBF_IN, WithinRel(ax.ride_rate_lbf_in, 1e-12));
        REQUIRE_THAT(tot.value().roll_stiffness_total / LBF_FT_DEG,
                     WithinRel(ax.total_lbf_ft_deg, 1e-12));
    }
}

// --------------------------------------------------------------- §12.4 erratum E-2
TEST_CASE("E-2: the published ride rates are sqrt(2) x their stated frequencies",
          "[rideroll][errata]") {
    // §17.4: with the correct per-corner mass, the rates implied by the stated
    // 1.0 Hz / 1.1 Hz targets are 81.8 and 61.9 lbf/in; the published 163.6 and
    // 123.7 correspond to 1.4142 Hz and 1.5556 Hz.
    struct Case { double axle_lbf, f_target, correct_lbf_in, published_lbf_in, published_f; };
    const Case cases[] = {
        {1600.0, 1.0, 81.8, 163.6, 1.41421},
        {1000.0, 1.1, 61.9, 123.7, 1.55563},
    };
    for (const Case& c : cases) {
        INFO("axle " << c.axle_lbf << " lbf at " << c.f_target << " Hz");
        const double k = ride_rate_for_frequency(c.f_target, c.axle_lbf * LBF, G);
        REQUIRE_THAT(k / LBF_IN, WithinRel(c.correct_lbf_in, 1e-3));

        const double f_pub = ride_frequency_Hz(c.published_lbf_in * LBF_IN, c.axle_lbf * LBF, G);
        REQUIRE_THAT(f_pub, WithinRel(c.published_f, 1e-4));
        // sqrt(2) times the stated target — the signature of pairing a whole-axle
        // mass with a single-wheel rate. The agreement is limited by the source's
        // own rounding: 163.6 and 123.7 lbf/in are 4-significant-figure values, so
        // the ratio recovers sqrt(2) only to ~1e-4, not exactly. §17.1's default
        // 3e-3 applies for that reason.
        REQUIRE_THAT(f_pub / c.f_target, WithinRel(std::sqrt(2.0), 3e-3));
        // Against the UNROUNDED legacy rate the relation is exact.
        const double k_legacy = ride_rate_for_frequency(c.f_target, c.axle_lbf * LBF, G, true);
        REQUIRE_THAT(ride_frequency_Hz(k_legacy, c.axle_lbf * LBF, G) / c.f_target,
                     WithinRel(std::sqrt(2.0), 1e-14));

        // --legacy-ride-frequency reproduces the source exactly, for traceability.
        const double f_legacy =
            ride_frequency_Hz(c.published_lbf_in * LBF_IN, c.axle_lbf * LBF, G, true);
        REQUIRE_THAT(f_legacy, WithinRel(c.f_target, 1e-3));
    }
}

// ------------------------------------------------------------- R-10 ... R-13
TEST_CASE("R-10: rigid tires degenerate the series step", "[rideroll][acceptance]") {
    // t = 5 ft, k_W = 150 lbf/in, target K_phi = 1000 lbf*ft/deg, tires assumed
    // RIGID, so chassis-to-ground == chassis-to-wheel-centre.
    TotalsRequest req;
    req.desired_ride_rate = 150.0 * LBF_IN;   // with rigid tires, k_R == k_W
    req.desired_roll_stiffness = 1000.0 * LBF_FT_DEG;
    req.tire_rate_per_tire = std::numeric_limits<double>::infinity();
    req.wheel_track = 60.0 * IN;
    req.spring_track = 60.0 * IN;

    const auto c = totals_to_components(req);
    REQUIRE(c.ok());
    REQUIRE_THAT(c.value().wheel_centre_rate_left / LBF_IN, WithinRel(150.0, 1e-12));
    REQUIRE_THAT(c.value().aux_roll_stiffness / LBF_FT_DEG, WithinRel(607.301, 1e-5));

    const auto t = components_to_totals(c.value(), G);
    REQUIRE(t.ok());
    REQUIRE(std::isinf(t.value().roll_stiffness_tire));
    REQUIRE_THAT(t.value().roll_stiffness_total / LBF_FT_DEG, WithinRel(1000.0, 1e-12));
}

TEST_CASE("R-11 ... R-13: ride rate is insensitive to tire rate", "[rideroll][acceptance]") {
    // The quantitative basis for the statement that tire pressure does not tune
    // ride rate: a 50 lbf/in tire-rate change (~1 psi) moves the ride rate by
    // 0.4 lbf/in, equivalent to a 0.5 lbf/in change in wheel-centre rate.
    const double r11 = ride_rate(1500.0 * LBF_IN, 150.0 * LBF_IN) / LBF_IN;
    const double r12 = ride_rate(1550.0 * LBF_IN, 150.0 * LBF_IN) / LBF_IN;
    REQUIRE_THAT(r11, WithinRel(136.364, 1e-5));
    REQUIRE_THAT(r12, WithinRel(136.765, 1e-5));
    REQUIRE(std::abs(r12 - r11) < 0.5);

    // R-13: the wheel-centre rate that gives 136.76 with k_T = 1500.
    const auto kW = wheel_centre_rate(1500.0 * LBF_IN, 136.76 * LBF_IN);
    REQUIRE(kW.ok());
    REQUIRE_THAT(kW.value() / LBF_IN, WithinRel(150.49, 1e-4));
    REQUIRE(std::abs(kW.value() / LBF_IN - 150.0) < 0.6);
}

// -------------------------------------------------------------------- I-5 / R-14
TEST_CASE("R-14 / I-5: the two directions are exact inverses", "[rideroll][invariant]") {
    // §12.3: direction B followed by direction A returns the original totals to
    // 1e-12, and vice versa, across randomised admissible inputs. This round-trip
    // property IS the acceptance test for P6.
    std::mt19937_64 rng(20260909);
    std::uniform_real_distribution<double> U(0.0, 1.0);
    auto pick = [&](double lo, double hi) { return lo + U(rng) * (hi - lo); };

    double worst_ride = 0.0, worst_roll = 0.0, worst_kW = 0.0, worst_aux = 0.0;
    int cases = 0;

    for (int i = 0; i < 10000; ++i) {
        const double k_T = pick(800.0, 2500.0) * LBF_IN;
        const double t_w = pick(45.0, 75.0) * IN;
        const double t_s = pick(35.0, 75.0) * IN;

        // ---- B then A ----------------------------------------------------
        TotalsRequest req;
        req.tire_rate_per_tire = k_T;
        req.wheel_track = t_w;
        req.spring_track = t_s;
        req.desired_ride_rate = pick(0.05, 0.55) * k_T;   // safely below the ceiling
        const double K_tire = roll_stiffness_from_pair(t_w, k_T, k_T);
        req.desired_roll_stiffness = pick(0.05, 0.7) * K_tire;

        const auto comp = totals_to_components(req);
        if (!comp.ok()) continue;
        const auto back = components_to_totals(comp.value(), G);
        if (!back.ok()) continue;
        ++cases;

        worst_ride = std::max(worst_ride,
                              std::abs(back.value().ride_rate - req.desired_ride_rate) /
                                  req.desired_ride_rate);
        worst_roll = std::max(worst_roll,
                              std::abs(back.value().roll_stiffness_total -
                                       req.desired_roll_stiffness) /
                                  req.desired_roll_stiffness);

        // ---- A then B ----------------------------------------------------
        AxleSpringing s;
        s.tire_rate_per_tire = k_T;
        s.wheel_track = t_w;
        s.spring_track = t_s;
        s.wheel_centre_rate_left = s.wheel_centre_rate_right = pick(80.0, 400.0) * LBF_IN;
        s.aux_roll_stiffness = pick(0.0, 900.0) * LBF_FT_DEG;

        const auto tot = components_to_totals(s, G);
        if (!tot.ok()) continue;
        TotalsRequest r2;
        r2.tire_rate_per_tire = k_T;
        r2.wheel_track = t_w;
        r2.spring_track = t_s;
        r2.desired_ride_rate = tot.value().ride_rate;
        r2.desired_roll_stiffness = tot.value().roll_stiffness_total;
        const auto comp2 = totals_to_components(r2);
        if (!comp2.ok()) continue;

        worst_kW = std::max(worst_kW,
                            std::abs(comp2.value().wheel_centre_rate_left -
                                     s.wheel_centre_rate_left) /
                                s.wheel_centre_rate_left);
        worst_aux = std::max(worst_aux,
                             std::abs(comp2.value().aux_roll_stiffness - s.aux_roll_stiffness) /
                                 std::max(1.0, std::abs(s.aux_roll_stiffness)));
    }

    INFO("cases " << cases << "; worst B->A ride " << worst_ride << ", roll " << worst_roll
                  << "; worst A->B k_W " << worst_kW << ", aux " << worst_aux);
    REQUIRE(cases > 9000);
    REQUIRE(worst_ride < 1e-12);
    REQUIRE(worst_roll < 1e-12);
    REQUIRE(worst_kW < 1e-12);
    REQUIRE(worst_aux < 1e-12);
}

// ------------------------------------------------------------------ §17.8 C-04
TEST_CASE("C-04: P6 output fed into P2 reproduces the target roll gradient",
          "[rideroll][crosscheck]") {
    // "P6 output fed into P2 as chassis-to-ground roll stiffnesses reproduces the
    // target roll gradient to 1e-9, using the exact inversion (12.11) — i.e.
    // against R-01b (1368.07 lbf*ft/deg), not the source's simplified R-01 (1300),
    // which lands at 3.166 deg/g."
    const double target_deg_per_g = 3.0;
    const double m_s = 2600.0 * LBF / G;
    const double h2 = 18.0 * IN;
    const double target = target_deg_per_g * units::DEGREE / G;

    const auto tgt = roll_stiffness_for_gradient(m_s, h2, target, G);

    // Split R-01b 75/25 and run each axle through P6 direction B, then A, to get
    // the chassis-to-ground stiffnesses P2 actually consumes (decision D-5).
    struct AxleSpec { double share, ride_rate_lbf_in, spring_track_in; };
    const AxleSpec specs[] = {{0.75, 163.6, 60.0}, {0.25, 123.7, 50.0}};

    double sum_K = 0.0;
    for (const AxleSpec& sp : specs) {
        TotalsRequest req;
        req.desired_ride_rate = sp.ride_rate_lbf_in * LBF_IN;
        req.desired_roll_stiffness = sp.share * tgt.exact;
        req.tire_rate_per_tire = kTireRate * LBF_IN;
        req.wheel_track = kWheelTrack * IN;
        req.spring_track = sp.spring_track_in * IN;

        const auto comp = totals_to_components(req);
        REQUIRE(comp.ok());
        const auto tot = components_to_totals(comp.value(), G);
        REQUIRE(tot.ok());
        sum_K += tot.value().roll_stiffness_total;
    }

    // Feeding that total back through (12.10) — the same expression (9.5) uses —
    // must return the target it was designed for.
    const double delivered = roll_gradient_rad_per_mps2(m_s, h2, sum_K, G) * G / units::DEGREE;
    REQUIRE_THAT(delivered, WithinRel(target_deg_per_g, 1e-9));

    // Against the source's simplified R-01 the same chain lands at 3.166, which is
    // exactly why C-04 is specified against R-01b.
    double sum_simplified = 0.0;
    for (const AxleSpec& sp : specs) {
        TotalsRequest req;
        req.desired_ride_rate = sp.ride_rate_lbf_in * LBF_IN;
        req.desired_roll_stiffness = sp.share * tgt.simplified;
        req.tire_rate_per_tire = kTireRate * LBF_IN;
        req.wheel_track = kWheelTrack * IN;
        req.spring_track = sp.spring_track_in * IN;
        const auto comp = totals_to_components(req);
        REQUIRE(comp.ok());
        sum_simplified += components_to_totals(comp.value(), G).value().roll_stiffness_total;
    }
    const double missed =
        roll_gradient_rad_per_mps2(m_s, h2, sum_simplified, G) * G / units::DEGREE;
    REQUIRE_THAT(missed, WithinRel(3.166, 1e-3));
    REQUIRE(missed > target_deg_per_g);   // a 5.5 % overshoot on the design target
}

// ------------------------------------------------------------------- §12.1 guards
TEST_CASE("E-RR-010: a ride rate at or above the tire rate is unachievable",
          "[rideroll][validation]") {
    const double k_T = 1500.0 * LBF_IN;
    REQUIRE(wheel_centre_rate(k_T, 1499.0 * LBF_IN).ok());
    REQUIRE(wheel_centre_rate(k_T, k_T).error().code == "E-RR-010");
    REQUIRE(wheel_centre_rate(k_T, 1600.0 * LBF_IN).error().code == "E-RR-010");

    // W-RR-011 above 80 % of the tire rate, where the required k_W becomes
    // extremely sensitive to the assumed tire rate.
    std::vector<core::Diagnostic> warn;
    REQUIRE(wheel_centre_rate(k_T, 0.9 * k_T, &warn).ok());
    REQUIRE(warn.size() == 1);
    REQUIRE(warn.front().code == "W-RR-011");

    warn.clear();
    REQUIRE(wheel_centre_rate(k_T, 0.5 * k_T, &warn).ok());
    REQUIRE(warn.empty());
}

TEST_CASE("E-RR-020: a roll stiffness above the tires' own is unachievable",
          "[rideroll][validation]") {
    TotalsRequest req;
    req.tire_rate_per_tire = 1500.0 * LBF_IN;
    req.wheel_track = 60.0 * IN;
    req.spring_track = 60.0 * IN;
    req.desired_ride_rate = 150.0 * LBF_IN;
    // The tires alone give 3926.98 lbf*ft/deg in roll.
    req.desired_roll_stiffness = 4000.0 * LBF_FT_DEG;
    REQUIRE(totals_to_components(req).error().code == "E-RR-020");

    req.desired_roll_stiffness = 3000.0 * LBF_FT_DEG;
    REQUIRE(totals_to_components(req).ok());
}

TEST_CASE("W-RR-021: a negative bar rate is reported, not clamped",
          "[rideroll][validation]") {
    // §12.3: a negative value is meaningful design feedback — soften the springs,
    // or accept more roll stiffness — so clamping it to zero would destroy the
    // information the user needs.
    TotalsRequest req;
    req.tire_rate_per_tire = 1500.0 * LBF_IN;
    req.wheel_track = 60.0 * IN;
    req.spring_track = 60.0 * IN;
    req.desired_ride_rate = 400.0 * LBF_IN;        // stiff springs...
    req.desired_roll_stiffness = 300.0 * LBF_FT_DEG;  // ...but a soft roll target

    const auto c = totals_to_components(req);
    REQUIRE(c.ok());
    REQUIRE(c.value().aux_roll_stiffness < 0.0);
    // And it still round-trips, because the algebra is signed throughout.
    const auto t = components_to_totals(c.value(), G);
    REQUIRE(t.ok());
    REQUIRE_THAT(t.value().roll_stiffness_total, WithinRel(req.desired_roll_stiffness, 1e-12));
}

// -------------------------------------------------------------- §12.1 conventions
TEST_CASE("12.1: the installation ratio is squared, and is travel per travel",
          "[rideroll]") {
    const double k_S = 200.0 * LBF_IN;
    // IR < 1 (spring inboard of the wheel) gives a SOFTER wheel-centre rate.
    REQUIRE(wheel_rate_from_spring(k_S, 0.6) < k_S);
    REQUIRE_THAT(wheel_rate_from_spring(k_S, 0.6), WithinRel(0.36 * k_S, 1e-15));
    // A 5 % installation-ratio error is a 10 % rate error.
    const double err = wheel_rate_from_spring(k_S, 0.63) / wheel_rate_from_spring(k_S, 0.6);
    REQUIRE_THAT(err - 1.0, WithinAbs(0.1025, 1e-3));

    // (12.4) reduces to (12.3) when theta1 == theta2.
    REQUIRE_THAT(rocker_rate_ratio(1.0, 0.6, 0.3, 0.3, 0.0), WithinRel(0.36, 1e-15));
    // Rocker rotation makes it rising-rate when the angles differ.
    REQUIRE(rocker_rate_ratio(1.0, 0.6, 0.2, 0.5, 0.1) !=
            rocker_rate_ratio(1.0, 0.6, 0.2, 0.5, 0.0));
}

TEST_CASE("12.1: split rates, and (12.6) as the symmetric case", "[rideroll]") {
    const double t = 60.0 * IN, k = 150.0 * LBF_IN;
    // (12.6) K = k*t^2/2 for k_left == k_right.
    REQUIRE_THAT(roll_stiffness_from_pair(t, k, k), WithinRel(0.5 * k * t * t, 1e-15));
    // (12.5) is linear in the pair, so a split setup is the average.
    REQUIRE_THAT(roll_stiffness_from_pair(t, 0.8 * k, 1.2 * k),
                 WithinRel(roll_stiffness_from_pair(t, k, k), 1e-15));
    REQUIRE(roll_stiffness_from_pair(t, 0.5 * k, k) < roll_stiffness_from_pair(t, k, k));
}

TEST_CASE("12.1: wheel track and spring track are not interchangeable", "[rideroll]") {
    // Using the wrong one is a (t_wheel/t_spring)^2 error — 44 % for 60 in / 50 in.
    const double wheel = roll_stiffness_from_pair(60.0 * IN, 150.0 * LBF_IN, 150.0 * LBF_IN);
    const double spring = roll_stiffness_from_pair(50.0 * IN, 150.0 * LBF_IN, 150.0 * LBF_IN);
    REQUIRE_THAT(wheel / spring, WithinRel((60.0 / 50.0) * (60.0 / 50.0), 1e-15));
    REQUIRE_THAT(wheel / spring - 1.0, WithinAbs(0.44, 1e-9));
}
