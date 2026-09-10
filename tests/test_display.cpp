// Spec §3.1 Rule U2's display boundary, and §13.1 / Rule A2 precision.
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <vector>

#include "rcvd/io/display.hpp"
#include "rcvd/models/bicycle.hpp"
#include "rcvd/numerics/accumulate.hpp"
#include "support/fixtures.hpp"

using namespace rcvd;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

// ------------------------------------------------------------- Rule U2 display
TEST_CASE("U2: the display system cannot change a computed value", "[display]") {
    // The property the two-boundary rule exists to guarantee. The SI value is the
    // computation; the display system only decides how it is spelled.
    const auto v = core::resolve(fixtures::ch5_input());
    REQUIRE(v.ok());
    const auto d = models::stability_derivatives(v.value(), fixtures::ch5_tires(),
                                                 44.0 * units::FOOT);

    const io::DisplayUnits si = io::DisplayUnits::of(io::UnitSystem::si);
    const io::DisplayUnits us = io::DisplayUnits::of(io::UnitSystem::us);

    for (auto [value, dim] : std::vector<std::pair<double, units::Dim>>{
             {d.Y_beta, units::Dim::CorneringStiffness},
             {d.N_beta, units::Dim::AngularStiffness},
             {d.Y_r, units::Dim::SideForceDamping},
             {d.N_r, units::Dim::YawDamping}}) {
        const auto a = io::to_display(value, dim, si);
        const auto b = io::to_display(value, dim, us);
        REQUIRE(a.has_value());
        REQUIRE(b.has_value());
        // Different numbers, same quantity: converting each back must recover the
        // identical SI double.
        REQUIRE(*units::to_si(*a, si.unit_for(dim), dim) == value);
        REQUIRE_THAT(*units::to_si(*b, us.unit_for(dim), dim), WithinRel(value, 1e-15));
    }
}

TEST_CASE("US display reproduces the published B-vectors", "[display][acceptance]") {
    // B-03 ... B-08 and B-10/B-11/B-26 in the units [R280] prints them in — which
    // is the point of having a selectable display system at all.
    const auto v = core::resolve(fixtures::ch5_input());
    REQUIRE(v.ok());
    const auto d = models::stability_derivatives(v.value(), fixtures::ch5_tires(),
                                                 30.0 * units::MPH);
    const io::DisplayUnits us = io::DisplayUnits::of(io::UnitSystem::us);
    const auto show = [&](double si, units::Dim dim) { return *io::to_display(si, dim, us); };

    REQUIRE_THAT(show(d.Y_beta, units::Dim::CorneringStiffness), WithinRel(-1220.0, 1e-12));
    REQUIRE_THAT(show(d.Y_r, units::Dim::SideForceDamping), WithinRel(10.965, 1e-4));
    REQUIRE_THAT(show(d.Y_delta, units::Dim::CorneringStiffness), WithinRel(640.0, 1e-12));
    REQUIRE_THAT(show(d.N_beta, units::Dim::AngularStiffness), WithinRel(482.44, 1e-4));
    REQUIRE_THAT(show(d.N_r, units::Dim::YawDamping), WithinRel(-528.74, 1e-4));
    REQUIRE_THAT(show(d.N_delta, units::Dim::AngularStiffness), WithinRel(2396.53, 1e-4));

    const auto m = models::handling_metrics(v.value(), fixtures::ch5_tires());
    REQUIRE(m.V_ch.has_value());
    // B-10: 170.5 ft/s. The US default for speed is mph, so ask for ft/s by
    // override — "American format" is not one convention, and §17.2 uses both.
    io::DisplayUnits fps = us;
    fps.speed = "ft/s";
    REQUIRE_THAT(*io::to_display(*m.V_ch, units::Dim::Speed, fps), WithinRel(170.50, 1e-4));
    REQUIRE_THAT(*io::to_display(*m.V_ch, units::Dim::Speed, us), WithinRel(116.25, 1e-4));
}

TEST_CASE("US and SI defaults cover every dimension in the vocabulary", "[display]") {
    // Every dimension must resolve to a unit that actually exists for it,
    // otherwise a channel of that dimension is unprintable.
    for (io::UnitSystem sys : {io::UnitSystem::si, io::UnitSystem::us}) {
        const io::DisplayUnits d = io::DisplayUnits::of(sys);
        for (const units::UnitEntry& e : units::kVocabulary) {
            const std::string_view chosen = d.unit_for(e.dim);
            INFO("system " << io::to_string(sys) << ", dimension of " << e.name
                           << " resolved to " << chosen);
            REQUIRE(units::from_si(1.0, chosen, e.dim).has_value());
        }
    }
}

TEST_CASE("acceleration displays in g in both systems", "[display]") {
    // §3.4: "A_Y and A_X are reported to the user in g". Rule U4 permits g here
    // and only here — it is a presentation unit, never a solver quantity.
    for (io::UnitSystem sys : {io::UnitSystem::si, io::UnitSystem::us}) {
        const io::DisplayUnits d = io::DisplayUnits::of(sys);
        REQUIRE(d.unit_for(units::Dim::Acceleration) == "g");
        REQUIRE_THAT(*io::to_display(units::G_STANDARD, units::Dim::Acceleration, d),
                     WithinRel(1.0, 1e-15));
    }
    // And it is overridable for anyone who wants raw m/s^2.
    io::DisplayUnits raw = io::DisplayUnits::of(io::UnitSystem::si);
    raw.acceleration = "m/s^2";
    REQUIRE_THAT(*io::to_display(units::G_STANDARD, units::Dim::Acceleration, raw),
                 WithinRel(units::G_STANDARD, 1e-15));
}

TEST_CASE("unit systems parse from the spellings users type", "[display]") {
    REQUIRE(io::parse_unit_system("us") == io::UnitSystem::us);
    REQUIRE(io::parse_unit_system("english") == io::UnitSystem::us);
    REQUIRE(io::parse_unit_system("imperial") == io::UnitSystem::us);
    REQUIRE(io::parse_unit_system("si") == io::UnitSystem::si);
    REQUIRE(io::parse_unit_system("metric") == io::UnitSystem::si);
    REQUIRE_FALSE(io::parse_unit_system("cubits").has_value());
}

// ------------------------------------------------------- §13.1 / A2 precision
TEST_CASE("compensated summation is exact where naive summation is not",
          "[numerics][precision]") {
    // The textbook case: naive summation loses both small terms entirely.
    const std::vector<double> hard{1.0, 1e100, 1.0, -1e100};
    double naive = 0.0;
    for (double x : hard) naive += x;
    REQUIRE(naive == 0.0);                                   // both 1.0 terms gone
    REQUIRE_THAT(numerics::compensated_sum(hard), WithinAbs(2.0, 0.0));
}

TEST_CASE("the run clock does not drift over a long lap", "[numerics][precision]") {
    // The realistic case: a lap solve at 1 ms takes thousands of steps, and t is
    // summed once per step then compared against §11.4's 1e-9 s event tolerance.
    constexpr int kSteps = 300000;
    constexpr double h = 1.0 / 3.0 * 1e-3;   // a step with no exact binary form

    double naive = 0.0;
    numerics::CompensatedSum compensated;
    for (int i = 0; i < kSteps; ++i) {
        naive += h;
        compensated.add(h);
    }
    const double exact = static_cast<double>(kSteps) * h;

    const double naive_err = std::abs(naive - exact);
    const double comp_err = std::abs(compensated.value() - exact);
    INFO("naive error " << naive_err << " s, compensated error " << comp_err << " s");

    // Naive accumulation drifts; compensated does not. The absolute numbers are
    // small either way, but §11.4 asks for 1e-9 s and (11.1) compares against
    // 2*pi, so the margin is not free.
    REQUIRE(comp_err <= naive_err);
    REQUIRE(comp_err < 1e-12);
}

TEST_CASE("compensation is order-fixed, so Rule A2 still holds",
          "[numerics][precision]") {
    // A2 forbids "parallel reduction over floating point without a fixed reduction
    // tree". This is the sequential tree: identical input, identical result, every
    // time and bit for bit.
    std::vector<double> xs;
    for (int i = 1; i <= 5000; ++i) xs.push_back(1.0 / static_cast<double>(i));

    const double first = numerics::compensated_sum(xs);
    for (int rep = 0; rep < 5; ++rep) REQUIRE(numerics::compensated_sum(xs) == first);

    // Reordering the input DOES change the result — which is correct, and is why
    // A2 requires the order to be fixed rather than merely the algorithm chosen.
    std::vector<double> reversed(xs.rbegin(), xs.rend());
    REQUIRE_THAT(numerics::compensated_sum(reversed), WithinRel(first, 1e-15));
}
