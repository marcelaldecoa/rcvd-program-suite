// Spec §4.6 (Rule A3) and §14.1-14.2.
#include <catch2/catch_test_macros.hpp>

#include "rcvd/core/result.hpp"

using namespace rcvd::core;

TEST_CASE("Result carries either a value or a Diagnostic", "[core]") {
    Result<double> good(2.5);
    REQUIRE(good.ok());
    REQUIRE(static_cast<bool>(good));
    REQUIRE(good.value() == 2.5);

    auto bad = err<double>("E-SS-030", "system singular at 170 ft/s");
    REQUIRE_FALSE(bad.ok());
    REQUIRE_FALSE(static_cast<bool>(bad));
    REQUIRE(bad.error().code == "E-SS-030");
    REQUIRE(bad.value_or(-1.0) == -1.0);
}

TEST_CASE("severity is taken from the register, not the caller", "[core]") {
    // A caller cannot raise an E- code as a warning, or vice versa.
    REQUIRE(make_diagnostic("E-SS-030", "x").severity == Severity::Error);
    REQUIRE(make_diagnostic("W-NUM-220", "x").severity == Severity::Warning);
    REQUIRE(make_diagnostic("EV-LIFT", "x").severity == Severity::Event);
    REQUIRE(make_diagnostic("H-SPD-001", "x").severity == Severity::Hint);
}

TEST_CASE("an unregistered code degrades to E-INTERNAL", "[core]") {
    const auto d = make_diagnostic("E-NOT-A-CODE", "typo");
    REQUIRE(d.code == "E-INTERNAL");
    REQUIRE(d.severity == Severity::Error);
}

TEST_CASE("diagnostics carry when and where", "[core]") {
    const auto d = make_diagnostic("EV-LIFT", "inside rear lifted", 1.25, Wheel::RR);
    REQUIRE(d.t_s.has_value());
    REQUIRE(*d.t_s == 1.25);
    REQUIRE(d.wheel.has_value());
    REQUIRE(to_string(*d.wheel) == "RR");
}

TEST_CASE("the 14.2 register is well formed", "[core]") {
    // Appendix D requires every code to have a test that provokes it. That gate
    // iterates this table, so the table must have unique, correctly-prefixed codes.
    for (const auto& e : codes::kRegister) {
        INFO("code " << e.code);
        REQUIRE_FALSE(e.code.empty());
        REQUIRE_FALSE(e.meaning.empty());
        switch (e.severity) {
            case Severity::Error:   REQUIRE(e.code.starts_with("E-"));  break;
            case Severity::Warning: REQUIRE(e.code.starts_with("W-"));  break;
            case Severity::Event:   REQUIRE(e.code.starts_with("EV-")); break;
            case Severity::Hint:    REQUIRE(e.code.starts_with("H-"));  break;
        }
        // Lookup must round-trip.
        const auto found = codes::find(e.code);
        REQUIRE(found.has_value());
        REQUIRE(found->severity == e.severity);
    }
    // No duplicates.
    for (const auto& a : codes::kRegister) {
        int n = 0;
        for (const auto& b : codes::kRegister) {
            if (a.code == b.code) ++n;
        }
        INFO("code " << a.code);
        REQUIRE(n == 1);
    }
}
