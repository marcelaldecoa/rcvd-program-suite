// rcvd/core/diagnostic.hpp — the E-/W-/EV-/H- register of spec §14.2.
#ifndef RCVD_CORE_DIAGNOSTIC_HPP
#define RCVD_CORE_DIAGNOSTIC_HPP

#include <optional>
#include <string>
#include <string_view>

namespace rcvd::core {

// §14.1 policy. Only Error produces a failed Result (§4.6 Rule A3); Warning,
// Event and Hint travel alongside a successful one.
enum class Severity { Error, Warning, Event, Hint };

enum class Wheel { LF, RF, LR, RR };

[[nodiscard]] constexpr std::string_view to_string(Wheel w) noexcept {
    switch (w) {
        case Wheel::LF: return "LF";
        case Wheel::RF: return "RF";
        case Wheel::LR: return "LR";
        case Wheel::RR: return "RR";
    }
    return "??";
}

struct Diagnostic {
    std::string_view      code;      // "E-SS-030"; must appear in the §14.2 register
    Severity              severity{Severity::Error};
    std::string           message;
    std::optional<double> t_s;       // when, if raised during a run
    std::optional<Wheel>  wheel;     // where, if wheel-specific
};

// §14.2 register. Kept as one table so that Appendix D's "every warning and error
// code has a test that provokes it" is mechanically checkable: the test suite
// iterates this array and asserts each code was raised at least once.
namespace codes {

struct Entry {
    std::string_view code;
    Severity         severity;
    std::string_view meaning;
};

inline constexpr Entry kRegister[] = {
    {"E-VEH-010",  Severity::Error,   "Over-specified geometry, inertia or roll-stiffness group"},
    {"E-VEH-011",  Severity::Error,   "Sprung/unsprung consistency (5.1a)-(5.1c) violated"},
    {"E-VEH-140",  Severity::Error,   "Roll divergence: K_phiF + K_phiR <= 1.05*m_s g h2"},
    {"E-TIRE-003", Severity::Error,   "Positive cornering stiffness supplied"},
    {"E-TIRE-004", Severity::Error,   "Quadratic C_alpha(Fz) changes sign in the load range"},
    {"E-TIRE-005", Severity::Error,   "Two independent sources of cornering stiffness"},
    {"E-TIRE-006", Severity::Error,   "Friction block for a model with has_mu() == false"},
    {"E-TIRE-020", Severity::Error,   "Load outside tabulated range by more than 25 %"},
    {"E-SS-030",   Severity::Error,   "Steady-state system singular (at or above V_crit)"},
    {"E-P2-050",   Severity::Error,   "Load-transfer fixed point failed to converge"},
    {"E-RR-010",   Severity::Error,   "Desired ride rate >= tire rate"},
    {"E-RR-020",   Severity::Error,   "Desired roll stiffness >= tire roll stiffness"},
    {"E-NUM-300",  Severity::Error,   "Root bracket contains no sign change"},
    {"E-NUM-310",  Severity::Error,   "Speed below floor in a speed-normalised expression"},
    {"W-TIRE-101", Severity::Warning, "Load beyond the maximum of the quadratic C_alpha(Fz)"},
    {"W-VEH-120",  Severity::Warning, "Roll-axis inclination beyond the small-angle assumption"},
    {"W-VEH-141",  Severity::Warning, "Roll gradient outside the (9.10c) band - suspect K_phi units"},
    {"W-NUM-200",  Severity::Warning, "Integration step too large for the fastest mode present"},
    {"W-NUM-210",  Severity::Warning, "Fixed-step integrator with a C0 tire model"},
    {"W-NUM-220",  Severity::Warning, "small_angle_slip forced on beyond small_angle_limit"},
    {"W-RR-011",   Severity::Warning, "Desired ride rate above 80 % of tire rate"},
    {"W-RR-021",   Severity::Warning, "Required anti-roll bar rate is negative"},
    {"EV-LIFT",    Severity::Event,   "Wheel load reached zero"},
    {"EV-SPIN",    Severity::Event,   "Vehicle sideslip exceeded 90 deg"},
    {"EV-WHEEL-STALL", Severity::Event, "Wheel longitudinal velocity below the speed floor"},
    {"EV-LAP",     Severity::Event,   "Lap completed"},
    {"EV-WALL-IN", Severity::Event,   "Contact with the inner boundary"},
    {"EV-WALL-OUT",Severity::Event,   "Departure past the outer boundary"},
    {"EV-TIMEOUT", Severity::Event,   "Calculation length reached without lap completion"},
    {"EV-CLAMP-MU",Severity::Event,   "Friction coefficient clamped to mu_min"},
    {"EV-CLAMP-ELLIPSE", Severity::Event, "Friction-ellipse radicand clamped to zero"},
    {"H-SPD-001",  Severity::Hint,    "Inner-wall contact attributable to the initial yaw transient"},
};

[[nodiscard]] constexpr std::optional<Entry> find(std::string_view code) noexcept {
    for (const Entry& e : kRegister) {
        if (e.code == code) return e;
    }
    return std::nullopt;
}

}  // namespace codes

// Build a Diagnostic from a registered code, taking severity from the register so
// that a caller cannot raise "E-SS-030" as a warning.
[[nodiscard]] inline Diagnostic make_diagnostic(std::string_view code, std::string message,
                                                std::optional<double> t_s = std::nullopt,
                                                std::optional<Wheel> wheel = std::nullopt) {
    const std::optional<codes::Entry> e = codes::find(code);
    Diagnostic d;
    d.code     = e ? e->code : std::string_view{"E-INTERNAL"};
    d.severity = e ? e->severity : Severity::Error;
    d.message  = std::move(message);
    d.t_s      = t_s;
    d.wheel    = wheel;
    return d;
}

}  // namespace rcvd::core

#endif  // RCVD_CORE_DIAGNOSTIC_HPP
