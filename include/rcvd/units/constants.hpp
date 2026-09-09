// rcvd/units/constants.hpp — exact conversion constants, spec §3.1 Rules U3–U5.
//
// Rule U3: these are the exact defined values, stored as constexpr double and
// never re-derived at runtime. Rule U5: composite factors (lb/deg, lb-ft/deg)
// SHALL be formed from the four exact primitives, never hard-coded as decimals.
#ifndef RCVD_UNITS_CONSTANTS_HPP
#define RCVD_UNITS_CONSTANTS_HPP

namespace rcvd::units {

// ---- exact primitives (§3.1 Rule U3) ---------------------------------------
inline constexpr double FOOT        = 0.3048;            // m, exact by definition
inline constexpr double INCH        = 0.0254;            // m, exact
inline constexpr double POUND_FORCE = 4.4482216152605;   // N, exact
// Derived at compile time from the two primitives above rather than transcribed.
// §3.1 lists it as 14.5939029372064 kg, but that decimal is the true quotient
// rounded to 15 figures and differs from it by 2.6e-15 relative — enough to break
// an exact round-trip assertion. Rule U3 forbids re-deriving conversion factors at
// *runtime*; a constexpr quotient is not that, and Rule U5's principle (form
// composites from the primitives, never hard-code the decimal) applies here too.
inline constexpr double SLUG        = POUND_FORCE / FOOT;  // kg
inline constexpr double MPH         = 0.44704;           // m/s, exact

// pi to binary64. Not <numbers> so the header stays usable pre-C++20 in capi/.
inline constexpr double PI          = 3.14159265358979323846;
inline constexpr double DEGREE      = PI / 180.0;        // rad

// §3.1 Rule U4: g is a FIELD CONSTANT, not a unit. Legacy vectors computed with
// 32.174 or 32.2 ft/s^2 have the discrepancy accounted for in the test tolerance
// (§17.1), never absorbed by changing this value.
inline constexpr double G_STANDARD  = 9.80665;           // m/s^2, exact, ISO 80000-3

// ---- composites, formed from the primitives (§3.1 Rule U5) -----------------
// Cornering stiffness: 1 lbf/deg = 254.864324893 N/rad.
inline constexpr double LBF_PER_DEG_TO_N_PER_RAD = POUND_FORCE / DEGREE;
// Roll stiffness: 1 lbf*ft/deg = 77.6826 N*m/rad.
inline constexpr double LBF_FT_PER_DEG_TO_NM_PER_RAD = POUND_FORCE * FOOT / DEGREE;
// Spring rate: 1 lbf/in -> N/m.
inline constexpr double LBF_PER_IN_TO_N_PER_M = POUND_FORCE / INCH;
// Inertia: 1 slug*ft^2 -> kg*m^2.
inline constexpr double SLUG_FT2_TO_KG_M2 = SLUG * FOOT * FOOT;

// §10.5 E-4: the legacy English Ackermann-gradient constant, retained only so the
// unit layer can DEMONSTRATE that 858 is 0.120 % high. Solvers use l/V^2 in SI and
// SHALL NOT reference this.
inline constexpr double LEGACY_ACKERMANN_CONSTANT_EXACT =
    (1.0 / DEGREE) * (G_STANDARD / FOOT) / ((MPH / FOOT) * (MPH / FOOT));  // 856.970

// Guard the derived SLUG against the published decimal, so a future edit to either
// primitive that breaks the documented value fails the build rather than the suite.
static_assert(SLUG > 14.593902937206 && SLUG < 14.593902937207,
              "SLUG must agree with the §3.1 published value 14.5939029372064 kg");

}  // namespace rcvd::units

#endif  // RCVD_UNITS_CONSTANTS_HPP
