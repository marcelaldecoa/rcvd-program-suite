// rcvd/units/convert.hpp — the closed unit vocabulary and the two-point
// conversion boundary, spec §3.1 Rules U1/U2 and §5 (unit strings).
//
// Rule U1: the core computes in strict SI. Rule U2: conversion happens exactly
// twice per data path — once on deserialisation, once on display. This header is
// the only place permitted to know about feet, inches, pounds, degrees or mph.
#ifndef RCVD_UNITS_CONVERT_HPP
#define RCVD_UNITS_CONVERT_HPP

#include <optional>
#include <string_view>

#include "rcvd/units/constants.hpp"

namespace rcvd::units {

// Physical dimension of a quantity. Conversion is only legal within a dimension;
// this is what makes "Newton_per_radian + Newton" a load-time error (§3.1 U6)
// even for the raw-double path.
enum class Dim {
    Dimensionless,
    Length,
    Mass,
    Force,
    Angle,
    Speed,
    Area,
    Inertia,            // kg*m^2
    Stiffness,          // N/m          (spring / tire rate)
    AngularStiffness,   // N*m/rad      (roll stiffness)
    CorneringStiffness, // N/rad
    SideForceDamping,   // N*s/rad     (Y_r)
    YawDamping,         // N*m*s/rad   (N_r)
    Moment,             // N*m
    Time,
    Density,            // kg/m^3
    Acceleration,       // m/s^2
    PerForce,           // 1/N          (mu1 of (6.9))
    PerAngle,           // 1/rad        (c0 of (6.10))
    PerForceAngle,      // 1/(N*rad)    (c1 of (6.10))
};

struct UnitEntry {
    std::string_view name;
    Dim              dim;
    double           to_si;  // multiply a value in this unit to obtain SI
};

// The closed vocabulary (§5). A unit string outside this table is rejected at
// parse time; there is no fallback and no silent pass-through.
inline constexpr UnitEntry kVocabulary[] = {
    {"-",             Dim::Dimensionless,      1.0},
    {"m",             Dim::Length,             1.0},
    {"ft",            Dim::Length,             FOOT},
    {"in",            Dim::Length,             INCH},
    {"kg",            Dim::Mass,               1.0},
    {"slug",          Dim::Mass,               SLUG},
    {"N",             Dim::Force,              1.0},
    {"lbf",           Dim::Force,              POUND_FORCE},
    {"rad",           Dim::Angle,              1.0},
    {"deg",           Dim::Angle,              DEGREE},
    {"m/s",           Dim::Speed,              1.0},
    {"ft/s",          Dim::Speed,              FOOT},
    {"mph",           Dim::Speed,              MPH},
    {"m^2",           Dim::Area,               1.0},
    {"ft^2",          Dim::Area,               FOOT * FOOT},
    {"kg*m^2",        Dim::Inertia,            1.0},
    {"slug*ft^2",     Dim::Inertia,            SLUG_FT2_TO_KG_M2},
    {"N/m",           Dim::Stiffness,          1.0},
    {"lbf/in",        Dim::Stiffness,          LBF_PER_IN_TO_N_PER_M},
    {"lbf/ft",        Dim::Stiffness,          POUND_FORCE / FOOT},
    {"N*m/rad",       Dim::AngularStiffness,   1.0},
    {"lbf*ft/deg",    Dim::AngularStiffness,   LBF_FT_PER_DEG_TO_NM_PER_RAD},
    {"lbf*ft/rad",    Dim::AngularStiffness,   POUND_FORCE * FOOT},
    {"N/rad",         Dim::CorneringStiffness, 1.0},
    {"lbf/deg",       Dim::CorneringStiffness, LBF_PER_DEG_TO_N_PER_RAD},
    {"lbf/rad",       Dim::CorneringStiffness, POUND_FORCE},
    // Y_r and N_r. Distinct dimensions from the stiffnesses they are so often
    // displayed with; giving them their own entries is what lets the US display
    // path express B-04 and B-07 at all.
    {"N*s/rad",       Dim::SideForceDamping,   1.0},
    {"lbf*s/deg",     Dim::SideForceDamping,   LBF_PER_DEG_TO_N_PER_RAD},
    {"N*m*s/rad",     Dim::YawDamping,         1.0},
    {"lbf*ft*s/deg",  Dim::YawDamping,         LBF_FT_PER_DEG_TO_NM_PER_RAD},
    {"N*m",           Dim::Moment,             1.0},
    {"lbf*ft",        Dim::Moment,             POUND_FORCE * FOOT},
    {"s",             Dim::Time,               1.0},
    {"kg/m^3",        Dim::Density,            1.0},
    {"m/s^2",         Dim::Acceleration,       1.0},
    {"ft/s^2",        Dim::Acceleration,       FOOT},
    // "g" is admissible ONLY as a presentation unit for acceleration (§3.4). It is
    // a field constant, not a unit (Rule U4); the core never sees it.
    {"g",             Dim::Acceleration,       G_STANDARD},
    {"1/N",           Dim::PerForce,           1.0},
    {"1/lbf",         Dim::PerForce,           1.0 / POUND_FORCE},
    {"1/rad",         Dim::PerAngle,           1.0},
    {"1/deg",         Dim::PerAngle,           1.0 / DEGREE},
    {"1/(N*rad)",     Dim::PerForceAngle,      1.0},
    {"1/(lbf*deg)",   Dim::PerForceAngle,      1.0 / (POUND_FORCE * DEGREE)},
};

[[nodiscard]] constexpr std::optional<UnitEntry> lookup(std::string_view unit) noexcept {
    for (const UnitEntry& e : kVocabulary) {
        if (e.name == unit) return e;
    }
    return std::nullopt;
}

// Convert `value` given in `unit` to SI. Returns nullopt if the unit is not in the
// vocabulary, or if it is but belongs to a different dimension than `expected`.
[[nodiscard]] constexpr std::optional<double> to_si(double value, std::string_view unit,
                                                    Dim expected) noexcept {
    const std::optional<UnitEntry> e = lookup(unit);
    if (!e || e->dim != expected) return std::nullopt;
    return value * e->to_si;
}

// The display half of Rule U2.
[[nodiscard]] constexpr std::optional<double> from_si(double si, std::string_view unit,
                                                      Dim expected) noexcept {
    const std::optional<UnitEntry> e = lookup(unit);
    if (!e || e->dim != expected) return std::nullopt;
    return si / e->to_si;
}

}  // namespace rcvd::units

#endif  // RCVD_UNITS_CONVERT_HPP
