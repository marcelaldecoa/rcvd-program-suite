// rcvd/io/display.hpp — Rule U2's display boundary, made selectable.
//
// §3.1 U1: the core computes in strict SI, always. U2: conversion happens exactly
// twice per data path — once on deserialisation (io/document.hpp) and once here.
// Nothing between those two boundaries knows what a foot is.
//
// This header is the "once here" half. Choosing US customary output changes only
// what the last conversion does; it cannot change a single computed value, which
// is exactly the property Rule U2 exists to guarantee.
#ifndef RCVD_IO_DISPLAY_HPP
#define RCVD_IO_DISPLAY_HPP

#include <string>
#include <string_view>

#include "rcvd/units/convert.hpp"

namespace rcvd::io {

enum class UnitSystem {
    si,  // metre, kilogram, second, newton, radian
    us,  // foot, slug, pound-force, degree — and g for acceleration (§3.4)
};

[[nodiscard]] inline std::string_view to_string(UnitSystem s) {
    return s == UnitSystem::us ? "us" : "si";
}

[[nodiscard]] inline std::optional<UnitSystem> parse_unit_system(std::string_view s) {
    if (s == "si" || s == "SI" || s == "metric") return UnitSystem::si;
    if (s == "us" || s == "US" || s == "english" || s == "imperial") return UnitSystem::us;
    return std::nullopt;
}

// Which unit each dimension is displayed in. A UnitSystem picks the defaults; any
// individual channel may still override, because "American format" is not one
// convention — §17.2 quotes speeds in both ft/s and mph, and lengths in both ft
// and in, on the same page.
struct DisplayUnits {
    UnitSystem system = UnitSystem::si;
    // Per-dimension overrides; empty means "use the system default".
    std::string length, mass, force, angle, speed, acceleration, moment, inertia;
    std::string stiffness, angular_stiffness, cornering_stiffness, time, area, density;
    std::string side_force_damping, yaw_damping;

    [[nodiscard]] static DisplayUnits of(UnitSystem s) {
        DisplayUnits d;
        d.system = s;
        return d;
    }

    // §3.4: "A_Y and A_X are reported to the user in g, and C_N is dimensionless."
    // That holds in BOTH systems — g is the conventional presentation unit for
    // acceleration in vehicle dynamics regardless of the rest of the choice, and
    // Rule U4 permits it here precisely because this is the presentation layer.
    [[nodiscard]] std::string_view unit_for(units::Dim dim) const {
        const bool us = system == UnitSystem::us;
        switch (dim) {
            case units::Dim::Length:             return pick(length, us ? "ft" : "m");
            case units::Dim::Mass:               return pick(mass, us ? "slug" : "kg");
            case units::Dim::Force:              return pick(force, us ? "lbf" : "N");
            case units::Dim::Angle:              return pick(angle, us ? "deg" : "rad");
            case units::Dim::Speed:              return pick(speed, us ? "mph" : "m/s");
            case units::Dim::Acceleration:       return pick(acceleration, "g");
            case units::Dim::Moment:             return pick(moment, us ? "lbf*ft" : "N*m");
            case units::Dim::Inertia:            return pick(inertia, us ? "slug*ft^2" : "kg*m^2");
            case units::Dim::Stiffness:          return pick(stiffness, us ? "lbf/in" : "N/m");
            case units::Dim::AngularStiffness:
                return pick(angular_stiffness, us ? "lbf*ft/deg" : "N*m/rad");
            case units::Dim::CorneringStiffness:
                return pick(cornering_stiffness, us ? "lbf/deg" : "N/rad");
            case units::Dim::SideForceDamping:
                return pick(side_force_damping, us ? "lbf*s/deg" : "N*s/rad");
            case units::Dim::YawDamping:
                return pick(yaw_damping, us ? "lbf*ft*s/deg" : "N*m*s/rad");
            case units::Dim::Time:               return pick(time, "s");
            case units::Dim::Area:               return pick(area, us ? "ft^2" : "m^2");
            case units::Dim::Density:            return pick(density, "kg/m^3");
            // Dimensions with no US customary spelling in the closed vocabulary
            // fall back to SI rather than inventing one. §5's vocabulary is closed
            // on purpose: an unrecognised unit must be an error, not a guess.
            case units::Dim::PerForce:           return "1/N";
            case units::Dim::PerAngle:           return pick(angle.empty() ? "" : "", us ? "1/deg" : "1/rad");
            case units::Dim::PerForceAngle:      return us ? "1/(lbf*deg)" : "1/(N*rad)";
            case units::Dim::Dimensionless:      return "-";
        }
        return "-";
    }

private:
    static std::string_view pick(const std::string& override_unit, std::string_view fallback) {
        return override_unit.empty() ? fallback : std::string_view(override_unit);
    }
};

// The single conversion. Returns nullopt only if a channel asked for a unit that
// is not in the closed vocabulary for its dimension, which is E-IO-004.
[[nodiscard]] inline std::optional<double> to_display(double si_value, units::Dim dim,
                                                      const DisplayUnits& d) {
    return units::from_si(si_value, d.unit_for(dim), dim);
}

}  // namespace rcvd::io

#endif  // RCVD_IO_DISPLAY_HPP
