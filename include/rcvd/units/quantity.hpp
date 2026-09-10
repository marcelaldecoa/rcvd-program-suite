// rcvd/units/quantity.hpp — Rule U6, the compile-time dimensional type system.
//
// §3.1: "The public API SHOULD use a compile-time dimensional type system ... so
// that `Newton_per_radian + Newton` fails to compile."
//
// WHY ANGLE IS A TRACKED DIMENSION. In strict SI the radian is dimensionless, so
// a textbook dimensional analysis cannot tell N from N/rad. But every unit trap
// this specification warns about is an ANGLE trap: lbf/deg (Rule U5), lbf*ft/deg
// (U5), 1/deg and 1/(lbf*deg) (§5.2), the deg<->rad roll-stiffness error that
// §9.5's I-3c exists to catch. Treating angle as a fourth base dimension is what
// makes those errors type errors rather than silent factors of 57.
//
// The cost is that a genuinely dimensionless ratio formed from two angles needs an
// explicit strip; that is a small price for catching the class of defect that
// dominates this domain.
#ifndef RCVD_UNITS_QUANTITY_HPP
#define RCVD_UNITS_QUANTITY_HPP

#include <compare>
#include <string_view>
#include <type_traits>

#include "rcvd/units/constants.hpp"

namespace rcvd::units {

// Exponents of (length, mass, time, angle).
//
// Named Dims, not Dim: rcvd::units::Dim is already the enum naming the closed
// unit vocabulary in convert.hpp, and the two must coexist in one translation
// unit wherever io/csv.hpp meets models/bicycle.hpp.
template <int L, int M, int T, int A>
struct Dims {
    static constexpr int len = L, mass = M, time = T, angle = A;
};

template <class D1, class D2>
using DimMul = Dims<D1::len + D2::len, D1::mass + D2::mass, D1::time + D2::time,
                   D1::angle + D2::angle>;
template <class D1, class D2>
using DimDiv = Dims<D1::len - D2::len, D1::mass - D2::mass, D1::time - D2::time,
                   D1::angle - D2::angle>;

// A quantity is a double carrying its dimension. The stored value is ALWAYS SI
// (Rule U1): metre, kilogram, second, newton, radian.
template <class D>
class Quantity {
public:
    using Dimension = D;

    constexpr Quantity() = default;
    // Explicit: a bare double is not a quantity, which is the whole point.
    explicit constexpr Quantity(double si) : v_(si) {}

    // The SI magnitude. Named so that reaching for it is visible in review.
    [[nodiscard]] constexpr double si() const { return v_; }

    constexpr Quantity& operator+=(Quantity o) { v_ += o.v_; return *this; }
    constexpr Quantity& operator-=(Quantity o) { v_ -= o.v_; return *this; }
    constexpr Quantity& operator*=(double s) { v_ *= s; return *this; }
    constexpr Quantity& operator/=(double s) { v_ /= s; return *this; }

    friend constexpr Quantity operator+(Quantity a, Quantity b) { return Quantity{a.v_ + b.v_}; }
    friend constexpr Quantity operator-(Quantity a, Quantity b) { return Quantity{a.v_ - b.v_}; }
    friend constexpr Quantity operator-(Quantity a) { return Quantity{-a.v_}; }
    friend constexpr Quantity operator*(Quantity a, double s) { return Quantity{a.v_ * s}; }
    friend constexpr Quantity operator*(double s, Quantity a) { return Quantity{s * a.v_}; }
    friend constexpr Quantity operator/(Quantity a, double s) { return Quantity{a.v_ / s}; }

    friend constexpr auto operator<=>(Quantity a, Quantity b) = default;

private:
    double v_ = 0.0;
};

// Products and quotients combine exponents, so `Force * Length` is a Moment and
// `Force / Angle` is a cornering stiffness — without either being spelled out.
template <class D1, class D2>
[[nodiscard]] constexpr Quantity<DimMul<D1, D2>> operator*(Quantity<D1> a, Quantity<D2> b) {
    return Quantity<DimMul<D1, D2>>{a.si() * b.si()};
}
template <class D1, class D2>
[[nodiscard]] constexpr Quantity<DimDiv<D1, D2>> operator/(Quantity<D1> a, Quantity<D2> b) {
    return Quantity<DimDiv<D1, D2>>{a.si() / b.si()};
}
template <class D>
[[nodiscard]] constexpr Quantity<Dims<-D::len, -D::mass, -D::time, -D::angle>> operator/(
    double s, Quantity<D> a) {
    return Quantity<Dims<-D::len, -D::mass, -D::time, -D::angle>>{s / a.si()};
}

// ---------------------------------------------------------------- the vocabulary
using Scalar             = Quantity<Dims<0, 0, 0, 0>>;
using Length             = Quantity<Dims<1, 0, 0, 0>>;
using Mass               = Quantity<Dims<0, 1, 0, 0>>;
using Duration           = Quantity<Dims<0, 0, 1, 0>>;
using Angle              = Quantity<Dims<0, 0, 0, 1>>;
using Area               = Quantity<Dims<2, 0, 0, 0>>;
using Speed              = Quantity<Dims<1, 0, -1, 0>>;
using Acceleration       = Quantity<Dims<1, 0, -2, 0>>;
using Force              = Quantity<Dims<1, 1, -2, 0>>;
using Moment             = Quantity<Dims<2, 1, -2, 0>>;
using Inertia            = Quantity<Dims<2, 1, 0, 0>>;      // kg*m^2
using Density            = Quantity<Dims<-3, 1, 0, 0>>;
using AngularVelocity    = Quantity<Dims<0, 0, -1, 1>>;     // rad/s
using Stiffness          = Quantity<Dims<0, 1, -2, 0>>;     // N/m
// §3.4's derivatives. These are the four that trip people up.
using CorneringStiffness = Quantity<Dims<1, 1, -2, -1>>;    // N/rad      (Y_beta, C_alpha)
using AngularStiffness   = Quantity<Dims<2, 1, -2, -1>>;    // N*m/rad    (N_beta, K_phi)
using SideForceDamping   = Quantity<Dims<1, 1, -1, -1>>;    // N*s/rad    (Y_r)
using YawDamping         = Quantity<Dims<2, 1, -1, -1>>;    // N*m*s/rad  (N_r)
using PerForce           = Quantity<Dims<-1, -1, 2, 0>>;    // 1/N        (mu1)
using PerAngle           = Quantity<Dims<0, 0, 0, -1>>;     // 1/rad      (c0)
using PerForceAngle      = Quantity<Dims<-1, -1, 2, -1>>;   // 1/(N*rad)  (c1)

// ---------------------------------------------------------------- unit tags
//
// A Unit carries its dimension, so a conversion can only be applied to a quantity
// of the SAME dimension. This is what turns "converted N*s/rad with the lbf*ft/deg
// factor" from a plausible-looking line into a compile error.
template <class D>
struct Unit {
    double to_si;
    std::string_view name;
    using Dimension = D;
};

template <class D>
[[nodiscard]] constexpr Quantity<D> of(double value, Unit<D> u) {
    return Quantity<D>{value * u.to_si};
}
// Rule U2's display half. The dimensions must match or this does not compile.
template <class D>
[[nodiscard]] constexpr double in(Quantity<D> q, Unit<D> u) {
    return q.si() / u.to_si;
}

// SI
inline constexpr Unit<Dims<1, 0, 0, 0>>   metre{1.0, "m"};
inline constexpr Unit<Dims<0, 1, 0, 0>>   kilogram{1.0, "kg"};
inline constexpr Unit<Dims<0, 0, 1, 0>>   second{1.0, "s"};
inline constexpr Unit<Dims<0, 0, 0, 1>>   radian{1.0, "rad"};
inline constexpr Unit<Dims<1, 1, -2, 0>>  newton{1.0, "N"};
inline constexpr Unit<Dims<1, 0, -1, 0>>  metre_per_second{1.0, "m/s"};
inline constexpr Unit<Dims<1, 0, -2, 0>>  metre_per_second2{1.0, "m/s^2"};
inline constexpr Unit<Dims<2, 1, -2, 0>>  newton_metre{1.0, "N*m"};

// English, formed from the Rule U3 exact primitives — never transcribed decimals.
inline constexpr Unit<Dims<1, 0, 0, 0>>   foot{FOOT, "ft"};
inline constexpr Unit<Dims<1, 0, 0, 0>>   inch{INCH, "in"};
inline constexpr Unit<Dims<0, 1, 0, 0>>   slug{SLUG, "slug"};
inline constexpr Unit<Dims<0, 0, 0, 1>>   degree{DEGREE, "deg"};
inline constexpr Unit<Dims<1, 1, -2, 0>>  pound_force{POUND_FORCE, "lbf"};
inline constexpr Unit<Dims<1, 0, -1, 0>>  mile_per_hour{MPH, "mph"};
inline constexpr Unit<Dims<1, 0, -1, 0>>  foot_per_second{FOOT, "ft/s"};
inline constexpr Unit<Dims<2, 1, -2, 0>>  pound_foot{POUND_FORCE * FOOT, "lbf*ft"};
inline constexpr Unit<Dims<2, 1, 0, 0>>   slug_foot2{SLUG_FT2_TO_KG_M2, "slug*ft^2"};

// Rule U4: g is a FIELD CONSTANT, not a unit — but it is admissible as a
// PRESENTATION unit for acceleration (§3.4), and only there.
inline constexpr Unit<Dims<1, 0, -2, 0>>  gee{G_STANDARD, "g"};

// Rule U5 composites. Every one is formed from the primitives.
inline constexpr Unit<Dims<1, 1, -2, -1>> lbf_per_deg{LBF_PER_DEG_TO_N_PER_RAD, "lbf/deg"};
inline constexpr Unit<Dims<1, 1, -2, -1>> newton_per_rad{1.0, "N/rad"};
inline constexpr Unit<Dims<2, 1, -2, -1>> lbf_ft_per_deg{LBF_FT_PER_DEG_TO_NM_PER_RAD,
                                                        "lbf*ft/deg"};
inline constexpr Unit<Dims<2, 1, -2, -1>> newton_metre_per_rad{1.0, "N*m/rad"};
inline constexpr Unit<Dims<0, 1, -2, 0>>  lbf_per_in{LBF_PER_IN_TO_N_PER_M, "lbf/in"};
inline constexpr Unit<Dims<0, 1, -2, 0>>  newton_per_metre{1.0, "N/m"};
// Y_r and N_r: the two whose units are most often mis-stated, now distinct types.
inline constexpr Unit<Dims<1, 1, -1, -1>> lbf_s_per_deg{LBF_PER_DEG_TO_N_PER_RAD, "lbf*s/deg"};
inline constexpr Unit<Dims<2, 1, -1, -1>> lbf_ft_s_per_deg{LBF_FT_PER_DEG_TO_NM_PER_RAD,
                                                          "lbf*ft*s/deg"};

// The field constant, typed.
inline constexpr Acceleration g_standard{G_STANDARD};

// An angle ratio is genuinely dimensionless, but stripping the angle has to be
// deliberate and visible — hence a named function rather than an implicit
// conversion.
[[nodiscard]] constexpr Scalar strip_angle(Quantity<Dims<0, 0, 0, 0>> q) { return q; }
template <class D>
[[nodiscard]] constexpr double raw(Quantity<D> q) { return q.si(); }

}  // namespace rcvd::units

#endif  // RCVD_UNITS_QUANTITY_HPP
