# Race Car Vehicle Dynamics Program Suite — Software Specification

**Document ID:** RCVD-SUITE-SPEC
**Revision:** 1.0
**Date:** 2026-09-09
**Status:** Baseline for implementation

---

## 0. Document Control

### 0.1 Purpose

This document specifies, to implementation depth, a suite of six vehicle-dynamics
programs functionally equivalent to — and mathematically stricter than — the *Race Car
Vehicle Dynamics Program Suite* distributed with Milliken, Kasprzak, Metz & Milliken,
*Race Car Vehicle Dynamics: Problems, Answers and Experiments* (SAE R-280, 2003).

The specification is **normative on the mathematics**. Every governing equation, sign
convention, discretisation and tolerance is stated explicitly so that two independent
implementations must agree to the tolerances in §17. Architecture and API are normative
on structure and contract, permissive on style.

### 0.2 Conformance language

- **SHALL / MUST** — mandatory. A non-conforming implementation fails acceptance.
- **SHOULD** — strongly recommended; deviation requires written justification.
- **MAY** — permitted.

### 0.3 Deliverable summary

| # | Program | Model | Primary output |
|---|---------|-------|----------------|
| P1 | Bicycle Model | Linear 2-DOF (yaw + sideslip) | Time history, steady state, stability derivatives, eigenstructure |
| P2 | Four-Wheeler | Nonlinear 3-DOF + quasi-static roll/load transfer | Time history, 25 output channels incl. four wheel loads |
| P3 | Moment Method (MMM) | Constrained force–moment analysis | C_N–A_Y diagram, trim/stability/control metrics |
| P4 | RCVD Speedway | P1 on a constant-radius circuit | Lap time, real-time path plot |
| P5 | RCVD Speedway 2 | P2 on a constant-radius circuit | Lap time, path, wheel-load history |
| P6 | Ride & Roll Rate Calculator | Single-track springing algebra | Wheel-centre rates, ARB rates, roll stiffnesses |

### 0.4 Intellectual-property note

The equations of motion, tire relations and load-transfer algebra below are standard
engineering mechanics and are re-derived here from first principles. Published *numerical
answers* from SAE R-280 are used only as acceptance test vectors and are cited by page.
No proprietary MRA source code, no CD-borne User's Manual text, and no copyrighted tire
data files are reproduced. Any tire data shipped with the product SHALL be either
synthetic, public-domain, or licensed.

---

## 1. Scope

### 1.1 In scope

Steady-state and transient handling of a road/race vehicle on a flat or constant-tilt
surface, in the linear and mildly-nonlinear regimes, with:

- pneumatic tire lateral force and aligning moment as a function of slip angle and
  vertical load (five interchangeable tire models, §6);
- quasi-static lateral and longitudinal load transfer with roll-stiffness distribution
  and roll-centre geometry (§9.3);
- aerodynamic drag and front/rear downforce (§7);
- open-loop driver inputs (fixed, step, ramp, sinusoid, table) (§5.6);
- ride and roll rate synthesis in both directions (§12).

### 1.2 Out of scope for Rev 1.0

Ride/heave/pitch dynamics as a dynamic (rather than algebraic) subsystem; dampers;
kinematic suspension solvers; compliances; steering-system dynamics; driveline;
combined-slip lap-time optimisation over an arbitrary circuit; multibody. §20 defines the
extension seams that keep these additions from forcing an architectural rewrite.

---

## 2. Normative References

| Ref | Document |
|-----|----------|
| [RCVD] | Milliken & Milliken, *Race Car Vehicle Dynamics*, SAE R-146, 1995 |
| [R280] | Milliken, Kasprzak, Metz & Milliken, *RCVD: Problems, Answers and Experiments*, SAE R-280, 2003 |
| [J670] | SAE J670e, *Vehicle Dynamics Terminology*, July 1976 |
| [J1594] | SAE J1594, *Vehicle Aerodynamics Terminology*, June 1987 |
| [SEGEL] | L. Segel, *Theoretical Prediction and Experimental Substantiation of the Response of the Automobile to Steering Control*, IMechE, 1956–57 |
| [RADT] | H. S. Radt, non-dimensional tire data treatment, as summarised in [RCVD] Ch. 14 |
| [PAC] | H. B. Pacejka, *Tyre and Vehicle Dynamics*, 3rd ed. |
| [SAE760030] | Weber & Persch, *Frequency Response of Tires: Slip Angle and Lateral Force*, SAE 760030 |
| [ISO80000] | ISO 80000-1/-3/-4, Quantities and units |
| [IEEE754] | IEEE 754-2019, binary64 floating point |

---

## 3. Conventions

### 3.1 Units — the dual-unit contract

**Rule U1 (normative).** All computation inside the core library SHALL be performed in
**strict SI base and coherent derived units**: metre, kilogram, second, newton, radian,
pascal. No English unit, no degree, and no "g" ever appears inside a solver.

**Rule U2.** Unit conversion SHALL occur exactly twice per data path: once on
deserialisation/input, once on serialisation/display. The conversion layer is the only
component permitted to know about feet, inches, pounds, slugs, degrees, mph or g.

**Rule U3.** Conversion factors SHALL be the exact defined values below, stored as
`constexpr double`, never re-derived at runtime:

```
FOOT              = 0.3048                m           (exact, by definition)
INCH              = 0.0254                m           (exact)
POUND_FORCE       = 4.4482216152605       N           (exact)
SLUG              = 14.5939029372064      kg          (POUND_FORCE / FOOT)
DEGREE            = pi/180                rad
MPH               = 0.44704               m/s         (exact)
G_STANDARD        = 9.80665               m/s^2       (exact, ISO 80000-3)
```

**Rule U4 — gravity.** `g` is a *field constant* `G_STANDARD = 9.80665 m/s^2`, not a unit.
Standard gravity is exactly `32.17404855643…  ft/s²`. Legacy test vectors computed with
the rounded `32.174 ft/s²` (= 9.8066352 m/s², low by 1.5e−6 relative) or `32.2 ft/s²`
(= 9.81456 m/s², high by 8.1e−4 relative) SHALL have the discrepancy accounted for in the
test tolerance and recorded in the test-vector table (§17.2), never absorbed by changing
the constant.

**Rule U5 — derived-unit traps.** Cornering stiffness in the source literature is quoted
in `lb/deg`. The conversion is

```
1 lb/deg  =  POUND_FORCE / DEGREE  =  4.4482216152605 / (pi/180)  =  254.864324893 N/rad
```

Roll stiffness in `lb-ft/deg` converts as

```
1 lb-ft/deg = POUND_FORCE * FOOT / DEGREE = 77.6826 N.m/rad
```

Implementations SHALL NOT hard-code decimal approximations of these composites; they SHALL
be formed from the four exact primitives above.

**Rule U6 — typed quantities.** The public API SHOULD use a compile-time dimensional type
system (a header-only strong-typedef over `double`, or `mp-units`) so that
`Newton_per_radian + Newton` fails to compile. Where raw `double` is used at an interface,
the parameter name SHALL carry the SI unit as a suffix (`cornering_stiffness_N_per_rad`).

### 3.2 Axis systems

Four right-handed frames are defined, all following [J670].

**F1 — Earth-fixed (inertial) frame `{X_E, Y_E, Z_E}`.** Origin at the simulation start
point. `X_E` horizontal, `Y_E` horizontal 90° clockwise from `X_E` viewed from above,
`Z_E` vertically **downward**. Gravity is therefore `+g` along `Z_E`.

**F2 — SAE vehicle frame `{x, y, z}`.** Origin at the total-vehicle CG. `x` positive
forward, `y` positive to the driver's right, `z` positive downward. Angular rates: roll
`p` about `x`, pitch `q` about `y`, yaw `r` about `z`, all positive by the right-hand rule.
Positive roll `φ` therefore puts the **right** side down.

**F3 — SAE tire frame (one per wheel).** Origin at the intersection of the wheel plane
with the ground plane. `x_t` forward in the wheel plane projected on the ground, `y_t` to
the right, `z_t` downward. Tire vertical load acting **up** on the tire is therefore a
**negative** `F_z` in this frame.

> **Convention decision D-1 (normative).** Because a negative-signed vertical load is a
> perennial source of implementation error, the core library SHALL carry vertical load as
> a **non-negative magnitude** `Fz >= 0` in newtons in all tire-model interfaces, and
> perform the sign conversion in the wheel-frame adapter. The tire-model contract in §6.1
> is stated in terms of this magnitude. `Fz = 0` denotes a lifted wheel.

**F4 — SAE aerodynamic frame.** Origin at ground level, at the mid-point of wheelbase and
track. Axes parallel to F2.

**Euler sequence.** Orientation of F2 relative to F1 is given by the ordered rotation
**yaw ψ, then pitch θ, then roll φ**. The transformation from body to earth is

```
        | cosθcosψ   sinφsinθcosψ - cosφsinψ   cosφsinθcosψ + sinφsinψ |
A_EB =  | cosθsinψ   sinφsinθsinψ + cosφcosψ   cosφsinθsinψ - sinφcosψ |
        | -sinθ      sinφcosθ                  cosφcosθ                |
```

so that `{x,y,z}_E = A_EB · {x,y,z}_B`. Rotations do not commute; the implementation SHALL
apply this sequence and SHALL provide a unit test demonstrating that the reversed sequence
yields a different orientation ([R280] Fig. 4.1 vs 4.3).

`A_EB` is orthonormal; the inverse SHALL be computed as the transpose, and a runtime
assertion SHALL check `‖A_EB^T A_EB − I‖_F < 1e-12`.

### 3.3 Sign-convention table (normative)

For a **modestly understeering vehicle in a steady right-hand turn** the following signs
SHALL be produced. This table is an acceptance test, not commentary ([R280] p. 49).

| Quantity | Symbol | Sign |
|---|---|---|
| Longitudinal velocity | u | + |
| Lateral velocity | v | same sign as β (see below) |
| Yaw rate | r | + |
| Longitudinal acceleration | A_X | 0 (constant-speed steady state) |
| Lateral acceleration | A_Y | + |
| Tire longitudinal force | F_x | + (overcoming induced drag) |
| Tire lateral force | F_y | + |
| Vehicle yaw moment | M_z | 0 (steady state) |
| Steer angle | δ | + |
| Tire slip angle | α | − |
| Vehicle sideslip | β | + below tangent speed, − above |
| — | — | (`β = arctan(v/u)` with `u > 0`, so `v` and `β` always share a sign; the source's unconditional "`v` positive" is correct only below the tangent speed) |
| Body roll angle | φ | − (leans left, i.e. outward) |
| Cornering stiffness | C_α | − (always) |

**Slip angle definition (normative).**

```
α_F = β + a·r/u − δ            α_R = β − b·r/u                    (3.1)
```

with `β = arctan(v/u)`. **The arctangent form is normative and unconditional.** There is
no automatic switch to the small-angle form: an implementation that silently changes its
force path at a threshold cannot be bit-reproducible across a manoeuvre that crosses it
(Rule A2), and the crossing would land inside an integrator stage.

The small-angle form `β ≈ v/u` is available only through the explicit
`SolverConfig.small_angle_slip` switch (§5.5), which forces it unconditionally. That
switch exists for one purpose: bicycle-equivalence mode (C-02, §17.8), where P2 must
reproduce P1's linearised (8.3) exactly. When it is set and `|v/u|` exceeds
`small_angle_limit` (default 0.15 rad), `W-NUM-220` SHALL be emitted with the maximum
excursion; the force path SHALL NOT change in response. `small_angle_limit` is therefore
a diagnostic threshold, never a branch condition.

**Cornering stiffness sign.** `C_α ≡ ∂F_y/∂α |_{α=0} < 0`. Lateral force is
`F_y = C_α·α` in the linear range. Implementations SHALL reject a positive `C_α` at input
validation with error `E-TIRE-003`.

### 3.4 Nomenclature

All symbols are SI unless a unit is stated.

| Symbol | Quantity | SI unit |
|---|---|---|
| m | total vehicle mass | kg |
| m_s, m_u | sprung, unsprung mass | kg |
| W = m·g | total weight | N |
| I_z, I_x, I_y | yaw, roll, pitch inertia about CG | kg·m² |
| I_xz | yaw–roll product of inertia | kg·m² |
| ℓ | wheelbase | m |
| a, b | CG to front axle, CG to rear axle (a + b = ℓ) | m |
| a_s | front axle to **sprung**-mass CG | m |
| t_F, t_R | front, rear **wheel** track | m |
| t_sF, t_sR | front, rear **spring** track | m |
| h | total CG height above ground | m |
| h_s | sprung CG height above ground | m |
| h₂ | perpendicular distance, sprung CG to roll axis | m |
| z_RF, z_RR | front, rear roll-centre height | m |
| z_WF, z_WR | front, rear unsprung CG height | m |
| u, v | body-frame longitudinal, lateral velocity | m/s |
| V | speed magnitude, √(u²+v²) | m/s |
| r | yaw rate | rad/s |
| β | vehicle sideslip angle | rad |
| φ | body roll angle | rad |
| δ | road-wheel steer angle | rad |
| δ_SW | steering-wheel angle | rad |
| G_s | overall steering ratio, δ_SW/δ | – |
| α | tire slip angle | rad |
| C_αF, C_αR | axle cornering stiffness (negative) | N/rad |
| F_z | tire vertical load (magnitude, D-1) | N |
| F_y, F_x | tire lateral, longitudinal force | N |
| M_z | tire aligning moment | N·m |
| t_p | pneumatic trail | m |
| μ | peak friction coefficient | – |
| σ | tire relaxation length | m |
| k_T | tire vertical rate | N/m |
| k_W | wheel-centre (ride) spring rate | N/m |
| k_R | ride rate, chassis-to-ground | N/m |
| K_φF, K_φR | axle roll stiffness | N·m/rad |
| K_φaux | auxiliary (anti-roll bar) roll stiffness | N·m/rad |
| RG | roll gradient | rad per (m/s²) |
| ρ | air density | kg/m³ |
| A_f | frontal reference area | m² |
| C_D | drag coefficient | – |
| C_LF, C_LR | front, rear lift coefficient (negative ⇒ downforce) | – |
| A_Y, A_X | lateral, longitudinal acceleration | m/s² |
| C_N | yaw-moment coefficient, N/(W·ℓ) | – |
| K | stability factor | s²/m² |
| K_us | understeer gradient | rad per (m/s²) |
| V_ch, V_crit, V_tan | characteristic, critical, tangent speed | m/s |

**Normalised presentation quantities.** `A_Y` and `A_X` are reported to the user in **g**
(divide by `G_STANDARD`) and `C_N` is dimensionless. The core returns SI; the presentation
layer normalises.

---

## 4. Architecture

### 4.1 Decomposition

```
rcvd/
├── units/        header-only strong-typed quantities + exact conversion constants
├── core/         Vehicle, Tire, Aero, Environment, Maneuver, Result value types
│                 validation, JSON (de)serialisation, schema versioning
├── tire/         TireModel interface + 5 concrete models + relaxation filter
├── aero/         AeroModel interface + quadratic model
├── numerics/     integrators, root finders, dense linear algebra, eigen, interpolation
├── models/       BicycleModel, FourWheelerModel  (plant definitions)
├── analysis/     SteadyState, StabilityDerivatives, Eigenstructure, MomentMethod,
│                 RideRollCalculator, LapSolver
├── io/           CSV/Parquet writers, tabular output, run manifest, provenance
├── capi/         extern "C" ABI  ──►  bindings (Python, C#, MATLAB)
└── app/          thin GUI (Qt 6 Widgets) + rcvd-cli
```

**Dependency rule (normative).** Dependencies point strictly downward in the list above.
`models` and `analysis` SHALL NOT include any header from `app`, `io` or `capi`. A build
rule SHALL enforce this (CMake `target_link_libraries` with `PRIVATE`, plus an
include-cycle check in CI).

**Rule A1 — the GUI is thin.** `app` SHALL contain no equation, no unit conversion beyond
calling `units`, and no numerical algorithm. Any behaviour reachable through the GUI SHALL
be reachable through `rcvd-cli` and through the C API with identical results, bit-for-bit
(§18.3).

### 4.2 Language and toolchain

- **C++20**, no compiler extensions. `-Wall -Wextra -Wpedantic -Werror`.
- `-ffast-math` and any flag implying it are **forbidden** (breaks IEEE 754 reproducibility
  and NaN propagation). `-fno-fast-math -ffp-contract=off` SHALL be set explicitly.
- Dense linear algebra: **Eigen 3.4** (header-only, no runtime dependency).
- Build: CMake ≥ 3.24, presets for gcc/clang/msvc, Linux/macOS/Windows, x86-64 and arm64.
- Tests: Catch2 v3. Coverage target ≥ 90 % line, 100 % on `analysis` and `tire`.
- GUI: Qt 6 Widgets. Plotting: QCustomPlot or equivalent; the plot widget consumes only
  `Result` value types.

### 4.3 Core API sketch (normative on shape)

```cpp
namespace rcvd {

// ---- tire ------------------------------------------------------------
struct TireState  { double alpha_rad;      // slip angle
                    double Fz_N;           // vertical load magnitude, >= 0 (D-1)
                    double gamma_rad = 0;  // inclination angle (reserved)
                    double kappa     = 0;  // slip ratio (reserved)
                    double u_mps     = 0; }; // rolling speed, for relaxation

struct TireOutput { double Fy_N;           // lateral force, SAE tire frame
                    double Mz_Nm;          // aligning moment
                    double trail_m;        // pneumatic trail; see T8, never NaN
                    std::optional<double> mu_peak;  // engaged iff has_mu(); see §6.1 T5
                    double C_alpha_N_per_rad; // local slope dFy/dalpha
                    bool   saturated; };

class TireModel {
public:
  virtual ~TireModel() = default;
  virtual TireOutput evaluate(const TireState&) const = 0;
  // Analytic where available; otherwise central difference with the step of §13.6.
  virtual double dFy_dalpha(const TireState&) const = 0;
  virtual double dFy_dFz  (const TireState&) const = 0;
  virtual Interval valid_alpha_rad() const = 0;
  virtual Interval valid_Fz_N()      const = 0;
  virtual std::string_view id()      const = 0;

  // Model-capability queries used by the conformance suite (§6.1) and the solvers.
  virtual bool has_mu()  const = 0;   // false for the pure-stiffness models L1, L2
  virtual bool is_C1()   const = 0;   // false for L2 and for a linear-interpolated table
  virtual std::span<const double> slope_break_points() const = 0;  // sorted alpha values
};

// ---- plants ----------------------------------------------------------
class Plant {                       // xdot = f(t, x, u)
public:
  virtual ~Plant() = default;
  virtual int  state_dim() const = 0;
  virtual void derivative(double t, std::span<const double> x,
                          const DriverInput& u, std::span<double> xdot) const = 0;
  virtual void observe(double t, std::span<const double> x,
                       const DriverInput& u, Observation& out) const = 0;
};

class BicycleModel   : public Plant { /* §8  */ };
class FourWheelerModel : public Plant { /* §9 */ };

// ---- error channel ---------------------------------------------------
struct Diagnostic { std::string_view code;   // "E-SS-030"; §14.2 register
                    Severity        severity;
                    std::string     message;
                    std::optional<double> t_s;        // when, if during a run
                    std::optional<Wheel>  wheel; };   // where, if wheel-specific

template <class T>
class Result {                      // §4.6; C++20, no <expected> dependency
  std::variant<T, Diagnostic> v_;
public:
  Result(T);  Result(Diagnostic);
  bool ok() const noexcept;
  explicit operator bool() const noexcept { return ok(); }
  const T&          value() const;  // precondition: ok(); asserts otherwise
  const Diagnostic& error() const;  // precondition: !ok()
};

// ---- analysis --------------------------------------------------------
Result<StabilityDerivatives> stability_derivatives(const Vehicle&, double V_mps);
Result<SteadyStateResult>    steady_state        (const Plant&, const Vehicle&,
                                                  double V_mps, double delta_rad);
Result<Eigenstructure>       eigenstructure      (const Vehicle&, double V_mps);
Result<MomentDiagram>        moment_method       (const Vehicle&, const MmmSweep&);
Result<RideRollResult>       ride_roll_totals_to_components(const RideRollInput&);
Result<RideRollResult>       ride_roll_components_to_totals(const RideRollInput&);
Result<LapResult>            lap_solve           (const Plant&, const Circuit&,
                                                  const DriverPlan&, const SolverConfig&);
} // namespace rcvd
```

### 4.4 Determinism

**Rule A2.** For identical inputs, identical build and identical `SolverConfig`, every
program SHALL produce bit-identical `double` outputs across runs on the same platform, and
outputs agreeing to `1e-12` relative across supported platforms.

Consequences: no unordered iteration affecting accumulation order; no parallel reduction
over floating point without a fixed reduction tree; no reliance on `std::unordered_map`
iteration order; no use of the `-ffast-math` family; RNG (if ever used) explicitly seeded
and recorded in the run manifest.

### 4.5 Run manifest

Every run SHALL emit a manifest containing: spec revision, library version, git SHA,
compiler and flags, platform, the fully-resolved input document (post-defaulting,
post-unit-conversion, in SI), the `SolverConfig`, wall time, integrator statistics, and the
SHA-256 of the output data file. Results without a manifest are not reportable.

### 4.6 Error channel (normative)

**Rule A3.** Every operation that §14.2 can fail SHALL return `Result<T>` by value. The
core SHALL NOT throw across any API boundary, and `capi/` SHALL NOT wrap calls in
`try`/`catch` — there is nothing to catch. Exceptions remain permitted only for
programming defects that no input can provoke (`std::bad_alloc`, a failed assertion), which
are not `E-*` conditions and are not recoverable.

**Why not `std::expected`.** It is C++23; §4.2 mandates C++20 and the §18.2 platform floor
is set accordingly. `Result<T>` is the same shape over `std::variant` and is a drop-in
replacement should a later revision raise the standard — the member names are chosen to
match, so the migration is a type alias.

**Warnings are not errors.** A `W-*` or `EV-*` diagnostic does not produce an error
`Result`; it is appended to the run manifest and to `Observation::diagnostics`, and the
computation proceeds (§14.1). Only `E-*` produces `!ok()`. Consequently a successful
`Result` may still carry warnings, and callers SHALL surface them (§15.3) rather than
treating `ok()` as "nothing to report".

---

## 5. Data Model

All documents are JSON, UTF-8, with a JSON Schema (Draft 2020-12) shipped alongside. Every
document carries `"schema_version": "1.0"` and SHALL be rejected on major-version mismatch.
Physical quantities are given as `{"value": <number>, "unit": "<string>"}`; the unit string
is parsed by the `units` module against a closed vocabulary.

### 5.1 Vehicle

```jsonc
{
  "schema_version": "1.0",
  "name": "CH5",
  "inertia": {
    "weight":          {"value": 3700, "unit": "lbf"},   // OR "mass" in kg/slug, not both
    "yaw_inertia":     {"value": 1800, "unit": "slug*ft^2"},
    "roll_inertia":    {"value": 700,  "unit": "slug*ft^2"},   // reserved, §20 seam 1
    "yaw_roll_product":{"value": 0,    "unit": "slug*ft^2"},   // reserved
    "unsprung_weight_front": {"value": 240, "unit": "lbf"},    // per AXLE, not per wheel
    "unsprung_weight_rear":  {"value": 260, "unit": "lbf"},
    "unsprung_cg_height_front": {"value": 11, "unit": "in"},
    "unsprung_cg_height_rear":  {"value": 11, "unit": "in"}
  },
  "geometry": {
    "wheelbase":     {"value": 104.5, "unit": "in"},
    "front_weight_fraction": 0.57,        // => b/l ; a = l*(1 - front_fraction)
    "track_front":   {"value": 60, "unit": "in"},
    "track_rear":    {"value": 60, "unit": "in"},
    "spring_track_front": {"value": 60, "unit": "in"},
    "spring_track_rear":  {"value": 50, "unit": "in"},
    "cg_height":     {"value": 20, "unit": "in"},          // TOTAL vehicle CG
    "sprung_cg_height": {"value": 21.406, "unit": "in"},   // OR omit and derive, see below
    "roll_centre_height_front": {"value": 0, "unit": "in"},
    "roll_centre_height_rear":  {"value": 4, "unit": "in"},
    "sprung_cg_front_distance": {"value": 43.466, "unit": "in"} // a_s; OR omit and derive
  },
  "springing": {
    // Supply EITHER this group ...
    "roll_stiffness_chassis_to_ground_front": {"value": 975, "unit": "lbf*ft/deg"},
    "roll_stiffness_chassis_to_ground_rear":  {"value": 325, "unit": "lbf*ft/deg"}

    // ... OR the P6 component group, never both (E-VEH-010):
    //   "wheel_centre_rate_front":  {"value": 183.627, "unit": "lbf/in"},   // per wheel
    //   "wheel_centre_rate_rear":   {"value": 134.818, "unit": "lbf/in"},
    //   "aux_roll_stiffness_front": {"value": 816.293, "unit": "lbf*ft/deg"},
    //   "aux_roll_stiffness_rear":  {"value": 109.218, "unit": "lbf*ft/deg"},
    //   "tire_rate_front":          {"value": 1500, "unit": "lbf/in"},      // per tire
    //   "tire_rate_rear":           {"value": 1500, "unit": "lbf/in"}
    // which yields 975.00 / 325.00 lbf*ft/deg via (12.7).
  },
  // NOTE: the springing block above is the §17.4 P6 case (1600/1000 lbf axles,
  // 18 in CG, roll axis on the ground), reproduced here only to illustrate the
  // two mutually-exclusive forms. It is NOT dimensionally matched to the CH5
  // inertia/geometry above; do not build a CH5 fixture from this snippet. The
  // normative CH5 fixture is defined in §17.2.

  "steering": { "ratio": 14.0 },
  "tires": { "LF": "sedan_front", "RF": "sedan_front",
             "LR": "sedan_rear",  "RR": "sedan_rear" },
  "aero":  { "ref": "sedan_aero" }
}
```

**Redundancy rule (normative).** Where a quantity can be specified more than one way
(`a` and `b` vs `wheelbase` + `front_weight_fraction`; `mass` vs `weight`), the schema
SHALL accept exactly one form per group and reject documents that over-specify, with error
`E-VEH-010`. Silent precedence is forbidden.

**Derived-quantity rule.** `a = ℓ·(1 − f_front)`, `b = ℓ·f_front`, where `f_front` is the
**front weight fraction**. Note the inversion: a car with 57 % front weight has the CG
*closer to the front axle*, so `a = 0.43·ℓ`. This is the single most frequently inverted
relation in vehicle-dynamics code and SHALL carry a dedicated unit test.

**Sprung/unsprung consistency (normative).** The sprung mass and its CG height are not free
parameters once the total mass, total CG height and unsprung masses are given. The
constraint is

```
m    = m_s + m_uF + m_uR                                                       (5.1a)
m·h  = m_s·h_s  + m_uF·z_WF + m_uR·z_WR          (vertical moment)             (5.1b)
m·a  = m_s·a_s  + m_uR·ℓ                          (longitudinal moment)        (5.1c)
```

(5.1c) places the unsprung masses at their axles: the front unsprung mass sits at `x = 0`
measured from the front axle and contributes nothing, the rear at `x = ℓ`.

`sprung_cg_height` (`h_s`) and `sprung_cg_front_distance` (`a_s`) are each **optional**. If
supplied, (5.1b) / (5.1c) are checked to `1e-9` relative and violation raises `E-VEH-011`;
if omitted, the value is derived. Both constraints matter: (5.1b) keeps the roll moment
consistent with the total CG height used by (9.1), and (5.1c) keeps the front/rear split of
the sprung weight in (9.6a)–(9.6b) consistent with the weight distribution used by (9.1).
Leaving `a_s` free is a common and quiet source of load-transfer error.

`m_uF` and `m_uR` are **per axle**; the field names say so. There is no
`sprung_mass_fraction` field — the sprung mass follows from (5.1a).

**Roll-stiffness group.** Either the chassis-to-ground roll stiffnesses (used directly by
§9.3) **or** the P6 component set (`wheel_centre_rate`, `aux_roll_stiffness`, `tire_rate`,
`spring_track`, `track`) may be given, not both. When the component set is given, the
chassis-to-ground values are computed by (12.7) at load time and recorded in the manifest.
This is what makes cross-program check C-04 (§17.8) implementable.

### 5.2 Tire document

```jsonc
{
  "schema_version": "1.0",
  "id": "sedan_front",
  "model": "linear_breakaway",     // linear | linear_breakaway | table | radt | mf
  "linear": { "cornering_stiffness": {"value": -640, "unit": "lbf/deg"},
              "breakaway_slip":      {"value": 5,    "unit": "deg"} },
  "pneumatic_trail": {"value": 1.1, "unit": "in"},
  "relaxation": { "mode": "length", "sigma": {"value": 6, "unit": "ft"} }
}
```

The alternative cornering-stiffness source — mutually exclusive with
`linear.cornering_stiffness` above, per the redundancy rule below — is the quadratic:

```jsonc
  "load_sensitivity": { "mode": "quadratic",            // C_alpha(Fz) = c0*Fz + c1*Fz^2
                        "c0": {"value": -0.4,  "unit": "1/deg"},
                        "c1": {"value": 0.8e-4,"unit": "1/(lbf*deg)"} }
```

and the friction block, admissible only for a model that declares `has_mu() == true`
(§6.1 T5 — **not** `linear` or `linear_breakaway`; supplying it for either raises
`E-TIRE-006`):

```jsonc
  "friction":   { "mode": "linear",                     // mu = mu0 - mu1*Fz
                  "mu0": 1.22,
                  "mu1": {"value": 0.00024, "unit": "1/lbf"} }
```

**Every dimensional coefficient carries a unit (Rules U2/U6).** `c0` has dimensions of
1/angle, `c1` of 1/(force·angle), `mu1` of 1/force. Writing them as bare numbers — as the
source's English-unit expressions `C_α = −0.4·F_z + 0.8e−4·F_z²` and
`μ = 1.22 − 0.00024·F_z` invite — is a factor-of-57 error in `c0`, 13 in `c1` and 4.4 in `μ1` when the file is read as SI. The schema SHALL reject a dimensional coefficient given without a unit.

**Schemas for the other tire models.**

```jsonc
// model: "table"
"table": { "alpha": {"values": [...], "unit": "deg"},
           "Fz":    {"values": [...], "unit": "lbf"},
           "Fy":    {"rows_are": "alpha", "values": [[...],[...]], "unit": "lbf"},
           "Mz":    {"values": [[...],[...]], "unit": "lbf*ft"},   // optional
           "mirror_negative_alpha": true }

// model: "radt"
"radt": { "master_Fy": {"alpha_bar": [...], "F_bar": [...]},
          "master_Mz": {"alpha_bar": [...], "M_bar": [...]},       // optional
          "C_alpha_of_Fz": {"Fz": {...}, "C": {...}},
          "mu_of_Fz":      {"Fz": {...}, "mu": [...]},
          "trail_of_Fz":   {"Fz": {...}, "t_p": {...}} }

// model: "mf"   (per-load coefficient sets, PCHIP-interpolated in Fz)
"mf": { "loads": {"values": [700,1300,1900,2500], "unit": "lbf"},
        "B":  {"values": [...], "unit": "1/deg"},        // dimensional: 1/angle
        "C":  [...],                                     // dimensionless
        "D":  {"values": [...], "unit": "lbf"},          // peak force, signed (see below)
        "E":  [...],                                     // dimensionless
        "SH": {"values": [...], "unit": "deg"},
        "SV": {"values": [...], "unit": "lbf"} }
```

`pneumatic_trail` is required by (6.2) for every model that does not supply `Mz` directly,
and `table.Mz`, when present, carries the same `"rows_are": "alpha"` indexing as `table.Fy`.

**Redundancy rule for tires (normative).** A tire document SHALL supply **exactly one**
source of cornering stiffness. `linear.cornering_stiffness` and a `load_sensitivity`
quadratic are alternatives, not a base value plus a modifier; supplying both raises
`E-TIRE-005`. There is no precedence rule, deliberately: silent precedence between two
plausible sources of `C_α` is how a tire silently becomes a different tire.

**Sign convention in the dimensional MF block.** §6.6's box constraints
(`B > 0`, `1 ≤ C ≤ 3`, `D > 0`) apply to the fit of the **normalised** curve of §6.5, where
`ᾱ` and `F̄` share a sign and `B·C·D = +1`. In the dimensional per-load `mf` block the tire
must satisfy T3 (`α·F_y ≤ 0`), so `D` is stored **negative** (peak lateral force in the SAE
tire frame) and `B·C·D = C_α < 0`. The loader SHALL check `B·C·D < 0` for a dimensional MF
block and `B·C·D > 0` for a normalised one, and raise `E-TIRE-003` otherwise.

### 5.3 Aero document

```jsonc
{ "id": "f1_aero",
  "rho": {"value": 1.2250, "unit": "kg/m^3"},
  "frontal_area": {"value": 14.0, "unit": "ft^2"},
  "CD": 1.10, "CLF": -2.60, "CLR": -2.40,
  "reference_speed_floor": {"value": 1.0, "unit": "m/s"} }
```

### 5.4 Environment

`gravity` (default 9.80665; overridable for altitude or a non-terrestrial study only — Rule
U4 forbids using this override to absorb a test-vector discrepancy, and any non-default value
SHALL appear in the run manifest and on every plot caption), `road_tilt` (bank/crown angle, rad, positive = right side
down as seen by the driver), `surface_friction_scale` (multiplies every tire `μ`),
`air_density` override.

### 5.5 SolverConfig

```jsonc
{ "integrator": "rk4_fixed",         // rk4_fixed | dopri54 | rk4_richardson
  "calculation_increment": {"value": 0.001, "unit": "s"},
  "output_record_increment": {"value": 0.01, "unit": "s"},
  "calculation_length": {"value": 10.0, "unit": "s"},
  "rtol": 1e-8, "atol": 1e-10,
  "max_step": {"value": 0.01, "unit": "s"},
  "small_angle_limit": {"value": 0.15, "unit": "rad"},
  "speed_floor": {"value": 0.5, "unit": "m/s"},
  "load_transfer_coupling": "implicit",      // explicit | implicit  (§9.7)
  "atol_ay": 1e-9,
  "tire_relaxation": false,                  // §6.9

  // Model-reduction switches. Each is individually tested; together they define
  // "bicycle-equivalence mode" for cross-program check C-02 (§17.8).
  "load_transfer": true,
  "aero": true,
  "track_velocity_terms": true,              // the +/- r*t/2 of (9.14)
  "small_angle_slip": false,                 // force v/u instead of arctan(v/u), §3.3
  "steer_projection": true,                  // the cos/sin delta factors of (9.11)-(9.13)
  "aligning_moment": true,                   // include sum M_z in (9.13)
  "longitudinal_forces": true,

  // Source-compatibility switches (§19)
  "legacy_gravity_sign": false,
  "legacy_ride_frequency": false,
  "legacy_linear_interp": false }
```

### 5.6 Maneuver / driver input

```
kind := fixed | step | ramp | sinusoid | doublet | table
fixed     : delta(t) = A
step      : delta(t) = A · H(t − t0)
ramp      : delta(t) = A · clamp((t − t0)/T_r, 0, 1)     [rate = A/T_r]
sinusoid  : delta(t) = A · sin(2π t / T)
doublet   : delta(t) = A · (H(t−t0) − 2H(t−t0−T/2) + H(t−t0−T))
table     : monotone-cubic interpolation of (t, delta) pairs, §13.5
```

Speed control:

```
speed_mode := constant | constant_accel
constant       : u is held; the longitudinal equation is not integrated
constant_accel : path speed increases at a prescribed A_X (default 0.1 g), §9.6
```

**Discontinuity rule (normative).** `step` and `doublet` introduce discontinuities in the
input. A fixed-step integrator SHALL place a step boundary exactly on each discontinuity
time; an adaptive integrator SHALL restart at each discontinuity. Integrating across a
discontinuity destroys the method order and is a defect, not a tolerance issue.

### 5.7 Value-type index

Every type named in the §4.3 API sketch, and where it is defined:

| Type | Defined by | Notes |
|---|---|---|
| `Vehicle` | §5.1 | |
| `TireModel`, `TireState`, `TireOutput` | §4.3, §6.1 | |
| `AeroModel` | §5.3, §7 | |
| `Environment` | §5.4 | |
| `SolverConfig` | §5.5 | |
| `DriverInput` | §5.6 | the *evaluated* input at time `t`: `{delta_rad, speed_cmd}` |
| `Maneuver` | §5.6 | the *schedule*; `Maneuver::evaluate(t) → DriverInput` |
| `DriverPlan` | §11.3 | `{initial_speed, Maneuver, start_position, speed_mode}` |
| `Circuit` | §11.2 | |
| `Observation` | §8.9, §9.8 | the named output-channel bundle for the active plant |
| `Interval` | §6.1 | `{lo, hi}` closed real interval |
| `Result<T>`, `Diagnostic` | §4.6 | the error channel; `E-*` only, per Rule A3 |
| `StabilityDerivatives` | §8.3 | the six derivatives plus the speed at which they were formed |
| `SteadyStateResult` | §8.5 | state, gains, condition number |
| `Eigenstructure` | §8.6 | poles, `ω_n`, `ζ`, `τ_dominant`, eigenvectors |
| `MmmSweep` | §10.3 | `{V, R, beta_grid, delta_grid}` |
| `MomentDiagram` | §10.6 | |
| `RideRollInput`, `RideRollResult` | §12.2–12.3 | |
| `LapResult` | §11.4–11.5 | lap time, path, terminating event |

---

## 6. Tire Subsystem

The tire is the dominant model-form uncertainty in every result this suite produces.
Accordingly the tire interface is the most tightly specified component.

### 6.1 Contract

Given `TireState{α, F_z ≥ 0, u}`, a `TireModel` SHALL return `F_y`, `M_z`, the local slope
`∂F_y/∂α`, the peak friction `μ(F_z)` and a saturation flag. The following invariants are
**normative** and SHALL be enforced by a shared conformance test applied to every model:

- **T1 — Odd symmetry.** `F_y(−α, F_z) = −F_y(α, F_z)` to within `1e-12` relative
  (absent camber/conicity, which Rev 1.0 does not model).
- **T2 — Zero crossing.** `F_y(0, F_z) = 0` exactly; `M_z(0, F_z) = 0` exactly.
- **T3 — Correct sign.** `α · F_y ≤ 0` for all `α` in the valid range.
- **T4 — Zero load.** `F_z = 0 ⇒ F_y = 0` and `M_z = 0` exactly. A lifted wheel produces
  no force. Implementations SHALL guard the `F_z` divisions in §6.5–6.6 against this case
  before dividing, not after.
- **T5 — Friction bound.** For any model that defines a peak friction coefficient
  (`has_mu() == true`): `|F_y| ≤ μ(F_z)·F_z · (1 + 1e-9)`. Models L1 and L2 (§6.2–6.3) are
  *pure stiffness* models with no friction limit; they SHALL return `has_mu() == false` and
  a **disengaged** `mu_peak` (`std::optional`, not a NaN sentinel — §13.1 makes NaN a fatal
  defect, so it must not be used as an "absent" marker), and T5 is not applied to them. Consequently an L2 tire can be
  asked to produce a force exceeding any physical `μF_z` — as the §17.6 MMM cases do — and
  that is a property of the model, not a defect. The GUI SHALL state this where an L1/L2
  tire is selected.
- **T6 — Slope consistency.** The analytic `dFy_dalpha` SHALL agree with a Richardson-
  extrapolated central difference to `1e-6` relative at 20 sample points spanning the valid
  range, excluding declared slope discontinuities **and interpolation knots**.

  The knot exclusion is not a convenience. (13.2)'s stencil samples `x ± h` and `x ± 2h`
  and extrapolates on the assumption that the function is smooth across all four points.
  A PCHIP interpolant (§13.5) is C¹ but **not** C², so a stencil straddling a knot sees a
  curvature jump and degrades from `O(h⁴)` to `O(h)`. Measured on a 5-node monotone table
  with `h = eps^(1/3)`: the check returns **6.9e−11** away from the knots and **1.3e−6**
  with the sample point sitting on one — i.e. a *correct* L3 implementation fails a
  `1e-6` bar for a reason that has nothing to do with its analytic derivative.

  Note that a knot is invisible to `slope_break_points()`, and correctly so: that
  accessor reports C⁰ slope breaks (§6.1 T7), of which a PCHIP interpolant has none. The
  conformance suite SHALL therefore obtain the knot locations from the model's own
  abscissa grid and place its sample points at least `10·h` away from each, or apply a
  one-sided difference there per §13.6.
- **T7 — C⁰ minimum, C¹ preferred.** Every model SHALL be continuous. Models that are only
  C⁰ (`linear_breakaway`, `table` with linear interpolation) SHALL declare
  `is_C1() == false`; the integrator selects a discontinuity-tolerant path (§13.3) and
  the MMM sweep refines near the kink (§10.5).
- **T8 — trail is total, never 0/0.** `TireOutput::trail_m` is the pneumatic trail
  `t_p = −M_z/F_y`. T2 makes both numerator and denominator **exactly** zero at `α = 0`,
  and T4 makes them zero at `F_z = 0` — the two states every straight-line run occupies
  from its first step. Since §13.1 makes a `NaN` a fatal defect, the quotient SHALL NOT be
  evaluated at those points. The model SHALL return the **limit** value instead:

  ```
                ⎧ −M_z / F_y                     |F_y| ≥ F_y,floor  (default 1e-6 N)
  trail_m  =    ⎨                                                                (6.2a)
                ⎩ t_p(F_z)   [α → 0 limit]       otherwise
  ```

  where `t_p(F_z)` is the model's own trail datum: the `pneumatic_trail` input for L1/L2,
  the `trail_of_Fz` table for L4, and `lim_{α→0} −M_z/F_y` evaluated analytically or by
  (13.2) for L3/L5. At `F_z = 0` the trail is a geometric property of a tire carrying no
  load and is **undefined**; the model SHALL return `t_p(0⁺)` and SHALL NOT return zero,
  because a zero trail is a physical claim (the force acts at the contact centre) that a
  lifted wheel does not make. The conformance suite SHALL assert finiteness of `trail_m`
  at `α = 0` and at `F_z = 0` for every model.

### 6.2 Model L1 — linear

```
F_y = C_α · α                                     C_α < 0                       (6.1)
M_z = −t_p · F_y                                                                (6.2)
```

Valid for `|α| ≤ α_lin` (default 0.035 rad = 2°). Outside, a diagnostic is emitted.
`∂F_y/∂α = C_α` exactly.

### 6.3 Model L2 — linear with breakaway saturation

```
              ⎧ C_α · α                    |α| ≤ α_bk
F_y(α) =      ⎨                                                                 (6.3)
              ⎩ C_α · α_bk · sgn(α)        |α| >  α_bk
```

This is the model used by the [R280] Bicycle Model and the Moment Method program
(`α_bk` = 5° in Ch. 5 Prob. 5; 6° in Ch. 8 Prob. 2). It is **C⁰ but not C¹** at
`α = ±α_bk`; `is_C1()` SHALL return false and the slope discontinuity locations SHALL be
exposed via `slope_break_points()` so that the MMM sweep and the integrator can respect
them.

`∂F_y/∂α = C_α` for `|α| < α_bk`, `0` for `|α| > α_bk`; at exactly `|α| = α_bk` the
one-sided derivative from below SHALL be returned (documented choice, tested).

### 6.4 Model L3 — tabular

Input: a rectangular grid `F_y[i][j] = F_y(α_i, F_{z,j})` and, optionally,
`M_z[i][j]`, with `α_i` strictly increasing and `F_{z,j}` strictly increasing.

**Interpolation (normative).** Bilinear interpolation SHALL NOT be used, because the
resulting slope discontinuities across cell boundaries produce the visible "kinks" in
steady-state-response plots noted in [R280] p. 255 and corrupt the numerical Jacobians used
by the steady-state and MMM solvers. Instead:

- In `α`: **Fritsch–Carlson monotone cubic Hermite** interpolation (PCHIP). This is C¹,
  shape-preserving (no spurious overshoot near the tire's force peak), and its derivative
  is available in closed form.
- In `F_z`: PCHIP likewise.
- Tensor product of the two, evaluated α-first then F_z.

**Extrapolation.** Outside the tabulated `α` range, hold the boundary value (constant
extrapolation) and set `saturated = true`. Outside the tabulated `F_z` range, linear
extrapolation is permitted up to 25 % beyond the last node, beyond which the model SHALL
raise `E-TIRE-020`. Silent extrapolation of tire data is a correctness hazard, not a
convenience.

**Odd symmetry.** If the table covers only `α ≥ 0`, the model SHALL mirror per T1 rather
than requiring the user to duplicate rows.

### 6.5 Model L4 — Radt non-dimensional

Three normalising groups, per [RCVD] Ch. 14 and confirmed numerically against the tables in
[R280] pp. 163–164:

```
ᾱ  = C_α · tan α / (μ · F_z)                                                    (6.4)
F̄  = F_y / (μ · F_z)                                                            (6.5)
M̄  = M_z / (μ · F_z · t_p)                                                      (6.6)
```

where `C_α`, `μ` and `t_p` are the load-dependent per-load coefficients:

```
C_α(F_z) = ∂F_y/∂α at α → 0, evaluated at load F_z
μ(F_z)   = max_α |F_y(α, F_z)| / F_z
t_p(F_z) = −M_z / F_y  evaluated at small α
```

**Collapse property.** All loads SHALL collapse onto a single `F̄(ᾱ)` master curve. The
implementation SHALL report the collapse residual (max deviation of any load's normalised
curve from the master fit) as a data-quality metric; a residual above 3 % indicates the
supplied `C_α`, `μ` or `t_p` estimates are inconsistent with the raw table.

**Expansion to an untested load.** Given master curves and interpolated `C_α(F_z)`,
`μ(F_z)`, `t_p(F_z)`:

```
ᾱ = C_α(F_z)·tan α /(μ(F_z)·F_z)  →  F̄ = master(ᾱ)  →  F_y = F̄·μ(F_z)·F_z      (6.7)
```

The inverse (peak location) is `α_peak = arctan( ᾱ_peak · μ F_z / C_α )`.

**Guard.** `μ·F_z` appears in the denominator of (6.4). When `F_z < F_z,min` (default 1 N)
the model SHALL return zero force per T4 without evaluating (6.4).

### 6.6 Model L5 — Pacejka Magic Formula (pure lateral slip)

```
F_y = D · sin( C · arctan( B·x − E·(B·x − arctan(B·x)) ) ) + S_V,   x = α + S_H  (6.8)
```

with the standard identifications `D = peak`, `BCD = ∂F_y/∂x|_{x=0}` = cornering stiffness.
When fitting the *normalised* curves of §6.5, the constraint `B·C·D = 1` holds by
construction ([R280] p. 161). The fitter SHALL impose it as a constraint rather than
discover it, with a tolerance of `1e-3` rather than exact equality (published coefficient
sets are rounded: `0.714 × 1.4 × 1.0 = 0.9996`, and re-solving `B` from `C` and `D` to make
the product exactly 1 changes `B` in its fourth significant figure).

Load dependence of `B, C, D, E` SHALL be supplied either as explicit per-load coefficient
sets with PCHIP interpolation in `F_z`, or through the standard `MF` load-scaling
coefficients. Rev 1.0 requires the former; the latter is an extension seam.

**Fitting.** The fitter SHALL use Levenberg–Marquardt with analytic Jacobians, box
constraints (`B > 0`, `1 ≤ C ≤ 3`, `D > 0`, `E ≤ 1`), and SHALL report per-parameter
standard errors from the covariance estimate `σ²(J^T J)^{-1}` together with RMS and
maximum residual. A fit is not accepted into a tire document without its residual report.

### 6.7 Load sensitivity

Two forms SHALL be supported.

**Friction form** (used in [R280] Ch. 7):

```
μ(F_z) = μ0 − μ1 · F_z              μ(F_z) clamped to [μ_min, μ0], μ_min default 0.1  (6.9)
```

**Cornering-stiffness form** (used in [R280] Ch. 5 Prob. 6):

```
C_α(F_z) = c0·F_z + c1·F_z²         c0 < 0, c1 > 0                              (6.10)
```

With `c0 < 0` and `c1 > 0`, `|C_α|` is greatest at `F_z* = −c0/(2c1)`; beyond that,
cornering-stiffness *magnitude* falls with load, and at `F_z = −c0/c1` the expression
changes sign, which is unphysical. The model SHALL evaluate both breakpoints at
construction, emit `W-TIRE-101` if any wheel load in the run exceeds `F_z*`, and raise
`E-TIRE-004` if any wheel load reaches `−c0/c1`. Note that `E-TIRE-003` (positive `C_α`)
validates the *input coefficients*, not the evaluated `C_α(F_z)`, to avoid a spurious
failure at the sign change.

Load sensitivity is what makes lateral load transfer reduce an axle's total lateral force
capability, and therefore what makes roll-stiffness distribution a handling tuning
parameter. A build in which (6.9)/(6.10) is disabled SHALL make P2's roll-stiffness
distribution input inert, and the GUI SHALL grey it out rather than accept a value that
does nothing.

### 6.8 Combined slip — friction ellipse

Rev 1.0 models combined slip only as a *capacity limit*, not as a full combined-slip force
model:

```
(F_x / (μ_x F_z))² + (F_y / (μ_y F_z))² ≤ 1                                     (6.11)
```

Given a demanded `F_y` and available friction, the remaining longitudinal capacity is

```
F_x,max = μ_x F_z · sqrt( max(0, 1 − (F_y/(μ_y F_z))²) )                        (6.12)
```

The `max(0, ·)` guard is mandatory: rounding can drive the radicand slightly negative when
the tire is at the lateral limit, and an unguarded `sqrt` produces NaN that propagates
silently through an entire time history.

### 6.9 Relaxation-length dynamics

The tire does not generate lateral force instantaneously. The build-up is a first-order lag
in *distance travelled*, not time:

```
σ · (dF_y/ds) + F_y = F_y,ss(α, F_z)                                            (6.13)
```

With `ds = u·dt` this becomes a time-domain first-order lag with a speed-dependent time
constant:

```
τ(u) = σ / u                                                                    (6.14)
τ · Ḟ_y + F_y = F_y,ss                                                          (6.15)
```

Step response: `F_y(t) = F_y,ss·(1 − e^{−t/τ})`, reaching 63.2 % at `t = τ` and 98.2 % at
`t = 4τ`.

Ramp response (slope `α̇ = A`), obtained by Laplace inversion of `(τs+1)F̄_y = C_α A/s²`:

```
F_y(t) = A·τ·C_α·( e^{−t/τ} + t/τ − 1 )                                         (6.16)
```

which lags the quasi-static line `C_α·A·t` by exactly `A·τ·C_α` at large `t` — i.e. a pure
time delay of `τ`.

**Speed guard (normative).** `τ = σ/u` is singular at `u = 0`. When `u < u_floor`
(`speed_floor`, default 0.5 m/s) the relaxation filter SHALL be bypassed and the
steady-state force used, with the state re-initialised to `F_y,ss` so that no transient is
injected when the vehicle re-accelerates. This is a *stiffness* issue as well: at
`u = 100 m/s` and `σ = 0.3 m`, `τ = 3 ms`, roughly 30× faster than the vehicle yaw mode,
so an explicit fixed-step integrator needs `Δt ≲ τ/5`. §13.4 specifies the step-size
consequence.

**When to enable.** Tire lag matters when `τ` is comparable to the vehicle's own time
constants. At 30 m/s with `σ = 0.5 m`, `τ = 17 ms` against a vehicle yaw time constant of
order 100 ms — negligible. At 5 m/s, `τ = 100 ms` — dominant. The default SHALL be
**disabled** for P1/P3 (which are quasi-static or linear-analysis tools) and
**user-selectable, default disabled** for P2/P4/P5, with the ratio `τ/τ_vehicle` reported
in the run manifest so the user can see when the choice matters.

### 6.10 Tire-model validation

| ID | Check | Reference |
|----|-------|-----------|
| TV-1 | Cornering stiffness from slope at α=0 of tabulated data | [R280] p. 17 |
| TV-2 | `μ = F_y,peak/F_z` decreases with load (load sensitivity) | [R280] p. 16, 20 |
| TV-3 | Radt normalisation on **dataset A**: ᾱ = −0.996 for C_α = −240 lb/deg, F_z = 1000 lb, μ = 0.965, α = +4° (the source quotes the magnitude 0.996) | [R280] p. 161 |
| TV-4 | Radt expansion on **dataset A**: F_y = −736.4 lb at that point; master-curve peak at ᾱ = 3.0 ⇒ α = 11.89°, F_y = −965.0 lb | [R280] p. 161 |
| TV-5 | MF fit of the normalised lateral curve of **dataset B**: B=0.714, C=1.4, D=1.0, E=−0.45, BCD = 0.9996 | [R280] p. 164 |
| TV-6 | Relaxation τ = σ/V: σ = 6 ft, V = 10 mph ⇒ τ = 0.409 s | [R280] Fig. 2.6 |
| TV-7 | Linear-model error vs tabulated data: +8 % at 1.5°, +45 % at 4.0° | [R280] pp. 14–15 |
| TV-8 | Friction circle: F_x = √((μF_z)² − F_y²), zero at the lateral peak | [R280] p. 18 |

**Datasets A and B are different tires.** TV-3/TV-4 use the tabulated master curve of
[R280] Ch. 14 Problem 1 (peak `F̄ = 1.000` at `ᾱ = 3.0`); TV-5 fits the master curve derived
from the four-load dataset of Ch. 14 Problem 2 (peak near `ᾱ ≈ 2.46`). Evaluating the TV-5
Magic Formula at the TV-3 operating point gives `F̄ = 0.786` and a peak at `α = 9.81°`,
neither of which matches TV-4 — correctly, because it is a different tire. The test suite
SHALL keep the two datasets in separate fixtures and SHALL NOT cross-apply them; a single
fixture cannot satisfy both, and a "fix" that makes it appear to would be a data error.

---

## 7. Aerodynamics

```
q  = ½ · ρ · V²                                          [Pa]                   (7.1)
F_D = q · C_D · A_f                             (drag, along −x)                (7.2)
F_LF = q · C_LF · A_f                           (front axle vertical)           (7.3)
F_LR = q · C_LR · A_f                           (rear axle vertical)            (7.4)
```

**Sign.** `C_L < 0` denotes downforce. The resulting vertical force **increases** the tire
vertical-load magnitude `F_z` of §6 (D-1). The adapter SHALL therefore add `−q·C_L·A_f` to
each axle's load. A sign test with `C_LF = −2.0` at 50 m/s SHALL show front load
increasing.

**Air density.** Default `ρ = 1.2250 kg/m³` (ISO standard atmosphere, 15 °C, 101.325 kPa).
Altitude/temperature correction:

```
ρ = ρ_std · (p/p_std) · (T_std/T)          T in kelvin                          (7.5)
```

**Aerodynamic balance** is reported as `%front = C_LF/(C_LF + C_LR)` and the centre of
pressure position `x_CP = ℓ·C_LR/(C_LF + C_LR)` measured aft of the front axle. Both are
undefined when `C_LF + C_LR = 0`; the implementation SHALL report `n/a` rather than a
division result.

**Velocity used.** `V` in (7.1) SHALL be the magnitude `√(u² + v²)`, not `u`. At the
sideslip angles reached in P2/P5 the difference is small but the choice SHALL be explicit
and documented, not incidental.

**Important physical note for the user documentation.** Downforce grows as `V²` and the
lateral force required on a fixed radius also grows as `V²`, which naively suggests
unbounded cornering speed. Tire load sensitivity (§6.7) breaks that argument: `μ` falls
with load, so lateral force does not keep pace with downforce. This behaviour SHALL be
reproducible in P2 as an acceptance demonstration.

---

## 8. Program P1 — Bicycle Model

### 8.1 Assumptions

Two axles collapsed to single centreline tires; constant forward speed `u`; small angles;
linear tires; **no** load transfer, **no** roll degree of freedom, **no** aerodynamics,
**no** tire load sensitivity. Two degrees of freedom: lateral velocity `v` and yaw rate `r`.
Planar motion; the CG height is irrelevant to this model and SHALL be ignored (accepting it
as an input and silently ignoring it is a usability defect; the GUI SHALL grey it out).

### 8.2 Derivation

Newton–Euler in the body-fixed frame F2. The absolute lateral acceleration of the CG of a
body translating and rotating in plane is `a_y = v̇ + u·r`. Hence

```
m (v̇ + u r) = F_yF + F_yR                                                       (8.1)
I_z ṙ        = a·F_yF − b·F_yR                                                   (8.2)
```

With the slip angles of (3.1) written in terms of `v`,

```
α_F = (v + a r)/u − δ                α_R = (v − b r)/u                          (8.3)
```

and linear tires `F_yF = C_F α_F`, `F_yR = C_R α_R`, substitution gives the governing
system:

```
m v̇ = (C_F + C_R)/u · v  +  [ (a C_F − b C_R)/u − m u ] · r  −  C_F · δ         (8.4)
I_z ṙ = (a C_F − b C_R)/u · v  +  (a² C_F + b² C_R)/u · r  −  a C_F · δ         (8.5)
```

**Dimensional audit of (8.4).** `[C]=N/rad`, `[v/u]=rad`, so `(C_F+C_R)v/u` is N. ✓
`m·u·r` = kg·(m/s)·(rad/s) = N. ✓ `C_F·δ` = N. ✓ Left side `m·v̇` = N. ✓
For (8.5): `(a C_F)·(v/u)` = m·N = N·m ✓; `I_z ṙ` = kg·m²·rad/s² = N·m ✓.

### 8.3 Stability derivatives

```
Y_β = C_F + C_R                       [N/rad]                                   (8.6)
Y_r = (a C_F − b C_R)/u               [N/(rad/s)] = N·s/rad                     (8.7)
Y_δ = −C_F                            [N/rad]                                   (8.8)
N_β = a C_F − b C_R                   [N·m/rad]                                 (8.9)
N_r = (a² C_F + b² C_R)/u             [N·m/(rad/s)]                            (8.10)
N_δ = −a C_F                          [N·m/rad]                                (8.11)
```

Interpretation, which the GUI SHALL surface as tooltips: `N_β` is the yawing moment due to
sideslip — the weathervane term, and the sign that decides steady-state under/oversteer.
`N_r` is the yawing moment due to yaw rate — a pure rotational damper.

```
N_β > 0  ⇒ understeer          N_β = 0 ⇒ neutral steer          N_β < 0 ⇒ oversteer
```

Note that `Y_r` and `N_r` are speed-dependent by construction; the API SHALL therefore
require `V` as an argument and SHALL NOT cache derivatives on the `Vehicle`.

### 8.4 State-space form

With state `x = [v, r]^T` and input `δ`:

```
        ⎡ Y_β/(m u)          Y_r/m − u ⎤          ⎡ Y_δ/m   ⎤
A(u) =  ⎢                              ⎥ ,  B(u) = ⎢         ⎥                 (8.12)
        ⎣ N_β/(I_z u)        N_r/I_z   ⎦          ⎣ N_δ/I_z ⎦
```

**Second-column trap.** `Y_r` and `N_r` already carry the `1/u` in their definitions
(8.7), (8.10). The `r`-column entries are therefore `Y_r/m − u` and `N_r/I_z`, **not**
`Y_r/(m u)` and `N_r/(I_z u)`. Dividing by `u` a second time is a speed-dependent error
that leaves the eigenvalues plausible-looking while making the steady-state yaw-rate gain
wrong by a factor of order `u`. Identity I-2 detects it at every speed.

with `ẋ = A x + B δ`.

**Sign note (a documented trap).** Reading (8.4)–(8.5), the `δ` coefficients are `−C_F/m`
and `−a·C_F/I_z`. Since `Y_δ ≡ −C_F` and `N_δ ≡ −a·C_F`, these are `+Y_δ/m` and `+N_δ/I_z`
— **positive**, because `C_F < 0`. Writing `B` with a leading minus (a natural slip, since
the derivation shows a minus sign in front of `C_F`) double-negates the input and produces a
steady-state yaw rate of the wrong sign and wrong magnitude. Identity I-2 (§8.5) catches
this immediately and is the reason it is normative.

**Sideslip state variant.** If the user selects `β` as a state (`β = v/u`), the transformed
matrices are `A_β = T A T^{-1}`, `B_β = T B` with `T = diag(1/u, 1)`. Both parameterisations
SHALL be available and SHALL be shown by a unit test to have identical eigenvalues.

### 8.5 Steady state

Setting `ẋ = 0`:

```
x_ss = −A^{-1} B δ                                                             (8.13)
```

The implementation SHALL solve (8.13) by **LU with partial pivoting** on the 2×2 system,
never by an explicit inverse, and SHALL check the reciprocal condition number; if
`rcond < 1e-12` the vehicle is at or beyond its critical speed and the solver SHALL return
`E-SS-030` rather than a huge number.

Closed forms, retained because they are the cheapest cross-check available:

```
Stability factor      K = m·N_β / ( ℓ·(N_β Y_δ − Y_β N_δ) )         [s²/m²]    (8.14)
Yaw-rate gain         r/δ = (V/ℓ) / (1 + K V²)                      [1/s]      (8.15)
Lateral-accel gain    A_Y/δ = V·(r/δ) = (V²/ℓ)/(1 + K V²)           [m/s²/rad] (8.16)
Ackermann steer       δ_Ack = ℓ/R                                              (8.17)
Understeer gradient   K_us = K·ℓ                                    [rad/(m/s²)] (8.18)
Characteristic speed  V_ch = √(1/K)                     (K > 0, understeer)    (8.19)
Critical speed        V_crit = √(−1/K)                  (K < 0, oversteer)     (8.20)
Tangent speed         V_tan = √( −b·ℓ·C_R / (m·a) )                            (8.21)
Neutral steer point   NSP = C_R/(C_F + C_R)   (fraction of ℓ aft of front axle)(8.22)
Static margin         SM  = NSP − a/ℓ                                          (8.23)
```

**Identity I-1 (normative test).** The derivative form (8.14) SHALL be shown to equal the
classical axle form

```
K·ℓ·g = W_F/|C_F| − W_R/|C_R|          [rad per g]                            (8.24)
```

to `1e-12` relative, where `W_F = m g b/ℓ` and `W_R = m g a/ℓ`. This is a strong
cross-check on the sign conventions of §8.3 and has been verified numerically for the CH5
vehicle: both sides give `9.63787e-3 rad/g = 0.5522 deg/g`.

**Identity I-2.** (8.15) computed from (8.14) SHALL equal the direct derivative expression

```
r/δ = (Y_β N_δ − N_β Y_δ) / (N_β Y_r − N_β m V − Y_β N_r)                      (8.25)
```

to machine precision. Verified: both give `4.73716 s⁻¹` for CH5 at `V = 13.41 m/s`.

**Derivation of the tangent speed (8.21).** At steady state `r = V/R` and the rear axle
carries `F_yR = m·(V²/R)·(a/ℓ)`. With `α_R = β − b/R` and `F_yR = C_R α_R`,

```
β = b/R + m V² a /(R ℓ C_R)
```

Setting `β = 0` and cancelling `1/R` gives (8.21). Since `C_R < 0`, the radicand is
positive. Below `V_tan` the vehicle corners nose-out (`β > 0`); above, nose-in (`β < 0`).
This is the mechanism behind the sideslip sign reversal observed in the step-steer
transient (§17.3, X-04).

### 8.6 Transient response

The characteristic polynomial of `A` is `s² + 2ζω_n s + ω_n² = 0` with

```
2 ζ ω_n = −tr A = −( Y_β/(m u) + N_r/I_z )                                     (8.26)
ω_n²    = det A = [ Y_β N_r − N_β (Y_r − m u) ] / (m I_z u)                    (8.27)
```

The implementation SHALL obtain eigenvalues from `A` via a **balanced 2×2 closed-form
solve** (discriminant), not a general QR routine, and SHALL cross-check `tr A` and `det A`
against the eigenvalue sum and product to `1e-12`.

```
ω_d = ω_n √(1 − ζ²)   (ζ < 1)      s = −ζω_n ± i ω_d
```

Stability: the system is asymptotically stable iff `tr A < 0` **and** `det A > 0`. For the
bicycle model `tr A < 0` always (both `Y_β` and `N_r` are negative), so stability reduces to
`det A > 0`. `det A` and `1 + K V²` differ only by a positive speed-dependent factor, so
they are **sign-equivalent**, and `det A > 0 ⟺ 1 + K V² > 0` — recovering the
critical-speed result (8.20) independently. Verified numerically for the **CH5-OS**
variant — CH5 with the weight distribution reversed to 43 % front (`a = 0.57·ℓ`,
`b = 0.43·ℓ`), `I_z = 1800 slug·ft²`, tires unchanged: `det A` and `1 + K V²` are both
positive at 100 ft/s and both negative at 170 ft/s, bracketing `V_crit`. This equivalence
is test `B-31` (§17.2), since it links the transient and steady-state code paths.

**Yaw-rate transfer function.** With `C = [0, 1]`,

```
r(s)/δ(s) = C (sI − A)^{-1} B = ( n₁ s + n₀ ) / ( s² + 2ζω_n s + ω_n² )        (8.28)
n₁ = B₂ ,      n₀ = −B₂ A₁₁ + B₁ A₂₁
```

The DC gain `n₀/ω_n²` SHALL equal (8.15) to machine precision — a third independent
cross-check (verified: both give `4.92645 s⁻¹` for the RCVD p.256 "First Example").

Bode magnitude and phase SHALL be computed from (8.28) on a user-specified logarithmic grid
and reported in dB and degrees.

**Time constant reporting.** For an overdamped or well-damped response the program SHALL
report the **exact** first-order-equivalent time constant from the dominant pole,
`τ = 1/|Re(s_dominant)|`, alongside the empirical "98 % of steady state divided by four"
figure used in the source exercises. These differ; both SHALL be labelled unambiguously.

### 8.7 Kinematics and path

To plot the ground path the earth-frame position is integrated alongside the dynamics:

```
ψ̇   = r
Ẋ_E = u cos ψ − v sin ψ
Ẏ_E = u sin ψ + v cos ψ                                                        (8.29)
```

The plant is 2-state (`v, r`); adding heading makes 3, and adding position makes the
5-state vector `[v, r, ψ, X_E, Y_E]` used by default. Position states
SHALL be integrated with the same integrator and step as the dynamics.

**Plotting convention warning (normative UI behaviour).** In the SAE frame `Y_E` points
90° clockwise from `X_E`. Plotting `X_E` on the horizontal axis and `Y_E` on the vertical
axis mirrors the path, so a right-hand turn is drawn as a left-hand turn. The default plot
SHALL therefore be `X_E` on the **vertical** axis and `Y_E` on the **horizontal** axis, with
equal axis scaling ("square axes") enabled by default so that a circular path renders as a
circle. A one-click axis swap SHALL be available and SHALL annotate the plot when
non-default.

### 8.8 Road tilt

A constant road tilt (crown or bank) of angle `θ_t` about the vehicle `x` axis introduces a
gravity component in the lateral equation:

```
m(v̇ + u r) = F_yF + F_yR + m g sin θ_t                                         (8.30)
```

The static-path answer for a straight run on a tilted road follows from setting `r = 0`:
the required axle forces are `F_yF = −m g sinθ_t·(b/ℓ)`, `F_yR = −m g sinθ_t·(a/ℓ)`,
whence `α_F = F_yF/C_F`, `α_R = F_yR/C_R`, `β = α_R`, `δ = β − α_F`. This result is
**speed-independent**, which is a useful invariant test.

### 8.9 P1 outputs

Minimum channel set (all in SI internally, converted on output):

`time, delta, u, v, beta, r, r_dot, A_Y, A_X, alpha_F, alpha_R, Fy_F, Fy_R, psi, X_E, Y_E,
path_radius, path_curvature`

plus the scalar report: `Y_beta, Y_r, Y_delta, N_beta, N_r, N_delta, K, K_us, V_ch or
V_crit, V_tan, NSP, SM, omega_n, zeta, poles, tau_dominant, DC gains (r/δ, A_Y/δ, β/δ)`.

`path_radius = V/r` with the guard `|r| < 1e-9 ⇒ radius = ∞` reported as `inf`, and
`path_curvature = r/V` reported instead as the numerically robust primary channel.

---

## 9. Program P2 — Four-Wheeler

### 9.1 Scope of the added physics

Relative to P1: four individual wheels with individual vertical loads; lateral and
longitudinal load transfer; roll-stiffness distribution; roll-centre heights; aerodynamic
drag and front/rear downforce; nonlinear, load-sensitive tires. Roll remains **quasi-static**
(algebraic), not a dynamic degree of freedom — this is the defining simplification of P2 and
SHALL be stated in the GUI.

### 9.2 State vector

```
x = [ v, r, ψ, X_E, Y_E ]                       (speed_mode = constant)
x = [ u, v, r, ψ, X_E, Y_E ]                    (speed_mode = constant_accel)
```

plus, if tire relaxation is enabled, four lagged lateral-force states `F_y,LF … F_y,RR`.

### 9.3 Wheel loads — the load-transfer model

This is the numerically delicate part of P2 and is specified in full.

**Step 1 — static loads.**

```
F_z,F,static = m g · b/ℓ            F_z,R,static = m g · a/ℓ                    (9.1)
each front wheel: ½ F_z,F,static    each rear wheel: ½ F_z,R,static
```

**Step 2 — aerodynamic vertical load.** From (7.3)–(7.4), added per axle and split evenly
left/right:

```
ΔF_z,F,aero = −q C_LF A_f           ΔF_z,R,aero = −q C_LR A_f                   (9.2)
```

(positive for downforce, since `C_L < 0`).

**Step 3 — longitudinal load transfer.**

```
ΔF_z,x = m · A_X · h / ℓ                                                        (9.3)
```

subtracted from the front axle and added to the rear under acceleration
(`A_X > 0`), split evenly left/right. Aerodynamic drag acts at the CG height in Rev 1.0
(a documented simplification; the extension seam for a drag-height input is in §20).

**Step 4 — lateral load transfer.** Define the roll-axis height beneath the sprung CG and
the perpendicular distance from the sprung CG to the roll axis:

```
z_R,eff = (1 − a_s/ℓ)·z_RF + (a_s/ℓ)·z_RR                                       (9.4a)
h₂      = h_s − z_R,eff                                                         (9.4b)
```

(small-angle approximation for the roll-axis inclination; the implementation SHALL emit
`W-VEH-120` if `|z_RR − z_RF|/ℓ > 0.05`, where the approximation begins to bite.)

**Roll angle.** The sprung mass rolls about the roll axis under the applied moment
`m_s h₂ A_Y` plus the destabilising moment from the lateral displacement of its own weight,
`m_s g h₂ φ̃`, resisted by the total roll stiffness `ΣK = K_φF + K_φR`:

```
ΣK · φ̃ = m_s h₂ A_Y + m_s g h₂ φ̃

     ⇒  φ̃ = m_s h₂ A_Y / ( ΣK − m_s g h₂ )                                      (9.5)
```

`φ̃` is defined as the **outward** lean magnitude with sign — positive for positive `A_Y`,
and **odd** in `A_Y`. The SAE roll angle of §3.3 is `φ = −φ̃`. All load-transfer expressions
below use `φ̃`; using `|φ̃|` would make the transfer even in `A_Y` and break left/right
antisymmetry (a symmetric vehicle would transfer different loads turning left and right).

The denominator `ΣK − m_s g h₂` is the *effective* roll stiffness. When it approaches zero
the vehicle is roll-divergent. The implementation SHALL check `ΣK > 1.05 · m_s g h₂` at
construction and reject otherwise with `E-VEH-140`. Both terms must be in N·m/rad — this is
where a `lbf·ft/deg` conversion error announces itself as an absurd roll gradient.

**Transferred load per axle.** The roll moment reacted through the springs and bars at each
axle is that axle's share of the total, `K_φF·φ̃` and `K_φR·φ̃`. Adding the moment reacted
directly through the suspension links at the roll centre (the "geometric" or jacking term)
and the unsprung mass acting at its own CG height:

```
ΔF_z,F = [ K_φF·φ̃  +  (1 − a_s/ℓ)·m_s·z_RF·A_Y  +  m_uF·z_WF·A_Y ] / t_F        (9.6a)
ΔF_z,R = [ K_φR·φ̃  +      (a_s/ℓ)·m_s·z_RR·A_Y  +  m_uR·z_WR·A_Y ] / t_R        (9.6b)
```

**Why this form and not the four-term decomposition.** The published treatment ([R280]
p. 218, [RCVD] Ch. 18) splits the sprung-mass contribution into a spring term
`(m_s g/t)·h₂·K_φF/ΣK·(A_Y/g)` plus a separate "gravity" term
`(m_s g/t)·(1 − a_s/ℓ)·h₂·φ`, i.e. it distributes the spring moment by the roll-stiffness
ratio but the gravity moment by the **sprung-weight** ratio. Those two ratios are not equal
in general, and the split is therefore inconsistent: the load transferred through an axle's
springs is `K_φ,axle·φ̃/t` by definition of a roll spring, whatever the origin of the moment
that produced `φ̃`. The single term `K_φF·φ̃/t_F` in (9.6a) is that quantity exactly, and it
carries the gravity effect automatically through the reduced denominator of (9.5).

The published decomposition is recovered — and its two defects exposed — by expanding
(9.6a):

```
K_φF·φ̃/t_F  =  (m_s/t_F)·h₂·(K_φF/ΣK)·A_Y  +  (m_s/t_F)·h₂·(K_φF/ΣK)·g·φ̃
```

so the gravity contribution is distributed by `K_φF/ΣK`, **not** by `(1 − a_s/ℓ)`, and it
**adds** to the transfer rather than subtracting from it. Note that the published *spring*
term is itself correct, `ΣK` denominator and all — the two defects are confined to the
gravity term's sign and its distribution ratio. See errata E-1a and E-1b (§19).

**Which roll stiffness.** `K_φF` and `K_φR` in (9.5)–(9.6b) are **chassis-to-ground** roll
stiffnesses, i.e. the springs and anti-roll bar in series with the tire vertical rates, per
(12.7). This is the stiffness that governs how the roll moment divides at the contact
patches, which is what load transfer is. Using the chassis-to-wheel-centre value instead
overstates both axles' stiffness (by 33 % for the front axle of the §17.4 case) and, unless
the front and rear tire rates happen to be equal, misallocates the split. The `Vehicle`
schema field is named `roll_stiffness_chassis_to_ground_{front,rear}` for this reason, and
P6 direction A (§12.2) produces exactly this quantity.

**Step 5 — assembly.**

```
F_z,LF = ½(F_z,F,static + ΔF_z,F,aero − ΔF_z,x) + ΔF_z,F
F_z,RF = ½(F_z,F,static + ΔF_z,F,aero − ΔF_z,x) − ΔF_z,F
F_z,LR = ½(F_z,R,static + ΔF_z,R,aero + ΔF_z,x) + ΔF_z,R
F_z,RR = ½(F_z,R,static + ΔF_z,R,aero + ΔF_z,x) − ΔF_z,R                        (9.7)
```

(signs shown for a right-hand turn, `A_Y > 0`, in which the **left** wheels are outside).

**Step 6 — wheel lift.** Each load is clamped: `F_z ← max(F_z, 0)`. A wheel at `F_z = 0`
produces zero force (T4). The clamp SHALL raise event `EV-LIFT` with the wheel identity and
time, and the event SHALL appear in the tabular output and on the plots, because a lifted
wheel invalidates the quasi-static roll model's premises. Rev 1.0 does **not** redistribute
the lost moment to the other axle; it reports the condition and continues. This limitation
SHALL be stated in the user documentation.

### 9.4 Definitions of transfer and fraction (disambiguation)

The source literature uses two conflicting conventions. This specification fixes one:

```
ΔF_z,axle   ≡ load ADDED to the outside wheel and SUBTRACTED from the inside wheel
            = M_axle / t_axle                                                   (9.8)
FLT         ≡ (F_z,outside − F_z,inside) / F_z,axle = 2·ΔF_z,axle / F_z,axle    (9.9)
```

so `FLT = 1` is exactly the wheel-lift condition. For a rigid vehicle with equal tracks,
`ΔF_z,total = m·A_Y·h/t`, and `FLT_axle = 2·(A_Y/g)·(h_e/t)` where `h_e` is that axle's
effective CG height. Both API and file format SHALL use the names `transfer_N` and
`flt` for these two distinct quantities. Confusing them is a factor-of-two error that
silently halves or doubles every wheel load.

### 9.5 Invariants I-3 and I-3b (normative tests)

**I-3 — whole-vehicle roll-moment balance.** For any parameter set and any `A_Y`:

```
ΔF_z,F·t_F + ΔF_z,R·t_R  =  m_s·A_Y·h_s  +  m_s·g·h₂·φ̃
                            + m_uF·A_Y·z_WF + m_uR·A_Y·z_WR                    (9.10)
```

*Proof.* From (9.6a)–(9.6b), the left side is `ΣK·φ̃ + m_s A_Y z_R,eff + m_uF A_Y z_WF +
m_uR A_Y z_WR`. Substituting `ΣK·φ̃ = m_s h₂ A_Y + m_s g h₂ φ̃` from (9.5) and
`h₂ + z_R,eff = h_s` from (9.4a)–(9.4b) gives the right side identically. ∎

Note that the gravity term `m_s g h₂ φ̃` carries **no** `A_Y/g` factor: `φ̃` is already
first-order in `A_Y` through (9.5). Verified numerically over 2000 randomised admissible
parameter sets: maximum relative residual `6.0e−16`.

**I-3b — per-axle allocation.** I-3 is a *summed* identity and therefore cannot detect a
misallocation between axles that preserves the total. Swapping `K_φF ↔ K_φR` changes the
axle transfers substantially while leaving the I-3 residual at machine zero (demonstrated:
1485.7 / 1138.0 N per unit `A_Y` becomes 943.6 / 1698.2 N, I-3 residual exactly 0). A
second, per-axle check is therefore required:

```
ΔF_z,F·t_F − K_φF·φ̃ − (1 − a_s/ℓ)·m_s·z_RF·A_Y − m_uF·z_WF·A_Y  =  0           (9.10b)
```

and likewise for the rear.

**What I-3b is and is not.** Against (9.6a) it is an algebraic tautology — it is (9.6a)
multiplied by `t_F` — so it carries no independent physics. Its value is as a **structural**
check: it pins the *implementation* to the specified split, and defect injection shows it
firing where I-3 is blind. Conversely I-3 fires where I-3b is blind. Measured residuals
under injected defects:

| Injected defect | I-3 residual | I-3b residual |
|---|---|---|
| none | 1e−16 | 1e−16 |
| `K_φF ↔ K_φR` swapped | **0** (blind) | **5.8e−1** |
| `φ̃` denominator `ΣK` instead of `ΣK − m_s g h₂` | **3.9e−2** | 1e−16 (blind) |
| `φ̃` sign flipped (E-1a legacy form) | **1.6e+0** | 1e−16 (blind) |
| `z_R,eff` inverted in (9.4a) | **1.2e−2** | 1e−16 (blind) |
| `K_φF`, `K_φR` both scaled by 57.2958 (a `deg`↔`rad` unit error) | **0** (blind) | **0** (blind) |

**I-3c — absolute scale check.** The last row is the important one: neither I-3 nor I-3b
detects a *consistent* unit error in the roll stiffnesses, because (9.10) is an identity in
`ΣK` and (9.10b) reads `K_φ` from the same resolved document the model does. A third check
with an independent dimensional reference is therefore normative:

```
| roll gradient from (9.5), expressed in deg/g |  ∈  [0.1, 12.0]                (9.10c)
```

together with an equality assertion against (12.10) computed independently in the
presentation layer. A vehicle whose roll gradient falls outside that band is either
misconfigured or has a `lbf·ft/deg` ↔ `N·m/rad` error, and SHALL raise `W-VEH-141`.

I-3, I-3b and I-3c SHALL each be run at 200 randomised parameter sets in CI, at both signs
of `A_Y` (which additionally tests the antisymmetry required by §9.3). Together they pin
the total, the split and the scale. Note what they do **not** cover: the `a` vs `b`
inversion of §5.1 enters through (9.1) and (9.3), which none of these three touches — that
inversion is caught by the dedicated unit test §5.1 requires, and by cross-program check
C-02.

### 9.6 Equations of motion

```
m(v̇ + u r) = Σ_{i∈{LF,RF,LR,RR}} F_y,i cos δ_i + Σ F_x,i sin δ_i + m g sin θ_t (9.11)
m(u̇ − v r) = Σ F_x,i cos δ_i − Σ F_y,i sin δ_i − F_D                            (9.12)
I_z ṙ       = Σ_i [ x_i·(F_y,i cos δ_i + F_x,i sin δ_i)
                    − y_i·(F_x,i cos δ_i − F_y,i sin δ_i) ]  + Σ M_z,i         (9.13)

with (x_i, y_i) the wheel positions in F2:
   LF (+a, −t_F/2)   RF (+a, +t_F/2)   LR (−b, −t_R/2)   RR (−b, +t_R/2)

Expanded for the common case δ_LF = δ_RF = δ, δ_LR = δ_RR = 0:

I_z ṙ = a·[(F_y,LF + F_y,RF) cos δ + (F_x,LF + F_x,RF) sin δ]
        − b·(F_y,LR + F_y,RR)
        + (t_F/2)·[(F_x,LF − F_x,RF) cos δ − (F_y,LF − F_y,RF) sin δ]
        + (t_R/2)·(F_x,LR − F_x,RR)
        + Σ M_z,i                                                             (9.13a)
```

**Sign trap.** The right-hand wheels sit at `y = +t/2`, so their longitudinal force
produces a **negative** yaw moment: the couple is `(t/2)(F_x,left − F_x,right)`, not
`(F_x,right − F_x,left)`. Writing it the other way silently reverses the yaw effect of
asymmetric braking and of a locked differential — a defect that a symmetric-input test can
never reveal. A dedicated asymmetric-`F_x` unit test SHALL cover it.

Rev 1.0 applies the same steer angle `δ` to both front wheels (no Ackermann geometry,
no toe); the per-wheel `δ_i` in (9.11)–(9.13) is retained in the code so that steering
geometry can be added without changing the equation structure.

**Aligning moments.** `Σ M_z,i` SHALL be included when the tire model provides `M_z`. It is
a small term but it is the physical origin of steering feel and is required for any future
steering-system model; omitting it silently would make later extension a rewrite.

**Slip angles per wheel.**

```
α_LF = arctan( (v + a r) / (u + r t_F/2) ) − δ
α_RF = arctan( (v + a r) / (u − r t_F/2) ) − δ
α_LR = arctan( (v − b r) / (u + r t_R/2) )
α_RR = arctan( (v − b r) / (u − r t_R/2) )                                     (9.14)
```

**Derivation and sign.** The velocity of a point at body coordinates `(x_i, y_i)` is
`(u − r·y_i, v + r·x_i)`. The left wheels are at `y = −t/2`, so their longitudinal velocity
is `u + r·t/2`: in a right-hand turn (`r > 0`) the left wheels are on the outside and
travel **faster**, as they must. Placing the minus sign on the left wheels is the natural
transcription error and inverts this. The `± r·t/2` terms are second-order in most
manoeuvres but become significant at small radius, which is exactly the regime of
P5's 50-ft-radius circuit. The denominators SHALL be guarded: if `|u ± r t/2| < u_floor`,
the wheel's slip angle is set to `±π/2·sgn(numerator)` and event `EV-WHEEL-STALL` is
raised (distinct from `EV-SPIN`, which is reserved for `|β| > 90°`, §11.5).

**Definition of `A_X` in the load-transfer equations.** `A_X` in (9.3) SHALL be the
**specific longitudinal force** on the CG in the body frame, evaluated from the right-hand
side of (9.12) with the forces of the current integrator stage:

```
A_X = ( Σ F_x,i cos δ_i − Σ F_y,i sin δ_i − F_D ) / m                        (9.12a)
```

This definition holds under **both** speed modes, and it is deliberately *not* written as
`A_X = u̇ − v·r`. Under `speed_mode = constant`, `u` is held, so `u̇ = 0` and that
expression would evaluate to `−v·r` — silently discarding the drag-induced longitudinal
transfer this paragraph exists to preserve. (9.12) is still **evaluated** in that mode to
obtain (9.12a); the `u̇` it implies is then discarded. Using the commanded schedule value
instead would omit drag-induced transfer entirely. This choice SHALL be recorded in the
manifest, and a dedicated unit test SHALL confirm that a constant-speed run with `C_D > 0`
and no tire longitudinal force produces a **non-zero** `ΔF_z,x` of the sign that unloads
the front axle.

**Speed mode `constant_accel`.** The source programs increase *path* speed at a constant
rate (default 0.1 g) irrespective of vehicle attitude. This is retained for compatibility
but is a kinematic driver, not a dynamics: `V̇ = A_X,cmd` with `V = √(u²+v²)`, and `u̇` is
back-solved. The implementation SHALL label this mode "kinematic speed schedule" in the GUI
and SHALL warn that it does not respect the friction ellipse.

### 9.7 Solution of the algebraic loop

`A_Y` appears in the load-transfer equations (9.5)–(9.6b) and the loads determine the tire forces
that determine `A_Y`. Two options SHALL be provided:

**Mode `explicit` (default, matches the source programs).** Use `A_Y` from the previous
accepted step (or from the current stage's state, `A_Y = (v̇ + u r)` evaluated with the
previous forces). Cheap, and adequate when `Δt` is small relative to the load-transfer
dynamics. Introduces an `O(Δt)` lag in the algebraic coupling.

**Mode `implicit` (recommended for analysis).** Solve the fixed point

```
A_Y = Φ(A_Y)        Φ = (1/m)·Σ F_y( α, F_z(A_Y) )                             (9.15)
```

at each integrator stage using **damped Newton** on the residual `R(A_Y) = Φ(A_Y) − A_Y`,
with the derivative `Φ'` assembled from `∂F_y/∂F_z` (§6.1) and `∂F_z/∂A_Y` from (9.6a)–(9.6b).
Convergence criterion `|R| < atol_ay` (default `1e-9 m/s²`), maximum 20 iterations,
fall back to bisection on `[−3g, +3g]` if Newton stalls. Non-convergence raises
`E-P2-050` and aborts the run rather than continuing with an unconverged load state.

`|Φ'| < 1` is the contraction condition; the solver SHALL report the observed `Φ'` in the
manifest, because `Φ' → 1` is the numerical signature of an axle at its load-transfer
limit, and users should see it.

### 9.8 P2 outputs (25 channels, normative)

```
 1 time                  9 alpha_LF            17 Fz_LR
 2 delta                 10 alpha_RF           18 Fz_RR
 3 u                     11 alpha_LR           19 Fy_LF
 4 v                     12 alpha_RR           20 Fy_RF
 5 beta                  13 A_X                21 Fy_LR
 6 r                     14 A_Y                22 Fy_RR
 7 psi                   15 Fz_LF              23 roll_angle
 8 X_E, Y_E (2 ch)       16 Fz_RF              24 path_curvature
```

Derived report channels: `FLT_front`, `FLT_rear`, `roll_stiffness_distribution_front`,
`aero_downforce_front`, `aero_downforce_rear`, `drag`, `understeer_gradient_local`,
`total_lateral_force`, `yaw_moment_residual`.

`yaw_moment_residual` = right-hand side of (9.13). At steady state it must be zero; carrying
it as an output channel makes the approach to steady state visible and is the primary
debugging aid for P2.

---

## 10. Program P3 — Moment Method (MMM)

### 10.1 Principle

The Moment Method is a **constrained test**. The vehicle is held at a prescribed speed and
path curvature while body slip angle `β` and steer angle `δ` are swept independently. At
each `(β, δ)` the tire forces are evaluated and the resulting **lateral force** and
**residual yaw moment** are recorded. The vehicle is generally *not* in yaw equilibrium —
that is the point: the residual moment measures the vehicle's ability to change its yaw
state, which is invisible to any steady-state-only analysis.

Physically this is the road equivalent of a wind-tunnel force-and-moment balance.

### 10.2 Normalisation

The MMM solver computes in SI like every other solver (Rule U1); the two diagram
coordinates are **normalised presentation quantities**, formed at the boundary:

```
A_Y  = ( Σ F_y ) / m                            [m/s²]  — solver quantity     (10.1)
N    = a·F_yF − b·F_yR   (+ Σ M_z when modelled) [N·m]  — solver quantity     (10.2)

Ā_Y  = A_Y / g                                  [–, "g"]  — diagram abscissa  (10.3a)
C_N  = N / ( m · g · ℓ )                        [–]       — diagram ordinate  (10.3b)
```

`Ā_Y` and `C_N` are formed in the presentation layer, never inside the sweep. (`g` also
appears as a normalising divisor in the *reported* forms of FLT (9.9), ride frequency
(12.9), the pair-analysis quadratic (17.1) and the understeer-gradient identity (8.24) —
all likewise reporting or design-side, none inside a time-integration loop.) Dividing the moment by `m·g·ℓ` rather than
`m·g` alone makes `C_N` comparable across vehicles of different wheelbase, exactly as an
aerodynamic moment coefficient is divided by a reference length. The API SHALL expose `N`
in N·m and `A_Y` in m/s² alongside the normalised pair.

### 10.3 The sweep

Inputs: `V`, path radius `R` (default ∞, i.e. `r = 0`; "straight-line constrained test"),
grids `β ∈ [β_min, β_max]` and `δ ∈ [δ_min, δ_max]`.

For each grid point:

```
u      = V cos β                         (longitudinal component, per (3.1))
r      = V/R                             (0 for the straight-line test)
α_F    = β + a r/u − δ
α_R    = β − b r/u
F_yF   = tire_F(α_F, F_zF)               F_yR = tire_R(α_R, F_zR)
Ā_Y    = (F_yF + F_yR)/(m g)
C_N    = (a F_yF − b F_yR + Σ M_z)/(m g ℓ)                                    (10.4)
```

Note `r/u`, not `r/V`: (3.1) is normative on the body-frame longitudinal velocity, and MMM
sweeps `β` to ±10° or more, where `cos β` differs from 1 by 1.5 %. The `Σ M_z` term is
included whenever `SolverConfig.aligning_moment` is set, exactly as in (9.13) — P3 SHALL NOT
silently disable a force path that P1 and P2 include.

For the **linearised** MMM specified here (matching the source program), the axle loads
`F_zF`, `F_zR` are the static values; the four-wheel variant with load transfer is an
extension seam (§20) that changes only the load computation, not the sweep.

**Grid.** Default 41 × 41. The sweep SHALL be exposed as a lazily-evaluated function
`(β, δ) ↦ (A_Y, C_N)` so that the contouring and root-finding stages can refine adaptively
rather than being limited to the display grid.

### 10.4 Construction lines (analytic limits)

For the linear-with-breakaway tire (§6.3) the diagram boundary is bounded by four analytic
points that SHALL be computed in closed form and drawn as reference lines:

```
Front limit:   Ā_Y = ± C_F α_bk,F /(m g)       C_N = ± a·C_F α_bk,F /(m g ℓ)
Rear limit:    Ā_Y = ± C_R α_bk,R /(m g)       C_N = ∓ b·C_R α_bk,R /(m g ℓ)   (10.5)
```

(sign of `C_N` opposite for the rear because rear lateral force produces a *restoring*
yaw moment). These are the "front construction line" and "rear construction line".

**Boundary for the other tire models.** L3/L4/L5 have no closed-form saturation point, so
the diagram boundary SHALL be computed numerically. The true boundary is generally
**non-convex** — the re-entrant corner is precisely what distinguishes terminal plow from
terminal spin — so any support-function or convex-hull construction is forbidden: sweeping
360 directions and taking the point of maximum projection returns exactly the convex hull's
extreme points and provably cannot see a re-entrant notch.

The correct construction exploits the fact that the sweep is a *parameterised map*. The
diagram is the image of the rectangle `[β_min, β_max] × [δ_min, δ_max]` under
`(β, δ) ↦ (Ā_Y, C_N)`; its outer edge is the image of the rectangle's **boundary**, traced
in order. The algorithm SHALL therefore:

1. Walk the four edges of the `(β, δ)` rectangle, evaluating (10.4) at each node, to obtain
   a closed polyline in the `(Ā_Y, C_N)` plane.
2. Refine adaptively: subdivide any segment whose chord-to-arc deviation exceeds a
   tolerance (default `1e-3` in `C_N` units), and any segment spanning a declared tire
   slope-break (§6.1 T7).
3. Detect and resolve self-intersections of that polyline, which occur when the map is not
   injective on the boundary; the outer loop is the diagram boundary.

The front and rear construction lines are obtained by the same sweep with the other axle's
force set to zero, which reduces to (10.5) for L2 and generalises correctly for the rest.

### 10.5 Derived metrics

**Trim line.** The locus `C_N = 0`. Found per `δ` line by Brent root-finding in `β` on the
bracketing interval, to `1e-10` absolute in `C_N`. Where the tire model is only C⁰ (§6.1
T7), the bracket SHALL be subdivided at the declared slope-break points before rooting.

**Stability index (static directional stability).**

```
SI = ∂C_N/∂Ā_Y |_{δ = const, on the trim line}                                (10.6)
```

`SI < 0` is stable. Computed by central difference along the trim line with a step chosen
per §13.6, and cross-checked against the linear-range analytic value

```
SI_linear = N_β /(m g ℓ) · (∂β/∂Ā_Y)  =  N_β / ( ℓ · (C_F + C_R) )             (10.7)
```

**Control moment.**

```
CM = ∂C_N/∂δ |_{β = const}          (analytic in the linear range: −a C_F/(m g ℓ))
```

**Steering sensitivity** (industry convention, per 100° of steering wheel):

```
SS = 100 · ΔĀ_Y / Δδ_SW             [g per 100 deg SW]                        (10.8)
```

measured across the trim line at the stated `A_Y`.

**Sideslip sensitivity.**

```
SSl = Δβ / ΔĀ_Y                     [deg/g]                                   (10.9)
```

**Understeer gradient from the diagram.**

```
UG = (1/G_s)·(dδ_SW/dA_Y) − dδ_Ack/dA_Y ,      dδ_Ack/dA_Y = ℓ / V²          (10.10)
```

with `A_Y` in m/s², so `UG` is in rad per (m/s²). Expressed per **g** — which is how it is
always quoted — the Ackermann gradient is `ℓ·g/V²` [rad/g], and that is the form the
presentation layer uses. The legacy English-unit form
`UG[deg/g] = (dδ_SW/dA_Y)/G_s − 858·ℓ[ft]/V[mph]²` is recovered exactly by the unit layer:
`57.2958 × 32.17405 / 1.466667² = 856.970`, the source's rounded 858 being 0.120 % high.
The implementation SHALL use (10.10) and SHALL NOT hard-code 858.

**Limit classification.** Let `A` and `L` be the extreme left- and right-hand limit points
of the diagram boundary. Relative to the trim line:

```
limit point above trim line  ⇒ terminal spin  (limit oversteer)
limit point on   trim line   ⇒ terminal drift (limit neutral)
limit point below trim line  ⇒ terminal plow  (limit understeer)
```

reported per turn direction, together with two scalar measures: the trim-line-to-limit-
intersection distance, and the maximum residual yaw moment available from the rear when the
front tires are saturated (a measure of recoverability).

**Maximum trimmed lateral acceleration.** The largest `|Ā_Y|` on the trim line. Reported
with the `(β, δ)` at which it occurs.

### 10.6 Output

A `MomentDiagram` value type carrying: the `(β, δ)` grid with `(A_Y, C_N)` at each node;
the constant-`δ` and constant-`β` iso-curves; the trim line; the boundary hull; the
construction lines; and the metric block above. The plot is `C_N` on the vertical axis,
`A_Y` on the horizontal, with iso-`δ` and iso-`β` curves distinguished by line style, not
only by colour (accessibility, §16.4).

### 10.7 Consistency requirement I-4

The maximum trimmed `Ā_Y` from P3 SHALL agree with the steady-state limit from P1 for the
same vehicle and tire model to within 2 %. Larger disagreement indicates an inconsistency
between the two force paths and SHALL fail acceptance. (For context, [R280] p. 257 reports
103.4 vs 104.9 ft/s² — 1.4 % — for a 60-slug vehicle with `a = 4 ft`, `b = 6 ft` and its
"Race" tire; that tire's data is not reproduced here, so the vector is cited as precedent
for the 2 % criterion, not as a reproducible test case. The reproducible instance of I-4
uses the L2 tires of §17.6.)

---

## 11. Programs P4 / P5 — RCVD Speedway and Speedway 2

### 11.1 Purpose

A minimal closed-circuit exercise: circumnavigate a circular obstacle of known radius in
minimum time. P4 uses the P1 plant, P5 uses the P2 plant. The value is pedagogical — the
user tunes vehicle parameters and open-loop driver inputs and immediately sees the effect
on lap time — so responsiveness and clarity of failure modes matter more than fidelity.

### 11.2 Circuit definition

```jsonc
{ "kind": "constant_radius",
  "inner_radius": {"value": 50, "unit": "ft"},
  "outer_radius": {"value": 150, "unit": "ft"},     // optional outer wall
  "centre": [0.0, 0.0],
  "start": { "position": [...], "heading": {"value": 0, "unit": "deg"} } }
```

A compound circuit (`"kind": "sequence"`, a list of constant-radius and straight segments)
is an extension seam; Rev 1.0 implements `constant_radius` only.

### 11.3 Driver plan

Open loop: `initial_speed`, `steer` (any maneuver of §5.6), `start_position`,
`speed_mode`. There is **no** closed-loop driver in Rev 1.0. The absence of feedback is
what makes the exercise instructive — and what makes the initial transient decisive
(§11.6).

### 11.4 Lap timing

Lap completion is detected by the accumulated **path angle about the circuit centre**
reaching `2π`, not by re-crossing a start line (which is ambiguous for a spiralling path):

```
θ(t) = atan2( Y_E − Y_c , X_E − X_c )         unwrapped continuously
lap complete when  |θ(t) − θ(0)| ≥ 2π                                          (11.1)
```

The crossing time SHALL be located by **dense output** from the integrator (Hermite
interpolation of the state across the bracketing step) plus Brent refinement to `1e-9 s`,
never by taking the first step at which the condition holds. Reporting a lap time
quantised to the integration step is a defect.

### 11.5 Termination conditions

| Event | Condition | Result |
|---|---|---|
| `EV-LAP` | (11.1) satisfied | success, lap time reported |
| `EV-WALL-IN` | distance to centre < inner radius | crash, time to crash reported |
| `EV-WALL-OUT` | distance to centre > outer radius | off-course |
| `EV-SPIN` | \|β\| > 90° | spin |
| `EV-TIMEOUT` | t > calculation_length | did not finish |

Wall contact SHALL likewise be located by dense output and root refinement on the signed
distance function, and the run SHALL terminate at the refined instant.

### 11.6 The theoretical optimum and why it crashes

For P4 with a symmetric vehicle, the theoretical minimum-time solution places the CG at
mid-wheelbase, uses the maximum permitted cornering stiffness at both ends, sets the steer
angle to the Ackermann angle for the target radius, and sets the speed from the available
tire force. **This solution crashes into the inner wall.** The reason is a genuine and
instructive one: at `t = 0` the yaw rate and lateral velocity are zero, so the vehicle
requires a transient — of duration a few times the vehicle time constant (§8.6) — before it
reaches the steady-state circular path. During that transient the path is not concentric
with the intended circle, and the steady-state circle it eventually settles on is offset
inward.

The program SHALL make this diagnosable rather than mysterious: on `EV-WALL-IN` that is
attributable to the initial transient it SHALL emit hint `H-SPD-001` naming that cause and
suggesting the two remedies (reduce steer angle slightly, or start further from the wall),
together with the computed `τ` and the steady-state path radius the current inputs imply.

**Attribution test (normative).** The hint SHALL fire when **both** hold:

1. the contact occurs within the **first revolution** (accumulated path angle < 2π), and
2. the steady-state path radius the current inputs imply, `R_ss = V/(r/δ · δ)` from (8.15),
   is **at least the inner radius** — i.e. the vehicle *would* have cleared the wall once
   settled, so only the transient offset explains the contact.

When (2) fails the steer is simply too tight for the circuit and the hint SHALL NOT fire;
that is a different fault with different advice.

**Not `t < 5τ`.** An earlier form of this rule used a fixed multiple of the yaw time
constant, which misses the failure this section is about by two orders of magnitude. The
transient *causes* the inward offset within a few `τ`, but the offset circle's closest
approach to the wall occurs up to **half a revolution later**, so the governing time scale
is the lap period, not `τ`. Measured on the CH5 fixture at 40 ft/s on a 50 ft inner radius:
`τ_dominant` = 0.0593 s, so `5τ` = 0.297 s, while contact from the tangentially-started
optimum occurs at **1.77 – 3.33 s**, i.e. 30–56 `τ`, against a half-lap time of
`πR/V` = 3.93 s. A `5τ` window fires only when the vehicle is already pointed into the
wall at `t = 0`, which is a different — and much less instructive — failure.

### 11.7 Real-time display

The path SHALL be drawn progressively during integration at a fixed wall-clock frame rate
(default 60 Hz), decoupled from the integration rate by a bounded lock-free single-producer
single-consumer ring buffer. **The display SHALL NOT affect the numerical result**: frame
drops, resizing, or pausing the view change nothing in the computed trajectory (Rule A2).
A headless run through `rcvd-cli` SHALL produce bit-identical results to a displayed run —
this is an acceptance test, not an aspiration.

---

## 12. Program P6 — Ride and Roll Rate Calculator

A single-track (one axle) springing calculator, exact and algebraic. Despite being the
simplest program it is the one most used in anger, so its algebra is specified completely
and its two directions are specified as exact inverses.

### 12.1 Element relations

**Ride rate — tire in series with the wheel-centre spring rate.** Chassis-to-ground:

```
k_R = k_T · k_W / ( k_T + k_W )                                                (12.1)
```

Inverse, wheel-centre rate required for a desired ride rate:

```
k_W = k_R · k_T / ( k_T − k_R )              requires  k_R < k_T               (12.2)
```

**Guard.** (12.2) is singular at `k_R = k_T` and negative beyond. A desired ride rate at or
above the tire rate is unachievable with any finite spring: the tire alone is softer than
that. The solver SHALL raise `E-RR-010` with a message naming `k_T` as the ceiling, and
SHALL warn (`W-RR-011`) when `k_R > 0.8·k_T`, where the required `k_W` becomes extremely
sensitive to the assumed tire rate.

**Installation ratio.** Let `r_S` be the pivot-to-spring distance and `r_W` the
pivot-to-wheel-centre distance (renamed from the source's `a`, `b` to avoid collision with
the CG distances of §3.4). Moments about the pivot give `F_z·r_W = F_S·r_S`, and by similar
triangles the spring travel and wheel travel are related by `Δy/Δx = r_S/r_W`. Hence

```
k_W = F_z/Δx = k_S·(r_S/r_W)² = IR²·k_S ,      IR ≡ r_S/r_W = Δy/Δx            (12.3)
```

So `IR` is the **spring travel per unit wheel travel**, and `IR < 1` (spring inboard of the
wheel) gives a wheel-centre rate *softer* than the spring rate. The **square** is the point:
an installation-ratio error of 5 % is a 10 % rate error. Some texts define the motion ratio
as the reciprocal; the schema field is named `spring_travel_per_wheel_travel` so the
convention cannot be mistaken.

**Bell-crank / rocker installation** with rocker arm lengths `L₁` (wheel link) and `L₂`
(spring link) at link angles `θ₁`, `θ₂` and rocker rotation `φ_r`:

```
k_W / k_S = (L₂/L₁)² · cos²(θ₂ + φ_r) / cos²(θ₁ + φ_r)                         (12.4)
```

which reduces to (12.3) when `θ₁ = θ₂`. This is the source of rising-rate behaviour, and
therefore of ride-height-dependent roll stiffness: a nose-down attitude stiffens the front
and softens the rear, shifting roll-stiffness distribution forward. Rev 1.0 reports
`k_W/k_S` at a user-specified `φ_r` and its slope `d(k_W/k_S)/dφ_r`; it does not couple this
back into P2 (extension seam).

**Roll stiffness from a pair of wheel rates.** Under a roll angle `φ`, the outside wheel
deflects `+t·φ/2` and the inside `−t·φ/2`, so the restoring moment is

```
K_φ = ( t² / 4 ) · ( k_left + k_right )                                        (12.5)
```

which for the symmetric case `k_left = k_right = k` gives the familiar

```
K_φ = ½ · k · t²                                                               (12.6)
```

The general form (12.5) SHALL be implemented so that split-rate (left ≠ right) setups are
supported, with (12.6) as the tested special case.

**Track selection is not incidental.** The tire contribution uses the **wheel track**; the
ride-spring contribution uses the **spring track**. The anti-roll bar's contribution
`K_φaux` is specified **directly as a roll stiffness** in N·m/rad, chassis-to-wheel-centre,
so no track enters it — a bar's rate is naturally quoted that way and its own lever arms are
part of its design, not of the vehicle's track. For an independent suspension these are
equal; for a beam axle with inboard springs they are not. Using the wrong one is a
`(t_wheel/t_spring)²` error — 44 % for the 60 in / 50 in case in §17.4.

### 12.2 Direction A — components to totals

```
k_R      = k_T k_W /(k_T + k_W)                                     per wheel
K_φ,tire = (t_wheel²/4)(k_T,L + k_T,R)                              WC to ground
K_φ,spr  = (t_spring²/4)(k_W,L + k_W,R)                             chassis to WC
K_φ,c→WC = K_φ,spr + K_φ,aux
K_φ,total = ( 1/K_φ,c→WC + 1/K_φ,tire )^{-1}                        chassis to ground
                                                                               (12.7)
```

The series combination of the tire roll stiffness with the spring+bar roll stiffness is the
step most often omitted; it is why measured roll stiffness is always lower than the value
computed from springs and bars alone.

### 12.3 Direction B — totals to components

Given a desired chassis-to-ground ride rate `k_R` and a desired chassis-to-ground roll
stiffness `K_φ,des`:

```
k_W       = k_R k_T/(k_T − k_R)                              (12.2), guarded
K_φ,tire  = (t_wheel²/4)(k_T,L + k_T,R)
K_φ,spr   = (t_spring²/4)(k_W,L + k_W,R)
K_φ,c→WC  = ( 1/K_φ,des − 1/K_φ,tire )^{-1}                  requires K_φ,des < K_φ,tire
K_φ,aux   = K_φ,c→WC − K_φ,spr                                                 (12.8)
```

**Guards.** `K_φ,des ≥ K_φ,tire` is unachievable (the tires alone are softer in roll than
that) → `E-RR-020`. `K_φ,aux < 0` means the ride springs alone already exceed the desired
roll stiffness → `W-RR-021`, reported as a *negative* bar rate rather than clamped to zero,
because a negative value is meaningful design feedback (soften the springs, or accept more
roll stiffness).

**Exactness requirement I-5.** Direction B followed by Direction A SHALL return the
original totals to `1e-12` relative, and vice versa, across 10⁴ randomised admissible
inputs. This round-trip property is the acceptance test for P6.

### 12.4 Ride frequency

```
f = (1/2π) · √( k_R / m_corner ) ,        m_corner = W_axle / (2 g)            (12.9)
```

**Erratum E-2 (normative deviation from the source).** [R280] p. 258 computes the required
ride rate from `ω = √(K/m)` using the **whole-axle** mass with a **single-wheel** rate,
obtaining 163.6 lb/in front and 123.7 lb/in rear for stated 1.0 Hz and 1.1 Hz targets.
That pairing is dimensionally inconsistent: with per-corner mass `m_corner = W_axle/2g` and
the same 163.6 lb/in, the actual ride frequency is 1.4142 Hz — exactly `√2 ×` the stated
target. The correct per-corner rates for 1.0 Hz and 1.1 Hz are **81.8 lb/in** and
**61.9 lb/in**.

This specification implements (12.9). The published values remain usable as verification
data for the *rest* of the calculator chain (§17.4), because the error is confined to the
ride-frequency-to-rate step; every downstream number in [R280] Figs. A.7/A.8 is reproduced
exactly by §12.1–12.3 when 163.6 and 123.7 lb/in are supplied as *given* ride rates. A
`--legacy-ride-frequency` flag SHALL reproduce the source behaviour for traceability, and
the GUI SHALL show the computed frequency next to any user-entered rate so the
inconsistency cannot recur silently.

### 12.5 Roll gradient

```
RG ≡ φ̃ / A_Y = m_s h₂ / ( K_φF + K_φR − m_s g h₂ )        [rad per (m/s²)]   (12.10)
```

where `φ̃` is the outward-lean magnitude-with-sign of (9.5). The SAE roll angle is
`φ = −φ̃`; roll gradient is conventionally quoted as a positive number, so `RG` is defined
on `φ̃`. This is the same expression as (9.5) — one definition, two places, no sign
discrepancy.

For the design target, the required total roll stiffness follows by inversion:

```
K_φF + K_φR = m_s h₂ / RG_target  +  m_s g h₂                                 (12.11)
```

The commonly used shortcut drops the second term (equivalently, uses `ΣK` rather than
`ΣK − m_s g h₂` in the denominator of (12.10)):

```
(K_φF + K_φR)_simplified = m_s h₂ / RG_target                                 (12.12)
```

Both SHALL be reported, together with their difference `m_s g h₂`, so the user sees the
size of the gravity-term correction rather than having it applied invisibly. Because
(12.11) and (12.12) differ only by that one term — same `m_s`, same `h₂` — the difference
isolates exactly what it claims to.

---

## 13. Numerics

### 13.1 Floating point

All computation in IEEE 754 `binary64`. No `-ffast-math`. Comparisons against zero SHALL use
explicit tolerances scaled to the quantity, never bare equality. `NaN` SHALL be treated as
a fatal defect, not a value: every solver SHALL check its output for finiteness and abort
with the state and step at which non-finiteness first appeared.

### 13.2 Integrators

| Name | Order | Use |
|---|---|---|
| `rk4_fixed` | 4 | default; matches source behaviour; deterministic cost |
| `rk4_richardson` | 4 + error estimate by step doubling | fixed step with reported error |
| `dopri54` | 5(4) embedded, adaptive, dense output | recommended for analysis and for lap timing |

`dopri54` SHALL provide 4th-order dense output (the standard Dormand–Prince interpolant)
for event location (§11.4) and for output resampling at `output_record_increment` without
re-integration.

Step acceptance for `dopri54`:

```
err = ‖ (y5 − y4) / (atol + rtol·max(|y_n|,|y_{n+1}|)) ‖_∞
accept if err ≤ 1;   h_new = h · clamp( 0.9·err^{-1/5}, 0.2, 5.0 )            (13.1)
```

State scaling SHALL be applied before the norm: `v` in m/s, `r` in rad/s and `X_E` in m
have wildly different magnitudes, and an unscaled norm silently lets the position states
dominate the error control. Per-state `atol` SHALL be configurable, defaulting to
`[1e-8 m/s, 1e-9 rad/s, 1e-9 rad, 1e-6 m, 1e-6 m]`.

### 13.3 Step-size guidance (normative defaults and warnings)

The step must resolve the fastest mode present, not merely the slowest:

| Mode | Time scale | Implied `Δt` (≥10 points/cycle) |
|---|---|---|
| Vehicle yaw/sideslip | period `2π/ω_n`, typically 0.4–2 s | `≤ 2π/(10 ω_n)` ≈ 40–200 ms |
| Steering input at frequency `f` | period `1/f` | `≤ 1/(10 f)` |
| Tire relaxation | `τ = σ/u`, 3–100 ms | `≤ τ/5` = 0.6–20 ms |

The binding constraint is normally the tire relaxation lag when it is enabled, and the
steering input frequency otherwise. Note that these are *resolution* limits; accuracy of a
4th-order method at a given tolerance may demand a smaller step still, which is what the
adaptive controller of (13.1) is for.

The solver SHALL compute `ω_n` from the linearised system at the initial condition and, if
`Δt > 2π/(10·ω_n)` or `Δt > 1/(10·f_input)` or (relaxation enabled) `Δt > τ/5`, SHALL emit
`W-NUM-200` naming the limiting mode and the suggested step. The source material's own
finding — that a 5 Hz sinusoidal steer input (period 0.2 s) sampled at `Δt = 0.1 s` yields only two
samples per period, both landing where the input is zero, producing a completely spurious
result — is the
canonical failure this warning prevents. Beating and aliasing (input periods of 0.205 s or
0.051 s at `Δt = 0.1 s`) are the same defect in other clothing.

For C⁰ tire models (§6.1 T7) with a fixed-step integrator, the classical order is lost
whenever a step brackets a slope break. `dopri54` SHALL be recommended in that
configuration, and `rk4_fixed` SHALL emit `W-NUM-210`.

### 13.4 Stiffness

With tire relaxation enabled at high speed the system is mildly stiff: `τ_tire ≈ 3 ms`
against `τ_vehicle ≈ 100 ms`, a ratio of ~30. Explicit methods remain adequate at that
ratio; below `u ≈ 3 m/s` with short relaxation lengths the ratio can exceed 100 and an
explicit fixed step becomes wasteful. Rev 1.0 handles this by the `speed_floor` bypass of
§6.9 rather than by an implicit integrator. If a future revision models damper or
compliance dynamics, an L-stable implicit method (SDIRK or BDF with an analytic Jacobian)
becomes necessary; the `Plant` interface reserves a `jacobian()` hook for that purpose.

### 13.5 Interpolation

Monotone cubic Hermite (Fritsch–Carlson) everywhere table data is interpolated: tire data
(§6.4), driver-input tables (§5.6), aerodynamic maps. Rationale: it is C¹ — so numerical
Jacobians and the MMM contouring behave — and shape-preserving, so it introduces no
overshoot near a tire's force peak, which a natural cubic spline would.

### 13.6 Numerical differentiation

Where an analytic derivative is unavailable, a central difference with Richardson
extrapolation SHALL be used, with step

```
h = max( |x| , x_typ ) · eps^(1/3) ,     eps = 2.22e-16                       (13.2)
```

(`eps^(1/3) ≈ 6.06e-6`, the truncation/round-off optimum for a central difference). A
one-sided difference SHALL be used only at a declared slope break, and the choice recorded
in the output.

### 13.7 Root finding

Brent's method (bisection + secant + inverse quadratic) on a verified bracket, absolute
tolerance `1e-12` on the independent variable or `1e-10` on the residual, whichever is
reached first, maximum 100 iterations. If no sign change exists on the supplied interval
the solver SHALL report `E-NUM-300` with the interval and endpoint values rather than
returning an endpoint.

### 13.8 Linear algebra

2×2 and 4×4 systems: LU with partial pivoting; the reciprocal condition number SHALL be
computed and returned with every solve. Eigenvalues of the 2×2 bicycle-model `A`: closed
form from the discriminant, with the trace/determinant cross-check of §8.6.

### 13.9 Conditioning and the physical singularities

The following are physical, not numerical, singularities and SHALL be detected and named
rather than allowed to produce large numbers:

| Condition | Detection | Error |
|---|---|---|
| `V → 0` in slip-angle or derivative expressions | `V < speed_floor` | `E-NUM-310` |
| `1 + K V² → 0` (at/above critical speed) | `rcond(A) < 1e-12` | `E-SS-030` |
| `K_φF+K_φR → m_s g h₂` (roll divergence) | ratio < 1.05 | `E-VEH-140` |
| `k_R → k_T` in (12.2) | ratio > 0.999 | `E-RR-010` |
| `K_φ,des → K_φ,tire` in (12.8) | ratio > 0.999 | `E-RR-020` |
| `F_z → 0` in tire normalisation | `F_z < 1 N` | return zero force (T4) |
| `C_LF + C_LR → 0` in aero balance | \|sum\| < 1e-9 | report `n/a` |

---

## 14. Errors, Diagnostics and Guards

### 14.1 Policy

Three severities:

- **Error (`E-*`)** — the computation cannot proceed correctly. The run aborts and returns a
  failed `Result<T>` carrying a `Diagnostic` (§4.6, Rule A3). No partial result is
  presented as a result.
- **Warning (`W-*`)** — the computation proceeds but a modelling assumption is being
  stretched. Warnings are written to the manifest **and** displayed; they are not
  suppressible below a "shown once per run" level.
- **Hint (`H-*`)** — pedagogical guidance attached to an outcome (e.g. §11.6).

**Rule D1.** No silent clamping. Every clamp (wheel load to zero, friction radicand to zero,
`μ` to `μ_min`) SHALL raise an event the first time it fires in a run, with the time and
the wheel/axle identity.

**Rule D2.** No silent extrapolation. Tire and aero data used beyond their declared valid
range SHALL warn on first occurrence and record the maximum excursion.

**Rule D3.** Validation happens once, at construction, on a fully-resolved SI input
document. Solvers do not re-validate per step; they assert.

### 14.2 Register (abridged)

| Code | Meaning |
|---|---|
| `E-VEH-010` | Over-specified geometry, inertia or roll-stiffness group |
| `E-VEH-011` | Sprung/unsprung consistency (5.1a)–(5.1c) violated |
| `E-VEH-140` | Roll divergence: `K_φF + K_φR ≤ 1.05·m_s g h₂` |
| `E-TIRE-003` | Positive cornering stiffness supplied |
| `E-TIRE-004` | Quadratic `C_α(F_z)` changes sign within the operating load range |
| `E-TIRE-005` | Tire document supplies two independent sources of cornering stiffness |
| `E-TIRE-006` | Friction block supplied for a model that declares `has_mu() == false` (§6.1 T5) |
| `E-TIRE-020` | Load outside tabulated range by more than 25 % |
| `E-SS-030` | Steady-state system singular (at or above critical speed) |
| `E-P2-050` | Load-transfer fixed point failed to converge |
| `E-RR-010` | Desired ride rate ≥ tire rate |
| `E-RR-020` | Desired roll stiffness ≥ tire roll stiffness |
| `E-NUM-300` | Root bracket contains no sign change |
| `E-NUM-310` | Speed below floor in a speed-normalised expression |
| `W-TIRE-101` | Load beyond the maximum of the quadratic `C_α(F_z)` |
| `W-VEH-120` | Roll-axis inclination beyond the small-angle assumption |
| `W-VEH-141` | Roll gradient outside the physical band of (9.10c) — suspect `K_φ` units |
| `W-NUM-200` | Integration step too large for the fastest mode present |
| `W-NUM-210` | Fixed-step integrator with a C⁰ tire model |
| `W-NUM-220` | `small_angle_slip` forced on with `\|v/u\|` beyond `small_angle_limit` (§3.3) |
| `W-RR-011` | Desired ride rate above 80 % of tire rate (ill-conditioned) |
| `W-RR-021` | Required anti-roll bar rate is negative |
| `EV-LIFT` | Wheel load reached zero |
| `EV-SPIN` | Vehicle sideslip exceeded 90° (§11.5) |
| `EV-WHEEL-STALL` | A wheel's longitudinal velocity fell below the speed floor (§9.6) |
| `EV-LAP` | Lap completed (§11.5) |
| `EV-WALL-IN` | Contact with the inner boundary |
| `EV-WALL-OUT` | Departure past the outer boundary |
| `EV-TIMEOUT` | Calculation length reached without lap completion |
| `EV-CLAMP-MU` | Friction coefficient clamped to `μ_min` |
| `EV-CLAMP-ELLIPSE` | Friction-ellipse radicand clamped to zero |
| `H-SPD-001` | Inner-wall contact attributable to the initial yaw transient |

---

## 15. Output and Reporting

### 15.1 Tabular output

CSV (RFC 4180) and Parquet. One row per output record. Header carries the channel name and
the unit in the active display system, e.g. `A_Y [g]`, `Fz_LF [lbf]`. Floating point written
with `%.17g` in CSV so that a round trip is lossless. Every file is accompanied by the run
manifest of §4.5.

Output records are produced at `output_record_increment` by **dense-output interpolation**
of the integrator, not by forcing the step size to land on record times, so that record
frequency and accuracy are independent controls.

### 15.2 Plotting

Any channel against any channel. Defaults per program: P1 path (with the axis convention of
§8.7), P2 wheel loads vs time, P3 the `C_N–A_Y` diagram, P4/P5 the real-time path, P6 a
stiffness breakdown bar chart. Square-axis mode SHALL be available and SHALL be the default
for any plot whose two axes carry the same unit.

### 15.3 Diagnostics panel

A permanently visible panel listing, for the current run: active warnings, the integrator
statistics (steps accepted/rejected, minimum and maximum step, error norm), the
`τ_tire/τ_vehicle` ratio, the fixed-point contraction factor `|Φ'|` from §9.7, and the
condition numbers from every linear solve. Numerical health is a first-class output, not
a debug log.

---

## 16. Graphical User Interface

### 16.1 Structure

Qt 6 Widgets, one window per program, launched from a common shell. Each program window
has: **Define** (vehicle, tires, aero, environment), **Run** (maneuver, solver config),
**Results** (tabular + plots), **Diagnostics** (§15.3).

### 16.2 Thinness (restating Rule A1)

The GUI marshals values into the core's value types and renders `Result`. It contains no
equation. Any GUI action maps to a `rcvd-cli` invocation, and the window SHALL be able to
display the equivalent CLI command for the current configuration — which makes every GUI
result reproducible and scriptable.

### 16.3 Input behaviour

- Every numeric field carries an explicit unit selector; there are no implied units.
- Fields inert for the current model (e.g. CG height in P1) are disabled with a tooltip
  saying why, never merely ignored.
- Validation is live: an out-of-range or over-specified input is flagged at the field, with
  the same error code the core would raise.
- The four default vehicles (Sedan, Formula One, NASCAR-style stock car, and one further
  configuration) are provided as **editable, versioned JSON documents shipped with the
  product**, not as hard-coded constants. Their parameter values are the vendor's own; they
  are not reproduced from the source CD.

### 16.4 Accessibility and plotting hygiene

Curves SHALL be distinguishable by line style and marker as well as colour (deuteranopia-
safe palette). Axis labels SHALL always carry units. Text and gridlines SHALL meet WCAG AA
contrast against the plot background — in particular, no axis label, series label or
annotation SHALL be drawn in a colour close in luminance to the background it sits on.

---

## 17. Verification and Validation

### 17.1 Test architecture

Four tiers, all in CI:

1. **Unit** — every formula in §6–§12 against a hand-computed value.
2. **Invariant** — the identities I-1 … I-5 and tire contracts T1 … T7, each over ≥200
   randomised admissible parameter sets (property-based testing).
3. **Acceptance** — the published test vectors of §17.2–§17.6, each traced to a page in
   [R280].
4. **Regression** — golden output files with SHA-256, regenerated only by explicit approval.

**Tolerance policy.** Acceptance tolerances are stated per vector and are dominated by the
precision to which the source published the answer (typically 3–4 significant figures) and
by its use of `g = 32.2 ft/s²` (0.08 % high relative to standard gravity). A default
relative tolerance of `3e-3` applies unless stated; invariant tests use `1e-12`.

### 17.2 P1 — Bicycle Model acceptance vectors

Reference vehicle **CH5** ([R280] p. 51): ℓ = 104.5 in, W = 3700 lbf, 57 % front,
`C_F = −640 lbf/deg`, `C_R = −580 lbf/deg`, `h` = 20 in. The source does not state a yaw
inertia (Ch. 5 is steady-state throughout); the suite's CH5 fixture adopts
`I_z = 1800 slug·ft²`, which affects only the transient vectors, never B-01 … B-19.
Vector **CH5-OS** is CH5 with the weight distribution reversed to 43 % front.
Vectors B-13/B-14 additionally use the [R280] Ch. 5 Prob. 5 saturating tire
(breakaway `α_bk = 5°` at both axles) on a skidpad of radius **R = 100 ft**; B-15/B-16 use
`R = 50 ft` and a 14:1 steering ratio.
All values below were reproduced by an independent SI implementation of §8; the
"computed" column is that result, the "published" column is [R280].

| ID | Quantity | Published | Computed (SI, converted) | Rel. diff | Source |
|----|----------|-----------|--------------------------|-----------|--------|
| B-01 | m [slug] | 115 | 115.000 | 4e-6 | p. 59 |
| B-02 | a, b [ft] | 3.74, 4.96 | 3.7446, 4.9637 | 1.2e-3 | p. 59 |
| B-03 | Y_β [lbf/deg] | −1220 | −1220.00 | 0 | p. 60 |
| B-04 | Y_r [lbf/(deg/s)] | +10.98 | +10.965 | 1.4e-3 | p. 60 |
| B-05 | Y_δ [lbf/deg] | +640 | +640.00 | 0 | p. 60 |
| B-06 | N_β [lbf·ft/deg] | +483.2 | +482.44 | 1.6e-3 | p. 60 |
| B-07 | N_r [lbf·ft/(deg/s)] | −527.7 | −528.74 | 2.0e-3 | p. 60 |
| B-08 | N_δ [lbf·ft/deg] | +2393.6 | +2396.53 | 1.2e-3 | p. 60 |
| B-09 | K [s²/ft²] | 3.45e−5 | 3.4399e−5 | 2.9e-3 | p. 61 |
| B-10 | V_ch [ft/s] | 170.2 | 170.50 | 1.8e-3 | p. 61 |
| B-11 | NSP, SM | 0.475, 0.045 | 0.47541, 0.04541 | 9e-3 (SM) | p. 62 |
| B-12 | Neutral-steer `a` [ft] | 4.14 | 4.1400 | 7e-6 | p. 58 |
| B-13 | Skidpad limit A_Y [g] | 1.51 | 1.5173 | 4.8e-3 † | p. 57 |
| B-14 | Skidpad limit V [ft/s] | 69.8 | 69.870 | 1.0e-3 | p. 57 |
| B-15 | δ_Ack, R = 50 ft [deg] | 9.98 | 9.9790 | 1.1e-4 | p. 55 |
| B-16 | δ_SW at 14:1 [deg] | 139.7 | 139.706 | 4e-5 | p. 55 |
| B-17 | Crowned road, 5° tilt: β, δ [deg] | 0.239, −0.048 | 0.2392, −0.0480 | <2e-3 | p. 57 |
| B-18 | Crosswind 100 lbf @1.5 ft: F_yF, F_yR [lbf] | −74.3, −25.7 | −74.225, −25.775 | 1.0e-3 | p. 56 |
| B-19 | 43/57 reversed: N_β [lbf·ft/deg] | −1005.2 | −1004.94 | 2.6e-4 | p. 61 |

† B-13: the published 1.51 is a truncation of 1.517 obtained from the source's own rounded
`a`, `b`; the tolerance for this vector is set to 6e-3 and the reason recorded.

**B-18 is pure statics** — no `g`, no tire model, no speed. With the side force applied
`d` = 1.5 ft ahead of the CG, moment balance about each axle in turn gives
`F_yF = −F·(b + d)/ℓ` and `F_yR = −F·(a − d)/ℓ`, i.e. −74.2249 and −25.7751 lbf for
`ℓ` = 104.5 in and 57 % front. The two SHALL sum to `−F` and SHALL satisfy
`a·F_yF − b·F_yR = −F·d` to machine precision; a computed pair that misses the second
identity has the wrong moment arm, which is the failure this vector exists to catch.

Reference vehicle **"First Example"** ([RCVD] p. 256, [R280] p. 74): W = 2000 lbf,
ℓ = 8.33 ft, `I_z` = 832 slug·ft², 65 % front, `C_F = −240`, `C_R = −200` lbf/deg,
V = 40 mph.

| ID | Quantity | Published | Computed | Rel. diff |
|----|----------|-----------|----------|-----------|
| B-20 | Characteristic equation | s² + 16.188 s + 87.698 | s² + 16.190 s + 87.813 | 1.3e-3 |
| B-21 | Poles | −8.094 ± 4.710i | −8.0951 ± 4.7204i | 2.1e-3 |
| B-22 | ω_n [rad/s] | 9.36 | 9.3709 | 1.2e-3 |
| B-23 | ζ | 0.865 | 0.86386 | 1.3e-3 |
| B-24 | r/δ numerator [per deg] | 0.84 s + 7.57 | 0.8410 s + 7.5504 | 2.6e-3 |
| B-25 | DC gain (8.15) vs (8.28) | — | identical to 1.8e-16 | invariant |

Additional invariant vectors:

| ID | Check | Result |
|----|-------|--------|
| B-26 | Identity I-1: `K·ℓ·g` vs `W_F/\|C_F\| − W_R/\|C_R\|` | agree to 1e-12; 9.63787e−3 rad/g = 0.5522 deg/g |
| B-27 | Identity I-2: (8.15) vs (8.25) for CH5 at 44 ft/s | both 4.73716 s⁻¹, agree to 1.9e-16 |
| B-28 | Sign table §3.3 reproduced for a right-hand turn, run **above** `V_tan` (`v` and `β` negative) and again **below** it (both positive) | all 13 signs, both cases |
| B-29 | Tilted-road answer independent of speed | Δ < 1e-14 over 5–100 m/s |
| B-30 | Yaw-rate coefficient identity `r = 84·V/R` (deg/s, mph, ft) | computed 84.034 vs 84 ([R280] p. 64) |
| B-31 | Sign-equivalence of `det A` and `1 + K V²`, vehicle CH5-OS | both positive at 100 ft/s, both negative at 170 ft/s; the sign change brackets `V_crit` |

### 17.3 P1 behavioural acceptance (source exercises A.2)

| ID | Exercise | Expected behaviour |
|----|----------|--------------------|
| X-01 | Default run, default plot | Path is a circle after the initial transient; steady state reached ≈0.20 s; square-axis default renders it circular |
| X-02 | Axis swap | Right-hand turn renders as a left-hand turn — the convention warning of §8.7 |
| X-03 | 5° step steer | Yaw rate reaches 98 % of steady state at 0.175 s ⇒ empirical τ = 0.044 s; the dominant-pole τ SHALL also be reported and SHALL differ |
| X-04 | 5° step steer above tangent speed | β, v and α_R initially move opposite to their steady-state sign, then reverse (nose-out → nose-in) |
| X-05 | Same at 20 ft/s (below V_tan) | No sign reversal of β |
| X-06 | Step-size sweep 1e−5 … 1 s, fixed steer | Final position after 4 s differs by only ≈5 ft between 0.001 and 0.05 s; below 0.001 s no meaningful accuracy gain |
| X-07 | Same with 5 Hz sinusoidal steer | Δt = 0.1 s produces a spurious/null result; `W-NUM-200` SHALL fire |
| X-08 | Steer period 0.205 s at Δt = 0.1 s | Beating visible in the steer-vs-time plot |
| X-09 | Steer period 0.051 s at Δt = 0.1 s | Aliasing: apparent period far longer than input |
| X-10 | 20° road tilt, δ = 0, neutral-steer vehicle | Straight path at an angle; β ≡ 0; `N_β` reported as 0 |
| X-11 | Same with (a,b) = (4,6) ft then (6,4) ft | Understeer path, then oversteer path (turns up-slope) |
| X-12 | Understeer car, low speed, constant acceleration | Spirals outward; rear slip angle saturates at the breakaway value |
| X-13 | Oversteer car, a = 5.1, b = 4.9 ft | Spirals inward until both axles saturate; SHALL raise `EV-SPIN` and SHALL warn that the open-loop model is not valid past that point |

### 17.4 P6 — Ride and Roll Rate acceptance vectors

Source case ([R280] pp. 258–260): tracks 60 in front and rear, independent front, beam rear
with 50 in spring track, 1600 lbf front axle, 1000 lbf rear axle, tire rate 1500 lbf/in per
tire, CG height 18 in, roll axis on the ground, target roll gradient 3.0 deg/g with 75 % of
roll stiffness at the front.

| ID | Quantity | Published | Computed | Rel. diff |
|----|----------|-----------|----------|-----------|
| R-01 | Total roll stiffness from a 3.0 deg/g target — **simplified** (12.12) | 1300 | 1300.00 | 2e-16 |
| R-01b | Same target — **exact** (12.11), including the `m_s g h₂` = 68.07 lbf·ft/deg gravity term | (not published) | 1368.07 | — |
| R-02 | Front / rear split [lbf·ft/deg] | 975 / 325 | 975 / 325 | exact |
| R-03 | Front wheel-centre rate [lbf/in] | 183.627 | 183.628 | 3.6e-6 |
| R-04 | Front auxiliary roll stiffness [lbf·ft/deg] | 816.293 | 816.292 | 7.8e-7 |
| R-05 | Roll stiffness due to tires [lbf·ft/deg] | 3926.98 | 3926.99 | 2.8e-6 |
| R-06 | Front roll stiffness due to ride springs [lbf·ft/deg] | 480.735 | 480.736 | 2.2e-6 |
| R-07 | Rear wheel-centre rate [lbf/in] | 134.818 | 134.818 | 7.2e-8 |
| R-08 | Rear auxiliary roll stiffness [lbf·ft/deg] | 109.218 | 109.218 | 3.5e-7 |
| R-09 | Rear roll stiffness due to ride springs [lbf·ft/deg] | 245.105 | 245.106 | 4.2e-6 |

**R-01 vs R-01b.** The source computes the required roll stiffness as `W·h/φ_target`, i.e.
(12.12), which omits the gravity term. Feeding its 1300 lbf·ft/deg back through the exact
roll-gradient relation (12.10) gives **3.166 deg/g**, not the 3.0 deg/g asked for. This is
not an erratum in the calculator — every downstream number in R-03 … R-09 follows correctly
from 1300 as a *given* total — but it is a 5.5 % miss on the design target, and it is why
cross-program check C-04 (§17.8) SHALL be run against R-01b, not R-01. P6 SHALL report both
(12.11) and (12.12) side by side (§12.5) so the user chooses knowingly.

These nine values reproduce the two calculator screens of [R280] Figs. A.7 and A.8 to
better than 5 significant figures using only (12.1)–(12.8), which constitutes a complete
verification of the P6 algebra including the wheel-track/spring-track distinction (note
R-05 uses the 60 in wheel track at both ends while R-09 uses the 50 in rear spring track).

**Erratum check E-2.** With the correct per-corner mass (12.9), the ride rates implied by
the stated 1.0 Hz / 1.1 Hz targets are 81.8 and 61.9 lbf/in; the published 163.6 and 123.7
lbf/in correspond to 1.4142 Hz and 1.5556 Hz. A dedicated test SHALL assert both the
correct values and the legacy values under `--legacy-ride-frequency`.

Additional P6 vectors:

| ID | Case | Published | Computed |
|----|------|-----------|----------|
| R-10 | `K_φaux` for t = 5 ft, `k_W` = 150 lbf/in, target `K_φ` = 1000 lbf·ft/deg, **tires assumed rigid** (`k_T → ∞`, so the series step of (12.8) degenerates and chassis-to-ground = chassis-to-WC) | 607.3 | 607.301 |
| R-11 | Ride rate from `k_T` = 1500, `k_W` = 150 lbf/in | 136.36 | 136.364 |
| R-12 | Same with `k_T` = 1550 | 136.76 | 136.76 |
| R-13 | Equivalent `k_W` giving 136.76 with `k_T` = 1500 | 150.5 | 150.49 |
| R-14 | Round-trip A∘B = identity, 10⁴ random cases | — | max rel. error 4e-16 |

R-11 → R-13 demonstrate the practically important insensitivity of ride rate to tire rate:
a 50 lbf/in tire-rate change (≈1 psi) moves the ride rate by 0.4 lbf/in, equivalent to a
0.5 lbf/in change in wheel-centre rate. The suite SHALL reproduce this, because it is the
quantitative basis for the statement that tire pressure does not tune ride rate.

### 17.5 P2 — Four-Wheeler acceptance vectors

| ID | Quantity | Published | Computed | Source |
|----|----------|-----------|----------|--------|
| F-01 | Static loads, 3700 lbf, 57/43 [lbf] | 1054.5 / 795.5 | exact | p. 214 |
| F-02 | Longitudinal transfer, h/ℓ = 20/104.5, 0.4 g [lbf] | 283.2 | 283.25 | p. 214 |
| F-03 | Lateral transfer, h/t = 20/60, 0.4 g [lbf] | 493.3 | 493.33 | p. 214 |
| F-04 ‡ | Banked corner, **W = 3500 lbf**, 150 mph, R = 600 ft, 20° bank: A_Y [g] | 2.50 | 2.5072 | p. 215 |
| F-05 ‡ | Same: normal load W_e [lbf] | 6290 | 6290.2 | p. 215 |
| F-06 ‡ | Same: required lateral force [lbf] | 7050 | 7048.9 | p. 215 |
| F-07 | Pair analysis, FLT front at 0.35 g, h_e/t = 16.5/61.5 (W = 3700 lbf, 57 % front ⇒ W_F = 2109 lbf) | 0.188 | 0.1878 | p. 90 |
| F-08 | Pair analysis, inside-rear lift at h_e/t = 23.5/57.8 [g] | 1.23 | 1.2298 | p. 91 |
| F-09 | Sustainable A_Y, front axle: η = 16.5/61.5, pW = 2109 lbf, μ = 1.22 − 2.4e−4·F_z | 0.907 | 0.90698 | p. 92 |
| F-10 | Sustainable A_Y, rear axle: η = 23.5/57.8, pW = 1591 lbf | 0.922 | 0.92181 | p. 92 |
| F-11 | Invariant I-3 (9.10) over 2000 random parameter sets | — | max rel. residual 6.0e−16 |
| F-11b | Invariant I-3b (9.10b) over 2000 random parameter sets, both signs of A_Y | — | max rel. residual < 1e−15; transfers exactly odd in A_Y |
| F-12 | Load-transfer gradient sweep, **Case 1** (below), `z_RF` 0→16 in ([R280] Fig. 18.2) | ≈300→440 F, ≈370→232 R | 313.7→437.5 F, 380.4→237.5 R |

‡ **F-04 … F-06 are not P2 model outputs.** The §9.3 load model of Rev 1.0 is flat-surface
only: Step 1 takes the static loads as `m g·b/ℓ` with no `cos θ_t`, and `road_tilt` enters
the suite solely through the `m g sin θ_t` term of (9.11). There is therefore no path by
which P2 as specified produces a banked-corner wheel load. These three vectors verify the
**closed-form pair-analysis** relations

```
Ā_Y = V²/(g R) ,   W_e = W·(cos θ_t + Ā_Y sin θ_t) ,   F_y,req = W·(Ā_Y cos θ_t − sin θ_t)
                                                                               (17.2)
```

which the implementation SHALL provide as an analytic utility alongside P2, exactly as
F-09/F-10 are provided for (17.1). Banked-corner load transfer through the full §9.3 chain
is extension seam 11 (§20). Until then the GUI SHALL disable `road_tilt` for P2 wheel-load
output, or state that the reported loads are the flat-surface values.

**F-12 Case-1 vehicle (fully specified).** `W = W_s = 2000 lbf` (no unsprung mass),
`t_F = t_R = 60 in`, `a_s/ℓ = 0.50`, `h_s = 20 in`, `z_RR = 4 in`, `K_φF = K_φR` with
`ΣK = 1200 lbf·ft/deg` (chassis-to-ground), `z_RF` swept 0 → 16 in. Using (9.4a)–(9.6b):

| `z_RF` [in] | `h₂` [in] | roll gradient [deg/g] | `ΔF_z,F/A_Y` [lbf/g] | `ΔF_z,R/A_Y` [lbf/g] |
|---|---|---|---|---|
| 0 | 18.0 | 2.614 | 313.7 | 380.4 |
| 4 | 16.0 | 2.312 | 344.1 | 344.1 |
| 8 | 14.0 | 2.013 | 374.9 | 308.2 |
| 16 | 10.0 | 1.423 | 437.5 | 237.5 |

against the plotted ≈300 → 440 (front) and ≈370 → 232 (rear). The published four-term
decomposition, with its gravity term subtracting and distributed by sprung-weight fraction,
gives 286.3 → 429.2 and 353.0 → 229.2 — further from the plotted curve in **aggregate**:
summed absolute deviation across the four plotted corners is **32.0 lbf/g for (9.6a)–(9.6b)
against 44.3 for the published form**.

**Not, however, at every point**, and an implementation SHALL NOT assert that it is:

| `z_RF` | axle | (9.6a)–(9.6b) | published | plotted | \|err\| correct | \|err\| published |
|---|---|---|---|---|---|---|
| 0 in | front | 313.7 | 286.3 | ≈300 | 13.7 | 13.7 |
| 0 in | rear | 380.4 | 353.0 | ≈370 | 10.4 | 17.0 |
| 16 in | front | 437.5 | 429.2 | ≈440 | 2.5 | 10.8 |
| 16 in | rear | 237.5 | 229.2 | ≈232 | 5.5 | **2.8** |

The front pair at `z_RF` = 0 is an exact tie, and at `z_RF` = 16 in the rear the published
form is the nearer of the two. Both gaps are a few lbf against values read off a plotted
figure ([R280] Fig. 18.2), whose read-off error is comparable, so neither corner is
evidence either way.

**The decisive evidence for E-1a is therefore I-3, not this curve.** The published gravity
sign violates the whole-vehicle roll-moment balance (9.10) with a residual of order 1 —
exactly, analytically, and independently of anything read off a page. F-12 corroborates in
aggregate; it does not carry the argument alone. The acceptance test SHALL assert the
aggregate deviation and the I-3 residual, never a pointwise ordering.

F-09/F-10 use the closed-form solution of the load-sensitive pair equation, which the
implementation SHALL also reproduce as a limiting analytic check on the numerical solver.
Here `μ₀`, `μ₁` are the **friction** coefficients of (6.9) — not the cornering-stiffness
coefficients `c0`, `c1` of (6.10), which have different dimensions:

```
( 2 μ₁ η² p W ) Ā_Y² + Ā_Y + ( μ₁ p W / 2 − μ₀ ) = 0 ,     η = h_e/t ,  p = axle weight
                                                                        fraction        (17.1)
```

F-12 **corroborates** errata E-1a and E-1b (§9.3, §19); the primary evidence is the exact
I-3 moment-balance violation, per the note above.

### 17.6 P3 — Moment Method acceptance vectors

60-slug vehicle, ℓ = 10 ft, linear-with-breakaway tires, `α_bk` = 6°, V = 30 mph.

| Case | a, b [ft] | C_F, C_R [lbf/deg] | Front line A_Y [g] / C_N | Rear line A_Y [g] / C_N | Character |
|---|---|---|---|---|---|
| a | 5, 5 | −420, −420 | ±1.30 / ±0.65 | ±1.30 / ∓0.65 | terminal drift |
| b | 6, 4 | −420, −420 | ±1.30 / ±0.78 | ±1.30 / ∓0.52 | terminal **spin** (see below) |
| c | 4, 6 | −420, −420 | ±1.30 / ±0.52 | ±1.30 / ∓0.78 | terminal **plow** (see below) |
| d | 5, 5 | −504, −336 | ±1.56 / ±0.78 | ±1.04 / ∓0.52 | terminal spin |
| e | 5, 5 | −336, −504 | ±1.04 / ±0.52 | ±1.56 / ∓0.78 | terminal plow |

**Erratum E-6 — cases b and c have their character interchanged.** `a` is the CG-to-front-axle
distance (§3.4), so case b (`a` = 6, `b` = 4) puts the CG 6 ft behind the front axle — a
**rear-heavy** car — and case c is front-heavy. Two independent routes both give the opposite
of the printed characters, and both confirm cases a, d and e as printed:

- **Understeer gradient.** From I-1, `K·ℓ·g = W_F/|C_F| − W_R/|C_R|`, and with
  `W_F = m g b/ℓ`, `W_R = m g a/ℓ` and `|C_F| = |C_R|` this is proportional to `b − a`.
  Case b has `b − a = −2` ⇒ `K < 0` ⇒ oversteer ⇒ **spin**. Case c has `b − a = +2` ⇒
  understeer ⇒ **plow**.
- **Saturation order.** On the trim line `a·F_yF = b·F_yR`, so with equal stiffnesses
  `α_F/α_R = b/a`. Case b gives `α_F = 0.667·α_R`: the **rear** reaches `α_bk` first, which
  is terminal spin. Case c gives `α_F = 1.5·α_R`: the front saturates first, terminal plow.

The same two arguments applied to cases d and e reproduce the printed "spin" and "plow"
exactly — d has the stiffer front (`α_F = 0.667·α_R`, rear saturates first) and e the
stiffer rear — which is what identifies the defect as confined to the b/c rows rather than
to the convention.

Equivalently, and as the implementation computes it: at the diagram's extreme `Ā_Y` both
axles are saturated, so `C_N = (a·F_yF − b·F_yR)/(m g ℓ)` has the sign of `a − b` for equal
stiffnesses. Case b gives `C_N = +0.261` (above the trim line ⇒ spin), case c `−0.261`
(below ⇒ plow). The acceptance suite SHALL assert the characters as corrected here, and
SHALL additionally require the diagram's classification to agree with the sign of `K` from
(8.14) for every case — the two routes are independent and their agreement is the check.

All twenty construction-line coordinates reproduce to **4.2e−3** relative with the mandated
`g = 9.80665 m/s²` (worst case: case a, `Ā_Y = 1.3054` vs a published 1.30). With the
source's `g = 32.2 ft/s²` — which Rule U4 forbids a solver from using — the worst case is
3.3e−3. The residual is therefore dominated by the source's rounding to three significant
figures, not by the gravity constant, and the acceptance tolerance for this block is set to
**5e−3** with that reason recorded. Source: [R280] p. 104.

Metric-extraction vectors from a real diagram ([R280] pp. 105–111, [RCVD] Fig. 8.7,
60 mph, ℓ = 8.5 ft, `G_s` = 20:1).

**Reproducibility of this block (normative).** The diagram these are read from is generated
by a tire dataset that this specification does not reproduce, for the same reason §10.7
gives for its own precedent case. Consequently **M-01, M-02, M-03, M-05 and M-06 are cited
as provenance, not as reproducible acceptance vectors**, and the suite SHALL NOT assert
them against a fixture that cannot generate that diagram. Only **M-04 is reproducible**: it
is pure arithmetic on (10.10) given `SS`, `G_s`, `ℓ` and `V`, all of which are stated, and
it is what erratum E-3 turns on. The metric *implementations* (SI, CM, SS, SSl, UG, limit
classification, max trimmed `Ā_Y`) SHALL instead be verified against the §17.6 five-case
L2 fixture above, where every input is specified — and, for SI, against its analytic
linear-range value (10.7).

| ID | Metric | At A_Y = 0.3 g | At A_Y = 0.7 g |
|----|--------|----------------|----------------|
| M-01 | Stability index [C_N/g] | −0.25 | −0.40 |
| M-02 | Steering sensitivity [g/100° SW] | 1.18 (see E-3) | 0.37 (see E-3) |
| M-03 | Sideslip sensitivity [deg/g] | −1.61 | −3.33 |
| M-04 | Understeer gradient [deg/g] | published 2.211; exact-constant 2.214 | published 11.49; exact-constant 11.490 |
| M-05 | Cross-check against the constant-speed response test | 2.2 | 11.4 |
| M-06 | Max trimmed A_Y | 0.82 g at δ_SW = 120°, β = −2.0°, limit understeer |

M-04's published figures were computed with the rounded 858 constant of E-4; the exact
constant gives 2.214 and 11.490, both inside the 3e−3 default tolerance. The test SHALL
assert against the exact-constant values and record the published ones as provenance.

M-04 vs M-05 is a genuine independent cross-check: the understeer gradient computed from
the steering-sensitivity inversion (10.10) is compared with the slope read from the
directly-simulated constant-speed steady-state response. Agreement to 0.5 % is the
acceptance criterion.

M-01 vs M-04 also demonstrates the point the metric exists to make: the stability index
changes by a factor 1.6 between 0.3 g and 0.7 g while the understeer gradient changes by a
factor 5.2. The suite SHALL reproduce both ratios, and the documentation SHALL state that
the understeer gradient is a low-`A_Y` concept while the stability index remains meaningful
to the limit.

### 17.7 P4 / P5 acceptance

| ID | Check |
|----|-------|
| S-01 | Theoretical-optimum setup on a 50 ft radius crashes into the inner wall; `H-SPD-001` fires; slightly reducing the steer angle (or increasing speed) completes the lap |
| S-02 | Lap time is invariant to `output_record_increment` to 1e-9 s (event location is by dense output, not by record sampling) |
| S-03 | Headless CLI run and GUI run produce bit-identical lap time and path |
| S-04 | Halving the integration step changes lap time by less than the stated convergence tolerance; a Richardson estimate of the discretisation error is reported |
| S-05 | P5 on the 200 ft radius with a high-downforce configuration reaches lap times in the reported single-digit-second regime, and the optimum configuration differs between a 400 ft and a 50 ft radius (aerodynamic speed dependence) |

### 17.8 Cross-program consistency

| ID | Check |
|----|-------|
| C-01 | I-4: P3 max trimmed `A_Y` vs P1 steady-state limit, same vehicle — within 2 % |
| C-02 | P2 in **bicycle-equivalence mode** reproduces P1 to 1e-10. That mode is an explicit, tested configuration — `load_transfer: off`, `aero: off`, tire model L1, `track_velocity_terms: off` (drops the `± r·t/2` of (9.14)), `small_angle_slip: on` (uses `v/u` rather than `arctan`), `steer_projection: off` (drops the `cos δ`/`sin δ` factors of (9.11)–(9.13)), `aligning_moment: off`, `longitudinal_forces: off` — not an emergent property. Each switch SHALL exist in `SolverConfig` and SHALL be individually tested. **`steer_projection` is not optional for this test:** P1's (8.1) sums the axle forces unprojected, so retaining `cos δ` leaves a residual of `1 − cos δ` — 1.5e−4 at δ = 1°, 3.8e−3 at δ = 5° — which exceeds the 1e−10 criterion at every steer angle that produces a measurable response. |
| C-03 | P4 with the P1 plant and P5 with a P2 plant configured as in C-02 produce identical lap times to 1e-9 s |
| C-04 | P6 output fed into P2 as chassis-to-ground roll stiffnesses reproduces the target roll gradient to 1e-9, using the exact inversion (12.11) — i.e. against R-01b (1368.07 lbf·ft/deg), not the source's simplified R-01 (1300), which lands at 3.166 deg/g |
| C-05 | Understeer gradient agrees across three independent routes: (8.18) analytic, P2 slow-ramp simulation, P3 diagram inversion — within 2 % |

C-02 is the single most valuable structural test in the suite: it forces P1 and P2 to share
one force path rather than two implementations that drift apart.

**Which switches C-02 can and cannot exercise.** "Individually tested" above cannot mean
"individually tested *through C-02*", because two of the switches are **provably inert in
C-02's own configuration**. C-02 mandates tire model L1, and an L1 tire with a constant
`C_α` has `F_y` independent of `F_z`. Both `load_transfer` and `aero` reach the lateral
force *only* through `F_z` — load transfer by redistributing it, downforce by adding to it —
so with a constant-stiffness tire that path is severed and neither switch can change the
result. This is §6.7's own observation ("a build in which (6.9)/(6.10) is disabled SHALL
make P2's roll-stiffness distribution input inert") applied to C-02.

Measured on the CH5 fixture at 30 mph, `δ = 5°`, `v = 1.5 m/s`, `r = 0.35 rad/s`, as the
relative change in the state derivative when each switch is returned to `true`:

| Switch | Effect under C-02's L1 tire | Effect with a load-sensitive (6.10) tire |
|---|---|---|
| `track_velocity_terms` | 6.6e−4 | — |
| `small_angle_slip` | 2.4e−2 | — |
| `steer_projection` | 6.3e−3 | — |
| `load_transfer` | **0 (exactly)** | 2.2e−1 |
| `aero` | **0 (exactly)** | 2.8e−2 |
| `aligning_moment` | 0 — L1 with `t_p = 0` returns `M_z = 0` | — |
| `longitudinal_forces` | 0 — Rev 1.0 has no driveline, so `F_x ≡ 0` | — |

The acceptance suite SHALL therefore test `track_velocity_terms`, `small_angle_slip` and
`steer_projection` by requiring each to **break** C-02 when returned to `true`, and SHALL
test `load_transfer` and `aero` separately, against a load-sensitive tire, by requiring each
to change P2's own output. `aligning_moment` and `longitudinal_forces` are carried in the
mode for correctness under models that do produce those terms, and are untestable until
such a model exists (extension seams 3 and 7).

**A trap when testing switches pointwise.** Under `load_transfer_coupling: explicit` — the
default — `A_Y` comes from the *previous* accepted step, so the very first `derivative()`
call of a run sees `A_Y = 0` and therefore **no lateral load transfer at all**. A single
evaluation will report `load_transfer` as inert even with a load-sensitive tire. A pointwise
switch test SHALL use `implicit` coupling, or iterate the explicit map to its fixed point.

---

## 18. Non-Functional Requirements

### 18.1 Performance

| Operation | Budget (single core, contemporary x86-64) |
|---|---|
| P1, 10 s at Δt = 1 ms, 5 states | ≤ 5 ms |
| P2, 10 s at Δt = 1 ms, implicit load-transfer loop | ≤ 250 ms |
| P3, 41 × 41 sweep with a tabular tire | ≤ 100 ms |
| P3, 201 × 201 sweep | ≤ 2 s |
| P5, one 50 ft lap at Δt = 1 ms | ≤ 100 ms; real-time factor ≥ 20× |
| P6, one solve | ≤ 10 µs |
| Tire evaluation, tabular PCHIP | ≤ 200 ns |

A parameter sweep API (`sweep(vehicle, parameter_grid, evaluate)`) SHALL be provided and MAY
be parallelised across grid points, since each point is independent — but the reduction
order for any aggregate SHALL be fixed (Rule A2).

### 18.2 Portability

Linux (glibc ≥ 2.31), macOS ≥ 12, Windows 10+; x86-64 and arm64. No dependency on
architecture-specific intrinsics in the core. Cross-platform agreement to `1e-12` relative
on the full acceptance suite is a release gate.

### 18.3 Interface equivalence

For every acceptance vector, the GUI, `rcvd-cli` and the C API SHALL be exercised and SHALL
agree bit-for-bit. This is what makes Rule A1 testable rather than aspirational.

### 18.4 Documentation

Ships with: this specification; a theory manual containing the derivations of §8–§12 with
the dimensional audits; a user guide; API reference (Doxygen); and a **validation report**
reproducing §17 with the actual numbers from the built binary, regenerated on every release.

---

## 19. Errata and Documented Deviations from the Source

Every deviation below is deliberate, justified, tested and flagged behind a compatibility
switch. Nothing here is a silent "improvement".

| ID | Location | Source behaviour | This specification | Justification |
|----|----------|------------------|--------------------|---------------|
| **E-1a** | Lateral load transfer, gravity-term **sign** ([R280] p. 218) | Term subtracts from the transfer | Carried inside `φ̃` via (9.5), where it **adds** | As printed it violates the whole-vehicle moment balance I-3: the sprung CG displaces toward the outside wheels as the body rolls, which must increase the load there. Flag: `--legacy-gravity-sign`. |
| **E-1b** | Lateral load transfer, gravity-term **distribution** ([R280] p. 218) | Distributed between axles by sprung-weight fraction `(1 − a_s/ℓ)` while the spring term is distributed by `K_φF/ΣK` | Single term `K_φF·φ̃/t_F`, i.e. distributed by `K_φF/ΣK` (§9.3) | Load transferred through an axle's springs is `K_φ,axle·φ̃/t` by definition of a roll spring, whatever produced `φ̃`. The two ratios coincide only when `K_φF/ΣK = 1 − a_s/ℓ`. Fully specified example: the F-12 Case-1 vehicle with `K_φF:K_φR = 2:1` and `a_s/ℓ = 0.47` (53 % front sprung weight), roll gradient 2.63 deg/g. Correct: 421.17 / 273.25 lbf/g. Wrong distribution alone: 417.37 / 277.04, i.e. **−0.90 % front, +1.39 % rear**. Wrong distribution *and* wrong sign (the form as printed): 387.96 lbf/g front, **−7.88 %**. Detected by invariant I-3b, not by I-3. |
| **E-1c** | Roll stiffness required for a target roll gradient ([R280] p. 258) | `ΣK = W·h/φ_target` | (12.11), `ΣK = m_s h₂/RG + m_s g h₂` | The printed form omits the destabilising `m_s g h₂`, so the delivered roll gradient is 5.5 % larger than the target (3.166 vs 3.000 deg/g for the §17.4 case). Reported alongside the simplified value rather than silently corrected (§12.5). |
| **E-2** | Ride frequency ([R280] p. 258) | Whole-axle mass paired with a single-wheel rate | Per-corner mass (12.9) | Dimensionally inconsistent as printed; the published rates correspond to `√2 ×` the stated frequencies. Flag: `--legacy-ride-frequency`. Downstream calculator algebra is unaffected and is reproduced exactly (§17.4). |
| **E-3** | MMM steering sensitivity ([R280] pp. 108–109) | 1.30 and 0.33 g/100° SW quoted, but 1.18 and 0.37 used in the understeer-gradient calculation on the following page | Acceptance vectors use 1.18 / 0.37 | Only 1.18 / 0.37 are consistent with the published understeer gradients (2.211, 11.49 deg/g) and with the independent constant-speed response-test cross-check (2.2, 11.4 deg/g). The 1.30 / 0.33 pair reproduces neither. |
| **E-4** | Ackermann gradient constant ([R280] p. 109) | `858·ℓ[ft]/V[mph]²` | `ℓ/V²` in SI (rad per m/s²), `ℓ·g/V²` per g | The exact English-unit constant is `57.2958 × 32.17405 / 1.466667² = 856.970`; 858 is 0.120 % high. The SI form has no magic number. |
| **E-5** | Yaw-rate transfer function matrix ([R280] p. 76) | RHS entry printed as 33463.2 | `−a·C_F` per radian ≈ 40 091 lbf·ft/rad (40 153 with the source's rounded `a` = 2.92 ft) | The printed value is inconsistent with the transfer function printed on the same page, which the correct value reproduces (numerator `0.841 s + 7.550` vs published `0.84 s + 7.57`) and which satisfies the DC-gain identity I-2 exactly. |
| **E-6** | MMM limit character, cases b and c ([R280] p. 104) | b (`a`=6, `b`=4) labelled "terminal plow", c (`a`=4, `b`=6) "terminal spin" | Interchanged: b is **spin**, c is **plow** | `a` is CG-to-front-axle, so b is the rear-heavy car. Both the understeer-gradient sign from I-1 (`K ∝ b − a`) and the trim-line saturation order (`α_F/α_R = b/a`) give the opposite of the printed labels, while reproducing cases a, d and e as printed — which confines the defect to those two rows. Detected by the §17.6 classification test and by its cross-check against the sign of `K`. |
| **D-1** | Tire vertical load sign | Negative in the SAE tire frame | Non-negative magnitude in the tire-model interface | Eliminates a systematic class of sign errors; conversion isolated in the wheel adapter (§3.2). |
| **D-2** | Tabular tire interpolation | Linear (produces the visible slope "kinks" noted at [R280] p. 255) | Monotone cubic Hermite | Linear interpolation makes the numerical Jacobians used by the steady-state, MMM and implicit load-transfer solvers discontinuous. Flag: `--legacy-linear-interp`. |
| **D-3** | Transfer/fraction naming | `ΔF_z` means the full inside-to-outside difference in [R280] Ch. 7 and the transferred amount in Ch. 18 | Single definition (9.8)–(9.9) | Factor-of-two hazard. |
| **D-4** | Roll angle symbol in load transfer | `φ` (SAE, negative in a right-hand turn) used with a magnitude sense | Explicit `φ̃ = −φ`, odd in `A_Y` (§9.3) | Using `\|φ\|` makes the load transfer even in `A_Y`, so a symmetric vehicle transfers different loads turning left and right. |
| **D-5** | Roll stiffness used in load transfer | Not stated whether chassis-to-ground or chassis-to-wheel-centre | Chassis-to-ground, normatively (§9.3) | The two differ by 33 % on the front axle of the §17.4 case; the choice also changes the front/rear split whenever the tire rates differ. |

---

## 20. Extension Seams

Designed-in, not retrofitted. Each is a named interface that a future revision fills without
restructuring:

1. **Roll as a dynamic DOF.** The three-equation Segel form is already the target shape:
   ```
   m V(r + β̇) + m_s h₂ φ̈           = Y_β β + Y_r r + Y_φ φ + Y_δ δ
   I_z ṙ + I_xz φ̈                   = N_β β + N_r r + N_φ φ + N_δ δ
   I_x φ̈ + I_xz ṙ + m_s h₂ V(r+β̇)  = L_β β + L_φ φ + L_φ̇ φ̇
   ```
   with `h₂` the sprung-CG-to-roll-axis distance of (9.4b) and inertias taken about the
   roll axis. `Plant::state_dim()` is already dynamic; adding `φ, φ̇` extends the state
   vector without touching the integrator or output layers. `L_φ` is the roll spring term
   (always significant) and **already contains the destabilising `−m_s g h₂` of (9.5)** —
   i.e. `L_φ = −(ΣK − m_s g h₂)` — which is why the roll-divergence check `E-VEH-140` is
   the quasi-static shadow of a genuine dynamic instability. `L_φ̇` is the roll damping;
   `L_r` and `L_δ` are coupling terms conventionally omitted.
2. **Ride/pitch model.** The 2-DOF heave/pitch system
   `m z̈ = −(K_F+K_R)z − (K_F a − K_R b)θ + F_z(t)`,
   `I_y θ̈ = −(K_F a − K_R b)z − (K_F a² + K_R b²)θ + M_y(t)`
   with characteristic equation
   `m I_y s⁴ + [m(K_F a² + K_R b²) + I_y(K_F + K_R)]s² + K_F K_R (a+b)² = 0`
   and the classical decoupling condition `K_F/K_R = b/a`.
3. **Combined-slip tire.** `TireState` already carries `kappa`; the `TireModel` contract
   extends to return `F_x` without a signature change.
4. **Camber / inclination.** `TireState::gamma_rad` is reserved.
5. **Kinematic suspension.** A `SuspensionModel` returning roll-centre height, camber and
   installation ratio as functions of ride and roll position, replacing the constant
   geometry of §9.3 and feeding (12.4).
6. **Compliances.** Lateral-force and aligning-torque compliance steer added as slip-angle
   offsets in (9.14).
7. **Steering system.** Per-wheel `δ_i` in (9.11)–(9.13) is already the interface;
   Ackermann geometry, ratio nonlinearity and kingpin torque follow.
8. **Arbitrary circuits and closed-loop driver.** `Circuit` is already polymorphic;
   `DriverPlan` becomes a `Driver` interface with a preview-tracking controller.
9. **Four-wheel MMM.** Replace the static axle loads in (10.4) with the §9.3 load model.
10. **Optimisation.** The sweep API plus a Pareto front over (lap time, robustness) is the
    natural home for the multi-radius set-up trade-off exercised by P5.
11. **Banked and graded surfaces in the load model.** §9.3 Steps 1–5 are flat-surface;
    `road_tilt` reaches only (9.11). Adding `cos θ_t` to (9.1), the `m g sin θ_t`
    contribution to the roll moment of (9.5), and a road-grade term to (9.12) turns the
    closed forms of (17.2) into a model result and promotes F-04 … F-06 from analytic
    checks to P2 acceptance vectors. No interface changes.

---

## Appendix A — Consolidated Equation Index

| Eq | Subject | Section |
|----|---------|---------|
| 3.1 | Slip angle definitions | §3.3 |
| 5.1a–5.1c | Sprung/unsprung mass, CG-height and CG-position consistency | §5.1 |
| 6.1–6.3 | Linear and breakaway tire | §6.2–6.3 |
| 6.2a | Pneumatic trail, limit form (T8) | §6.1 |
| 6.4–6.7 | Radt non-dimensionalisation and expansion | §6.5 |
| 6.8 | Magic Formula | §6.6 |
| 6.9–6.10 | Load sensitivity | §6.7 |
| 6.11–6.12 | Friction ellipse | §6.8 |
| 6.13–6.16 | Relaxation-length dynamics | §6.9 |
| 7.1–7.5 | Aerodynamics | §7 |
| 8.1–8.5 | Bicycle-model derivation | §8.2 |
| 8.6–8.11 | Stability derivatives | §8.3 |
| 8.12 | State-space form | §8.4 |
| 8.13–8.25 | Steady state, gains, characteristic speeds, identities | §8.5 |
| 8.26–8.28 | Eigenstructure and transfer function | §8.6 |
| 8.29–8.30 | Path kinematics and road tilt | §8.7–8.8 |
| 9.1–9.3 | Static, aerodynamic and longitudinal wheel loads | §9.3 |
| 9.4a–9.6b | Roll axis, roll angle, lateral load transfer | §9.3 |
| 9.7 | Wheel-load assembly | §9.3 |
| 9.8–9.9 | Transfer and fraction-load-transfer definitions | §9.4 |
| 9.10, 9.10b | Invariants I-3 and I-3b | §9.5 |
| 9.11–9.15 | Four-wheeler EOM and algebraic loop (9.12a defines `A_X`) | §9.6–9.7 |
| 10.1–10.10 | Moment method (10.3a/10.3b normalised pair) | §10 |
| 11.1 | Lap detection | §11.4 |
| 12.1–12.12 | Ride and roll rates | §12 |
| 13.1–13.2 | Step control and differentiation step | §13 |
| 17.1 | Load-sensitive pair-analysis quadratic | §17.5 |
| 17.2 | Banked-corner pair analysis (F-04 … F-06) | §17.5 |

## Appendix B — Invariant Index (all normative)

| ID | Statement | Tolerance |
|----|-----------|-----------|
| T1–T8 | Tire model contract (symmetry, zero crossing, sign, zero load, friction bound, slope consistency, continuity, finite trail) | 1e-12 / 1e-6 |
| I-1 | `K·ℓ·g = W_F/\|C_F\| − W_R/\|C_R\|` | 1e-12 |
| I-2 | Yaw-rate gain: derivative form = `(V/ℓ)/(1+KV²)` = TF DC gain | 1e-14 |
| I-3 | Summed axle roll moments = sprung + unsprung applied moments | 1e-12 |
| I-3b | Per-axle allocation matches the specified roll-stiffness split (9.10b) | 1e-12 |
| I-3c | Roll gradient within the physical band and equal to (12.10) | 1e-12 / band |
| I-4 | P3 max trimmed `A_Y` = P1 steady-state limit | 2e-2 |
| I-5 | P6 direction A ∘ direction B = identity | 1e-12 |
| A2 | Run-to-run bit reproducibility; cross-platform 1e-12 | — |
| C-02 | P2 degenerates exactly to P1 under stated settings | 1e-10 |

## Appendix C — Implementation Order

A build order that keeps the suite testable at every stage:

1. `units` + exact constants + the conversion round-trip tests.
2. `core` value types, JSON schema, validation (§5, §14).
3. `numerics`: PCHIP, Brent, LU, RK4, DOPRI5(4) with dense output — each against closed-form
   test problems before any vehicle physics exists.
4. `tire`: L1, L2, then the conformance suite T1–T7; L3, L4, L5 afterwards.
5. `models::BicycleModel` + `analysis` (steady state, derivatives, eigenstructure) →
   acceptance vectors B-01 … B-31. **Gate: do not proceed until all pass.**
6. `aero`, then `models::FourWheelerModel` load transfer → invariants I-3, I-3b, I-3c and
   vectors F-01 … F-12 (incl. F-11b). **Gate: I-3, I-3b, I-3c and C-02 must all pass.**
7. `analysis::moment_method` → M-01 … M-06, I-4.
8. `analysis::ride_roll` → R-01 … R-14, I-5. (Independent of everything above; may be built
   in parallel by a second developer.)
9. `analysis::lap_solve` + event location → S-01 … S-05.
10. `io`, `capi`, `rcvd-cli`.
11. `app` (GUI), then the interface-equivalence gate §18.3.

## Appendix D — Acceptance Checklist

- [ ] All acceptance vectors in §17 pass at their stated tolerances, on all supported platforms.
- [ ] All invariants in Appendix B pass at 200+ randomised parameter sets each.
- [ ] Sign-convention table §3.3 reproduced end-to-end for a right-hand turn.
- [ ] Every warning and error code in §14.2 has a test that provokes it.
- [ ] Bit-reproducibility (A2) demonstrated across 10 repeat runs and across GUI/CLI/API.
- [ ] Every clamp and every extrapolation raises an event (Rules D1, D2) — verified by
      fault-injection tests.
- [ ] No `-ffast-math` in any build configuration — verified by inspecting the compile
      commands in CI.
- [ ] Unit-conversion layer exercised in both directions with round-trip error < 1e-15.
- [ ] Validation report regenerated from the built binary and archived with the release.
- [ ] Errata E-1a … E-5 and deviations D-1 … D-5 documented in the release notes with their
      compatibility flags, and
      each flag tested in both positions.

---

*End of specification.*
