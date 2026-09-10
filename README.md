# RCVD Program Suite

Implementation of `RCVD-SUITE-SPEC` Rev 1.0
([RCVD_Program_Suite_Specification.md](RCVD_Program_Suite_Specification.md)) — six
vehicle-dynamics programs derived from Milliken, Kasprzak, Metz & Milliken,
*Race Car Vehicle Dynamics: Problems, Answers and Experiments* (SAE R-280).

The specification is **normative on the mathematics**. Where this code and the
spec disagree, the spec wins and the code is a defect.

## Status

Following the build order of spec Appendix C:

| # | Component | State |
|---|-----------|-------|
| 1 | `units` — exact constants, closed vocabulary, round-trip | **done** |
| 2 | `core` — `Result<T>`/`Diagnostic`, §14.2 register, `Vehicle` | **partial** — error channel, `Vehicle` and (5.1a–c) validation done; JSON schema outstanding |
| 3 | `numerics` — PCHIP, Brent, LU + `rcond`, RK4, DOPRI5(4) dense output | **done** |
| 4 | `tire` — L1, L2, conformance T1–T8, then L3/L4/L5 | **partial** — L1, L2 and the shared T1–T8 suite done; L3/L4/L5 outstanding |
| 5 | `models::BicycleModel` + `analysis` → B-01 … B-31 | **done** — gate passed |
| 6 | `aero` + `FourWheelerModel` → I-3/I-3b/I-3c, F-01 … F-12, C-02 | **done** — gate passed |
| 7 | `analysis::moment_method` → §17.6, I-4 | **done** — plus C-01 |
| 8 | `analysis::ride_roll` → R-01 … R-14, I-5 | **done** — plus C-04 |
| 9 | `analysis::lap_solve` + event location → S-01 … S-05 | **done** — plus C-03 |
| 10–11 | io/capi/cli, GUI | not started |

Both hard gates are passed: B-01 … B-31 (step 5) and I-3/I-3b/I-3c + C-02
(step 6). Cross-checks C-01, C-02, C-03 and C-04 all hold; C-05 waits on a
slow-ramp P2 run. The transient vectors (X-01 … X-13, §17.3) need a run driver
with event location and are not yet built.

## Building

Requires CMake ≥ 3.24 and a C++20 compiler. Catch2 v3 is fetched automatically.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build
ctest --test-dir build --output-on-failure
```

### Without CMake

This machine has MSVC 14.44 but no CMake or Ninja, so the Catch2 suite in
`tests/` has not yet been executed against real Catch2 — it has been
type-checked and run against a local shim only. `tools/verify_standalone.cpp`
mirrors the same assertions with no dependencies, and **is** verified:

```bash
cl /nologo /std:c++20 /EHsc /W4 /WX /permissive- /fp:precise /I include /Fe:verify.exe tools/verify_standalone.cpp
```

225 assertions, all passing. To get the authoritative suite running,
`winget install Kitware.CMake` and use the CMake path above.

## The conformance suite is itself tested

`tire/conformance.hpp` implements T1–T8 once, against the abstract `TireModel`,
so a new model earns its conformance by construction rather than by someone
remembering to write its checks. Because a conformance suite that cannot fail is
worthless, `tests/support/defective_tire.hpp` injects each violation in turn and
the suite is required to catch it — and the un-injected control is required to
pass, so the result is not merely "everything fails".

Two of the checks needed care:

- **T7 continuity** cannot use an absolute tolerance on `Fy(p+ε) − Fy(p−ε)`. With
  `C_α ≈ −1.6e5 N/rad` the *legitimate* change over `ε = 1e-9` is already 3e-4 N,
  so any fixed bound either rejects correct models or is too loose to catch a real
  jump. It uses a scale-free ratio instead: for a continuous curve the gap shrinks
  linearly with `ε`, so `gap(ε) ≈ gap(10ε)/10`; at a discontinuity both equal the
  jump.
- **T6** excludes interpolation knots as well as declared slope breaks, for the
  reason recorded in the spec at §6.1.

The same discipline is applied to the B-vectors. All 18 cases passed on their
first run, which is not by itself evidence they are load-bearing, so each defect
§8 explicitly warns about was injected into a scratch copy and the suite was
required to catch it:

| Injected defect | Caught by |
|---|---|
| `a`/`b` inversion (§5.1) | 14 assertions, including the static axle loads |
| Second-column trap, `Y_r/(m u)` (§8.4) | 5, including I-2 and B-31 |
| `B` with a leading minus (§8.4) | 8, including I-2 and the B-24 numerator |

Which is what the spec predicts: "Identity I-2 detects it at every speed."

## Layout

```
include/rcvd/
  units/     constants.hpp   exact primitives + U5 composites (§3.1)
             convert.hpp     closed unit vocabulary, the two-point boundary (U1/U2)
  core/      diagnostic.hpp  the §14.2 E-/W-/EV-/H- register
             result.hpp      Result<T>, the §4.6 error channel (Rule A3)
             vehicle.hpp     resolved SI Vehicle + (5.1a)-(5.1c) validation
  numerics/  pchip.hpp       Fritsch-Carlson monotone cubic Hermite (§13.5)
             roots.hpp       Brent (§13.7) + the (13.2) differentiation step
             linalg.hpp      LU with partial pivoting and rcond (§13.8)
             integrate.hpp   RK4 fixed, DOPRI5(4) with dense output (§13.2)
  tire/      tire_model.hpp  the §6.1 contract; TireState/TireOutput (§4.3)
             linear.hpp      L1 (6.1) and L2 (6.3), with (6.10) load sensitivity
             conformance.hpp the shared T1-T8 check, applied to every model
  models/    bicycle.hpp     P1: derivatives, state space, steady state,
                             eigenstructure, transfer function (§8)
             load_transfer.hpp  §9.3-9.5 wheel loads, I-3/I-3b/I-3c, E-1a/E-1b
             plant.hpp       the §4.3 Plant contract, §5.6 maneuvers, §5.5 switches
             four_wheeler.hpp   P2: (9.11)-(9.14), the §9.7 algebraic loop
  aero/      aero_model.hpp  §7 drag, lift, balance, density
  analysis/  pair_analysis.hpp  (17.1) and (17.2) analytic checks
             ride_roll.hpp   P6: §12 springing algebra, both directions
             moment_method.hpp  P3: §10 sweep, boundary, trim line, metrics
             lap_solver.hpp  P4/P5: §11 run driver with dense-output events
tests/       Catch2 v3 suite, authoritative
  support/   defective_tire.hpp — fault injection for the conformance suite
             fixtures.hpp       — CH5, CH5-OS and "First Example" (§17.2)
tools/       verify_standalone.cpp — dependency-free mirror
```

## Conventions that bite

These are the ones the spec flags as recurring sources of error, and that the
code is written to make hard to get wrong:

- **`a` and `b` are inverted relative to intuition.** `a = ℓ·(1 − f_front)`:
  a car with 57 % front weight has `a = 0.43·ℓ`, because the CG sits *closer* to
  the front axle (§5.1).
- **Cornering stiffness is negative.** `C_α ≡ ∂F_y/∂α|₀ < 0`. A positive value is
  rejected at input validation as `E-TIRE-003` (§3.3).
- **Vertical load is a non-negative magnitude** in every tire interface
  (decision D-1), not the negative `F_z` of the SAE tire frame. The sign
  conversion lives in the wheel adapter alone.
- **The `r`-column of `A(u)` already carries `1/u`.** `Y_r` and `N_r` are defined
  with it (8.7, 8.10), so the entries are `Y_r/m − u` and `N_r/I_z`, never
  `Y_r/(m u)` (§8.4). This one bit the initial test data for `eig2x2`.
- **No `-ffast-math`**, ever. It breaks Rule A2 reproducibility and NaN
  propagation, and §13.1 requires a NaN to be detectable as a fatal defect. The
  CMake configure step fails if such a flag is present.
