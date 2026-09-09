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
| 2 | `core` — `Result<T>`/`Diagnostic`, §14.2 register | **partial** — error channel done; value types, JSON schema and validation outstanding |
| 3 | `numerics` — PCHIP, Brent, LU + `rcond`, RK4, DOPRI5(4) dense output | **done** |
| 4 | `tire` — L1, L2, conformance T1–T8, then L3/L4/L5 | not started |
| 5 | `models::BicycleModel` + `analysis` → B-01 … B-31 | not started |
| 6–11 | four-wheeler, MMM, ride/roll, lap solver, io/capi/cli, GUI | not started |

Step 5 is a hard gate: do not proceed past it until all of B-01 … B-31 pass.

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

52 assertions, all passing. To get the authoritative suite running,
`winget install Kitware.CMake` and use the CMake path above.

## Layout

```
include/rcvd/
  units/     constants.hpp   exact primitives + U5 composites (§3.1)
             convert.hpp     closed unit vocabulary, the two-point boundary (U1/U2)
  core/      diagnostic.hpp  the §14.2 E-/W-/EV-/H- register
             result.hpp      Result<T>, the §4.6 error channel (Rule A3)
  numerics/  pchip.hpp       Fritsch-Carlson monotone cubic Hermite (§13.5)
             roots.hpp       Brent (§13.7) + the (13.2) differentiation step
             linalg.hpp      LU with partial pivoting and rcond (§13.8)
             integrate.hpp   RK4 fixed, DOPRI5(4) with dense output (§13.2)
tests/       Catch2 v3 suite, authoritative
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
