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
| 2 | `core` — `Result<T>`/`Diagnostic`, §14.2 register, `Vehicle` | **done** — document layer landed with step 10 |
| 3 | `numerics` — PCHIP, Brent, LU + `rcond`, RK4, DOPRI5(4) dense output | **done** |
| 4 | `tire` — L1, L2, conformance T1–T8, then L3/L4/L5 | **done** — all five models pass T1–T8 |
| 5 | `models::BicycleModel` + `analysis` → B-01 … B-31 | **done** — gate passed |
| 6 | `aero` + `FourWheelerModel` → I-3/I-3b/I-3c, F-01 … F-12, C-02 | **done** — gate passed |
| 7 | `analysis::moment_method` → §17.6, I-4 | **done** — plus C-01 |
| 8 | `analysis::ride_roll` → R-01 … R-14, I-5 | **done** — plus C-04 |
| 9 | `analysis::lap_solve` + event location → S-01 … S-05 | **done** — plus C-03 |
| 10 | `io`, `capi`, `rcvd-cli` | **done** |
| 11 | `app` (GUI) + §18.3 interface-equivalence gate | **done** — six windows on Qt 6.8.3; §18.3 closed in both halves |

Both hard gates are passed: B-01 … B-31 (step 5) and I-3/I-3b/I-3c + C-02
(step 6). **All five cross-checks C-01 … C-05 hold**, the behavioural vectors
X-01 … X-13 pass, and all eleven Appendix C stages are built. 196 of 196 ctest
cases pass.

## Testing

Three ways in, in increasing order of what they need installed.

### 1. One command, nothing to install

```powershell
pwsh tools/run_tests.ps1
```

Finds the compiler itself (or use it from a *Developer PowerShell for VS*),
builds the dependency-free acceptance driver, runs it, then runs the Rule U6
compile-failure gate. Prints `ALL GREEN` and exits 0 on success.

### 2. By hand, if you prefer

```bash
cl /nologo /std:c++20 /EHsc /W4 /WX /permissive- /fp:precise /I include /I tests /Fe:verify.exe tools/verify_standalone.cpp
verify.exe
```

Note **both** include paths: `tools/verify_standalone.cpp` pulls in the shared
fixtures under `tests/support`, so `/I include` alone will not compile. 225
assertions covering B-01 … B-31, F-01 … F-12, R-01 … R-14, the tire conformance
fault injection and the numerics.

The Rule U6 gate is separate, because it asserts that certain files **fail** to
build and so cannot live in a passing translation unit:

```bash
python tools/check_compile_fail.py --compiler cl --include include
```

### 3. The authoritative suite — needs CMake

`tests/` is the real suite (spec §4.2 mandates Catch2 v3): 17 files, ~7,900
checks, including everything above plus the C ABI equivalence gate, the §17.3
transients, the moment method, the lap solver and the §14.2 diagnostic-coverage
gate.

```bash
winget install Kitware.CMake
cmake -S . -B build
cmake --build build --config RelWithDebInfo
ctest --test-dir build -C RelWithDebInfo --output-on-failure
```

The config goes on the **build** line, not the configure line. CMake's default
generator here is Visual Studio 17 2022, which is multi-config, so
`-DCMAKE_BUILD_TYPE=...` is silently ignored — CMake says so:
`Manually-specified variables were not used by the project: CMAKE_BUILD_TYPE`.

Catch2 is fetched automatically. **Verified: 195/195 tests pass** — 7,944
assertions across 194 Catch2 cases plus the Rule U6 compile-failure gate, each
case in its own process, under CMake 4.4.4 and MSVC 19.44 at RelWithDebInfo.

## Running the CLI

```powershell
pwsh tools/build_cli.ps1        # produces .\rcvd-cli.exe
.\rcvd-cli.exe --help
```

`rcvd-cli` is a single translation unit — everything it needs is header-only —
so it needs no linker configuration and no CMake.

Three commands, all taking a §5.1 vehicle document:

```powershell
# stability derivatives and handling metrics (B-03 ... B-08, B-10, B-11, B-26)
.\rcvd-cli.exe derivs --vehicle examples/ch5.json --units us --in-units us `
                --cf -640 --cr -580 --speed 30

# steady-state response at a given steer angle
.\rcvd-cli.exe steady --vehicle examples/ch5.json --units us --in-units us `
                --cf -640 --cr -580 --speed 30 --steer 3

# a lap of a circular circuit, optionally writing the path and a run manifest
.\rcvd-cli.exe lap --vehicle examples/ch5.json --units us --in-units us `
                --cf -640 --cr -580 --speed 27.3 --steer 3.99 `
                --inner 100 --outer 400 --start 150 `
                --csv path.csv --manifest run.json
```

`--in-units` and `--units` are independent, so US-customary input can print as SI
and vice versa. Cornering stiffnesses are **axle** values and must be negative
(§3.3). Everything between the two unit boundaries is SI (Rule U1), which is why
the lap time is identical to all printed digits whichever display system you pick.

`--digits N` sets the significant figures, 1 to 17, default 6. Use 17 when you
need to compare front ends: §18.3 demands bit-for-bit agreement, and six figures
cannot demonstrate it. Seventeen round-trips a binary64 exactly.

## Running the GUI

`rcvd-gui` is Qt 6 Widgets (§16): a launcher shell plus one window per program,
each with the Define / Run / Results / Diagnostics tabs §16.1 requires.

```powershell
pwsh tools\build_gui.ps1 -Run
```

The script finds Qt under `C:\Qt` and builds through CMake — unlike `rcvd-cli`,
the GUI cannot be built with a bare `cl` invocation, because its `Q_OBJECT`
widgets need `moc`. If Qt is missing, the script prints the two commands that
install just `qtbase` (about 260 MB, no account needed):

```powershell
pip install aqtinstall
python -m aqt install-qt windows desktop 6.8.3 win64_msvc2022_64 --archives qtbase --outputdir C:\Qt
```

Other entry points, all of which open no window:

| command | what it does |
|---|---|
| `rcvd-gui P3` | skip the shell, open one program directly |
| `rcvd-gui --selftest [P#]` | every result at 17 significant figures |
| `rcvd-gui --dump-vehicle P1` | the §5.1 document the Define tab would save |
| `rcvd-gui --dump-cli P1 <doc>` | the command line the Run tab displays |
| `rcvd-gui --render P1 out.png [tab]` | rasterise a window to PNG |

`rcvd-gui` is a WIN32-subsystem binary, so it has no console of its own: to see
the output of those modes, redirect it (`cmd /c "rcvd-gui --selftest > out.txt"`)
or use `pwsh tools\build_gui.ps1 -Selftest`.

### What Rule A1 means here

§4.1 forbids `app` from containing any equation, any unit conversion beyond
calling `units`, and any numerical algorithm. The GUI obeys that structurally
rather than by inspection:

* `app/gui/program_spec.hpp` describes each program as a field list plus a run
  function that is a sequence of calls into `models`, `analysis` and `tire`.
  There is no arithmetic on a physical quantity anywhere in `app/gui`.
* The Define tab does not hand its values to `core::resolve`. It writes a real
  §5.1 vehicle document and feeds it through `io::load_vehicle` — the same
  function `rcvd-cli` calls on a file — so both front ends share one input path.
* Quantities with no entry in §3.4's closed vocabulary (the stability factor K,
  the MMM indices, `rcond`) are labelled verbatim and never converted, so nothing
  in the GUI can masquerade as a unit conversion.
* The deg/g form of the understeer gradient is a core function
  (`models::understeer_gradient_deg_per_g`), not a conversion, because Rule U4
  makes g a field constant rather than a unit.

### §18.3 is closed in both halves

> "Any behaviour reachable through the GUI SHALL be reachable through `rcvd-cli`
> and through the C API with identical results, bit-for-bit."

`tests/test_capi.cpp` closes the C++/C half. `tools/check_front_ends.py` closes
the GUI/CLI half, and runs as the ctest case `front_ends_agree_bit_for_bit`
whenever the GUI was built. It retypes no number: the GUI emits the vehicle
document, emits its own equivalent command line, runs it through `rcvd-cli
--digits 17`, and the two sets of digits are compared as strings. Seventeen
significant figures round-trip a binary64 exactly, so string equality there is
bit equality.

That gate earned its keep on its first run, by catching the Run tab emitting
`--start 0` for a field whose zero means "default": `rcvd-cli` reads a supplied
zero literally, which puts the car inside the inner wall, so the displayed
command did not reproduce the GUI's own run.

## Building

Requires CMake ≥ 3.24 and a C++20 compiler. The CMake configure step fails
outright if any `-ffast-math`-family flag is present, rather than leaving it to
the Appendix D audit.

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

## The §14.2 register documents behaviour, not intent

Appendix D requires that "every warning and error code in §14.2 has a test that
provokes it". `tests/test_diagnostic_coverage.cpp` is that gate, and it is
mechanical: it walks `core::codes::kRegister` and fails — naming each one — if
any entry was not actually raised by library code. It also checks the converse,
so a typo'd code cannot masquerade as coverage.

When first written it found **five codes that no code path could emit**:
`E-TIRE-020`, `EV-CLAMP-MU`, `EV-CLAMP-ELLIPSE`, `W-NUM-220` and `W-TIRE-101`.
All five now have emission points.

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
             step_guidance.hpp  §13.3 step adequacy, W-NUM-200/210
             accumulate.hpp  Neumaier compensated summation (§13.1, A2)
  tire/      tire_model.hpp  the §6.1 contract; TireState/TireOutput (§4.3)
             linear.hpp      L1 (6.1) and L2 (6.3), with (6.10) load sensitivity
             conformance.hpp the shared T1-T8 check, applied to every model
             table.hpp       L3 tabular, PCHIP tensor product (§6.4, D-2)
             radt.hpp        L4 Radt non-dimensional (§6.5)
             magic_formula.hpp  L5 Pacejka, pure lateral slip (§6.6)
             friction.hpp    (6.9) load sensitivity, (6.11) ellipse
             relaxation.hpp  (6.13)-(6.16) relaxation-length lag
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
  capi/      rcvd.h          the §4.1 extern "C" ABI (pure C)
  io/        display.hpp     Rule U2's display boundary, SI or US customary
             json.hpp        minimal JSON/jsonc reader and writer
             document.hpp    §5 document layer, Rule U2's input boundary
             csv.hpp         §15.1 RFC 4180 output, %.17g lossless
             manifest.hpp    §4.5 run manifest
tests/       Catch2 v3 suite, authoritative
  support/   defective_tire.hpp — fault injection for the conformance suite
             fixtures.hpp       — CH5, CH5-OS and "First Example" (§17.2)
src/capi/    rcvd_capi.cpp — the C ABI implementation
app/         rcvd_cli.cpp — the §4.1 command-line front end
examples/    ch5.json — the §17.2 reference vehicle as a §5.1 document
tools/       verify_standalone.cpp — dependency-free mirror
             check_compile_fail.py — the Rule U6 negative cases
```

## Units: SI inside, your choice outside

Rule U1 makes the core compute in strict SI, always. Rule U2 allows exactly two
conversions per data path — one on input, one on display — so the output system
is a presentation choice that provably cannot reach a computed value:

```bash
rcvd-cli derivs --vehicle examples/ch5.json --units us --in-units us          --cf -640 --cr -580 --speed 30
```

```
  Y_beta            -1220  lbf/deg          B-03
  Y_r             10.9646  lbf*s/deg        B-04
  Y_delta             640  lbf/deg          B-05
  N_beta          482.442  lbf*ft/deg       B-06
  N_r            -528.739  lbf*ft*s/deg     B-07
  N_delta         2396.53  lbf*ft/deg       B-08
  K_us           0.552209  deg/g            B-26
```

`--units si` prints the same run in N/rad and N*m/rad. The lap solver gives a
lap time identical to all printed digits either way, because the display system
is applied after the last arithmetic.

"American format" is not one convention — §17.2 quotes speeds in both ft/s and
mph on the same page — so `DisplayUnits` takes per-dimension overrides on top of
whichever system is selected.

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
