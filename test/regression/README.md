# ROSCO C++ controller regression suite

30 scenarios drive the compiled `libdiscon` through the controller's modes and
compare **7,260,000 float64 values** (6,292,000 controller outputs, plus the
time and wind-speed inputs each baseline records) against frozen baselines — *bit-for-bit,
not to a tolerance*. It is the strongest guarantee in the repo: if it passes, a
refactor changed no controller behaviour at all. Full run is about 75 s.

The baselines were captured from the verified pure-C++ build and are
byte-identical to the original Fortran outputs (see `REFACTOR_NOTES.md`). They
are the crown jewels — treat a diff as a bug in your change until proven
otherwise.

## What these scenarios are — and are not

Read this before trusting a green run to mean more than it does.

- **This is a change detector, not a validation suite.** A passing run says the
  controller computes exactly what it computed before. It says nothing about
  whether that is *correct*, or whether the turbine would behave well. Nobody
  has checked these outputs against measurements.
- **The plant is a 1-DOF rotor spin-up**, not OpenFAST: one rotor inertia driven
  by a Cp surface (`sim_ws_series`, or the loop in `run_synthetic`). There is no
  tower, no blade flexibility, no real wind field. Most signals a real turbine
  would feed back — blade root moments, tower acceleration, nacelle IMU — are
  **zero unless a scenario injects them**.
- **A mode being on does not mean it is exercised.** Several scenarios switch a
  feature on while its input stays zero, so the feature runs and contributes
  nothing. That still pins the code path against crashes and bit drift, which is
  what it is for, but it is weaker than it looks. Scenario 3 says so explicitly;
  scenarios 2 and 27 turned out to be weaker still (see "Two scenarios test less
  than they were written to" below).
- **`mode_coverage.py` tells you what is configured**, not what executed. Neither
  it nor a green run proves a branch was taken; only coverage instrumentation
  would.
- **Scenario numbers are permanent**, fixtures are generated from a table and are
  never hand-edited, and a baseline changes only by deliberate decision. Those
  three rules are what keep the suite trustworthy; the sections below give the
  detail.

If you are adding a feature to the controller, the honest question is not "does
the suite still pass" — it should — but "does anything here execute my new code,
with a non-zero input?" If not, add a scenario.

## Running it

```bash
pytest test/regression                              # one result per scenario
python test/regression/run_regression.py            # same check, CLI report
python test/regression/run_regression.py --scenario 3
python test/regression/run_regression.py --rebuild  # cmake build first
python test/regression/run_regression.py --hdf5     # also check HDF5 output
pytest test/regression/test_tuning.py               # tuning only, ~3 s, no DLL
python test/regression/mode_coverage.py --gaps      # what no scenario configures
python test/regression/compare_baselines.py         # what a baseline change changed
```

Expected: `RESULT: ALL IDENTICAL — 7,260,000 total float64 values compared`.

No setup steps beyond installing the package (`pip install -e .`, which builds
the DLL into `rosco/lib/`). The suite regenerates every input it needs from
`Examples/Tune_Cases/NREL5MW.yaml` and writes all scratch output — the
generated `DISCON_*.IN`, the controller's `*.RO.dbg*` — into a temp directory
that is deleted on exit. Nothing is left in the working tree.

`--rebuild` builds into `rosco/controller/build` and copies `libdiscon.*` into
`rosco/lib/`. That path is the one build directory the repo uses; the CMake
presets and `.github/copilot-instructions.md` agree with it.

## Reading a failure

A mismatch reports the array, `max_diff`, and `first_diff_idx`:

```
scenario_13: gen_torque: max_diff=4.55e-13 first_diff_idx=18204
```

`first_diff_idx` × 0.025 s is the simulation time where the paths diverged —
usually more informative than the magnitude, because a tiny `max_diff` late in
the run and a tiny one at index 0 are very different bugs. To see it:

```bash
python test/regression/plot_regression.py --scenario 13 --output /tmp/plots
```

Then re-run just that scenario while you bisect:
`python test/regression/run_regression.py --scenario 13 --rebuild`.

## Changing a baseline on purpose

Almost never. A baseline changes only when the controller's output is *meant*
to change — a bug fix with a known physical effect, an intentional algorithm
change, or a scenario deliberately rewritten to test something it did not test
before. A refactor, a cleanup, or a port must not move a single value.

The rule that makes this safe: **you predict the change before you look at it,
and the evidence has to match your prediction.**

### The procedure

1. **Write down what you expect, first.** Which scenarios should move, which
   arrays within them, roughly how much, and from what time. A prediction made
   after seeing the diff is not evidence.
2. **Make the change and run the suite.** Everything you did *not* predict must
   still be `IDENTICAL`. A scenario you expected to be untouched moving is a
   bug in your change — stop there.
3. **Regenerate only the affected scenarios**, never the whole set:
   ```bash
   python test/regression/run_regression.py --rebuild --scenario 2 --update-baseline
   ```
   Each capture appends a record to `baselines/PROVENANCE.json` naming the
   scenarios it covers, so the other baselines keep their own history.
4. **Inspect what actually moved:**
   ```bash
   python test/regression/compare_baselines.py --plots /tmp/baseline-change
   ```
   It reports, per array: max absolute change, size relative to that signal's
   peak, how many samples moved, and the time of first divergence — plus
   before/after and difference plots. Compare it with your step-1 prediction.
5. **Re-run the full suite** so the new baselines are what a clean run
   reproduces, and confirm the run is deterministic (run the changed scenarios
   a few times; the printed MD5s must agree).
6. **Commit the baselines on their own**, after the code change, with the
   evidence in the message: which scenarios, which arrays, the magnitudes, the
   time of first divergence, and *why the controller now behaves differently*.
   Attach the plots to the PR. `PROVENANCE.json` goes in the same commit.

### Reading the evidence

- **First divergence time** matters more than magnitude. A change that starts
  at t = 0 is a different bug from one that appears at t = 187 s. If you changed
  a filter's initialisation, expect t = 0; if you changed a mode that only
  engages above rated, expect the time the wind crosses it.
- **Size relative to peak** tells you whether you changed the answer or the
  last bits. ~1e-13 of peak is floating-point reassociation — which is *not*
  acceptable from a refactor: it means the arithmetic moved, and this suite
  exists to catch exactly that.
- **An array you did not expect to move** is the important signal. Cable,
  structural and flap channels sit at zero in most scenarios; if one wakes up,
  something is now feeding it.

### Worked example: fixing scenarios 2 and 27

These two are the open case in this repo (see "Two scenarios test less than
they were written to"). If you take it on, the prediction is already written:

- Pass the synthetic signals through `turbine_state` in `run_synthetic()`
  instead of writing avrSWAP directly.
- **Scenario 2:** expect `nac_yaw`, `bld_pitch*` and `gen_torque` to move,
  starting at t = 0 (the vane is non-zero from the first step), and the yaw
  channels to become non-trivial rather than flat.
- **Scenario 27:** expect pitch and torque channels to move from t = 0, since
  tower damping and floating feedback start contributing pitch immediately.
- **Expect nothing else to move at all** — no other scenario shares that loop
  state. If scenario 7 or 8 moves, the change leaked.
- The commit message should say the scenarios were not testing what they
  claimed, name the mechanism (`call_controller` overwrites avrSWAP(24), (37),
  (53), (83) from `turbine_state`), and state that the new baselines are the
  first ones to exercise those inputs.

## Determinism — why the machinery is here

Bit-identical comparison only works because three things are true. All three
are easy to break by accident.

1. **`-ffp-contract=off` / `/fp:precise`.** Set in
   `rosco/controller/CMakeLists.txt`. Without it the compiler fuses
   multiply-adds and the same source produces different bits at different
   optimisation levels. **No CMake preset may override these or add
   `-ffast-math`** — a preset that does will fail the whole suite in a way that
   looks like a controller bug.
2. **One subprocess per scenario.** The controller DLL holds static state that
   is only reset by unloading the process, so `run_regression.py` runs each
   scenario in a fresh `python scenarios.py --scenario N`.
3. **`libscrub.so`.** Scenario 3 is otherwise non-deterministic: scipy's
   FITPACK `fpbisp` reads an uninitialised stack variable whose value depends
   on residual stack contents from earlier Fortran calls. `scenarios.py` caches
   the `RectBivariateSpline` and scrubs 64 KB of stack before each evaluation
   (dev note 202603261512). `run_regression.py` builds the library on demand
   with `gcc`; if that fails it warns and continues, and scenario 3 may then
   flap.

HDF5 output is optional. `--hdf5` and `test_hdf5_matches_text_output` skip
rather than fail when `h5py` or libhdf5 is absent.

## What is verified today

Each scenario captures the controller's avrSWAP outputs (`gen_torque`,
`bld_pitch`, `gen_speed`, `gen_power`, `nac_yaw`, plus per-blade pitch, flap,
cable and structural-control channels) over a 1000 s, 0.025 s sim.

### What each test covers

Three separate things happen between the tuning YAML and a baseline array. If
one test covered all three, every failure would look the same — a float
mismatch in a time series — no matter which part actually broke.

| Step | Transformation | Owned by | Test |
|---|---|---|---|
| Tuning | `NREL5MW.yaml` → `scenario_01.IN` | Python toolbox (`tune_controller`) | `test_tuning.py` |
| Input parsing | `DISCON.IN` → `ControlParameters` | generated C++ parser | *none yet — see below* |
| Control | `ControlParameters` + plant → time series | C++ controller | `test_regression.py` |

The committed fixtures in `fixtures/` are the boundary between tuning and
control, so each side fails on its own terms. **The tuner does not run during a
regression run** — scenarios read their committed fixture, so wisdem and scipy
cannot break a controller test.

Input parsing — "did the controller read what the file actually said?" — has no
test yet. It needs the controller to report the values it parsed. `Echo` is
declared and parsed but never implemented (nothing writes the file), so that
check is blocked until a parameter dump exists. This is the one step that has
already produced a silent bug: a generated parser ignored the registry's
per-field defaults, and no fixture happened to set the affected parameter.

### The fixtures

**Scenario 1 applies no patches, so `fixtures/scenario_01.IN` *is* the raw
tuner output.** That is deliberate: `test_tuning.py` pins the very file that
scenario 1 runs on, so the tuning check and the controller check cannot drift
apart — there is only one file. If scenario 1 ever gains patches,
`test_tuning.py` will need a fixture of its own.

Every other `fixtures/scenario_NN.IN` is that same tuner output plus the
scenario's own parameter changes. So a scenario's definition is readable as a
diff:

```bash
diff fixtures/scenario_01.IN fixtures/scenario_10.IN
```

Path parameters (`PerfFileName`, `OL_Filename`) are stored **relative to the
fixture file**, which is what makes them portable — the controller resolves a
relative value against the directory of the DISCON file itself. Never write an
absolute path into a fixture.

To regenerate after an intentional tuner change:

```bash
python test/regression/scenarios.py --write-fixtures
```

That re-runs the tuner once, writes `scenario_01.IN`, and builds every other
fixture from it by applying the scenario's `patches` from the table in
`scenarios.py`. It does not run the scenarios: follow it with
`run_regression.py`, which must still be `ALL IDENTICAL` — a fixture change that
moves a baseline is a finding, not a baseline to update.

Regeneration is **idempotent**: running it with an unchanged tuner produces a
zero-line diff. That is why the version/date stamp `write_DISCON` puts on line 2
is normalised away — left in, every regeneration would dirty all 28 files and a
real tuner change would be invisible in the noise. Git already records when a
fixture changed. A non-empty `git diff fixtures/` therefore means something
real moved.

**Do not hand-edit a fixture.** Change the scenario's `patches` and
regenerate. `test_fixtures.py` checks every committed fixture equals
`scenario_01.IN` plus its `patches`, so a hand-edit fails there even when it
touches a parameter inert under the scenario's modes, which the baselines
would never notice.

| # | Exercises |
|---|-----------|
| 1 | Standard step-wind 1-DOF sim; also re-runs to check DLL deallocation |
| 2 | Yaw-by-IPC, `Y_ControlMode=2` — with zero yaw error; the intended synthetic vane/heading never reaches the controller (see below) |
| 3 | Notch filters, cable control, many mode flags at once |
| 4 | Flap control, `Flp_Mode=2` (`PIIController`) |
| 5 | Active wake control, `AWC_Mode=4` (`ResController`) |
| 6 | IPC, `IPC_ControlMode=1` (`NotchFilterSlopes`) |
| 7 | Synthetic inputs to functions that see zeros in the 1-DOF sim |
| 8 | IPC + AWC with non-zero blade root moments |
| 9 | Startup, shutdown, reference-speed exclusion |
| 10 | Rotor position control, `OL_Mode=2` (`PIDController`) |
| 11 | Open-loop complex-number AWC, `AWC_Mode=1` |
| 12 | K·Ω² torque control, `VS_ControlMode=1` |
| 13 | Fixed blade pitch power overspeed, `VS_FBP=1` |
| 14 | Time-based open-loop pitch/torque/yaw, `OL_Mode=1` |
| 15 | Coleman-transform AWC, `AWC_Mode=2` |
| 16 | Coleman-transform cyclic flap control, `Flp_Mode=3` |
| 17 | I&I wind speed estimator, `WE_Mode=1` |
| 18 | 2P harmonic IPC, `IPC_ControlMode=2` |
| 19 | Pitch actuator LP + pitch offset fault + constant power + `Fl_Mode=2` |
| 20 | Pitch actuator SecLP + pitch stuck fault + PRC lookup table |
| 21 | Closed-loop PI AWC, `AWC_Mode=3` |
| 22 | Strouhal-based AWC, `AWC_Mode=5` |
| 23 | Disabled paths, `PS_Mode=0` + `SS_Mode=0` |
| 24 | Open-loop cable and structural control, `CC_Mode=2` + `StC_Mode=2` |
| 25 | Dynamic power rating, `PRC_Mode=2` |
| 26 | `Flp_Mode=3` driven to non-zero flap output |
| 27 | Stress test: many modes at once — tower damping and floating feedback get zero input (see below) |
| 28 | HDF5 output format — same sim as 1, `OutputFormat=1`, `LoggingLevel=3` |
| 29 | Power-based TSR tracking, `VS_ControlMode=3` — what the NREL-2.8 and MHK_RM1 Test_Cases run |
| 30 | Constant torque above rated, `VS_ConstPower=0` — what the IEA-15, BAR_10 and NREL-2.8 Test_Cases run |

### Two scenarios test less than they were written to
`ControllerInterface.call_controller()` writes avrSWAP(24), (37), (53) and (83)
from its `turbine_state` argument on every call, overwriting anything set
directly beforehand. Scenario 2 set a synthetic nacelle vane and heading that
way, and scenario 27 set tower and IMU accelerations that way; all four arrive
at the controller as 0. Their baselines record that behaviour, so it is kept
exactly. Making them do what was intended means passing the signals through
`turbine_state`, which will move both baselines — a deliberate decision, not a
refactor. The procedure, with the expected outcome already written down, is
under "Changing a baseline on purpose" above.

Scenario 28 has no baseline file of its own. It is scenario 1's simulation with
HDF5 logging at `LoggingLevel=3`, and logging must not change a single control
output, so `run_regression.py` compares it bit-for-bit against
`baselines/scenario_1.npz` (`SHARED_BASELINE`). Separately, `--hdf5` and
`test_hdf5_matches_text_output` check its `.RO.h5` against scenario 1's text
`.RO.dbg`, and its `/avrSWAP` dataset against the avrSWAP values captured in the
sim loop.

### What is *not* covered

```bash
python test/regression/mode_coverage.py          # every mode value × who configures it
python test/regression/mode_coverage.py --gaps   # only values no scenario sets
```

This tabulates each mode parameter's values across the fixtures, the committed
`Examples/Test_Cases/` DISCON files, and any DISCON files that running the
Examples has left in `Examples/examples_out/` (gitignored, so that column
depends on what you have run). A setting whose parent mode is off is not
counted — `IPC_SatMode` means nothing with IPC off. It reports what is
*configured*, not what *executes*.

As of 2026-09-22, no scenario configures:

- `F_LPFType = 2` (second-order low-pass), `WE_Mode = 0`, `IPC_SatMode = 0/1/3`,
  `Ext_Mode = 1`, `ZMQ_Mode = 1` — each used by at least one Example. The last
  two need an external DLL and a ZeroMQ server, so they do not suit this suite.
- `VS_ControlMode = 0/4`, `VS_FBP = 2/3`, `PRC_Comm = 1/2`, `OL_BP_Mode = 1`,
  `Ext_Interface = 0`, `LoggingLevel = 0/2` — configured nowhere in the repo.

Scenarios 29 and 30 closed the gap that mattered most: `VS_ControlMode = 3` and
`VS_ConstPower = 0`, the torque control every real turbine configuration in
`Test_Cases/` uses.

`test_mode_coverage.py` fails if the registry gains a mode the report does not
know about, so the table cannot silently go stale.

**Scenario numbers are permanent.** They name the baseline files and are cited
in `REFACTOR_NOTES.md` and commit history. Never renumber one.

## Adding a scenario

1. Add a `Scenario(N, ...)` entry to `_SCENARIO_LIST` in `scenarios.py`: its
   `patches` against scenario 1, the wind (`tlen`, `ws0`, `step_wind`), and —
   only if the toolbox simulation leaves a signal you need at zero — the
   `synthetic` inputs for the manual loop. Most scenarios need nothing else.
2. Add `N` to `scenario_order` in `main()`, and append it to `ALL_SCENARIOS`
   in `run_regression.py`.
3. Generate its fixture: `python test/regression/scenarios.py --write-fixtures`,
   and commit `fixtures/scenario_NN.IN`. Check the diff against
   `scenario_01.IN` reads as the scenario you meant to write.
4. Confirm it is deterministic: run it 5+ times and check the MD5s printed for
   each array agree.
5. Capture the baseline:
   `python test/regression/run_regression.py --scenario N --update-baseline`.
6. Add a row to the table above, and confirm
   `python test/regression/run_regression.py` is still `ALL IDENTICAL`.

## Layout

```
test/regression/
    README.md              this file
    run_regression.py      CLI runner — build, run, compare, report
    scenarios.py           the scenario table and the runners that execute it
    test_regression.py     pytest wrapper: one test per scenario
    test_tuning.py         tuner still reproduces scenario_01.IN
    test_fixtures.py       each fixture is still scenario_01.IN + its patches
    compare_baselines.py   what a baseline change actually changed
    mode_coverage.py       which mode values the scenarios and Examples configure
    test_mode_coverage.py  keeps mode_coverage.py in step with the registry
    plot_regression.py     failure-diagnosis plots
    fixtures/              committed DISCON inputs — one per scenario
    baselines/             29 compressed .npz files (~9 MB; 28 shares 1's) + PROVENANCE.json
```

This lives at the repo top level rather than under `rosco/` so the baselines
stay out of the installed wheel. `rosco/test/` holds the Python toolbox tests
and is a separate thing.
