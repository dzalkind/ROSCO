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

## Baselines are per platform

Baselines live in `baselines/<system>-<machine>/` — today `darwin-arm64/` and
`linux-x86_64/`. Every run prints the folder it is using, so a green run cannot
be mistaken for one that checked a different platform's numbers.

The split is not about the controller. **The controller's output reproduces
bit-for-bit across platforms** — that much was checked, and it is the thing this
suite exists to pin. What does not reproduce is the Python plant model that
drives it (`rosco/toolbox/sim.py`). On Linux x86_64 its `gen_speed` lands 1-2
ULP from the macOS arm64 value, and `gen_power` about 1e-9 from it on a
MW-scale signal. That is libm and FMA contraction inside numpy/scipy, not in
anything this repo compiles — `-ffp-contract=off` covers the controller, and
cannot reach into scipy. Against a single frozen set, that was enough to fail
most scenarios on Ubuntu while the controller was provably unchanged.

The alternative — comparing to a tolerance loose enough to absorb it — would
throw away exactly what the suite is for. A 1-ULP move in the controller is a
finding; a tolerance that hides the plant's last bits hides the controller's
too. So each platform keeps its own frozen set and the comparison stays exact.

A platform with no folder yet **skips** rather than fails: `pytest` skips all
31 tests, and `run_regression.py` exits 1 naming the folder it looked for, the
sets that do exist, and the command to generate each. Missing baselines are an
absent reference, not a regression, and should not read as one.

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
3. **Regenerate only the affected scenarios**, never the whole set — and
   regenerate them **on both platforms**. `--update-baseline` only ever writes
   the folder for the machine it runs on, so macOS is done locally and Linux
   comes from CI:
   ```bash
   # darwin-arm64, locally:
   python test/regression/run_regression.py --rebuild --scenario 2 --update-baseline

   # linux-x86_64, from CI. The branch must be pushed first — GitHub can only
   # run a workflow from a ref it has.
   git push
   test/regression/fetch_linux_baselines.sh
   ```
   `fetch_linux_baselines.sh` dispatches
   `.github/workflows/regression_linux_baselines.yml` with `gh workflow run`,
   watches it, and unpacks the artifact into `baselines/linux-x86_64/`. The
   workflow installs ROSCO exactly as the CI job that runs this suite does, so
   the environment matches. It needs the GitHub CLI, authenticated
   (`gh auth login`). Note it regenerates **every** scenario, since the job
   runs `--update-baseline` with no `--scenario`; the ones you did not intend
   to change must come back byte-identical, and `git status` is how you check.

   Each capture appends a record to that platform's
   `baselines/<platform>/PROVENANCE.json` naming the scenarios it covers, so
   the other baselines keep their own history. The histories are per platform
   deliberately: each set has its own dates, SHAs and numpy/scipy versions.
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
   Attach the plots to the PR. Both platform folders and both
   `PROVENANCE.json` files go in that one commit — leaving a set stale means
   the next person's CI fails for a reason that has nothing to do with their
   change. The two sets must move *together and for the same reason*: a
   scenario that moves on one platform and not the other is a finding, not a
   baseline update.

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

### Worked example: the 2026-09-22 change

Three things moved together, and it is worth reading as a case where the
prediction was right about *what* and wrong about *how much*.

The intended change was narrow: scenarios 2 and 27 were passing their synthetic
signals through avrSWAP indices that `call_controller` overwrites, so the
controller saw zeros (see "Two scenarios test less than they were written to",
below, as it read at the time). Routing them through `turbine_state` was
predicted to move those two baselines and nothing else.

It moved 27 of them. Chasing the unpredicted ones found a second, larger
problem: the harness built every `ControllerInterface` at the toolbox's default
`DT = 0.1` and then simulated at 0.025. Every filter in the controller sizes its
coefficients on the `iStatus == 0` call, so every filtered signal in the suite
had been running at four times its configured corner frequency. That is a
harness defect, not a controller one, and fixing it is what moved the other 25.

Two pieces of evidence closed it:

- The gate reproduces the new set exactly, twice over, and the two scenarios
  that did *not* move (13 and 14) are the two whose outputs never pass through
  a filter — 14 is a pure open-loop table lookup, 13 is a degenerate run with
  torque pinned. Nothing moved that had no mechanism to move.
- Upstream Fortran ROSCO 2.9.0, built from `main` with `-ffp-contract=off` and
  driven through the same harness, reproduces the new baselines bit-for-bit on
  27 of 30 scenarios. Before the fix it reproduced 26. The three that remain
  are known and documented: scenario 2 (an aliasing bug in the Fortran, where
  the C++ is correct — `Controllers.f90:517` passes `objInst%instSecLPF` to
  `LPFilter`, colliding with the nacelle-vane sine filter), scenario 8 (4.3e-19,
  rounding), and scenario 4 (the 2.9.0 parser cannot read the fixture).

Note the second point in particular: the old baselines were *also* cross-checked
against Fortran and *also* agreed, because the Fortran was mis-driven in exactly
the same way. `HPFilter` in the Fortran recomputes `K = 2/DT` on every call and
self-corrected, which is why scenario 27 was the one place the two disagreed —
and why it now agrees. Agreement with the reference implementation does not
prove the harness is driving either one correctly.

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
4. **One `DT`, declared once.** `scenarios.py` passes `DT=DT` when it builds
   every `ControllerInterface`, and the simulation loop steps at that same
   `DT`. The controller sizes every filter coefficient on its first call
   (`iStatus == 0`) and caches them, so a `ControllerInterface` built at the
   toolbox default of 0.1 and then stepped at 0.025 runs every filter in the
   suite at four times its configured corner frequency. This is not a variable
   timestep — the suite does not test those — it is a single constant `DT` that
   the construction call and the loop must agree on. Changing `DT` moves 28 of
   the 30 baselines.

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

### Synthetic inputs go through `turbine_state`, not avrSWAP
`ControllerInterface.call_controller()` writes avrSWAP(24), (37), (53) and (83)
from its `turbine_state` argument on every call, overwriting anything set
directly beforehand. Scenarios 2 and 27 once set a synthetic nacelle vane,
heading and tower/IMU accelerations by writing those indices, and all four
arrived at the controller as 0 — so for two years those scenarios recorded a
controller that was being fed nothing. Fixed 2026-09-22. If you add a synthetic
input, set it in `turbine_state` inside `run_synthetic()`, and confirm the
baseline actually moves; a new input that changes nothing is the symptom.

Scenario 7 has a related but different weakness: its tower and floating inputs
*do* reach the controller, but the fixture leaves `FA_KI`, `FA_IntSat` and
`Fl_Kp` at zero, so those paths contribute exactly zero however hard they are
driven. Scenario 27 was given non-zero gains for this reason; scenario 7 has
not been, and its tower/floating coverage is still nominal only.

Scenario 28 has no baseline file of its own. It is scenario 1's simulation with
HDF5 logging at `LoggingLevel=3`, and logging must not change a single control
output, so `run_regression.py` compares it bit-for-bit against
its platform's `scenario_1.npz` (`SHARED_BASELINE`). Separately, `--hdf5` and
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
5. Capture the baseline on both platforms —
   `python test/regression/run_regression.py --scenario N --update-baseline`
   locally, then `test/regression/fetch_linux_baselines.sh` for
   `linux-x86_64` (see "Baselines are per platform"). A scenario with only one
   platform's baseline fails on the other.
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
    fetch_linux_baselines.sh  download the linux-x86_64 baselines from CI
    fixtures/              committed DISCON inputs — one per scenario
    baselines/             one folder per platform, each holding
        darwin-arm64/      29 compressed .npz files (~9 MB; 28 shares 1's) + PROVENANCE.json
        linux-x86_64/      the same set, captured on ubuntu-latest
```

This lives at the repo top level rather than under `rosco/` so the baselines
stay out of the installed wheel. `rosco/test/` holds the Python toolbox tests
and is a separate thing.
