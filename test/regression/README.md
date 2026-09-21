# ROSCO C++ controller regression suite

27 scenarios drive the compiled `libdiscon` through the controller's modes and
compare **5,252,000 float64 values** against frozen baselines — *bit-for-bit,
not to a tolerance*. It is the strongest guarantee in the repo: if it passes, a
refactor changed no controller behaviour at all. Full run is about 75 s.

The baselines were captured from the verified pure-C++ build and are
byte-identical to the original Fortran outputs (see `REFACTOR_NOTES.md`). They
are the crown jewels — treat a diff as a bug in your change until proven
otherwise.

## Running it

```bash
pytest test/regression                              # one result per scenario
python test/regression/run_regression.py            # same check, CLI report
python test/regression/run_regression.py --scenario 3
python test/regression/run_regression.py --rebuild  # cmake build first
python test/regression/run_regression.py --hdf5     # also check HDF5 output
pytest test/regression/test_tuning.py               # tuning only, ~3 s, no DLL
```

Expected: `RESULT: ALL IDENTICAL — 5,252,000 total float64 values compared`.

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

## Updating a baseline

Almost never. A baseline changes only when the controller's output is *meant*
to change — a bug fix with a known physical effect, or an intentional algorithm
change. A refactor, a cleanup, or a port must not move a single value.

```bash
python test/regression/run_regression.py --rebuild --update-baseline
```

Required discipline when you do:

- **Its own commit**, touching only `baselines/`, separate from the code change.
- `--update-baseline` rewrites `baselines/PROVENANCE.json` (git SHA, platform,
  numpy/scipy versions, `libdiscon` hash). Commit it alongside — every run
  prints it, so "what are we comparing against?" is answerable without
  `git log` archaeology.
- **Justify it in the commit message**: which scenarios moved, by how much, and
  the physical reason.
- **Attach before/after plots** from `plot_regression.py` to the PR.

If a baseline moved and you cannot explain why, you have found a bug, not a
stale baseline.

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

That re-runs the tuner, re-applies each scenario's `patches=`, and rewrites the
paths. It must still produce 27/27 identical — a fixture change that moves a
baseline is a finding, not a baseline to update.

Regeneration is **idempotent**: running it with an unchanged tuner produces a
zero-line diff. That is why the version/date stamp `write_DISCON` puts on line 2
is normalised away — left in, every regeneration would dirty all 28 files and a
real tuner change would be invisible in the noise. Git already records when a
fixture changed. A non-empty `git diff fixtures/` therefore means something
real moved.

**Do not hand-edit a fixture.** A hand-edit that changes behaviour is caught by
the baselines, but one that does not — touching a parameter inert under the
scenario's modes — would silently persist. Change `patches=` and regenerate.

| # | Exercises |
|---|-----------|
| 1 | Standard step-wind 1-DOF sim; also re-runs to check DLL deallocation |
| 2 | Yaw-by-IPC, `Y_ControlMode=2` (`wrap_360`) |
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
| 27 | Stress test: 11 modes active simultaneously |
| 28 | HDF5 output format — same sim as 1, `OutputFormat=1`, `LoggingLevel=3` |

Scenario 28 has no frozen baseline; it is compared against scenario 1's text
output instead, and is excluded from `ALL_SCENARIOS`.

**Scenario numbers are permanent.** They name the baseline files and are cited
in `REFACTOR_NOTES.md` and commit history. Never renumber one.

## Adding a scenario

1. Write `run_scenario_N()` in `scenarios.py` — copy the closest existing one,
   change the `discon_fixture(N, patches={...})` modes, and give
   `ControllerInterface` a unique `sim_name='regression_N'`.
2. Register it in `scenario_functions` and `scenario_order` in `main()`, and
   append `N` to `ALL_SCENARIOS` in `run_regression.py`.
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
    scenarios.py           the 28 scenario definitions
    test_regression.py     pytest wrapper: one test per scenario
    test_tuning.py         tuner still reproduces scenario_01.IN
    plot_regression.py     failure-diagnosis plots
    fixtures/              committed DISCON inputs — one per scenario
    baselines/             27 frozen .npz files (~40 MB) + PROVENANCE.json
```

This lives at the repo top level rather than under `rosco/` so the baselines
stay out of the installed wheel. `rosco/test/` holds the Python toolbox tests
and is a separate thing.
