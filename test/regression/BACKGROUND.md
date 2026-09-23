# Regression suite — background

Why the suite is built the way it is, what it does and does not prove, and the
history worth not repeating. For how to run it or change a baseline, see
[README.md](README.md).

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
  what it is for, but it is weaker than it looks.
- **`mode_coverage.py` tells you what is configured**, not what executed.
  Neither it nor a green run proves a branch was taken; only coverage
  instrumentation would.

If you are adding a feature to the controller, the honest question is not "does
the suite still pass" — it should — but "does anything here execute my new code,
with a non-zero input?" If not, add a scenario.

## Determinism — why the machinery is here

Bit-identical comparison only works because four things are true. All four are
easy to break by accident.

1. **`-ffp-contract=off` / `/fp:precise`.** Set in
   `rosco/controller/CMakeLists.txt`. Without it the compiler fuses
   multiply-adds and the same source produces different bits at different
   optimisation levels. **No CMake preset may override these or add
   `-ffast-math`** — a preset that does will fail the whole suite in a way that
   looks like a controller bug.
2. **One subprocess per scenario.** The controller DLL holds static state that
   is only reset by unloading the process, so `run_regression.py` runs each
   scenario in a fresh `python scenarios.py --scenario N`.
3. **`libscrub.so`.** Scenario 3 is otherwise non-deterministic: scipy's FITPACK
   `fpbisp` reads an uninitialised stack variable whose value depends on
   residual stack contents from earlier Fortran calls. `scenarios.py` caches the
   `RectBivariateSpline` and scrubs 64 KB of stack before each evaluation (dev
   note 202603261512). `run_regression.py` builds the library on demand with
   `gcc`; if that fails it warns and continues, and scenario 3 may then flap.
4. **One `DT`, declared once.** `scenarios.py` passes `DT=DT` when it builds
   every `ControllerInterface`, and the simulation loop steps at that same `DT`.
   The controller sizes every filter coefficient on its first call
   (`iStatus == 0`) and caches them, so a `ControllerInterface` built at the
   toolbox default of 0.1 and then stepped at 0.025 runs every filter in the
   suite at four times its configured corner frequency. This is not a variable
   timestep — the suite does not test those — it is a single constant `DT` that
   the construction call and the loop must agree on. Changing `DT` moves 28 of
   the 30 baselines.

HDF5 output is optional. `--hdf5` and `test_hdf5_matches_text_output` skip
rather than fail when `h5py` or libhdf5 is absent.

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

A platform with no folder yet **skips** rather than fails: `run_regression.py`
exits 1 naming the folder it looked for, the sets that do exist, and the command
to generate each. Missing baselines are an absent reference, not a regression,
and should not read as one.

## What each test covers

Three separate things happen between the tuning YAML and a baseline array. If
one test covered all three, every failure would look the same — a float mismatch
in a time series — no matter which part actually broke.

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

## The fixtures

**Scenario 1 applies no patches, so `fixtures/scenario_01.IN` *is* the raw tuner
output.** That is deliberate: `test_tuning.py` pins the very file that scenario 1
runs on, so the tuning check and the controller check cannot drift apart — there
is only one file. If scenario 1 ever gains patches, `test_tuning.py` will need a
fixture of its own.

Every other `fixtures/scenario_NN.IN` is that same tuner output plus the
scenario's own parameter changes, so a scenario's definition is readable as a
diff:

```bash
diff fixtures/scenario_01.IN fixtures/scenario_10.IN
```

Path parameters (`PerfFileName`, `OL_Filename`) are stored **relative to the
fixture file**, which is what makes them portable — the controller resolves a
relative value against the directory of the DISCON file itself. Never write an
absolute path into a fixture.

Regeneration is idempotent: running it with an unchanged tuner produces a
zero-line diff. That is why the version/date stamp `write_DISCON` puts on line 2
is normalised away — left in, every regeneration would dirty all 28 files and a
real tuner change would be invisible in the noise. Git already records when a
fixture changed.

## Synthetic inputs go through `turbine_state`, not avrSWAP

`ControllerInterface.call_controller()` writes avrSWAP(24), (37), (53) and (83)
from its `turbine_state` argument on every call, overwriting anything set
directly beforehand. Scenarios 2 and 27 once set a synthetic nacelle vane,
heading and tower/IMU accelerations by writing those indices, and all four
arrived at the controller as 0 — so for two years those scenarios recorded a
controller that was being fed nothing. Fixed 2026-09-22.

If you add a synthetic input, set it in `turbine_state` inside `run_synthetic()`,
and **confirm the baseline actually moves**. A new input that changes nothing is
the symptom.

The same year, scenarios 7 and 27 were both found to have `FA_KI`, `FA_IntSat`
and `Fl_Kp` at zero, so their tower damper and floating feedback multiplied real
inputs by nothing. Both now carry small non-zero gains — chosen to keep the rotor
operating normally, since these are code-path tests and this turbine is neither
floating nor tower-damped, not tuned values for anything.

## What is *not* covered

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

## Case history: the 2026-09-22 change

Worth reading as a case where the prediction was right about *what* and wrong
about *how much*.

The intended change was narrow: scenarios 2 and 27 were passing their synthetic
signals through avrSWAP indices that `call_controller` overwrites, so the
controller saw zeros. Routing them through `turbine_state` was predicted to move
those two baselines and nothing else.

It moved 27 of them. Chasing the unpredicted ones found a second, larger
problem: the harness built every `ControllerInterface` at the toolbox's default
`DT = 0.1` and then simulated at 0.025. Every filter in the controller sizes its
coefficients on the `iStatus == 0` call, so every filtered signal in the suite
had been running at four times its configured corner frequency. That is a
harness defect, not a controller one, and fixing it is what moved the other 25.

Two pieces of evidence closed it:

- The gate reproduces the new set exactly, and the two scenarios that did *not*
  move (13 and 14) are the two whose outputs never pass through a filter — 14 is
  a pure open-loop table lookup, 13 is a degenerate run with torque pinned.
  Nothing moved that had no mechanism to move.
- Upstream Fortran ROSCO 2.9.0, built from `main` with `-ffp-contract=off` and
  driven through the same harness, reproduces the new baselines bit-for-bit on
  27 of 30 scenarios. Before the fix it reproduced 26. The three that remain are
  known: scenario 2 (an aliasing bug in the Fortran, where the C++ is correct —
  `Controllers.f90:517` passes `objInst%instSecLPF` to `LPFilter`, colliding
  with the nacelle-vane sine filter), scenario 8 (4.3e-19, rounding), and
  scenario 4 (the 2.9.0 parser cannot read the fixture).

Note the second point in particular: the old baselines were *also* cross-checked
against Fortran and *also* agreed, because the Fortran was mis-driven in exactly
the same way. `HPFilter` in the Fortran recomputes `K = 2/DT` on every call and
self-corrected, which is why scenario 27 was the one place the two disagreed —
and why it now agrees.

**Agreement with the reference implementation does not prove the harness is
driving either one correctly.**

## Reproducing the Fortran cross-check

The chain of custody is not a one-time artifact — it can be re-run. Check out
upstream ROSCO on `main` in a scratch worktree, build it with
`-ffp-contract=off`, and drive it through the same scenario fixtures. Expect 27
of 30 bit-identical, with the three exceptions above. Any *new* disagreement is
a finding about the C++ translation.
