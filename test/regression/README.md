# ROSCO C++ controller regression suite

30 scenarios drive the compiled `libdiscon` through the controller's modes and
compare **7,260,000 float64 values** against frozen baselines — *bit-for-bit, not
to a tolerance*. If it passes, a refactor changed no controller behaviour at
all. Full run is about 75 s.

The baselines are the crown jewels: treat a diff as a bug in your change until
proven otherwise. **A passing run says the controller computes what it computed
before — not that the result is correct.** That distinction, and the rest of the
reasoning behind this suite, is in [BACKGROUND.md](BACKGROUND.md).

## Running it

```bash
pytest test/regression                              # one result per scenario
python test/regression/run_regression.py            # same check, CLI report
python test/regression/run_regression.py --scenario 3
python test/regression/run_regression.py --rebuild  # cmake build first
python test/regression/run_regression.py --hdf5     # also check HDF5 output
pytest test/regression/test_tuning.py               # tuning only, ~3 s, no DLL
python test/regression/mode_coverage.py --gaps      # what no scenario configures
```

Expected: `RESULT: ALL IDENTICAL — 7,260,000 total float64 values compared`.

No setup beyond `pip install -e .`, which builds the DLL into `rosco/lib/`. The
suite regenerates every input it needs and writes all scratch output to a temp
directory that is deleted on exit. Nothing is left in the working tree.

`--rebuild` builds into `rosco/controller/build` and copies `libdiscon.*` into
`rosco/lib/`.

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

Baselines are per platform (`baselines/darwin-arm64/`, `baselines/linux-x86_64/`)
and every run prints which folder it used. A platform with no folder **skips**
rather than fails. Why the split exists: [BACKGROUND.md](BACKGROUND.md#baselines-are-per-platform).

## Regenerating a baseline

Almost never. A baseline changes only when the controller's output is *meant* to
change — a bug fix with a known physical effect, an intentional algorithm change,
or a scenario deliberately rewritten to test something it did not test before. A
refactor, a cleanup, or a port must not move a single value.

**The rule that makes this safe: you predict the change before you look at it,
and the evidence has to match your prediction.**

1. **Write down what you expect, first.** Which scenarios should move, which
   arrays within them, roughly how much, and from what time. A prediction made
   after seeing the diff is not evidence.

2. **Make the change and run the full suite.** Everything you did *not* predict
   must still be `IDENTICAL`. A scenario you expected to be untouched moving is
   a bug in your change — stop there.

3. **Regenerate only the affected scenarios**, locally:
   ```bash
   python test/regression/run_regression.py --rebuild --scenario 7 --update-baseline
   ```
   This only ever writes the folder for the machine it runs on. Linux comes from
   CI — see the next section.

4. **Inspect what actually moved**, and check it against your step-1 prediction:
   ```bash
   python test/regression/compare_baselines.py --plots /tmp/baseline-change
   ```
   Per array: max absolute change, size relative to that signal's peak, how many
   samples moved, and the time of first divergence.

   For a change someone else has to review, build the report page instead — same
   evidence plus the plots, as one artifact you can hand over:
   ```bash
   python test/regression/baseline_report.py --out /tmp/report \
       --title "Scenario 7 Gains" --narrative note.html --notes notes.json
   ```

5. **Re-run the full suite** so the new baselines are what a clean run
   reproduces, and confirm determinism (run the changed scenarios a few times;
   the printed MD5s must agree).

6. **Commit the baselines** after the code change, with the evidence in the
   message: which scenarios, which arrays, the magnitudes, the time of first
   divergence, and *why the controller now behaves differently*.

### Reading the evidence

- **First divergence time** matters more than magnitude. If you changed a
  filter's initialisation, expect t = 0; if you changed a mode that only engages
  above rated, expect the time the wind crosses it.
- **Size relative to peak** tells you whether you changed the answer or the last
  bits. ~1e-13 of peak is floating-point reassociation — *not* acceptable from a
  refactor: it means the arithmetic moved, and this suite exists to catch that.
  Note the percentage is against the **old** signal's peak, so a channel that
  used to sit near zero can report far more than 100%.
- **An array you did not expect to move** is the important signal. Cable,
  structural and flap channels sit at zero in most scenarios; if one wakes up,
  something is now feeding it.

## Regenerating the linux-x86_64 baselines

Both platform folders must hold baselines captured from the *same* commit. A
local `--update-baseline` only rewrites `darwin-arm64/`, which leaves
`linux-x86_64/` stale and CI red until you do this. Do it as part of the same
change, not afterwards.

```bash
# 1. Commit the code + darwin-arm64 baselines, and push the branch.
#    GitHub can only run a workflow from a ref it has.
git push

# 2. Dispatch the workflow, wait for it, unpack the result.
test/regression/fetch_linux_baselines.sh
```

`fetch_linux_baselines.sh` runs `.github/workflows/regression_linux_baselines.yml`
with `gh workflow run`, watches it, and unpacks the artifact into
`baselines/linux-x86_64/`. It needs the GitHub CLI authenticated
(`gh auth login`). The workflow installs ROSCO exactly as the CI job that runs
this suite does, so the environment matches.

Then, before committing:

```bash
git status --short test/regression/baselines/linux-x86_64/
```

**The workflow regenerates every scenario**, because the job runs
`--update-baseline` with no `--scenario`. So this listing is the check that
matters: only the scenarios you intended to change may appear. Everything else
must come back byte-identical.

```bash
python test/regression/compare_baselines.py --against HEAD
```

Run on a Linux checkout this reports the same arrays and magnitudes as the macOS
run did. **The two sets must move together and for the same reason.** A scenario
that moves on one platform and not the other, or moves by a different amount, is
a finding — stop and explain it before committing.

Each capture appends a record to that platform's `baselines/<platform>/PROVENANCE.json`
naming the scenarios it covers. The histories are per platform deliberately:
each set has its own dates, SHAs and numpy/scipy versions.

## Regenerating the fixtures

After an intentional tuner change:

```bash
python test/regression/scenarios.py --write-fixtures
```

That re-runs the tuner once, writes `scenario_01.IN`, and builds every other
fixture from it by applying the scenario's `patches` from the table in
`scenarios.py`. It does not run the scenarios: follow it with
`run_regression.py`, which must still be `ALL IDENTICAL` — a fixture change that
moves a baseline is a finding, not a baseline to update.

Regeneration is idempotent, so a non-empty `git diff fixtures/` means something
real moved. **Do not hand-edit a fixture** — change the scenario's `patches` and
regenerate; `test_fixtures.py` enforces this.

## The scenarios

| # | Exercises |
|---|-----------|
| 1 | Standard step-wind 1-DOF sim; also re-runs to check DLL deallocation |
| 2 | Yaw-by-IPC, `Y_ControlMode=2`, with a yaw error that wraps past 0° and 360° |
| 3 | Notch filters, cable control, many mode flags at once |
| 4 | Flap control, `Flp_Mode=2` (`PIIController`) |
| 5 | Active wake control, `AWC_Mode=4` (`ResController`) |
| 6 | IPC, `IPC_ControlMode=1` (`NotchFilterSlopes`) |
| 7 | Synthetic yaw, tower, floating and flap inputs to functions the 1-DOF sim leaves idle |
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
| 27 | Stress test: six simultaneous pitch contributions through an actuator |
| 28 | HDF5 output format — same sim as 1, `OutputFormat=1`, `LoggingLevel=3` |
| 29 | Power-based TSR tracking, `VS_ControlMode=3` — what the NREL-2.8 and MHK_RM1 Test_Cases run |
| 30 | Constant torque above rated, `VS_ConstPower=0` — what the IEA-15, BAR_10 and NREL-2.8 Test_Cases run |

**Scenario numbers are permanent.** They name the baseline files and are cited
in `REFACTOR_NOTES.md` and commit history. Never renumber one.

Scenario 28 has no baseline file of its own — it is scenario 1's simulation with
HDF5 logging, which must not change a single control output, so it is compared
against `scenario_1.npz` (`SHARED_BASELINE`).

## Adding a scenario

1. Add a `Scenario(N, ...)` entry to `_SCENARIO_LIST` in `scenarios.py`: its
   `patches` against scenario 1, the wind (`tlen`, `ws0`, `step_wind`), and —
   only if the toolbox simulation leaves a signal you need at zero — the
   `synthetic` inputs for the manual loop.
2. Add `N` to `scenario_order` in `main()`, and append it to `ALL_SCENARIOS` in
   `run_regression.py`.
3. Generate its fixture: `python test/regression/scenarios.py --write-fixtures`.
   Check the diff against `scenario_01.IN` reads as the scenario you meant.
4. Confirm it is deterministic: run it 5+ times and check the printed MD5s agree.
5. Capture the baseline on **both** platforms (see the two sections above). A
   scenario with only one platform's baseline fails on the other.
6. Add a row to the table above, and confirm the full run is still
   `ALL IDENTICAL`.

**Turning a mode on is not the same as exercising it.** If the input that mode
reads stays zero, the feature runs and contributes nothing — which pins the code
path but nothing else. Both scenarios 7 and 27 shipped in that state for two
years. Confirm your new scenario's baseline actually moves when you drive it.

## Layout

```
test/regression/
    README.md              this file — how to run it and how to change it
    BACKGROUND.md          why it works this way; coverage; case history
    run_regression.py      CLI runner — build, run, compare, report
    scenarios.py           the scenario table and the runners that execute it
    test_regression.py     pytest wrapper: one test per scenario
    test_tuning.py         tuner still reproduces scenario_01.IN
    test_fixtures.py       each fixture is still scenario_01.IN + its patches
    compare_baselines.py   what a baseline change actually changed
    baseline_report.py     the same evidence as a reviewable report page
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
