<!-- Review + status tracking doc. Update the Status column as work lands. -->
# Plan: Turn the VIT verification scripts into a contributor-ready regression suite

## TL;DR
The 27-scenario verification already works and is fast (75 s, 5,252,000 float64 values,
27/27 identical as of 2026-09-21). What it lacks is *legibility*: it is named after a
translation tool that no longer exists, it is scattered across three directories, it
depends on an Example having been run first, and it is not in CI. This plan makes it a
documented, CI-enforced regression suite that an outside contributor can run, extend,
and trust — without changing what it actually verifies.

**Guiding constraint:** the baselines are the crown jewels. No step in this plan may
change a baseline value. Any step that *would* is called out explicitly and requires a
deliberate `--update-baseline` commit with justification.

## Current Status
P0 is committed: task 1 in `66e0e9da` (pushed), tasks 2–6 in `e08c79f1` (local, not yet
pushed). Tasks 7, 8a and 9 are **done but uncommitted** in the working tree. Remaining P1:
3b (C++ vocabulary) and 8b (needs a parameter dump).

| # | Task | Priority | Status |
|---|------|----------|--------|
| 1 | Delete dead translation scaffolding | P0 | DONE — commit `66e0e9da`, pushed; tag `archive/vit-translation` created + pushed 2026-09-21 |
| 2 | Consolidate into `test/regression/` | P0 | DONE — commit `e08c79f1` (unpushed); suite re-verified 27/27 (5,252,000 values) from the new path |
| 2b | Align build directory + CMake presets | P0 | DONE — commit `e08c79f1` (unpushed); `rosco/controller/build` everywhere, presets dropped to `"version": 1`, `default` preset removed, `--rebuild` now configures an unconfigured build dir |
| 3 | Rename VIT-era vocabulary | P0 | DONE for the harness — commit `e08c79f1` (unpushed); `vit_sim`→`scenarios`, `verify_cpp`→`run_regression`, `baseline_arrays`→`baselines`, sim names→`regression_N`. C++ source vocabulary split out as task 3b. |
| 3b | Rename VIT-era vocabulary in the C++ source | P1 | TODO — deferred deliberately; do it once the regression suite is settled, so the rename has a green baseline to verify against |
| 4 | Write `test/regression/README.md` | P0 | DONE — commit `e08c79f1` (unpushed) |
| 5 | Remove the hidden `01_turbine_model.py` dependency | P0 | DONE — commit `e08c79f1` (unpushed); pickle load replaced with `Turbine(inps['turbine_params'])`; verified 27/27 from a fresh clone with no prior steps |
| 6 | Add CI job | P0 | DONE — commit `e08c79f1` (unpushed); `pytest -v test/regression` step in `build_and_test_conda`, ubuntu only. Not yet exercised on a real CI runner. |
| 7 | Commit DISCON fixtures | P1 | DONE (uncommitted) — `fixtures/scenario_01..28.IN`; no separate base file (scenario 1 is unpatched, so its fixture *is* the tuner output); `patches=` kept as the regeneration recipe behind `--write-fixtures`; regeneration is idempotent; suite still 27/27 with the tuner out of the loop |
| 8a | Tuning test: YAML → DISCON text | P1 | DONE (uncommitted) — `test_tuning.py`, 3 s, no DLL; pins `scenario_01.IN`; verified it fails with a readable per-parameter diff |
| 8c | Assert fixtures still equal scenario_01 + patches | P2 | TODO — needs task 13's patches table; see 8c |
| 8b | Input-parsing test: DISCON → parsed parameters | P1 | TODO — blocked on a parameter dump, *not* on the whole input-modernization plan. `Echo` is parsed but never implemented (no writer exists); a generated `dump_to_toml()` in `rosco_types_io.cpp` unblocks it. See that plan's Further Considerations #0. |
| 9 | Baseline provenance metadata | P1 | DONE (uncommitted) — `baselines/PROVENANCE.json`, written by `--update-baseline`, printed in every run header; initial file backfilled honestly from git rather than fabricated |
| 10 | C++ line/branch coverage (gcovr) | P2 | TODO |
| 11 | Mode-coverage table: regression vs Examples | P2 | TODO |
| 12 | HDF5 scenario symmetry | P2 | TODO |
| 13 | Data-driven scenario definitions | P3 | TODO |
| 14 | Store `t`/`ws` in baselines, delete `SCENARIO_WIND` | P3 | TODO |
| 15 | Compress baselines | P3 | TODO |

---

## Findings that drive the plan

Measured/verified on 2026-09-21, **before** the P0 work landed. Findings are kept as the
original diagnosis; some file links below point at paths that tasks 2-5 have since moved.

1. **It works and it is cheap.** Full suite = 75 s wall clock; scenario 1 alone = 6.4 s.
   No need for tiering, nightly runs, or a "fast subset" — the whole thing fits in every PR.
2. **It is not in CI.** `.github/workflows/CI_rosco-compile.yml` runs `rosco/test` pytest
   and the Examples, but never `verify_cpp.py`. The strongest test in the repo is manual.
3. **It has a hidden prerequisite.** `load_turbine_and_controller()` in
   [vit_sim.py:158](/Users/dzalkind/Tools/ROSCO-C/Examples/vit_sim.py:158) loads
   `examples_out/01_NREL5MW_saved.p`, which only exists after someone runs
   `Examples/01_turbine_model.py`. That file is gitignored. A fresh clone cannot run the
   regression suite. This is the single biggest handoff blocker.
4. **The baselines cover the toolbox, not just the controller.** Every scenario calls
   `write_discon(turbine, controller, ...)`, regenerating its `DISCON_*.IN` from
   `Tune_Cases/NREL5MW.yaml` through the Python tuner at run time. A change in
   `tune_controller()` — or a wisdem/scipy upgrade — moves the controller's *inputs* and
   fails the regression with a message that points at the C++ code. Cause and symptom are
   not distinguishable today.
5. **Vocabulary is archaeological.** `vit_sim`, `verify_cpp`, `baseline_arrays`, `kernel/`,
   `upstream_arrays/`, `scripts/*_all.sh` all refer to the VIT/KGen Fortran→C++ translation
   workflow, which is finished and whose Fortran source has been deleted. A new contributor
   has to reverse-engineer what "VIT" means before they can read the test.
6. **887 MB of dead scaffolding is tracked.** `kernel/` (1,929 files), `upstream_arrays/`
   (40 MB, 26 files), `translations/` (57 files), and the Docker-era
   `scripts/{extract,integrate,verify}_all.sh`. Plus ~10 stray `*.RO.dbg` files loose in the
   repo root.
7. **Scenario 28 is asymmetric.** It exercises HDF5 output, has no frozen baseline, is
   excluded from `ALL_SCENARIOS`, and its `--hdf5` comparison writes into `Examples/` and
   requires scenario 1 to have been run first in the same working tree.
8. **`SCENARIO_WIND` in [plot_verification.py:31](/Users/dzalkind/Tools/ROSCO-C/scripts/plot_verification.py:31)
   re-declares each scenario's wind profile** because the `.npz` files do not store `t` or
   `ws`. Two sources of truth for the same thing, already drifting-prone.
9. **The DISCON files are mutated in place and shared between scenarios.** 29
   `write_discon()` calls, 28 of them with `patches=`, but only **15 distinct filenames** —
   `DISCON.IN` is written 8 separate times with different patches. Whatever is on disk
   afterwards reflects only the last scenario that ran. This is invisible today because
   every scenario rewrites its file immediately before use, but it means the files on disk
   after a run are *not* a record of what the suite tested. (Task 2 moved those writes into
   a scratch temp directory, so they no longer land in `Examples/` — but the 15-names-for-29-
   parameter-sets problem is unchanged and is still task 7's to solve.)
10. **Three build directories, and the docs point at the wrong one.**
    `.github/copilot-instructions.md` says `cd build && cmake ../rosco/controller` (repo-root
    `build/`); [verify_cpp.py:29](/Users/dzalkind/Tools/ROSCO-C/scripts/verify_cpp.py:29)
    defaults to `rosco/controller/build`; the `default` CMake preset resolves to the same
    `rosco/controller/build`. All three exist on the maintainer's machine, so the conflict is
    invisible locally. A newcomer follows the instructions, builds at the repo root, runs
    `verify_cpp.py --rebuild`, and gets `cmake --build` against an unconfigured directory.
11. **Floating-point reproducibility is already handled — do not regress it.**
    [CMakeLists.txt:16](/Users/dzalkind/Tools/ROSCO-C/rosco/controller/CMakeLists.txt:16)
    sets `-ffp-contract=off` (and `/fp:precise` on MSVC), and forces `RelWithDebInfo` when no
    build type is given. This is *why* bit-identical baselines survive across compilers and
    optimisation levels. Any new build preset (ASan, coverage) must keep those flags, and the
    README should say so, because it is not obvious and silently breaks the whole suite.

---

## Design decisions

- **One top-level `test/` directory, separate from `Examples/`.** Confirmed with Daniel
  2026-09-21. The suite is not an example, and the Examples are not regression tests.
  Keeping them separate lets the Examples stay readable teaching code while the regression
  stays strict.
- **Baselines stay *outside* the installed package.** This is the reason for top-level
  `test/` rather than folding into `rosco/test/` — 40 MB of `.npz` must not end up in the
  wheel. `rosco/test/` remains toolbox tests and is left alone.
- **Commit the DISCON fixtures (task 7).** This is the one structural change worth making.
  It splits today's single conflated test into two honest ones:
  - *Controller regression*: committed `DISCON_*.IN` → simulation arrays. Deterministic
    C++ behaviour, no tuner in the loop.
  - *Tuning regression*: `NREL5MW.yaml` → generated `DISCON.IN`, compared as text against
    the committed fixture. Fast, no DLL, and it fails with an obvious diff when the tuner
    changes.
  The same 27 files serve both. This is what "regression testing the DISCON files too"
  should mean.
- **Everything runs on every PR.** 75 s does not justify a tiering scheme.
- **Fixtures are text and reviewable.** A DISCON diff in a PR is a readable signal about
  what a change did to the controller's inputs.

---

## P0 — Handoff blockers

These are the ones that decide whether someone else can contribute. All are mechanical.

### 1. Delete dead translation scaffolding — **DONE** (`66e0e9da`)
Confirmed with Daniel 2026-09-21: `kernel/`, `upstream_arrays/`, and `translations/` are
VIT/KGen translation-era artifacts, untouched since the Fortran→C++ port completed, and a
newcomer should never open them. Same for the Docker-era
`scripts/{extract_all,integrate_all,verify_all,debug_test,prepare_commit,reset_to_clean,reset_to_upstream}.sh`,
and the stray `*.RO.dbg` / `sim*.RO.dbg` files loose in the repo root.

Add `*.RO.dbg`, `*.RO.dbg2`, `*.RO.dbg3`, `*.RO.h5` to `.gitignore` — the existing `*.dbg`
rules miss the `.RO.dbg2/3` variants, which is why the root accumulated them.

**What this actually buys:** a fresh `git clone` checks out only HEAD, so the working tree
drops by ~930 MB. The download does **not** shrink — `.git` stays at 113 MB, because
deleting a file in a new commit never removes it from history. That is fine, and it is the
whole point of the archive tag below.

#### Archive tag process
```bash
# 1. Tag the current state, before deleting anything. Annotated, not lightweight,
#    so it carries a message explaining what it is.
git tag -a archive/vit-translation -m \
  "Final state of the VIT/KGen Fortran->C++ translation scaffolding.
   kernel/ (golden KGen fixtures), upstream_arrays/, translations/, and the
   Docker-era scripts/*.sh, as used to produce the pure-C++ controller.
   Removed from the working tree after this point; recover with:
     git checkout archive/vit-translation -- kernel/"

# 2. Push the tag so it is not just local.
git push origin archive/vit-translation

# 3. Delete in a normal commit on the branch.
git rm -r --cached kernel upstream_arrays translations   # plus the .sh files
# ...remove from disk, commit.
```

Three things worth being clear about before doing it:

- **The tag is a bookmark, not a backup.** The commits and blobs are already permanent in
  history; the tag just gives them a findable name so nobody has to `git log` spelunk for
  "the commit before the big delete." Recovery is
  `git checkout archive/vit-translation -- kernel/` at any point in the future.
- **You cannot both keep the tag and shrink clone size.** Making the repo smaller to
  download means rewriting history (`git filter-repo`), which requires a force-push and
  every collaborator re-cloning — and the archive tag would *block* the pruning by keeping
  those objects reachable. Pick one. Recommendation: keep the tag, accept 113 MB. It is not
  a large repo by modern standards, and the newcomer-confusion problem is solved by the
  working-tree deletion alone.
- **Record the tag name in a durable place.** A line in `REFACTOR_NOTES.md` under the
  translation section — otherwise the tag is only discoverable by someone who already
  knows to run `git tag`.

*Check:* `python scripts/verify_cpp.py` still 27/27 after deletion. (Verified 2026-09-21:
the suite does not reference any of these paths.)

### 2. Consolidate into `test/regression/` — **DONE**

Landed layout (no `conftest.py`: pytest's own rootdir `sys.path` insertion is enough, and
there were no shared fixtures to put in one; `fixtures/` and `test_tuning.py` arrive with
task 7/8a):

```
test/
    regression/
        README.md                 # task 4
        conftest.py               # pytest discovery + shared fixtures
        test_regression.py        # pytest wrapper -> one test per scenario
        test_tuning.py            # task 8a — layer A
        run_regression.py         # today's verify_cpp.py (CLI entry point)
        scenarios.py              # was Examples/vit_sim.py
        plot_regression.py        # was scripts/plot_verification.py
        fixtures/                 # task 7 — base_DISCON.IN + scenario_NN.IN
        baselines/                # was baseline_arrays/*.npz
```
Top level, *not* under `rosco/`, so the 40 MB of baselines stays out of the wheel.
`rosco/test/` keeps the toolbox tests and is untouched.

Keep the CLI (`python test/regression/run_regression.py --scenario 3`) — it is how you
actually debug a failure — and add the pytest wrapper so `pytest test/` finds it and CI
reports per-scenario pass/fail instead of one opaque job.

### 2b. Align build directory + CMake presets — **DONE**
Findings 10 and 11. `rosco/controller/CMakePresets.json` is now tracked (it was untracked,
which quietly made `verify_cpp.py --preset asan` a maintainer-only feature). Remaining work:

- **Pick one build directory and make all three sources agree** — the runner's
  `DEFAULT_BUILD_DIR`, the `default` preset's `binaryDir`, and
  `.github/copilot-instructions.md` / `README.md`. Recommend `rosco/controller/build`, since
  two of the three already use it and presets resolve relative to `sourceDir`.
- **Fix the preset schema version.** `"version": 3` requires CMake ≥ 3.21, but
  `cmake_minimum_required(VERSION 3.14)`. Either drop the presets file to `"version": 1`
  (CMake 3.19) or raise the stated minimum. Today a user on 3.14–3.20 gets an opaque error
  from `--preset` rather than a clear version message.
- **Drop the redundant `default` preset**, or keep it only as documentation —
  `CMakeLists.txt` already forces `RelWithDebInfo` when `CMAKE_BUILD_TYPE` is unset.
- **Document the FP flags** (finding 11) in the regression README, and add a line to the
  "adding a preset" guidance: presets must not override `-ffp-contract=off` or add
  `-ffast-math`, or baselines will stop reproducing. The `coverage` preset from task 10 is
  the first one that will need to honour this.

### 3. Rename VIT-era vocabulary — **DONE**
`vit_sim` → `scenarios`, `verify_cpp` → `run_regression`, `baseline_arrays` → `baselines`,
`vit_simN.RO.dbg` sim names → `regression_N.RO.dbg`. Drop "VIT"/"KGen" from all docstrings,
or define them once in the README as historical context. Scenario *numbers* stay as they
are — they are referenced in `REFACTOR_NOTES.md` and in commit history.

*Careful:* the `sim_name` passed to `ControllerInterface` determines the `.RO.dbg` filename,
which `compare_hdf5_debug()` hardcodes. Rename both together. (Done — renamed together.)

The C++ source keeps its VIT-era filenames — split out as task 3b below.

### 3b. Rename VIT-era vocabulary in the C++ source
Deferred from task 3 by decision 2026-09-21: **do this after the regression suite is set up**,
not as part of it. The suite is what makes the rename safe, so it should exist and be green
first.

`rosco/controller/src/include/vit_types.h` and `vit_translated.h` are live headers. 49 files
reference them, including `write_registry.py` (it emits `#include "vit_types.h"` at lines 128
and 393), so the generator has to change with them.

Mechanical, but not zero-risk, and it wants its own commit:
- rename the two headers, update every `#include`, update both `write_registry.py` emit sites;
- regenerate the registry (`cd rosco/controller/rosco_registry && python write_registry.py`);
- `python test/regression/run_regression.py --rebuild` must still be 27/27 identical.

Worth doing: after task 3, these headers are the last place a newcomer meets "VIT" with no
explanation.

### 4. Write `test/regression/README.md` — **DONE**
The handoff document. Must cover:
- What the suite asserts (bit-identical float64 arrays, not tolerances) and why.
- How to run: full, single scenario, with rebuild, with plots.
- How to read a failure: `max_diff` / `first_diff_idx`, then `plot_regression.py` to see it.
- **When it is legitimate to update a baseline**, and the required PR discipline: separate
  commit, justification in the message, diff plots attached.
- The determinism machinery, which is currently tribal knowledge:
  - each scenario runs in a subprocess because the DLL has static state;
  - `libscrub.so` exists to work around a scipy FITPACK uninitialised-stack read that makes
    scenario 3 non-deterministic (dev note 202603261512);
  - HDF5 is optional and the suite degrades rather than fails without it.
- What each scenario exercises — one line each, generated from task 11's table.
- How to add a scenario.

### 5. Remove the hidden `01_turbine_model.py` dependency — **DONE**
Resolved by constructing the turbine from the YAML's `turbine_params` instead of loading the
pickle — exactly what `01_turbine_model.py` does before saving it. `load_from_fast()` then
repopulates everything else, so the pickle contributed nothing that is not re-derivable.

Finding 3. Either commit the saved turbine pickle as a fixture (fast, but a binary blob that
silently ages), or — preferred — drop the `turbine.load(...)` line entirely, since
`load_from_fast()` on the next line repopulates the object from `Test_Cases` + the Cp text
file anyway. Verify which fields the pickle actually contributes before deleting.

*Check:* `git clean -xdf && python test/regression/run_regression.py` from a fresh clone
must pass with no prior steps.

### 6. Add CI job — **DONE**
Added as a step in the existing `build_and_test_conda` job rather than a new job: that job
already installs the conda env, builds the DLL via `pip install -e .`, and runs ubuntu-only
steps, so a separate job would have duplicated all of it. Gate the HDF5 comparison on `h5py` + libhdf5 being
present, skipping rather than failing when absent. Runtime budget ~3 min including build.

---

## P1 — Make the harness honest

### 7. Commit DISCON fixtures — **DONE** (uncommitted)

**Terminology note:** this plan's "layer A/B/C" labels are internal to the plan. They are
deliberately *not* used in `test/regression/README.md` or in any shipped docstring, where the
steps are named plainly (tuning / input parsing / control) for readers who have not read this
document.

**Three blockers the original plan missed, all found by inspecting a generated file.** The
tuner's output is not portable: it contains absolute paths (`PerfFileName`, `OL_Filename`)
and a `version + today's date` stamp on line 2. Committing it verbatim would have broken
every other machine and made task 8a fail daily.

Resolved by:
- storing path parameters **relative to the fixture file**, which works because the
  controller resolves a relative value against the DISCON file's own directory
  (`priPath`, set from `fp.parent_path()` in `readconfigfiles.cpp:35`). Verified: no
  absolute paths remain in any fixture;
- normalising away the `version + date` stamp on line 2 entirely, rather than merely
  excluding it from the comparison. Left in the committed file, every `--write-fixtures`
  run would dirty all 28 fixtures and a real tuner change would be invisible in the noise.
  Git already records when each fixture changed. Regeneration is now **idempotent** —
  verified by regenerating twice and getting a zero-line `git diff`, which is what makes a
  non-empty fixture diff trustworthy as a signal.

**A third finding, from review:** `base_DISCON.IN` as specified would have been
byte-identical to `scenario_01.IN` (scenario 1 is the only unpatched scenario), with nothing
asserting the two stayed equal — a duplicated artifact free to drift. Dropped it; the tuning
test pins `scenario_01.IN` directly, so the file it guards is the file the controller runs
on.

**Deviation from the plan, deliberate:** the plan said the `patches=` mechanism "can be
deleted outright" once fixtures are committed. It is kept, as the argument to
`discon_fixture(N, patches={...})`. Deleting it would make the fixtures regenerable only by
hand-editing 28 files; keeping it means `--write-fixtures` can rebuild them all from the
YAML, and the dict still documents in-source what the scenario changes. It is inert during
a normal run.

**The question the plan flagged — whether in-place mutation had been masking a bug — is
answered: no.** Materialising 28 separate fixtures left all 27 baselines byte-identical.

*Original plan text follows.*

Move the generated `DISCON_*.IN` into `test/regression/fixtures/` and commit them.
`Examples/DISCON*.IN` stays gitignored — the fixtures live under `test/`, so the existing
ignore rule does not fight them.

*Updated after task 2:* the suite now writes its generated DISCON files into a scratch temp
directory, not `Examples/`, so the run no longer mutates the working tree at all. The four
tracked `Examples/DISCON_{awc,filters,flp,ipc}.IN` files — force-added against `.gitignore`,
referenced by nothing — have been **deleted** (2026-09-21); they were each one arbitrary
snapshot, since `DISCON_awc.IN` is written by 5 scenarios and `DISCON_{ipc,flp}.IN` by 2
each. Task 7 therefore starts from a clean slate: generate fresh, one file per distinct
parameter set.

Note the ignore rule is scoped `Examples/DISCON*.IN`, so `test/regression/fixtures/*.IN`
needs no force-add and no `.gitignore` change — relocation alone removes the trap.

**Scope is larger than it looks: the fixture files are currently mutated in place.** There
are 29 `write_discon()` calls across 28 scenarios but only **15 distinct filenames** —
`DISCON.IN` alone is written 8 times with *different* `patches=` each time. On disk, the
file's contents depend on which scenario ran last, so the 15 files cannot simply be lifted
and committed. Task 7 must first give every distinct parameter set its own name:

```
fixtures/
    base_DISCON.IN          # unpatched tuner output — the layer-A reference (8a)
    scenario_01.IN
    scenario_02.IN
    ...                     # one per scenario, = base + that scenario's patches
```

Scenarios then read their own fixture instead of calling `write_discon(...)`, and the
`patches=` regex-substitution mechanism is deleted — a patched parameter becomes a visible
line in a committed file.

*This is the one task that can change baseline values*, if the materialised fixtures differ
at all from what the baselines were captured with. Sequence carefully: generate fixtures,
run the suite, confirm 27/27 *before* committing. If it does not come out identical, that is
a finding worth chasing, not a baseline to update. The in-place mutation above is exactly
the kind of thing that could have left a scenario running against the wrong file all along.

### 8. Localise the regression: test each layer separately

There are three layers between the tuning YAML and a baseline array, and today a single
test covers all three at once. When it fails, it reports a float mismatch at
`first_diff_idx` regardless of which layer actually broke:

| Layer | Transformation | Owned by | Tested today |
|-------|----------------|----------|--------------|
| **A** | `NREL5MW.yaml` → `DISCON_*.IN` | Python tuner (`tune_controller`, `write_DISCON`) | only implicitly |
| **B** | `DISCON_*.IN` → `ControlParameters` struct | generated parser (`write_registry.py` → `rosco_types_io.cpp`, `readcontrolparameterfilesub.cpp`) | **not at all** |
| **C** | `ControlParameters` + plant → time series | C++ control algorithms | yes — this is the suite |

Committing the fixtures (task 7) pins the boundary between A and B. Each side then gets its
own cheap check, and a failure names its own layer.

#### 8a. Layer A — tuning regression — **DONE** (uncommitted)

**Scope is one file, not 27.** All 29 `write_discon()` calls in `scenarios.py` produce the
*same* tuner output — `write_DISCON(turbine, controller, ...)` is called identically every
time, and the scenario-specific differences are applied *afterwards* by regex text
substitution in the `patches=` argument
([scenarios.py:159](/Users/dzalkind/Tools/ROSCO-C/test/regression/scenarios.py:159)). So layer A has
exactly one output to pin.

**`test/regression/test_tuning.py`**

```
load NREL5MW.yaml -> tune_controller() -> write_DISCON() -> tmp file
diff against test/regression/fixtures/base_DISCON.IN
```

- No DLL, no simulation. Runs in seconds; goes in the same CI job.
- Fails with a readable text diff naming the parameters that moved, rather than a float
  mismatch at `first_diff_idx` in a time series.
- **Failure semantics, to state in the README:** a failure here means the *tuner* changed.
  That is sometimes intentional (a tuning improvement) and sometimes accidental (a wisdem
  or scipy upgrade). Either way, it is a separate decision from a controller regression,
  and updating `base_DISCON.IN` is a deliberate reviewable commit.
- **Fallback if it proves flaky:** float formatting in the text output may vary across
  platforms or library versions. If so, parse both files with `read_DISCON()` and compare
  numerically with a tight tolerance. Do not delete the test — the failure mode it catches
  is exactly the one that currently has no owner.

**The relationship to the scenario fixtures:** each of the 27 remaining scenario fixtures is
`scenario_01.IN` + that scenario's patches, frozen. Their diff against the base *is* the
scenario definition, in reviewable text. Once committed, the `patches=` mechanism and its
regex substitution can be deleted outright.

#### 8b. Layer B — input-parsing regression
**This is the layer that has already bitten us.** Follow-up #4 in
`plan-outputFileModernization.prompt.md` was exactly a layer-B bug: `_write_cpp_io()`
hardcoded `value_or(0)` and ignored the registry's per-field `equals:` defaults, so the
controller silently ran with the wrong `OutputFormat`. Nothing in the suite would have
caught it — the 27 fixtures all set the parameter explicitly, so layer C never saw the
difference. Any future registry or parser change can do the same thing to any parameter
that a fixture happens not to set.

The check needs one thing that does not exist yet: a way to dump what the controller
*actually parsed*. Note that `Echo` is already declared in `rosco_types.hpp`, parsed in
`readcontrolparameterfilesub.cpp`, and documented in every `DISCON.IN` as "Echo input data
to `<RootName>.echo`" — **but nothing in the C++ source writes that file.** The feature is
currently a lie inherited from the Fortran.

Implementing it is the laziest path because `write_registry.py` already generates a TOML
*reader* from `rosco_types.yaml`; the *writer* is the mirror image of the same generator
code. Doing it once yields three things:

1. `Echo = 1` starts doing what it advertises;
2. the layer-B regression fixture — echo the parsed parameters, diff against a committed
   snapshot, which catches default-value and parser regressions for *every* registry field,
   including ones no fixture sets;
3. Phase 4 of `plan-inputFileModernization.prompt.md` — the `DISCON.IN` → TOML migration
   utility — is very nearly the same function, since echoing a parsed `.IN` file as TOML
   *is* the conversion.

**Recommendation:** do not block the regression harness on this. Land 8a with task 7, and
schedule 8b as the opening move of the input-modernization plan, where the TOML writer is
already on the roadmap. Flagged here so the convergence is not missed.

#### 8c. Remaining gap: fixtures vs. their patches *(new, TODO)*
Nothing asserts that `scenario_NN.IN` still equals `scenario_01.IN` + that scenario's
`patches=`. A hand-edit that changes behaviour is caught by the baselines; one that does not
— touching a parameter inert under that scenario's modes — would persist silently. Closing it
properly needs the `patches=` dicts hoisted out of the 28 function bodies into a table, which
is task 13's refactor. Mitigations in place meanwhile: `--write-fixtures` regenerates all 28
atomically, regeneration is idempotent so `git diff fixtures/` is a clean signal, and the
README says not to hand-edit.

### 9. Baseline provenance metadata — **DONE** (uncommitted)
The initial `PROVENANCE.json` was **backfilled from git, not fabricated**: the baselines
predate the mechanism, so claiming they were "generated now" would have been a lie. It
records the last commit to change baseline *content* (`e491c935`, 2026-04-03 — the same
commit `REFACTOR_NOTES.md` cites for the Fortran chain of custody) and marks the environment
fields `unrecorded`, since they were never captured. The next `--update-baseline` fills them
in for real.

Make `--update-baseline` write `baselines/PROVENANCE.json`: git SHA, date, platform,
compiler, numpy/scipy versions, `libdiscon` hash. Today the only way to answer "when were
these last regenerated and against what?" is `git log` archaeology. Print it in the runner's
header so every run states what it is comparing against.

---

## P2 — Coverage

### 10. C++ line/branch coverage
Add a CMake option (`ROSCO_COVERAGE=ON` → `--coverage`), run the suite, report with
`gcovr`. One dependency, no custom tooling. Output: which controller branches 27 scenarios
never reach. Run it in CI as a non-gating informational job first; only consider a threshold
once the baseline number is known.

### 11. Mode-coverage table: regression vs Examples
The question "what do the regression and the Examples each cover, and where do they
overlap?" is answerable directly from the DISCON files once task 7 lands: every ROSCO
feature is gated by a `*_Mode` / `*_ControlMode` parameter. A ~40-line script tabulates
those across `test/regression/fixtures/*.IN` and `Examples/**/DISCON*.IN` and emits a
matrix: mode value × covered-by. That gives:
- which modes nothing tests,
- which Examples are redundant with the regression,
- which Examples cover something the regression does not (→ candidates for promotion to
  scenarios).

Pair it with task 10: mode coverage says *what is configured*, gcov says *what executed*.
Neither alone is the answer.

---

## P3 — Tidy, once the above is stable

### 12. HDF5 scenario symmetry
Give scenario 28 a frozen baseline like the rest, and make `compare_hdf5_debug()` work
inside the temp dir instead of requiring a prior scenario-1 run in `Examples/`.

### 13. Data-driven scenario definitions
`scenarios.py` is 2,028 lines of 28 near-identical hand-written functions. Most differ only
in fixture, wind profile, and which synthetic avrSWAP inputs get set. Collapse to a table +
one runner. Defer until after task 7 — committed fixtures remove much of the per-scenario
code by themselves, and the remainder will be easier to see.

### 14. Store `t`/`ws` in the baselines
Kills the duplicated `SCENARIO_WIND` table (finding 8). *Changes baseline file contents*
(adds keys); sequence after everything else and land as its own commit.

### 15. Compress baselines
`np.savez_compressed` on 40 MB of smooth time series should cut it several-fold. Values are
unchanged, so the comparison is unaffected — but it rewrites all 27 files, so do it once,
alone, at the end.

---

## Recommended sequence

1. **Task 1 alone, first** ("remove translation-era scaffolding"). Tag, delete, push. It
   touches nothing the regression suite uses, it is the thing a newcomer trips over first,
   and keeping it out of the reorganisation PR keeps that PR's diff readable.
2. **Tasks 2–6 as one PR** ("make the regression suite runnable and CI-enforced"). Nothing
   about *what* is verified changes; it is renames, moves, docs, and a workflow entry.
3. **Task 3b** once the harness is settled — the C++ header rename, verified against the
   now-green suite. Small and self-contained; slot it wherever convenient after the P0 PR.
4. **Tasks 7, 8a, 9 as a third PR** ("decouple controller regression from the tuner"). The
   one with real design content and the only one that touches baselines.
5. **Task 8b** once a parameter dump exists. It does *not* have to wait for the
   input-modernization plan to land — it needs one generated `dump_to_toml()`, which that
   plan wants anyway (its Further Considerations #0). Whoever gets there first unblocks it.
6. **Tasks 10–11** — coverage is a reporting layer; it should sit on top of a stable
   harness, not be built into a moving one.
7. **P3 opportunistically.**

## Open questions for review
- ~~Top-level `test/` vs folding into the existing `rosco/test/`?~~ **Decided 2026-09-21:**
  top-level `test/`, so baselines stay out of the wheel.
- ~~Archive `kernel/` before deleting?~~ **Decided 2026-09-21:** annotated tag
  `archive/vit-translation` — created. Deletion still pending.
- Should the Examples themselves become regression-tested (output comparison), or stay as
  smoke tests (`test_examples.py` only asserts they do not raise)? Task 11 will show whether
  they cover anything unique enough to be worth it.
