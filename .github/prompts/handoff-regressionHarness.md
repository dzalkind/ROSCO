# Handoff: finish the regression suite, then modernize inputs

You are picking up two threads of work in this repo (branch `c++`):

1. **Finishing the regression suite** —
   [`plan-regressionHarness.prompt.md`](plan-regressionHarness.prompt.md)
2. **Input file modernization** —
   [`plan-inputFileModernization.prompt.md`](plan-inputFileModernization.prompt.md)

Those two plans are authoritative. Read both before starting, and **update their
status tables as you land work** — they are the shared record.

[`plan-outputFileModernization.prompt.md`](plan-outputFileModernization.prompt.md)
is complete; it is context only, not work.

## Not your scope: CI

**Daniel is actively working CI in a parallel session.** Do not edit
`.github/workflows/`, and do not chase CI failures, compiler/linker flags, or
packaging errors — you will collide with him. If something you need is blocked on
CI, say so and move to other work rather than fixing it yourself.

The one exception is your own conda environment (below) — that is local, not CI.

---

## Use the `rosco-c` conda environment

```bash
conda activate rosco-c
```

This is not optional and it is the first thing to get wrong. The repo's
`Examples/Test_Cases/` moved to **OpenFAST v5** input format in `c90e217a`
(2026-09-21), so the suite needs `openfast_io` 5.x. Several environments on this
machine still have 4.x — including `base`. With an old one, every test dies
inside `turbine.load_from_fast()`, before the controller is ever loaded:

```
ValueError: invalid literal for int() with base 10: '1E+06'
  openfast_io/FAST_reader.py  (reading UJacSclFact as an int)
```

The v4 reader does not know about the v5 `.fst`'s `RhoInf`, `ConvTol`,
`MaxConvIter` and `NRotors`, so it walks off by several lines. Check before you
debug anything:

```bash
python -c "import importlib.metadata as m; print(m.version('openfast_io'))"   # want 5.x
```

**If you see that error, it is your environment — never "fix" it by touching
baselines, fixtures, or scenarios.**

Verified 2026-09-21 in `rosco-c` (openfast_io 5.0.0): **29/29 pass, all 27
baselines byte-identical**. So the OpenFAST v5 migration changed neither the
plant model nor the tuner output.

---

## The hard gate

Any change you make must leave this exactly as-is:

```bash
conda activate rosco-c                       # 4.x openfast_io fails before the controller loads
python test/regression/run_regression.py     # ALL IDENTICAL — 5,252,000 total float64 values
pytest test/regression                       # 29 passed  (27 scenarios + HDF5 + tuning)
```

5,252,000 float64 values compared **bit-for-bit, not to a tolerance**. The
baselines are the crown jewels — they trace back to the original Fortran
behaviour (see `REFACTOR_NOTES.md`). If a baseline value moves, you have broken
something. **Never run `--update-baseline` to make a failure go away.**

Also confirm you left no mess: `git status` clean, and a regression run writes
only into a temp dir.

---

## Where things stand

| | |
|---|---|
| Suite | `test/regression/` — runner, 28 scenarios, 28 committed DISCON fixtures, 27 baselines |
| Committed | P0 (`66e0e9da`, `e08c79f1`) and P1 tasks 7/8a/9 (`4523559c`) |
| Remaining | 8b (needs a parameter dump — see below), 8c, P2 (10, 11, 12), P3 (13, 14, 15, 3b last) |
| CI | A `pytest -v test/regression` step exists in `CI_rosco-compile.yml` (ubuntu only) but **has never been observed passing**. **Daniel owns this — do not touch it.** Nothing here is proven on Linux yet |

### What the suite now separates

Three things happen between the tuning YAML and a baseline array. They are
tested separately so a failure names its own cause:

| Step | Transformation | Test |
|---|---|---|
| Tuning | `NREL5MW.yaml` → `scenario_01.IN` | `test_tuning.py` |
| Input parsing | `DISCON.IN` → `ControlParameters` | **none yet — task 8b** |
| Control | `ControlParameters` + plant → time series | `test_regression.py` |

Scenarios read **committed fixtures**, so the Python tuner does not run during a
regression. A wisdem/scipy change can no longer fail a controller test.

> Write "tuning / input parsing / control" in anything a contributor reads. The
> plan's internal "layer A/B/C" labels must not leak into the README, docstrings,
> or code comments.

---

## Recommended next step

**Owning both plans changes the answer.** Earlier advice was to sequence task 8b
carefully because it collides with input modernization in `write_registry.py`.
That collision is now an opportunity: one change serves both plans, so do it
once, deliberately, and early.

### Start with a generated parameter dump

Regression task 8b wants to assert *input parsing*: "did the controller read what
the file actually said?" It is the one step with no test and the only one with a
**proven** past bug — a generated parser ignored the registry's per-field
defaults and the controller silently ran with the wrong `OutputFormat`. Nothing
in the 27 scenarios caught it, because every fixture set that parameter
explicitly.

It needs one missing capability: the controller cannot report what it parsed.
`Echo` is declared in `rosco_types.hpp`, parsed in
`readcontrolparameterfilesub.cpp`, and documented in every `DISCON.IN` as
"Echo input data to `<RootName>.echo`" — but **nothing in the C++ source writes
that file.** The feature is a lie inherited from the Fortran. Verify this yourself
before designing around it.

`write_registry.py` already generates a TOML *reader* (`load_from_toml`) into
`rosco_types_io.cpp`. The writer is the mirror image of the same generator code.
Doing it once yields three things:

1. `Echo = 1` starts doing what it advertises;
2. regression task 8b gets its assertion — dump the parsed parameters, diff
   against a committed snapshot, which catches default-value and parser
   regressions for *every* registry field, including ones no fixture sets;
3. the input plan's Phase 4 migration utility is very nearly the same function,
   since echoing a parsed `.IN` file as TOML *is* the conversion.

This is the natural bridge between your two plans. Both status tables should
reflect it when it lands.

### Cheap parallel win: task 11

If you want a low-risk warm-up, **task 11 (mode-coverage table)** got much
cheaper once fixtures landed. Every ROSCO feature is gated by a `*_Mode` /
`*_ControlMode` parameter, so a ~40-line script can tabulate those across
`test/regression/fixtures/*.IN` and `Examples/**/DISCON*.IN` and emit a
mode × covered-by matrix. No C++ changes, no baseline risk. It answers "what does
the suite *not* cover?" and validates the hand-written per-scenario table in the
regression README.

---

## Input modernization: what you are walking into

Read that plan's "Current State" section — it is accurate. The parts that matter
most for sequencing:

- C++ `load_from_toml()` already works and is generated from `rosco_types.yaml`.
  There is **no writer** on either side (no C++ dump, no Python TOML writer).
- **Two divergent schema sources**: `rosco_types.yaml` (C++ codegen) and
  `toolbox_schema.yaml` (Python toolbox). Phase 1 makes the former canonical.
- The Cp/Ct/Cq text parser (`ReadCpFile`) uses hardcoded line counting; Phase 2
  replaces it with keyword scanning. Independent of everything else — a good
  parallel task.
- Phase 5 step 13 says "update the scenarios to TOML inputs". Note what that
  means now: scenarios read committed fixtures, so it means converting
  `test/regression/fixtures/*.IN` to TOML and pointing the scenarios at them.
  **All 27 baselines must stay byte-identical** — controller behaviour must not
  depend on which format configured it. That equality is the strongest claim
  that plan can make, so treat it as the acceptance test, not an afterthought.

## Traps

1. **Auto-generated files.** `debug.cpp`, `rosco_types.hpp`, `rosco_types_io.cpp`,
   `DISCON_template.toml` are generated from `rosco_types.yaml`. Edit the YAML and
   run `cd rosco/controller/rosco_registry && python write_registry.py`.
2. **`-ffp-contract=off` / `/fp:precise`** in `rosco/controller/CMakeLists.txt` is
   why bit-identical baselines survive across compilers. No CMake preset may
   override it or add `-ffast-math`.
3. **One build directory: `rosco/controller/build`.** The runner, the presets and
   the docs all agree; keep it that way. `--rebuild` configures it if needed.
4. **Fixtures must stay portable.** `PerfFileName` / `OL_Filename` are stored
   *relative to the fixture file* (the controller resolves them against the DISCON
   file's own directory). The version/date stamp on line 2 is normalised away so
   regeneration is idempotent. Never hand-edit a fixture — change `patches=` in
   `scenarios.py` and run
   `python test/regression/scenarios.py --write-fixtures`. A non-empty
   `git diff test/regression/fixtures/` then means something real moved.
5. **`test_tuning.py` is an exact text comparison** of tuner output, so it is the
   most likely thing to fail on a different platform or library version — and it
   is the most likely way your work shows up as a CI failure in Daniel's session.
   The plan names the fallback: parse both with `read_DISCON()` and compare
   numerically with a tight tolerance. Do not delete the test — the failure it
   catches has no other owner. If CI reports it failing on Linux, that is a real
   signal about float formatting, not a reason to weaken the check.
6. **Scenario numbers are permanent.** They name baseline files and are cited in
   `REFACTOR_NOTES.md` and commit history. Never renumber.
7. **Each scenario runs in its own subprocess** because the DLL holds static
   state, and `libscrub.so` exists to make scenario 3 deterministic (a scipy
   FITPACK uninitialised-stack read). Both must survive any refactor.
8. **Known gap, task 8c:** nothing asserts `scenario_NN.IN` still equals
   `scenario_01.IN` + that scenario's `patches=`. A hand-edit that changes
   behaviour is caught by the baselines; one that is inert under that scenario's
   modes is not.

## Conventions

- No tool- or agent-branded markers in code, comments, or commit messages.
- Commit subjects in this repo are short and plain; bodies are rare but welcome
  for changes that alter *what* is tested.
- `test/` is excluded from the wheel (`pyproject.toml`) so 40 MB of baselines do
  not ship. Keep it that way.
- Full docs for the suite: [`test/regression/README.md`](../../test/regression/README.md).
