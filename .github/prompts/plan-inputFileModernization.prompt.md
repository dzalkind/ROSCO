<!-- checked in vscode -->
<!--  -->
# Plan: Input File Modernization — TOML-first with Cp Table Overhaul

Modernize ROSCO's input system to make TOML the sole forward-going format with rich inline comments (matching DISCON.IN readability), fix the fragile Cp/Ct/Cq text parser to use keyword-based scanning with improved section separators, unify the dual-schema descriptions into one source of truth, create a Python TOML writer, and provide a DISCON.IN → TOML migration utility. Drop DISCON.IN after one transition release.

## Sequencing (decided 2026-09-21)

This plan starts **after** the regression plan's remaining work: P2/P3 tasks 10–15, 8c,
then 3b (the C++ vocabulary rename, which touches `write_registry.py` and
`readcontrolparameterfilesub.cpp` — so it must land before this plan edits them). The
regression plan's task 8b (input-parsing test) has moved here, as Phase 0.

## Current State

- C++ `load_from_toml()` parses a TOML file (generated from `rosco_types.yaml`), **but the
  TOML path is incomplete** — see the next bullet. Nothing in `test/`, `rosco/test/` or the
  Examples runs a `.toml` input.
- **The TOML path skips all post-processing.** `read_config_files()`
  (`readconfigfiles.cpp`) sends `.toml` to `load_from_toml()` and everything else to the
  hand-written `ReadControlParameterFileSub()`. Only the `.IN` parser then runs the
  post-processing (`readcontrolparameterfilesub.cpp:456` onward):
  - computed constants: `n_DT_Out` (also setting `DT_Out` from `DT` when 0), `n_DT_ZMQ`,
    `PC_RtTq99`, `VS_MinOMTq`, `VS_MaxOMTq`;
  - resolving relative `PerfFileName` / `OL_Filename` against the input file's directory;
  - `Y_Rate *= R2D` (unit conversion);
  - reading `OL_Filename` into `OL_Channels` and splitting it into the `OL_*` arrays.

  Instead, `load_from_toml()` reads those *computed* fields straight from the file, falling
  back to 0. A `.toml` input therefore runs with `PC_RtTq99 = 0`, unconverted `Y_Rate`,
  relative paths resolved against the working directory, no open-loop data, and
  `n_DT_Out = 0` — which `debug.cpp:587` uses as a modulus whenever logging is on.
- `ControlParameters` (214 fields in `rosco_types.yaml`) mixes **user inputs** and
  **computed state** with nothing in the registry to tell them apart. That is why the TOML
  reader and `DISCON_template.toml` treat computed fields as inputs.
- `Examples/DISCON_template.toml` exists with `#` comments for descriptions
- **No Python TOML writer** — `write_DISCON()` only writes legacy `.IN`
- **Two divergent schema sources**: `rosco_types.yaml` (C++ codegen) and `toolbox_schema.yaml` (Python toolbox)
- Cp/Ct/Cq tables: separate `.txt` file with fragile line-counting parser (`ReadCpFile`) — format is fine, parser needs keyword-based scanning
- No conversion utilities exist
- **`Echo` is parsed but dead.** `readcontrolparameterfilesub.cpp:207` parses `Echo` into
  `CntrPar.Echo`, and nothing ever reads the flag — there is no echo writer anywhere in
  `rosco/controller/src`. The `rosco_types.hpp` comment advertises
  `1 - Echo input data to <RootName>.echo`, which is not true today. So the controller
  currently has **no way to report what it parsed**, in any format.

---

## Phase 0: One input path, a parameter dump, and the input-parsing test

*Added 2026-09-21. Absorbs regression-plan task 8b. Everything later in this plan assumes
it: a TOML writer, a migration tool and TOML fixtures are all pointless while the TOML path
produces a different controller from the `.IN` path.*

0a. **Split parsing from post-processing.** Move the post-processing out of
   `ReadControlParameterFileSub()` into one function that `read_config_files()` calls after
   *either* reader. It needs the input file's directory (for relative paths) and
   `LocalVar.DT`. For `.IN` inputs this only moves code, so the hard gate must stay
   bit-identical (27 baselines). For `.toml` it fixes the bug in Current State.
   *No dependency — first.*

0b. **Mark each registry field as input or computed.** Add a flag to `rosco_types.yaml`
   (e.g. `computed: true` on `n_DT_Out`, `PC_RtTq99`, `OL_Channels`, …). Generator changes:
   `load_from_toml()` reads only input fields; `DISCON_template.toml` lists only input
   fields. The `.IN` parser is hand-written, so it needs no change. *Depends on 0a.*

0c. **Generate `dump_to_toml()`** in `rosco_types_io.cpp`, the inverse of
   `load_from_toml()`, from the same YAML. Called at two points:
   - **after parsing, before post-processing, input fields only.** This is a valid input
     file: what `Echo = 1` should write to `<RootName>.echo`, and a C++ route to the
     `.IN` → TOML conversion (compare with Phase 4, which plans a Python route).
   - **after post-processing, all fields.** The full-state snapshot for the 8b test.

   Floats must round-trip exactly (`%.17g` or `std::to_chars`), or snapshots and the round
   trip will not compare bit-for-bit. Absolute paths must not appear in committed snapshots;
   write them relative, as the fixtures do. *Depends on 0b.*

0d. **Input-parsing regression tests (regression-plan task 8b).** Three checks, in
   increasing strength:
   1. *Snapshot:* each `test/regression/fixtures/scenario_NN.IN` → parse →
      full-state dump, compared with a committed snapshot. Catches parser and default-value
      changes for every registry field, including ones no fixture sets.
   2. *Round trip:* `.IN` → input dump (TOML) → `load_from_toml()` → input dump, and the
      two dumps must be identical. This exercises the generated TOML reader, and is the
      check that would have caught the `OutputFormat` defaults bug
      (`plan-outputFileModernization.prompt.md` follow-up #4).

      **Measured 2026-09-24 (regression plan task 10): `src/rosco_types_io.cpp` is at 0%
      line coverage — all 1,062 lines of it.** Every fixture is a `.IN` file, so the
      generated TOML reader is not merely undertested, it never executes in the suite at
      all. That is the largest single hole the coverage run found, and this check is what
      closes it.
   3. *Behaviour:* run the scenarios from the TOML dumps, and all **30** baselines must be
      byte-identical — on **both** platform folders, `darwin-arm64` and `linux-x86_64`
      (added 2026-09-22; see the regression README). This is step 13's acceptance test,
      reached early, and it is the only one of the three that would catch a post-processing
      change that re-times the filters (0f).

   Write "input parsing" in anything a contributor reads, not "layer B".
   *Depends on 0c.*

0e. **Array-length semantics, and the scenario 4 finding.** *Added 2026-09-24.* Upstream
   Fortran ROSCO 2.9.0 cannot read `test/regression/fixtures/scenario_04.IN` at all — it
   dies with `Did not find correct size F_NotchBetaDen` — while the C++ `.IN` parser accepts
   it and runs. This surfaced during the Fortran cross-check (regression plan task 16),
   where scenario 4 is the one scenario of 30 that cannot be checked against the reference
   implementation.

   Two things make this a Phase 0 item rather than a curiosity. First, it means the two
   `.IN` parsers disagree about how an array field's declared length relates to the values
   on the line, so "the `.IN` format" is not one format — worth knowing before writing a
   migration tool that claims to read every DISCON in the wild. Second, TOML arrays carry
   their own length, so the mismatch has to be resolved explicitly when a field is declared
   in `rosco_types.yaml`: is the count a separate input field, or derived from the array?
   Decide it with 0b, since it is the same question as input-vs-computed.

   *Check:* diff the C++ and Fortran readers on `F_NotchBetaDen` and its `*_N` companion,
   and establish which fixture is malformed — scenario 4's, or the Fortran's expectation.
   If the fixture is wrong, fixing it moves scenario 4's baseline.

0f. **Do not let post-processing quietly re-time the filters.** *Added 2026-09-24.* 0a moves
   post-processing to run after either reader, and it takes `LocalVar.DT`. Regression task
   16 is the cautionary tale: every filter in the controller sizes its coefficients on the
   `iStatus == 0` call and caches them, so anything that changes what `DT` is at that moment
   silently changes every filtered signal — in that case by a factor of four, undetected for
   the life of the baselines. The 0d behaviour test (scenarios run from TOML, bit-identical
   against the baselines) is what catches this, which is another reason 0d is not optional.

**Open decision — how the test gets a dump.** Recommended: via `Echo = 1`. The test copies
a fixture to a temp dir, sets `Echo = 1`, makes one `iStatus = 0` call, and reads the
`.echo` file. That makes the documented feature real without adding to the library's
public interface. Alternative: a separate exported dump function — simpler for the test,
but a new public symbol.

## Phase 1: Unify Schema Descriptions

**Known collisions to resolve here (found 2026-09-22):**

- **`PS_Mode` means two different things.** Tuning-side (`controller.py:502-508`) it
  selects *how* the minimum-pitch schedule is computed: 1 peak shaving, 2 Cp-maximizing,
  3 both. Runtime-side it is a plain on/off switch — `utilities.py:540` writes
  `int(PS_Mode > 0)` into the DISCON, and `pitchcontrol.cpp:64` only tests `> 0`. Both
  schema sections already document their own meaning correctly, so nothing is *wrong*; the
  name is. **Option C, deferred here:** rename the tuning-side parameter (`PS_TuneMode`, or
  `MinPitch_Mode`) and leave the DISCON's `PS_Mode` as the 0/1 switch. Costs a YAML
  migration in `Tune_Cases/*.yaml`, `toolbox_schema.yaml` and `controller.py`, which is why
  it waits for this phase rather than riding along with a checkinputs fix.
  *Done 2026-09-22 (option A):* the misleading `checkinputs.cpp` message ("must be 0 or 1"
  while accepting 0-3) is fixed and the registry description now states both meanings.
- **`TRA_Mode` had three definitions.** The registry said `{0 none, 1 fore-aft damping,
  2 exclusion zone, 3 both}`, the toolbox schema says `{0 none, 1 frequency exclusion}`
  (max 1), and the C++ runtime calls `RefSpeedExclusion` for `TRA_Mode > 0`
  (`speedsetpoints.cpp:71`). The registry description was stale — fore-aft damping is
  `TD_Mode`, which had inherited the *same* stale description. Both fixed 2026-09-22, and
  the dead `if (TRA_Mode > 1)` validation block in `checkinputs.cpp` now reads `> 0`.
- **`Examples/28_tower_resonance.py` is broken by this confusion.** It sets
  `controller_params['TRA_Mode'] = 2` (line 50), which exceeds the toolbox schema's own
  maximum of 1 — and because the example mutates the dict after `load_rosco_yaml()`, nothing
  validates it. The resulting DISCON makes the controller refuse: verified 2026-09-22,
  `ROSCO ERROR: CheckInputs: TRA_Mode must be 0 or 1.` Either the example means
  `TRA_Mode = 1`, or frequency exclusion was meant to be mode 2 of a 0-3 scheme that the C++
  never implemented. *Resolved 2026-09-22 (`9505fc08`): the example now sets `TRA_Mode = 1`.*
  The deeper question — whether frequency exclusion was meant to be mode 2 of a 0-3 scheme
  the C++ never implemented — is still open and belongs to this phase's audit.
- **The toolbox schema's own ranges are unreliable** (found by regression task 11, restated
  here because this phase owns the fix): `AWC_Mode` max 2 though 5 exists, `Flp_Mode` max 2
  though 3 exists, `PF_Mode` max 1 though 2 exists, and its two sections disagree with each
  other. `mode_coverage.py` had to take its value domains from `checkinputs.cpp` and the
  controller source instead. Step 3 below — generating the DISCON section of
  `toolbox_schema.yaml` from `rosco_types.yaml` — is what stops this recurring; until then,
  treat the schema's maxima as advisory.

1. **Audit descriptions in both YAMLs** — diff `rosco_types.yaml` vs `toolbox_schema.yaml` for mismatches and gaps
2. **Make `rosco_types.yaml` the canonical source** — enhance its descriptions and add `units` fields where missing
3. **Auto-generate the DISCON section of `toolbox_schema.yaml`** from `rosco_types.yaml` — add `_write_toolbox_schema()` to `write_registry.py`

## Phase 2: Fix Cp/Ct/Cq Parser + Improve File Format

4. **Update `write_rotor_performance()` in `utilities.py`** — use improved section separators with dimension/axis info:
   ```
   # ===== Pitch angles [deg], 36 values =====
   -5.0  -4.0  -3.0  ...  30.0

   # ===== TSR values [-], 26 values =====
   2.0  2.5  3.0  ...  14.5

   # ===== Power coefficient (Cp) [26 rows=TSR x 36 cols=Pitch] =====
   0.006673   0.009813   ...

   # ===== Thrust coefficient (Ct) [26 rows=TSR x 36 cols=Pitch] =====
   0.128717   0.128402   ...

   # ===== Torque coefficient (Cq) [26 rows=TSR x 36 cols=Pitch] =====
   0.000532   0.000624   ...
   ```
   *No dependency.*

5. **Rewrite C++ `ReadCpFile` with keyword-based scanning** — replace hardcoded `skipLines()` calls with a `scanToKeyword()` helper that searches for section markers (`Pitch angle`, `TSR`, `Power coefficient`, `Thrust coefficient`, `Torque coefficient`). This makes the parser tolerant of extra blank lines, reordered comments, or added annotations. Keep the same `PerformanceData` struct output. *No dependency — parallel with step 4.*

6. **Update Python `load_from_txt()`** — ensure it uses the same keyword-based scanning (it largely already does, but verify consistency with new separator format). *Depends on step 4.*

## Phase 3: Python TOML Writer *(parallel with Phase 2)*

7. **Create `write_DISCON_toml()`** in `utilities.py` — uses `tomlkit` to write commented TOML with `# description [units]` above each parameter, matching DISCON.IN readability *(depends on Phase 1)*
8. **Update `DISCON_dict()`** — ensure the parameter dictionary feeds the TOML writer correctly *(depends on 7)*

## Phase 4: Migration Utility

9. **Create `convert_discon_to_toml()`** — reads DISCON.IN via `read_DISCON()`, writes a `.toml` file with all *input* parameters (not the computed ones — see Phase 0b) *(depends on 7)*. Phase 0c's C++ input dump already produces the same file; decide then whether the Python route is still needed, or becomes a thin wrapper around it, and test that the two agree.
10. **Add CLI entry point**: `rosco convert-input old_DISCON.IN --output config.toml` *(depends on 9)*

## Phase 5: Deprecate DISCON.IN

11. **Add deprecation warnings** in `ReadControlParameterFileSub` (C++) and `write_DISCON()` (Python)
12. **Convert all example DISCON.IN files to `.toml`**; keep one legacy `.IN` for testing *(depends on 9)*. The regression fixtures (`test/regression/fixtures/scenario_01..28.IN`, committed in `4523559c`) are the canonical set to convert, and the "one legacy `.IN`" should be one of them, so the DISCON path keeps a live regression test until step 14 removes it.
13. **Point the regression scenarios at TOML fixtures** *(depends on 12)*. The scenarios no longer run the tuner: they read committed fixtures. So this step means converting `test/regression/fixtures/*.IN` to `.toml` and changing the fixture column of the scenario table (regression-plan task 13). Keep `--write-fixtures` able to regenerate the TOML set from the tuning YAML. It must leave all 27 baselines byte-identical, since the controller's behaviour must not depend on which format configured it. That equality is the strongest test in this plan — and Phase 0d check 3 will already have proven it once.
14. *(Future release)* **Remove DISCON.IN support entirely** — delete `readcontrolparameterfilesub.cpp` and `write_DISCON()`. `ReadCpFile` stays since Cp_Ct_Cq.txt is still used, and so does the post-processing function from Phase 0a — move it out of `readcontrolparameterfilesub.cpp` before deleting that file.

---

## Relevant Files

- `rosco/controller/rosco_registry/rosco_types.yaml` — canonical parameter registry; enhance descriptions
- `rosco/controller/rosco_registry/write_registry.py` — `_write_toml_template()`, `_write_cpp_io()`; add `_write_toolbox_schema()`
- `rosco/controller/src/ReadSetParameters/readcpfile.cpp` — rewrite with keyword-based scanning
- `rosco/controller/src/ReadSetParameters/readcontrolparameterfilesub.cpp` — legacy DISCON.IN parser (hand-written, not generated); post-processing moves out in Phase 0a; deprecate
- `rosco/controller/src/ReadSetParameters/readconfigfiles.cpp` — `read_config_files()` dispatches `.toml` vs `.IN`; gains the shared post-processing call in Phase 0a
- `rosco/controller/src/rosco_types_io.cpp` — generated; gains `dump_to_toml()` in Phase 0c
- `test/regression/fixtures/` — the committed `.IN` set; Phase 0d snapshots and step 13 TOML fixtures live alongside
- `rosco/toolbox/utilities.py` — add `write_DISCON_toml()`, update `write_rotor_performance()` separators, update `load_from_txt()`, add `convert_discon_to_toml()`
- `rosco/toolbox/inputs/toolbox_schema.yaml` — partially auto-generate from registry
- `Examples/Test_Cases/*/Cp_Ct_Cq.*.txt` — regenerate with improved separators
- `test/regression/scenarios.py` — scenario table; point it at TOML fixtures in step 13 (moved from `Examples/vit_sim.py` 2026-09-21)
- `test/regression/run_regression.py` — regression runner (moved from `scripts/verify_cpp.py` 2026-09-21)

## Verification

0. After Phase 0a (before any TOML work): `.IN` inputs still give all 27 scenarios byte-identical to baselines — the post-processing move is pure code motion
1. `python test/regression/run_regression.py --rebuild` with TOML inputs — all 27 scenarios byte-identical to baselines (or `pytest test/regression`)
2. Round-trip: `write_DISCON_toml()` → `load_from_toml()` → field-by-field comparison against `DISCON_dict()` output. The C++ side of this is Phase 0d check 2.
3. Migration: `convert_discon_to_toml()` on each existing example, then re-run verification
4. Regenerate registry, verify `rosco_types_io.cpp` compiles and passes all scenarios
5. Manual inspection of generated `.toml` for readability — comments should match DISCON.IN clarity

## Decisions

- TOML is the sole forward format; DISCON.IN deprecated then removed
- Cp/Ct/Cq stays as separate `.txt` file — format is fine, just needs better section separators and a robust keyword-based parser. Separators include `rows=TSR x cols=Pitch` axis labels for clarity.
- `rosco_types.yaml` is single source of truth for descriptions
- Count scalars (e.g., `PC_GS_n`) derived from array lengths — not written explicitly
- No backwards compatibility required — migration utility bridges the gap

## Further Considerations

0. **Superseded 2026-09-21 by Phase 0.** This note used to say a generated
   `dump_to_toml()` could unblock the regression plan's task 8b independently of this plan.
   Reading the code showed that premise was wrong: the TOML path skips the `.IN` parser's
   post-processing, so a dump of parsed state is not a valid input file, and a `.IN`-only
   dump test would never have exercised the generated TOML reader where the `OutputFormat`
   bug lived. The dump is still the right tool, but it has to come after the parse /
   post-processing split and the input-vs-computed flag, which is why 8b now lives here as
   Phase 0d.

1. **TOML table sections vs flat keys**: Current template uses flat keys. Adding `[filters]`, `[pitch_control]` hierarchy would improve organization but requires updating `load_from_toml()` key lookups. Recommendation: defer to a follow-up — migrate flat keys first, add sections later.
2. **Python TOML library**: `tomlkit` preserves comments (needed for descriptions). `tomllib` (stdlib 3.11+) for reading. Recommendation: use `tomlkit` for writing.
