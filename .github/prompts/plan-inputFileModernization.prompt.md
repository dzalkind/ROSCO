<!-- checked in vscode -->
<!--  -->
# Plan: Input File Modernization — TOML-first with Cp Table Overhaul

Modernize ROSCO's input system to make TOML the sole forward-going format with rich inline comments (matching DISCON.IN readability), fix the fragile Cp/Ct/Cq text parser to use keyword-based scanning with improved section separators, unify the dual-schema descriptions into one source of truth, create a Python TOML writer, and provide a DISCON.IN → TOML migration utility. Drop DISCON.IN after one transition release.

## Current State

- C++ `load_from_toml()` already works (auto-generated from `rosco_types.yaml`)
- `Examples/DISCON_template.toml` exists with `#` comments for descriptions
- **No Python TOML writer** — `write_DISCON()` only writes legacy `.IN`
- **Two divergent schema sources**: `rosco_types.yaml` (C++ codegen) and `toolbox_schema.yaml` (Python toolbox)
- Cp/Ct/Cq tables: separate `.txt` file with fragile line-counting parser (`ReadCpFile`) — format is fine, parser needs keyword-based scanning
- No conversion utilities exist

---

## Phase 1: Unify Schema Descriptions

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

9. **Create `convert_discon_to_toml()`** — reads DISCON.IN via `read_DISCON()`, writes a `.toml` file with all parameters *(depends on 7)*
10. **Add CLI entry point**: `rosco convert-input old_DISCON.IN --output config.toml` *(depends on 9)*

## Phase 5: Deprecate DISCON.IN

11. **Add deprecation warnings** in `ReadControlParameterFileSub` (C++) and `write_DISCON()` (Python)
12. **Convert all example DISCON.IN files to `.toml`**; keep one legacy `.IN` for testing *(depends on 9)*
13. **Update `verify_cpp.py` scenarios** to use TOML inputs *(depends on 12)*
14. *(Future release)* **Remove DISCON.IN support entirely** — delete `readcontrolparameterfilesub.cpp` and `write_DISCON()`. `ReadCpFile` stays since Cp_Ct_Cq.txt is still used.

---

## Relevant Files

- `rosco/controller/rosco_registry/rosco_types.yaml` — canonical parameter registry; enhance descriptions
- `rosco/controller/rosco_registry/write_registry.py` — `_write_toml_template()`, `_write_cpp_io()`; add `_write_toolbox_schema()`
- `rosco/controller/src/ReadSetParameters/readcpfile.cpp` — rewrite with keyword-based scanning
- `rosco/controller/src/ReadSetParameters/readcontrolparameterfilesub.cpp` — legacy DISCON.IN parser; deprecate
- `rosco/toolbox/utilities.py` — add `write_DISCON_toml()`, update `write_rotor_performance()` separators, update `load_from_txt()`, add `convert_discon_to_toml()`
- `rosco/toolbox/inputs/toolbox_schema.yaml` — partially auto-generate from registry
- `Examples/Test_Cases/*/Cp_Ct_Cq.*.txt` — regenerate with improved separators
- `scripts/verify_cpp.py` — update scenarios to TOML

## Verification

1. `python scripts/verify_cpp.py --rebuild` with TOML inputs — all 27 scenarios byte-identical to baselines
2. Round-trip: `write_DISCON_toml()` → `load_from_toml()` → field-by-field comparison against `DISCON_dict()` output
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

1. **TOML table sections vs flat keys**: Current template uses flat keys. Adding `[filters]`, `[pitch_control]` hierarchy would improve organization but requires updating `load_from_toml()` key lookups. Recommendation: defer to a follow-up — migrate flat keys first, add sections later.
2. **Python TOML library**: `tomlkit` preserves comments (needed for descriptions). `tomllib` (stdlib 3.11+) for reading. Recommendation: use `tomlkit` for writing.
