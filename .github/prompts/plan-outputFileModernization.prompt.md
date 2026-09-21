# Plan: Modernize ROSCO Debug Output System

## TL;DR
Replace the slow, large text-based `.RO.dbg` files with a compact binary format (HDF5 primary, text optional), eliminate the `DebugVar` struct by promoting its fields into `LocalVar`, keep avrSWAP logging, and auto-generate all debug I/O from the registry. The registry (`rosco_types.yaml` + `write_registry.py`) becomes the single source of truth -- no `source` field needed since all outputs are `LocalVar.{name}`.

## Current Status — ALL PHASES COMPLETE
- **Phase 4 DONE** (commit `72b68f60`): `load_hdf5_output()` in `output_processing.py` auto-detects `.RO.h5` vs `.RO.dbg`; the regression runner supports `--hdf5` comparison mode.
- **Phase 3 DONE** (commit `ce2d701b`): `HDF5DebugWriter::open_avrswap()`/`write_avrswap_row()` wired into `_write_cpp_debug()`; avrSWAP written as `/avrSWAP` dataset in the same `.RO.h5` file when `OutputFormat=1`; text `.dbg3` path unchanged. Scenario 28 verifies avrSWAP (39998, 85) byte-identical.
- **Phase 2 DONE** (commit `f67137f5`): `DebugWriter` abstraction (`debug_writer.hpp`, `text_debug_writer.cpp`, `hdf5_debug_writer.cpp`); `Debug()` is a thin dispatcher calling `write_row()`; `_write_cpp_debug()` generates `DebugWriter::create(OutputFormat)`. Verified 27/27 scenarios IDENTICAL, text `.dbg` byte-identical to pre-refactor, HDF5 == text (39,999x27).
- **Phase 1 DONE**: HDF5 optional dep in CMake (`ROSCO_HDF5`, auto-detect via conda prefix); `OutputFormat` param (0=text, 1=HDF5) wired through DISCON -> `ControlParameters`, now **defaults to HDF5** (commit `ce2d701b`).
- **Phase 0 DONE**: `DebugVar` eliminated; all 26 debug fields promoted into `LocalVariables`
- **Registry-driven debug.cpp DONE**: `write_registry.py` now generates `src/IO/debug.cpp` from YAML flags
- **`dbg: true` mechanism DONE**: Add this flag to any `LocalVariables` field in YAML → it appears in `.dbg`
- **`dbg_name` override DONE**: Column label can differ from field name (e.g., `PC_PitComT` → `PC_PICommand`)
- **LoggingLevel > 1 guard DONE**: `.dbg2` population/clamping skipped when not needed
- **`cpp: false` flag DONE**: Fortran-only fields excluded from C++ generation
- **`AWC_complexangle` split into `_re`/`_im`** real arrays; `ACC_INFILE_SIZE` removed
- **Fortran registry generation removed entirely** (commits `3084d3f3`, `88f3a841`): this is a pure-C++ controller now — `write_registry.py` no longer emits `ROSCO_Types.f90`/`ROSCO_IO.f90`, and those files are deleted from the repo.

## Design Decisions
- **HDF5 is the new default format** -- self-describing, excellent numpy/pandas support, widely used in wind energy. NetCDF-4 is HDF5 under the hood, so adding netCDF later is trivial.
- **Text output remains available** as opt-in (`OutputFormat = 0`) for quick terminal inspection.
- **.dbg (primary output) is a configurable list of LocalVar field names** -- defaults to fields with `dbg: true`. Users can override in config.
- **.dbg2 (all LocalVars) stays automatic** -- every LocalVariable scalar is always written. With HDF5, size/speed is no longer a concern.
- **.dbg3 (avrSWAP) is KEPT** -- avrSWAP is essential for solver interface debugging. In HDF5 mode, written as an additional dataset in the same file.
- **Registry-driven code generation** -- `write_registry.py` generates data packing, names, and units from `rosco_types.yaml`. Adding a new output = add `dbg: true` to YAML field, re-run generator.

## Remaining Steps

### Phase 1: Add HDF5 build infrastructure  — DONE
1. Add HDF5 as optional dependency in `CMakeLists.txt` via `find_package(HDF5)`. Guard with `ROSCO_HDF5` cmake option (default ON if found).
2. Create `DebugWriter` abstraction in `src/IO/debug_writer.hpp`:
   - `open(root_name, format, var_names, var_units)` -- opens file, writes metadata
   - `write_row(time, data_ptr, n)` -- appends one timestep
   - `close()` -- flushes and closes
   - Two backends: `HDF5DebugWriter` (chunked + gzip), `TextDebugWriter` (current format)
3. Add `OutputFormat` parameter to `ControlParameters` in ROSCO input file (DISCON): `{0 = text, 1 = HDF5}`.

### Phase 2: Refactor debug.cpp to use DebugWriter  — DONE
4. Refactor `debug.cpp` (generated) to use `DebugWriter` abstraction. The `Debug()` function becomes a thin dispatcher that:
   - Packs data into arrays (already done by registry generation)
   - Calls `writer->write_row()`
5. Update `_write_cpp_debug()` in `write_registry.py` to generate code that instantiates the appropriate backend based on `OutputFormat`.

### Phase 3: avrSWAP logging in HDF5  — DONE
6. In HDF5 mode, write avrSWAP as a second dataset (`/avrSWAP`) in the same `.RO.h5` file, with column labels `AvrSWAP(1)..AvrSWAP(N)`. In text mode, keep `.dbg3` as-is.

### Phase 4: Python tooling  — DONE
7. Update `rosco.toolbox` Python readers to auto-detect format (text vs HDF5) when loading `.RO.dbg` / `.RO.h5` files.
8. Update the regression runner to support HDF5 comparison.

## Registry YAML Flags (implemented)
- `dbg: true` — include field in `.dbg` output (LoggingLevel >= 1)
- `dbg_name: X` — override `.dbg` column header
- `cpp: false` — skip in C++ code generation (Fortran-only field)
- Units extracted from `[unit]` in description string; defaults to `[N/A]`

## Relevant Files
*Paths updated 2026-09-21: the verification harness moved from `scripts/verify_cpp.py` to
`test/regression/run_regression.py`, and the scenarios from `Examples/vit_sim.py` to
`test/regression/scenarios.py`. Sim names are now `regression_N.RO.dbg` / `.RO.h5`, not
`vit_simN` — relevant to the HDF5 comparison, which looks those filenames up.*

- `rosco/controller/rosco_registry/rosco_types.yaml` — source of truth for all types and debug flags
- `rosco/controller/rosco_registry/write_registry.py` — generates debug.cpp, rosco_types.hpp, rosco_types_io.cpp
- `rosco/controller/src/IO/debug.cpp` — **AUTO-GENERATED**, do not edit manually
- `rosco/controller/src/include/vit_types.h` — C++ struct definitions
- `rosco/controller/CMakeLists.txt` — add HDF5 dependency (Phase 1)
- `src/IO/debug_writer.hpp` (new, Phase 1) — writer abstraction
- `src/IO/hdf5_debug_writer.cpp` (new, Phase 1) — HDF5 backend
- `test/regression/run_regression.py` — verification harness (was `scripts/verify_cpp.py` until 2026-09-21)

## Verification
1. Current: `python test/regression/run_regression.py --rebuild` — 27 scenarios, 5,252,000 float64 values byte-identical
2. After Phase 2 with `OutputFormat=0`: same 27-scenario text verification
3. After Phase 2 with `OutputFormat=1`: load HDF5 in Python, compare against text baselines
4. Inspect HDF5 with `h5dump` / `h5py`: variable names, units, timestamps, avrSWAP dataset
5. `LoggingLevel=0`: no output files created

## Further Considerations
1. **Chunked + compressed HDF5**: gzip level 1-4 gives good write speed and ~5-10x size reduction vs text. Recommend chunked with light compression.
2. **Ring-buffer mode**: For very long sims, future option to keep only last N seconds.
3. **Row-major storage**: One row per timestep, natural for C++ append workloads.
4. **User-configurable .dbg columns**: Allow DISCON.IN/DISCON.toml to specify which LocalVar fields appear in .dbg (override the `dbg: true` defaults).

## Follow-up Issues (2026-08-03)
1. **Checkpoint/restart closes logging permanently**: Generated `Debug()` closes writers for every negative `iStatus`, including checkpoint (`-8`) and warm restart (`-9`); subsequent normal timesteps do not reopen them. Update `_write_cpp_debug()` to close only at terminal shutdown and correctly replace writers on restart.
2. **No-HDF5 fallback writes invalid `.h5` files**: Requesting HDF5 without compiled HDF5 support silently selects the text writer but retains the `.RO.h5` name and HDF5 avrSWAP route. Reject unavailable HDF5 explicitly, or route based on the effective writer format.
3. **Registry regeneration breaks generated Fortran**: Resolved — `write_roscoio()` and the generated `ROSCO_IO.f90` were dead code (no build target consumed them in this pure-C++ controller); removed instead of patched (commit `3084d3f3`). Follow-up (commit `88f3a841`) removed `write_types()`/`ROSCO_Types.f90` generation too, for the same reason — the registry now only generates C++.
4. **HDF5 default decision is not implemented**: Resolved — added `equals: 1` to `OutputFormat` in `rosco_types.yaml` and fixed `_write_cpp_io()` to honor per-field `equals` defaults in the TOML loader's `value_or(...)` (previously hardcoded to `0` regardless of the registry default). Regenerated; all 27 `.IN`-based verification scenarios remain byte-identical since those fixtures set `OutputFormat` explicitly.
5. **Toolbox reader drops HDF5 avrSWAP data**: Resolved — `load_hdf5_output()` now reads the `/avrSWAP` dataset and its `column_labels` attribute when present, returning them via `info['avrSWAP']` / `info['avrSWAP_channels']`, and `_load_fast_data()` surfaces them on `fast_data['avrSWAP']` / `fast_data['avrSWAP_channels']` without polluting the main channel matrix.
6. **HDF5 verification permits missing output**: Resolved — this uncovered that Phase 3 (avrSWAP-in-HDF5) was never wired up: `debug.cpp`'s `.dbg3` path always wrote text regardless of `OutputFormat`, even though `HDF5DebugWriter::open_avrswap()`/`write_avrswap_row()` existed unused. Fixed `_write_cpp_debug()` to route avrSWAP through `dbg_writer->open_avrswap()`/`write_avrswap_row()` (same HDF5 file as `.dbg`) when `OutputFormat=1`, keeping the text `.dbg3` path unchanged otherwise. Scenario 28 now sets `LoggingLevel=3` and captures the full `avrSWAP(1..85)` array from the Python sim loop as ground truth; `compare_hdf5_debug()`/`compare_hdf5_avrswap()` in `run_regression.py` now require exact channel-set equality (not just intersection) and verify avrSWAP column labels, shape, and values row-for-row (accounting for the controller's own init-call row, which precedes Python's tracked loop and has no ground truth). `python test/regression/run_regression.py --hdf5` passes: 27/27 scenarios byte-identical, HDF5 text/channel parity, and avrSWAP (39998, 85) identical.
