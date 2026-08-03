# ROSCO-C Refactoring Notes

## Overview

This branch (`c++`) contains a modernized pure C++ version of ROSCO, translated from Fortran by the VIT tool and subsequently refactored. The Fortran source has been removed entirely. This is intended for a future v3.0 release; `master` is unchanged for existing users.

---

## What Changed from the VIT Translation

### Registry-driven code generation

Parameter management is now automated. The authoritative source is:

- `rosco/controller/rosco_registry/rosco_types.yaml` — master parameter registry (YAML with anchors/aliases for 213 parameters)

Running the generator:
```bash
python3 rosco/controller/rosco_registry/write_registry.py
```

This emits three files automatically:
- `rosco/controller/src/include/rosco_types.hpp` — `ControlParameters` C++ struct with `std::vector<>` fields
- `rosco/controller/src/rosco_types_io.cpp` — TOML loader
- `rosco/controller/src/IO/debug.cpp` — `.RO.dbg` and `.RO.dbg2` output (field lists, column names, units)
- `Examples/DISCON_template.toml` — annotated TOML template with all parameters and defaults

**To add a new parameter:** edit `rosco_types.yaml` and re-run the generator. The struct, parser, bridge, and template all update automatically.

**To add a new debug output column:** add `dbg: true` to the field in `rosco_types.yaml` (optionally with `dbg_name:` for a custom column header) and re-run the generator.

### TOML input file support

The controller now accepts `.toml` input files in addition to the original `DISCON.IN` format. Detection is automatic based on file extension.

```toml
# Example DISCON.toml
[Filters]
F_LPFCornerFreq = 1.674
F_LPFType = 1
F_NotchFreqs = [1.5, 3.0]

[PitchControl]
PC_ControlMode = 1
PC_GS_angles = [0.057, 0.084, 0.106]
PC_GS_KP     = [-0.012, -0.013, -0.014]
```

Key improvements over `DISCON.IN`:
- Native array syntax — no more count scalars (`PC_GS_n`, `F_NumNotchFilts`, etc.)
- `#` comments, section headers, type validation at parse time
- See `Examples/DISCON_template.toml` for a fully annotated starting point

Existing `DISCON.IN` files continue to work unchanged.

### LabVIEW compatibility

The `DISCON` entry point is wrapped in a `try/catch` exception boundary so that C++ exceptions (e.g. `std::bad_alloc` from vector operations in `load_from_toml()`) cannot propagate out to LabVIEW's Call Library Function Node. If an exception occurs, `*aviFAIL` is set to `-1` and the message is written to `avcMSG` rather than crashing.

The C++ DLL is also a better fit for LabVIEW than the Fortran version because it has no Fortran runtime dependencies (`libgfortran`, `libgcc_s`, etc.) and uses a clean `extern "C"` signature with no hidden CHARACTER length arguments.

---

## Building

### Prerequisites

- CMake ≥ 3.14
- C++17 compiler (clang++ or g++)
- Python 3 with `rosco` toolbox installed (for verification)

### First-time setup

```bash
mkdir -p build
cd build
cmake ../rosco/controller
```

CMake will fetch `toml++` automatically via FetchContent on first configure. No other external dependencies are required (ZeroMQ is optional and detected automatically).

### Build the shared library

```bash
cmake --build build
```

Output: `build/libdiscon.dylib` (macOS) or `build/libdiscon.so` (Linux).

To install into `rosco/lib/` (where the Python toolbox and `verify_cpp.py` expect it):

```bash
cmake --install build
```

Or use the `--rebuild` flag in `verify_cpp.py` which handles the copy automatically.

---

## Verification

The `baseline_arrays/` directory contains 27 frozen `.npz` output files captured from the verified pure-C++ build. These are byte-identical to the original Fortran outputs and serve as the regression baseline for all refactoring work.

### Running the verification suite

```bash
# Verify current build against all 27 scenario baselines:
python3 scripts/verify_cpp.py

# After a code change that requires a rebuild:
python3 scripts/verify_cpp.py --rebuild

# Single scenario:
python3 scripts/verify_cpp.py --scenario 1
```

Each scenario runs in a **separate subprocess** to reset the DLL's static variables between scenarios (same isolation that Docker exec provided during the VIT translation workflow). No Docker is required.

`libscrub.so` is built automatically on first run — it prevents a scipy FITPACK non-determinism bug in Scenario 3 (see dev note 202603261512).

### Updating the baseline

If a refactoring intentionally changes controller outputs (e.g. Phase 2 filter class changes), re-capture and commit:

```bash
python3 scripts/verify_cpp.py --rebuild --update-baseline
git add baseline_arrays/
git commit -m "Update baseline arrays after intentional output change"
```

---

## Architecture

```
rosco_types.yaml          ← edit this to add/change parameters & debug outputs
       │
write_registry.py         ← run this to regenerate
       │
       ├── rosco_types.hpp         ← ControlParameters struct (generated)
       ├── rosco_types_io.cpp      ← TOML loader (generated)
       ├── debug.cpp               ← .dbg/.dbg2 output arrays (generated)
       └── DISCON_template.toml   ← annotated input template (generated)

discon.cpp                ← thin orchestrator, calls stages in order
  ├── stage_1_sensing()           ← ReadAvrSWAP (+ future sensor models)
  ├── stage_2_setup()             ← defaults, banner, config, SetParameters, ExtDLL, ZMQ
  ├── stage_3_filtering()         ← LP / notch filtering
  ├── stage_4_estimation()        ← wind speed estimator
  ├── stage_5_supervisory()       ← power-ref setpoints, shutdown, startup
  ├── stage_6_setpoints()         ← speed setpoints, torque state machine
  ├── stage_7_actuators()         ← torque, pitch, yaw, flap, cable, StC
  ├── stage_8_output()            ← debug logging, checkpoint writing
  └── try/catch boundary          ← LabVIEW compatibility

Stages/                   ← one file per stage, numbered for execution order
  stage_1_sensing.cpp … stage_8_output.cpp
```

---

## Refactoring Steps

The following steps were taken after the initial VIT translation and TOML/registry setup. Each step passes all 27 regression scenarios.

### 1. ArrayView wrapper for lookup tables

The VIT-translated `interp1d` took six arguments for a single 1-D interpolation: a raw pointer and integer length for each axis. An `ArrayView` struct (`{pointer, length}`) was introduced to bundle these into pairs, reducing call-site clutter:

```cpp
// Before
interp1d(CntrPar->PS_WindSpeeds, CntrPar->n_PS_WindSpeeds,
         CntrPar->PS_BldPitchMin, CntrPar->n_PS_BldPitchMin,
         LocalVar->WE_Vw_F, ErrVar);

// After
interp1d({CntrPar->PS_WindSpeeds, CntrPar->n_PS_WindSpeeds},
         {CntrPar->PS_BldPitchMin, CntrPar->n_PS_BldPitchMin},
         LocalVar->WE_Vw_F, ErrVar);
```

This was a transitional step — once the C struct bridge was removed (step 3), `interp1d` takes `std::vector` directly and `ArrayView` is no longer needed.

### 2. Per-instance state structs for filters and controllers

The flat `[1024]` parallel arrays in `filterparameters_t`, `piparams_t`, `resparams_t`, and `rlparams_t` were replaced with `std::vector<PerInstanceState>`. Each filter/controller instance now groups its fields in one struct instead of spreading them across a dozen parallel arrays:

```cpp
// Before — parallel arrays (Fortran COMMON block style)
piP->ITerm[idx] = piP->ITerm[idx] + DT * ki * error;
piP->ITermLast[idx] = piP->ITerm[idx];

// After — one struct per instance
PIState& s = inst_ref(piP->pi, idx);
s.iterm = s.iterm + DT * ki * error;
s.iterm_last = s.iterm;
```

State structs are defined in `src/include/rosco_objects.hpp`. Vectors grow on first use and stay sized to actual instance count.

**Restart file format**: the checkpoint binary format changed (previously wrote fixed 1024-element arrays, now writes count + N elements). Old `.chkp` files from prior builds are not compatible.

### 3. Removed the C parameter struct bridge

The VIT translation produced a flat C struct (`controlparameters_view_t`) with raw `double*` pointers and `int` size fields for every array — mirroring the Fortran memory layout. A `populate_view()` function copied all 213 parameters from the C++ `ControlParameters` object into this C struct so downstream functions could use it.

This bridge was eliminated: all functions now take `const ControlParameters&` directly. The `populate_view()` function and `controlparameters_view_t` struct were deleted. `interp1d` now takes `std::vector` arguments, so no separate size fields or `ArrayView` wrappers are needed.

### 4. Filters, rate limiters, and PI controllers → C++ objects

Building on the per-instance state structs (step 2), filters and controllers were turned into proper C++ classes. Each class owns its state and exposes a `step()` method, replacing the free-function + instance-counter pattern inherited from Fortran.

### 5. Source file reorganization

Source files were reorganized into groups by function:
- `Controllers/` — pitch, torque, yaw, IPC, structural, cable control
- `ControllerBlocks/` — setpoints, peak shaving, wind speed estimator
- `Estimators/` — wind speed, power, etc.
- `Filters/` — LP, HP, notch, second-order LP

### 6. References and dot notation throughout

Replaced pointer/arrow syntax with C++ references and dot notation across the codebase, so the code reads more like the control equations it implements:

```cpp
// Before
CntrPar->VS_TSRopt * LocalVar->WE_Vw / CntrPar->WE_BladeRadius

// After
CntrPar.VS_TSRopt * LocalVar.WE_Vw / CntrPar.WE_BladeRadius
```

### 7. Removed objInsts

The `objectinstances_t` struct held Fortran-style instance counters for each filter type (`instLPF`, `instHPF`, etc.). With filters now being proper objects that own their own state, these counters are no longer needed and the struct was removed.

### 8. C++ native error handling, removed ErrVars

Replaced the Fortran-heritage error pattern — passing `ErrVar` structs through every function and checking `ErrMsg` / `aviFAIL` flags after each call — with C++ exceptions. A `RoscoError` exception class (defined in `rosco_error.hpp`) is thrown at the point of failure and caught once at the `DISCON` boundary, where it is converted to the `aviFAIL = -1` / `avcMSG` channel that OpenFAST and LabVIEW expect.

```cpp
// Before — ErrVar threaded through every call, checked after each
interp1d(xData, yData, x, ErrVar);
if (ErrVar->aviFAIL < 0) return;

// After — throw at the error site, catch once at the top
double y = interp1d(xData, yData, x);  // throws RoscoError on failure
```

This removed `ErrVar` from nearly every function signature and eliminated hundreds of lines of error-forwarding boilerplate. A `rosco_warn()` function handles non-fatal warnings (e.g. restart file I/O) by printing to stderr without interrupting execution.

The verification script (`verify_cpp.py`) was also updated to check for memory errors using AddressSanitizer when available.

### 9. Stage functions — controller pipeline decomposition

The monolithic `DISCON()` function body was decomposed into eight numbered stage functions, each in its own source file under `src/Stages/`. DISCON is now a thin orchestrator that calls them in sequence:

| # | Stage | Functions inside |
|---|-------|------------------|
| 1 | `stage_1_sensing` | ReadAvrSWAP (+ future sensor models) |
| 2 | `stage_2_setup` | SetParameters, ExtController, UpdateZeroMQ |
| 3 | `stage_3_filtering` | PreFilterMeasuredSignals |
| 4 | `stage_4_estimation` | WindSpeedEstimator |
| 5 | `stage_5_supervisory` | PowerControlSetpoints, Shutdown, Startup |
| 6 | `stage_6_setpoints` | SpeedSetpoints, TorqueStateMachine, SetpointSmoother |
| 7 | `stage_7_actuators` | TorqueControl, PitchControl, YawRateControl, FlapControl, CableControl, StructuralControl |
| 8 | `stage_8_output` | Debug, WriteRestartFile |

All stages share a uniform signature `(float* avrSWAP, ControlParameters&, LocalVariables&, PerformanceData&, ExtControlType&)` so they can be called generically or exported individually for Simulink integration.

Key ordering notes:
- Stage 1 (sensing) precedes stage 2 (setup) because `ReadAvrSWAP` sets `iStatus`, which the DISCON orchestrator needs to gate first-call config loading (banner, DISCON.IN/TOML, performance tables). The numbering reflects execution order, not conceptual priority.
- First-call config loading runs inline in DISCON between stages 1 and 2 (not inside a stage function) because it requires the `accINFILE` parameter which is only available to DISCON.
- Shutdown and Startup are combined into `stage_5_supervisory` alongside PowerControlSetpoints. Order within the stage: PCS first (sets baseline PRC_R_Speed), then Shutdown, then Startup (overrides PRC_R_Speed during ramp).
- The ZMQ final-call edge case (`iStatus == -1`) is handled directly by DISCON outside the stage pipeline.

Files added: `src/Stages/stage_1_sensing.cpp`, `stage_2_setup.cpp`, `stage_{3..8}_*.cpp`, `src/include/rosco_stages.h`.

### 10. DISCON orchestrator cleanup — self-contained stages

Moved all inline logic from the `DISCON()` function body into `stage_2_setup` and `ReadSetParameters/`, leaving DISCON as a pure orchestrator (~30 lines of logic: store file paths, call stages, catch exceptions).

**What moved into `stage_2_setup`:**
- Default actuator signals (`avrSWAP[34]`, `[35]`, `[40]`, `[45]`, etc.)
- First-call banner print (`ROSCO_VERSION`)
- First-call config loading (now calls `read_config_files`)
- Warm restart logic (`ReadRestartFile` + `read_config_files` + re-run `ReadAvrSWAP`)

**What moved to `ReadSetParameters/readconfigfiles.cpp`:**
- The `read_config_files()` function (was `static` in `discon.cpp`) — auto-detects DISCON.IN vs TOML, calls the appropriate parser, loads Cp/Ct/Cq performance tables.

**RootName simplification:**
- Added `std::string RootName` field to `LocalVariables` (in `vit_types.h`)
- DISCON sets `LocalVar.RootName` from `avcOUTNAME` (via `GetRoot`) before calling stages
- `Debug`, `WriteRestartFile`, `ReadRestartFile` now read `LocalVar.RootName` directly — removed `char* RootName, int size_avcOUTNAME` parameters from all three
- `stage_8_output` now has the uniform stage signature (no extra `RootName` args)
- Eliminated the Fortran-style space-padded `char RootName[1024]` buffer and `trim_fortran_string` round-trip

**Warm restart in stage_2_setup:** The original DISCON ran warm restart *before* `ReadAvrSWAP` so checkpointed sensor values would be overwritten by current readings. With warm restart now in `stage_2_setup` (after `stage_1_sensing`), the sequence is: `ReadAvrSWAP` → `ReadRestartFile` (restores internal state, overwrites sensor fields) → `ReadAvrSWAP` again (re-reads current sensors). Net effect is identical.

### 11. ACC_INFILE modernization and GetRoot relocation

Replaced the Fortran-heritage `char ACC_INFILE[1024]` + `int ACC_INFILE_SIZE` pair with `std::string ACC_INFILE` in `LocalVariables`. DISCON now sets it with a one-liner: `LocalVar.ACC_INFILE = TrimBladedString(accINFILE, accINFILE_size)`. The `read_config_files()` function uses `LocalVar.ACC_INFILE` directly — no more manual null/space trimming.

`GetRoot()` moved from `discon.cpp` to `ReadSetParameters/readconfigfiles.cpp` as a non-static function declared in `vit_translated.h`. Simplified signature: takes `const std::string&` (no raw `char*` + length). DISCON calls `GetRoot(TrimBladedString(...))`.

Added `std::string` write/read overloads to `restart_fields.h` (length-prefixed, like vectors). Removed the `ACC_INFILE_SIZE` checkpoint field — the string length is encoded in the string serialization itself.

**Checkpoint compatibility:** This changes the binary `.chkp` format. Old checkpoint files from prior builds are not compatible.

### 12. Removed debugvariables_t, registry-driven debug output

The `debugvariables_t` struct was a Fortran-era artifact — a separate struct threaded through every function signature solely to collect ~30 output values for the `.RO.dbg` log file. Most of its fields were redundant copies of values already in `LocalVariables` (e.g. `DebugVar.GenSpeedF = LocalVar.GenSpeedF`), and the handful of unique fields were only computed in `WindSpeedEstimator` and `YawRateControl`.

**Struct elimination:**

- **10 unique fields promoted into `LocalVariables`** (in `rosco_types.yaml` and `vit_types.h`): `WE_Cp`, `WE_b`, `WE_w`, `WE_t`, `WE_lambda`, `YawRateCom`, `NacHeadingTarget`, `NacVaneOffset`, `Yaw_Err`, `YawState`
- **`debugvariables_t*` parameter removed** from all 8 stage functions, `Debug()`, and ~15 controller/estimator/filter functions
- **`static debugvariables_t DebugVar`** global removed from `discon.cpp`
- **Yaw state machine** (`yawratecontrol.cpp`): three local statics (`NacVaneOffset`, `YawState`, `NacHeadingError`) promoted to `LocalVar` fields. `YawState` stored as `double`, cast to `int` for state machine logic
- **~20 redundant copy lines removed** across `prefiltermeasuredsignals.cpp`, `pitchcontrol.cpp`, `speedsetpoints.cpp`, `windspeedestimator.cpp`
- **`DebugVariables` section retained in `rosco_types.yaml`** for Fortran registry generation only (not used by C++ code)

**Registry-driven debug.cpp (auto-generated):**

`debug.cpp` is now generated by `write_registry.py` (function `_write_cpp_debug()`). The file is tagged `AUTO-GENERATED from rosco_types.yaml — do not edit manually`. The output list is driven entirely by YAML annotations:

- **`.RO.dbg` columns** (26 fields): controlled by `dbg: true` on LocalVariables fields in `rosco_types.yaml`. The column order matches YAML declaration order. Fields can specify `dbg_name:` for a display name different from the field name (e.g. `PC_PitComT` has `dbg_name: PC_PICommand`, `NACIMU_FA_AccF` has `dbg_name: NacIMU_FA_AccF`). Units are extracted from the field's `description:` string.
- **`.RO.dbg2` columns** (172 fields): all scalar numeric `LocalVariables` fields (with `cpp: true` or omitted). This is the full internal state dump — no manual curation needed; adding a field to `LocalVariables` in the YAML automatically adds it to `.dbg2`. The `.dbg2` data array population is guarded by `LoggingLevel > 1` at runtime — when only `.dbg` output is requested, the 172-element array is never built, avoiding unnecessary overhead.
- **`.RO.dbg3`** (avrSWAP log): unchanged — still writes indices 1–85 plus CC/StC group indices.

To add a new debug output column: add `dbg: true` to the field in `rosco_types.yaml` and re-run `write_registry.py`.

**Column order change:** The `.RO.dbg` column order now follows YAML declaration order rather than the previous hardcoded Fortran order. The same 26 fields are present with the same values, but appear in a different sequence. Python tooling that reads `.RO.dbg` files by column name (header parsing) is unaffected; tools that index by column number will see a different layout.

**Name mappings in generated code:**
| YAML field | `.dbg` column name | Source expression |
|---|---|---|
| `PC_PitComT` | `PC_PICommand` | `LocalVar.PC_PitComT` |
| `NACIMU_FA_AccF` | `NacIMU_FA_AccF` | `LocalVar.NACIMU_FA_AccF` |
| `WE_v_m` | `WE_Vm` | `LocalVar.WE_v_m` |
| `WE_v_t` | `WE_Vt` | `LocalVar.WE_v_t` |

All other columns use the field name directly as the display name.

### 13. Windows/MSVC portability

Three commits addressed cross-platform build compatibility for Windows (Visual Studio / MSVC):

- **Removed `stdc++` linker library** — MSVC links the C++ runtime automatically; explicit `-lstdc++` is a GCC/Clang-ism that fails on Windows. `target_link_libraries` now lists only `tomlplusplus::tomlplusplus` (plus `dl` on non-MSVC platforms via a conditional guard).
- **Removed variable-length arrays (VLAs)** — VLAs are a C99/GCC extension not supported by MSVC. Replaced with fixed-size or `std::vector` allocations.
- **Conditional `dlfcn.h`** — `dlopen`/`dlsym` (used by `ExtController`) are POSIX-only. The external DLL loading code is now wrapped in `#ifndef _MSC_VER` guards, with a stub/error path for Windows builds (Windows equivalent via `LoadLibrary` is future work).

---

## Completed: Output File Modernization (HDF5)

Replaced the text-based `.RO.dbg` files with a compact HDF5 binary format (text remains available as opt-in). The registry (`rosco_types.yaml`) remains the single source of truth for all output fields.

**Design decisions:**
- HDF5 is the new **default** format (`OutputFormat` defaults to `1`) — self-describing, excellent numpy/pandas support, widely used in wind energy
- Text output available via `OutputFormat = 0`; HDF5 via `OutputFormat = 1`
- `.dbg` (primary) remains configurable via `dbg: true` fields; `.dbg2` (all LocalVars) stays automatic; `.dbg3` (avrSWAP) is written as a `/avrSWAP` dataset in the same `.RO.h5` file when `OutputFormat=1`, otherwise text as before
- Chunked + gzip level 1–4 for ~5–10× size reduction vs text

**Implementation phases (all DONE):**

| Phase | Description | Key changes |
|-------|-------------|-------------|
| 1 | HDF5 build infrastructure | `find_package(HDF5)` in CMakeLists, `DebugWriter` abstraction (`debug_writer.hpp`), `HDF5DebugWriter` + `TextDebugWriter` backends, `OutputFormat` parameter (now defaults to HDF5) |
| 2 | Refactor debug.cpp | Generated code uses `DebugWriter` abstraction; backend selected by `OutputFormat` |
| 3 | avrSWAP in HDF5 | `/avrSWAP` dataset wired into the same `.RO.h5` file via `open_avrswap()`/`write_avrswap_row()`; text `.dbg3` unchanged |
| 4 | Python tooling | `load_hdf5_output()` auto-detects and reads `/avrSWAP` + labels; `verify_cpp.py --hdf5` compares HDF5 vs text with exact channel-set + avrSWAP equality |

**Also removed along the way:** the Fortran registry generation (`ROSCO_Types.f90`, `ROSCO_IO.f90`) was dead code in this pure-C++ controller and has been deleted — `write_registry.py` now only emits C++ artifacts.

**Verification:** `python scripts/verify_cpp.py --hdf5 --rebuild` — 27/27 scenarios byte-identical (text and HDF5), HDF5 channel set matches text exactly, avrSWAP (39998, 85) identical row-for-row.

See `.github/prompts/plan-outputFileModernization.prompt.md` for full plan details and follow-up issue history.

---

## Future Work

- **Controller class refactor**: Wrap all static globals in a `RoscoController` class. Stage functions become methods. Prerequisite: migrate static locals (e.g. `static LPFilter` in shutdown.cpp, startup.cpp, speedsetpoints.cpp) into `LocalVariables` to enable multi-instance.
- **Simulink S-Function export**: Stage functions are `extern "C"` exportable. Create a MEX wrapper that maps Simulink mdlOutputs/mdlUpdate to the stage pipeline.
- **`discon_convert.py`**: Migration script to convert existing `DISCON.IN` files to TOML format for users who want to migrate without retuning.
