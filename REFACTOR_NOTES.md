# ROSCO-C Refactoring Notes

## Overview

This branch (`labview`) contains a modernized pure C++ version of ROSCO, translated from Fortran by the VIT tool and subsequently refactored. The Fortran source has been removed entirely. This is intended for a future v3.0 release; `master` is unchanged for existing users.

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
- `rosco/controller/src/rosco_types_io.cpp` — TOML loader + `populate_view()` bridge to the legacy C struct
- `Examples/DISCON_template.toml` — annotated TOML template with all parameters and defaults

**To add a new parameter:** edit `rosco_types.yaml` and re-run the generator. The struct, parser, bridge, and template all update automatically.

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
rosco_types.yaml          ← edit this to add/change parameters
       │
write_registry.py         ← run this to regenerate
       │
       ├── rosco_types.hpp         ← ControlParameters struct (generated)
       ├── rosco_types_io.cpp      ← TOML loader + populate_view() (generated)
       └── DISCON_template.toml   ← annotated input template (generated)

discon.cpp
  ├── is_toml_file()              ← branches on .toml extension
  ├── TOML path: CntrParOwner.load_from_toml() → populate_view(&CntrPar)
  ├── DISCON.IN path: existing two-pass parser (unchanged)
  └── try/catch boundary          ← LabVIEW compatibility
```

---

## Phase 2: Filter and Integrator State Refactor

The flat `[1024]` parallel arrays in `filterparameters_t`, `piparams_t`, `resparams_t`, and `rlparams_t` have been replaced with `std::vector<PerInstanceState>`.

**Before:**
```c
// 47 separate double[1024] arrays — ~385 KB of zeroed static state
typedef struct {
    double lpf1_a1[1024];
    double lpf1_a0[1024];
    double lpf1_InputSignalLast[1024];
    double lpf1_OutputSignalLast[1024];
    // ... 43 more
} filterparameters_t;
```

**After:**
```cpp
// One struct per instance, one vector per filter type
struct filterparameters_t {
    std::vector<LPF1State> lpf1;   // grows to actual usage (~31 instances)
    std::vector<LPF2State> lpf2;
    std::vector<HPFState>  hpf;
    // ...
};
```

Each instance now has all its fields grouped in one struct (`s.a1`, `s.output_last`) instead of spread across a dozen parallel arrays. Vectors grow on first use and stay sized to actual instance count.

All function signatures are unchanged — callers still pass `&LocalVar->FP`, `&LocalVar->piP` etc. Only the 10 filter/controller function bodies were updated.

**Restart file format**: the checkpoint binary format changed (previously wrote fixed 1024-element arrays, now writes count + N elements). Old `.chkp` files from prior builds are not compatible.

State structs are defined in `src/include/rosco_objects.hpp`.

---

## Future Work

- **`discon_convert.py`**: Migration script to convert existing `DISCON.IN` files to TOML format for users who want to migrate without retuning.
