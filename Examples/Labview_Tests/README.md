# Labview_Tests_DZ

Standalone C test harness for the ROSCO DISCON controller, used to verify the
C-shim / Fortran-library call chain without LabVIEW.

## Architecture

```
test_harness  ──dlopen──►  discon_wrapper.so/.dll  ──dlopen──►  libdiscon.dylib/.dll
     ↕                              ↕
LabVIEW                    same DLL used in production
```

`discon_wrapper` is a thin C shim that translates a simple scalar interface into
the Bladed-style `avrSWAP` array expected by the Fortran DISCON library.
LabVIEW calls `discon_wrapper`; the harness does the same thing without LabVIEW.

## Prerequisites

- `USFLOWT_10_DISCON.IN` must be present in the working directory (already here)
- `USFLOWT_10_Cp_Ct_Cq.txt` must be present in the working directory (already here)
- `libdiscon.dylib` (Unix) or `libdiscon.dll` (Windows) must be on the library
  search path — the path is hardcoded in `discon_wrapper.c` (`LIB_NAME`)
- `Logs/` directory must exist: `mkdir -p Logs`

---

## Compile and run — Unix (macOS / Linux)

```bash
# From this directory
mkdir -p Logs

# Build the C shim shared library
clang -shared -fPIC discon_wrapper.c -o discon_wrapper.so

# Build the test harness
gcc test_harness.c -o test_harness -ldl

# Run
./test_harness
```

---

## Compile and run — Windows (MinGW)

```bat
rem From this directory
mkdir Logs

rem Build the C shim DLL
gcc -shared discon_wrapper.c -o discon_wrapper.dll

rem Build the test harness
gcc test_harness.c -o test_harness.exe

rem libdiscon.dll must be in this directory or on PATH
test_harness.exe
```

---

## Expected output

```
Init:  aviFAIL=0  msg=
Run:   aviFAIL=0  msg=
  gen_torque = <non-zero value> N·m
  pitch1     = 0.0000 rad
  pitch2     = 0.0000 rad
  pitch3     = 0.0000 rad
  yaw_rate   = 0.0000 rad/s
```

### What to look for

| Check | Good | Bad |
|---|---|---|
| `aviFAIL` | `0` on both calls | Non-zero — see `avcMSG` and `Logs/discon_log.txt` |
| `avcMSG` | Empty string | Any text indicates a controller warning or error |
| `gen_torque` | Non-zero (rated ~43 kN·m for NREL 5MW) | `0.0` suggests init didn't propagate |
| `pitch1/2/3` | Near `0.0` at below-rated wind (10 m/s) | Large values suggest bad inputs |
| `Logs/discon_log.txt` | Not created | Created only on `aviFAIL != 0` |
| `rosco_test.RO.dbg*` | Created after run | Absent means DISCON never executed |

---

## File descriptions

| File | Purpose |
|---|---|
| `discon_wrapper.c` | C shim — wraps DISCON in a LabVIEW-friendly scalar interface |
| `test_harness.c` | Standalone harness — loads the shim and calls `run_discon()` |
| `hello_world.c` / `hello_fortran.F90` | Earlier proof-of-concept for C↔Fortran interop via LabVIEW |
| `USFLOWT_10_DISCON.IN` | ROSCO controller parameter file for the USFLOWT turbine |
| `USFLOWT_10_Cp_Ct_Cq.txt` | Rotor performance tables referenced by `DISCON.IN` |
| `Logs/` | Runtime log output directory |
