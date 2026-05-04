# ROSCO LabVIEW Interface for Phar Lap ETS

LabVIEW-callable DLL interface for the [ROSCO](https://github.com/NREL/ROSCO)
wind turbine controller, targeting a National Instruments PXIe-8133 running
**Phar Lap ETS 13.1** with **LabVIEW 2019 (32-bit)**.

## What This Is

ROSCO is an open-source wind turbine controller written in C++. This project
provides the build configuration, wrapper code, and instructions to compile
ROSCO into a single self-contained 32-bit DLL that runs on Phar Lap's minimal
Win32 environment — no UCRT, no modern MSVC runtime, no dynamic DLL loading.

## Target Environment

| Component | Value |
|-----------|-------|
| Hardware | NI PXIe-8133 |
| RTOS | Phar Lap ETS 13.1 |
| LabVIEW | 2019, 32-bit RT |
| Toolchain | winlibs MinGW i686, MSVCRT runtime |
| Linkage | Fully static (only KERNEL32.dll + msvcrt.dll at runtime) |

## Three-Stage Build Philosophy

The build is split into three stages so that each layer can be tested
independently before adding complexity:

1. **Stage 1 — hello_ping.dll**  
   A trivial DLL that returns `42`. Verifies the toolchain is correct, the
   DLL loads on Phar Lap, and LabVIEW can call it. Zero dependencies on ROSCO.

2. **Stage 2 — libdiscon.dll**  
   The full ROSCO controller built from source with the `ROSCO_PHARLAP` CMake
   option, which enables static linkage and disables features incompatible
   with Phar Lap (dynamic DLL loading, ZeroMQ).

3. **Stage 3 — discon_wrapper.dll**  
   A thin C shim linked against libdiscon that exposes a LabVIEW-friendly
   scalar interface (individual float inputs/outputs) instead of ROSCO's raw
   Bladed-style `avrSWAP` float array.

If any stage fails, you know exactly which layer is the problem.

## Project Files

| File | Purpose |
|------|---------|
| `hello_ping.c` | Stage 1 source — minimal DLL returning 42 |
| `discon_wrapper.c` | Stage 3 source — scalar LabVIEW interface wrapping ROSCO |
| `build_instructions.md` | **Start here** — step-by-step build guide for Windows |
| `rosco_audit.md` | Compatibility audit of ROSCO source for Phar Lap |
| `cmake_changes.md` | Proposed patches to ROSCO's CMake/source (apply before Stage 2) |
| `USFLOWT_10_DISCON.IN` | ROSCO controller configuration for USFLOWT turbine |
| `USFLOWT_10_Cp_Ct_Cq.txt` | Rotor performance tables (referenced by DISCON.IN) |
| `test_harness.c` | Desktop test harness (calls discon_wrapper without LabVIEW) |
| `Logs/` | Runtime log output directory |

## Getting Started

**→ See [build_instructions.md](build_instructions.md) for the full build guide.**

The instructions are written for someone with command-line experience but new to
MinGW cross-platform DLL builds. They include toolchain installation, exact build
commands, verification steps with pass/fail criteria, and deployment procedures.

## ROSCO Source

The ROSCO C++ source is at: `ROSCO-C/rosco/controller/`

Before building Stage 2, three files in the ROSCO source need patches
(documented in `cmake_changes.md`):
- `CMakeLists.txt` — add Phar Lap static build option
- `readconfigfiles.cpp` — remove `std::filesystem` dependency
- `extcontroller.cpp` — disable dynamic DLL loading for Phar Lap

## Exported Functions

The final `discon_wrapper.dll` exports:

| Function | Signature | Purpose |
|----------|-----------|---------|
| `hello_ping` | `int hello_ping(void)` | Returns 42 — deployment sanity check |
| `run_discon` | `void run_discon(int, float, float, float, float, float, float, float*, float*, float*, float*, float*, int*, char*)` | Calls ROSCO with scalar inputs, returns scalar outputs |
