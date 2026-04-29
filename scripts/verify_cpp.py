#!/usr/bin/env python3
"""
verify_cpp.py — Run all 27 vit_sim scenarios against frozen baseline_arrays.

Each scenario runs in a separate subprocess (required to reset the C++ DLL's
static variables between scenarios — same isolation that Docker exec provided
during the VIT translation workflow).

Usage:
    python3 scripts/verify_cpp.py              # all 27 scenarios
    python3 scripts/verify_cpp.py --scenario 1 # single scenario
    python3 scripts/verify_cpp.py --rebuild     # cmake build before running

Expected result: ALL IDENTICAL
"""

import argparse
import os
import subprocess
import sys
import tempfile

import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXAMPLES_DIR = os.path.join(REPO_ROOT, "Examples")
BASELINE_DIR = os.path.join(REPO_ROOT, "baseline_arrays")
DEFAULT_BUILD_DIR = os.path.join(REPO_ROOT, "rosco", "controller", "build")
CONTROLLER_DIR = os.path.join(REPO_ROOT, "rosco", "controller")
LIB_DIR = os.path.join(REPO_ROOT, "rosco", "lib")
SCRUB_SRC = os.path.join(CONTROLLER_DIR, "src", "scrub_stack.c")
SCRUB_LIB = os.path.join(LIB_DIR, "libscrub.so")

ALL_SCENARIOS = list(range(1, 28))


def build_discon(build_dir, preset=None):
    """cmake --build the controller and copy libdiscon to rosco/lib."""
    if preset:
        # Configure with preset first
        print(f"Configuring with preset '{preset}'...")
        r = subprocess.run(
            ["cmake", "--preset", preset],
            cwd=CONTROLLER_DIR,
            capture_output=False,
        )
        if r.returncode != 0:
            print("ERROR: cmake configure failed.", file=sys.stderr)
            sys.exit(1)

    print("Building libdiscon...")
    r = subprocess.run(
        ["cmake", "--build", build_dir],
        capture_output=False,
    )
    if r.returncode != 0:
        print("ERROR: cmake build failed.", file=sys.stderr)
        sys.exit(1)

    # Copy built library to rosco/lib (where rosco.discon_lib_path points)
    import glob
    libs = glob.glob(os.path.join(build_dir, "libdiscon.*"))
    if not libs:
        print("ERROR: no libdiscon.* found in build dir.", file=sys.stderr)
        sys.exit(1)
    import shutil
    for lib in libs:
        dst = os.path.join(LIB_DIR, os.path.basename(lib))
        shutil.copy2(lib, dst)
        print(f"  Copied {os.path.basename(lib)} → rosco/lib/")
    print()


def build_scrub():
    """Build libscrub.so if missing (needed for Scenario 3 determinism)."""
    if os.path.exists(SCRUB_LIB):
        return
    if not os.path.exists(SCRUB_SRC):
        print("  Warning: scrub_stack.c not found, skipping libscrub build.")
        print("  Scenario 3 may be non-deterministic.")
        return
    print("Building libscrub.so for Scenario 3 determinism...")
    r = subprocess.run(
        ["gcc", "-shared", "-fPIC", "-o", SCRUB_LIB, SCRUB_SRC],
        capture_output=True, text=True,
    )
    if r.returncode != 0:
        print(f"  Warning: libscrub build failed: {r.stderr.strip()}")
        print("  Scenario 3 may be non-deterministic.")
    else:
        print(f"  Built {SCRUB_LIB}")
    print()


def run_scenario(scenario_num, output_dir, asan_env=None):
    """Run a single scenario in a subprocess. Returns True on success."""
    env = None
    if asan_env:
        env = os.environ.copy()
        env.update(asan_env)
    r = subprocess.run(
        [sys.executable, "vit_sim.py",
         "--scenario", str(scenario_num),
         "--output-dir", output_dir],
        cwd=EXAMPLES_DIR,
        capture_output=True,
        text=True,
        env=env,
    )
    if r.returncode != 0:
        print(f"  scenario_{scenario_num}: SUBPROCESS FAILED")
        print(r.stderr[-2000:] if r.stderr else "(no stderr)")
        return False
    return True


def compare_scenario(scenario_num, output_dir):
    """Compare scenario output against baseline. Returns (identical, details)."""
    baseline_path = os.path.join(BASELINE_DIR, f"scenario_{scenario_num}.npz")
    output_path = os.path.join(output_dir, f"scenario_{scenario_num}.npz")

    if not os.path.exists(baseline_path):
        return False, f"no baseline file at {baseline_path}"
    if not os.path.exists(output_path):
        return False, "output file not written"

    b = np.load(baseline_path)
    o = np.load(output_path)

    if set(b.files) != set(o.files):
        return False, f"key mismatch: baseline={sorted(b.files)} output={sorted(o.files)}"

    mismatches = []
    for key in sorted(b.files):
        if len(b[key]) != len(o[key]):
            mismatches.append(f"{key}: length {len(b[key])} vs {len(o[key])}")
        elif not np.array_equal(b[key], o[key]):
            diff = np.abs(b[key] - o[key])
            first = int(np.where(b[key] != o[key])[0][0])
            mismatches.append(
                f"{key}: max_diff={diff.max():.2e} first_diff_idx={first}"
            )

    if mismatches:
        return False, "; ".join(mismatches)
    return True, f"{sum(len(b[k]) for k in b.files)} values identical"


def main():
    parser = argparse.ArgumentParser(description="Verify C++ controller against frozen baselines")
    parser.add_argument("--scenario", type=int, default=0,
                        help="Run single scenario (1-27). Default 0 = all.")
    parser.add_argument("--rebuild", action="store_true",
                        help="Run cmake --build before verifying.")
    parser.add_argument("--update-baseline", action="store_true",
                        help="Overwrite baseline_arrays/ with current outputs instead of comparing.")
    parser.add_argument("--preset", type=str, default=None,
                        help="CMake preset name (e.g. 'asan'). Sets build dir to build-{preset}.")
    args = parser.parse_args()

    # Resolve build directory from preset
    if args.preset:
        build_dir = os.path.join(CONTROLLER_DIR, f"build-{args.preset}")
    else:
        build_dir = DEFAULT_BUILD_DIR

    if not os.path.exists(BASELINE_DIR):
        print(f"ERROR: baseline_arrays/ not found at {BASELINE_DIR}", file=sys.stderr)
        sys.exit(1)

    if args.rebuild:
        build_discon(build_dir, preset=args.preset)

    build_scrub()

    # On macOS, ASan-instrumented shared libraries loaded via dlopen need
    # DYLD_INSERT_LIBRARIES pointing to the ASan runtime.
    asan_env = None
    if args.preset and "asan" in args.preset:
        import glob as globmod
        asan_libs = globmod.glob("/Library/Developer/CommandLineTools/usr/lib/clang/*/lib/darwin/libclang_rt.asan_osx_dynamic.dylib")
        if asan_libs:
            asan_env = {"DYLD_INSERT_LIBRARIES": asan_libs[-1]}
            print(f"ASan runtime: {asan_libs[-1]}")
        else:
            print("WARNING: Could not find ASan runtime library. ASan may not work.")
        print()

    scenarios = [args.scenario] if args.scenario > 0 else ALL_SCENARIOS

    print(f"Running {len(scenarios)} scenario(s) — each in a separate subprocess")
    print(f"Baseline: {BASELINE_DIR}")
    print()

    if args.update_baseline:
        print(f"Updating baseline_arrays/ with current outputs...")
        print()
        os.makedirs(BASELINE_DIR, exist_ok=True)
        for s in scenarios:
            sys.stdout.write(f"  scenario_{s:2d}: running... ")
            sys.stdout.flush()
            ok = run_scenario(s, BASELINE_DIR, asan_env=asan_env)
            print("saved" if ok else "FAILED")
        print()
        print("Baseline updated. Commit baseline_arrays/ to lock in the new reference.")
        return

    with tempfile.TemporaryDirectory(prefix="rosco_verify_") as tmpdir:
        results = {}

        for s in scenarios:
            sys.stdout.write(f"  scenario_{s:2d}: running... ")
            sys.stdout.flush()
            ok = run_scenario(s, tmpdir, asan_env=asan_env)
            if not ok:
                results[s] = (False, "subprocess error")
                print("FAIL (subprocess)")
                continue
            identical, detail = compare_scenario(s, tmpdir)
            results[s] = (identical, detail)
            status = "IDENTICAL" if identical else "MISMATCH"
            print(f"{status}  ({detail})")

        print()
        print("=" * 60)
        passed = sum(1 for ok, _ in results.values() if ok)
        total = len(results)
        if passed == total:
            total_vals = 0
            for s in scenarios:
                bp = os.path.join(BASELINE_DIR, f"scenario_{s}.npz")
                if os.path.exists(bp):
                    b = np.load(bp)
                    total_vals += sum(len(b[k]) for k in b.files)
            print(f"RESULT: ALL IDENTICAL — {total_vals:,} total float64 values compared")
        else:
            print(f"RESULT: {passed}/{total} scenarios identical — DIFFERENCES FOUND")
            for s, (ok, detail) in results.items():
                if not ok:
                    print(f"  scenario_{s}: {detail}")
            sys.exit(1)


if __name__ == "__main__":
    main()
