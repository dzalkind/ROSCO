#!/usr/bin/env python3
"""
run_regression.py — Run all 27 scenarios against the frozen baselines.

Each scenario runs in a separate subprocess, because the controller DLL keeps
static state that is only reset by unloading the process.

Usage:
    python3 test/regression/run_regression.py              # all 27 scenarios
    python3 test/regression/run_regression.py --scenario 1 # single scenario
    python3 test/regression/run_regression.py --rebuild    # cmake build first

Expected result: ALL IDENTICAL

`pytest test/regression` runs the same comparison, one test per scenario.
"""

import argparse
import hashlib
import json
import os
import platform
import subprocess
import sys
import tempfile
from datetime import datetime, timezone

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(HERE))
SCENARIOS = os.path.join(HERE, "scenarios.py")
BASELINE_DIR = os.path.join(HERE, "baselines")
PROVENANCE = os.path.join(BASELINE_DIR, "PROVENANCE.json")
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
        print(f"Configuring with preset '{preset}'...", flush=True)
        cmd = ["cmake", "--preset", preset]
    elif not os.path.exists(os.path.join(build_dir, "CMakeCache.txt")):
        print(f"Configuring {build_dir}...", flush=True)
        cmd = ["cmake", "-S", CONTROLLER_DIR, "-B", build_dir]
    else:
        cmd = None

    if cmd:
        r = subprocess.run(cmd, cwd=CONTROLLER_DIR, capture_output=False)
        if r.returncode != 0:
            print("ERROR: cmake configure failed.", file=sys.stderr)
            sys.exit(1)

    print("Building libdiscon...", flush=True)
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


def run_scenario(scenario_num, output_dir, work_dir=None, asan_env=None):
    """Run a single scenario in a subprocess. Returns True on success.

    `output_dir` receives the scenario_N.npz arrays. `work_dir` (default:
    `output_dir`) is the subprocess cwd, and so receives the generated
    DISCON_*.IN and the controller's *.RO.dbg* / *.RO.h5 output.
    """
    env = None
    if asan_env:
        env = os.environ.copy()
        env.update(asan_env)
    work_dir = work_dir or output_dir
    os.makedirs(work_dir, exist_ok=True)
    r = subprocess.run(
        [sys.executable, SCENARIOS,
         "--scenario", str(scenario_num),
         "--output-dir", output_dir],
        cwd=work_dir,
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


def compare_hdf5_debug(work_dir):
    """Compare Scenario 28 (.RO.h5) debug output against Scenario 1 (.RO.dbg)
    text output — same simulation, two OutputFormat values. Both scenarios must
    already have been run with `work_dir` as their working directory.

    Also verifies the "/avrSWAP" HDF5 dataset (written only at LoggingLevel=3,
    which Scenario 28 sets) against the ground-truth avrSWAP values captured
    directly from the Python sim loop (scenario_28.npz's 'avrSWAP_full').
    """
    text_path = os.path.join(work_dir, "regression_1.RO.dbg")
    h5_path = os.path.join(work_dir, "regression_28.RO.h5")
    if not os.path.exists(h5_path):
        return None, "regression_28.RO.h5 not found (HDF5 support may not be compiled in)"
    if not os.path.exists(text_path):
        return False, "regression_1.RO.dbg not found (run scenario 1 first)"

    sys.path.insert(0, os.path.join(REPO_ROOT, "rosco"))
    from toolbox.ofTools.fast_io.output_processing import load_ascii_output, load_hdf5_output

    text_data, text_info = load_ascii_output(text_path)
    h5_data, h5_info = load_hdf5_output(h5_path)
    text_channels = dict(zip(text_info["channels"], text_data.T))
    h5_channels = dict(zip(h5_info["channels"], h5_data.T))

    if set(text_channels) != set(h5_channels):
        only_text = sorted(set(text_channels) - set(h5_channels))
        only_h5 = sorted(set(h5_channels) - set(text_channels))
        return False, f"channel set mismatch: only in text={only_text} only in h5={only_h5}"

    mismatches = []
    for key in sorted(text_channels):
        t, h = text_channels[key], h5_channels[key]
        if len(t) != len(h):
            mismatches.append(f"{key}: length {len(t)} vs {len(h)}")
        # text .dbg uses "%20.5E" (6 significant figures); allow for that rounding
        elif not np.allclose(t, h, rtol=2e-5, atol=1e-9):
            mismatches.append(f"{key}: max_diff={np.abs(t - h).max():.2e}")

    if mismatches:
        return False, "; ".join(mismatches)

    # --- avrSWAP dataset verification (LoggingLevel=3) ---
    avr_ok, avr_detail = compare_hdf5_avrswap(h5_path, work_dir)
    if not avr_ok:
        return False, avr_detail

    return True, f"{len(text_channels)} channels identical (text vs HDF5); {avr_detail}"


def compare_hdf5_avrswap(h5_path, npz_dir):
    """Verify the "/avrSWAP" dataset in an .RO.h5 file: presence, column
    labels, shape, and values against the Python-captured ground truth
    (scenario_28.npz's 'avrSWAP_full', saved by run_scenario_28)."""
    npz_path = os.path.join(npz_dir, "scenario_28.npz")
    if not os.path.exists(npz_path):
        return False, "scenario_28.npz not found (run scenario 28 first)"

    import h5py
    with h5py.File(h5_path, "r") as f:
        if "avrSWAP" not in f:
            return False, "'/avrSWAP' dataset missing from .RO.h5 (Phase 3 not wired up)"
        avr_h5 = f["avrSWAP"][:]
        labels_attr = f["avrSWAP"].attrs.get("column_labels")
        labels = [l.decode() if isinstance(l, bytes) else l for l in labels_attr] \
            if labels_attr is not None else []

    expected_labels = [f"AvrSWAP({i + 1})" for i in range(85)]
    if labels[:85] != expected_labels:
        return False, f"avrSWAP column_labels mismatch: got {labels[:5]}... expected {expected_labels[:5]}..."

    avr_truth = np.load(npz_path)["avrSWAP_full"]
    # sim_ws_series `continue`s at i=0 (no controller call there), but the
    # HDF5 writer's row 0 is the controller's own init call (e.g. MSG/INFILE
    # length setup) that happens before Python's tracked loop even starts —
    # it has no Python ground truth. h5 row i (i>=1) otherwise aligns exactly
    # with avrSWAP_full row i.
    if avr_h5.shape[0] != avr_truth.shape[0] - 1:
        return False, f"avrSWAP row count mismatch: h5={avr_h5.shape[0]} truth={avr_truth.shape[0] - 1}"
    n = avr_h5.shape[0]
    avr_h5, avr_truth = avr_h5[1:], avr_truth[1:n]
    if avr_h5.shape != avr_truth.shape:
        return False, f"avrSWAP shape mismatch: h5={avr_h5.shape} truth={avr_truth.shape}"
    if not np.allclose(avr_h5, avr_truth, rtol=2e-5, atol=1e-9):
        return False, f"avrSWAP value mismatch: max_diff={np.abs(avr_h5 - avr_truth).max():.2e}"

    return True, f"avrSWAP {avr_h5.shape} identical (labels + values)"


def _git(*args):
    try:
        r = subprocess.run(["git", *args], cwd=REPO_ROOT, capture_output=True, text=True)
        return r.stdout.strip() if r.returncode == 0 else "unknown"
    except OSError:
        return "unknown"


def _discon_hash():
    for name in ("libdiscon.so", "libdiscon.dylib", "libdiscon.dll"):
        path = os.path.join(LIB_DIR, name)
        if os.path.exists(path):
            with open(path, "rb") as f:
                return f"{name}:sha256:{hashlib.sha256(f.read()).hexdigest()[:16]}"
    return "unknown"


def write_provenance():
    """Record what the baselines were generated from, so the question is answerable
    without git archaeology."""
    import scipy
    data = {
        "generated_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "git_sha": _git("rev-parse", "HEAD"),
        "git_dirty": bool(_git("status", "--porcelain")),
        "platform": platform.platform(),
        "machine": platform.machine(),
        "python": platform.python_version(),
        "numpy": np.__version__,
        "scipy": scipy.__version__,
        "libdiscon": _discon_hash(),
    }
    with open(PROVENANCE, "w") as f:
        json.dump(data, f, indent=2, sort_keys=True)
        f.write("\n")
    return data


def read_provenance():
    try:
        with open(PROVENANCE) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def main():
    parser = argparse.ArgumentParser(description="Verify C++ controller against frozen baselines")
    parser.add_argument("--scenario", type=int, default=0,
                        help="Run single scenario (1-27). Default 0 = all.")
    parser.add_argument("--rebuild", action="store_true",
                        help="Run cmake --build before verifying.")
    parser.add_argument("--update-baseline", action="store_true",
                        help="Overwrite baselines/ with current outputs instead of comparing.")
    parser.add_argument("--preset", type=str, default=None,
                        help="CMake preset name (e.g. 'asan'). Sets build dir to build-{preset}.")
    parser.add_argument("--hdf5", action="store_true",
                        help="Also run Scenario 28 (HDF5 OutputFormat) and compare its .RO.h5 "
                             "debug output against Scenario 1's .RO.dbg text output.")
    args = parser.parse_args()

    # Resolve build directory from preset
    if args.preset:
        build_dir = os.path.join(CONTROLLER_DIR, f"build-{args.preset}")
    else:
        build_dir = DEFAULT_BUILD_DIR

    if not os.path.exists(BASELINE_DIR):
        print(f"ERROR: baselines/ not found at {BASELINE_DIR}", file=sys.stderr)
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
    prov = read_provenance()
    if prov:
        dirty = " (dirty tree)" if prov.get("git_dirty") else ""
        print(f"  captured {prov['generated_utc']} from {prov['git_sha'][:12]}{dirty}")
        print(f"  on {prov['platform']} / numpy {prov['numpy']} / scipy {prov['scipy']}")
    else:
        print("  (no PROVENANCE.json — origin unknown)")
    print()

    if args.update_baseline:
        print("Updating baselines/ with current outputs...")
        print()
        os.makedirs(BASELINE_DIR, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix="rosco_regression_") as workdir:
            for s in scenarios:
                sys.stdout.write(f"  scenario_{s:2d}: running... ")
                sys.stdout.flush()
                ok = run_scenario(s, BASELINE_DIR, work_dir=workdir, asan_env=asan_env)
                print("saved" if ok else "FAILED")
        prov = write_provenance()
        print()
        print(f"Provenance: {prov['git_sha'][:12]} on {prov['platform']}")
        print("Baseline updated. Commit test/regression/baselines/ to lock in the new reference.")
        return

    with tempfile.TemporaryDirectory(prefix="rosco_regression_") as tmpdir:
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

        if args.hdf5:
            print()
            print("Running Scenario 28 (HDF5 OutputFormat) for text/HDF5 comparison...")
            if 1 not in scenarios:
                run_scenario(1, tmpdir, asan_env=asan_env)
            ok28 = run_scenario(28, tmpdir, asan_env=asan_env)
            if not ok28:
                print("  Scenario 28: SUBPROCESS FAILED")
                sys.exit(1)
            identical, detail = compare_hdf5_debug(tmpdir)
            if identical is None:
                print(f"  SKIPPED: {detail}")
            else:
                status = "IDENTICAL" if identical else "MISMATCH"
                print(f"  {status}  ({detail})")
                if not identical:
                    sys.exit(1)


if __name__ == "__main__":
    main()
