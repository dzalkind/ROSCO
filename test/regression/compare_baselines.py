#!/usr/bin/env python3
"""
compare_baselines.py — What did a baseline change actually change?

A baseline diff is 40 MB of binary; `git diff` says only "differs". This says
which arrays moved, by how much, and from what time — the evidence a reviewer
needs to accept or reject the change.

Run it after `--update-baseline`, while the new files are in the working tree:

    python test/regression/compare_baselines.py                  # all changed
    python test/regression/compare_baselines.py --scenario 2
    python test/regression/compare_baselines.py --plots /tmp/bl  # before/after

It compares the working tree against `--against` (default HEAD), so it works
equally on a commit under review:

    python test/regression/compare_baselines.py --against HEAD~1

See "Changing a baseline on purpose" in README.md for the full procedure.
"""

import argparse
import io
import os
import re
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(HERE))
BASELINE_DIR = os.path.join(HERE, "baselines")
DT = 0.025


def _git_show(ref, relpath):
    r = subprocess.run(["git", "show", f"{ref}:{relpath}"],
                       cwd=REPO_ROOT, capture_output=True)
    return r.stdout if r.returncode == 0 else None


def changed_scenarios(ref):
    """Scenario numbers whose baseline file differs from `ref`."""
    r = subprocess.run(["git", "diff", "--name-only", ref, "--", "test/regression/baselines"],
                       cwd=REPO_ROOT, capture_output=True, text=True, check=True)
    nums = []
    for line in r.stdout.split():
        m = re.search(r"scenario_(\d+)\.npz$", line)
        if m:
            nums.append(int(m.group(1)))
    return sorted(nums)


def compare(num, ref):
    """(status, rows) for one scenario. rows: (array, max_abs, frac_of_peak, n_diff, first_t)."""
    relpath = f"test/regression/baselines/scenario_{num}.npz"
    path = os.path.join(REPO_ROOT, relpath)
    if not os.path.exists(path):
        return "missing in working tree", []
    blob = _git_show(ref, relpath)
    if blob is None:
        return f"new file (not in {ref})", []

    before, after = np.load(io.BytesIO(blob)), np.load(path)
    rows = []
    for key in sorted(set(before.files) | set(after.files)):
        if key not in before.files:
            rows.append((key, None, None, None, None, "added"))
            continue
        if key not in after.files:
            rows.append((key, None, None, None, None, "removed"))
            continue
        b, a = before[key], after[key]
        if len(b) != len(a):
            rows.append((key, None, None, None, None, f"length {len(b)} -> {len(a)}"))
            continue
        if np.array_equal(b, a):
            continue
        diff = np.abs(a - b)
        # Relative to the signal's own peak, not pointwise: a 1e-9 change where
        # the old value is 0 is not an infinite error.
        peak = float(np.abs(b).max())
        idx = int(np.argmax(diff > 0))
        rows.append((key, float(diff.max()), float(diff.max() / peak) if peak else float("nan"),
                     int(np.count_nonzero(diff)), idx * DT, ""))
    return ("identical" if not rows else ""), rows


def plot(num, ref, out_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    relpath = f"test/regression/baselines/scenario_{num}.npz"
    blob = _git_show(ref, relpath)
    if blob is None:
        return None
    before = np.load(io.BytesIO(blob))
    after = np.load(os.path.join(REPO_ROOT, relpath))
    keys = [k for k in sorted(before.files)
            if k not in ("t", "ws") and k in after.files
            and len(before[k]) == len(after[k]) and not np.array_equal(before[k], after[k])]
    if not keys:
        return None

    t = after["t"] if "t" in after.files else np.arange(len(after[keys[0]])) * DT
    fig, axes = plt.subplots(len(keys), 2, figsize=(13, 2.2 * len(keys)), squeeze=False)
    for row, key in enumerate(keys):
        axes[row][0].plot(t, before[key], lw=1.0, label=f"{ref}", color="steelblue")
        axes[row][0].plot(t, after[key], lw=1.0, label="new", color="darkorange", alpha=0.8)
        axes[row][0].set_ylabel(key)
        axes[row][0].legend(fontsize=7, loc="upper left")
        axes[row][1].plot(t, after[key] - before[key], lw=1.0, color="crimson")
        axes[row][1].set_ylabel("new - old")
        for ax in axes[row]:
            ax.grid(True, alpha=0.3)
    axes[-1][0].set_xlabel("Time [s]")
    axes[-1][1].set_xlabel("Time [s]")
    fig.suptitle(f"Scenario {num}: baseline change vs {ref}", fontweight="bold")
    fig.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f"scenario_{num}_baseline_change.png")
    fig.savefig(path, dpi=120, bbox_inches="tight")
    plt.close(fig)
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--scenario", type=int, nargs="+", default=[],
                        help="Scenario number(s). Default: every baseline that differs.")
    parser.add_argument("--against", default="HEAD", help="Git ref to compare with (default HEAD).")
    parser.add_argument("--plots", metavar="DIR", help="Also write before/after + difference plots.")
    args = parser.parse_args()

    scenarios = args.scenario or changed_scenarios(args.against)
    if not scenarios:
        print(f"No baseline differs from {args.against}.")
        return 0

    sha = subprocess.run(["git", "rev-parse", "--short", args.against],
                         cwd=REPO_ROOT, capture_output=True, text=True).stdout.strip()
    print(f"Comparing working tree against {args.against} ({sha})")
    print()
    for num in scenarios:
        status, rows = compare(num, args.against)
        print(f"scenario {num}: {status}" if status else f"scenario {num}:")
        for key, max_abs, frac, n_diff, first_t, note in rows:
            if note:
                print(f"    {key:16s} {note}")
            else:
                rel = "was all zero" if frac != frac else f"{frac:.3%} of peak"
                print(f"    {key:16s} max |Δ| {max_abs:.6g} ({rel})   "
                      f"{n_diff:,} samples   from t = {first_t:.3f} s")
        if args.plots:
            path = plot(num, args.against, args.plots)
            if path:
                print(f"    plot: {path}")
        print()

    print("Every row above must be explainable by the change you made. If one is not,")
    print("you have found a bug, not a stale baseline — see README.md.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
