#!/usr/bin/env python3
"""
mode_coverage.py — Which controller modes does the regression suite exercise?

Every ROSCO feature is switched on by a mode parameter in the DISCON file
(`PC_ControlMode`, `AWC_Mode`, ...). This tabulates, for each value of each
mode, which regression scenarios and which Examples configure it:

    python test/regression/mode_coverage.py            # markdown report
    python test/regression/mode_coverage.py --gaps     # only what nothing covers

Sources, in order:
  - regression   test/regression/fixtures/scenario_NN.IN (committed)
  - test case    Examples/Test_Cases/**/*DISCON*.IN (committed)
  - example      Examples/examples_out/**/*DISCON*.IN — generated when an Example
                 is run, gitignored, so this column reflects whichever Examples
                 have been run on *this* machine. Absent if none have.

This says what is *configured*, not what *executed*: a scenario can set a mode
whose code path is never reached in its 1000 s run. Line coverage (gcovr) is
the complement.
"""

import argparse
import glob
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(HERE))
FIXTURES_DIR = os.path.join(HERE, "fixtures")
EXAMPLES_DIR = os.path.join(REPO_ROOT, "Examples")
REGISTRY = os.path.join(REPO_ROOT, "rosco", "controller", "rosco_registry", "rosco_types.yaml")


def _always(p):
    return True


# name -> (values that select distinct behaviour, when the setting takes effect).
#
# Values come from the range checks in checkinputs.cpp where one exists, and
# otherwise from the comparisons the controller source makes on the parameter.
# A setting whose parent mode is off is not counted as covering anything —
# IPC_SatMode = 2 in a file with IPC off tests nothing.
MODES = {
    "LoggingLevel":    ([0, 1, 2, 3], _always),
    "OutputFormat":    ([0, 1], lambda p: p.get("LoggingLevel", 0) > 0),
    "Ext_Interface":   ([0, 1], _always),
    "F_LPFType":       ([1, 2], _always),
    "IPC_ControlMode": ([0, 1, 2], _always),
    "IPC_SatMode":     ([0, 1, 2, 3], lambda p: p.get("IPC_ControlMode", 0) > 0),
    "VS_ControlMode":  ([0, 1, 2, 3, 4], _always),
    "VS_ConstPower":   ([0, 1], _always),
    "VS_FBP":          ([0, 1, 2, 3], _always),
    "PC_ControlMode":  ([0, 1], _always),
    "Y_ControlMode":   ([0, 1, 2], _always),
    "SS_Mode":         ([0, 1], _always),
    "PRC_Mode":        ([0, 1, 2], _always),
    "PRC_Comm":        ([0, 1, 2], lambda p: p.get("PRC_Mode", 0) == 2),
    "WE_Mode":         ([0, 1, 2], _always),
    # checkinputs accepts 0-3, but the controller only tests PS_Mode > 0.
    "PS_Mode":         ([0, 1], _always),
    "SU_Mode":         ([0, 1], _always),
    "SD_Mode":         ([0, 1], _always),
    "Fl_Mode":         ([0, 1, 2], _always),
    "TD_Mode":         ([0, 1], _always),
    "TRA_Mode":        ([0, 1], _always),
    "Flp_Mode":        ([0, 1, 2, 3], _always),
    "OL_Mode":         ([0, 1, 2], _always),
    "OL_BP_Mode":      ([0, 1], lambda p: p.get("OL_Mode", 0) > 0),
    "PA_Mode":         ([0, 1, 2], _always),
    "PF_Mode":         ([0, 1, 2], _always),
    "AWC_Mode":        ([0, 1, 2, 3, 4, 5], _always),
    "Ext_Mode":        ([0, 1], _always),
    "ZMQ_Mode":        ([0, 1], _always),
    "CC_Mode":         ([0, 1, 2], _always),
    "StC_Mode":        ([0, 1, 2], _always),
}

_LINE = re.compile(r"^\s*(-?\d+)\s+!\s*([A-Za-z_][A-Za-z0-9_]*)")


def read_modes(path):
    """Integer settings of a DISCON file, keyed by name. Tolerant of any DISCON
    version: a parameter the file does not have is simply absent."""
    found = {}
    with open(path, errors="replace") as f:
        for line in f:
            m = _LINE.match(line)
            if m and m.group(2) in MODES:
                found[m.group(2)] = int(m.group(1))
    return found


def registry_modes():
    """Every mode-like parameter the controller registry declares."""
    import yaml
    with open(REGISTRY) as f:
        params = yaml.safe_load(f)["ControlParameters"]
    return {k for k in params if k.endswith("Mode")}


def _label(path, kind):
    """Scenario number, test-case directory, or the Example's output name —
    several files from one Example collapse to one label."""
    if kind == "regression":
        return str(int(re.search(r"scenario_(\d+)", path).group(1)))
    if kind == "test case":
        return os.path.basename(os.path.dirname(path))
    rel = os.path.relpath(path, os.path.join(EXAMPLES_DIR, "examples_out"))
    return rel.split(os.sep)[0] if os.sep in rel else rel.replace("_DISCON.IN", "")


def collect_sources():
    """[(kind, label, settings)] for every DISCON file found."""
    groups = [
        ("regression", sorted(glob.glob(os.path.join(FIXTURES_DIR, "scenario_*.IN")))),
        ("test case", sorted(glob.glob(os.path.join(EXAMPLES_DIR, "Test_Cases", "**", "*DISCON*.IN"),
                                       recursive=True))),
        ("example", sorted(glob.glob(os.path.join(EXAMPLES_DIR, "examples_out", "**", "*DISCON*.IN"),
                                     recursive=True))),
    ]
    sources = []
    for kind, paths in groups:
        for path in paths:
            settings = read_modes(path)
            if settings:  # skip non-ROSCO files, e.g. the DTU controller's DISCON.IN
                sources.append((kind, _label(path, kind), settings))
    return sources


def tabulate(sources):
    """{(mode, value): {kind: [labels]}}, counting a setting only where it takes
    effect. Values outside the known domain are kept, so they show up."""
    table = {}
    for mode, (values, _) in MODES.items():
        for v in values:
            table[(mode, v)] = {}
    for kind, label, settings in sources:
        for mode, value in settings.items():
            _, active = MODES[mode]
            if not active(settings):
                continue
            if mode == "PS_Mode":
                value = min(value, 1)
            labels = table.setdefault((mode, value), {}).setdefault(kind, [])
            if label not in labels:
                labels.append(label)
    return table


def _fmt(labels, limit=8):
    if not labels:
        return ""
    shown = ", ".join(labels[:limit])
    return shown + (f" (+{len(labels) - limit})" if len(labels) > limit else "")


def report(sources, gaps_only=False):
    table = tabulate(sources)
    kinds = [k for k in ("regression", "test case", "example") if any(s[0] == k for s in sources)]
    lines = []
    counts = {k: sum(1 for s in sources if s[0] == k) for k in kinds}
    lines.append("Sources: " + ", ".join(f"{counts[k]} {k}" for k in kinds))
    if "example" not in kinds:
        lines.append("(no Examples/examples_out/ DISCON files — run the Examples to include them)")
    lines.append("")

    header = ["Mode", "Value", "Regression scenarios"] + [k.capitalize() for k in kinds if k != "regression"]
    lines.append("| " + " | ".join(header) + " |")
    lines.append("|" + "---|" * len(header))

    untested, example_only = [], []
    for (mode, value), by_kind in table.items():
        reg = by_kind.get("regression", [])
        others = [by_kind.get(k, []) for k in kinds if k != "regression"]
        known = value in MODES[mode][0]
        if not reg:
            (example_only if any(others) else untested).append(f"{mode}={value}")
        if gaps_only and reg:
            continue
        val = str(value) if known else f"{value} (unknown)"
        cells = [mode, val, _fmt(reg, limit=12) or "**none**"] + [_fmt(o) for o in others]
        lines.append("| " + " | ".join(cells) + " |")

    lines.append("")
    lines.append(f"Not configured anywhere ({len(untested)}): " + (", ".join(untested) or "—"))
    lines.append(f"Configured only outside the regression ({len(example_only)}): "
                 + (", ".join(example_only) or "—"))

    # An Example whose every setting a scenario already configures adds no mode
    # coverage. (It may still differ in turbine, wind input or gains.)
    unique = {label for (mode, value), by_kind in table.items() if not by_kind.get("regression")
              for kind in kinds if kind != "regression" for label in by_kind.get(kind, [])}
    redundant = sorted({label for kind, label, _ in sources if kind != "regression"} - unique)
    lines.append(f"Outside the regression, adding no mode value it lacks ({len(redundant)}): "
                 + (", ".join(redundant) or "—"))

    missing = sorted(registry_modes() - set(MODES))
    if missing:
        lines.append("")
        lines.append("WARNING: registry modes not tabulated here — add them to MODES: "
                     + ", ".join(missing))
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--gaps", action="store_true",
                        help="Only list mode values that no regression scenario configures.")
    args = parser.parse_args()
    print(report(collect_sources(), gaps_only=args.gaps))


if __name__ == "__main__":
    sys.exit(main())
