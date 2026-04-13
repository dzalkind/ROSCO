#!/usr/bin/env python3
"""
plot_verification.py — Plot key signals from vit_sim baseline_arrays npz files.

Shows wind speed (reconstructed), blade pitch, generator speed, generator torque,
generator power, and any other active signals on a shared time axis.

Usage:
    python3 scripts/plot_verification.py              # plot all 27 scenarios
    python3 scripts/plot_verification.py --scenario 1 # single scenario
    python3 scripts/plot_verification.py --scenario 1 3 5  # multiple scenarios
    python3 scripts/plot_verification.py --output plots/  # save figures to dir
"""

import argparse
import os
import sys

import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BASELINE_DIR = os.path.join(REPO_ROOT, "baseline_arrays")

# dt is 0.025 s for all scenarios
DT = 0.025

# Wind speed profile per scenario: (ws0_m_s, step_wind)
# step_wind=True  → ws[i] = ws0 + t[i] // 100  (steps up 1 m/s per 100 s)
# step_wind=False → ws = ws0 * ones  (constant)
SCENARIO_WIND = {
    1:  (7.0, True),
    2:  (9.0, False),
    3:  (9.0, True),
    4:  (9.0, True),
    5:  (9.0, True),
    6:  (9.0, True),
    7:  (9.0, False),
    8:  (9.0, False),
    9:  (7.0, True),
    10: (9.0, False),
    11: (9.0, True),
    12: (7.0, True),
    13: (9.0, True),
    14: (9.0, False),
    15: (9.0, True),
    16: (9.0, True),
    17: (7.0, True),
    18: (9.0, True),
    19: (9.0, True),
    20: (9.0, True),
    21: (9.0, True),
    22: (9.0, True),
    23: (7.0, True),
    24: (9.0, False),
    25: (7.0, True),
    26: (9.0, False),
    27: (9.0, True),
}

# Short label for each scenario
SCENARIO_LABEL = {
    1:  "Standard step-wind (baseline)",
    2:  "Yaw-by-IPC (wrap_360)",
    3:  "Filter coverage (multi-mode)",
    4:  "Flap control (Flp_Mode=2, PIIController)",
    5:  "Active wake (AWC_Mode=4, ResController)",
    6:  "IPC (IPC_ControlMode=1)",
    7:  "Synthetic inputs (manual loop)",
    8:  "IPC+AWC real blade moments",
    9:  "Startup/shutdown (SU_Mode+SD_Mode)",
    10: "Rotor position (OL_Mode=2, PIDController)",
    11: "AWC open-loop complex (AWC_Mode=1)",
    12: "K·Ω² torque (VS_ControlMode=1)",
    13: "Power overspeed (VS_FBP=1)",
    14: "Time-based open-loop (OL_Mode=1)",
    15: "Coleman transform AWC (AWC_Mode=2)",
    16: "Coleman flap (Flp_Mode=3)",
    17: "I&I wind estimator (WE_Mode=1)",
    18: "1P+2P IPC (IPC_ControlMode=2)",
    19: "PA_Mode=1 + PF_Mode=1 + VS_ConstPower",
    20: "PA_Mode=2 + PF_Mode=2 + PRC_Mode=1",
    21: "Closed-loop PI AWC (AWC_Mode=3)",
    22: "Strouhal transform AWC (AWC_Mode=5)",
    23: "PS_Mode=0 + SS_Mode=0 disabled paths",
    24: "Open-loop cable+structural (CC+StC Mode=2)",
    25: "Dynamic power rating (PRC_Mode=2)",
    26: "Flp_Mode=3 synthetic rootMOOP",
    27: "Max-coverage stress test (11 modes)",
}


def wind_speed(n, ws0, step_wind):
    """Reconstruct wind speed array matching vit_sim.py logic."""
    t = np.arange(n) * DT
    ws = np.ones(n) * ws0
    if step_wind:
        for i in range(n):
            ws[i] = ws0 + t[i] // 100
    return t, ws


def r2d(arr):
    """Radians to degrees."""
    return np.degrees(arr)


def plot_scenario(scenario_num, ax_dict=None, show=True, save_path=None):
    """
    Load baseline npz for scenario_num and plot key signals.

    Parameters
    ----------
    scenario_num : int
    ax_dict : dict or None
        If provided, reuse existing axes (keys: row names). Otherwise create new figure.
    show : bool
        Call plt.show() at the end.
    save_path : str or None
        If provided, save figure to this path.
    """
    npz_path = os.path.join(BASELINE_DIR, f"scenario_{scenario_num}.npz")
    if not os.path.exists(npz_path):
        print(f"  WARNING: {npz_path} not found, skipping.")
        return

    d = np.load(npz_path)
    n = len(d["gen_speed"])
    ws0, step_wind = SCENARIO_WIND.get(scenario_num, (9.0, True))
    t, ws = wind_speed(n, ws0, step_wind)

    # Decide which optional signal rows to show
    optional_signals = {}
    for key, label, unit, scale in [
        ("nac_yaw",       "Nac. yaw",      "deg",    np.degrees),
        ("flp_angle_1",   "Flap angle 1",  "deg",    np.degrees),
        ("flp_angle_2",   "Flap angle 2",  "deg",    np.degrees),
        ("flp_angle_3",   "Flap angle 3",  "deg",    np.degrees),
        ("cc_actuated_l", "Cable length",  "m",      lambda x: x),
        ("cc_actuated_dl","Cable Δ-length","m",      lambda x: x),
        ("stc_input",     "StC input",     "–",      lambda x: x),
    ]:
        arr = d.get(key, np.zeros(1))
        if np.any(arr != 0):
            optional_signals[key] = (label, unit, scale, arr)

    # Build row list
    rows = ["wind_speed", "bld_pitch", "gen_speed", "gen_torque", "gen_power"]
    rows += list(optional_signals.keys())

    nrows = len(rows)
    fig, axes = plt.subplots(nrows, 1, figsize=(12, 2.4 * nrows), sharex=True)
    if nrows == 1:
        axes = [axes]

    label = SCENARIO_LABEL.get(scenario_num, "")
    fig.suptitle(f"Scenario {scenario_num}: {label}", fontsize=11, fontweight="bold", y=0.995)

    ax = dict(zip(rows, axes))

    # --- Wind speed ---
    ax["wind_speed"].plot(t, ws, color="steelblue", lw=1.2)
    ax["wind_speed"].set_ylabel("Wind speed\n[m/s]")
    ax["wind_speed"].grid(True, alpha=0.3)

    # --- Blade pitch ---
    colors_pitch = ["#d62728", "#2ca02c", "#ff7f0e"]
    labels_pitch = ["Blade 1", "Blade 2", "Blade 3"]
    keys_pitch = ["bld_pitch", "bld_pitch_2", "bld_pitch_3"]
    for k, lab, col in zip(keys_pitch, labels_pitch, colors_pitch):
        if k in d:
            ax["bld_pitch"].plot(t, r2d(d[k]), color=col, lw=1.0, label=lab, alpha=0.85)
    ax["bld_pitch"].set_ylabel("Blade pitch\n[deg]")
    ax["bld_pitch"].legend(fontsize=7, loc="upper left", ncol=3)
    ax["bld_pitch"].grid(True, alpha=0.3)

    # --- Generator speed (rad/s → RPM) ---
    rpm = d["gen_speed"] * 60.0 / (2.0 * np.pi)
    ax["gen_speed"].plot(t, rpm, color="darkorange", lw=1.2)
    ax["gen_speed"].set_ylabel("Gen. speed\n[RPM]")
    ax["gen_speed"].grid(True, alpha=0.3)

    # --- Generator torque (N·m → kN·m) ---
    ax["gen_torque"].plot(t, d["gen_torque"] / 1e3, color="purple", lw=1.2)
    ax["gen_torque"].set_ylabel("Gen. torque\n[kN·m]")
    ax["gen_torque"].grid(True, alpha=0.3)

    # --- Generator power (W → MW) ---
    ax["gen_power"].plot(t, d["gen_power"] / 1e6, color="green", lw=1.2)
    ax["gen_power"].set_ylabel("Gen. power\n[MW]")
    ax["gen_power"].grid(True, alpha=0.3)

    # --- Optional signals ---
    opt_colors = ["#8c564b", "#e377c2", "#7f7f7f", "#bcbd22", "#17becf", "#aec7e8", "#ffbb78"]
    for i, (key, (label_s, unit, scale, arr)) in enumerate(optional_signals.items()):
        c = opt_colors[i % len(opt_colors)]
        ax[key].plot(t, scale(arr), color=c, lw=1.0)
        ax[key].set_ylabel(f"{label_s}\n[{unit}]")
        ax[key].grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time [s]")
    fig.tight_layout()

    if save_path:
        os.makedirs(os.path.dirname(save_path) or ".", exist_ok=True)
        fig.savefig(save_path, dpi=120, bbox_inches="tight")
        print(f"  Saved: {save_path}")

    if show:
        plt.show()

    return fig


def main():
    parser = argparse.ArgumentParser(description="Plot vit_sim verification scenarios")
    parser.add_argument("--scenario", type=int, nargs="+", default=[],
                        help="Scenario number(s) to plot (1-27). Default: all.")
    parser.add_argument("--output", type=str, default=None,
                        help="Directory to save PNG figures (e.g. plots/). "
                             "If omitted, figures are shown interactively.")
    args = parser.parse_args()

    scenarios = args.scenario if args.scenario else list(range(1, 28))

    for s in scenarios:
        if s not in SCENARIO_WIND:
            print(f"  WARNING: scenario {s} not in range 1-27, skipping.")
            continue
        print(f"Plotting scenario {s}: {SCENARIO_LABEL.get(s, '')}")
        save_path = None
        if args.output:
            save_path = os.path.join(args.output, f"scenario_{s}.png")
        plot_scenario(s, show=(args.output is None), save_path=save_path)

    if args.output:
        print(f"\nAll figures saved to: {args.output}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
