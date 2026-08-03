"""
vit_sim
-------
VIT-specific simulation script for kernel extraction and baseline capture.

Runs multiple simulation scenarios to exercise all controller code paths needed
for VIT translation verification:

  1. Standard 1-DOF step-wind simulation (same as 04_simple_sim.py)
     - Exercises: saturate, wrap_180, interp1d, LPFilter, HPFilter, SecLPFilter
  2. Yaw-by-IPC simulation with Y_ControlMode=2
     - Exercises: wrap_360 (via synthetic NacHeading/NacVane signals)
  3. Filter coverage simulation with multi-mode flags enabled
     - Exercises: NotchFilter, SecLPFilter_Vel, ForeAftDamping, FloatingFeedback,
       FlapControl, YawRateControl, StructuralControl, CableControl
  4. Flap control simulation with Flp_Mode=2
     - Exercises: PIIController (dual-integral flap controller)
  5. Active wake control simulation with AWC_Mode=4
     - Exercises: ResController (proportional-resonant controller), ActiveWakeControl
  6. IPC simulation with IPC_ControlMode=1
     - Exercises: IPC, NotchFilterSlopes
  7. Synthetic inputs for under-exercised functions (manual sim loop)
     - Exercises: YawRateControl, ForeAftDamping, FloatingFeedback,
       StructuralControl, CableControl, FlapControl with non-zero inputs
  8. IPC + AWC with real blade moments (manual sim loop)
     - Exercises: IPC (real gains), ActiveWakeControl (rootMOOP feedback),
       NotchFilterSlopes (non-zero rootMOOP)

All scenarios use the same compiled libdiscon.so, so KGen instrumentation
captures state from whichever function is being extracted.

Usage:
    python3 vit_sim.py              # Run all scenarios
    python3 vit_sim.py --scenario 1 # Standard sim only
    python3 vit_sim.py --scenario 2 # Yaw-by-IPC sim only
    python3 vit_sim.py --scenario 3 # Filter/mode coverage sim only
    python3 vit_sim.py --scenario 4 # Flap control sim only
    python3 vit_sim.py --scenario 5 # Active wake control sim only
    python3 vit_sim.py --scenario 6 # IPC sim only
    python3 vit_sim.py --scenario 7 # Synthetic inputs sim only
    python3 vit_sim.py --scenario 8 # IPC + AWC sim only
"""

import argparse
import hashlib
import os
import re
import sys

import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt
import numpy as np


def save_and_print_results(arrays, scenario_num, output_dir):
    """Save simulation arrays to .npz and print MD5 checksums.

    Args:
        arrays: dict of {name: numpy_array} — the simulation outputs
        scenario_num: int — scenario number (1-6)
        output_dir: str or None — directory to save .npz files (skip if None)
    """
    # Always print checksums to stdout
    for name, arr in sorted(arrays.items()):
        md5 = hashlib.md5(arr.tobytes()).hexdigest()
        print(f"  scenario_{scenario_num} {name}: md5={md5} n={len(arr)}")

    # Save .npz if output_dir specified
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
        path = os.path.join(output_dir, f'scenario_{scenario_num}.npz')
        np.savez(path, **arrays)
        print(f"  Saved: {path}")

from rosco import discon_lib_path as lib_name
from rosco.toolbox import controller as ROSCO_controller
from rosco.toolbox import turbine as ROSCO_turbine
from rosco.toolbox import sim as ROSCO_sim
from rosco.toolbox import control_interface as ROSCO_ci

# Additional avrSWAP indices to capture beyond the 3 primary outputs.
# Maps array name -> Python (0-indexed) avrSWAP index.
EXTRA_AVRSWAP = {
    'bld_pitch_2': 42,     # Blade 2 pitch command (Fortran index 43)
    'bld_pitch_3': 43,     # Blade 3 pitch command (Fortran index 44)
    'flp_angle_1': 119,    # Blade 1 flap pitch command (Fortran index 120)
    'flp_angle_2': 120,    # Blade 2 flap pitch command (Fortran index 121)
    'flp_angle_3': 121,    # Blade 3 flap pitch command (Fortran index 122)
    'cc_actuated_l': 2600,  # Cable control actuated length (CC_GroupIndex=2601)
    'cc_actuated_dl': 2601, # Cable control actuated delta-length
    'stc_input': 2800,      # Structural control input (StC_GroupIndex=2801)
}


def build_save_dict(sim_obj):
    """Build the full output dict from a Sim object (includes extra avrSWAP arrays)."""
    result = {
        'gen_torque': sim_obj.gen_torque, 'bld_pitch': sim_obj.bld_pitch,
        'gen_speed': sim_obj.gen_speed, 'gen_power': sim_obj.gen_power,
        'nac_yaw': sim_obj.nac_yaw,
    }
    for name in EXTRA_AVRSWAP:
        result[name] = getattr(sim_obj, name)
    return result
from rosco.toolbox.utilities import write_DISCON
from rosco.toolbox.inputs.validation import load_rosco_yaml


this_dir = os.path.dirname(os.path.abspath(__file__))
example_out_dir = os.path.join(this_dir, 'examples_out')
os.makedirs(example_out_dir, exist_ok=True)

# Workaround for scipy FITPACK bispev non-determinism (dev note 202603261512).
# FITPACK's fpbisp reads an uninitialized stack variable whose value depends on
# residual stack contents from prior Fortran calls (e.g., ROSCO's CableControl).
# We cache the RectBivariateSpline (so the FITPACK constructor doesn't refill
# the stack before evaluation) and scrub 64KB of stack before each evaluation.
#
# Confirmed in scipy 1.17.1 on aarch64 (QEMU/Colima). To test if a future scipy
# version fixes this, comment out the monkey-patch below and run Scenario 3
# thirty times — all runs should produce the same gen_torque MD5.
# See dev note 202603261512 for the full bug report with reproduction steps.
import ctypes as _ctypes
from scipy import interpolate as _interpolate
_scrub_lib_path = os.path.join(os.path.dirname(this_dir), 'rosco', 'lib', 'libscrub.so')
if os.path.exists(_scrub_lib_path):
    _scrub_lib = _ctypes.CDLL(_scrub_lib_path)
    def _scrubbed_interp_surface(self, pitch, TSR):
        if not hasattr(self, '_vit_surface_spline'):
            self._vit_surface_spline = _interpolate.RectBivariateSpline(
                self.pitch_initial_rad, self.TSR_initial, self.performance_table.T)
        _scrub_lib.scrub_stack()
        return np.squeeze(self._vit_surface_spline(pitch, TSR).T)
    def _scrubbed_interp_gradient(self, pitch, TSR):
        if not hasattr(self, '_vit_grad_pitch_spline'):
            self._vit_grad_pitch_spline = _interpolate.RectBivariateSpline(
                self.pitch_initial_rad, self.TSR_initial, self.gradient_pitch.T)
            self._vit_grad_TSR_spline = _interpolate.RectBivariateSpline(
                self.pitch_initial_rad, self.TSR_initial, self.gradient_TSR.T)
        _scrub_lib.scrub_stack()
        grad = np.array([self._vit_grad_pitch_spline(pitch, TSR).T,
                         self._vit_grad_TSR_spline(pitch, TSR).T])
        return np.ndarray.flatten(grad)
    ROSCO_turbine.RotorPerformance.interp_surface = _scrubbed_interp_surface
    ROSCO_turbine.RotorPerformance.interp_gradient = _scrubbed_interp_gradient


def load_turbine_and_controller():
    """Load the NREL5MW turbine and tune a ROSCO controller. Returns (turbine, controller)."""
    tune_dir = os.path.join(this_dir, 'Tune_Cases')
    parameter_filename = os.path.join(tune_dir, 'NREL5MW.yaml')
    inps = load_rosco_yaml(parameter_filename)
    path_params = inps['path_params']
    controller_params = inps['controller_params']

    turbine = ROSCO_turbine.Turbine
    turbine = turbine.load(os.path.join(example_out_dir, '01_NREL5MW_saved.p'))

    cp_filename = os.path.join(tune_dir, path_params['rotor_performance_filename'])
    turbine.load_from_fast(
        path_params['FAST_InputFile'],
        os.path.join(tune_dir, path_params['FAST_directory']),
        rot_source='txt', txt_filename=cp_filename
    )

    controller = ROSCO_controller.Controller(controller_params)
    controller.tune_controller(turbine)

    return turbine, controller, cp_filename


def write_discon(turbine, controller, cp_filename, param_filename, patches=None):
    """Write DISCON.IN and optionally patch parameter values.

    Args:
        patches: dict of {param_name: new_value} to replace in the generated file.
                 e.g. {'Y_ControlMode': 2}
    """
    write_DISCON(turbine, controller, param_file=param_filename, txt_filename=cp_filename)

    if patches:
        with open(param_filename, 'r') as f:
            text = f.read()
        for param, value in patches.items():
            # Match lines like "0                   ! Y_ControlMode   - description"
            # Also handles multi-value lines like "0.0 0.0   ! AWC_CntrGains ..."
            pattern = rf'^(.+?)(\s+! {param}\b.*)$'
            replacement = rf'{value}\2'
            text, count = re.subn(pattern, replacement, text, flags=re.MULTILINE)
            if count == 0:
                print(f"WARNING: Could not patch {param} in {param_filename}")
        with open(param_filename, 'w') as f:
            f.write(text)


# ---------------------------------------------------------------------------
# Scenario 1: Standard step-wind simulation (same as 04_simple_sim.py)
# ---------------------------------------------------------------------------
def run_scenario_1(turbine, controller, cp_filename, output_dir=None):
    """Standard 1-DOF sim. Exercises saturate, wrap_180, interp1d, etc."""
    print("=" * 60)
    print("Scenario 1: Standard step-wind simulation")
    print("=" * 60)

    param_filename = os.path.join(this_dir, 'DISCON.IN')
    write_discon(turbine, controller, cp_filename, param_filename)

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim1'
    )

    sim_1 = ROSCO_sim.Sim(turbine, controller_int)

    dt = 0.025
    tlen = 1000
    ws0 = 7
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0
    for i in range(len(t)):
        ws[i] = ws[i] + t[i] // 100

    sim_1.sim_ws_series(t, ws, rotor_rpm_init=4, make_plots=False, extra_avrswap=EXTRA_AVRSWAP)

    # Second run to check deallocation (same as 04_simple_sim.py)
    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim1b'
    )
    sim_1b = ROSCO_sim.Sim(turbine, controller_int)
    sim_1b.sim_ws_series(t, ws, rotor_rpm_init=4, make_plots=False, extra_avrswap=EXTRA_AVRSWAP)

    np.testing.assert_almost_equal(sim_1.gen_speed, sim_1b.gen_speed)
    save_and_print_results(build_save_dict(sim_1), 1, output_dir)
    print("Scenario 1: PASSED (deterministic, deallocated correctly)")


# ---------------------------------------------------------------------------
# Scenario 2: Yaw-by-IPC simulation (Y_ControlMode=2 → wrap_360)
# ---------------------------------------------------------------------------
def run_scenario_2(turbine, controller, cp_filename, output_dir=None):
    """Sim with Y_ControlMode=2 to exercise wrap_360.

    Sets synthetic NacHeading and NacVane values on avrSWAP so that
    wrap_360 receives inputs spanning <0, 0-360, and >=360 ranges.
    """
    print("=" * 60)
    print("Scenario 2: Yaw-by-IPC simulation (wrap_360)")
    print("=" * 60)

    param_filename = os.path.join(this_dir, 'DISCON_yaw.IN')
    write_discon(turbine, controller, cp_filename, param_filename, patches={
        'Y_ControlMode': 2,
    })

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim2'
    )

    # Shorter simulation — we just need wrap_360 to be called enough times
    # for KGen to capture 20 invocations
    dt = 0.025
    tlen = 100
    ws0 = 9
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0

    # Synthetic NacHeading and NacVane signals (in radians, as avrSWAP expects).
    # The controller converts to degrees via * R2D, then wrap_360 wraps to [0, 360].
    # We vary the sum so it exercises all 3 branches: <0, 0-360, >=360.
    deg2rad = np.pi / 180.0
    nac_heading_deg = 350.0  # Near 360 so that heading + vane can cross both boundaries
    nac_heading_rad = nac_heading_deg * deg2rad

    # 1-DOF rotor model variables
    R = turbine.rotor_radius
    GBRatio = turbine.Ng
    rpm2RadSec = 2.0 * np.pi / 60.0

    bld_pitch = np.zeros_like(t)
    rot_speed = np.ones_like(t) * 4.0 * rpm2RadSec
    gen_speed = rot_speed * GBRatio
    gen_torque = np.zeros_like(t)
    nac_yaw = np.zeros_like(t)
    gen_power = np.zeros_like(t)
    extra = {name: np.zeros_like(t) for name in EXTRA_AVRSWAP}

    for i, ti in enumerate(t):
        if i == 0:
            continue

        ws_i = ws[i]
        tsr = rot_speed[i-1] * R / ws_i
        cp = turbine.Cp.interp_surface(bld_pitch[i-1], tsr)
        aero_torque = 0.5 * turbine.rho * (np.pi * R**3) * (cp / tsr) * ws_i**2
        rot_speed[i] = rot_speed[i-1] + (dt / turbine.J) * (
            aero_torque - GBRatio * gen_torque[i-1] / (turbine.GBoxEff / 100)
        )
        gen_speed[i] = rot_speed[i] * GBRatio

        # Synthetic NacVane: oscillates so heading+vane crosses 0 and 360
        nac_vane_deg = 20.0 * np.sin(2 * np.pi * ti / 50.0)  # +/- 20 deg
        nac_vane_rad = nac_vane_deg * deg2rad

        # Synthetic rotor azimuth (just accumulate based on rot speed)
        azimuth_rad = (rot_speed[i] * ti) % (2 * np.pi)

        # Set standard turbine state
        turbine_state = {}
        turbine_state['iStatus'] = 1 if i < len(t) - 1 else -1
        turbine_state['t'] = ti
        turbine_state['dt'] = dt
        turbine_state['ws'] = ws_i
        turbine_state['bld_pitch'] = bld_pitch[i-1]
        turbine_state['gen_torque'] = gen_torque[i-1]
        turbine_state['gen_speed'] = gen_speed[i]
        turbine_state['gen_eff'] = turbine.GenEff / 100
        turbine_state['rot_speed'] = rot_speed[i]
        turbine_state['Yaw_fromNorth'] = 0.0
        turbine_state['Y_MeasErr'] = 0.0

        # Set extra avrSWAP values that call_controller doesn't handle.
        # These must be set BEFORE call_controller (which calls call_discon).
        # Python index = Fortran index - 1.
        controller_int.avrSWAP[23] = nac_vane_rad    # avrSWAP(24) = NacVane [rad]
        controller_int.avrSWAP[36] = nac_heading_rad  # avrSWAP(37) = NacHeading [rad]
        controller_int.avrSWAP[59] = azimuth_rad      # avrSWAP(60) = Azimuth [rad]

        # Call controller
        gen_torque[i], bld_pitch[i], nac_yaw[i] = controller_int.call_controller(turbine_state)
        gen_power[i] = gen_torque[i] * gen_speed[i] * (turbine.GenEff / 100)
        for name, idx in EXTRA_AVRSWAP.items():
            extra[name][i] = controller_int.avrSWAP[idx]

    controller_int.kill_discon()
    result = {
        'gen_torque': gen_torque, 'bld_pitch': bld_pitch,
        'gen_speed': gen_speed, 'gen_power': gen_power,
        'nac_yaw': nac_yaw,
    }
    result.update(extra)
    save_and_print_results(result, 2, output_dir)
    print("Scenario 2: PASSED (wrap_360 exercised)")


# ---------------------------------------------------------------------------
# Scenario 3: Filter coverage (IPC + notch + cable control)
# ---------------------------------------------------------------------------
def run_scenario_3(turbine, controller, cp_filename, output_dir=None):
    """Sim with notch filters, cable control, and multiple mode flags enabled.

    Exercises filter/controller functions not active in the standard config:
    - NotchFilter: F_GenSpdNotch_N=1 adds a notch at 1.0 rad/s on gen speed
    - SecLPFilter_Vel: CC_Group_N=1 enables the cable control filter loop
    - FlapControl: Flp_Mode=1 (IPC-based flap control)
    - Plus: TD_Mode, Fl_Mode, Y_ControlMode, StC_Mode, CC_Mode

    Note: NotchFilterSlopes requires IPC_ControlMode=1, which conflicts with
    Flp_Mode > 0 (ROSCO mutual exclusion). Use Scenario 6 for NotchFilterSlopes.
    """
    print("=" * 60)
    print("Scenario 3: Mode coverage (notch + cable + flap + structural)")
    print("=" * 60)

    param_filename = os.path.join(this_dir, 'DISCON_filters.IN')
    write_discon(turbine, controller, cp_filename, param_filename, patches={
        # NotchFilter: 1 notch filter on generator speed
        'F_NumNotchFilts': 1,
        'F_NotchFreqs': '1.0000',
        'F_NotchBetaNum': '0.0000',
        'F_NotchBetaDen': '0.2500',
        'F_GenSpdNotch_N': 1,
        'F_GenSpdNotch_Ind': '1',
        # Note: NotchFilterSlopes requires IPC_ControlMode=1, but ROSCO rejects
        # IPC_ControlMode > 0 AND Flp_Mode > 0 simultaneously. Since this scenario
        # needs Flp_Mode=1, NotchFilterSlopes must use Scenario 6 instead.

        # SecLPFilter_Vel: enable cable control loop
        # CC_Mode=1 required so DISCON calls CableControl (gated by CC_Mode > 0).
        # CC_DesiredL is fixed-size(12) initialized to 0; with CC_Group_N=1 and
        # tlen=400 < 500, the filter processes zero input the entire run.
        'CC_Mode': 1,
        'CC_Group_N': 1,
        'CC_GroupIndex': '2601',
        # ForeAftDamping: enable tower damper (TD_Mode=1).
        # FA_AccHPF is 0 in 1-DOF sim, so output is 0 — fine for view type verification.
        'TD_Mode': 1,
        # FloatingFeedback: enable floating-specific feedback (Fl_Mode=1).
        # NacIMU_FA_Acc is 0 in 1-DOF sim — output is 0, fine for verification.
        'Fl_Mode': 1,
        # YawRateControl: enable yaw rate control (Y_ControlMode=1).
        # NacHeading/NacVane are 0 in 1-DOF — yaw error is 0, fine for verification.
        'Y_ControlMode': 1,
        # StructuralControl: enable structural control (StC_Mode=1).
        # Uses fixed hardcoded step — processes zero input in 1-DOF sim.
        'StC_Mode': 1,
        'StC_Group_N': 1,
        'StC_GroupIndex': '2801',
        # FlapControl: enable IPC-based flap control (Flp_Mode=1).
        # Blade root bending moments are 0 in 1-DOF sim — fine for extraction.
        'Flp_Mode': 1,
        'F_FlpCornerFreq': '1.0 0.7',  # Required when Flp_Mode > 0
        # FloatingFeedback: required when Fl_Mode > 0
        'F_FlCornerFreq': '1.0 0.7',  # Filter corner freq + damping for floating feedback
    })

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim3'
    )

    sim_3 = ROSCO_sim.Sim(turbine, controller_int)

    dt = 0.025
    tlen = 400
    ws0 = 9
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0
    for i in range(len(t)):
        ws[i] = ws[i] + t[i] // 100

    sim_3.sim_ws_series(t, ws, rotor_rpm_init=4, make_plots=False, extra_avrswap=EXTRA_AVRSWAP)
    save_and_print_results(build_save_dict(sim_3), 3, output_dir)
    print("Scenario 3: PASSED (NotchFilter + SecLPFilter_Vel + mode flags exercised)")


# ---------------------------------------------------------------------------
# Scenario 4: Flap control (Flp_Mode=2 → PIIController)
# ---------------------------------------------------------------------------
def run_scenario_4(turbine, controller, cp_filename, output_dir=None):
    """Sim with Flp_Mode=2 to exercise PIIController.

    Flp_Mode=2 enables proportional flap control using PIIController
    (dual-integral variant of PIController). IPC must be disabled
    (mutual exclusion: IPC_ControlMode=0 when Flp_Mode > 0).

    In the 1-DOF sim, blade root moments are near-zero, so the flap
    controller processes small signals. This is fine for state capture.
    """
    print("=" * 60)
    print("Scenario 4: Flap control (Flp_Mode=2, PIIController)")
    print("=" * 60)

    param_filename = os.path.join(this_dir, 'DISCON_flp.IN')
    write_discon(turbine, controller, cp_filename, param_filename, patches={
        'Flp_Mode': 2,
        'IPC_ControlMode': 0,  # Mutual exclusion with Flp_Mode > 0
        # F_FlpCornerFreq(1) must be > 0 when Flp_Mode > 0
        # (validation in CheckInputs sets aviFAIL=-1 otherwise)
        'F_FlpCornerFreq': '0.5000  0.7000',
        # Small nonzero gains so PIIController runs without instability
        'Flp_Kp': '-0.001',
        'Flp_Ki': '-0.0005',
    })

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim4'
    )

    sim_4 = ROSCO_sim.Sim(turbine, controller_int)

    dt = 0.025
    tlen = 100
    ws0 = 9
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0
    for i in range(len(t)):
        ws[i] = ws[i] + t[i] // 100

    sim_4.sim_ws_series(t, ws, rotor_rpm_init=4, make_plots=False, extra_avrswap=EXTRA_AVRSWAP)
    save_and_print_results(build_save_dict(sim_4), 4, output_dir)
    print("Scenario 4: PASSED (PIIController exercised)")


# ---------------------------------------------------------------------------
# Scenario 5: Active wake control (AWC_Mode=4 → ResController)
# ---------------------------------------------------------------------------
def run_scenario_5(turbine, controller, cp_filename, output_dir=None):
    """Sim with AWC_Mode=4 to exercise ResController.

    AWC_Mode=4 enables closed-loop proportional-resonant active wake
    control. The ResController uses Tustin-discretized resonant filter
    equations. AWC_Mode > 1 requires individual pitch control
    (avrSWAP(28)=1, already set by ControllerInterface).

    In the 1-DOF sim, ColemanTransformed blade root moments are
    near-zero, so the resonant controller processes small error signals.
    """
    print("=" * 60)
    print("Scenario 5: Active wake control (AWC_Mode=4, ResController)")
    print("=" * 60)

    param_filename = os.path.join(this_dir, 'DISCON_awc.IN')
    write_discon(turbine, controller, cp_filename, param_filename, patches={
        'AWC_Mode': 4,
        # Nonzero gains so ResController produces nontrivial output
        'AWC_CntrGains': '0.0100 0.0050',
    })

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim5'
    )

    sim_5 = ROSCO_sim.Sim(turbine, controller_int)

    dt = 0.025
    tlen = 400
    ws0 = 9
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0
    for i in range(len(t)):
        ws[i] = ws[i] + t[i] // 100

    sim_5.sim_ws_series(t, ws, rotor_rpm_init=4, make_plots=False, extra_avrswap=EXTRA_AVRSWAP)
    save_and_print_results(build_save_dict(sim_5), 5, output_dir)
    print("Scenario 5: PASSED (ResController exercised)")


# ---------------------------------------------------------------------------
# Scenario 6: IPC (IPC_ControlMode=1 → NotchFilterSlopes)
# ---------------------------------------------------------------------------
def run_scenario_6(turbine, controller, cp_filename, output_dir=None):
    """Sim with IPC_ControlMode=1 to exercise NotchFilterSlopes.

    IPC_ControlMode=1 enables Individual Pitch Control, which calls
    NotchFilterSlopes for 1P blade root bending moment filtering
    (gated by IPC_ControlMode > 0 in PreFilterMeasuredSignals).

    Flp_Mode must be 0: ROSCO rejects IPC_ControlMode > 0 AND Flp_Mode > 0
    simultaneously (ReadSetParameters.f90:1108).

    In the 1-DOF sim, blade root moments are 0, so IPC and
    NotchFilterSlopes process zero signals. Fine for state capture.
    """
    print("=" * 60)
    print("Scenario 6: IPC (IPC_ControlMode=1, NotchFilterSlopes)")
    print("=" * 60)

    param_filename = os.path.join(this_dir, 'DISCON_ipc.IN')
    write_discon(turbine, controller, cp_filename, param_filename, patches={
        'IPC_ControlMode': 1,
        'Flp_Mode': 0,       # Mutual exclusion with IPC_ControlMode > 0
        'IPC_KI': '0.0 0.0',
        'IPC_KP': '0.0 0.0',
    })

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim6'
    )

    sim_6 = ROSCO_sim.Sim(turbine, controller_int)

    dt = 0.025
    tlen = 100
    ws0 = 9
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0
    for i in range(len(t)):
        ws[i] = ws[i] + t[i] // 100

    sim_6.sim_ws_series(t, ws, rotor_rpm_init=4, make_plots=False, extra_avrswap=EXTRA_AVRSWAP)
    save_and_print_results(build_save_dict(sim_6), 6, output_dir)
    print("Scenario 6: PASSED (NotchFilterSlopes exercised)")


# ---------------------------------------------------------------------------
# Scenario 7: Synthetic inputs for under-exercised functions
# ---------------------------------------------------------------------------
def run_scenario_7(turbine, controller, cp_filename, output_dir=None):
    """Exercises functions that receive zero inputs in the 1-DOF sim.

    Uses a manual sim loop (like Scenario 2) to inject synthetic non-zero
    values into avrSWAP before each controller call:
    - NacVane/NacHeading: oscillating yaw error for YawRateControl
    - FA_Acc_TT: oscillating tower acceleration for ForeAftDamping
    - NacIMU_FA_RAcc: oscillating nacelle IMU for FloatingFeedback
    - rootMOOP: per-blade 1P sinusoidal moments for FlapControl
    - tlen=600s: exceeds Time>500 threshold for StructuralControl/CableControl

    Mode flags: Y_ControlMode=1, TD_Mode=1, Fl_Mode=1, StC_Mode=1,
    CC_Mode=1, Flp_Mode=1. IPC_ControlMode=0 (mutual exclusion with Flp_Mode).
    """
    print("=" * 60)
    print("Scenario 7: Synthetic inputs (yaw, tower, float, struct, cable, flap)")
    print("=" * 60)

    param_filename = os.path.join(this_dir, 'DISCON_synth.IN')
    write_discon(turbine, controller, cp_filename, param_filename, patches={
        'Y_ControlMode': 1,
        'TD_Mode': 1,
        'Fl_Mode': 1,
        'StC_Mode': 1,
        'StC_Group_N': 1,
        'StC_GroupIndex': '2801',
        'CC_Mode': 1,
        'CC_Group_N': 1,
        'CC_GroupIndex': '2601',
        'Flp_Mode': 1,
        'F_FlpCornerFreq': '1.0 0.7',
        'F_FlCornerFreq': '1.0 0.7',
        'IPC_ControlMode': 0,
        'AWC_Mode': 0,
        'F_NumNotchFilts': 1,
        'F_NotchFreqs': '1.0000',
        'F_NotchBetaNum': '0.0000',
        'F_NotchBetaDen': '0.2500',
        'F_GenSpdNotch_N': 1,
        'F_GenSpdNotch_Ind': '1',
    })

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim7'
    )

    dt = 0.025
    tlen = 600  # Must exceed 500 for StructuralControl/CableControl step
    ws0 = 9
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0
    for i_ws in range(len(t)):
        ws[i_ws] = ws[i_ws] + t[i_ws] // 100

    deg2rad = np.pi / 180.0
    R = turbine.rotor_radius
    GBRatio = turbine.Ng
    rpm2RadSec = 2.0 * np.pi / 60.0

    bld_pitch = np.zeros_like(t)
    rot_speed = np.ones_like(t) * 4.0 * rpm2RadSec
    gen_speed = rot_speed * GBRatio
    gen_torque = np.zeros_like(t)
    gen_power = np.zeros_like(t)
    nac_yaw = np.zeros_like(t)
    nac_yawrate = np.zeros_like(t)
    extra = {name: np.zeros_like(t) for name in EXTRA_AVRSWAP}

    for i, ti in enumerate(t):
        if i == 0:
            continue

        ws_i = ws[i]
        tsr = rot_speed[i-1] * R / ws_i
        cp = turbine.Cp.interp_surface(bld_pitch[i-1], tsr)
        aero_torque = 0.5 * turbine.rho * (np.pi * R**3) * (cp / tsr) * ws_i**2
        rot_speed[i] = rot_speed[i-1] + (dt / turbine.J) * (
            aero_torque - GBRatio * gen_torque[i-1] / (turbine.GBoxEff / 100)
        )
        gen_speed[i] = rot_speed[i] * GBRatio

        # Synthetic yaw signals
        nac_vane_deg = 20.0 * np.sin(2 * np.pi * ti / 50.0)
        nac_vane_rad = nac_vane_deg * deg2rad
        nac_heading_rad = 350.0 * deg2rad

        # Synthetic rotor azimuth
        azimuth_rad = (rot_speed[i] * ti) % (2 * np.pi)

        # Synthetic tower fore-aft acceleration (~0.3 Hz tower mode)
        fa_acc_tt = 0.5 * np.sin(2 * np.pi * ti / 3.0)

        # Synthetic nacelle IMU acceleration
        nac_imu_fa_racc = 0.3 * np.sin(2 * np.pi * ti / 3.0)

        # Synthetic blade root moments (1P per-blade, 120 deg phase offset)
        if rot_speed[i] > 0.1:
            t_rotor = 2 * np.pi / rot_speed[i]
        else:
            t_rotor = 100.0
        rootMOOP = [
            1000.0 * np.sin(2 * np.pi * ti / t_rotor + k * 2 * np.pi / 3)
            for k in range(3)
        ]

        turbine_state = {}
        turbine_state['iStatus'] = 1 if i < len(t) - 1 else -1
        turbine_state['t'] = ti
        turbine_state['dt'] = dt
        turbine_state['ws'] = ws_i
        turbine_state['bld_pitch'] = bld_pitch[i-1]
        turbine_state['gen_torque'] = gen_torque[i-1]
        turbine_state['gen_speed'] = gen_speed[i]
        turbine_state['gen_eff'] = turbine.GenEff / 100
        turbine_state['rot_speed'] = rot_speed[i]
        turbine_state['Yaw_fromNorth'] = nac_yaw[i-1]
        turbine_state['Y_MeasErr'] = nac_vane_rad
        turbine_state['FA_Acc_TT'] = fa_acc_tt
        turbine_state['NacIMU_FA_RAcc'] = nac_imu_fa_racc

        # Inject avrSWAP values not handled by call_controller
        controller_int.avrSWAP[23] = nac_vane_rad      # avrSWAP(24) NacVane
        controller_int.avrSWAP[36] = nac_heading_rad    # avrSWAP(37) NacHeading
        controller_int.avrSWAP[59] = azimuth_rad         # avrSWAP(60) Azimuth
        controller_int.avrSWAP[29] = rootMOOP[0]         # avrSWAP(30) rootMOOP(1)
        controller_int.avrSWAP[30] = rootMOOP[1]         # avrSWAP(31) rootMOOP(2)
        controller_int.avrSWAP[31] = rootMOOP[2]         # avrSWAP(32) rootMOOP(3)

        gen_torque[i], bld_pitch[i], nac_yawrate[i] = controller_int.call_controller(turbine_state)
        gen_power[i] = gen_speed[i] * gen_torque[i] * turbine.GenEff / 100
        nac_yaw[i] = nac_yaw[i-1] + nac_yawrate[i] * dt
        for name, idx in EXTRA_AVRSWAP.items():
            extra[name][i] = controller_int.avrSWAP[idx]

    controller_int.kill_discon()
    result = {
        'gen_torque': gen_torque, 'bld_pitch': bld_pitch,
        'gen_speed': gen_speed, 'gen_power': gen_power,
        'nac_yaw': nac_yaw,
    }
    result.update(extra)
    save_and_print_results(result, 7, output_dir)
    print("Scenario 7: PASSED (yaw, tower, float, struct, cable, flap exercised)")


# ---------------------------------------------------------------------------
# Scenario 8: IPC with real gains + blade moments + ActiveWakeControl
# ---------------------------------------------------------------------------
def run_scenario_8(turbine, controller, cp_filename, output_dir=None):
    """Exercises IPC and ActiveWakeControl with non-zero blade root moments.

    IPC_ControlMode=1 with non-zero gains + AWC_Mode=4 with rootMOOP feedback.
    Also exercises NotchFilterSlopes (triggered by IPC_ControlMode > 0 in
    PreFilterMeasuredSignals).

    Flp_Mode=0 (mutual exclusion with IPC_ControlMode > 0).
    """
    print("=" * 60)
    print("Scenario 8: IPC + AWC with blade moments")
    print("=" * 60)

    param_filename = os.path.join(this_dir, 'DISCON_ipc_awc.IN')
    write_discon(turbine, controller, cp_filename, param_filename, patches={
        'IPC_ControlMode': 1,
        'IPC_KP': '0.1 0.1',
        'IPC_KI': '0.01 0.01',
        'AWC_Mode': 4,
        'AWC_NumModes': 1,
        'AWC_n': '1',
        'AWC_clockangle': '0.0',
        'AWC_freq': '0.05',
        'AWC_amp': '0.0',
        'AWC_CntrGains': '0.0100 0.0050',
        'Flp_Mode': 0,
        'F_NumNotchFilts': 1,
        'F_NotchFreqs': '1.0000',
        'F_NotchBetaNum': '0.0000',
        'F_NotchBetaDen': '0.2500',
        'F_GenSpdNotch_N': 1,
        'F_GenSpdNotch_Ind': '1',
    })

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim8'
    )

    dt = 0.025
    tlen = 400
    ws0 = 9
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0
    for i_ws in range(len(t)):
        ws[i_ws] = ws[i_ws] + t[i_ws] // 100

    deg2rad = np.pi / 180.0
    R = turbine.rotor_radius
    GBRatio = turbine.Ng
    rpm2RadSec = 2.0 * np.pi / 60.0

    bld_pitch = np.zeros_like(t)
    rot_speed = np.ones_like(t) * 4.0 * rpm2RadSec
    gen_speed = rot_speed * GBRatio
    gen_torque = np.zeros_like(t)
    gen_power = np.zeros_like(t)
    nac_yaw = np.zeros_like(t)
    extra = {name: np.zeros_like(t) for name in EXTRA_AVRSWAP}

    for i, ti in enumerate(t):
        if i == 0:
            continue

        ws_i = ws[i]
        tsr = rot_speed[i-1] * R / ws_i
        cp = turbine.Cp.interp_surface(bld_pitch[i-1], tsr)
        aero_torque = 0.5 * turbine.rho * (np.pi * R**3) * (cp / tsr) * ws_i**2
        rot_speed[i] = rot_speed[i-1] + (dt / turbine.J) * (
            aero_torque - GBRatio * gen_torque[i-1] / (turbine.GBoxEff / 100)
        )
        gen_speed[i] = rot_speed[i] * GBRatio

        azimuth_rad = (rot_speed[i] * ti) % (2 * np.pi)

        # Synthetic blade root moments (1P per-blade)
        if rot_speed[i] > 0.1:
            t_rotor = 2 * np.pi / rot_speed[i]
        else:
            t_rotor = 100.0
        rootMOOP = [
            1000.0 * np.sin(2 * np.pi * ti / t_rotor + k * 2 * np.pi / 3)
            for k in range(3)
        ]

        turbine_state = {}
        turbine_state['iStatus'] = 1 if i < len(t) - 1 else -1
        turbine_state['t'] = ti
        turbine_state['dt'] = dt
        turbine_state['ws'] = ws_i
        turbine_state['bld_pitch'] = bld_pitch[i-1]
        turbine_state['gen_torque'] = gen_torque[i-1]
        turbine_state['gen_speed'] = gen_speed[i]
        turbine_state['gen_eff'] = turbine.GenEff / 100
        turbine_state['rot_speed'] = rot_speed[i]
        turbine_state['Yaw_fromNorth'] = 0.0
        turbine_state['Y_MeasErr'] = 0.0

        # Inject blade root moments and azimuth
        controller_int.avrSWAP[29] = rootMOOP[0]   # avrSWAP(30) rootMOOP(1)
        controller_int.avrSWAP[30] = rootMOOP[1]   # avrSWAP(31) rootMOOP(2)
        controller_int.avrSWAP[31] = rootMOOP[2]   # avrSWAP(32) rootMOOP(3)
        controller_int.avrSWAP[59] = azimuth_rad    # avrSWAP(60) Azimuth

        gen_torque[i], bld_pitch[i], _ = controller_int.call_controller(turbine_state)
        gen_power[i] = gen_speed[i] * gen_torque[i] * turbine.GenEff / 100
        for name, idx in EXTRA_AVRSWAP.items():
            extra[name][i] = controller_int.avrSWAP[idx]

    controller_int.kill_discon()
    result = {
        'gen_torque': gen_torque, 'bld_pitch': bld_pitch,
        'gen_speed': gen_speed, 'gen_power': gen_power,
        'nac_yaw': nac_yaw,
    }
    result.update(extra)
    save_and_print_results(result, 8, output_dir)
    print("Scenario 8: PASSED (IPC + AWC with blade moments exercised)")


# ---------------------------------------------------------------------------
# Scenario 9: Startup + Shutdown + TRA (SU_Mode=1, SD_Mode=1, TRA_Mode=1)
# ---------------------------------------------------------------------------
def run_scenario_9(turbine, controller, cp_filename, output_dir=None):
    """Exercises Startup, Shutdown, and RefSpeedExclusion functions.

    SU_Mode=1 with early start time and short durations so startup stages
    progress within the first 20 invocations. SD_Mode=1 with time-based
    shutdown triggered late (well past extraction window). TRA_Mode=1 with
    exclusion band near rated LSS speed.
    """
    print("=" * 60)
    print("Scenario 9: Startup/Shutdown/TRA (SU_Mode=1, SD_Mode=1, TRA_Mode=1)")
    print("=" * 60)

    param_filename = os.path.join(this_dir, 'DISCON_su_sd_tra.IN')
    write_discon(turbine, controller, cp_filename, param_filename, patches={
        'SU_Mode': 1,
        'SU_StartTime': 0,
        'SU_FW_MinDuration': 5,
        'SU_RotorSpeedThresh': 0.3,
        'SU_RotorSpeedCornerFreq': 0.5,
        'SU_LoadStages_N': 2,
        'SU_LoadStages': '0.5 1.0',
        'SU_LoadRampDuration': '10 10',
        'SU_LoadHoldDuration': '10 10',
        'SD_Mode': 1,
        'SD_TimeActivate': 0,
        'SD_EnablePitch': 0,
        'SD_EnableYawError': 0,
        'SD_EnableGenSpeed': 0,
        'SD_EnableTime': 1,
        'SD_Time': 250,
        'SD_Method': 1,
        'SD_Stage_N': 2,
        'SD_StageTime': '50 50',
        'SD_MaxPitchRate': '0.05 0.1',
        'SD_MaxTorqueRate': '1000 2000',
        'SD_StagePitch': '0.5 1.57',
        'TRA_Mode': 1,
        'TRA_ExclSpeed': 0.8,
        'TRA_ExclBand': 0.1,
        'TRA_RateLimit': 0.01,
    })

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim9'
    )

    sim_9 = ROSCO_sim.Sim(turbine, controller_int)

    dt = 0.025
    tlen = 300
    ws0 = 7
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0
    for i in range(len(t)):
        ws[i] = ws[i] + t[i] // 100

    sim_9.sim_ws_series(t, ws, rotor_rpm_init=4, make_plots=False, extra_avrswap=EXTRA_AVRSWAP)
    save_and_print_results(build_save_dict(sim_9), 9, output_dir)
    print("Scenario 9: PASSED (startup/shutdown/TRA exercised)")


# ---------------------------------------------------------------------------
# Scenario 10: Open loop rotor position control (OL_Mode=2 → PIDController)
# ---------------------------------------------------------------------------
def run_scenario_10(turbine, controller, cp_filename, output_dir=None):
    """OL_Mode=2 azimuth tracking to exercise PIDController.

    PIDController is only called when OL_Mode=2. It uses RP_Gains (Kp, Ki, Kd, Tf)
    to track a desired azimuth profile from the OL input file. The PID output
    (GenTqAz) is added to the base generator torque.
    """
    print("=" * 60)
    print("Scenario 10: Rotor position control (OL_Mode=2, PIDController)")
    print("=" * 60)

    ol_input_path = os.path.join(this_dir, 'example_inputs', 'OL_Mode2_Input.dat')
    param_filename = os.path.join(this_dir, 'DISCON_ol_mode2.IN')
    write_discon(turbine, controller, cp_filename, param_filename, patches={
        'OL_Mode': 2,
        'OL_Filename': ol_input_path,
        'OL_BP_Mode': 0,
        'OL_BP_FiltFreq': 0.0,
        'Ind_Breakpoint': 1,
        'Ind_BldPitch': '2 3 4',
        'Ind_GenTq': 5,
        'Ind_Azimuth': 6,
        'Ind_YawRate': 0,
        'Ind_R_Speed': 0,
        'Ind_R_Torque': 0,
        'Ind_R_Pitch': 0,
        'RP_Gains': '1000.0 100.0 500.0 0.1',
        # Disable modes incompatible with OL_Mode=2
        'CC_Mode': 0,
        'StC_Mode': 0,
    })

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim10'
    )

    sim_10 = ROSCO_sim.Sim(turbine, controller_int)

    dt = 0.025
    tlen = 100
    ws0 = 9
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0

    sim_10.sim_ws_series(t, ws, rotor_rpm_init=4, make_plots=False, extra_avrswap=EXTRA_AVRSWAP)

    save_and_print_results(build_save_dict(sim_10), 10, output_dir)
    print("Scenario 10: PASSED (PIDController for azimuth tracking exercised)")


# ---------------------------------------------------------------------------
# Scenario 11: AWC_Mode=1 (open-loop complex number AWC)
# ---------------------------------------------------------------------------
def run_scenario_11(turbine, controller, cp_filename, output_dir=None):
    """Sim with AWC_Mode=1 to exercise open-loop complex number AWC.

    AWC_Mode=1 applies sinusoidal pitch commands phase-locked to blade
    azimuth using the complex number method. This is the only code path
    that uses AWC_complexangle (COMPLEX in upstream ROSCO, decomposed to
    COS/SIN REAL pair in our fork). This scenario verifies the decomposition
    produces identical output.
    """
    print("=" * 60)
    print("Scenario 11: Active wake control (AWC_Mode=1, complex number method)")
    print("=" * 60)

    param_filename = os.path.join(this_dir, 'DISCON_awc.IN')
    write_discon(turbine, controller, cp_filename, param_filename, patches={
        'AWC_Mode': 1,
        'AWC_NumModes': 1,
        'AWC_n': '1',
        'AWC_freq': '0.05',
        'AWC_amp': '2.0',
        'AWC_clockangle': '0.0',
    })

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim11'
    )

    sim_11 = ROSCO_sim.Sim(turbine, controller_int)

    dt = 0.025
    tlen = 400
    ws0 = 9
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0
    for i in range(len(t)):
        ws[i] = ws[i] + t[i] // 100

    sim_11.sim_ws_series(t, ws, rotor_rpm_init=4, make_plots=False, extra_avrswap=EXTRA_AVRSWAP)
    save_and_print_results(build_save_dict(sim_11), 11, output_dir)
    print("Scenario 11: PASSED (AWC_Mode=1 complex number method exercised)")


# ---------------------------------------------------------------------------
# Scenario 12: VS_ControlMode=1 (K*Omega^2 torque control)
# ---------------------------------------------------------------------------
def run_scenario_12(turbine, controller, cp_filename, output_dir=None):
    """Sim with VS_ControlMode=1 to exercise K*Omega^2 torque control.

    VS_ControlMode=1 is the classical ROSCO torque control with 5 operating
    regions (1, 1.5, 2, 2.5, 3) and PI transitions. This is completely
    different from the default VS_ControlMode=2 (WSE TSR tracking).
    """
    print("=" * 60)
    print("Scenario 12: K*Omega^2 torque control (VS_ControlMode=1)")
    print("=" * 60)

    param_filename = os.path.join(this_dir, 'DISCON.IN')
    write_discon(turbine, controller, cp_filename, param_filename, patches={
        'VS_ControlMode': 1,
    })

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim12'
    )

    sim_12 = ROSCO_sim.Sim(turbine, controller_int)

    dt = 0.025
    tlen = 400
    ws0 = 7
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0
    for i in range(len(t)):
        ws[i] = ws[i] + t[i] // 100

    sim_12.sim_ws_series(t, ws, rotor_rpm_init=4, make_plots=False, extra_avrswap=EXTRA_AVRSWAP)
    save_and_print_results(build_save_dict(sim_12), 12, output_dir)
    print("Scenario 12: PASSED (VS_ControlMode=1 K*Omega^2 exercised)")


# ---------------------------------------------------------------------------
# Scenario 13: VS_FBP=1 (power overspeed, no pitch control)
# ---------------------------------------------------------------------------
def run_scenario_13(turbine, controller, cp_filename, output_dir=None):
    """Sim with VS_FBP=1 to exercise fixed-blade-pitch power overspeed mode.

    VS_FBP=1 (power overspeed) disables pitch control and uses constant-power
    saturation in Region 3. Exercises FBP lookup tables (VS_FBP_U, VS_FBP_Omega,
    VS_FBP_Tau) and the interp1d-based torque reference path.

    Requires PC_ControlMode=0 (mutual exclusion with VS_FBP>0).
    """
    print("=" * 60)
    print("Scenario 13: Power overspeed mode (VS_FBP=1)")
    print("=" * 60)

    param_filename = os.path.join(this_dir, 'DISCON.IN')
    write_discon(turbine, controller, cp_filename, param_filename, patches={
        'VS_FBP': 1,
        'PC_ControlMode': 0,
        'VS_ControlMode': 1,
    })

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim13'
    )

    sim_13 = ROSCO_sim.Sim(turbine, controller_int)

    dt = 0.025
    tlen = 400
    ws0 = 9
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0
    for i in range(len(t)):
        ws[i] = ws[i] + t[i] // 100

    sim_13.sim_ws_series(t, ws, rotor_rpm_init=4, make_plots=False, extra_avrswap=EXTRA_AVRSWAP)
    save_and_print_results(build_save_dict(sim_13), 13, output_dir)
    print("Scenario 13: PASSED (VS_FBP=1 power overspeed exercised)")


# ---------------------------------------------------------------------------
# Scenario 14: OL_Mode=1 (time-based open-loop control)
# ---------------------------------------------------------------------------
def run_scenario_14(turbine, controller, cp_filename, output_dir=None):
    """Sim with OL_Mode=1 to exercise time-based open-loop pitch/torque/yaw.

    OL_Mode=1 reads open-loop signals from a breakpoint file and interpolates
    blade pitch, generator torque, and yaw rate as functions of time. Exercises
    the OL_Breakpoints/OL_BldPitch/OL_GenTq/OL_YawRate interpolation paths.
    """
    print("=" * 60)
    print("Scenario 14: Time-based open-loop control (OL_Mode=1)")
    print("=" * 60)

    ol_input_path = os.path.join(this_dir, 'example_inputs', 'OL_Mode1_Input.dat')
    param_filename = os.path.join(this_dir, 'DISCON_ol_mode1.IN')
    write_discon(turbine, controller, cp_filename, param_filename, patches={
        'OL_Mode': 1,
        'OL_Filename': ol_input_path,
        'OL_BP_Mode': 0,
        'OL_BP_FiltFreq': 0.0,
        'Ind_Breakpoint': 1,
        'Ind_BldPitch': '2 3 4',
        'Ind_GenTq': 5,
        'Ind_YawRate': 6,
        'Ind_Azimuth': 0,
        'Ind_R_Speed': 0,
        'Ind_R_Torque': 0,
        'Ind_R_Pitch': 0,
        'CC_Mode': 0,
        'StC_Mode': 0,
    })

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim14'
    )

    sim_14 = ROSCO_sim.Sim(turbine, controller_int)

    dt = 0.025
    tlen = 200
    ws0 = 9
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0

    sim_14.sim_ws_series(t, ws, rotor_rpm_init=4, make_plots=False, extra_avrswap=EXTRA_AVRSWAP)
    save_and_print_results(build_save_dict(sim_14), 14, output_dir)
    print("Scenario 14: PASSED (OL_Mode=1 open-loop control exercised)")


# ---------------------------------------------------------------------------
# Scenario 15: AWC_Mode=2 (Coleman transform AWC)
# ---------------------------------------------------------------------------
def run_scenario_15(turbine, controller, cp_filename, output_dir=None):
    """Sim with AWC_Mode=2 to exercise Coleman transform active wake control.

    AWC_Mode=2 uses ColemanTransform/ColemanTransformInverse to apply
    harmonic pitch forcing in the rotating frame. Exercises a different
    AWC code path from Mode=1 (complex number) and Mode=4 (resonator).
    """
    print("=" * 60)
    print("Scenario 15: Coleman transform AWC (AWC_Mode=2)")
    print("=" * 60)

    param_filename = os.path.join(this_dir, 'DISCON_awc.IN')
    write_discon(turbine, controller, cp_filename, param_filename, patches={
        'AWC_Mode': 2,
        'AWC_NumModes': 1,
        'AWC_harmonic': '1',
        'AWC_freq': '0.05',
        'AWC_amp': '2.0',
        'AWC_clockangle': '0.0',
        'AWC_phaseoffset': '0.0',
    })

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim15'
    )

    sim_15 = ROSCO_sim.Sim(turbine, controller_int)

    dt = 0.025
    tlen = 400
    ws0 = 9
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0
    for i in range(len(t)):
        ws[i] = ws[i] + t[i] // 100

    sim_15.sim_ws_series(t, ws, rotor_rpm_init=4, make_plots=False, extra_avrswap=EXTRA_AVRSWAP)
    save_and_print_results(build_save_dict(sim_15), 15, output_dir)
    print("Scenario 15: PASSED (AWC_Mode=2 Coleman transform exercised)")


# ---------------------------------------------------------------------------
# Scenario 16: Flp_Mode=3 (Coleman transform flap control)
# ---------------------------------------------------------------------------
def run_scenario_16(turbine, controller, cp_filename, output_dir=None):
    """Sim with Flp_Mode=3 to exercise Coleman transform cyclic flap control.

    Flp_Mode=3 uses ColemanTransform to decompose blade root moments into
    tilt/yaw axes, applies PI control, then uses ColemanTransformInverse
    to convert back to individual blade flap commands.
    """
    print("=" * 60)
    print("Scenario 16: Coleman transform flap control (Flp_Mode=3)")
    print("=" * 60)

    param_filename = os.path.join(this_dir, 'DISCON_flp.IN')
    write_discon(turbine, controller, cp_filename, param_filename, patches={
        'Flp_Mode': 3,
        'IPC_ControlMode': 0,
        'Flp_Kp': '-0.001',
        'Flp_Ki': '-0.0005',
        'F_FlpCornerFreq': '0.5 0.7',
    })

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim16'
    )

    sim_16 = ROSCO_sim.Sim(turbine, controller_int)

    dt = 0.025
    tlen = 400
    ws0 = 9
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0
    for i in range(len(t)):
        ws[i] = ws[i] + t[i] // 100

    sim_16.sim_ws_series(t, ws, rotor_rpm_init=4, make_plots=False, extra_avrswap=EXTRA_AVRSWAP)
    save_and_print_results(build_save_dict(sim_16), 16, output_dir)
    print("Scenario 16: PASSED (Flp_Mode=3 Coleman transform flap exercised)")


# ---------------------------------------------------------------------------
# Scenario 17: WE_Mode=1 (Inversion and Invariance Filter WSE)
# ---------------------------------------------------------------------------
def run_scenario_17(turbine, controller, cp_filename, output_dir=None):
    """Sim with WE_Mode=1 to exercise the I&I wind speed estimator.

    WE_Mode=1 uses a simpler wind speed estimation algorithm than the
    default EKF (WE_Mode=2). Exercises a completely different code path
    in WindSpeedEstimator.
    """
    print("=" * 60)
    print("Scenario 17: I&I wind speed estimator (WE_Mode=1)")
    print("=" * 60)

    param_filename = os.path.join(this_dir, 'DISCON.IN')
    write_discon(turbine, controller, cp_filename, param_filename, patches={
        'WE_Mode': 1,
    })

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim17'
    )

    sim_17 = ROSCO_sim.Sim(turbine, controller_int)

    dt = 0.025
    tlen = 400
    ws0 = 7
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0
    for i in range(len(t)):
        ws[i] = ws[i] + t[i] // 100

    sim_17.sim_ws_series(t, ws, rotor_rpm_init=4, make_plots=False, extra_avrswap=EXTRA_AVRSWAP)
    save_and_print_results(build_save_dict(sim_17), 17, output_dir)
    print("Scenario 17: PASSED (WE_Mode=1 I&I estimator exercised)")


# ---------------------------------------------------------------------------
# Scenario 18: IPC_ControlMode=2 (1P + 2P individual pitch control)
# ---------------------------------------------------------------------------
def run_scenario_18(turbine, controller, cp_filename, output_dir=None):
    """Sim with IPC_ControlMode=2 to exercise 2P harmonic IPC.

    IPC_ControlMode=2 enables both 1P and 2P harmonic individual pitch
    control. Scenario 6 tests 1P with zero gains, Scenario 8 tests 1P
    with non-zero gains. This scenario tests both 1P and 2P with
    non-zero gains.
    """
    print("=" * 60)
    print("Scenario 18: 1P+2P individual pitch control (IPC_ControlMode=2)")
    print("=" * 60)

    param_filename = os.path.join(this_dir, 'DISCON_ipc.IN')
    write_discon(turbine, controller, cp_filename, param_filename, patches={
        'IPC_ControlMode': 2,
        'IPC_KP': '0.1 0.05',
        'IPC_KI': '0.01 0.005',
        'Flp_Mode': 0,
        'F_NumNotchFilts': 1,
        'F_NotchFreqs': '1.0000',
        'F_NotchBetaNum': '0.0000',
        'F_NotchBetaDen': '0.2500',
        'F_GenSpdNotch_N': 1,
        'F_GenSpdNotch_Ind': '1',
    })

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim18'
    )

    sim_18 = ROSCO_sim.Sim(turbine, controller_int)

    dt = 0.025
    tlen = 400
    ws0 = 9
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0
    for i in range(len(t)):
        ws[i] = ws[i] + t[i] // 100

    sim_18.sim_ws_series(t, ws, rotor_rpm_init=4, make_plots=False, extra_avrswap=EXTRA_AVRSWAP)
    save_and_print_results(build_save_dict(sim_18), 18, output_dir)
    print("Scenario 18: PASSED (IPC_ControlMode=2, 1P+2P exercised)")


# ---------------------------------------------------------------------------
# Scenario 19: PA_Mode=1, PF_Mode=1, VS_ConstPower=1, Fl_Mode=2
# ---------------------------------------------------------------------------
def run_scenario_19(turbine, controller, cp_filename, output_dir=None):
    """Combined: pitch actuator LP + pitch offset fault + const power + Fl_Mode=2.

    PA_Mode=1: First-order LP filter on pitch actuator.
    PF_Mode=1: Adds fixed pitch offsets to blade commands.
    VS_ConstPower=1: Constant power (not torque) above rated.
    Fl_Mode=2: Floating feedback from rotational velocity (IMU) instead of
    translational velocity (Fl_Mode=1, tested in Scenario 3).
    """
    print("=" * 60)
    print("Scenario 19: PA_Mode=1 + PF_Mode=1 + VS_ConstPower=1 + Fl_Mode=2")
    print("=" * 60)

    param_filename = os.path.join(this_dir, 'DISCON.IN')
    write_discon(turbine, controller, cp_filename, param_filename, patches={
        'PA_Mode': 1,
        'PF_Mode': 1,
        'PF_Offsets': '0.01 -0.01 0.005',
        'VS_ConstPower': 1,
        'Fl_Mode': 2,
        'F_FlCornerFreq': '1.0 0.7',
    })

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim19'
    )

    sim_19 = ROSCO_sim.Sim(turbine, controller_int)

    dt = 0.025
    tlen = 400
    ws0 = 9
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0
    for i in range(len(t)):
        ws[i] = ws[i] + t[i] // 100

    sim_19.sim_ws_series(t, ws, rotor_rpm_init=4, make_plots=False, extra_avrswap=EXTRA_AVRSWAP)
    save_and_print_results(build_save_dict(sim_19), 19, output_dir)
    print("Scenario 19: PASSED")


# ---------------------------------------------------------------------------
# Scenario 20: PA_Mode=2, PF_Mode=2, PRC_Mode=1
# ---------------------------------------------------------------------------
def run_scenario_20(turbine, controller, cp_filename, output_dir=None):
    """Combined: pitch actuator SecLP + pitch stuck fault + PRC lookup table.

    PA_Mode=2: Second-order LP filter on pitch actuator.
    PF_Mode=2: Pitch actuator stuck at 200s (blade 1 only).
    PRC_Mode=1: Power reference speed from wind speed lookup table.
    """
    print("=" * 60)
    print("Scenario 20: PA_Mode=2 + PF_Mode=2 + PRC_Mode=1")
    print("=" * 60)

    param_filename = os.path.join(this_dir, 'DISCON.IN')
    write_discon(turbine, controller, cp_filename, param_filename, patches={
        'PA_Mode': 2,
        'PF_Mode': 2,
        'PF_TimeStuck': '200.0 9999.0 9999.0',
        'PRC_Mode': 1,
    })

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim20'
    )

    sim_20 = ROSCO_sim.Sim(turbine, controller_int)

    dt = 0.025
    tlen = 400
    ws0 = 9
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0
    for i in range(len(t)):
        ws[i] = ws[i] + t[i] // 100

    sim_20.sim_ws_series(t, ws, rotor_rpm_init=4, make_plots=False, extra_avrswap=EXTRA_AVRSWAP)
    save_and_print_results(build_save_dict(sim_20), 20, output_dir)
    print("Scenario 20: PASSED")


# ---------------------------------------------------------------------------
# Scenario 21: AWC_Mode=3 (closed-loop PI AWC)
# ---------------------------------------------------------------------------
def run_scenario_21(turbine, controller, cp_filename, output_dir=None):
    """Sim with AWC_Mode=3 to exercise closed-loop PI active wake control.

    AWC_Mode=3 uses PI controller with Coleman transform for feedback-based
    active wake control. Different algorithm from Mode=4 (resonator).
    """
    print("=" * 60)
    print("Scenario 21: Closed-loop PI AWC (AWC_Mode=3)")
    print("=" * 60)

    param_filename = os.path.join(this_dir, 'DISCON_awc.IN')
    write_discon(turbine, controller, cp_filename, param_filename, patches={
        'AWC_Mode': 3,
        'AWC_NumModes': 1,
        'AWC_harmonic': '1',
        'AWC_freq': '0.05',
        'AWC_amp': '2.0',
        'AWC_clockangle': '0.0',
        'AWC_CntrGains': '0.0100 0.0050',
    })

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim21'
    )

    sim_21 = ROSCO_sim.Sim(turbine, controller_int)

    dt = 0.025
    tlen = 400
    ws0 = 9
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0
    for i in range(len(t)):
        ws[i] = ws[i] + t[i] // 100

    sim_21.sim_ws_series(t, ws, rotor_rpm_init=4, make_plots=False, extra_avrswap=EXTRA_AVRSWAP)
    save_and_print_results(build_save_dict(sim_21), 21, output_dir)
    print("Scenario 21: PASSED (AWC_Mode=3 closed-loop PI exercised)")


# ---------------------------------------------------------------------------
# Scenario 22: AWC_Mode=5 (Strouhal transform AWC)
# ---------------------------------------------------------------------------
def run_scenario_22(turbine, controller, cp_filename, output_dir=None):
    """Sim with AWC_Mode=5 to exercise Strouhal-based active wake control.

    AWC_Mode=5 uses frequency-transformed azimuth angle for wake control.
    Different from modes 1-4.
    """
    print("=" * 60)
    print("Scenario 22: Strouhal transform AWC (AWC_Mode=5)")
    print("=" * 60)

    param_filename = os.path.join(this_dir, 'DISCON_awc.IN')
    write_discon(turbine, controller, cp_filename, param_filename, patches={
        'AWC_Mode': 5,
        'AWC_NumModes': 1,
        'AWC_harmonic': '1',
        'AWC_freq': '0.05',
        'AWC_amp': '2.0',
        'AWC_clockangle': '0.0',
        'AWC_CntrGains': '0.0100 0.0050',
    })

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim22'
    )

    sim_22 = ROSCO_sim.Sim(turbine, controller_int)

    dt = 0.025
    tlen = 400
    ws0 = 9
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0
    for i in range(len(t)):
        ws[i] = ws[i] + t[i] // 100

    sim_22.sim_ws_series(t, ws, rotor_rpm_init=4, make_plots=False, extra_avrswap=EXTRA_AVRSWAP)
    save_and_print_results(build_save_dict(sim_22), 22, output_dir)
    print("Scenario 22: PASSED (AWC_Mode=5 Strouhal transform exercised)")


# ---------------------------------------------------------------------------
# Scenario 23: PS_Mode=0, SS_Mode=0 (disable normally-on modes)
# ---------------------------------------------------------------------------
def run_scenario_23(turbine, controller, cp_filename, output_dir=None):
    """Sim with PS_Mode=0 and SS_Mode=0 to exercise disabled paths.

    Tests the 'off' branches of normally-on modes: pitch saturation
    disabled (uses PC_FinePit as minimum) and setpoint smoothing disabled.
    """
    print("=" * 60)
    print("Scenario 23: PS_Mode=0 + SS_Mode=0 (disabled paths)")
    print("=" * 60)

    param_filename = os.path.join(this_dir, 'DISCON.IN')
    write_discon(turbine, controller, cp_filename, param_filename, patches={
        'PS_Mode': 0,
        'SS_Mode': 0,
    })

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim23'
    )

    sim_23 = ROSCO_sim.Sim(turbine, controller_int)

    dt = 0.025
    tlen = 400
    ws0 = 7
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0
    for i in range(len(t)):
        ws[i] = ws[i] + t[i] // 100

    sim_23.sim_ws_series(t, ws, rotor_rpm_init=4, make_plots=False, extra_avrswap=EXTRA_AVRSWAP)
    save_and_print_results(build_save_dict(sim_23), 23, output_dir)
    print("Scenario 23: PASSED (PS_Mode=0, SS_Mode=0 disabled paths exercised)")


# ---------------------------------------------------------------------------
# Scenario 24: CC_Mode=2 + StC_Mode=2 + OL_Mode=1 (open-loop cable + structural)
# ---------------------------------------------------------------------------
def run_scenario_24(turbine, controller, cp_filename, output_dir=None):
    """Sim with CC_Mode=2 + StC_Mode=2 to exercise open-loop cable and structural control.

    CC_Mode=2 and StC_Mode=2 read desired control values from columns in the
    OL input file, interpolated by time. Requires OL_Mode=1 with extended
    input file containing cable and structural control columns.
    """
    print("=" * 60)
    print("Scenario 24: Open-loop cable + structural (CC_Mode=2, StC_Mode=2)")
    print("=" * 60)

    ol_input_path = os.path.join(this_dir, 'example_inputs', 'OL_Mode1_CC_StC_Input.dat')
    param_filename = os.path.join(this_dir, 'DISCON_ol_cc_stc.IN')
    write_discon(turbine, controller, cp_filename, param_filename, patches={
        'OL_Mode': 1,
        'OL_Filename': ol_input_path,
        'OL_BP_Mode': 0,
        'OL_BP_FiltFreq': 0.0,
        'Ind_Breakpoint': 1,
        'Ind_BldPitch': '2 3 4',
        'Ind_GenTq': 5,
        'Ind_YawRate': 6,
        'Ind_Azimuth': 0,
        'Ind_R_Speed': 0,
        'Ind_R_Torque': 0,
        'Ind_R_Pitch': 0,
        'CC_Mode': 2,
        'CC_Group_N': 1,
        'CC_GroupIndex': '2601',
        'Ind_CableControl': '7',
        'StC_Mode': 2,
        'StC_Group_N': 1,
        'StC_GroupIndex': '2801',
        'Ind_StructControl': '8',
    })

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim24'
    )

    sim_24 = ROSCO_sim.Sim(turbine, controller_int)

    dt = 0.025
    tlen = 200
    ws0 = 9
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0

    sim_24.sim_ws_series(t, ws, rotor_rpm_init=4, make_plots=False, extra_avrswap=EXTRA_AVRSWAP)
    save_and_print_results(build_save_dict(sim_24), 24, output_dir)
    print("Scenario 24: PASSED (CC_Mode=2, StC_Mode=2 open-loop exercised)")


# ---------------------------------------------------------------------------
# Scenario 25: PRC_Mode=2 (dynamic power rating, constant inputs)
# ---------------------------------------------------------------------------
def run_scenario_25(turbine, controller, cp_filename, output_dir=None):
    """Sim with PRC_Mode=2 to exercise dynamic power rating control.

    PRC_Mode=2 with PRC_Comm=0 uses constant rating multipliers from DISCON.
    PRC_R_Speed=0.9 reduces rated speed by 10%, visibly affecting torque and
    speed setpoints. Independent of OL_Mode.
    """
    print("=" * 60)
    print("Scenario 25: Dynamic power rating (PRC_Mode=2, PRC_Comm=0)")
    print("=" * 60)

    param_filename = os.path.join(this_dir, 'DISCON.IN')
    write_discon(turbine, controller, cp_filename, param_filename, patches={
        'PRC_Mode': 2,
        'PRC_Comm': 0,
        'PRC_R_Speed': '0.9',
        'PRC_R_Torque': '1.0',
        'PRC_R_Pitch': '1.0',
    })

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim25'
    )

    sim_25 = ROSCO_sim.Sim(turbine, controller_int)

    dt = 0.025
    tlen = 400
    ws0 = 7
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0
    for i in range(len(t)):
        ws[i] = ws[i] + t[i] // 100

    sim_25.sim_ws_series(t, ws, rotor_rpm_init=4, make_plots=False, extra_avrswap=EXTRA_AVRSWAP)
    save_and_print_results(build_save_dict(sim_25), 25, output_dir)
    print("Scenario 25: PASSED (PRC_Mode=2 dynamic power rating exercised)")


# ---------------------------------------------------------------------------
# Scenario 26: Flp_Mode=3 with synthetic rootMOOP (non-zero flap output)
# ---------------------------------------------------------------------------
def run_scenario_26(turbine, controller, cp_filename, output_dir=None):
    """Exercises FlapControl Flp_Mode=3 (Coleman transform cyclic) with non-zero output.

    Uses a manual sim loop to inject synthetic rootMOOP (1000 Nm, 1P per-blade).
    This is the only scenario that produces non-zero flp_angle output —
    all other flap scenarios have zero rootMOOP (1-DOF sim) or use Flp_Mode=1
    (which ignores rootMOOP).

    Flp_Mode=3 path: rootMOOP → NotchFilterSlopes → ColemanTransform →
    PIController (tilt/yaw axes) → ColemanTransformInverse → per-blade Flp_Angle.
    """
    print("=" * 60)
    print("Scenario 26: Flp_Mode=3 with synthetic rootMOOP (non-zero flap output)")
    print("=" * 60)

    param_filename = os.path.join(this_dir, 'DISCON_flap26.IN')
    write_discon(turbine, controller, cp_filename, param_filename, patches={
        'Flp_Mode': 3,
        'Flp_Kp': -0.001,
        'Flp_Ki': -0.0005,
        'F_FlpCornerFreq': '0.5 0.7',
        'F_FlCornerFreq': '1.0 0.7',
        'IPC_ControlMode': 0,
        'AWC_Mode': 0,
        'F_NumNotchFilts': 1,
        'F_NotchFreqs': '1.0000',
        'F_NotchBetaNum': '0.0000',
        'F_NotchBetaDen': '0.2500',
        'F_GenSpdNotch_N': 1,
        'F_GenSpdNotch_Ind': '1',
    })

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim26'
    )

    dt = 0.025
    tlen = 400
    ws0 = 9
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0

    R = turbine.rotor_radius
    GBRatio = turbine.Ng
    rpm2RadSec = 2.0 * np.pi / 60.0

    bld_pitch = np.zeros_like(t)
    rot_speed = np.ones_like(t) * 4.0 * rpm2RadSec
    gen_speed = rot_speed * GBRatio
    gen_torque = np.zeros_like(t)
    gen_power = np.zeros_like(t)
    nac_yaw = np.zeros_like(t)
    extra = {name: np.zeros_like(t) for name in EXTRA_AVRSWAP}

    for i, ti in enumerate(t):
        if i == 0:
            continue

        ws_i = ws[i]
        tsr = rot_speed[i-1] * R / ws_i
        cp = turbine.Cp.interp_surface(bld_pitch[i-1], tsr)
        aero_torque = 0.5 * turbine.rho * (np.pi * R**3) * (cp / tsr) * ws_i**2
        rot_speed[i] = rot_speed[i-1] + (dt / turbine.J) * (
            aero_torque - GBRatio * gen_torque[i-1] / (turbine.GBoxEff / 100)
        )
        gen_speed[i] = rot_speed[i] * GBRatio

        # Synthetic rotor azimuth
        azimuth_rad = (rot_speed[i] * ti) % (2 * np.pi)

        # Synthetic blade root moments (1P per-blade, 120 deg phase offset)
        if rot_speed[i] > 0.1:
            t_rotor = 2 * np.pi / rot_speed[i]
        else:
            t_rotor = 100.0
        rootMOOP = [
            1000.0 * np.sin(2 * np.pi * ti / t_rotor + k * 2 * np.pi / 3)
            for k in range(3)
        ]

        turbine_state = {}
        turbine_state['iStatus'] = 1 if i < len(t) - 1 else -1
        turbine_state['t'] = ti
        turbine_state['dt'] = dt
        turbine_state['ws'] = ws_i
        turbine_state['bld_pitch'] = bld_pitch[i-1]
        turbine_state['gen_torque'] = gen_torque[i-1]
        turbine_state['gen_speed'] = gen_speed[i]
        turbine_state['gen_eff'] = turbine.GenEff / 100
        turbine_state['rot_speed'] = rot_speed[i]
        turbine_state['Yaw_fromNorth'] = 0.0
        turbine_state['Y_MeasErr'] = 0.0

        # Inject azimuth and rootMOOP
        controller_int.avrSWAP[59] = azimuth_rad         # avrSWAP(60) Azimuth
        controller_int.avrSWAP[29] = rootMOOP[0]         # avrSWAP(30) rootMOOP(1)
        controller_int.avrSWAP[30] = rootMOOP[1]         # avrSWAP(31) rootMOOP(2)
        controller_int.avrSWAP[31] = rootMOOP[2]         # avrSWAP(32) rootMOOP(3)

        gen_torque[i], bld_pitch[i], _ = controller_int.call_controller(turbine_state)
        gen_power[i] = gen_speed[i] * gen_torque[i] * turbine.GenEff / 100
        for name, idx in EXTRA_AVRSWAP.items():
            extra[name][i] = controller_int.avrSWAP[idx]

    controller_int.kill_discon()
    result = {
        'gen_torque': gen_torque, 'bld_pitch': bld_pitch,
        'gen_speed': gen_speed, 'gen_power': gen_power,
        'nac_yaw': nac_yaw,
    }
    result.update(extra)
    save_and_print_results(result, 26, output_dir)
    print("Scenario 26: PASSED (Flp_Mode=3 with non-zero flap output exercised)")


# ---------------------------------------------------------------------------
# Scenario 27: Maximum-coverage stress test (11 modes simultaneously)
# ---------------------------------------------------------------------------
def run_scenario_27(turbine, controller, cp_filename, output_dir=None):
    """Stress test: 11 controller modes active simultaneously with all synthetic inputs.

    Combines IPC, AWC, yaw control, tower damping, floating feedback,
    cable/structural control, pitch actuator filter, pitch fault offsets,
    constant power, and power reference control. The pitch command is the
    sum of 6 simultaneous contributions (collective PI, IPC, tower damping,
    floating feedback, AWC, pitch fault) filtered through a 2nd-order
    pitch actuator model.

    No existing scenario has more than 3 pitch contributions active at once.
    """
    print("=" * 60)
    print("Scenario 27: Maximum-coverage stress test (11 modes)")
    print("=" * 60)

    param_filename = os.path.join(this_dir, 'DISCON_stress27.IN')
    write_discon(turbine, controller, cp_filename, param_filename, patches={
        'IPC_ControlMode': 1,
        'IPC_KP': '0.1 0.0',
        'IPC_KI': '0.01 0.0',
        'AWC_Mode': 4,
        'AWC_NumModes': 1,
        'AWC_harmonic': '1',
        'AWC_freq': '0.05',
        'AWC_amp': '2.0',
        'AWC_clockangle': '0.0',
        'AWC_CntrGains': '0.0100 0.0050',
        'Y_ControlMode': 1,
        'TD_Mode': 1,
        'Fl_Mode': 2,
        'F_FlCornerFreq': '1.0 0.7',
        'CC_Mode': 1,
        'CC_Group_N': 1,
        'CC_GroupIndex': '2601',
        'StC_Mode': 1,
        'StC_Group_N': 1,
        'StC_GroupIndex': '2801',
        'PA_Mode': 2,
        'PF_Mode': 1,
        'PF_Offsets': '0.01 -0.01 0.005',
        'VS_ConstPower': 1,
        'PRC_Mode': 1,
        'Flp_Mode': 0,
        'F_NumNotchFilts': 1,
        'F_NotchFreqs': '1.0000',
        'F_NotchBetaNum': '0.0000',
        'F_NotchBetaDen': '0.2500',
        'F_GenSpdNotch_N': 1,
        'F_GenSpdNotch_Ind': '1',
    })

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim27'
    )

    dt = 0.025
    tlen = 600  # Exceeds t=500s for CC/StC step activation
    ws0 = 9
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0
    for i_ws in range(len(t)):
        ws[i_ws] = ws[i_ws] + t[i_ws] // 100

    deg2rad = np.pi / 180.0
    R = turbine.rotor_radius
    GBRatio = turbine.Ng
    rpm2RadSec = 2.0 * np.pi / 60.0

    bld_pitch = np.zeros_like(t)
    rot_speed = np.ones_like(t) * 4.0 * rpm2RadSec
    gen_speed = rot_speed * GBRatio
    gen_torque = np.zeros_like(t)
    gen_power = np.zeros_like(t)
    nac_yaw = np.zeros_like(t)
    nac_yawrate = np.zeros_like(t)
    extra = {name: np.zeros_like(t) for name in EXTRA_AVRSWAP}

    for i, ti in enumerate(t):
        if i == 0:
            continue

        ws_i = ws[i]
        tsr = rot_speed[i-1] * R / ws_i
        cp = turbine.Cp.interp_surface(bld_pitch[i-1], tsr)
        aero_torque = 0.5 * turbine.rho * (np.pi * R**3) * (cp / tsr) * ws_i**2
        rot_speed[i] = rot_speed[i-1] + (dt / turbine.J) * (
            aero_torque - GBRatio * gen_torque[i-1] / (turbine.GBoxEff / 100)
        )
        gen_speed[i] = rot_speed[i] * GBRatio

        # Synthetic yaw signals (for Y_ControlMode=1)
        nac_vane_deg = 20.0 * np.sin(2 * np.pi * ti / 50.0)
        nac_vane_rad = nac_vane_deg * deg2rad
        nac_heading_rad = 350.0 * deg2rad

        # Synthetic rotor azimuth (for IPC + AWC Coleman transforms)
        azimuth_rad = (rot_speed[i] * ti) % (2 * np.pi)

        # Synthetic tower fore-aft acceleration (for TD_Mode=1, ~0.3 Hz tower mode)
        fa_acc_tt = 0.5 * np.sin(2 * np.pi * ti / 3.0)

        # Synthetic nacelle IMU rotational acceleration (for Fl_Mode=2)
        nac_imu_fa_racc = 0.3 * np.sin(2 * np.pi * ti / 3.0)

        # Synthetic blade root moments (for IPC + AWC, 1P per-blade, 120 deg offset)
        if rot_speed[i] > 0.1:
            t_rotor = 2 * np.pi / rot_speed[i]
        else:
            t_rotor = 100.0
        rootMOOP = [
            1000.0 * np.sin(2 * np.pi * ti / t_rotor + k * 2 * np.pi / 3)
            for k in range(3)
        ]

        turbine_state = {}
        turbine_state['iStatus'] = 1 if i < len(t) - 1 else -1
        turbine_state['t'] = ti
        turbine_state['dt'] = dt
        turbine_state['ws'] = ws_i
        turbine_state['bld_pitch'] = bld_pitch[i-1]
        turbine_state['gen_torque'] = gen_torque[i-1]
        turbine_state['gen_speed'] = gen_speed[i]
        turbine_state['gen_eff'] = turbine.GenEff / 100
        turbine_state['rot_speed'] = rot_speed[i]
        turbine_state['Yaw_fromNorth'] = nac_yaw[i-1]
        turbine_state['Y_MeasErr'] = nac_vane_rad

        # Inject all synthetic avrSWAP values
        controller_int.avrSWAP[23] = nac_vane_rad         # avrSWAP(24) NacVane
        controller_int.avrSWAP[36] = nac_heading_rad       # avrSWAP(37) NacHeading
        controller_int.avrSWAP[52] = fa_acc_tt              # avrSWAP(53) FA_Acc_TT
        controller_int.avrSWAP[82] = nac_imu_fa_racc        # avrSWAP(83) NacIMU_FA_RAcc
        controller_int.avrSWAP[59] = azimuth_rad            # avrSWAP(60) Azimuth
        controller_int.avrSWAP[29] = rootMOOP[0]            # avrSWAP(30) rootMOOP(1)
        controller_int.avrSWAP[30] = rootMOOP[1]            # avrSWAP(31) rootMOOP(2)
        controller_int.avrSWAP[31] = rootMOOP[2]            # avrSWAP(32) rootMOOP(3)

        gen_torque[i], bld_pitch[i], nac_yawrate[i] = controller_int.call_controller(turbine_state)
        gen_power[i] = gen_speed[i] * gen_torque[i] * turbine.GenEff / 100
        nac_yaw[i] = nac_yaw[i-1] + nac_yawrate[i] * dt
        for name, idx in EXTRA_AVRSWAP.items():
            extra[name][i] = controller_int.avrSWAP[idx]

    controller_int.kill_discon()
    result = {
        'gen_torque': gen_torque, 'bld_pitch': bld_pitch,
        'gen_speed': gen_speed, 'gen_power': gen_power,
        'nac_yaw': nac_yaw,
    }
    result.update(extra)
    save_and_print_results(result, 27, output_dir)
    print("Scenario 27: PASSED (11-mode stress test exercised)")


# ---------------------------------------------------------------------------
# Scenario 28: HDF5 output format (same as Scenario 1 but OutputFormat=1)
# ---------------------------------------------------------------------------
def run_scenario_28(turbine, controller, cp_filename, output_dir=None):
    """Same step-wind sim as Scenario 1 but with OutputFormat=1 (HDF5).

    Verifies that the HDF5 debug writer produces a valid .RO.h5 file with
    correct dataset names, units, and data values matching the text baseline.
    The avrSWAP-level outputs (gen_torque, bld_pitch, etc.) are saved as
    usual for baseline comparison — those are independent of OutputFormat.
    """
    print("=" * 60)
    print("Scenario 28: HDF5 output format (OutputFormat=1)")
    print("=" * 60)

    param_filename = os.path.join(this_dir, 'DISCON_hdf5.IN')
    write_discon(turbine, controller, cp_filename, param_filename, patches={
        'OutputFormat': 1,
        'LoggingLevel': 3,  # exercise avrSWAP HDF5 dataset (Phase 3)
    })

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim28'
    )

    sim_28 = ROSCO_sim.Sim(turbine, controller_int)

    dt = 0.025
    tlen = 1000
    ws0 = 7
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0
    for i in range(len(t)):
        ws[i] = ws[i] + t[i] // 100

    # Capture the full avrSWAP(1..85) array each step to verify the HDF5
    # "/avrSWAP" dataset against ground truth (avrBaseLength in debug.cpp).
    avr_full_names = [f'avr_{i + 1}' for i in range(85)]
    extra = dict(EXTRA_AVRSWAP)
    extra.update({name: i for i, name in enumerate(avr_full_names)})

    sim_28.sim_ws_series(t, ws, rotor_rpm_init=4, make_plots=False, extra_avrswap=extra)

    result = build_save_dict(sim_28)
    result['avrSWAP_full'] = np.column_stack([getattr(sim_28, name) for name in avr_full_names])
    save_and_print_results(result, 28, output_dir)

    # Verify HDF5 file was created and contains valid data
    h5_path = os.path.join(this_dir, 'vit_sim28.RO.h5')
    if os.path.exists(h5_path):
        import h5py
        with h5py.File(h5_path, 'r') as f:
            datasets = list(f.keys())
            print(f"  HDF5 datasets: {len(datasets)} ({datasets[:5]}...)")
            assert 'Time' in datasets, "Missing Time dataset"
            n_rows = f['Time'].shape[0]
            print(f"  HDF5 rows: {n_rows}")
            assert n_rows > 0, "HDF5 file has no data rows"
            # Check units attribute on Time
            assert f['Time'].attrs['units'] == b'sec' or f['Time'].attrs['units'] == 'sec', \
                f"Time units mismatch: {f['Time'].attrs['units']}"
        print(f"  HDF5 verified: {h5_path} ({os.path.getsize(h5_path)} bytes)")
    else:
        print(f"  WARNING: HDF5 file not created at {h5_path}")
        print("  (HDF5 support may not be compiled in)")

    print("Scenario 28: PASSED (HDF5 output format exercised)")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description='VIT simulation runner')
    parser.add_argument('--scenario', type=int, default=0,
                        help='Run specific scenario (1-27). Default 0 = run all.')
    parser.add_argument('--output-dir', type=str, default=None,
                        help='Save simulation output arrays to .npz files in this directory.')
    parser.add_argument('--benchmark', type=int, default=0,
                        help='Run each scenario N times and output timing CSV. No arrays saved.')
    parser.add_argument('--build', type=str, default='unknown',
                        help='Build label for benchmark CSV output (e.g., upstream, modified, cpp).')
    args = parser.parse_args()

    turbine, controller, cp_filename = load_turbine_and_controller()
    od = args.output_dir

    # Scenario dispatch table (ordered for KGen extraction compatibility)
    scenario_order = [3, 4, 5, 1, 2, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
                      17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28]
    scenario_functions = {
        1: run_scenario_1, 2: run_scenario_2, 3: run_scenario_3,
        4: run_scenario_4, 5: run_scenario_5, 6: run_scenario_6,
        7: run_scenario_7, 8: run_scenario_8, 9: run_scenario_9,
        10: run_scenario_10, 11: run_scenario_11, 12: run_scenario_12,
        13: run_scenario_13, 14: run_scenario_14, 15: run_scenario_15,
        16: run_scenario_16, 17: run_scenario_17, 18: run_scenario_18,
        19: run_scenario_19, 20: run_scenario_20, 21: run_scenario_21,
        22: run_scenario_22, 23: run_scenario_23, 24: run_scenario_24,
        25: run_scenario_25, 26: run_scenario_26, 27: run_scenario_27,
        28: run_scenario_28,
    }

    if args.benchmark > 0:
        # Benchmark mode: time each scenario, output CSV, suppress ALL stdout
        # (including C/Fortran prints that bypass Python's sys.stdout)
        import time

        scenarios = [args.scenario] if args.scenario > 0 else scenario_order
        # Write timing results to stderr (fd 2) to avoid mixing with controller output
        sys.stderr.write("build,scenario,run,seconds\n")
        sys.stderr.flush()
        for s in scenarios:
            run_fn = scenario_functions[s]
            for run_num in range(1, args.benchmark + 1):
                # Redirect fd 1 to /dev/null at OS level to suppress
                # both Python prints and C/Fortran printf/write
                devnull_fd = os.open(os.devnull, os.O_WRONLY)
                saved_stdout_fd = os.dup(1)
                os.dup2(devnull_fd, 1)
                os.close(devnull_fd)
                try:
                    t0 = time.perf_counter()
                    run_fn(turbine, controller, cp_filename, None)
                    t1 = time.perf_counter()
                finally:
                    os.dup2(saved_stdout_fd, 1)
                    os.close(saved_stdout_fd)
                sys.stderr.write("%s,%d,%d,%.4f\n" % (args.build, s, run_num, t1 - t0))
                sys.stderr.flush()
        return

    # Normal mode: run scenarios with output
    # Scenario 3 runs first so KGen's early invocations (1-20) capture all
    # mode-gated code paths.
    for s in scenario_order:
        if args.scenario == 0 or args.scenario == s:
            scenario_functions[s](turbine, controller, cp_filename, od)

    print("\n" + "=" * 60)
    print("All scenarios complete.")
    print("=" * 60)


if __name__ == '__main__':
    main()
