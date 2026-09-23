"""
scenarios.py
------------
Simulation scenarios for the ROSCO C++ controller regression suite.

Each scenario drives the compiled ``libdiscon`` through a different set of
controller modes and captures the resulting avrSWAP time series. The arrays are
compared bit-for-bit against the frozen baselines in ``baselines/`` — see
``README.md``.

Scenario numbers are stable: they are referenced by baseline filenames,
``REFACTOR_NOTES.md``, and commit history. Never renumber an existing scenario.

Normally driven by ``run_regression.py``, which runs each scenario in its own
subprocess (the DLL keeps static state) from a scratch working directory.
Invoking this module directly writes its generated ``DISCON_*.IN`` and the
controller's ``*.RO.dbg*`` output into the *current* directory:

    cd $(mktemp -d)
    python /path/to/test/regression/scenarios.py --scenario 1
"""

import argparse
import hashlib
import os
import re
import sys
from dataclasses import dataclass, field
from typing import Callable

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
        np.savez_compressed(path, **arrays)
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
REPO_ROOT = os.path.dirname(os.path.dirname(this_dir))
EXAMPLES_DIR = os.path.join(REPO_ROOT, 'Examples')
TUNE_DIR = os.path.join(EXAMPLES_DIR, 'Tune_Cases')
EXAMPLE_INPUTS_DIR = os.path.join(EXAMPLES_DIR, 'example_inputs')
FIXTURES_DIR = os.path.join(this_dir, 'fixtures')

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
_scrub_lib_path = os.path.join(REPO_ROOT, 'rosco', 'lib', 'libscrub.so')
if os.path.exists(_scrub_lib_path):
    _scrub_lib = _ctypes.CDLL(_scrub_lib_path)
    def _scrubbed_interp_surface(self, pitch, TSR):
        if not hasattr(self, '_cached_surface_spline'):
            self._cached_surface_spline = _interpolate.RectBivariateSpline(
                self.pitch_initial_rad, self.TSR_initial, self.performance_table.T)
        _scrub_lib.scrub_stack()
        return np.squeeze(self._cached_surface_spline(pitch, TSR).T)
    def _scrubbed_interp_gradient(self, pitch, TSR):
        if not hasattr(self, '_cached_grad_pitch_spline'):
            self._cached_grad_pitch_spline = _interpolate.RectBivariateSpline(
                self.pitch_initial_rad, self.TSR_initial, self.gradient_pitch.T)
            self._cached_grad_TSR_spline = _interpolate.RectBivariateSpline(
                self.pitch_initial_rad, self.TSR_initial, self.gradient_TSR.T)
        _scrub_lib.scrub_stack()
        grad = np.array([self._cached_grad_pitch_spline(pitch, TSR).T,
                         self._cached_grad_TSR_spline(pitch, TSR).T])
        return np.ndarray.flatten(grad)
    ROSCO_turbine.RotorPerformance.interp_surface = _scrubbed_interp_surface
    ROSCO_turbine.RotorPerformance.interp_gradient = _scrubbed_interp_gradient


def load_turbine_only():
    """Load just the turbine (the plant model). No tuning — the DISCON fixtures are
    committed, so `tune_controller()` is not on the regression's critical path."""
    inps = load_rosco_yaml(os.path.join(TUNE_DIR, 'NREL5MW.yaml'))
    path_params = inps['path_params']
    turbine = ROSCO_turbine.Turbine(inps['turbine_params'])
    cp_filename = os.path.join(TUNE_DIR, path_params['rotor_performance_filename'])
    turbine.load_from_fast(
        path_params['FAST_InputFile'],
        os.path.join(TUNE_DIR, path_params['FAST_directory']),
        rot_source='txt', txt_filename=cp_filename
    )
    return turbine, cp_filename


def load_turbine_and_controller():
    """Load the NREL5MW turbine and tune a ROSCO controller. Returns (turbine, controller)."""
    parameter_filename = os.path.join(TUNE_DIR, 'NREL5MW.yaml')
    inps = load_rosco_yaml(parameter_filename)
    path_params = inps['path_params']
    controller_params = inps['controller_params']

    turbine = ROSCO_turbine.Turbine(inps['turbine_params'])

    cp_filename = os.path.join(TUNE_DIR, path_params['rotor_performance_filename'])
    turbine.load_from_fast(
        path_params['FAST_InputFile'],
        os.path.join(TUNE_DIR, path_params['FAST_directory']),
        rot_source='txt', txt_filename=cp_filename
    )

    controller = ROSCO_controller.Controller(controller_params)
    controller.tune_controller(turbine)

    return turbine, controller, cp_filename


# Parameters whose value is a path. The controller resolves a relative value against
# the directory of the DISCON file itself (priPath in readcontrolparameterfilesub.cpp),
# so fixtures store these relative and stay portable across machines and CI.
PATH_PARAMS = ('PerfFileName', 'OL_Filename')


# Scenario 1 applies no patches, so its fixture *is* the raw tuner output. That is
# what test_tuning.py pins, so the tuning check and the controller check share one
# artifact rather than two copies free to drift apart.
TUNER_FIXTURE = os.path.join(FIXTURES_DIR, 'scenario_01.IN')


def fixture_path(num):
    return os.path.join(FIXTURES_DIR, f'scenario_{num:02d}.IN')


def write_tuner_output(turbine, controller, cp_filename, path):
    """Write the unpatched tuner output to `path`.

    Paths inside the file are always made relative to FIXTURES_DIR, not to `path`,
    so writing to a temp location still produces a file byte-comparable with the
    committed fixture.
    """
    os.makedirs(os.path.dirname(path), exist_ok=True)
    write_DISCON(turbine, controller, param_file=path, txt_filename=cp_filename)
    with open(path) as f:
        text = f.read()
    with open(path, 'w') as f:
        f.write(_make_portable(text, FIXTURES_DIR))
    return path


def apply_patches(text, patches):
    """A fixture's text: `text` (the scenario 1 fixture) with `patches` applied and
    any path a patch introduces made relative to FIXTURES_DIR.

    This is both how `--write-fixtures` builds each fixture and what
    test_fixtures.py checks the committed files against.
    """
    for param, value in patches.items():
        # Match lines like "0                   ! Y_ControlMode   - description"
        # Also handles multi-value lines like "0.0 0.0   ! AWC_CntrGains ..."
        pattern = rf'^(.+?)(\s+! {param}\b.*)$'
        text, count = re.subn(pattern, rf'{value}\2', text, flags=re.MULTILINE)
        if count == 0:
            raise ValueError(f"Could not patch {param}: no such parameter line")
    return _make_portable(text, FIXTURES_DIR)


def write_fixtures():
    """Regenerate every fixture: tune once, then apply each scenario's patches."""
    turbine, controller, cp_filename = load_turbine_and_controller()
    write_tuner_output(turbine, controller, cp_filename, TUNER_FIXTURE)
    with open(TUNER_FIXTURE) as f:
        base = f.read()
    for num, s in SCENARIOS.items():
        if num == 1:
            continue
        with open(fixture_path(num), 'w') as f:
            f.write(apply_patches(base, s.patches))
    print(f"Wrote {len(SCENARIOS)} fixtures to {FIXTURES_DIR}")


def _make_portable(text, fixture_dir):
    """Strip the two things that make tuner output unfit to commit.

    1. `write_DISCON` stamps the toolbox version and *today's date* into line 2.
       Left in, every regeneration would dirty all 28 fixtures and a real tuner
       change would be invisible among the noise. Git already records when each
       fixture changed, so the stamp is redundant as well as harmful.
    2. Path parameters are written absolute. The controller resolves a relative
       value against the DISCON file's own directory, so relative is portable.
    """
    text = re.sub(
        r'^!\s+- File written using ROSCO version .*$',
        '!    - Generated by test/regression/scenarios.py --write-fixtures — do not hand-edit',
        text, flags=re.MULTILINE)

    for param in PATH_PARAMS:
        pattern = rf'^(\s*)(\S+?)(\s+! {param}\b.*)$'

        def repl(m):
            value = m.group(2).strip('"')
            if not os.path.isabs(value):
                return m.group(0)
            rel = os.path.relpath(value, fixture_dir).replace(os.sep, '/')
            quoted = f'"{rel}"' if m.group(2).startswith('"') else rel
            return f'{m.group(1)}{quoted}{m.group(3)}'

        text = re.sub(pattern, repl, text, flags=re.MULTILINE)
    return text


# ---------------------------------------------------------------------------
# Runners
# ---------------------------------------------------------------------------
DT = 0.025
RPM2RADSEC = 2.0 * np.pi / 60.0


def _controller(num, suffix=''):
    # DT must match the timestep the scenario then simulates at. The toolbox
    # defaults to 0.1, and the controller's first call (iStatus 0) is what sizes
    # every filter's coefficients — leaving the default would tune them for a
    # timestep the simulation never uses.
    return ROSCO_ci.ControllerInterface(
        lib_name, param_filename=fixture_path(num), sim_name=f'regression_{num}{suffix}',
        DT=DT,
    )


def run_sim(s, turbine):
    """The toolbox 1-DOF simulation. Most scenarios need nothing else."""
    t, ws = s.wind()
    sim = ROSCO_sim.Sim(turbine, _controller(s.num))
    sim.sim_ws_series(t, ws, rotor_rpm_init=4, make_plots=False, extra_avrswap=EXTRA_AVRSWAP)
    return build_save_dict(sim)


def run_twice(s, turbine):
    """Scenario 1: run, then run again in a fresh ControllerInterface. The second
    run must match the first, which checks the library deallocates cleanly."""
    t, ws = s.wind()
    sim = ROSCO_sim.Sim(turbine, _controller(s.num))
    sim.sim_ws_series(t, ws, rotor_rpm_init=4, make_plots=False, extra_avrswap=EXTRA_AVRSWAP)
    again = ROSCO_sim.Sim(turbine, _controller(s.num, suffix='b'))
    again.sim_ws_series(t, ws, rotor_rpm_init=4, make_plots=False, extra_avrswap=EXTRA_AVRSWAP)
    np.testing.assert_almost_equal(sim.gen_speed, again.gen_speed)
    return build_save_dict(sim)


def run_hdf5(s, turbine):
    """Scenario 28: the scenario 1 simulation, also capturing avrSWAP(1..85) every
    step as ground truth for the HDF5 "/avrSWAP" dataset (avrBaseLength in
    debug.cpp), then sanity-checking the .RO.h5 file."""
    t, ws = s.wind()
    sim = ROSCO_sim.Sim(turbine, _controller(s.num))
    avr_full_names = [f'avr_{i + 1}' for i in range(85)]
    extra = dict(EXTRA_AVRSWAP)
    extra.update({name: i for i, name in enumerate(avr_full_names)})
    sim.sim_ws_series(t, ws, rotor_rpm_init=4, make_plots=False, extra_avrswap=extra)

    result = build_save_dict(sim)
    result['avrSWAP_full'] = np.column_stack([getattr(sim, name) for name in avr_full_names])

    h5_path = os.path.abspath(f'regression_{s.num}.RO.h5')
    if os.path.exists(h5_path):
        import h5py
        with h5py.File(h5_path, 'r') as f:
            datasets = list(f.keys())
            print(f"  HDF5 datasets: {len(datasets)} ({datasets[:5]}...)")
            assert 'Time' in datasets, "Missing Time dataset"
            n_rows = f['Time'].shape[0]
            print(f"  HDF5 rows: {n_rows}")
            assert n_rows > 0, "HDF5 file has no data rows"
            assert f['Time'].attrs['units'] in (b'sec', 'sec'), \
                f"Time units mismatch: {f['Time'].attrs['units']}"
        print(f"  HDF5 verified: {h5_path} ({os.path.getsize(h5_path)} bytes)")
    else:
        print(f"  WARNING: HDF5 file not created at {h5_path}")
        print("  (HDF5 support may not be compiled in)")
    return result


def run_synthetic(s, turbine):
    """A hand-written 1-DOF loop that feeds the controller signals the toolbox
    simulation leaves at zero. `s.synthetic` selects them:

      azimuth    rotor azimuth from the simulated speed -> avrSWAP(60)
      yaw_by_ipc 20 deg vane oscillation as the yaw error, with the heading
                 ramping -45 to 405 deg, so heading + vane spans <0, 0-360 and
                 >=360 — the three branches of wrap_360 that ipc.cpp feeds
      root_moop  1000 N·m, 1P per blade, 120° apart -> avrSWAP(30..32)
      tower      fore-aft tower-top and nacelle IMU accelerations, 1/3 Hz
      yaw_rate   20° vane oscillation as the yaw error; the commanded yaw rate
                 is integrated into nac_yaw and fed back as the heading

    Signals reach avrSWAP only through `turbine_state` or through indices
    call_controller() does not itself write: it overwrites avrSWAP(24), (37),
    (53) and (83) from `turbine_state` on every call.
    """
    t, ws = s.wind()
    ci = _controller(s.num)
    inputs = set(s.synthetic)

    deg2rad = np.pi / 180.0
    R = turbine.rotor_radius
    GBRatio = turbine.Ng

    bld_pitch = np.zeros_like(t)
    rot_speed = np.ones_like(t) * 4.0 * RPM2RADSEC
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
        rot_speed[i] = rot_speed[i-1] + (DT / turbine.J) * (
            aero_torque - GBRatio * gen_torque[i-1] / (turbine.GBoxEff / 100)
        )
        gen_speed[i] = rot_speed[i] * GBRatio

        turbine_state = {
            'iStatus': 1 if i < len(t) - 1 else -1,
            't': ti,
            'dt': DT,
            'ws': ws_i,
            'bld_pitch': bld_pitch[i-1],
            'gen_torque': gen_torque[i-1],
            'gen_speed': gen_speed[i],
            'gen_eff': turbine.GenEff / 100,
            'rot_speed': rot_speed[i],
            'Yaw_fromNorth': 0.0,
            'Y_MeasErr': 0.0,
        }
        if 'yaw_rate' in inputs:
            turbine_state['Yaw_fromNorth'] = nac_yaw[i-1]
            turbine_state['Y_MeasErr'] = 20.0 * np.sin(2 * np.pi * ti / 50.0) * deg2rad
        if 'yaw_by_ipc' in inputs:
            turbine_state['Y_MeasErr'] = 20.0 * np.sin(2 * np.pi * ti / 50.0) * deg2rad
            turbine_state['Yaw_fromNorth'] = (-45.0 + 450.0 * ti / t[-1]) * deg2rad
        if 'tower' in inputs:
            turbine_state['FA_Acc_TT'] = 0.5 * np.sin(2 * np.pi * ti / 3.0)
            turbine_state['NacIMU_FA_RAcc'] = 0.3 * np.sin(2 * np.pi * ti / 3.0)

        if 'azimuth' in inputs:
            ci.avrSWAP[59] = (rot_speed[i] * ti) % (2 * np.pi)  # avrSWAP(60)
        if 'root_moop' in inputs:
            t_rotor = 2 * np.pi / rot_speed[i] if rot_speed[i] > 0.1 else 100.0
            for k in range(3):                                   # avrSWAP(30..32)
                ci.avrSWAP[29 + k] = 1000.0 * np.sin(2 * np.pi * ti / t_rotor + k * 2 * np.pi / 3)

        gen_torque[i], bld_pitch[i], yaw_out = ci.call_controller(turbine_state)
        # call_controller returns float32; widen before arithmetic, or NumPy keeps
        # yaw_out * DT in float32 and the integrated yaw loses bits.
        yaw_out = float(yaw_out)
        if s.legacy_power:
            gen_power[i] = gen_torque[i] * gen_speed[i] * (turbine.GenEff / 100)
        else:
            gen_power[i] = gen_speed[i] * gen_torque[i] * turbine.GenEff / 100
        if 'yaw_rate' in inputs:
            nac_yaw[i] = nac_yaw[i-1] + yaw_out * DT
        elif s.record_yaw_output:
            nac_yaw[i] = yaw_out
        for name, idx in EXTRA_AVRSWAP.items():
            extra[name][i] = ci.avrSWAP[idx]

    ci.kill_discon()
    result = {
        'gen_torque': gen_torque, 'bld_pitch': bld_pitch,
        'gen_speed': gen_speed, 'gen_power': gen_power,
        'nac_yaw': nac_yaw,
    }
    result.update(extra)
    return result


# ---------------------------------------------------------------------------
# The scenarios
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Scenario:
    """One regression scenario.

    `patches` are the DISCON changes relative to scenario 1 — the recipe for
    fixtures/scenario_NN.IN, which is what the scenario actually runs on.
    """
    num: int
    title: str
    patches: dict = field(default_factory=dict)
    tlen: float = 400
    ws0: float = 9
    step_wind: bool = True              # +1 m/s every 100 s
    synthetic: tuple = ()               # see run_synthetic; empty -> the toolbox Sim
    runner: Callable = None             # overrides the choice above
    # Scenario 2 only. It predates the others and multiplies gen_power in a
    # different order; its baseline holds those bits.
    legacy_power: bool = False
    record_yaw_output: bool = False     # Scenario 2 only: nac_yaw = raw yaw command

    def wind(self):
        t = np.arange(0, self.tlen, DT)
        ws = np.ones_like(t) * self.ws0
        if self.step_wind:
            ws = ws + t // 100
        return t, ws

    def run(self, turbine):
        runner = self.runner or (run_synthetic if self.synthetic else run_sim)
        return runner(self, turbine)


# One notch at 1 rad/s on generator speed; several scenarios reuse it.
_NOTCH = {
    'F_NumNotchFilts': 1,
    'F_NotchFreqs': '1.0000',
    'F_NotchBetaNum': '0.0000',
    'F_NotchBetaDen': '0.2500',
    'F_GenSpdNotch_N': 1,
    'F_GenSpdNotch_Ind': '1',
}

# Open-loop input file columns; OL_Mode scenarios differ in the last few.
_OL_COMMON = {
    'OL_BP_Mode': 0,
    'OL_BP_FiltFreq': 0.0,
    'Ind_Breakpoint': 1,
    'Ind_BldPitch': '2 3 4',
    'Ind_GenTq': 5,
}
_OL_NO_SPEED_REF = {'Ind_R_Speed': 0, 'Ind_R_Torque': 0, 'Ind_R_Pitch': 0}


_SCENARIO_LIST = [
    Scenario(1, "Standard step-wind simulation; re-run checks DLL deallocation",
             tlen=1000, ws0=7, runner=run_twice),

    # The vane and heading reach the controller through turbine_state, because
    # call_controller() overwrites avrSWAP(24)/(37) from it on every call. Until
    # 2026-09-22 this scenario wrote those indices directly and the controller
    # saw 0, so wrap_360 never left its middle branch.
    Scenario(2, "Yaw-by-IPC, Y_ControlMode=2, with a yaw error that wraps",
             patches={'Y_ControlMode': 2},
             tlen=100, step_wind=False, synthetic=('azimuth', 'yaw_by_ipc'),
             legacy_power=True, record_yaw_output=True),

    # Flp_Mode > 0 excludes IPC_ControlMode > 0, so NotchFilterSlopes lives in 6.
    # In the 1-DOF sim the extra modes see zero input; CC_DesiredL stays 0 with
    # tlen=400 < 500, so the cable filter processes zeros throughout.
    Scenario(3, "Notch filter, cable/structural control and several mode flags at once",
             patches={
                 **_NOTCH,
                 'CC_Mode': 1, 'CC_Group_N': 1, 'CC_GroupIndex': '2601',
                 'TD_Mode': 1,
                 'Fl_Mode': 1,
                 'Y_ControlMode': 1,
                 'StC_Mode': 1, 'StC_Group_N': 1, 'StC_GroupIndex': '2801',
                 'Flp_Mode': 1,
                 'F_FlpCornerFreq': '1.0 0.7',   # required when Flp_Mode > 0
                 'F_FlCornerFreq': '1.0 0.7',    # required when Fl_Mode > 0
             }),

    Scenario(4, "Flap control, Flp_Mode=2 (PIIController)",
             patches={
                 'Flp_Mode': 2,
                 'IPC_ControlMode': 0,              # excluded by Flp_Mode > 0
                 'F_FlpCornerFreq': '0.5000  0.7000',
                 'Flp_Kp': '-0.001',                # small gains keep it stable
                 'Flp_Ki': '-0.0005',
             }, tlen=100),

    Scenario(5, "Active wake control, AWC_Mode=4 (ResController)",
             patches={'AWC_Mode': 4, 'AWC_CntrGains': '0.0100 0.0050'}),

    Scenario(6, "IPC, IPC_ControlMode=1 (NotchFilterSlopes), zero gains",
             patches={
                 'IPC_ControlMode': 1,
                 'Flp_Mode': 0,                     # excluded by IPC > 0
                 'IPC_KI': '0.0 0.0',
                 'IPC_KP': '0.0 0.0',
             }, tlen=100),

    # tlen > 500 s so StructuralControl/CableControl pass their t > 500 step.
    Scenario(7, "Synthetic yaw, tower, floating, flap inputs to otherwise-idle functions",
             patches={
                 'Y_ControlMode': 1,
                 'TD_Mode': 1,
                 # Without these the tower damper and floating feedback are
                 # switched on but multiply their inputs by zero, so the
                 # synthetic 'tower' signals reach them and change nothing.
                 # Same values as scenario 27: small enough to keep the rotor
                 # operating normally, since this is a code-path test and this
                 # turbine is neither floating nor tower-damped.
                 'FA_KI': '0.001',
                 'FA_HPFCornerFreq': '0.1',
                 'FA_IntSat': '0.0873',
                 'Fl_Mode': 1,
                 'Fl_Kp': '-1.0',
                 'StC_Mode': 1, 'StC_Group_N': 1, 'StC_GroupIndex': '2801',
                 'CC_Mode': 1, 'CC_Group_N': 1, 'CC_GroupIndex': '2601',
                 'Flp_Mode': 1,
                 'F_FlpCornerFreq': '1.0 0.7',
                 'F_FlCornerFreq': '1.0 0.7',
                 'IPC_ControlMode': 0,
                 'AWC_Mode': 0,
                 **_NOTCH,
             }, tlen=600, synthetic=('azimuth', 'yaw_rate', 'tower', 'root_moop')),

    Scenario(8, "IPC with gains + AWC_Mode=4, driven by blade root moments",
             patches={
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
                 **_NOTCH,
             }, synthetic=('azimuth', 'root_moop')),

    # Startup stages progress in the first ~20 calls; shutdown is time-triggered
    # at 250 s; the speed exclusion band sits near rated LSS speed.
    Scenario(9, "Startup, shutdown, reference-speed exclusion",
             patches={
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
             }, tlen=300, ws0=7),

    # RP_Gains (Kp, Ki, Kd, Tf) track the azimuth column of the OL file; the PID
    # output is added to the generator torque.
    Scenario(10, "Rotor position control, OL_Mode=2 (PIDController)",
             patches={
                 'OL_Mode': 2,
                 'OL_Filename': os.path.join(EXAMPLE_INPUTS_DIR, 'OL_Mode2_Input.dat'),
                 **_OL_COMMON,
                 'Ind_Azimuth': 6,
                 'Ind_YawRate': 0,
                 **_OL_NO_SPEED_REF,
                 'RP_Gains': '1000.0 100.0 500.0 0.1',
                 'CC_Mode': 0,                      # incompatible with OL_Mode=2
                 'StC_Mode': 0,
             }, tlen=100, step_wind=False),

    Scenario(11, "Open-loop AWC, AWC_Mode=1 (complex-number method)",
             patches={
                 'AWC_Mode': 1,
                 'AWC_NumModes': 1,
                 'AWC_n': '1',
                 'AWC_freq': '0.05',
                 'AWC_amp': '2.0',
                 'AWC_clockangle': '0.0',
             }),

    Scenario(12, "K·Ω² torque control, VS_ControlMode=1",
             patches={'VS_ControlMode': 1}, ws0=7),

    Scenario(13, "Fixed blade pitch power overspeed, VS_FBP=1",
             patches={
                 'VS_FBP': 1,
                 'PC_ControlMode': 0,               # excluded by VS_FBP > 0
                 'VS_ControlMode': 1,
             }),

    Scenario(14, "Time-based open-loop pitch/torque/yaw, OL_Mode=1",
             patches={
                 'OL_Mode': 1,
                 'OL_Filename': os.path.join(EXAMPLE_INPUTS_DIR, 'OL_Mode1_Input.dat'),
                 **_OL_COMMON,
                 'Ind_YawRate': 6,
                 'Ind_Azimuth': 0,
                 **_OL_NO_SPEED_REF,
                 'CC_Mode': 0,
                 'StC_Mode': 0,
             }, tlen=200, step_wind=False),

    Scenario(15, "Coleman-transform AWC, AWC_Mode=2",
             patches={
                 'AWC_Mode': 2,
                 'AWC_NumModes': 1,
                 'AWC_harmonic': '1',
                 'AWC_freq': '0.05',
                 'AWC_amp': '2.0',
                 'AWC_clockangle': '0.0',
                 'AWC_phaseoffset': '0.0',
             }),

    Scenario(16, "Coleman-transform cyclic flap control, Flp_Mode=3",
             patches={
                 'Flp_Mode': 3,
                 'IPC_ControlMode': 0,
                 'Flp_Kp': '-0.001',
                 'Flp_Ki': '-0.0005',
                 'F_FlpCornerFreq': '0.5 0.7',
             }),

    Scenario(17, "I&I wind speed estimator, WE_Mode=1",
             patches={'WE_Mode': 1}, ws0=7),

    Scenario(18, "1P + 2P individual pitch control, IPC_ControlMode=2",
             patches={
                 'IPC_ControlMode': 2,
                 'IPC_KP': '0.1 0.05',
                 'IPC_KI': '0.01 0.005',
                 'Flp_Mode': 0,
                 **_NOTCH,
             }),

    Scenario(19, "Pitch actuator LP + pitch offset fault + Fl_Mode=2",
             patches={
                 'PA_Mode': 1,
                 'PF_Mode': 1,
                 'PF_Offsets': '0.01 -0.01 0.005',
                 'VS_ConstPower': 1,
                 'Fl_Mode': 2,
                 'F_FlCornerFreq': '1.0 0.7',
             }),

    Scenario(20, "Pitch actuator SecLP + pitch stuck fault + PRC lookup table",
             patches={
                 'PA_Mode': 2,
                 'PF_Mode': 2,
                 'PF_TimeStuck': '200.0 9999.0 9999.0',   # blade 1 sticks at 200 s
                 'PRC_Mode': 1,
             }),

    Scenario(21, "Closed-loop PI AWC, AWC_Mode=3",
             patches={
                 'AWC_Mode': 3,
                 'AWC_NumModes': 1,
                 'AWC_harmonic': '1',
                 'AWC_freq': '0.05',
                 'AWC_amp': '2.0',
                 'AWC_clockangle': '0.0',
                 'AWC_CntrGains': '0.0100 0.0050',
             }),

    Scenario(22, "Strouhal-transform AWC, AWC_Mode=5",
             patches={
                 'AWC_Mode': 5,
                 'AWC_NumModes': 1,
                 'AWC_harmonic': '1',
                 'AWC_freq': '0.05',
                 'AWC_amp': '2.0',
                 'AWC_clockangle': '0.0',
                 'AWC_CntrGains': '0.0100 0.0050',
             }),

    Scenario(23, "Normally-on modes switched off, PS_Mode=0 + SS_Mode=0",
             patches={'PS_Mode': 0, 'SS_Mode': 0}, ws0=7),

    Scenario(24, "Open-loop cable and structural control, CC_Mode=2 + StC_Mode=2",
             patches={
                 'OL_Mode': 1,
                 'OL_Filename': os.path.join(EXAMPLE_INPUTS_DIR, 'OL_Mode1_CC_StC_Input.dat'),
                 **_OL_COMMON,
                 'Ind_YawRate': 6,
                 'Ind_Azimuth': 0,
                 **_OL_NO_SPEED_REF,
                 'CC_Mode': 2, 'CC_Group_N': 1, 'CC_GroupIndex': '2601',
                 'Ind_CableControl': '7',
                 'StC_Mode': 2, 'StC_Group_N': 1, 'StC_GroupIndex': '2801',
                 'Ind_StructControl': '8',
             }, tlen=200, step_wind=False),

    Scenario(25, "Dynamic power rating, PRC_Mode=2 with constant inputs",
             patches={
                 'PRC_Mode': 2,
                 'PRC_Comm': 0,
                 'PRC_R_Speed': '0.9',              # rated speed -10%
                 'PRC_R_Torque': '1.0',
                 'PRC_R_Pitch': '1.0',
             }, ws0=7),

    # The only scenario with non-zero flp_angle output: rootMOOP -> notch ->
    # Coleman -> PI (tilt/yaw) -> inverse Coleman -> per-blade flap angle.
    Scenario(26, "Flp_Mode=3 driven to non-zero flap output",
             patches={
                 'Flp_Mode': 3,
                 'Flp_Kp': -0.001,
                 'Flp_Ki': -0.0005,
                 'F_FlpCornerFreq': '0.5 0.7',
                 'F_FlCornerFreq': '1.0 0.7',
                 'IPC_ControlMode': 0,
                 'AWC_Mode': 0,
                 **_NOTCH,
             }, step_wind=False, synthetic=('azimuth', 'root_moop')),

    # Six simultaneous pitch contributions: collective PI, IPC, tower damping,
    # floating feedback, AWC and the pitch fault offset, through a second-order
    # actuator. Until 2026-09-22 the tower and IMU accelerations were written to
    # avrSWAP(53)/(83) directly and arrived as 0, so two of the six were absent.
    Scenario(27, "Stress test: many modes active at once",
             patches={
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
                 # The tuner leaves FA_* and Fl_Kp at zero for this turbine, so
                 # tower damping and floating feedback would contribute exactly
                 # zero however hard they are driven. No config in the repo uses
                 # the tower damper, and this turbine is not floating, so these
                 # gains are chosen small enough to keep the rotor operating
                 # normally rather than tuned — a code-path test, not a physical
                 # one. The IEA-15 semi's Fl_Kp (-9.2) feathers this rotor.
                 'FA_KI': '0.001',
                 'FA_HPFCornerFreq': '0.1',
                 'FA_IntSat': '0.0873',
                 'Fl_Mode': 2,
                 'Fl_Kp': '-1.0',
                 'F_FlCornerFreq': '1.0 0.7',
                 'CC_Mode': 1, 'CC_Group_N': 1, 'CC_GroupIndex': '2601',
                 'StC_Mode': 1, 'StC_Group_N': 1, 'StC_GroupIndex': '2801',
                 'PA_Mode': 2,
                 'PF_Mode': 1,
                 'PF_Offsets': '0.01 -0.01 0.005',
                 'VS_ConstPower': 1,
                 'PRC_Mode': 1,
                 'Flp_Mode': 0,
                 **_NOTCH,
             }, tlen=600, synthetic=('azimuth', 'yaw_rate', 'tower', 'root_moop')),

    # avrSWAP-level outputs do not depend on OutputFormat; the .RO.h5 itself is
    # compared against scenario 1's text output by run_regression.py --hdf5.
    Scenario(28, "HDF5 debug output, OutputFormat=1 + LoggingLevel=3",
             patches={'OutputFormat': 1, 'LoggingLevel': 3},
             tlen=1000, ws0=7, runner=run_hdf5),

    # 29 and 30 cover the torque-control settings real turbine configurations use
    # but scenarios 1-28 never did: the NREL-2.8 and MHK_RM1 Test_Cases run
    # VS_ControlMode=3, and the IEA-15, BAR_10 and NREL-2.8 ones run
    # VS_ConstPower=0. Both winds cross rated (~11.4 m/s), because that is where
    # either setting changes what the controller does.
    Scenario(29, "Power-based TSR tracking, VS_ControlMode=3",
             patches={'VS_ControlMode': 3}, tlen=600, ws0=7),

    Scenario(30, "Constant generator torque above rated, VS_ConstPower=0",
             patches={'VS_ConstPower': 0}, tlen=400, ws0=11),
]

SCENARIOS = {s.num: s for s in _SCENARIO_LIST}


def run_scenario(num, turbine, output_dir=None):
    s = SCENARIOS[num]
    print("=" * 60)
    print(f"Scenario {num}: {s.title}")
    print("=" * 60)
    result = s.run(turbine)
    # The inputs travel with the outputs, so a baseline says which wind it was
    # captured under and plot_regression.py needs no copy of this table.
    result['t'], result['ws'] = s.wind()
    save_and_print_results(result, num, output_dir)
    print(f"Scenario {num}: done")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description='ROSCO regression scenario runner')
    parser.add_argument('--scenario', type=int, default=0,
                        help='Run one scenario. Default 0 = run all.')
    parser.add_argument('--output-dir', type=str, default=None,
                        help='Save simulation output arrays to .npz files in this directory.')
    parser.add_argument('--benchmark', type=int, default=0,
                        help='Run each scenario N times and output timing CSV. No arrays saved.')
    parser.add_argument('--build', type=str, default='unknown',
                        help='Build label for benchmark CSV output (e.g., upstream, modified, cpp).')
    parser.add_argument('--write-fixtures', action='store_true',
                        help='Regenerate fixtures/ from the tuner and exit. Maintenance '
                             'operation: run_regression.py must still be ALL IDENTICAL after.')
    args = parser.parse_args()

    if args.write_fixtures:
        write_fixtures()
        return

    # Fixtures are committed, so the tuner is not in the loop. The turbine is
    # still needed as the plant model for the 1-DOF simulation.
    turbine, _ = load_turbine_only()

    # The order is historical and only matters when running every scenario in
    # one process; run_regression.py isolates each one in its own subprocess.
    scenario_order = [3, 4, 5, 1, 2, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
                      17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30]
    selected = [args.scenario] if args.scenario > 0 else scenario_order

    if args.benchmark > 0:
        # Benchmark mode: time each scenario, output CSV, suppress ALL stdout
        # (including C/Fortran prints that bypass Python's sys.stdout)
        import time

        # Write timing results to stderr (fd 2) to avoid mixing with controller output
        sys.stderr.write("build,scenario,run,seconds\n")
        sys.stderr.flush()
        for s in selected:
            for run_num in range(1, args.benchmark + 1):
                # Redirect fd 1 to /dev/null at OS level to suppress
                # both Python prints and C/Fortran printf/write
                devnull_fd = os.open(os.devnull, os.O_WRONLY)
                saved_stdout_fd = os.dup(1)
                os.dup2(devnull_fd, 1)
                os.close(devnull_fd)
                try:
                    t0 = time.perf_counter()
                    SCENARIOS[s].run(turbine)
                    t1 = time.perf_counter()
                finally:
                    os.dup2(saved_stdout_fd, 1)
                    os.close(saved_stdout_fd)
                sys.stderr.write("%s,%d,%d,%.4f\n" % (args.build, s, run_num, t1 - t0))
                sys.stderr.flush()
        return

    for s in selected:
        run_scenario(s, turbine, args.output_dir)

    print("\n" + "=" * 60)
    print("All scenarios complete.")
    print("=" * 60)


if __name__ == '__main__':
    main()
