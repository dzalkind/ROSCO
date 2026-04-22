"""
5_multi_case/run_all_cases.py
Run the Wave Tank 3-model verification pipeline for all cases in the WEIS
case matrix.  For each case:
  1. Extract and filter RtVAvgxh from OpenFAST output
  2. (DLC1.6 only) Save TurbSim plane-average wind for reference
  3. Interpolate 6-DOF aero loads from steady lookup table
  4. Run 1-DOF ROSCO closed-loop simulation
  5. Save 3-way time-series comparison plot
Then aggregate NRMS errors across all cases into a summary table.

Usage:
    python 5_multi_case/run_all_cases.py
"""

import os
import sys
import signal
import multiprocessing as mp
import yaml
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')   # save to file; no interactive window needed
import matplotlib.pyplot as plt
from scipy.signal import butter, sosfiltfilt
from scipy.interpolate import interp1d as scipy_interp1d

from openfast_io.turbsim_file import TurbSimFile
from rosco.toolbox.ofTools.fast_io.output_processing import output_processing
from rosco.toolbox import controller as ROSCO_controller
from rosco.toolbox import turbine as ROSCO_turbine
from rosco.toolbox import sim as ROSCO_sim
from rosco.toolbox import control_interface as ROSCO_ci
from rosco.toolbox.utilities import write_DISCON
from rosco.toolbox.inputs.validation import load_rosco_yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '0_shared'))
import config as wt_config
from utils import interpolate_1d


# ── YAML case matrix loader ───────────────────────────────────────────────────

def _load_case_matrix(yaml_path: str) -> dict:
    """
    Load case_matrix_combined.yaml.

    The file uses YAML block mapping with sequence keys such as
      [AeroDyn, Wake_Mod]: {case_name: value, ...}
    which pyyaml safe_load cannot parse because lists are unhashable.
    A custom loader converts those sequence keys to tuples first.
    """
    class _TupleKeyLoader(yaml.SafeLoader):
        pass

    _orig = yaml.SafeLoader.construct_mapping

    def _construct_mapping(self, node, deep=False):
        if isinstance(node, yaml.MappingNode):
            for key_node, _ in node.value:
                if isinstance(key_node, yaml.SequenceNode):
                    key_node.tag = 'tag:yaml.org,2002:python/tuple'
        return _orig(self, node, deep=deep)

    _TupleKeyLoader.add_constructor(
        'tag:yaml.org,2002:python/tuple',
        lambda loader, node: tuple(loader.construct_sequence(node)),
    )
    _TupleKeyLoader.construct_mapping = _construct_mapping

    with open(yaml_path, 'r') as fh:
        return yaml.load(fh, Loader=_TupleKeyLoader)


def build_case_list(cm: dict) -> list:
    """
    Return a list of dicts, one per case:
      case_name, dlc, tstart, tmax, hwind_speed, of_out_file, bts_file
    bts_file is None for DLCsteady cases (matrix value is 'unused').
    """
    # Derive case names from the keys of any field dict
    any_field = next(iter(cm.values()))
    case_names = list(any_field.keys())

    of_run_dir = os.path.dirname(wt_config.CASE_MATRIX_YAML)

    cases = []
    for cn in case_names:
        bts_raw = cm.get(('InflowWind', 'FileName_BTS'), {}).get(cn, 'unused')
        bts_file = None if str(bts_raw).strip().lower() == 'unused' else str(bts_raw)

        cases.append({
            'case_name':   cn,
            'dlc':         cm.get(('DLC',), {cn: None}).get(cn),
            'tstart':      cm.get(('Fst', 'TStart'), {cn: 0.0}).get(cn, 0.0),
            'tmax':        cm.get(('Fst', 'TMax'), {cn: None}).get(cn),
            'hwind_speed': cm.get(('InflowWind', 'HWindSpeed'), {cn: None}).get(cn),
            'of_out_file': os.path.join(of_run_dir, cn + '.out'),
            'bts_file':    bts_file,
        })
    return cases


# ── ROSCO one-time initialisation ─────────────────────────────────────────────

def init_rosco_once():
    """
    Load turbine pickle, tune controller, write DISCON.IN once.
    Returns (turbine, controller) — both are reused across all cases.
    ControllerInterface must be created fresh per simulation (Fortran state).
    """
    inps              = load_rosco_yaml(wt_config.ROSCO_YAML)
    path_params       = inps['path_params']
    controller_params = inps['controller_params']

    turbine = ROSCO_turbine.Turbine
    turbine = turbine.load(wt_config.TURBINE_PICKLE)

    cp_filename = os.path.join(wt_config.WAVE_TANK_DIR,
                               path_params['rotor_performance_filename'])
    turbine.load_from_fast(
        path_params['FAST_InputFile'],
        os.path.join(wt_config.WAVE_TANK_DIR, path_params['FAST_directory']),
        rot_source='txt', txt_filename=cp_filename,
    )

    controller = ROSCO_controller.Controller(controller_params)
    controller.tune_controller(turbine)

    write_DISCON(
        turbine, controller,
        param_file=wt_config.DISCON_IN_FILE,
        txt_filename=cp_filename,
    )

    return turbine, controller


# ── 1-DOF subprocess worker ───────────────────────────────────────────────────

def _run_1dof_worker(result_csv, turbine, u_tt, u_avg, case_name):
    """
    Run 1-DOF ROSCO sim in an isolated subprocess.
    Writes result CSV and exits.  If the C library segfaults,
    only this child process dies — the parent continues.
    """
    controller_int = ROSCO_ci.ControllerInterface(
        wt_config.ROSCO_LIB_PATH,
        param_filename=wt_config.DISCON_IN_FILE,
        sim_name=f'sim_{case_name}',
    )
    sim_1 = ROSCO_sim.Sim(turbine, controller_int)
    sim_1.sim_ws_series(u_tt, u_avg, rotor_rpm_init=4)

    resp_1 = pd.DataFrame({
        'Time':      sim_1.t_array,
        'RtVAvgxh':  sim_1.ws_array,
        'GenSpeed':  sim_1.gen_speed * 60 / (2 * np.pi),
        'BldPitch1': np.degrees(sim_1.bld_pitch),
        'GenTq':     sim_1.gen_torque / 1000,
        'RtAeroFxi': sim_1.rot_thrust,
        'RtAeroMxi': sim_1.aero_torque,
    })
    resp_1.to_csv(result_csv, index=False)


# ── Per-case pipeline ─────────────────────────────────────────────────────────

def run_case(case: dict, turbine, df_6dof: pd.DataFrame, out_dir: str):
    """
    Run one case.  Returns a dict of NRMS error values, or None if the
    OpenFAST output file is missing.
    """
    cn          = case['case_name']
    of_out_file = case['of_out_file']

    if not os.path.isfile(of_out_file):
        print(f'  [SKIP] .out file not found: {of_out_file}')
        return None

    os.makedirs(out_dir, exist_ok=True)

    # ── Step 1: load OpenFAST output, filter RtVAvgxh ────────────────────────
    op      = output_processing()
    fastout = op.load_fast_out(of_out_file)
    fast    = fastout[0]     # dict of channel_name -> array, plus 'meta'

    dt  = float(np.mean(np.diff(fast['Time'])))
    sos = butter(wt_config.FILTER_ORDER,
                 wt_config.FILTER_CUTOFF / 2.0,
                 btype='low', fs=1.0 / dt, output='sos')
    u_avg = sosfiltfilt(sos, fast['RtVAvgxh'])
    u_tt  = fast['Time']

    wind_csv = os.path.join(out_dir, f'{cn}_RtVAvgxh_filtered.csv')
    pd.DataFrame({'# Time(s)': u_tt, 'U_avg(m/s)': u_avg}).to_csv(wind_csv, index=False)

    # ── Step 2: TurbSim plane-average wind (reference, not used in pipeline) ──
    if case['bts_file'] and os.path.isfile(case['bts_file']):
        os.makedirs(wt_config.PLANE_AVG_WIND_DIR, exist_ok=True)
        ts_file = TurbSimFile(case['bts_file'])
        ts_file.read()
        pa_avg = np.mean(ts_file['u'][0, :, :, :], axis=(1, 2))
        pa_tt  = ts_file['t']
        plane_csv = os.path.join(wt_config.PLANE_AVG_WIND_DIR, f'{cn}_plane_avg.csv')
        pd.DataFrame({'# Time(s)': pa_tt, 'U_avg(m/s)': pa_avg}).to_csv(plane_csv, index=False)

    # ── Step 3: 6-DOF steady lookup ──────────────────────────────────────────
    interp_outs = {'Time': u_tt}
    for ch in wt_config.CHANNELS_6DOF:
        interp_outs[ch] = interpolate_1d(df_6dof, 'Wind1VelX', ch, u_avg,
                                         extrapolate=True)
    resp_6 = pd.DataFrame(interp_outs)
    resp_6.to_csv(os.path.join(out_dir, f'{cn}_interp_6dof.csv'), index=False)

    # ── Step 4: 1-DOF ROSCO simulation (isolated subprocess) ────────────────
    result_csv = os.path.join(out_dir, f'{cn}_1dof_sim.csv')
    ctx = mp.get_context('fork')
    p = ctx.Process(
        target=_run_1dof_worker,
        args=(result_csv, turbine, u_tt, u_avg + wt_config.SIM_OFFSET, cn),
    )
    p.start()
    p.join(timeout=600)

    if p.exitcode is None:
        # Timed out
        p.kill()
        p.join()
        print(f'  [ERROR] 1-DOF subprocess timed out for {cn}')
        return None
    elif p.exitcode != 0:
        sig = -p.exitcode if p.exitcode < 0 else p.exitcode
        sig_name = signal.Signals(sig).name if p.exitcode < 0 else f'code {sig}'
        print(f'  [ERROR] 1-DOF subprocess crashed ({sig_name}) for {cn}')
        return None

    resp_1 = pd.read_csv(result_csv)

    # ── Step 5: 3-way time-series comparison plot ─────────────────────────────
    channels = wt_config.CHANNELS_COMPARE
    fig, axs = plt.subplots(len(channels), 1, sharex=True,
                            figsize=(10, 2 * len(channels)),
                            constrained_layout=True)
    fig.suptitle(cn, fontsize=10)

    # Wind speed (row 0)
    axs[0].plot(u_tt, u_avg, color='C0', label='6-DOF Lookup')
    axs[0].set_ylabel('RtVAvgxh\n(m/s)')
    axs[0].grid(True)

    for i, ch in enumerate(channels[1:], start=1):
        ax = axs[i]
        if ch in resp_6.columns:
            ax.plot(resp_6['Time'], resp_6[ch], color='C0', label='6-DOF Lookup')
        if ch in resp_1.columns:
            ax.plot(resp_1['Time'], resp_1[ch], color='C1',
                    label='1-DOF Sim', alpha=0.7)
        if ch in fast:
            unit_idx = fast['meta']['channels'].index(ch)
            unit     = fast['meta']['attribute_units'][unit_idx]
            ax.plot(fast['Time'], fast[ch], color='C2',
                    label='OpenFAST', alpha=0.7)
            ax.set_ylabel(f'{ch}\n({unit})')
        else:
            ax.set_ylabel(ch)
        ax.grid(True)

    axs[0].legend(loc='upper right', fontsize=8)
    axs[-1].set_xlabel('Time (s)')
    fig.align_ylabels()
    fig.savefig(os.path.join(out_dir, f'{cn}_timeseries.png'), dpi=120)
    plt.close(fig)

    # ── Step 6: NRMS errors vs OpenFAST (t > 200 s) ──────────────────────────
    T_RMS_START   = 200.0
    tof           = fast['Time']
    t6            = resp_6['Time'].to_numpy()
    t1            = resp_1['Time'].to_numpy()
    t_max_common  = min(t6.max(), t1.max(), tof.max())
    mask_of       = (tof >= T_RMS_START) & (tof <= t_max_common)
    t_common      = tof[mask_of]

    nrms = {}
    for ch in wt_config.CHANNELS_6DOF:
        if ch not in fast:
            nrms[f'{ch}_6dof'] = np.nan
            nrms[f'{ch}_1dof'] = np.nan
            continue

        yof      = fast[ch][mask_of]
        mean_abs = np.mean(np.abs(yof))

        if ch in resp_6.columns and mean_abs > 0:
            f6 = scipy_interp1d(t6, resp_6[ch].to_numpy(),
                                bounds_error=False, fill_value='extrapolate')
            nrms[f'{ch}_6dof'] = 100.0 * np.sqrt(
                np.mean((f6(t_common) - yof) ** 2)) / mean_abs
        else:
            nrms[f'{ch}_6dof'] = np.nan

        if ch in resp_1.columns and mean_abs > 0:
            f1 = scipy_interp1d(t1, resp_1[ch].to_numpy(),
                                bounds_error=False, fill_value='extrapolate')
            nrms[f'{ch}_1dof'] = 100.0 * np.sqrt(
                np.mean((f1(t_common) - yof) ** 2)) / mean_abs
        else:
            nrms[f'{ch}_1dof'] = np.nan

    return nrms


# ── Summary table ─────────────────────────────────────────────────────────────

def build_summary_table(all_nrms: list, case_list: list, out_dir: str):
    """
    Write summary_rms_errors.csv (and .xlsx if openpyxl is available).
    Rows = cases, columns = case_name | dlc | hwind_speed |
      RtAeroFxi_6dof | RtAeroFxi_1dof | ... (one pair per CHANNELS_6DOF entry).
    """
    records = []
    for case, nrms in zip(case_list, all_nrms):
        if nrms is None:
            continue
        row = {
            'case_name':   case['case_name'],
            'dlc':         case['dlc'],
            'hwind_speed': case['hwind_speed'],
        }
        row.update(nrms)
        records.append(row)

    df = pd.DataFrame(records)

    csv_path = os.path.join(out_dir, 'summary_rms_errors.csv')
    df.to_csv(csv_path, index=False)
    print(f'\nSummary table → {csv_path}')

    try:
        xlsx_path = os.path.join(out_dir, 'summary_rms_errors.xlsx')
        df.to_excel(xlsx_path, index=False)
        print(f'Excel summary  → {xlsx_path}')
    except ImportError:
        pass   # openpyxl not installed; CSV is sufficient


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    print('Loading case matrix...')
    cm        = _load_case_matrix(wt_config.CASE_MATRIX_YAML)
    case_list = build_case_list(cm)
    # case_list = [case_list[0]]. # debugging single case
    print(f'Found {len(case_list)} cases.')

    df_6dof = pd.read_csv(wt_config.STEADY_6DOF_LOOKUP_CSV)

    print('Initialising ROSCO (turbine + controller)...')
    turbine, _controller = init_rosco_once()

    os.makedirs(wt_config.MULTI_CASE_OUT_DIR, exist_ok=True)

    all_nrms = []
    for i, case in enumerate(case_list, 1):
        cn = case['case_name']
        print(f'\n[{i}/{len(case_list)}] {cn}')
        out_dir = os.path.join(wt_config.MULTI_CASE_OUT_DIR, cn)
        nrms = run_case(case, turbine, df_6dof, out_dir)
        all_nrms.append(nrms)

    build_summary_table(all_nrms, case_list, wt_config.MULTI_CASE_OUT_DIR)
    print('\nDone.')


if __name__ == '__main__':
    main()
