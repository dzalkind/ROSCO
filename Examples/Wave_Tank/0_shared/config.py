import os

# Root anchors
WAVE_TANK_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PII_BASE = os.environ.get(
    'USFLOWT_PII_DIR',
    os.path.expanduser('~/Library/CloudStorage/Box-Box/USFLOWT_PII')
)

# --- Active simulation run: change these 3 lines to switch campaigns ---
OF_CAMPAIGN  = '2026.04.01_WaveVerification'
OF_CASE_NAME = 'DLC1.6_6_weis_job_1'           # matches .out filename stem
TS_WIND_FILE = 'weis_job_0_NTM_U11.400000_Seed714712467.0.bts'

# Derived paths (PII inputs)
_of_run_dir       = os.path.join(PII_BASE, '05_SimModel', 'OpenFAST', OF_CAMPAIGN, 'rank_0')
OPENFAST_OUT_FILE = os.path.join(_of_run_dir, OF_CASE_NAME + '.out')
TURBSIM_BTS_FILE  = os.path.join(_of_run_dir, 'wind', TS_WIND_FILE)

# Derived paths (within Wave_Tank)
STEADY_6DOF_LOOKUP_CSV = os.path.join(WAVE_TANK_DIR, '2_6DOF', 'steady_6dof_lookup.csv')
PLANE_AVG_WIND_CSV     = os.path.join(WAVE_TANK_DIR, '2_6DOF', 'plane_avg_wind.csv')
INTERP_6DOF_RESP_CSV   = os.path.join(WAVE_TANK_DIR, '2_6DOF', 'interp_6dof_responses.csv')
FILTERED_WIND_CSV      = os.path.join(WAVE_TANK_DIR, '3_model_verification', 'filtered_rt_vavghx.csv')
SIM_1DOF_OUT_CSV       = os.path.join(WAVE_TANK_DIR, '1dof_sim_outs.csv')
ROSCO_YAML             = os.path.join(WAVE_TANK_DIR, 'USFLOWT_ROSCO_opt.yaml')

# Shared channel lists
CHANNELS_COMPARE = [
    'RtVAvgxh',
    'BldPitch1', 'GenSpeed', 'GenTq', 'GenPwr',
    'RtAeroFxi',
    'RtAeroMxi',
]
CHANNELS_6DOF = [
    'RtAeroFxi', 'RtAeroFyi', 'RtAeroFzi',
    'RtAeroMxi', 'RtAeroMyi', 'RtAeroMzi',
]

# Butterworth filter (applied to OpenFAST RtVAvgxh in model verification)
FILTER_ORDER  = 4
FILTER_CUTOFF = 1.0   # Hz

# 1-DOF simulation defaults (used when wind_input == 'step')
SIM_DT     = 0.025    # s
SIM_TLEN   = 1000     # s
SIM_WS0    = 7.0      # m/s initial wind speed
SIM_OFFSET = 0.0      # m/s offset applied to CSV wind input
