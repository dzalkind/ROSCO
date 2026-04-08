"""
04_simple_sim
-------------
Demonstrate the simple 1-DOF wind turbine simulator with ROSCO

In this example:

* Load turbine from saved pickle and tune a ROSCO controller
* Run and plot a step wind simulation using 1-DOF model in ``rosco.toolbox.sim`` and the ROSCO dynamic library

.. figure:: /images/examples/03_GainSched.png
   :align: center
   :width: 70%


Notes:

* You must have a compiled controller in ROSCO/rosco/lib/, and properly point to it using the `lib_name` variable.
* Using wind speed estimators in this simple simulation is known to cause problems. We suggest using WE_Mode = 0 in the DISCON.IN or increasing sampling rate of simulation as workarounds.
* The simple simulation is run twice to check that arrays are deallocated properly.

"""
# Python modules
import gc
import pandas as pd
import re


import matplotlib.pyplot as plt 
import numpy as np
import os
# ROSCO toolbox modules 
from rosco import discon_lib_path as lib_name
from rosco.toolbox import controller as ROSCO_controller
from rosco.toolbox import turbine as ROSCO_turbine
from rosco.toolbox import sim as ROSCO_sim
from rosco.toolbox import control_interface as ROSCO_ci
from rosco.toolbox.utilities import write_DISCON
from rosco.toolbox.inputs.validation import load_rosco_yaml

def parse_dbg3_file(filename):

    with open(filename, 'r') as f:
        lines = f.readlines()

    # Find header lines
    header_idx = None
    for i, line in enumerate(lines):
        if line.strip().startswith('LocalVar%Time'):
            header_idx = i
            break
    if header_idx is None:
        raise ValueError("Header not found")

    # Parse column names
    raw_col_names = lines[header_idx].split('AvrSWAP(')
    col_names = []
    for name in raw_col_names:
        match = re.match(r'(.*\d+)\)', name)
        if match:
            col_names.append(match.group(1))  # Use only the integer inside the parentheses

    col_names = [name.strip() for name in col_names]
    col_names.insert(0, 'Time')

    units = lines[header_idx+1].split()
    data_lines = lines[header_idx+2:]

    # Remove any non-data lines at the end
    data = []
    for line in data_lines:
        if line.strip() == '' or not line.strip()[0].isdigit():
            continue
        data.append([float(x) for x in line.split()])

    arr = np.array(data)
    result = {}
    for idx, name in enumerate(col_names):
        result[name] = arr[:, idx]

    return result

def main():
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    # Load yaml file 
    this_dir = os.path.dirname(os.path.abspath(__file__))
    tune_dir =  os.path.join(this_dir,'Tune_Cases')
    parameter_filename = os.path.join(this_dir,'USFLOWT_ROSCO_opt.yaml')
    inps = load_rosco_yaml(parameter_filename)
    path_params         = inps['path_params']
    turbine_params      = inps['turbine_params']
    controller_params   = inps['controller_params']

    # Specify controller dynamic library path and name

    #directories
    rosco_dir           = os.path.dirname(this_dir)
    example_out_dir     = os.path.join(this_dir,'examples_out')
    os.makedirs(example_out_dir,exist_ok=True)

    # # Load turbine model from saved pickle
    turbine         = ROSCO_turbine.Turbine
    turbine         = turbine.load('USFLOWT_10.p')

    # Load turbine data from OpenFAST and rotor performance text file
    cp_filename = os.path.join(os.path.dirname(parameter_filename),path_params['rotor_performance_filename'])
    turbine.load_from_fast(
        path_params['FAST_InputFile'],
        os.path.join(os.path.dirname(parameter_filename),path_params['FAST_directory']),
        rot_source='txt',txt_filename=cp_filename
        )

    # Tune controller 
    controller      = ROSCO_controller.Controller(controller_params)
    controller.tune_controller(turbine)

    # Write parameter input file
    param_filename = os.path.join(this_dir,'1_ElastoDyn','USFLOWT_10_DISCON.IN')

    if True:
        # Load controller library
        controller_int = ROSCO_ci.ControllerInterface(lib_name,param_filename=param_filename,sim_name='sim1')

        # Load the simulator
        sim_1 = ROSCO_sim.Sim(turbine,controller_int)

        wind_input = '/Users/dzalkind/Tools/ROSCO-USFLOWT/Examples/Wave_Tank/3_model_verification/filtered_rt_vavghx.csv'
        offset = 0

        # Define a wind speed history
        if wind_input == 'step':
            dt = 0.025
            tlen = 1000      # length of time to simulate (s)
            ws0 = 7         # initial wind speed (m/s)
            t= np.arange(0,tlen,dt) 
            ws = np.ones_like(t) * ws0
            # add steps at every 100s
            for i in range(len(t)):
                ws[i] = ws[i] + t[i]//100
        else:
            df_u = pd.read_csv(wind_input)
            ws = df_u['U_avg(m/s)'].to_numpy() + offset
            t = df_u['# Time(s)'].to_numpy()

        # Run simulator and plot results
        sim_1.sim_ws_series(t,ws,rotor_rpm_init=4)

        outs = {}
        outs['Time'] = sim_1.t_array
        outs['RtVAvgxh'] = sim_1.ws_array
        outs['GenSpeed'] = sim_1.gen_speed * 60 / (2*np.pi) # convert to rpm
        outs['BldPitch1'] = np.degrees(sim_1.bld_pitch)
        outs['GenTq'] = sim_1.gen_torque / 1000
        outs['RtAeroFxi'] = sim_1.rot_thrust
        outs['RtAeroMxi'] = sim_1.aero_torque

        pd.DataFrame(outs).to_csv(os.path.join(this_dir,'1dof_sim_outs.csv'), index=False)

    #         channels = [
    #     'RtVAvgxh',
    #     'BldPitch1', 'GenSpeed', 'GenTq',
    #     'RtAeroFxi', 'RtAeroFyi', 'RtAeroFzi',
    #     'RtAeroMxi', 'RtAeroMyi', 'RtAeroMzi',
    # ]

        print('here')


    if False:
        avr_swap = parse_dbg3_file(os.path.join(this_dir,'sim1.RO.dbg3'))

        # Save avr_swap as csv for SC
        df = pd.DataFrame(avr_swap)
        df.to_csv(os.path.join(this_dir,'sim1_avr_swap.csv'), index=False)


        # Load controller library again to run in open loop
        controller_int = ROSCO_ci.ControllerInterface(lib_name,param_filename=param_filename,sim_name='sim2')

        avr_len = 85
        avr_swap_in = np.zeros(avr_len)
        for i_row, row in df.iterrows():

            if i_row == 0:
                # Controller is set up at t=0 when initialized above
                avr_swap_out = controller_int.avrSWAP[:avr_len].reshape(-1,1)
                continue
            
            t = row['Time']
            for i in range(avr_len):
                avr_swap_in[i] = row.iloc[i+1] # labels are 1-indexed in the dbg3 file

            
            controller_int.avrSWAP = avr_swap_in
            controller_int.call_discon()
            avr_swap_out = np.c_[avr_swap_out, controller_int.avrSWAP]

        fig, axs = plt.subplots(3,1)
        axs[0].plot(df['Time'], df['20'], label='gen_speed CL')
        axs[0].plot(df['Time'], avr_swap_out[19,:], label='gen_speed OL')
        axs[0].legend()
        
        axs[1].plot(df['Time'], df['42'], label='pitch CL')
        axs[1].plot(df['Time'], avr_swap_out[41,:], label='pitch OL')
        axs[1].legend()

        axs[2].plot(df['Time'], df['47'], label='torque CL')
        axs[2].plot(df['Time'], avr_swap_out[46,:], label='torque OL')
        axs[2].legend()

        plt.show()


if __name__ == "__main__":
    main()

