'''
31_fixed_pitch_mhk
------------------

This example demonstrates the fixed-pitch control of a marine hydrodkinetic (MHK) turbine.

There are several ways to control the power output of a turbine in above-rated conditions.  
In this example we demonstrate the following control configurations:

#. Constant power underspeed (should be the default)
#. Constant power overspeed
#. Linear increasing power
#. Linear increasing power, leveling out
#. Generic numeric function
#. Constant power overspeed, nonlinear lookup table control

More details about the controller methods can be found in :ref:`marine_hydro`.

The desired power curves of each configuration are as follows:

.. image:: ../images/examples/31_fixed_pitch_mhk_sched.png

In the first case, the reference generator speed is decreased (underspeed) to maintain a constant rated power above rated.
To slow down the generator, a higher torque must be used:

.. image:: ../images/examples/31_fixed_pitch_mhk_sim.png



'''

# Copying images, from docs/:
# cp ../Examples/examples_out/30_fixed_pitch_mhk_sched.png images/
# cp ../Examples/examples_out/30_fixed_pitch_mhk_sim.png images/

import os
from rosco.toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO
from rosco.toolbox.ofTools.case_gen import CaseLibrary as cl
from rosco.toolbox.ofTools.fast_io import output_processing
from rosco.toolbox import controller as ROSCO_controller
from rosco.toolbox import turbine as ROSCO_turbine
from rosco.toolbox import utilities as ROSCO_utilities
from rosco.toolbox.utilities import write_DISCON
from rosco.toolbox.inputs.validation import load_rosco_yaml
import matplotlib.pyplot as plt
import numpy as np



#directories
this_dir            = os.path.dirname(os.path.abspath(__file__))
rosco_dir           = os.path.dirname(this_dir)
example_out_dir     = os.path.join(this_dir, 'examples_out')
os.makedirs(example_out_dir,exist_ok=True)

def main():

    FULL_TEST = True   # Run a full test locally (True) or a shorter one for CI

    # Input yaml and output directory
    parameter_filename = os.path.join(this_dir, 'Tune_Cases/RM1_MHK_FBP.yaml')
    tune_dir = os.path.dirname(parameter_filename)

    inps = load_rosco_yaml(parameter_filename)
    path_params         = inps['path_params']
    turbine_params      = inps['turbine_params']
    controller_params   = inps['controller_params']

    # Instantiate turbine, controller, and file processing classes
    turbine         = ROSCO_turbine.Turbine(turbine_params)
    controller      = ROSCO_controller.Controller(controller_params)

    # Load turbine data from OpenFAST and rotor performance text file
    cp_filename = os.path.join(tune_dir, path_params['rotor_performance_filename'])
    turbine.load_from_fast(
        path_params['FAST_InputFile'],
        os.path.join(tune_dir, path_params['FAST_directory']),
        rot_source='txt', txt_filename= cp_filename
        )


    ### Control configurations: overrides applied to the tuning yaml controller_params
    control_configs = {
        'Constant Power Underspeed': {      # should be the default
            'VS_FBP': 3,                    # Torque-lookup reference
            'VS_FBP_speed_mode': 0,
            'VS_FBP_U': [2.0, 4.0],
            'VS_FBP_P': [1.0, 1.0],
            },
        'Constant Power Overspeed': {
            'VS_FBP': 2,                    # WSE reference
            'VS_ControlMode': 2,            # Region 2 mode paired with VS_FBP = 2
            'VS_FBP_speed_mode': 1,
            'VS_FBP_U': [2.0, 4.0],
            'VS_FBP_P': [1.0, 1.0],
            },
        'Linear Increasing Power': {
            'VS_FBP_speed_mode': 0,
            'VS_FBP_U': [2.0, 4.0],
            'VS_FBP_P': [1.0, 2.0],
            },
        'Increasing Leveled Power': {
            'VS_FBP_U': [2.0, 3.0],
            'VS_FBP_P': [1.0, 2.0],
            },
        'Generic User-Defined': {
            'VS_FBP': 2,                    # WSE reference
            'VS_ControlMode': 2,            # Region 2 mode paired with VS_FBP = 2
            'VS_FBP_U': [2.0, 2.2, 2.4, 2.6, 2.8, 3.0, 3.2, 3.4, 3.6, 3.8, 4.0],
            'VS_FBP_P': [1.0, 1.3, 1.6, 1.8, 1.9, 2.0, 1.9, 1.8, 1.7, 1.6, 1.5],
            },
        'Constant Power Overspeed, Lookup Table': {
            'VS_FBP': 1,                    # Constant power overspeed
            'VS_ControlMode': 1,            # Region 2 mode paired with VS_FBP = 1
            'VS_FBP_speed_mode': 1,
            'VS_FBP_U': [2.0, 4.0],
            'VS_FBP_P': [1.0, 1.0],
            },
        }

    ### Tune controller cases
    controllers = []
    for overrides in control_configs.values():
        controller = ROSCO_controller.Controller({**controller_params, **overrides})
        controller.tune_controller(turbine)
        controllers.append(controller)

    fig, axs = plt.subplots(3,1)
    for (label, cont, line_style) in zip(control_configs, controllers, ['-','--','-','-','-',':']):
        axs[0].plot(cont.v, cont.power_op, label=label, linestyle=line_style)
        axs[1].plot(cont.v, cont.omega_gen_op, label=label, linestyle=line_style)
        axs[2].plot(cont.v, cont.tau_op, label=label, linestyle=line_style)
    axs[0].set_ylabel('Gen Power [W]')
    axs[1].set_ylabel('Gen Speed [rad/s]')
    axs[2].set_ylabel('Gen Torque [N m]')
    axs[2].set_xlabel('Flow Speed [m/s]')
    axs[0].legend(loc='upper left', bbox_to_anchor=(.2, 2.35))

    fig.align_ylabels()
    plt.subplots_adjust(hspace=0.5)
                        

    if False:
        plt.show()
    else:
        fig_fname = os.path.join(example_out_dir, '31_fixed_pitch_mhk_sched.png')
        print('Saving figure ' + fig_fname)
        plt.savefig(fig_fname,bbox_inches='tight',)

    # Simulate all control configurations, in parallel
    run_dir = os.path.join(example_out_dir, '31_MHK')
    os.makedirs(run_dir,exist_ok=True)

    # simulation set up
    if FULL_TEST:
        TMax = 60
    else:
        TMax = 5

    r = run_FAST_ROSCO()
    r.tuning_yaml   = parameter_filename
    r.wind_case_fcn = cl.power_curve
    r.wind_case_opts    = {
        'U': [3.0],
        'TMax': TMax,
        }
    r.case_inputs = {}
    r.control_sweep_fcn = cl.sweep_yaml_input
    r.control_sweep_opts = {'param_sweeps': list(control_configs.values())}
    r.save_dir      = run_dir
    r.rosco_dir     = rosco_dir
    r.n_cores       = min(len(control_configs), os.cpu_count())

    r.run_FAST()

    op = output_processing.output_processing()
    out_files = [os.path.join(run_dir,f'RM1_MHK_FBP_{i}.out') for i in range(len(control_configs))]
    fast_out = op.load_fast_out(out_files, tmin=0)
    assert len(fast_out) == len(control_configs)

    fig, axs = plt.subplots(4,1)
    for label, fo in zip(control_configs, fast_out):
        axs[0].plot(fo['Time'], fo['Wind1VelX'],             label=label)
        axs[1].plot(fo['Time'], fo['GenSpeed'] * 2*np.pi/60, label=label)
        axs[2].plot(fo['Time'], fo['GenTq'] * 1e3,           label=label)
        axs[3].plot(fo['Time'], fo['GenPwr'] * 1e3,          label=label)
    axs[0].set_ylabel('Flow Speed [m/s]',rotation=0, labelpad=50)
    axs[1].set_ylabel('Gen Speed [rad/s]',rotation=0, labelpad=50)
    axs[2].set_ylabel('Gen Torque [N m]',rotation=0, labelpad=50)
    axs[3].set_ylabel('Gen Power [W]',rotation=0, labelpad=50)
    axs[3].set_xlabel('Time [s]')
    axs[0].legend(loc='upper left', bbox_to_anchor=(.2, 1.9))

    plt.subplots_adjust(hspace=0.5)
    fig.align_ylabels()


    # TODO: Compare result to desired operating schedule

    if False:
        plt.show()
    else:
        fig_fname = os.path.join(example_out_dir, '31_fixed_pitch_mhk_sim.png')
        print('Saving figure ' + fig_fname)
        plt.savefig(fig_fname, bbox_inches='tight')


if __name__=="__main__":
    main()
