"""
26_marine_hydro
---------------
Run MHK turbine in OpenFAST with ROSCO torque controller
"""

import os
from rosco.toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO
from rosco.toolbox.ofTools.case_gen import CaseLibrary as cl

TEST_RUN = True

def main():
    #directories
    this_dir            = os.path.dirname(os.path.abspath(__file__))
    rosco_dir           = os.path.dirname(this_dir)
    example_out_dir     = os.path.join(this_dir,'examples_out')
    os.makedirs(example_out_dir,exist_ok=True)

    # Input yaml and output directory
    parameter_filename = os.path.join(this_dir,'Tune_Cases/RM1_MHK.yaml')
    run_dir = os.path.join(example_out_dir,'26_MHK/26_full_run')
    os.makedirs(run_dir,exist_ok=True)
    full_run = True


    # simulation set up
    r = run_FAST_ROSCO()
    r.tuning_yaml   = parameter_filename
    # r.wind_case_fcn = cl.power_curve
    # r.wind_case_opts    = {
    #     'U': [2.5],
    #     'TMax': 100.0,
    #     }
    r.wind_case_fcn = cl.user_hh
    if full_run: 
        r.wind_case_opts    = {
            'wind_filenames': ['/Users/dzalkind/Tools/ROSCO-main/Examples/26_MHK/Cook_Inlet.txt'],
            'TMax': 23750.0,
            }
        r.case_inputs = {}
        r.case_inputs[("Fst","DT_Out")]      = {'vals':[1.], 'group':0}
        r.case_inputs[("ElastoDyn","RotSpeed")]      = {'vals':[2.], 'group':0}
        r.controller_params = {}
        r.controller_params['LoggingLevel'] = 0
    else:
        r.wind_case_opts    = {
            'wind_filenames': ['/Users/dzalkind/Tools/ROSCO-main/Examples/26_MHK/Cook_Inlet_subset.txt'],
            'TMax': 1000.0,
            }
    # r.control_sweep_fcn = cl.sweep_yaml_input
    r.control_sweep_opts = {
            'control_param': 'ptfm_freq',
            'param_values': [.2,.4,.6,.8, 1.0],
            # 'discon_param': 'F_FlHighPassFreq',
            # 'param_values': [0.01, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5],
            # 'control_param': 'omega_pc',
            # 'param_values': [0.3, 0.5, 0.7, 0.9]
        }
    # r.case_inputs = {}
    # r.controller_params = {}
    # r.controller_params['LoggingLevel'] = 2
    # r.controller_params['DISCON'] = {}
    # r.controller_params['DISCON']['VS_ConstPower'] = 0
    r.save_dir      = run_dir
    r.rosco_dir     = rosco_dir
    r.n_cores = 1

    r.run_FAST()



if __name__=="__main__":
    main()
