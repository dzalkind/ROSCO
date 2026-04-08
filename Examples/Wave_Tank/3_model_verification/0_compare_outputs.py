import sys
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from rosco.toolbox.ofTools.fast_io.output_processing import output_processing
from scipy.signal import butter, sosfiltfilt

def plot_not_available(ax, channel):
    return
    ax.set_ylabel(channel)
    ax.text(0.5, 0.5, 'not available', transform=ax.transAxes, ha='center')
    ax.grid(True)

def main():
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    

    channels = [
        'RtVAvgxh',
        'BldPitch1', 'GenSpeed', 'GenTq', 'GenPwr',
        'RtAeroFxi', #'RtAeroFyi', 'RtAeroFzi',
        'RtAeroMxi', #'RtAeroMyi', 'RtAeroMzi',
    ]


    ## 6-DOF
    u_rot = pd.read_csv('/Users/dzalkind/Tools/ROSCO-USFLOWT/Examples/Wave_Tank/2_6DOF/plane_avg_wind.csv')
    resp_6 = pd.read_csv('/Users/dzalkind/Tools/ROSCO-USFLOWT/Examples/Wave_Tank/2_6DOF/interp_6dof_responses.csv')

    fig, axs = plt.subplots(len(channels), 1, sharex=True, figsize=(10, 2 * len(channels)), constrained_layout=True)
    

    axs[0].plot(resp_6['Time'], u_rot['U_avg(m/s)'])
    axs[0].set_title('Average Wind Speed')
    for i, r in enumerate(channels[1:]):
        if r in resp_6.columns:
            axs[i+1].plot(resp_6['Time'], resp_6[r],color='C0',label='6-DOF Lookup')
            axs[i+1].set_ylabel(r)
        else:
            plot_not_available(axs[i+1], r)



    ## 1-DOF
    resp_1 = pd.read_csv('/Users/dzalkind/Tools/ROSCO-USFLOWT/Examples/Wave_Tank/1dof_sim_outs.csv')

    resp_1['GenPwr'] = resp_1['GenTq'] * resp_1['GenSpeed'] * 2 * np.pi / 60  # convert to kW

    for ax, channel in zip(axs, channels):
        if channel in resp_1.columns:
            ax.plot(resp_1['Time'], resp_1[channel], color='C1',label='1-DOF Sim',alpha=0.7)
            ax.set_ylabel(channel)
        else:
            plot_not_available(ax, channel)

    axs[-1].set_xlabel('Time (s)')

    
    
    ## OpenFAST output file
    filename = '/Users/dzalkind/Library/CloudStorage/Box-Box/USFLOWT_PII/05_SimModel/OpenFAST/2026.04.01_WaveVerification/rank_0/DLC1.6_6_weis_job_1.out'

    op = output_processing()
    fastout = op.load_fast_out(filename)

    # fastout is a list; get the single dict
    fast_data = fastout[0]

    if True:
        dt = np.mean(np.diff(fast_data['Time']))
        sos = butter(4, 1.0 / 2.0, btype='low', fs=1.0 / dt, output='sos')
        rt_avg = sosfiltfilt(sos, fast_data['RtVAvgxh'])
        
        pd.DataFrame({'# Time(s)': fast_data['Time'], 'U_avg(m/s)': rt_avg}).to_csv('filtered_rt_vavghx.csv', index=False)


    for ax, channel in zip(axs, channels):
        if channel in fast_data:
            ax.plot(fast_data['Time'], fast_data[channel], color='C2',label='OpenFAST',alpha=0.7)
            unit_idx = fast_data['meta']['channels'].index(channel)
            unit = fast_data['meta']['attribute_units'][unit_idx]
            ax.set_ylabel(f'{channel}\n({unit})')
        else:
            plot_not_available(ax, channel)
    [a.grid() for a in axs]
    axs[-1].set_xlabel('Time (s)')
    axs[0].legend(loc='upper right')

    plt.tight_layout()
    axs[-1].set_xlabel('Time (s)')
    fig.align_ylabels()

    plt.show()

    print('here')


if __name__=="__main__":
    main()
