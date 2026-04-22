import numpy as np
import pandas as pd
import os
import sys
import matplotlib.pyplot as plt
from openfast_io.turbsim_file import TurbSimFile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '0_shared'))
import config as wt_config
from utils import interpolate_1d


def main():
    os.chdir(os.path.dirname(__file__))
    df_6dof = pd.read_csv(wt_config.STEADY_6DOF_LOOKUP_CSV)

    if False: # read turbsim file and make plane-average input

        ts_file = TurbSimFile(wt_config.TURBSIM_BTS_FILE)
        ts_file.read()
        u_avg = np.mean(ts_file['u'][0,:, :, :], axis=(1,2))
        u_tt = ts_file['t']

        plt.plot(u_tt, u_avg)
        plt.xlabel('Time (s)')
        plt.ylabel('Average Wind Speed (m/s)')
        plt.title('Plane-Average Wind Speed')
        plt.show()

        M = np.c_[u_tt,u_avg]
        np.savetxt(wt_config.PLANE_AVG_WIND_CSV, M, header='Time(s),U_avg(m/s)', delimiter=',')
    else:
        df_u = pd.read_csv(wt_config.FILTERED_WIND_CSV)
        u_avg = df_u['U_avg(m/s)'].to_numpy()
        u_tt = df_u['# Time(s)'].to_numpy()

    # Interpolate df based on u_avg and plot the 6DOF responses

    responses = wt_config.CHANNELS_6DOF

    interp_outs = {}

    interp_outs['Time'] = u_tt
    for r in responses:
        interp_outs[r] = interpolate_1d(df_6dof, 'Wind1VelX', r, u_avg)

    if True:
        fig, axs = plt.subplots(7, 1, figsize=(6, 10))
        axs[0].plot(u_tt, u_avg)
        axs[0].set_title('Average Wind Speed')
        for i, r in enumerate(responses):
            axs[i+1].plot(u_tt, interp_outs[r])
            axs[i+1].set_title(r)
            axs[i+1].set_ylabel(r)
        plt.tight_layout()
        axs[-1].set_xlabel('Time (s)')
        plt.show()
        fig.align_ylabels()

    pd.DataFrame(interp_outs).to_csv(wt_config.INTERP_6DOF_RESP_CSV, index=False)




if __name__ == "__main__":
    main()