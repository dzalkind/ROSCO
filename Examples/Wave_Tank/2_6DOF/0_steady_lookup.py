import numpy as np
import pandas as pd
import os
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

from openfast_io.turbsim_file import TurbSimFile


def interpolate_1d(
    df: pd.DataFrame,
    x_col: str,
    y_col: str,
    x_query: float | list | np.ndarray,
    extrapolate: bool = False,
) -> np.ndarray:
    """
    Linearly interpolate y values from a DataFrame given one or more x query points.

    Parameters
    ----------
    df : pd.DataFrame
        Source data containing x and y columns.
    x_col : str
        Name of the column to use as the independent variable (x).
    y_col : str
        Name of the column to use as the dependent variable (y).
    x_query : float or array-like
        One or more x values to interpolate at.
    extrapolate : bool, optional
        If True, linearly extrapolate beyond the data range.
        If False (default), raises ValueError for out-of-range queries.

    Returns
    -------
    np.ndarray
        Interpolated y values, same length as x_query.

    Raises
    ------
    ValueError
        If x_col has duplicate values, or if x_query is out of range
        and extrapolate=False.
    """
    df_sorted = df[[x_col, y_col]].dropna().sort_values(x_col)
    x = df_sorted[x_col].to_numpy(dtype=float)
    y = df_sorted[y_col].to_numpy(dtype=float)

    if len(np.unique(x)) != len(x):
        raise ValueError(
            f"Column '{x_col}' contains duplicate values — "
            "interpolation requires unique x points."
        )

    x_query = np.atleast_1d(np.asarray(x_query, dtype=float))

    fill = "extrapolate" if extrapolate else None
    f = interp1d(x, y, kind="linear", bounds_error=not extrapolate, fill_value=fill)

    return f(x_query)


def main():
    os.chdir(os.path.dirname(__file__))
    df_6dof = pd.read_csv('steady_6dof_lookup.csv')

    if False: # read turbsim file and make plane-average input

        ts_file = TurbSimFile('/Users/dzalkind/Downloads/weis_job_0_NTM_U11.400000_Seed438466540.0.bts')
        ts_file.read()
        u_avg = np.mean(ts_file['u'][0,:, :, :], axis=(1,2))
        tt = ts_file['t']

        plt.plot(tt, u_avg)
        plt.xlabel('Time (s)')
        plt.ylabel('Average Wind Speed (m/s)')
        plt.title('Plane-Average Wind Speed')
        plt.show()

        M = np.c_[tt,u_avg]
        np.savetxt('plane_avg_wind.csv', M, header='Time(s),U_avg(m/s)', delimiter=',')
    else:
        df_u = pd.read_csv('plane_avg_wind.csv')
        u_avg = df_u['U_avg(m/s)'].to_numpy()
        u_tt = df_u['# Time(s)'].to_numpy()

    # Interpolate df based on u_avg and plot the 6DOF responses

    responses = [
        'RtAeroFxi', 'RtAeroFyi', 'RtAeroFzi',
        'RtAeroMxi', 'RtAeroMyi', 'RtAeroMzi',
    ]

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

    pd.DataFrame(interp_outs).to_csv('interp_6dof_responses.csv', index=False)




if __name__ == "__main__":
    main()