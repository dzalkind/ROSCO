"""
0_shared/utils.py
Shared utility functions for the Wave Tank model verification workflow.
"""
import numpy as np
import pandas as pd
from scipy.interpolate import interp1d


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
