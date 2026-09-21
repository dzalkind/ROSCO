"""Pytest wrapper around run_regression.py — one test per scenario.

`python run_regression.py` remains the way to debug a failure (it prints
max_diff / first_diff_idx per array); this exists so CI reports which scenario
broke instead of one opaque pass/fail.
"""

import pytest

import run_regression as rr


@pytest.fixture(scope="session")
def work_dir(tmp_path_factory):
    """Scratch dir shared by all scenarios: scenario_N.npz plus the generated
    DISCON_*.IN and the controller's *.RO.dbg* output."""
    rr.build_scrub()
    return str(tmp_path_factory.mktemp("regression"))


@pytest.mark.parametrize("scenario", rr.ALL_SCENARIOS)
def test_scenario_matches_baseline(scenario, work_dir):
    assert rr.run_scenario(scenario, work_dir), f"scenario {scenario} subprocess failed"
    identical, detail = rr.compare_scenario(scenario, work_dir)
    assert identical, detail


def test_hdf5_matches_text_output(work_dir):
    """Scenario 28 writes the same simulation as scenario 1 in HDF5 format."""
    pytest.importorskip("h5py")
    assert rr.run_scenario(1, work_dir), "scenario 1 subprocess failed"
    assert rr.run_scenario(28, work_dir), "scenario 28 subprocess failed"
    identical, detail = rr.compare_hdf5_debug(work_dir)
    if identical is None:
        pytest.skip(detail)
    assert identical, detail
