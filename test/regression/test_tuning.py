"""Regression test for the Python tuning toolbox.

`NREL5MW.yaml` → `tune_controller()` → `write_DISCON()` must still reproduce
`fixtures/scenario_01.IN`. No DLL and no simulation, so this runs in seconds and
fails with a text diff naming the parameters that moved — rather than as a float
mismatch in a time series that points at the C++ controller.

Scenario 1 is the one scenario that applies no patches, so its fixture *is* the
raw tuner output. Pinning that file means the artifact this test guards is
exactly the one scenario 1 runs on, so the tuning check and the controller
check cannot drift apart — there is only one file. If scenario 1 ever gains
patches, this test must switch to a fixture of its own.

A failure here means the *tuner* changed: either deliberately (a tuning
improvement) or accidentally (a wisdem/scipy upgrade). Either way it is a
separate decision from a controller regression. See README.md.
"""

import difflib

import scenarios

def _comparable(path):
    # scenarios._make_portable() already strips the version/date stamp from both
    # sides, so a plain line-by-line comparison is exact.
    with open(path) as f:
        return f.read().splitlines()


def test_tuner_reproduces_base_fixture(tmp_path):
    turbine, controller, cp_filename = scenarios.load_turbine_and_controller()
    generated = scenarios.write_tuner_output(
        turbine, controller, cp_filename, path=str(tmp_path / "tuner_output.IN")
    )

    expected = _comparable(scenarios.TUNER_FIXTURE)
    actual = _comparable(generated)

    if expected != actual:
        diff = "\n".join(difflib.unified_diff(
            expected, actual, fromfile="fixtures/scenario_01.IN",
            tofile="tuner output", lineterm="",
        ))
        raise AssertionError(
            "The tuner no longer reproduces fixtures/scenario_01.IN.\n"
            "If this change is intended, regenerate with:\n"
            "    python test/regression/scenarios.py --write-fixtures\n"
            "and commit fixtures/ separately, with justification.\n\n" + diff
        )
