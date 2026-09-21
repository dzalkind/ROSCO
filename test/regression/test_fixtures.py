"""Every committed fixture is scenario 1's fixture plus that scenario's `patches`.

The baselines catch a hand-edit to a fixture that changes behaviour. They cannot
catch one that does not — a parameter inert under the scenario's modes — which
would persist silently and mislead the next reader. This closes that gap. No DLL
and no tuner, so it runs in well under a second.

A failure means a fixture and its recipe in scenarios.py disagree. Fix the
`patches` and regenerate (`python test/regression/scenarios.py --write-fixtures`);
never hand-edit a fixture.
"""

import difflib
import glob
import os

import pytest

import scenarios


def _read(path):
    with open(path) as f:
        return f.read()


@pytest.mark.parametrize("num", sorted(scenarios.SCENARIOS))
def test_fixture_matches_its_patches(num):
    expected = scenarios.apply_patches(_read(scenarios.TUNER_FIXTURE), scenarios.SCENARIOS[num].patches)
    path = scenarios.fixture_path(num)
    actual = _read(path)
    if expected != actual:
        diff = "\n".join(difflib.unified_diff(
            expected.splitlines(), actual.splitlines(),
            fromfile="scenario_01.IN + patches", tofile=os.path.basename(path), lineterm="",
        ))
        raise AssertionError(diff)


def test_one_fixture_per_scenario():
    on_disk = {os.path.basename(p) for p in glob.glob(os.path.join(scenarios.FIXTURES_DIR, "*.IN"))}
    expected = {os.path.basename(scenarios.fixture_path(n)) for n in scenarios.SCENARIOS}
    assert on_disk == expected


def test_scenario_1_is_unpatched():
    # test_tuning.py pins scenario_01.IN as the raw tuner output.
    assert not scenarios.SCENARIOS[1].patches
