"""Keeps mode_coverage.py honest. No DLL, runs in well under a second.

The coverage report is only as good as its MODES table. These fail when the
controller gains a mode the table does not know about, or when a fixture sets a
mode to a value the table does not list — either way the report would silently
under-count.
"""

import os

import mode_coverage as mc


def test_every_registry_mode_is_tabulated():
    missing = sorted(mc.registry_modes() - set(mc.MODES))
    assert not missing, f"add to MODES in mode_coverage.py: {missing}"


def test_fixtures_use_only_known_mode_values():
    unknown = []
    for kind, label, settings in mc.collect_sources():
        if kind != "regression":
            continue
        for mode, value in settings.items():
            values, _ = mc.MODES[mode]
            if mode == "PS_Mode":
                value = min(value, 1)
            if value not in values:
                unknown.append(f"scenario {label}: {mode}={value}")
    assert not unknown, unknown


def test_every_fixture_sets_every_mode():
    """A fixture missing a mode line would be read as 'not configured' rather
    than as its default, and the report would be wrong about it."""
    fixtures = [s for s in mc.collect_sources() if s[0] == "regression"]
    assert len(fixtures) == len(os.listdir(mc.FIXTURES_DIR))
    for _, label, settings in fixtures:
        assert set(settings) == set(mc.MODES), f"scenario {label}: {set(mc.MODES) - set(settings)}"
