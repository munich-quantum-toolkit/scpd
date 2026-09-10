# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Reading a design-rule report back.

The core renders ``drc.json``; what is checked here is that a report reads as what it says and
that only the active findings decide the exit code.
"""

from __future__ import annotations

import json

import pytest

from mqt.scpd.drc import findings_of, summarize

EMPTY = json.dumps({"reports": [{"stage": "final", "findings": [], "feedlines_skipped": 0}]})

MIXED = json.dumps({
    "reports": [
        {
            "stage": "final",
            "feedlines_skipped": 3,
            "findings": [
                {"rule": "wire-clearance", "severity": "active", "wires": [1, 2], "measured": 170.0},
                {"rule": "min-bend-radius", "severity": "advisory", "wires": [4], "measured": 40.0},
            ],
        }
    ]
})


def test_a_report_without_findings_counts_nothing() -> None:
    """A run that holds every rule has an empty report, not a missing one."""
    assert summarize(EMPTY) == (0, 0)
    assert findings_of(EMPTY) == []


def test_only_the_active_findings_decide_the_exit_code() -> None:
    """An advisory rule is compiled and unit-tested but never fails a run."""
    assert summarize(MIXED) == (1, 1)
    assert len(findings_of(MIXED)) == 2


def test_a_report_that_is_not_json_is_refused() -> None:
    """The report is machine-readable or it is nothing."""
    with pytest.raises(json.JSONDecodeError):
        summarize("not a report")
