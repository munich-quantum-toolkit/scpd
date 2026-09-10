# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""The design-rule report of a run.

The core renders ``drc.json``, because the report is schema-defined and a violation has to stay
machine-readable. What is here reads it back and counts it, which is what decides the exit code.
"""

from __future__ import annotations

import json

__all__ = ["findings_of", "summarize"]


def findings_of(text: str) -> list[dict]:
    """Every finding of every report in ``drc.json``.

    Returns:
        The findings, in the order the checker produced them.
    """
    written = json.loads(text)
    return [finding for report in written.get("reports", []) for finding in report.get("findings", [])]


def summarize(text: str) -> tuple[int, int]:
    """Count the findings of ``drc.json``.

    Only the active ones decide the exit code; an advisory rule is compiled and unit-tested but
    never fails a run.

    Returns:
        How many active and how many advisory findings the report holds.
    """
    active = 0
    advisory = 0
    for finding in findings_of(text):
        if finding.get("severity") == "advisory":
            advisory += 1
        else:
            active += 1
    return active, advisory
