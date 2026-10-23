# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""The steps of a run, in the order they run.

A step is a stage of the pipeline. ``[run] stop_after`` and ``mqt-scpd plan --stop-after`` name
one of them as the last step a run takes, or one of the five phases of the Final stage, which then
stops after that phase. A later release appends its steps to the end, so a step name keeps its
place in the order.
"""

from __future__ import annotations

__all__ = ["FINAL_PHASES", "STEPS"]

#: The steps this release runs, in order. Each is a stage that writes one artifact. The Global stage
#: runs before the Assignment stage: the assignment consumes the outer port ring that the inner
#: circuit extends with the coupler ports it surfaces at.
STEPS: tuple[str, ...] = ("capacity", "global", "assign", "corridor", "detail", "final")

#: The five phases of the Final stage, in the order they run: the routing before the feedlines and
#: its refinement, the feedline insertion, the routing under the feedline constraints and its
#: refinement. Each leaves a snapshot in the artifact, and a run may stop after any of them.
FINAL_PHASES: tuple[str, ...] = (
    "pre-feedline-routing",
    "pre-feedline-refinement",
    "feedline-insertion",
    "feedline-constrained-routing",
    "feedline-constrained-refinement",
)
