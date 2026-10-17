# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""The steps of a run, in the order they run.

A step is a stage of the pipeline, or a phase of a stage that has phases. ``[run] stop_after`` and
``mqt-scpd plan --stop-after`` name one of them as the last step a run takes. A later release
appends its steps to the end, so a step name keeps its place in the order.
"""

from __future__ import annotations

__all__ = ["STEPS"]

#: The steps this release runs, in order. Each is a stage that writes one artifact. The Global stage
#: runs before the Assignment stage: the assignment consumes the outer port ring that the inner
#: circuit extends with the coupler ports it surfaces at.
STEPS: tuple[str, ...] = ("capacity", "global", "assign", "corridor")
