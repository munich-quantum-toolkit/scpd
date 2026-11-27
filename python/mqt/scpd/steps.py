# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""The steps of a run, in the order they run.

Set the last planning step with ``[run] stop_after`` or ``mqt-scpd plan --stop-after``.
"""

from __future__ import annotations

__all__ = ["STEPS"]

#: The planning steps, in order. Each is a stage that writes one artifact. The Global stage
#: runs before the Assignment stage: the assignment consumes the outer port ring that the inner
#: circuit extends with the coupler ports it surfaces at.
STEPS: tuple[str, ...] = ("capacity", "global", "assign", "corridor", "detail")
