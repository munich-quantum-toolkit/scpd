# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""The local SAT benchmark: the slot check of one feedline against a rip-up and re-route of its wires.

For every segment of every chain, the coupler session searches up to eight runs of options as the
targeted repair does, applies each, and judges every feedline the run may touch twice: by the slot
check of that feedline's wires alone, and by ripping those wires up and routing them again. The
check is meant to be pessimistic: a feedline it calls SAT routes clean. ``docs/routing.md`` explains
the four classes.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import pytest
from sat_benchmarks import CHIPS, gated, plan_to_the_insertion, write_report

from mqt.scpd.debug import bench

if TYPE_CHECKING:
    from pathlib import Path

pytestmark = gated


@pytest.mark.parametrize("chip", CHIPS)
def test_the_local_slot_check_is_never_optimistic(chip: str, tmp_path: Path) -> None:
    """A feedline the local slot check calls SAT routes clean when its wires alone are rerouted."""
    directory, config = plan_to_the_insertion(chip, tmp_path / chip)
    session = directory.coupler_session(config)
    table = bench.local_cases(session, bench.RUNS_PER_SEGMENT, bench.SPAN)
    table["chip"] = chip
    write_report("local", chip, table)

    classes = table["classes"]
    assert table["cases"] > 0
    assert classes["satFailing"] == 0, f"{classes['satFailing']} feedlines the check called SAT failed to route"
