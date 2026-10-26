# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""The global SAT benchmark: the whole-chip check against the feedline routing of the chip.

The cases are whole-chip states: the chip as the coupler insertion left it, and states that move
the couplers of one or more chains onto runs of options the local comparison found. Each case is
judged by the whole-chip check of the insertion and then routed by the feedline routing, in a child
process so that the session keeps its own state. The check is meant to be pessimistic: a state it
calls SAT routes with no bad wire. ``docs/routing.md`` explains the four classes.
"""

from __future__ import annotations

import os
import sys
from typing import TYPE_CHECKING

import pytest
from sat_benchmarks import CHIPS, gated, plan_to_the_insertion, write_report

from mqt.scpd.debug import bench

if TYPE_CHECKING:
    from pathlib import Path

pytestmark = [gated, pytest.mark.skipif(not hasattr(os, "fork"), reason="the cases route in a forked child")]

#: How many states beyond the committed one a chip contributes.
CASES = int(os.environ.get("MQT_SCPD_SAT_CASES", str(bench.CASES)))


def _say(text: str) -> None:
    print(text, file=sys.stderr)


@pytest.mark.parametrize("chip", CHIPS)
def test_the_whole_chip_check_is_never_optimistic(chip: str, tmp_path: Path) -> None:
    """A whole-chip state the capacity check calls SAT routes with no bad wire."""
    directory, config = plan_to_the_insertion(chip, tmp_path / chip)
    session = directory.coupler_session(config)
    local = bench.local_cases(session, bench.RUNS_PER_SEGMENT, bench.SPAN)
    table = bench.global_cases(session, local, CASES, say=_say)
    table["chip"] = chip
    write_report("global", chip, table)

    classes = table["classes"]
    assert table["cases"] > 0
    assert classes["satFailing"] == 0, f"{classes['satFailing']} states the check called SAT routed with bad wires"
