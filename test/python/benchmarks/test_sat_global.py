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

import json
import os
import random
import sys
import time
from typing import TYPE_CHECKING, Any

import pytest
from sat_benchmarks import CHIPS, CLASSES, gated, plan_to_the_insertion, write_report

if TYPE_CHECKING:
    from pathlib import Path

    from mqt.scpd import pyscpd

pytestmark = [gated, pytest.mark.skipif(not hasattr(os, "fork"), reason="the cases route in a forked child")]

#: How many states beyond the committed one a chip contributes, and the seed of their choice.
CASES = int(os.environ.get("MQT_SCPD_SAT_CASES", "12"))
SEED = 2026
RUNS_PER_SEGMENT = 8
SPAN = 3


def routed_bad(session: pyscpd.CouplerSession) -> int:
    """Route the feedlines in a forked child and return how many wires ended bad.

    Returns:
        The bad count the feedline routing reports.
    """
    reading, writing = os.pipe()
    pid = os.fork()
    if pid == 0:  # pragma: no cover - the child reports through the pipe and exits
        os.close(reading)
        try:
            routed = json.loads(session.route_feedlines())
            os.write(writing, str(routed["bad"]).encode())
        finally:
            os.close(writing)
            os._exit(0)
    os.close(writing)
    with os.fdopen(reading) as answer:
        text = answer.read()
    _, status = os.waitpid(pid, 0)
    assert status == 0, f"the routing child ended with status {status}"
    return int(text)


def cases_of(compared: dict[str, Any], rng: random.Random) -> list[list[tuple[int, int]]]:
    """Choose the moves of the cases: runs the local check called SAT, one per chain at most.

    Returns:
        One list of (coupler, option) moves per case; the first case moves nothing.
    """
    by_chain: dict[int, list[list[tuple[int, int]]]] = {}
    for run in compared["runs"]:
        if run["slot"]["verdict"] != "SAT" or not run["moves"]:
            continue
        by_chain.setdefault(int(run["chain"]), []).append([(int(m["coupler"]), int(m["option"])) for m in run["moves"]])
    chains = sorted(by_chain)
    cases: list[list[tuple[int, int]]] = [[]]
    seen = {()}
    tries = 0
    while len(cases) < CASES + 1 and chains and tries < 100 * CASES:
        tries += 1
        chosen = rng.sample(chains, rng.randint(1, len(chains)))
        moves = [move for chain in chosen for move in rng.choice(by_chain[chain])]
        key = tuple(sorted(moves))
        if key not in seen:
            seen.add(key)
            cases.append(moves)
    return cases


@pytest.mark.parametrize("chip", CHIPS)
def test_the_whole_chip_check_is_never_optimistic(chip: str, tmp_path: Path) -> None:
    """A whole-chip state the capacity check calls SAT routes with no bad wire."""
    directory, config = plan_to_the_insertion(chip, tmp_path / chip)
    session = directory.coupler_session(config)
    began = time.monotonic()
    compared = json.loads(session.compare_locally(RUNS_PER_SEGMENT, SPAN))
    committed = {int(c["index"]): int(c["chosen"]) for c in json.loads(session.couplers())["couplers"]}
    classes = dict.fromkeys(CLASSES, 0)
    judged: list[dict[str, Any]] = []
    # The cases are a fixed sample of the runs, not a secret.
    rng = random.Random(SEED)  # ruff: ignore[suspicious-non-cryptographic-random-usage]
    for moves in cases_of(compared, rng):
        for coupler, option in committed.items():
            session.set_option(coupler, option)
        for coupler, option in moves:
            session.set_option(coupler, option)
        verdict = json.loads(session.insertion_check())["verdict"]
        bad = routed_bad(session)
        name = ("sat" if verdict == "SAT" else "unsat") + ("Clean" if bad == 0 else "Failing")
        classes[name] += 1
        judged.append({"moves": moves, "verdict": verdict, "bad": bad})
        print(f"global {chip}: {len(moves)} moves · {verdict} · bad {bad}", file=sys.stderr)
    table = {
        "chip": chip,
        "cases": len(judged),
        "classes": classes,
        "judged": judged,
        "seconds": time.monotonic() - began,
    }
    write_report("global", chip, table)

    assert table["cases"] > 0
    assert classes["satFailing"] == 0, f"{classes['satFailing']} states the check called SAT routed with bad wires"
