# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Tests of the SAT dashboards and of the routes a coupler server adds for them."""

from __future__ import annotations

import copy
import http.client
import json
import re
import threading
from http.server import HTTPServer
from pathlib import Path
from typing import TYPE_CHECKING

from mqt.scpd.cli import main
from mqt.scpd.debug import pages, serve
from mqt.scpd.debug.serve import Answer, CouplerServer, Recorder, Request
from mqt.scpd.flatbuffers.config.Config import ConfigT
from mqt.scpd.flatbuffers.config.DebugParams import DebugParamsT
from mqt.scpd.run import RunDirectory

if TYPE_CHECKING:
    from typing import Any

BENCHMARKS = Path(__file__).resolve().parents[3] / "benchmarks"

#: The coupler of every waypoint of the two chains of the fake chip.
WAYPOINTS = {0: [None, 0, 1, None], 1: [None, 2, None]}
EDGES = {0: ["f0", "f1", "f2"], 1: ["f3", "f4"]}
#: The slot of the first feedline edge of every chain.
FIRST_EDGE = {0: 0, 1: 3}


def graph(marker: str, verdict: str = "SAT") -> dict[str, Any]:
    """A capacity graph whose summary names the state it was judged on.

    Returns:
        The graph.
    """
    return {
        "width": 1,
        "height": 1,
        "raster": [1],
        "nodes": [],
        "edges": [],
        "wires": [],
        "verdict": verdict,
        "summary": [marker],
    }


def state(chain: int, options: list[int], check: str, *, committed: bool = False) -> dict[str, Any]:
    """A `chain-state` record.

    Returns:
        The record.
    """
    first = [0, 0, 0, 0] if chain == 0 else [0, 0, 0]
    moved = [
        {"coupler": c, "from": f, "to": t}
        for c, f, t in zip(WAYPOINTS[chain], first, options, strict=True)
        if c is not None and f != t
    ]
    return {
        "chain": chain,
        "committed": committed,
        "segment": None if committed else 1,
        "span": 2,
        "atOver": True,
        "options": options,
        "couplers": WAYPOINTS[chain],
        "moved": moved,
        "check": check,
        "verdict": "UNSAT" if check == "none" else "SAT",
        "lost": [] if check == "standing" else ["w5"],
        "seconds": 0.25,
    }


def step(chain: int, at: int, options: list[int], verdict: str, *, refused: bool = False) -> dict[str, Any]:
    """A `step-check` record: the options of the waypoints after the step stay on 0.

    Returns:
        The record.
    """
    lost = [] if verdict == "SAT" else [f"w{5 + 2 * chain + at}"]
    return {
        "chain": chain,
        "step": at,
        "edge": FIRST_EDGE[chain] + at,
        "options": options,
        "couplers": WAYPOINTS[chain],
        "verdict": verdict,
        "lost": lost,
        "closed": lost,
        "refused": refused,
        "seconds": 0.01,
    }


def sat_records(*, searched: bool = True) -> dict[str, list[dict[str, Any]]]:
    """The records of a run whose chip is UNSAT after the commit and SAT after a search.

    The chain search checked feedline 0 SAT at every edge, though at edge f1 on UNSAT options
    alone, and feedline 1 UNSAT at its last edge f4. Feedline 0 has two SAT states, one found by
    the standing check and one by the alone check. Feedline 1 has no SAT state. Combination 1 is
    UNSAT, combination 2 SAT.

    Returns:
        The records by kind.
    """
    steps = [
        step(0, 0, [0, 1, 0, 0], "SAT"),
        step(0, 0, [0, 2, 0, 0], "UNSAT"),
        step(0, 1, [0, 1, 2, 0], "UNSAT", refused=True),
        step(0, 1, [0, 1, 1, 0], "UNSAT"),
        step(0, 2, [0, 1, 1, 0], "SAT"),
        step(1, 0, [0, 0, 0], "SAT"),
        step(1, 1, [0, 0, 0], "UNSAT", refused=True),
    ]
    if not searched:
        result = {"event": "result", "searched": False, "why": "SAT after the commit"}
        return {"step-check": steps, "global-sat": [result]}
    states = [
        state(0, [0, 0, 0, 0], "none", committed=True),
        state(0, [0, 1, 0, 0], "standing"),
        state(0, [0, 2, 0, 0], "alone"),
        state(0, [0, 0, 2, 0], "none"),
        state(1, [0, 0, 0], "none", committed=True),
        state(1, [0, 1, 0], "none"),
    ]
    events = [
        {"event": "start", "lost": ["w5"]},
        {"event": "states", "chain": 0, "states": 3, "options": [[0, 0, 0, 0], [0, 1, 0, 0], [0, 2, 0, 0]]},
        {"event": "states", "chain": 1, "states": 1, "options": [[0, 0, 0]]},
        {
            "event": "combination",
            "index": 1,
            "stage": "stage 1: ",
            "states": [1, 0],
            "verdict": "UNSAT",
            "lost": ["w9"],
        },
        {"event": "combination", "index": 2, "stage": "stage 1: ", "states": [2, 0], "verdict": "SAT", "lost": []},
        {
            "event": "result",
            "searched": True,
            "sat": True,
            "combinations": 2,
            "moved": "chain 0 state 2",
            "seconds": 0.3,
        },
    ]
    couplers = [
        {"index": 0, "chain": 0, "edgeIn": "f0", "edgeOut": "f1"},
        {"index": 1, "chain": 0, "edgeIn": "f1", "edgeOut": "f2"},
        {"index": 2, "chain": 1, "edgeIn": "f3", "edgeOut": "f4"},
    ]
    return {
        "chain-state": states,
        "step-check": steps,
        "global-sat": events,
        "coupler-options": [{"couplers": couplers}],
    }


def records_of(by_kind: dict[str, list[dict[str, Any]]]) -> pages.SatRecords:
    """The SAT records of records by kind.

    Returns:
        The records.
    """
    return pages.SatRecords.of(lambda kind: by_kind.get(kind, []))


def recorder(by_kind: dict[str, list[dict[str, Any]]]) -> Recorder:
    """A recorder that was handed records by kind, as a session hands them.

    Returns:
        The recorder.
    """
    kept = Recorder()
    for kind, records in by_kind.items():
        for record in records:
            kept(kind, json.dumps(record))
    return kept


class FakeSession:
    """A coupler session of two chains, three options per coupler, without the Final stage.

    A local check finds a chain SAT on the options in ``standing_sat`` (standing) and
    ``alone_sat`` (alone); the whole-chip check finds the chip SAT on the states in ``chip_sat``; a
    step check finds a step SAT on the chains, steps and options in ``step_sat``. Every graph names
    the state it was judged on.
    """

    def __init__(self, lines: list[str]) -> None:
        """Start on combination 2 of :func:`sat_records`: feedline 0 on state 2.

        Args:
            lines: Where the session says what it does.
        """
        self.lines = lines
        self.options = {0: [0, 2, 0, 0], 1: [0, 0, 0]}
        self.standing_sat = {(0, (0, 2, 0, 0)), (1, (0, 0, 0))}
        self.alone_sat = {(0, (0, 2, 0, 0))}
        self.chip_sat = {((0, 2, 0, 0), (0, 0, 0))}
        self.step_sat = {
            (s["chain"], s["step"], tuple(s["options"])) for s in sat_records()["step-check"] if s["verdict"] == "SAT"
        }
        self.moves: list[tuple[int, int]] = []

    @staticmethod
    def _where(coupler: int) -> tuple[int, int]:
        for chain, couplers in WAYPOINTS.items():
            if coupler in couplers:
                return chain, couplers.index(coupler)
        msg = f"there is no coupler {coupler}"
        raise ValueError(msg)

    def couplers(self) -> str:
        """The couplers as JSON.

        Returns:
            The JSON.
        """
        listed = []
        for chain, couplers in WAYPOINTS.items():
            for at, coupler in enumerate(couplers):
                if coupler is not None:
                    listed.append({"index": coupler, "chain": chain, "at": at, "chosen": self.options[chain][at]})
        return json.dumps({"couplers": listed})

    def set_option(self, coupler: int, option: int) -> str:
        """Put a coupler on another option.

        Returns:
            What became of its edges, as JSON.

        Raises:
            ValueError: When the coupler or the option does not exist.
        """
        chain, at = self._where(coupler)
        if option not in {0, 1, 2}:
            msg = f"coupler {coupler} has no option {option}"
            raise ValueError(msg)
        self.options[chain][at] = option
        self.moves.append((coupler, option))
        self.lines.append(f"coupler session: c{coupler} on option {option}")
        return json.dumps({"coupler": coupler, "option": option, "edges": [{"edge": EDGES[chain][at - 1]}]})

    def graph(self) -> str:
        """The capacity graph of the chip as it stands.

        Returns:
            The JSON.
        """
        self.lines.append("coupler insertion: CAPACITY GRAPH SAT")
        return json.dumps(graph(f"chip {self.options}"))

    def chains(self) -> str:
        """Every chain as it stands.

        Returns:
            The JSON.
        """
        return json.dumps({
            "chains": [
                {
                    "chain": chain,
                    "couplers": WAYPOINTS[chain],
                    "options": list(self.options[chain]),
                    "edges": EDGES[chain],
                }
                for chain in WAYPOINTS
            ]
        })

    def local_check(self, chain: int, options: list[int]) -> str:
        """The two local checks of a chain on options; the chain stays as it stands.

        Returns:
            The JSON.

        Raises:
            ValueError: When the chain or an option does not exist.
        """
        if chain not in WAYPOINTS or len(options) != len(WAYPOINTS[chain]) or any(o not in {0, 1, 2} for o in options):
            msg = f"chain {chain} has no options {options}"
            raise ValueError(msg)
        key = (chain, tuple(options))
        answer: dict[str, Any] = {"chain": chain, "options": options, "undrawn": []}
        for check, sat in (("standing", self.standing_sat), ("alone", self.alone_sat)):
            verdict = "SAT" if key in sat else "UNSAT"
            answer[check] = {
                "verdict": verdict,
                "lost": [] if verdict == "SAT" else ["w5"],
                "graph": graph(f"{check} {chain} {options}", verdict),
            }
        return json.dumps(answer)

    def step_check(self, chain: int, step: int, options: list[int]) -> str:
        """The step check of a chain at one step on options; the chain stays as it stands.

        Returns:
            The JSON.

        Raises:
            ValueError: When the chain, the step or an option does not exist.
        """
        if (
            chain not in WAYPOINTS
            or not 0 <= step < len(EDGES[chain])
            or len(options) != len(WAYPOINTS[chain])
            or any(o not in {0, 1, 2} for o in options)
        ):
            msg = f"chain {chain} has no step {step} on options {options}"
            raise ValueError(msg)
        standing = [o if at <= step + 1 else 0 for at, o in enumerate(options)]
        verdict = "SAT" if (chain, step, tuple(standing)) in self.step_sat else "UNSAT"
        lost = [] if verdict == "SAT" else ["w5"]
        return json.dumps({
            "chain": chain,
            "step": step,
            "edge": FIRST_EDGE[chain] + step,
            "options": standing,
            "verdict": verdict,
            "lost": lost,
            "closed": lost,
            "undrawn": [],
            "graph": graph(f"step {chain} {step} {standing}", verdict),
        })

    def insertion_check(self) -> str:
        """The whole-chip check of the chip as it stands.

        Returns:
            The JSON.
        """
        verdict = "SAT" if (tuple(self.options[0]), tuple(self.options[1])) in self.chip_sat else "UNSAT"
        lost = [] if verdict == "SAT" else ["w9"]
        return json.dumps({"verdict": verdict, "lost": lost, "graph": graph(f"chip {self.options}", verdict)})


def fake_server(by_kind: dict[str, list[dict[str, Any]]] | None = None) -> tuple[CouplerServer, FakeSession]:
    """A coupler server on a fake session, with the records of :func:`sat_records`.

    Returns:
        The server and its session.
    """
    lines: list[str] = []
    session = FakeSession(lines)
    return CouplerServer(session, lines, recorder(sat_records() if by_kind is None else by_kind)), session


def get(server: CouplerServer, path: str, query: dict[str, str] | None = None) -> Answer:
    """Ask the server a GET request.

    Returns:
        The answer.
    """
    return server.answer(Request("GET", path, query or {}))


def post(server: CouplerServer, path: str, body: object = None) -> Answer:
    """Ask the server a POST request.

    Returns:
        The answer.
    """
    return server.answer(Request("POST", path, {}, b"" if body is None else json.dumps(body).encode()))


def card(text: str, chain: int) -> str:
    """The card of one feedline on the feedline dashboard.

    Returns:
        The card's HTML.
    """
    found = re.search(rf'<section class="card item" id="chain-{chain}".*?</section>', text, re.DOTALL)
    assert found is not None
    return found.group(0)


def parts(card_text: str) -> tuple[str, str]:
    """The two parts of a feedline card: the step checks of the chain search, then the global search's states.

    Returns:
        The two parts' HTML.
    """
    steps, states = card_text.split("<h3>States of the global SAT search</h3>", 1)
    return steps, states


def rows(text: str) -> list[tuple[str, str, str]]:
    """The edge, the coupler tried and the options of every row of the step tables in some HTML.

    Returns:
        The rows, in order.
    """
    return re.findall(r'<tr title="step \d+"><td>(f\d+)</td><td>(.*?)</td><td><code>([\d,]+)</code>', text)


def embedded(text: str, name: str) -> dict[str, Any]:
    """The JSON object a page embeds in one of its script elements.

    Returns:
        The object.
    """
    found = re.search(rf'<script id="{name}" type="application/json">(.*?)</script>', text, re.DOTALL)
    assert found is not None
    return json.loads(found.group(1))


def test_a_feedline_card_lists_its_sat_states_or_else_its_unsat_states() -> None:
    """The global search's part of a card lists a feedline's SAT states first, or its UNSAT states when none is SAT."""
    text = pages.local_sat_page(records_of(sat_records()))

    sat = parts(card(text, 0))[1]
    assert "feedline 0" in card(text, 0)
    assert "f0 f1 f2" in card(text, 0)
    listed, hidden = sat.split("<details>", 1)
    assert 'id="chain-0-state-1"' in listed
    assert 'id="chain-0-state-2"' in listed
    assert "c0: 0→1" in listed
    assert "c0: 0→2" in listed
    # The UNSAT states, the commit among them, wait behind a summary.
    assert "2 UNSAT states" in hidden
    assert 'id="chain-0-state-0"' in hidden
    assert "c1: 0→2" in hidden

    unsat = parts(card(text, 1))[1]
    assert 'id="chain-1-state-0"' in unsat
    assert "c2: 0→1" in unsat
    assert "UNSAT states</summary>" not in unsat


def test_the_dashboards_count_the_feedlines_and_offer_a_filter() -> None:
    """The tiles count the locally SAT feedlines and the step checks; the bar filters by the badge and searches."""
    text = pages.local_sat_page(records_of(sat_records()))

    assert "1 / 2" in text
    assert "feedlines locally SAT / all" in text
    assert '<b class="">7</b><span>step checks</span>' in text
    assert '<b class="">43 %</b><span>of the step checks SAT</span>' in text
    assert '<b class="">2</b><span>steps refused</span>' in text
    assert '<b class="SAT">SAT</b><span>global SAT search</span>' in text
    for key, count in (("all", 2), ("SAT", 1), ("UNSAT", 1)):
        assert f'data-filter="{key}">' in text
        assert f"({count})</button>" in text
    assert 'id="find"' in text
    assert "position: sticky" in text
    assert "<script src" not in text
    assert "http" not in text


def test_a_page_without_a_session_shows_its_links_off() -> None:
    """A page that stands alone has no graph or mover address and says how to start a session."""
    for text in (pages.local_sat_page(records_of(sat_records())), pages.global_sat_page(records_of(sat_records()))):
        assert "graph ↗" in text
        assert 'href="graph?' not in text
        assert 'href="mover?' not in text
        assert 'class="off"' in text
        assert "start `mqt-scpd debug --serve` (or SAT Graph Debug in ChipBuilder) to open graphs" in text


def test_a_live_page_links_the_graph_and_the_mover_of_every_state() -> None:
    """A page a session serves links each state's graph by its check and its mover, relative to the server."""
    live = pages.Live(chains=[], verdict={"verdict": "UNSAT", "lost": ["w9"]})
    text = pages.local_sat_page(records_of(sat_records()), live)

    assert 'href="graph?chain=0&amp;options=0,1,0,0&amp;check=standing"' in text
    assert 'href="graph?chain=0&amp;options=0,2,0,0&amp;check=alone"' in text
    # A state neither check found SAT opens the standing check.
    assert 'href="graph?chain=1&amp;options=0,1,0&amp;check=standing"' in text
    assert 'href="mover?chain=0&amp;options=0,2,0,0"' in text
    assert 'href="graph?step=0&amp;chain=0&amp;options=0,1,0,0"' in text
    assert 'href="mover?chain=0&amp;options=0,1,0,0"' in text
    assert 'class="off"' not in text
    assert "the chip now (whole-chip check)" in text

    glob = pages.global_sat_page(records_of(sat_records()), live)
    assert 'href="graph?combination=1"' in glob
    assert 'href="sat#chain-0-state-2"' in glob


def test_a_combination_links_each_state_to_its_row_on_the_feedline_dashboard() -> None:
    """Each state number of a combination names an anchor the feedline dashboard has, by the state's options."""
    records = records_of(sat_records())
    local = pages.local_sat_page(records)
    glob = pages.global_sat_page(records)

    assert "SAT after 2 combinations: chain 0 state 2" in glob
    assert '<p class="banner SAT">' in glob
    assert "w5" in glob  # what the commit lost
    links = re.findall(r'href="local-sat\.html#(chain-\d+-state-\d+)"', glob)
    assert links == ["chain-0-state-1", "chain-0-state-2"]
    for anchor in links:
        assert f'id="{anchor}"' in local
    # The row of state 2 holds the options of the kept state 2.
    row = re.search(r'<tr id="chain-0-state-2" title="options ([\d,]+)">', local)
    assert row is not None
    assert row.group(1) == "0,2,0,0"
    assert 'id="combination-1" data-verdict="UNSAT"' in glob
    assert 'id="combination-2" data-verdict="SAT"' in glob


def test_a_chip_sat_after_the_commit_shows_the_step_checks() -> None:
    """With no search, both dashboards say the chip is SAT after the commit; the step checks give the badges."""
    records = records_of(sat_records(searched=False))
    local = pages.local_sat_page(records)
    glob = pages.global_sat_page(records)

    for text in (local, glob):
        assert "the chip is SAT after the commit" in text
    feedline = card(local, 0)
    assert '<span class="badge SAT">SAT</span>' in feedline
    assert '<span class="badge small none">global search: not needed</span>' in feedline
    assert "the global SAT search collected no state of this feedline" in feedline
    assert "not searched" not in local
    assert "Combinations" not in glob


def test_a_feedline_the_search_did_not_collect_is_sat_by_its_step_checks_and_links_them() -> None:
    """A feedline the global search never collected takes its badge from its step checks and links each one."""
    by_kind = sat_records()
    by_kind["chain-state"] = [s for s in by_kind["chain-state"] if s["chain"] != 0]
    live = pages.Live(
        chains=[
            {"chain": 0, "options": [0, 1, 1, 0], "edges": EDGES[0]},
            {"chain": 1, "options": [0, 0], "edges": ["f3"]},
        ],
        verdict={"verdict": "SAT", "lost": []},
    )
    local = pages.local_sat_page(records_of(by_kind), live)

    feedline = card(local, 0)
    assert 'data-verdict="SAT"' in feedline
    assert '<span class="badge SAT">SAT</span>' in feedline
    assert "global search: not needed" in feedline
    assert "the global SAT search collected no state of this feedline" in feedline
    now = re.search(r'<p class="facts">now: .*?</p>', feedline)
    assert now is not None
    for href in (
        "graph?chain=0&amp;options=0,1,1,0&amp;check=standing",
        "graph?chain=0&amp;options=0,1,1,0&amp;check=alone",
    ):
        assert f'href="{href}"' in now.group(0)
    assert 'href="graph?step=2&amp;chain=0&amp;options=0,1,1,0"' in feedline
    assert 'href="graph?step=0&amp;chain=0&amp;options=0,1,0,0"' in feedline
    assert 'href="mover?chain=0&amp;options=0,1,1,0"' in feedline
    assert 'class="off"' not in feedline
    # The step options of feedline 1 do not fit the two waypoints the session lists, so its step checks have no link.
    assert "graph?step=" not in card(local, 1)


def test_the_server_answers_both_dashboards_live() -> None:
    """`/sat` and `/global` are the two dashboards from the session's own records, with live links."""
    server, _ = fake_server()

    local = get(server, "/sat")
    assert local.status == 200
    assert 'href="mover?chain=0&amp;options=0,1,0,0"' in local.body
    # Feedline 0 stands on state 2, off the commit.
    assert "now: stands on 0,2,0,0 (moved off the commit)" in card(local.body, 0)
    glob = get(server, "/global")
    assert 'href="graph?combination=2"' in glob.body


def test_a_chain_graph_names_its_check_and_marks_a_mismatch_with_the_record() -> None:
    """`/graph?chain=` answers the graph of the asked check; its banner compares the verdict with the record."""
    server, session = fake_server()
    before = session.chains()

    matched = get(server, "/graph", {"chain": "0", "options": "0,2,0,0", "check": "alone"})
    assert matched.status == 200
    assert embedded(matched.body, "graph-data")["summary"] == ["alone 0 [0, 2, 0, 0]"]
    assert 'class="banner SAT"' in matched.body
    assert "the alone check: SAT" in matched.body
    assert "matches the search&#x27;s record (SAT)" in matched.body

    # The record says the standing check found state 1 SAT; the session now finds it UNSAT.
    mismatch = get(server, "/graph", {"chain": "0", "options": "0,1,0,0", "check": "standing"})
    assert 'class="banner mismatch"' in mismatch.body
    assert "MISMATCH: the search recorded SAT" in mismatch.body

    # The search never ran the alone check on a state the standing check found SAT.
    unknown = get(server, "/graph", {"chain": "0", "options": "0,1,0,0", "check": "alone"})
    assert "the search did not run the alone check on this state" in unknown.body
    assert session.chains() == before


def test_a_chain_graph_says_when_other_feedlines_stand_off_the_commit() -> None:
    """The search judged a state against the commit; the banner names the feedlines that stand elsewhere now."""
    server, _ = fake_server()
    post(server, "/options", {"moves": [{"coupler": 2, "option": 1}]})

    answer = get(server, "/graph", {"chain": "0", "options": "0,0,0,0"})
    assert "feedline 1 stands off the commit, which the search judged the state against" in answer.body
    # The asked feedline itself is not named.
    assert "feedlines 0, 1" not in answer.body


def test_a_combination_graph_puts_every_coupler_back() -> None:
    """`/graph?combination=` checks the chip on the combination's states and restores the options exactly."""
    server, session = fake_server()
    post(server, "/options", {"moves": [{"coupler": 2, "option": 2}]})
    before = session.chains()

    sat = get(server, "/graph", {"combination": "2"})
    assert sat.status == 200
    # Feedline 0 on its state 2, feedline 1 on the commit.
    assert embedded(sat.body, "graph-data")["summary"] == ["chip {0: [0, 2, 0, 0], 1: [0, 0, 0]}"]
    assert "matches the search&#x27;s record (SAT)" in sat.body
    assert session.chains() == before

    session.chip_sat.add(((0, 1, 0, 0), (0, 0, 0)))
    differs = get(server, "/graph", {"combination": "1"})
    assert 'class="banner mismatch"' in differs.body
    assert "MISMATCH: the search recorded UNSAT" in differs.body
    assert session.chains() == before

    assert get(server, "/graph", {"combination": "9"}).status == 400


def test_the_mover_keeps_the_options_and_opens_the_coupler_tab() -> None:
    """`/mover` puts a feedline on the options, checks it, and sends the browser to its first edge."""
    server, session = fake_server()

    answer = get(server, "/mover", {"chain": "0", "options": "0,1,0,0"})
    assert answer.status == 303
    assert answer.location == "./#feedline=f0&tab=couplers"
    assert session.options[0] == [0, 1, 0, 0]
    state = json.loads(get(server, "/state.json").body)
    assert state["undo"] == 1
    assert state["chains"][0]["options"] == [0, 1, 0, 0]
    assert state["chains"][0]["committed"] is False
    assert state["global"] == {"verdict": "UNSAT", "lost": ["w9"], "over": []}
    assert [local["chain"] for local in state["local"]] == [0]
    assert state["local"][0]["standing"]["verdict"] == "UNSAT"
    assert server.last["verdicts"]["global"]["verdict"] == "UNSAT"


def test_undo_and_reset_move_the_couplers_back() -> None:
    """`/undo` takes the last move back; `/reset` puts every feedline on the commit and can be taken back too."""
    server, session = fake_server()
    started = copy.deepcopy(session.options)

    moved = json.loads(
        post(server, "/options", {"moves": [{"coupler": 0, "option": 1}, {"coupler": 2, "option": 1}]}).body
    )
    assert [local["chain"] for local in moved["verdicts"]["local"]] == [0, 1]
    assert {e["edge"] for e in moved["edges"]} == {"f0", "f3"}
    back = json.loads(post(server, "/undo").body)
    assert back["action"] == "the last move taken back"
    assert session.options == started
    refused = post(server, "/undo")
    assert refused.status == 400
    assert "no move to take back" in json.loads(refused.body)["error"]

    reset = json.loads(post(server, "/reset").body)
    assert session.options == {0: [0, 0, 0, 0], 1: [0, 0, 0]}
    assert reset["verdicts"]["global"]["verdict"] == "UNSAT"
    assert json.loads(get(server, "/state.json").body)["chains"][0]["committed"] is True
    post(server, "/undo")
    assert session.options == started


def test_a_move_that_fails_moves_nothing() -> None:
    """A move with an option that does not exist answers 400 and leaves every coupler where it stood."""
    server, session = fake_server()
    started = copy.deepcopy(session.options)

    answer = post(server, "/options", {"moves": [{"coupler": 2, "option": 1}, {"coupler": 0, "option": 9}]})
    assert answer.status == 400
    assert "no option 9" in json.loads(answer.body)["error"]
    assert session.options == started
    assert json.loads(get(server, "/state.json").body)["undo"] == 0
    assert post(server, "/options", {"moves": []}).status == 400
    assert get(server, "/mover", {"chain": "0", "options": "0,1"}).status == 400
    assert session.options == started


def test_check_all_checks_every_feedline_locally() -> None:
    """`/check-all` answers the local checks of every feedline; the state lists them until the next move."""
    server, _ = fake_server()

    answer = json.loads(post(server, "/check-all").body)
    assert [local["chain"] for local in answer["verdicts"]["local"]] == [0, 1]
    assert answer["verdicts"]["local"][0]["alone"]["verdict"] == "SAT"
    assert "graph" not in answer["verdicts"]["local"][0]["standing"]
    assert len(json.loads(get(server, "/state.json").body)["local"]) == 2
    post(server, "/options", {"moves": [{"coupler": 2, "option": 1}]})
    assert [local["chain"] for local in json.loads(get(server, "/state.json").body)["local"]] == [1]


def test_the_coupler_page_carries_the_verdicts_and_the_session_tools() -> None:
    """The live coupler page knows it is live and how many moves can be taken back; the tools start hidden."""
    server, _ = fake_server()
    post(server, "/option", {"coupler": 0, "option": 1})

    text = get(server, "/").body
    session = embedded(text, "session-data")
    assert session["live"] is True
    assert session["undo"] == 1
    assert session["last"]["verdicts"]["local"][0]["chain"] == 0
    assert '<div class="session-tools" id="session-tools" hidden>' in text


def test_another_module_adds_routes_to_the_server() -> None:
    """A route added for `/graph` answers the queries the built-in routes leave; the built-in ones still answer."""
    server, _ = fake_server()

    def bench(request: Request) -> Answer | None:
        if "bench" not in request.query:
            return None
        return Answer.page(f"bench {request.query['bench']} case {request.query['case']}")

    server.add_route("GET", "/graph", bench)
    server.add_route("GET", "/bench/local", lambda _: Answer.page("the local bench"))

    assert get(server, "/graph", {"bench": "local", "case": "3"}).body == "bench local case 3"
    assert get(server, "/bench/local").body == "the local bench"
    assert "graph-data" in get(server, "/graph", {"chain": "1", "options": "0,0,0"}).body
    assert get(server, "/graph").status == 400
    assert get(server, "/nothing").status == 404


def test_the_handler_answers_redirects_and_queries_over_http() -> None:
    """Over HTTP the mover answers 303 with its address, and a query reaches the route."""
    server, _ = fake_server()
    httpd = HTTPServer(("127.0.0.1", 0), server.handler())
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    connection = http.client.HTTPConnection("127.0.0.1", httpd.server_address[1], timeout=10)
    try:
        connection.request("GET", "/mover?chain=1&options=0,1,0")
        reply = connection.getresponse()
        reply.read()
        assert reply.status == 303
        assert reply.getheader("Location") == "./#feedline=f3&tab=couplers"
        connection.request("POST", "/undo", body=b"")
        reply = connection.getresponse()
        assert reply.status == 200
        assert json.loads(reply.read())["action"] == "the last move taken back"
    finally:
        connection.close()
        httpd.shutdown()
        httpd.server_close()


def test_a_recorder_keeps_the_sat_records_alone() -> None:
    """The recorder keeps the kinds the dashboards read and drops the rest."""
    kept = Recorder()
    kept("chain-state", '{"chain": 0}')
    kept("search", '{"wire": "1"}')

    assert kept.read("chain-state") == [{"chain": 0}]
    assert kept.read("search") == []


def test_the_sat_records_are_turned_on_for_a_session() -> None:
    """A session's configuration asks for the SAT records whatever its `[debug]` section said."""
    config = ConfigT()
    serve.want_sat_records(config)
    assert config.debug is not None
    assert config.debug.localSat
    assert config.debug.globalSat

    config.debug = DebugParamsT()
    config.debug.feedlineInsertion = True
    serve.want_sat_records(config)
    assert config.debug.feedlineInsertion
    assert config.debug.globalSat


def test_a_real_session_serves_the_dashboards_and_restores_a_checked_chain(tmp_path: Path) -> None:
    """On the 4-qubit chip the server builds both dashboards; a graph and a move leave the chain as asked.

    The step check the session rebuilds on the options of the step check of the committed chain's last edge
    gives the recorded verdict.
    """
    run = tmp_path / "run"
    assert main(["plan", "-c", str(BENCHMARKS / "4q" / "config.toml"), "-o", str(run), "--stop-after", "detail"]) == 0
    directory = RunDirectory(run)
    config = directory.load()
    serve.want_sat_records(config)
    records = Recorder()
    lines: list[str] = []
    session = directory.coupler_session(config, debug=records)
    assert records.read("step-check")
    assert records.read("global-sat")
    server = CouplerServer(session, lines, records)

    local = get(server, "/sat")
    assert local.status == 200
    assert 'id="chain-0"' in local.body
    assert get(server, "/global").status == 200
    state = json.loads(get(server, "/state.json").body)
    chain = state["chains"][0]
    assert state["global"]["verdict"] in {"SAT", "UNSAT"}

    before = session.chains()
    options = ",".join(str(o) for o in chain["options"])
    answer = get(server, "/graph", {"chain": "0", "options": options, "check": "alone"})
    assert answer.status == 200
    assert "the alone check:" in answer.body
    assert session.chains() == before

    last = len(chain["edges"]) - 1
    recorded = next(s for s in records.read("step-check") if s["step"] == last and s["options"] == chain["options"])
    rebuilt = json.loads(session.step_check(0, last, chain["options"]))
    assert rebuilt["verdict"] == recorded["verdict"]
    assert rebuilt["lost"] == recorded["lost"]
    assert "wires" in rebuilt["graph"]
    step_page = get(server, "/graph", {"step": str(last), "chain": "0", "options": options})
    assert step_page.status == 200
    assert "rebuilt by the session" in step_page.body
    assert "MISMATCH" not in step_page.body
    assert session.chains() == before

    couplers = json.loads(session.couplers())["couplers"]
    coupler = next(c for c in couplers if len(c["options"]) > 1)
    option = next(o["index"] for o in coupler["options"] if o["index"] != coupler["chosen"])
    moved = json.loads(post(server, "/option", {"coupler": coupler["index"], "option": option}).body)
    assert moved["verdicts"]["local"][0]["chain"] == coupler["chain"]
    assert moved["verdicts"]["global"]["verdict"] in {"SAT", "UNSAT"}
    post(server, "/undo")
    assert json.loads(session.chains()) == json.loads(before)


def test_a_feedline_is_sat_when_a_step_check_of_its_last_edge_is_sat() -> None:
    """The badge comes from the step checks of the last edge; a feedline with no step check has none."""
    sat = [step(1, 0, [0, 0, 0], "UNSAT"), step(1, 1, [0, 1, 0], "UNSAT"), step(1, 1, [0, 2, 0], "SAT")]
    assert pages.local_verdict(sat, 2) == "SAT"
    # A SAT step of an earlier edge does not make the feedline SAT.
    assert pages.local_verdict([step(1, 0, [0, 0, 0], "SAT"), step(1, 1, [0, 0, 0], "UNSAT")], 2) == "UNSAT"
    assert pages.local_verdict([step(1, 0, [0, 0, 0], "SAT")], 2) == "UNSAT"
    # Without the number of edges, the last step seen counts.
    assert pages.local_verdict([step(1, 0, [0, 0, 0], "SAT")], None) == "SAT"
    assert pages.local_verdict([], 2) == "none"

    text = pages.local_sat_page(records_of(sat_records()))
    assert '<span class="badge SAT">SAT</span>' in card(text, 0)
    assert '<span class="badge UNSAT">UNSAT</span>' in card(text, 1)

    by_kind = sat_records()
    by_kind["step-check"] = [s for s in by_kind["step-check"] if s["chain"] != 1]
    unchecked = card(pages.local_sat_page(records_of(by_kind)), 1)
    assert 'data-verdict="none"' in unchecked
    assert '<span class="badge none">no local check</span>' in unchecked
    assert "the chain search made no step check" in unchecked


def test_a_card_groups_the_step_checks_by_edge_and_shows_unsat_options_where_an_edge_has_no_sat_one() -> None:
    """Per edge the card shows its SAT options, or its UNSAT options when it has none; the rest waits below."""
    feedline = card(pages.local_sat_page(records_of(sat_records())), 0)
    steps = parts(feedline)[0]
    shown, rest = steps.split("<details>", 1)

    assert rows(shown) == [
        ("f0", "c0: 1", "0,1,0,0"),
        ("f1", "c1: 2", "0,1,2,0"),
        ("f1", "c1: 1", "0,1,1,0"),
        ("f2", "launcher (end)", "0,1,1,0"),
    ]
    assert "refused" in shown
    assert "1 more step checks: the UNSAT options of edges with a SAT option" in rest
    assert rows(rest) == [("f0", "c0: 2", "0,2,0,0")]
    assert "local checks of the coupler insertion: 5 options checked · 2 SAT · 3 UNSAT · 1 refused · 0.05 s" in feedline
    # The links are off on a page that stands alone.
    assert "graph?step=" not in feedline


def test_a_card_names_the_global_searches_outcome_in_a_second_badge() -> None:
    """The smaller badge counts a feedline's SAT states, says UNSAT, or says the search did not need it."""
    text = pages.local_sat_page(records_of(sat_records()))
    assert '<span class="badge small SAT">global search: SAT states 2</span>' in card(text, 0)
    assert '<span class="badge small UNSAT">global search: UNSAT</span>' in card(text, 1)

    by_kind = sat_records()
    by_kind["chain-state"] = [s for s in by_kind["chain-state"] if s["chain"] != 1]
    assert "global search: not needed" in card(pages.local_sat_page(records_of(by_kind)), 1)
    by_kind.pop("global-sat")
    assert "global search: not recorded" in card(pages.local_sat_page(records_of(by_kind)), 1)


def test_a_step_graph_rebuilds_the_step_check_and_marks_a_mismatch_with_the_record() -> None:
    """`/graph?step=` answers the step check the session rebuilt; its banner compares it with the record."""
    server, session = fake_server()
    before = session.chains()

    matched = get(server, "/graph", {"step": "2", "chain": "0", "options": "0,1,1,0"})
    assert matched.status == 200
    assert embedded(matched.body, "graph-data")["summary"] == ["step 0 2 [0, 1, 1, 0]"]
    assert 'class="banner SAT"' in matched.body
    assert "the step check of feedline 0 at edge f2, step 2, rebuilt by the session" in matched.body
    assert "matches the chain search&#x27;s record (SAT)" in matched.body

    session.step_sat.discard((0, 0, (0, 1, 0, 0)))
    mismatch = get(server, "/graph", {"step": "0", "chain": "0", "options": "0,1,0,0"})
    assert 'class="banner mismatch"' in mismatch.body
    assert "MISMATCH: the chain search recorded SAT" in mismatch.body
    assert "lost: w5 · closed: w5" in mismatch.body

    unknown = get(server, "/graph", {"step": "0", "chain": "1", "options": "0,2,0"})
    assert "no step-check record of this step on these options" in unknown.body
    assert session.chains() == before

    assert get(server, "/graph", {"step": "5", "chain": "0", "options": "0,1,1,0"}).status == 400


def test_a_step_graph_compares_with_the_first_of_several_records() -> None:
    """Several records of one step on the same options: the banner compares with the first and says so."""
    by_kind = sat_records()
    by_kind["step-check"].append(step(0, 2, [0, 1, 1, 0], "UNSAT"))
    server, _ = fake_server(by_kind)

    answer = get(server, "/graph", {"step": "2", "chain": "0", "options": "0,1,1,0"})
    assert "matches the chain search&#x27;s record (SAT) (the first of 2 records)" in answer.body
