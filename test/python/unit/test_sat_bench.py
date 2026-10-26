# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Tests of the SAT benchmarks: their tables, their pages, ``mqt-scpd sat-bench`` and the bench routes of a session."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import TYPE_CHECKING

import pytest

from mqt.scpd.cli import build_parser, command_sat_bench, main
from mqt.scpd.debug import bench
from mqt.scpd.debug.serve import Answer, CouplerServer, Request

if TYPE_CHECKING:
    from collections.abc import Callable
    from typing import Any

BENCHMARKS = Path(__file__).resolve().parents[3] / "benchmarks"

#: The coupler of every waypoint of the one chain of the fake chip.
WAYPOINTS = [None, 0, 1, None]

#: The ways of the fake chip's wires as the feedline routing lays them.
WAYS = {"w1": [[0, 0], [4, 0]], "w2": [[0, 2], [4, 2]], "w3": [[0, 4], [4, 4]]}


class FakeSession:
    """A coupler session of one chain with two couplers of three options each, without the Final stage.

    The capacity graph names the options it was drawn on. The fake slot check of :func:`slot_check`
    loses ``w1`` when coupler 1 stands on option 2.
    """

    def __init__(self) -> None:
        """Stand on the committed options, 0 and 0."""
        self.options = [0, 0, 0, 0]
        self.moves: list[tuple[int, int]] = []
        self.compared = 0
        self.routed = 0

    def couplers(self) -> str:
        """The couplers as JSON.

        Returns:
            The JSON.
        """
        listed = [
            {"index": c, "chain": 0, "at": at, "chosen": self.options[at], "options": [{"index": o} for o in range(3)]}
            for at, c in enumerate(WAYPOINTS)
            if c is not None
        ]
        return json.dumps({"couplers": listed})

    def set_option(self, coupler: int, option: int) -> str:
        """Put a coupler on another option.

        Returns:
            What became of its edges, as JSON.

        Raises:
            ValueError: When the coupler or the option does not exist.
        """
        if coupler not in WAYPOINTS or option not in {0, 1, 2}:
            msg = f"coupler {coupler} has no option {option}"
            raise ValueError(msg)
        self.options[WAYPOINTS.index(coupler)] = option
        self.moves.append((coupler, option))
        return json.dumps({"edges": [{"edge": "f0"}]})

    def graph(self) -> str:
        """The capacity graph of the chip as it stands, with three wires and a terminal edge.

        Returns:
            The JSON.
        """
        wires = [{"name": name, "terminal": False, "path": [[9, 9], [9, 8]]} for name in WAYS]
        wires.append({"name": "f0", "terminal": True, "path": [[1, 1], [2, 1]]})
        return json.dumps({
            "width": 1,
            "height": 1,
            "raster": [1],
            "nodes": [],
            "edges": [],
            "wires": wires,
            "verdict": "SAT",
            "options": list(self.options),
        })

    def chains(self) -> str:
        """The chain as it stands.

        Returns:
            The JSON.
        """
        return json.dumps({
            "chains": [{"chain": 0, "couplers": WAYPOINTS, "options": list(self.options), "edges": ["f0", "f1", "f2"]}]
        })

    def local_check(self, chain: int, options: list[int]) -> str:
        """Not used by the benchmarks.

        Raises:
            NotImplementedError: Always.
        """
        raise NotImplementedError

    def step_check(self, chain: int, step: int, options: list[int]) -> str:
        """Not used by the benchmarks.

        Raises:
            NotImplementedError: Always.
        """
        raise NotImplementedError

    def insertion_check(self) -> str:
        """The whole-chip check: UNSAT when coupler 1 stands on option 2.

        Returns:
            The JSON.
        """
        lost = ["w1"] if self.options[2] == 2 else []
        verdict = "UNSAT" if lost else "SAT"
        graph = {**json.loads(self.graph()), "verdict": verdict}
        return json.dumps({"verdict": verdict, "lost": lost, "over": [], "graph": graph})

    def compare_locally(self, runs: int, span: int) -> str:
        """The comparison of :func:`compared`.

        Returns:
            The JSON.
        """
        self.compared += 1
        return json.dumps(compared(runs, span))

    def route_feedlines(self) -> str:
        """The routing of the chip as it stands: ``w2`` takes another way when coupler 0 stands on option 1.

        Returns:
            The JSON.
        """
        self.routed += 1
        return json.dumps(routing(self.options))


def routing(options: list[int]) -> dict[str, Any]:
    """What ``route_feedlines`` answers on options: ``w2`` moves and fails when coupler 0 stands on option 1.

    Returns:
        The routing.
    """
    moved = options[1] == 1
    ways = {**WAYS, "w2": [[0, 2], [2, 3], [4, 2]]} if moved else WAYS
    wires = [{"name": name, "kind": "plain", "drawn": True, "verdict": 0, "path": path} for name, path in ways.items()]
    return {"bad": int(moved), "unrouted": [], "open": ["w2"] if moved else [], "crossing": [], "wires": wires}


def feedline(*, sat: bool, clean: bool) -> dict[str, Any]:
    """One feedline of a run, as ``compare_locally`` judges it.

    Returns:
        The feedline.
    """
    return {
        "chain": 0,
        "window": ["w1", "w2"],
        "sat": sat,
        "lost": [] if sat else ["w1"],
        "wholeSat": sat,
        "wholeLost": [],
        "clean": clean,
        "failing": "" if clean else "w2",
        "blocked": "",
        "rounds": [{"round": 0}],
        "reused": False,
        "seconds": 0.25,
    }


def compared(runs: int = 8, span: int = 3) -> dict[str, Any]:
    """What ``compare_locally`` answers: one run of each mismatch class and one that agrees.

    The UNSAT-but-clean run comes first, so that a page has to sort the mismatches.

    Returns:
        The comparison.
    """
    made = [
        ("c0-w0-r0", [{"coupler": 0, "option": 0}, {"coupler": 1, "option": 2}], False, True, [0, 0, 2, 0]),
        ("c0-w0-r1", [{"coupler": 0, "option": 1}, {"coupler": 1, "option": 0}], True, False, [0, 1, 0, 0]),
        ("c0-w0-r2", [{"coupler": 0, "option": 2}, {"coupler": 1, "option": 1}], True, True, [0, 2, 1, 0]),
    ]
    return {
        "segments": 1,
        "runsPerSegment": runs,
        "span": span,
        "classes": {"satClean": 1, "satFailing": 1, "unsatClean": 1, "unsatFailing": 0},
        "unjudged": 0,
        "slotSeconds": 0.5,
        "rerouteSeconds": 1.5,
        "seconds": 2.0,
        "begun": [{"chain": 0, "sat": True, "clean": True, "failing": ""}],
        "runs": [
            {
                "id": name,
                "chain": 0,
                "lo": 0,
                "hi": 3,
                "segment": "chain 0 waypoints 0..3",
                "options": "launcher 0 0",
                "moves": moves,
                "cost": 1,
                "slot": {"verdict": "SAT" if sat else "UNSAT", "lost": [], "crowded": [], "over": []},
                "feedlines": [feedline(sat=sat, clean=clean)],
                "disagree": int(sat != clean),
                "seconds": {"slot": 0.1},
                "routed": routing(options),
                "pads": [],
            }
            for name, moves, sat, clean, options in made
        ],
    }


def slot_check_of(seen: list[list[str]]) -> Callable[[str], str]:
    """A fake slot check that keeps the wires of every graph it judges.

    Returns:
        The check: it loses ``w1`` when the graph was drawn with coupler 1 on option 2.
    """

    def check(text: str) -> str:
        graph = json.loads(text)
        seen.append([w["name"] for w in graph["wires"]])
        lost = ["w1"] if graph["options"][2] == 2 else []
        judged = {**graph, "verdict": "UNSAT" if lost else "SAT"}
        return json.dumps({"verdict": judged["verdict"], "lost": lost, "over": [], "graph": judged})

    return check


def local_table() -> dict[str, Any]:
    """The local table of the fake session.

    Returns:
        The table.
    """
    return {**bench.local_cases(FakeSession()), "chip": "fake", "inputs": "abc"}


def served(directory: Path) -> tuple[CouplerServer, FakeSession, list[list[str]]]:
    """A coupler server on a fake session with the bench routes of ``directory``.

    Returns:
        The server, its session and the wires of every graph the slot check judged.
    """
    session = FakeSession()
    server = CouplerServer(session, [])
    seen: list[list[str]] = []
    bench.add_routes(server, directory, slot_check=slot_check_of(seen))
    return server, session, seen


def get(server: CouplerServer, path: str, query: dict[str, str] | None = None) -> Answer:
    """Ask the server a GET request.

    Returns:
        The answer.
    """
    return server.answer(Request("GET", path, query or {}))


def test_the_agreement_is_the_share_of_the_agreeing_cases() -> None:
    """The agreement counts SAT and clean and UNSAT and failing over every case, and is whole without a case."""
    assert bench.agreement({"satClean": 3, "satFailing": 1, "unsatClean": 0, "unsatFailing": 1}) == pytest.approx(0.8)
    assert bench.agreement(dict.fromkeys(bench.CLASSES, 0)) == pytest.approx(1.0)
    assert bench.class_of(sat=True, clean=False) == "satFailing"
    assert bench.class_of(sat=False, clean=True) == "unsatClean"


def test_the_local_table_keeps_the_classes_and_every_way_of_every_run() -> None:
    """Every feedline of a run carries its class, and the ways of each run's routing can be read back."""
    table = local_table()
    assert table["cases"] == 3
    assert table["classes"]["satFailing"] == 1
    assert [r["feedlines"][0]["class"] for r in table["runs"]] == ["unsatClean", "satFailing", "satClean"]
    assert table["committed"] == [{"coupler": 0, "option": 0}, {"coupler": 1, "option": 0}]
    assert table["feedlines"] == [{"chain": 0, "edges": ["f0", "f1", "f2"]}]
    for run, options in zip(table["runs"], ([0, 0, 2, 0], [0, 1, 0, 0], [0, 2, 1, 0]), strict=True):
        expected = {w["name"]: w["path"] for w in routing(options)["wires"]}
        assert bench.ways_of(table, run["routed"]) == expected
    assert table["runs"][1]["routed"]["open"] == ["w2"]


def test_the_local_page_puts_the_mismatches_first_the_optimistic_ones_on_top() -> None:
    """The mismatch table lists SAT but failing before UNSAT but clean, and leaves the agreeing cases out."""
    text = bench.page("local", local_table(), live=True)
    mismatches = text[text.index("<h2>Mismatches") : text.index("<h2>Agreement per feedline")]
    assert mismatches.index('<tr class="satFailing">') < mismatches.index('<tr class="unsatClean">')
    assert '<tr class="satClean">' not in mismatches
    assert "c0: 0→1" in mismatches
    assert "Mismatches (2)" in text
    every = text[text.index("<h2>Every case") :]
    assert every.count('class="item ') == 3


def test_a_static_page_has_its_links_off_and_says_how_to_start_the_session() -> None:
    """A page that stands alone has no graph or routed address and names the command that serves it."""
    for kind, table in (("local", local_table()), ("global", global_table())):
        text = bench.page(kind, table, live=False)
        assert "../graph?" not in text
        assert "../routed?" not in text
        assert 'class="off"' in text
        assert "mqt-scpd debug --serve" in text
        assert bench.html_name("local") in text


def test_a_live_page_links_the_graph_and_the_routed_ways_of_every_case() -> None:
    """A live page links each case's graph and routed ways relative to the session's root."""
    local = bench.page("local", local_table(), live=True)
    assert 'href="../graph?bench=local&amp;case=c0-w0-r1&amp;chain=0"' in local
    assert 'href="../routed?bench=local&amp;case=c0-w0-r1&amp;chain=0"' in local
    assert 'href="../sat"' in local
    glob = bench.page("global", global_table(), live=True)
    assert 'href="../graph?bench=global&amp;case=1"' in glob
    assert 'href="../routed?bench=global&amp;case=1"' in glob
    assert 'class="off"' not in local + glob


def global_table() -> dict[str, Any]:
    """The global table of the fake session, routed without a child.

    Returns:
        The table.
    """
    session = FakeSession()
    local = bench.local_cases(session)
    return {**bench.global_cases(session, local, 2, route=lambda s: json.loads(s.route_feedlines())), "chip": "fake"}


def test_the_global_cases_start_on_the_commit_and_end_there() -> None:
    """The first case moves nothing, every case has its class, and the chip ends on the committed options."""
    session = FakeSession()
    local = bench.local_cases(session)
    table = bench.global_cases(session, local, 2, route=lambda s: json.loads(s.route_feedlines()))
    assert table["judged"][0]["moves"] == []
    assert table["judged"][0]["class"] == "satClean"
    assert table["cases"] == len(table["judged"]) == 3
    assert sum(table["classes"].values()) == 3
    assert session.options == [0, 0, 0, 0]
    page = bench.page("global", {**table, "chip": "fake"}, live=False)
    assert "the commit" in page


def test_the_global_benchmark_reuses_a_local_table_of_the_same_runs(tmp_path: Path) -> None:
    """A local table with the same runs, span and inputs is taken; the session then routes its feedlines first."""
    bench.write_table(tmp_path, "local", local_table())
    session = FakeSession()
    said: list[str] = []
    table = bench.run_bench(
        "global",
        session,
        tmp_path,
        name="fake",
        inputs="abc",
        cases=1,
        say=said.append,
        route=lambda _: routing([0, 0, 0, 0]),
    )
    assert session.compared == 0
    assert session.routed == 1
    assert any("the runs of" in line for line in said)
    assert (tmp_path / "sat-global.json").is_file()
    assert (tmp_path / "sat-global.html").is_file()
    assert said[-1] == bench.summary("global", "fake", table)


def test_the_global_benchmark_runs_the_local_cases_for_other_inputs(tmp_path: Path) -> None:
    """A local table of other inputs or another span is made again, and written beside the global one."""
    bench.write_table(tmp_path, "local", local_table())
    for inputs, span in (("other", 3), ("abc", 2)):
        session = FakeSession()
        bench.run_bench(
            "global",
            session,
            tmp_path,
            name="fake",
            inputs=inputs,
            span=span,
            cases=1,
            route=lambda _: routing([0, 0, 0, 0]),
        )
        assert session.compared == 1
        assert session.routed == 0
        written = json.loads((tmp_path / "sat-local.json").read_text(encoding="utf-8"))
        assert (written["inputs"], written["span"]) == (inputs, span)


def test_the_command_line_parses_a_sat_bench() -> None:
    """``sat-bench`` takes the kind and the run, with eight runs of three edges and twelve cases by default."""
    args = build_parser().parse_args(["sat-bench", "global", "runs/17q", "--cases", "3"])
    assert (args.kind, args.run_dir, args.runs, args.span, args.cases) == ("global", Path("runs/17q"), 8, 3, 3)
    assert args.output is None
    assert args.run is command_sat_bench
    with pytest.raises(SystemExit) as stopped:
        main(["sat-bench", "both", "runs/17q"])
    assert stopped.value.code == 2
    with pytest.raises(SystemExit):
        main(["sat-bench", "local", "runs/17q", "--runs", "0"])


def test_the_window_check_judges_the_window_alone_and_counts_crowded_wires() -> None:
    """Only the window's wires and the terminal edges reach the slot check; a window wire over an edge is lost."""
    seen: list[list[str]] = []

    def check(text: str) -> str:
        graph = json.loads(text)
        seen.append([w["name"] for w in graph["wires"]])
        edges = [{"overflow": 1, "through": ["w2", "w9"]}, {"overflow": 0, "through": ["w1"]}]
        return json.dumps({"verdict": "UNSAT", "lost": [], "over": ["g1"], "graph": {**graph, "edges": edges}})

    _, lost = bench.window_check(json.loads(FakeSession().graph()), ["w1", "w2"], check)
    assert seen == [["w1", "w2", "f0"]]
    assert lost == ["w2"]


def test_the_bench_pages_are_served_live_and_say_when_a_table_is_missing(tmp_path: Path) -> None:
    """A table that exists is served live and linked from the dashboards; a missing one says how to make it."""
    bench.write_table(tmp_path, "local", local_table())
    server, _, _ = served(tmp_path)
    answer = get(server, "/bench/local")
    assert answer.status == 200
    assert "../graph?bench=local" in answer.body
    missing = get(server, "/bench/global")
    assert missing.status == 404
    assert "mqt-scpd sat-bench global" in missing.body
    dashboard = get(server, "/sat").body
    assert 'href="bench/local"' in dashboard
    assert 'href="bench/global"' not in dashboard


def test_a_local_case_graph_runs_the_window_check_and_puts_the_couplers_back(tmp_path: Path) -> None:
    """The graph of a case judges the case's state, compares it with the table, and leaves the chip as it stood."""
    bench.write_table(tmp_path, "local", local_table())
    server, session, seen = served(tmp_path)
    before = list(session.options)
    answer = get(server, "/graph", {"bench": "local", "case": "c0-w0-r0", "chain": "0"})
    assert answer.status == 200
    assert "the window slot check now: UNSAT" in answer.body
    assert "lost: w1" in answer.body
    assert "MISMATCH" not in answer.body
    assert seen == [["w1", "w2", "f0"]]
    assert (1, 2) in session.moves
    assert session.options == before
    wrong = get(server, "/graph", {"bench": "local", "case": "c0-w0-r9", "chain": "0"})
    assert wrong.status == 400
    assert session.options == before


def test_a_global_case_graph_runs_the_whole_chip_check_and_puts_the_couplers_back(tmp_path: Path) -> None:
    """The graph of a global case is the whole-chip check of its state; the chip then stands as before."""
    bench.write_table(tmp_path, "global", global_table())
    server, session, _ = served(tmp_path)
    before = list(session.options)
    answer = get(server, "/graph", {"bench": "global", "case": "0"})
    assert answer.status == 200
    assert "the whole-chip check now: SAT" in answer.body
    assert session.options == before


def test_the_routed_page_draws_the_wires_along_the_ways_of_the_case(tmp_path: Path) -> None:
    """The routed page carries the ways the rip-up laid, also the ways the case shares with the first run."""
    bench.write_table(tmp_path, "local", local_table())
    server, session, _ = served(tmp_path)
    before = list(session.options)
    answer = get(server, "/routed", {"bench": "local", "case": "c0-w0-r1", "chain": "0"})
    assert answer.status == 200
    assert "wires as the rip-up laid them" in answer.body
    assert '"path":[[0,2],[2,3],[4,2]]' in answer.body
    assert '"path":[[0,0],[4,0]]' in answer.body
    assert '"path":[[1,1],[2,1]]' in answer.body
    assert session.options == before
    assert get(server, "/routed").status == 400


def test_a_chain_graph_without_a_bench_stays_with_the_dashboard_route(tmp_path: Path) -> None:
    """A ``/graph`` query without ``bench`` is not a case of a benchmark."""
    server, _, _ = served(tmp_path)
    assert get(server, "/graph", {"bench": "local", "case": "x", "chain": "0"}).status == 400
    assert "sat-local.json does not exist" in get(server, "/graph", {"bench": "local", "case": "x", "chain": "0"}).body


@pytest.mark.skipif(not hasattr(os, "fork"), reason="the global cases route in a forked child")
def test_sat_bench_runs_both_benchmarks_on_the_4_qubit_chip(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """On the 4-qubit chip the command writes both tables and pages; the global benchmark takes the local runs."""
    run = tmp_path / "run"
    assert main(["plan", "-c", str(BENCHMARKS / "4q" / "config.toml"), "-o", str(run), "--stop-after", "detail"]) == 0
    capsys.readouterr()
    assert main(["sat-bench", "local", str(run), "--runs", "1", "--span", "3", "--name", "4q"]) == 0
    local = json.loads((run / "sat-bench" / "sat-local.json").read_text(encoding="utf-8"))
    assert local["cases"] > 0
    assert local["chip"] == "4q"
    assert sum(local["classes"].values()) == local["cases"]
    assert "local SAT bench · 4q" in (run / "sat-bench" / "sat-local.html").read_text(encoding="utf-8")
    assert "agreement" in capsys.readouterr().out

    assert main(["sat-bench", "global", str(run), "--runs", "1", "--span", "3", "--cases", "1"]) == 0
    said = capsys.readouterr().out
    assert "global: the runs of" in said
    table = json.loads((run / "sat-bench" / "sat-global.json").read_text(encoding="utf-8"))
    assert table["cases"] == len(table["judged"]) >= 1
    assert table["judged"][0]["moves"] == []
    assert table["ways"]
