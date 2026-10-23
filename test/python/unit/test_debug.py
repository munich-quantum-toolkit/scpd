# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Tests of the debug pages: the record store, the pages built from it and the live coupler page."""

from __future__ import annotations

import http.client
import json
import re
import threading
from http.server import HTTPServer
from pathlib import Path
from typing import TYPE_CHECKING

import pytest

from mqt.scpd.cli import main
from mqt.scpd.debug import pages, serve, store
from mqt.scpd.flatbuffers.config.DebugParams import DebugParamsT
from mqt.scpd.run import RunDirectory

if TYPE_CHECKING:
    from typing import Any

BENCHMARKS = Path(__file__).resolve().parents[3] / "benchmarks"


def graph() -> dict[str, Any]:
    """A capacity graph of two chambers, one bottleneck and one wire through it.

    Returns:
        The graph as the Final stage writes it.
    """
    port = {"at": [1, 1], "heading": [1, 0], "cell": [1, 1], "port": [1.0, 1.0]}
    return {
        "width": 4,
        "height": 2,
        "cellSize": 10.0,
        "clearance": 19,
        "pitch": 20,
        "verdict": "SAT",
        "summary": ["coupler insertion: CAPACITY GRAPH SAT"],
        "wallKinds": ["border", "artwork", "coupler pad", "stub", "feedline"],
        "raster": [6, 3, 0, 2, 7, 3],
        "strokes": [],
        "nodes": [{"chamber": 0, "at": [0, 0], "cells": 3}, {"chamber": 1, "at": [3, 1], "cells": 3}],
        "edges": [
            {
                "index": 0,
                "kind": "bottleneck",
                "name": "g0",
                "chambers": [0, 1],
                "capacity": 1,
                "load": 1,
                "overflow": 0,
                "length": 19.0,
                "through": ["</script><b>0"],
                "users": None,
                "ends": ["f1", "artwork"],
                "line": [[2, 0], [2, 1]],
                "crossingNow": [],
            }
        ],
        "aside": [],
        "wires": [
            {
                "name": "</script><b>0",
                "resonator": False,
                "source": {**port, "label": "Chip.port1", "chambers": [0]},
                "target": {**port, "label": "Qb1.port1", "chambers": [1]},
                "path": [],
                "status": "routed",
                "way": [0],
            }
        ],
    }


def every_page() -> DebugParamsT:
    """A `[debug]` section with every page on.

    Returns:
        The section.
    """
    return pages.all_pages()


def records(debug: Path) -> None:
    """Write the records of a small run: one of every kind a page reads."""
    sink = store.Sink(debug)
    sink(
        "grid",
        json.dumps({
            "width": 4,
            "height": 2,
            "cellSize": 10.0,
            "clearance": 1,
            "blocked": [[0, 0, 1]],
            "ports": [],
            "wires": [],
        }),
    )
    sink(
        "search",
        json.dumps({
            "phase": "pre-feedline-routing",
            "pass": "outer",
            "round": 0,
            "forward": True,
            "wire": "</script>7",
            "kind": "normal",
            "ripped": [],
            "found": True,
            "cells": 3,
            "note": "",
            "way": [[0, 1], [2, 1]],
        }),
    )
    sink(
        "search",
        json.dumps({
            "phase": "feedline-constrained-routing",
            "pass": "feedline",
            "round": 0,
            "forward": True,
            "wire": "3",
            "kind": "relax 1",
            "ripped": ["4"],
            "found": False,
            "cells": 0,
            "note": "",
            "way": [],
        }),
    )
    sink("coupler-options", json.dumps({"couplers": couplers()}))
    sink("capacity-graph", json.dumps(graph()))
    sink(
        "step-check",
        json.dumps({
            "chain": 0,
            "step": 1,
            "edge": 1,
            "options": [0, 2],
            "verdict": "UNSAT",
            "lost": ["5"],
            "closed": ["5"],
            "refused": True,
            "seconds": 0.01,
        }),
    )
    sink("global-sat", json.dumps({"event": "start", "lost": ["5"]}))
    sink("global-sat", json.dumps({"event": "states", "chain": 0, "states": 3}))
    sink(
        "global-sat",
        json.dumps({
            "event": "combination",
            "index": 1,
            "stage": "stage 1: ",
            "states": [1, 0],
            "verdict": "SAT",
            "lost": [],
        }),
    )
    sink(
        "global-sat",
        json.dumps({
            "event": "result",
            "searched": True,
            "sat": True,
            "combinations": 1,
            "moved": "chain 0 state 1",
            "seconds": 0.5,
        }),
    )
    sink(
        "phase",
        json.dumps({
            "phase": "pre-feedline-routing",
            "wires": 12,
            "drawn": 12,
            "unrouted": 0,
            "open": 0,
            "short": 0,
            "long": 0,
            "crossing": 0,
            "loops": 0,
            "bad": 0,
        }),
    )
    sink.close()


def test_a_sink_appends_records_by_kind_and_reads_them_back_in_order(tmp_path: Path) -> None:
    """Every record goes into the compressed file of its kind, and comes back as it went in."""
    debug = tmp_path / "debug"
    sink = store.Sink(debug)
    sink("search", '{"wire": "1"}')
    sink("search", '{"wire": "2"}')
    sink("grid", "{}")
    sink.close()

    assert sorted(store.kinds(debug)) == ["grid", "search"]
    assert list(store.read(debug, "search")) == [{"wire": "1"}, {"wire": "2"}]
    assert list(store.read(debug, "grid")) == [{}]
    assert list(store.read(debug, "phase")) == []
    assert (debug / "data" / "search.jsonl.gz").is_file()


def test_a_new_sink_starts_the_store_over(tmp_path: Path) -> None:
    """A run writes its own records, never on top of an earlier run's."""
    debug = tmp_path / "debug"
    first = store.Sink(debug)
    first("grid", '{"run": 1}')
    first.close()
    second = store.Sink(debug)
    second("search", "{}")
    second.close()

    assert store.kinds(debug) == ["search"]


def test_the_index_links_every_page_the_section_asks_for(tmp_path: Path) -> None:
    """Every page on is built from its records, and the index links each one and lists the phases."""
    debug = tmp_path / "debug"
    records(debug)

    made = pages.build(debug, every_page())

    index = debug / "index.html"
    assert index in made
    text = index.read_text(encoding="utf-8")
    for key, name in pages.PAGES.items():
        page = debug / name
        assert page in made, key
        assert page.is_file()
        assert f'href="{name}"' in text
    # The phases the run came through, with their figures.
    assert "pre-feedline-routing" in text
    assert "12" in text


def test_a_dashboard_holds_the_searches_of_its_phase_alone(tmp_path: Path) -> None:
    """The table of one phase carries that phase's searches, escaped, and the grid to draw them on."""
    debug = tmp_path / "debug"
    records(debug)
    pages.build(debug, every_page())

    routing = (debug / pages.PAGES["pre_feedline_routing_dashboard"]).read_text(encoding="utf-8")
    assert "\\u003c/script>7" in routing
    assert '"wire":"3"' not in routing
    assert '"blocked":[[0,0,1]]' in routing
    constrained = (debug / pages.PAGES["feedline_constrained_routing_dashboard"]).read_text(encoding="utf-8")
    assert '"wire":"3"' in constrained
    assert '"ripped":["4"]' in constrained


def test_the_insertion_page_holds_the_graph_and_the_coupler_options(tmp_path: Path) -> None:
    """The insertion page is the capacity graph's page, with the coupler options as its session."""
    debug = tmp_path / "debug"
    records(debug)
    pages.build(debug, every_page())

    text = (debug / pages.PAGES["feedline_insertion"]).read_text(encoding="utf-8")
    assert text.startswith("<!doctype html>")
    assert "<title>Capacity graph · SAT</title>" in text
    embedded = re.search(r'<script id="graph-data" type="application/json">(.*?)</script>', text, re.DOTALL)
    assert embedded is not None
    assert json.loads(embedded.group(1)) == graph()
    session = re.search(r'<script id="session-data" type="application/json">(.*?)</script>', text, re.DOTALL)
    assert session is not None
    assert json.loads(session.group(1))["couplers"] == couplers()
    # The data, the session data, the page's script and the coupler tab's.
    assert text.count("</script>") == 4
    for view in ("chip", "graph", "both"):
        assert f'data-view="{view}"' in text


def test_the_sat_pages_list_the_checks_and_the_combinations(tmp_path: Path) -> None:
    """The local page lists every step check; the global page the search's course and its result."""
    debug = tmp_path / "debug"
    records(debug)
    pages.build(debug, every_page())

    local = (debug / pages.PAGES["local_sat"]).read_text(encoding="utf-8")
    assert "UNSAT" in local
    assert "refused" in local
    glob = (debug / pages.PAGES["global_sat"]).read_text(encoding="utf-8")
    assert "chain 0 state 1" in glob
    assert "stage 1" in glob


def test_a_page_without_records_says_so(tmp_path: Path) -> None:
    """A page whose records a run never wrote is still built, and says what is missing."""
    debug = tmp_path / "debug"
    debug.mkdir()
    debug_params = DebugParamsT()
    debug_params.localSat = True

    made = pages.build(debug, debug_params)

    page = debug / pages.PAGES["local_sat"]
    assert page in made
    assert "no step check" in page.read_text(encoding="utf-8")


def test_the_pages_of_a_run_follow_its_section(tmp_path: Path) -> None:
    """Only the pages the section names are built, and the index links those."""
    debug = tmp_path / "debug"
    records(debug)
    debug_params = DebugParamsT()
    debug_params.html = True
    debug_params.globalSat = True

    made = pages.build(debug, debug_params)

    assert made == [debug / pages.PAGES["global_sat"], debug / "index.html"]
    text = (debug / "index.html").read_text(encoding="utf-8")
    assert 'href="global-sat.html"' in text
    assert 'href="local-sat.html"' not in text


def test_plan_writes_the_pages_the_config_asks_for(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """A `[debug]` section in the configuration makes a run write its pages, with no flag."""
    source = BENCHMARKS / "4q" / "config.toml"
    config = tmp_path / "config.toml"
    config.write_text(
        source.read_text(encoding="utf-8").replace(
            'input = "routing_config.json"', f'input = "{(BENCHMARKS / "4q" / "routing_config.json").as_posix()}"'
        )
        + "\n[debug]\nhtml = true\nlocal_sat = true\n",
        encoding="utf-8",
    )
    run = tmp_path / "run"
    assert main(["plan", "-c", str(config), "-o", str(run)]) == 0

    debug = run / "debug"
    assert (debug / "index.html").is_file()
    assert (debug / pages.PAGES["local_sat"]).is_file()
    assert not (debug / pages.PAGES["pre_feedline_routing_dashboard"]).exists()
    assert store.kinds(debug) == ["phase", "step-check"]
    assert f"{debug / 'index.html'}" in capsys.readouterr().out


def test_plan_writes_every_page_with_debug(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """`-d` turns every page on, whatever the section says."""
    run = tmp_path / "run"
    assert main(["plan", "-c", str(BENCHMARKS / "4q" / "config.toml"), "-o", str(run), "-d"]) == 0

    debug = run / "debug"
    assert (debug / "index.html").is_file()
    for name in pages.PAGES.values():
        assert (debug / name).is_file(), name
    assert (debug / "run.log").is_file()
    assert "search" in store.kinds(debug)
    assert f"{debug / 'index.html'}" in capsys.readouterr().out


def test_the_debug_command_builds_the_pages_again(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """`mqt-scpd debug <run>` rebuilds every page the store has records for."""
    run = tmp_path / "run"
    assert main(["plan", "-c", str(BENCHMARKS / "4q" / "config.toml"), "-o", str(run), "-d"]) == 0
    index = run / "debug" / "index.html"
    index.unlink()
    (run / "debug" / pages.PAGES["local_sat"]).unlink()

    assert main(["debug", str(run)]) == 0

    assert index.is_file()
    assert (run / "debug" / pages.PAGES["local_sat"]).is_file()
    assert str(index) in capsys.readouterr().out


def test_the_debug_command_needs_records(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """A run without a debug store says what to run."""
    run = tmp_path / "run"
    assert main(["plan", "-c", str(BENCHMARKS / "4q" / "config.toml"), "-o", str(run)]) == 0

    assert main(["debug", str(run)]) == 1
    assert "no debug records" in capsys.readouterr().err


def couplers() -> list[dict[str, Any]]:
    """One coupler with two options, as a coupler session lists them.

    Returns:
        The couplers.
    """
    option = {
        "open": True,
        "orientation": 0,
        "offset": 0,
        "secondPort": False,
        "jog": 0,
        "centre": [1, 1],
        "in": [1, 0],
        "out": [1, 2],
        "pad": [[0.5, 0.5], [1.5, 0.5], [1.5, 1.5], [0.5, 1.5]],
        "lead": [[1, 1], [2, 1]],
        "resonator": 1000,
    }
    return [
        {
            "index": 0,
            "resonator": "</script>7",
            "chain": 0,
            "at": 1,
            "chosen": 0,
            "jogs": False,
            "edgeIn": "f0",
            "edgeOut": "f1",
            "options": [{**option, "index": 0}, {**option, "index": 1, "open": False}],
        }
    ]


class FakeSession:
    """What the server needs of a coupler session, without the Final stage."""

    def __init__(self, lines: list[str]) -> None:
        """Start with coupler 0 on option 0.

        Args:
            lines: Where the session says what it does, as the stage's progress does.
        """
        self.lines = lines
        self.chosen = 0

    def couplers(self) -> str:
        """The couplers as JSON.

        Returns:
            The JSON.
        """
        listed = couplers()
        listed[0]["chosen"] = self.chosen
        return json.dumps({"couplers": listed})

    def set_option(self, coupler: int, option: int) -> str:
        """Put the coupler on another option.

        Returns:
            What became of its edges, as JSON.

        Raises:
            ValueError: When the coupler or the option does not exist.
        """
        if coupler != 0 or option not in {0, 1}:
            msg = f"coupler {coupler} has no option {option}"
            raise ValueError(msg)
        self.chosen = option
        self.lines.append(f"[Coupler Session] '7' on option {option}: edge f0 NO WAY")
        return json.dumps({"coupler": 0, "option": option, "edges": [{"edge": "f0", "drawn": False, "cells": 0}]})

    def graph(self) -> str:
        """The capacity graph as JSON.

        Returns:
            The JSON.
        """
        self.lines.append(f"coupler insertion: CAPACITY GRAPH UNSAT — option {self.chosen}")
        return json.dumps(graph())


def test_the_session_data_is_embedded_and_no_name_ends_its_script() -> None:
    """The coupler tab's data is in the page as it was given."""
    session = {"couplers": couplers(), "last": {"action": "the coupler insertion", "lines": []}}
    text = serve.page_text(graph(), session)
    embedded = re.search(r'<script id="session-data" type="application/json">(.*?)</script>', text, re.DOTALL)
    assert embedded is not None
    assert json.loads(embedded.group(1)) == session
    assert text.count("</script>") == 4
    assert 'data-pane="couplers"' in text


def test_the_server_moves_a_coupler_and_answers_the_new_graph() -> None:
    """`POST /option` moves the coupler and analyses anew; the page then carries the new state."""
    lines: list[str] = []
    server = serve.CouplerServer(FakeSession(lines), lines)
    assert server.last["lines"] == ["coupler insertion: CAPACITY GRAPH UNSAT — option 0"]
    httpd = HTTPServer(("127.0.0.1", 0), server.handler())
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    port = httpd.server_address[1]

    def ask(method: str, path: str, body: bytes | None = None) -> tuple[int, str]:
        connection = http.client.HTTPConnection("127.0.0.1", port, timeout=10)
        try:
            connection.request(method, path, body=body, headers={"Content-Type": "application/json"})
            reply = connection.getresponse()
            return reply.status, reply.read().decode()
        finally:
            connection.close()

    try:
        status, text = ask("POST", "/option", json.dumps({"coupler": 0, "option": 1}).encode())
        assert status == 200
        answer = json.loads(text)
        assert answer["action"] == "coupler 0 on option 1"
        assert answer["edges"] == [{"edge": "f0", "drawn": False, "cells": 0}]
        assert answer["lines"] == [
            "[Coupler Session] '7' on option 1: edge f0 NO WAY",
            "coupler insertion: CAPACITY GRAPH UNSAT — option 1",
        ]

        status, text = ask("GET", "/")
        assert status == 200
        embedded = re.search(r'<script id="session-data" type="application/json">(.*?)</script>', text, re.DOTALL)
        assert embedded is not None
        session = json.loads(embedded.group(1))
        assert session["couplers"][0]["chosen"] == 1
        assert session["last"] == answer

        status, text = ask("POST", "/option", b'{"coupler": 5, "option": 0}')
        assert status == 400
        assert "no option" in json.loads(text)["error"]
    finally:
        httpd.shutdown()
        httpd.server_close()


def test_a_coupler_session_holds_the_final_stage_of_a_run(tmp_path: Path) -> None:
    """The session runs to the coupler insertion; a coupler moved there stands on its new option."""
    run = tmp_path / "run"
    assert main(["plan", "-c", str(BENCHMARKS / "4q" / "config.toml"), "-o", str(run)]) == 0
    directory = RunDirectory(run)
    lines: list[str] = []
    session = directory.coupler_session(directory.load(), lines.append)

    listed = json.loads(session.couplers())["couplers"]
    coupler = next(c for c in listed if len(c["options"]) > 1)
    option = next(o["index"] for o in coupler["options"] if o["index"] != coupler["chosen"])
    moved = json.loads(session.set_option(coupler["index"], option))
    assert moved["coupler"] == coupler["index"]
    assert {e["edge"] for e in moved["edges"]} == {e for e in (coupler["edgeIn"], coupler["edgeOut"]) if e}
    again = next(c for c in json.loads(session.couplers())["couplers"] if c["index"] == coupler["index"])
    assert again["chosen"] == option
    assert {"raster", "nodes", "edges", "wires"} <= json.loads(session.graph()).keys()
    assert any("CAPACITY GRAPH" in line for line in lines)
    with pytest.raises(ValueError, match="has no option"):
        session.set_option(coupler["index"], len(coupler["options"]))
