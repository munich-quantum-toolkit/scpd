# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Tests of the page the capacity graph is laid out on."""

from __future__ import annotations

import http.client
import json
import re
import threading
from http.server import HTTPServer
from pathlib import Path
from typing import TYPE_CHECKING

import pytest

from mqt.scpd import capacityview
from mqt.scpd.cli import main
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


def test_the_page_holds_the_graph_as_it_was_written(tmp_path: Path) -> None:
    """The page carries the data unchanged, and no name in it ends the script it sits in."""
    data = tmp_path / capacityview.DATA_NAME
    data.write_text(json.dumps(graph()), encoding="utf-8")
    page = tmp_path / capacityview.PAGE_NAME
    capacityview.build(data, page)

    text = page.read_text(encoding="utf-8")
    assert text.startswith("<!doctype html>")
    assert "<title>Capacity graph · SAT</title>" in text
    embedded = re.search(r'<script id="graph-data" type="application/json">(.*?)</script>', text, re.DOTALL)
    assert embedded is not None
    assert json.loads(embedded.group(1)) == graph()
    # The data, the session data, the page's script and the coupler tab's.
    assert text.count("</script>") == 4
    # A page that stands alone has no coupler session.
    session = re.search(r'<script id="session-data" type="application/json">(.*?)</script>', text, re.DOTALL)
    assert session is not None
    assert json.loads(session.group(1)) is None
    # The chip and the abstract graph can be shown apart or side by side.
    for view in ("chip", "graph", "both"):
        assert f'data-view="{view}"' in text


def test_data_that_is_no_capacity_graph_is_refused(tmp_path: Path) -> None:
    """A file without the raster, the nodes, the edges or the wires is not laid out."""
    data = tmp_path / capacityview.DATA_NAME
    data.write_text(json.dumps({"width": 4, "height": 2}), encoding="utf-8")
    with pytest.raises(ValueError, match="raster, nodes, edges, wires"):
        capacityview.build(data, tmp_path / capacityview.PAGE_NAME)


def test_the_command_line_writes_the_page_beside_the_data(tmp_path: Path) -> None:
    """Without `-o` the page goes next to the data it was built from."""
    data = tmp_path / capacityview.DATA_NAME
    data.write_text(json.dumps(graph()), encoding="utf-8")
    assert capacityview.main([str(data)]) == 0
    assert (tmp_path / capacityview.PAGE_NAME).is_file()


def test_plan_lays_the_capacity_graph_out_with_bottlenecks_and_debug(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    """`-d` with `SCPD_BOTTLENECKS=1` builds the page from the data the Final stage wrote."""
    monkeypatch.setenv("SCPD_BOTTLENECKS", "1")
    monkeypatch.setenv("SCPD_SEARCH_PICTURES", "0")
    run = tmp_path / "run"
    assert main(["plan", "-c", str(BENCHMARKS / "4q" / "config.toml"), "-o", str(run), "-d"]) == 0

    page = run / "debug" / capacityview.PAGE_NAME
    assert page.is_file()
    assert f"{page} (capacity graph)" in capsys.readouterr().out


def test_plan_leaves_the_data_of_an_earlier_run_alone(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """Without `SCPD_BOTTLENECKS` no page is built, whatever data an earlier run left."""
    monkeypatch.delenv("SCPD_BOTTLENECKS", raising=False)
    monkeypatch.setenv("SCPD_SEARCH_PICTURES", "0")
    run = tmp_path / "run"
    (run / "debug").mkdir(parents=True)
    (run / "debug" / capacityview.DATA_NAME).write_text(json.dumps(graph()), encoding="utf-8")
    assert main(["plan", "-c", str(BENCHMARKS / "4q" / "config.toml"), "-o", str(run), "-d"]) == 0

    assert not (run / "debug" / capacityview.PAGE_NAME).exists()


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
    text = capacityview.page_text(graph(), session)
    embedded = re.search(r'<script id="session-data" type="application/json">(.*?)</script>', text, re.DOTALL)
    assert embedded is not None
    assert json.loads(embedded.group(1)) == session
    assert text.count("</script>") == 4
    assert 'data-pane="couplers"' in text


def test_the_server_moves_a_coupler_and_answers_the_new_graph() -> None:
    """`POST /option` moves the coupler and analyses anew; the page then carries the new state."""
    lines: list[str] = []
    server = capacityview.CouplerServer(FakeSession(lines), lines)
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


def test_a_coupler_session_holds_the_final_stage_of_a_run(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """The session runs to the coupler insertion; a coupler moved there stands on its new option."""
    monkeypatch.setenv("SCPD_SEARCH_PICTURES", "0")
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
