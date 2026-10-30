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
import shutil
import subprocess
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
    from collections.abc import Sequence
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


def search_record(
    wire: str,
    round_: int,
    kind: str,
    *,
    found: bool,
    ripped: Sequence[str] = (),
    forward: bool = True,
    way: Sequence[Sequence[int]] = (),
) -> str:
    """One search of the feedline-constrained routing, as the Final stage records it.

    Returns:
        The record as JSON text.
    """
    return json.dumps({
        "phase": "feedline-constrained-routing",
        "pass": "feedline",
        "round": round_,
        "forward": forward,
        "wire": wire,
        "kind": kind,
        "ripped": list(ripped),
        "found": found,
        "cells": 9 if found else 0,
        "note": "",
        "way": [list(p) for p in way] if found else [],
    })


def dashboard_records(debug: Path) -> None:
    """The records of a feedline-constrained routing of two rounds.

    Wire ``a`` finds its way in round 0; wire ``b`` finds none, then finds one after letting go of
    ``a``. In the backward round 1 ``a`` finds no way, not even after letting go of ``b``.
    """
    sink = store.Sink(debug)
    sink(
        "grid",
        json.dumps({
            "width": 20,
            "height": 10,
            "cellSize": 10.0,
            "clearance": 2,
            "blocked": [[0, 0, 20], [1, 0, 20], [2, 5, 3]],
            "ports": [{"label": "Chip.port0", "x": 1, "y": 9}],
            "wires": [
                {"wire": "a", "inner": False, "resonator": True, "source": [1, 5], "target": [18, 5]},
                {"wire": "b", "inner": False, "resonator": False, "source": [1, 7], "target": [18, 7]},
            ],
        }),
    )
    sink("search", search_record("a", 0, "normal", found=True, way=[[1, 5], [18, 5]]))
    sink("search", search_record("b", 0, "normal", found=False))
    sink("search", search_record("b", 0, "relax 1", found=True, ripped=["a"], way=[[1, 7], [18, 7]]))
    sink("search", search_record("a", 1, "normal", found=False, forward=False))
    sink("search", search_record("a", 1, "relax 1", found=False, forward=False, ripped=["b"]))
    sink.close()


def test_a_dashboard_has_its_two_views_and_relative_links_only(tmp_path: Path) -> None:
    """The table and the view of one search are one file, and every link holds wherever it is served."""
    debug = tmp_path / "debug"
    dashboard_records(debug)
    params = DebugParamsT()
    params.feedlineConstrainedRoutingDashboard = True

    pages.build(debug, params)

    text = (debug / pages.PAGES["feedline_constrained_routing_dashboard"]).read_text(encoding="utf-8")
    for element in ('id="board"', 'id="detail"', 'id="hidden"', 'id="grid-data"', 'id="search-data"'):
        assert element in text
    # The cells link to fragments of the page and open them in a tab of their own.
    assert 'href="#search-${at}" target="_blank"' in text
    assert 'href="index.html"' in text
    assert re.search(r'href="(?:/|[a-z]+:)', text) is None
    assert "src=" not in text


def test_the_debug_command_builds_a_dashboard_only_for_a_phase_with_searches(tmp_path: Path) -> None:
    """A store with searches of two phases turns on the dashboards of those two phases alone."""
    debug = tmp_path / "debug"
    records(debug)

    params = pages.with_data(debug)

    assert params.preFeedlineRoutingDashboard
    assert params.feedlineConstrainedRoutingDashboard
    assert not params.preFeedlineRefinementDashboard
    assert not params.feedlineConstrainedRefinementDashboard


#: Runs the script of a dashboard on a few stand-in elements and prints what it wrote into them.
HARNESS = """
const vm = require("vm");
const input = JSON.parse(require("fs").readFileSync(process.argv[2], "utf8"));
class Element {
  constructor(text = "") {
    this.textContent = text; this.innerHTML = ""; this.hidden = false; this.dataset = {};
    this.style = { setProperty() {} }; this.attributes = {};
  }
  addEventListener() {}
  querySelectorAll() { return []; }
  querySelector() { return null; }
  setAttribute(key, value) { this.attributes[key] = value; }
  get clientWidth() { return 1000; }
  get clientHeight() { return 800; }
}
const elements = { "grid-data": new Element(input.grid), "search-data": new Element(input.searches) };
const classes = new Set();
global.document = {
  title: "feedline constrained routing",
  body: { classList: { add: (c) => classes.add(c), remove: (c) => classes.delete(c) } },
  getElementById: (id) => (elements[id] ||= new Element()),
};
global.location = { hash: input.hash };
global.addEventListener = () => {};
vm.runInThisContext(input.code);
const picture = elements.picture ? elements.picture.attributes : {};
console.log(JSON.stringify({
  board: elements.board.innerHTML, detail: elements.detail.innerHTML, hidden: elements.detail.hidden,
  title: document.title, detailClass: classes.has("detail"), viewBox: picture.viewBox || "",
}));
"""


def run_dashboard(page: Path, hash_: str, tmp_path: Path) -> dict[str, Any]:
    """What the script of a dashboard writes when the page is opened at ``hash_``.

    Returns:
        The table's and the search view's HTML, whether the search view is hidden, the title, whether
        the page is in its search view, and the picture's view box.
    """
    text = page.read_text(encoding="utf-8")

    def inline(pattern: str) -> str:
        found = re.search(pattern, text, re.DOTALL)
        assert found is not None, pattern
        return found.group(1)

    data = {
        "grid": inline(r'<script id="grid-data" type="application/json">(.*?)</script>'),
        "searches": inline(r'<script id="search-data" type="application/json">(.*?)</script>'),
        "code": inline(r"<script>\n(.*?)</script>"),
        "hash": hash_,
    }
    harness = tmp_path / "harness.js"
    harness.write_text(HARNESS, encoding="utf-8")
    given = tmp_path / "input.json"
    given.write_text(json.dumps(data), encoding="utf-8")
    node = shutil.which("node")
    assert node is not None
    done = subprocess.run([node, str(harness), str(given)], capture_output=True, text=True, check=True)  # ruff: ignore[subprocess-without-shell-equals-true]
    return json.loads(done.stdout)


@pytest.mark.skipif(shutil.which("node") is None, reason="needs node to run the page's script")
def test_the_dashboard_groups_its_columns_by_round_and_links_every_search(tmp_path: Path) -> None:
    """A round is one group of columns, a plain search and its relaxation levels; every cell links its search."""
    debug = tmp_path / "debug"
    dashboard_records(debug)
    params = DebugParamsT()
    params.feedlineConstrainedRoutingDashboard = True
    pages.build(debug, params)
    page = debug / pages.PAGES["feedline_constrained_routing_dashboard"]

    shown = run_dashboard(page, "", tmp_path)

    board = shown["board"]
    assert shown["hidden"]
    assert not shown["detailClass"]
    assert board.count('colspan="2"') == 2
    assert "round 0 →" in board
    assert "round 1 ←" in board
    for at in range(5):
        assert f'href="#search-{at}" target="_blank"' in board
    # Wire a ends without a way: its round 1 gave up; wire b stands.
    assert '<span class="badge">no way</span>' in board
    assert board.count('data-fail="1"') == 1
    assert '<td class="w resonator">a</td>' in board


@pytest.mark.skipif(shutil.which("node") is None, reason="needs node to run the page's script")
def test_a_search_opens_with_its_layers_and_what_the_records_say(tmp_path: Path) -> None:
    """``#search-<n>`` shows that search: the layers its picture holds and why it failed."""
    debug = tmp_path / "debug"
    dashboard_records(debug)
    params = DebugParamsT()
    params.feedlineConstrainedRoutingDashboard = True
    pages.build(debug, params)
    page = debug / pages.PAGES["feedline_constrained_routing_dashboard"]

    shown = run_dashboard(page, "#search-4", tmp_path)

    detail = shown["detail"]
    assert not shown["hidden"]
    assert shown["detailClass"]
    assert shown["title"].startswith("wire a · feedline · round 1 ← · relax 1")
    for layer in ("ob", "rp", "src", "tgt", "prt", "prl"):
        assert f'data-cls="{layer}"' in detail, layer
    # No other wire stands (b is let go of), so there is no clearance of one to draw.
    assert 'data-cls="cl"' not in detail
    # Nothing found and a was let go of by b's relaxation: no way of its own, and none of a to draw.
    assert 'data-cls="fd"' not in detail
    assert 'data-cls="ow"' not in detail
    assert "this round gave up" in detail
    assert "it let go of b" in detail
    assert 'href="#search-3"' in detail
    assert 'href="#table"' in detail
    # The picture is looked at around the search, y flipped: the box spans the wire's two ends.
    x, y, width, height = (float(v) for v in shown["viewBox"].split())
    assert x == 0
    assert width == 20
    assert y + height <= 10


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
    embedded_session = json.loads(session.group(1))
    assert embedded_session["couplers"] == couplers()
    assert embedded_session["searched"]
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
        "closed": None,
        "onFeedline": [],
        "orientation": 0,
        "offset": 0,
        "secondPort": False,
        "jog": 0,
        "reversed": False,
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
            "guardStoodDown": False,
            "options": [
                {**option, "index": 0},
                {
                    **option,
                    "index": 1,
                    "open": False,
                    "jog": 20,
                    "reversed": True,
                    "closed": {"kind": "jog-locked", "why": "jog locked"},
                },
            ],
            "refused": [
                {
                    "offset": 1,
                    "secondPort": True,
                    "jog": 0,
                    "reversed": False,
                    "places": 4,
                    "why": "the pad leaves the grid",
                },
                {
                    "offset": 3,
                    "secondPort": None,
                    "jog": None,
                    "reversed": None,
                    "places": 0,
                    "why": "offset not swept: coupler_offsets leaves it out",
                },
            ],
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
        self.placed: list[tuple[int, int, int, int]] = []

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
        if coupler != 0 or not 0 <= option < 2 + len(self.placed):
            msg = f"coupler {coupler} has no option {option}"
            raise ValueError(msg)
        self.chosen = option
        self.lines.append(f"coupler session: '7' on option {option}: edge f0 NO WAY")
        return json.dumps({"coupler": 0, "option": option, "edges": [{"edge": "f0", "drawn": False, "cells": 0}]})

    def add_option(self, coupler: int, x: int, y: int, offset: int = -1, port: int = 0) -> str:
        """Build an option of coupler 0 at the cell (1, 1) only.

        Returns:
            What came of it, as JSON.

        Raises:
            ValueError: When the coupler does not exist.
        """
        if coupler != 0:
            msg = f"there is no coupler {coupler}"
            raise ValueError(msg)
        if (x, y) != (1, 1):
            tried = [{"offset": 0, "secondPort": False, "why": "the pad leaves the grid"}]
            return json.dumps({"coupler": 0, "built": False, "cell": 0, "tried": tried})
        self.placed.append((x, y, offset, port))
        option = 1 + len(self.placed)
        return json.dumps({
            "coupler": 0,
            "built": True,
            "cell": 0,
            "option": option,
            "offset": max(offset, 0),
            "secondPort": port == 2,
            "tried": [],
        })

    def graph(self) -> str:
        """The capacity graph as JSON.

        Returns:
            The JSON.
        """
        self.lines.append(f"coupler insertion: CAPACITY GRAPH UNSAT — option {self.chosen}")
        return json.dumps(graph())

    def chains(self) -> str:
        """The one chain, with coupler 0 between edges f0 and f1.

        Returns:
            The JSON.
        """
        return json.dumps({
            "chains": [{"chain": 0, "couplers": [None, 0, None], "options": [0, self.chosen, 0], "edges": ["f0", "f1"]}]
        })

    @staticmethod
    def local_check(chain: int, options: list[int]) -> str:
        """Both local checks find the chain SAT.

        Returns:
            The JSON.
        """
        judged = {"verdict": "SAT", "lost": [], "graph": graph()}
        return json.dumps({"chain": chain, "options": options, "undrawn": [], "standing": judged, "alone": judged})

    @staticmethod
    def step_check(chain: int, step: int, options: list[int]) -> str:
        """The step check finds the step SAT.

        Returns:
            The JSON.
        """
        return json.dumps({
            "chain": chain,
            "step": step,
            "edge": step,
            "options": options,
            "verdict": "SAT",
            "lost": [],
            "closed": [],
            "undrawn": [],
            "graph": graph(),
        })

    @staticmethod
    def local_reroute(chain: int, options: list[int]) -> str:
        """The rip-up and reroute comes out clean.

        Returns:
            The JSON.
        """
        return json.dumps({
            "chain": chain,
            "options": options,
            "window": [],
            "clean": True,
            "failing": "",
            "blocked": "",
            "rounds": [],
            "seconds": 0.0,
            "wires": [],
        })

    @staticmethod
    def insertion_check() -> str:
        """The whole-chip check finds the chip SAT.

        Returns:
            The JSON.
        """
        return json.dumps({"verdict": "SAT", "lost": [], "graph": graph()})


def test_the_step_records_count_for_the_options_at_both_ends_of_their_edge() -> None:
    """A step record counts for the options at both ends of its edge; the far end's is the one tried."""
    steps = [
        {
            "chain": 0,
            "step": 1,
            "options": [0, 2, 1, 0],
            "couplers": [None, 5, 6, None],
            "verdict": "UNSAT",
            "lost": ["9"],
            "closed": ["9"],
            "refused": True,
        },
        {
            "chain": 0,
            "step": 1,
            "options": [0, 2, 3, 0],
            "couplers": [None, 5, 6, None],
            "verdict": "SAT",
            "lost": [],
            "closed": [],
            "refused": False,
        },
    ]
    skips = [{"chain": 0, "step": 2, "options": [0, 2, 3, 0], "couplers": [None, 5, 6, None], "why": "no way"}]
    found = serve.option_outcomes(steps, skips)

    assert set(found) == {"5", "6"}
    near = found["5"]["2"]
    assert (near["tried"], near["checks"], near["SAT"], near["UNSAT"], near["refused"]) == (0, 2, 1, 1, 1)
    assert near["lost"] == ["9"]
    assert near["closed"] == ["9"]
    assert set(near["example"]) == {"refused", "SAT"}
    refused = found["6"]["1"]
    assert (refused["tried"], refused["UNSAT"], refused["refused"]) == (1, 1, 1)
    assert refused["example"]["refused"] == {"chain": 0, "step": 1, "options": [0, 2, 1, 0]}
    # The skip of step 2 has no coupler at its far end and the option at its near end in the prefix.
    skipped = found["6"]["3"]
    assert (skipped["tried"], skipped["checks"], skipped["SAT"]) == (1, 1, 1)
    assert skipped["skipped"] == {"no way": 1}
    assert serve.option_outcomes([], []) == {}


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
            "coupler session: '7' on option 1: edge f0 NO WAY",
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
        assert "no coupler 5" in json.loads(text)["error"]
        status, text = ask("POST", "/option", b'{"coupler": 0, "option": 7}')
        assert status == 400
        assert "no option" in json.loads(text)["error"]
    finally:
        httpd.shutdown()
        httpd.server_close()


def test_the_server_places_an_option_by_hand_and_checks_the_chip_again() -> None:
    """`POST /place` builds an option, moves the coupler onto it and checks again; a refusal moves nothing."""
    lines: list[str] = []
    fake = FakeSession(lines)
    server = serve.CouplerServer(fake, lines)

    def place(body: dict[str, Any]) -> serve.Answer:
        return server.answer(serve.Request("POST", "/place", body=json.dumps(body).encode()))

    answer = place({"coupler": 0, "x": 1, "y": 1, "offset": 5, "port": 2})
    assert answer.status == 200
    moved = json.loads(answer.body)
    assert fake.placed == [(1, 1, 5, 2)]
    assert moved["placed"]["option"] == 2
    assert moved["action"] == "coupler 0 on option 2, placed by hand at (1, 1) · offset 5 · port 2"
    assert moved["verdicts"]["local"][0]["standing"]["verdict"] == "SAT"
    assert moved["verdicts"]["global"]["verdict"] == "SAT"
    assert server.standing() == {0: [0, 2, 0]}
    assert len(server.undo) == 1

    answer = place({"coupler": 0, "x": 1, "y": 1})
    assert json.loads(answer.body)["placed"]["option"] == 3
    assert fake.placed[-1] == (1, 1, -1, 0)

    answer = place({"coupler": 0, "x": 4, "y": 4})
    assert answer.status == 200
    refused = json.loads(answer.body)
    assert not refused["placed"]["built"]
    assert refused["action"].startswith("no option of coupler 0 fits at (4, 4)")
    assert server.standing() == {0: [0, 3, 0]}
    assert len(server.undo) == 2

    answer = place({"coupler": 5, "x": 1, "y": 1})
    assert answer.status == 400
    assert "no coupler 5" in json.loads(answer.body)["error"]


class LinesInto:
    """A reporter that keeps every line a stage says."""

    def __init__(self, lines: list[str]) -> None:
        """Append to ``lines``."""
        self.lines = lines

    def line(self, level: int, text: str) -> None:
        """Keep a line."""
        del level
        self.lines.append(text)

    @staticmethod
    def entry(level: int, label: str, figures: Sequence[tuple[str, str]]) -> None:
        """Drop an entry."""
        del level, label, figures

    @staticmethod
    def result(figures: Sequence[tuple[str, str]], fails: int | None) -> None:
        """Drop the result."""
        del figures, fails

    @staticmethod
    def progress(task: str, detail: str, done: int, total: int) -> None:
        """Drop the progress."""
        del task, detail, done, total


def test_a_coupler_session_holds_the_final_stage_of_a_run(tmp_path: Path) -> None:
    """The session runs to the coupler insertion; a coupler moved there stands on its new option."""
    run = tmp_path / "run"
    assert main(["plan", "-c", str(BENCHMARKS / "4q" / "config.toml"), "-o", str(run)]) == 0
    directory = RunDirectory(run)
    lines: list[str] = []
    session = directory.coupler_session(directory.load(), LinesInto(lines), 1)

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


def test_a_coupler_session_records_its_run_and_checks_a_chain_locally(tmp_path: Path) -> None:
    """The session hands its run's records to debug; it lists its chains and checks one locally."""
    run = tmp_path / "run"
    assert main(["plan", "-c", str(BENCHMARKS / "4q" / "config.toml"), "-o", str(run), "--stop-after", "detail"]) == 0
    directory = RunDirectory(run)
    config = directory.load()
    config.debug = DebugParamsT()
    config.debug.feedlineInsertion = True
    kinds: list[str] = []
    session = directory.coupler_session(config, debug=lambda kind, _record: kinds.append(kind))
    assert "coupler-options" in kinds

    chains = json.loads(session.chains())["chains"]
    assert [c["chain"] for c in chains] == [0]
    chain = chains[0]
    assert len(chain["couplers"]) == len(chain["options"]) == len(chain["edges"]) + 1
    assert all(o == 0 for c, o in zip(chain["couplers"], chain["options"], strict=True) if c is None)

    local = json.loads(session.local_check(0, chain["options"]))
    assert local["chain"] == 0
    assert local["options"] == chain["options"]
    for check in ("standing", "alone"):
        assert local[check]["verdict"] in {"SAT", "UNSAT"}
        assert {"raster", "nodes", "edges", "wires"} <= local[check]["graph"].keys()
    assert json.loads(session.chains())["chains"] == chains
    with pytest.raises(ValueError, match="there is no chain"):
        session.local_check(1, chain["options"])
    with pytest.raises(ValueError, match="waypoints"):
        session.local_check(0, chain["options"][:-1])
