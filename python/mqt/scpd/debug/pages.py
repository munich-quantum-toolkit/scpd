# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""The pages built from a run's debug records.

Every page is one HTML file with its data inlined, so it opens from the file
system. ``index.html`` links the pages the ``[debug]`` section asked for and
lists what every phase came to. A dashboard shows one phase: a row per wire,
a column per round and relaxation level, and the way a search found drawn
over the grid on a click. The insertion page is the capacity graph's page
(:mod:`.serve`) with the coupler options as its session. The local SAT page
lists every step check of the chain search, the global SAT page the course
of the search for a globally SAT state.
"""

from __future__ import annotations

import html
import json
from pathlib import Path
from typing import TYPE_CHECKING

from ..flatbuffers.config.DebugParams import DebugParamsT
from . import serve, store

if TYPE_CHECKING:
    from collections.abc import Iterable

#: The pages, by the key of ``[debug]`` that turns each on, with the file each is written to.
PAGES: dict[str, str] = {
    "pre_feedline_routing_dashboard": "pre-feedline-routing.html",
    "pre_feedline_refinement_dashboard": "pre-feedline-refinement.html",
    "feedline_insertion": "feedline-insertion.html",
    "local_sat": "local-sat.html",
    "global_sat": "global-sat.html",
    "feedline_constrained_routing_dashboard": "feedline-constrained-routing.html",
    "feedline_constrained_refinement_dashboard": "feedline-constrained-refinement.html",
}

#: The phase a dashboard shows.
DASHBOARDS: dict[str, str] = {
    "pre_feedline_routing_dashboard": "pre-feedline-routing",
    "pre_feedline_refinement_dashboard": "pre-feedline-refinement",
    "feedline_constrained_routing_dashboard": "feedline-constrained-routing",
    "feedline_constrained_refinement_dashboard": "feedline-constrained-refinement",
}

#: The record kinds every page reads, for the ``debug`` command to know which pages have data.
KINDS: dict[str, tuple[str, ...]] = {
    "pre_feedline_routing_dashboard": ("search",),
    "pre_feedline_refinement_dashboard": ("search",),
    "feedline_insertion": ("coupler-options", "capacity-graph"),
    "local_sat": ("step-check",),
    "global_sat": ("global-sat",),
    "feedline_constrained_routing_dashboard": ("search",),
    "feedline_constrained_refinement_dashboard": ("search",),
}

INDEX = "index.html"

STYLE = """
 :root { color-scheme: light dark }
 body { font: 13px/1.45 ui-sans-serif, system-ui, sans-serif; margin: 0; padding: 18px 22px; }
 h1 { font-size: 17px; margin: 0 0 4px } h2 { font-size: 14px; margin: 18px 0 6px }
 .sub { opacity: .7; margin: 0 0 14px }
 table { border-collapse: collapse; } th, td { padding: 2px 8px; text-align: left; vertical-align: top }
 th { font-weight: 600; font-size: 11px; opacity: .8; position: sticky; top: 0; background: Canvas }
 tr:nth-child(even) td { background: color-mix(in srgb, CanvasText 5%, transparent) }
 td.n, th.n { text-align: right; font-variant-numeric: tabular-nums }
 .bad { color: #D55E00; font-weight: 600 } .good { color: #009E73; font-weight: 600 }
 a { color: inherit }
 nav a { display: block; margin: 2px 0 }
 code { font-size: 12px }
"""


def _camel(key: str) -> str:
    first, *rest = key.split("_")
    return first + "".join(word.capitalize() for word in rest)


def enabled(debug: DebugParamsT | None) -> list[str]:
    """The pages a ``[debug]`` section turns on, in the order of :data:`PAGES`.

    Returns:
        The keys of the pages that are on.
    """
    if debug is None:
        return []
    return [key for key in PAGES if getattr(debug, _camel(key), False)]


def wanted(debug: DebugParamsT | None) -> bool:
    """Whether a section asks for anything at all: a page or the index.

    Returns:
        True when a run with this section records and writes something.
    """
    return bool(debug is not None and (debug.html or enabled(debug)))


def _embedded(value: object) -> str:
    # JSON has no "<" outside its strings, so writing every one as an escape
    # keeps the data from ending the script element it sits in.
    return json.dumps(value, separators=(",", ":")).replace("<", "\\u003c")


def _page(title: str, sub: str, body: str, script: str = "") -> str:
    return (
        '<!doctype html>\n<meta charset="utf-8">\n'
        f"<title>{html.escape(title)}</title>\n<style>{STYLE}</style>\n"
        f'<h1>{html.escape(title)}</h1>\n<p class="sub">{html.escape(sub)}</p>\n{body}\n{script}'
    )


def _table(head: Iterable[str], rows: Iterable[Iterable[str]], numeric: set[int] | None = None) -> str:
    numeric = numeric or set()

    def cls(at: int) -> str:
        return ' class="n"' if at in numeric else ""

    out = "<table><tr>" + "".join(f"<th{cls(at)}>{html.escape(h)}</th>" for at, h in enumerate(head)) + "</tr>"
    for row in rows:
        out += "<tr>" + "".join(f"<td{cls(at)}>{cell}</td>" for at, cell in enumerate(row)) + "</tr>"
    return out + "</table>"


#: The table and the picture of a dashboard, which read the inlined records.
DASHBOARD_SCRIPT = (
    '<div id="board"></div>\n'
    '<div id="panel" hidden><h2 id="panel-title"></h2><p id="panel-text"></p>'
    '<svg id="picture" width="640" height="480"></svg></div>\n'
    "<script>\n" + Path(__file__).with_name("dashboard.js").read_text(encoding="utf-8") + "</script>\n"
)


def _dashboard(debug: Path, phase: str, title: str) -> str:
    grid = next(iter(store.read(debug, "grid")), None)
    searches = [s for s in store.read(debug, "search") if s.get("phase") == phase]
    if grid is None or not searches:
        return _page(title, "", f"<p>no search of the {html.escape(phase)} phase was recorded</p>")
    body = (
        f'<script id="grid-data" type="application/json">{_embedded(grid)}</script>\n'
        f'<script id="search-data" type="application/json">{_embedded(searches)}</script>'
    )
    wires = len({s["wire"] for s in searches})
    sub = f"{wires} wires · {len(searches)} searches · a click on a cell draws the way the search found"
    return _page(title, sub, body, DASHBOARD_SCRIPT)


def _insertion(debug: Path) -> str:
    graphs = list(store.read(debug, "capacity-graph"))
    options = next(iter(store.read(debug, "coupler-options")), None)
    if not graphs:
        return _page("feedline insertion", "", "<p>no capacity graph was recorded</p>")
    graph = graphs[-1]
    session = None
    if options is not None:
        session = {
            "couplers": options.get("couplers", []),
            "last": {"action": "the coupler insertion", "lines": graph.get("summary", [])},
        }
    return serve.page_text(graph, session)


def _local_sat(debug: Path) -> str:
    steps = list(store.read(debug, "step-check"))
    if not steps:
        return _page("local SAT", "", "<p>no step check was recorded</p>")
    sat = sum(1 for s in steps if s["verdict"] == "SAT")
    refused = sum(1 for s in steps if s["refused"])
    seconds = sum(s.get("seconds", 0.0) for s in steps)
    rows = (
        [
            str(s["chain"]),
            str(s["step"]),
            html.escape(" ".join(str(o) for o in s["options"])),
            f'<span class="{"good" if s["verdict"] == "SAT" else "bad"}">{s["verdict"]}</span>',
            html.escape(", ".join(s["lost"])),
            html.escape(", ".join(s["closed"])),
            "refused" if s["refused"] else "",
            f"{s.get('seconds', 0.0):.3f}",
        ]
        for s in steps
    )
    sub = f"{len(steps)} step checks · {sat} SAT · {len(steps) - sat} UNSAT · {refused} refused · {seconds:.1f}s"
    return _page(
        "local SAT", sub, _table(["chain", "step", "options", "verdict", "lost", "closed", "", "s"], rows, {0, 1, 7})
    )


def _global_sat(debug: Path) -> str:
    events = list(store.read(debug, "global-sat"))
    if not events:
        return _page("global SAT", "", "<p>no global SAT search was recorded</p>")
    parts = []
    for event in events:
        if event["event"] == "start":
            parts.append(f"<p>the chip is UNSAT after the commit: {html.escape(', '.join(event['lost']))}</p>")
        elif event["event"] == "states":
            parts.append(f"<p>chain {event['chain']}: {event['states']} locally SAT states</p>")
    combinations = [e for e in events if e["event"] == "combination"]
    if combinations:
        rows = (
            [
                str(c["index"]),
                html.escape(str(c["stage"]).rstrip(": ")),
                html.escape(" ".join(str(s) for s in c["states"])),
                f'<span class="{"good" if c["verdict"] == "SAT" else "bad"}">{c["verdict"]}</span>',
                html.escape(", ".join(c["lost"])),
            ]
            for c in combinations
        )
        parts.append("<h2>Combinations</h2>" + _table(["#", "stage", "state per chain", "verdict", "lost"], rows, {0}))
    result = next((e for e in events if e["event"] == "result"), None)
    sub = ""
    if result is not None:
        if not result.get("searched"):
            sub = f"nothing searched: {result.get('why', '')}"
        elif result.get("sat"):
            sub = (
                f"SAT after {result['combinations']} combinations: {result.get('moved', '')}; "
                f"{result.get('seconds', 0.0):.1f}s"
            )
        else:
            sub = f"no SAT combination in {result['combinations']} checked; {result.get('seconds', 0.0):.1f}s"
    return _page("global SAT", sub, "\n".join(parts))


def _index(debug: Path, made: list[tuple[str, Path]]) -> str:
    phases = list(store.read(debug, "phase"))
    body = (
        "<nav>"
        + "".join(f'<a href="{html.escape(path.name)}">{html.escape(title)}</a>' for title, path in made)
        + "</nav>"
    )
    if (debug / "run.log").is_file():
        body += '<nav><a href="run.log">the lines the run said</a></nav>'
    if phases:
        rows = (
            [html.escape(p["phase"])]
            + [str(p.get(k, 0)) for k in ("wires", "drawn", "unrouted", "open", "short", "long", "crossing", "bad")]
            for p in phases
        )
        body += "<h2>Phases</h2>" + _table(
            ["phase", "wires", "drawn", "unrouted", "open", "short", "long", "crossing", "bad"], rows, set(range(1, 9))
        )
    return _page("debug pages", f"{debug.parent}", body)


def _title(key: str) -> str:
    return key.replace("_dashboard", "").replace("_", " ")


def build(debug: Path, params: DebugParamsT) -> list[Path]:
    """Write the pages ``params`` turns on, and the index when anything is on.

    Args:
        debug: The run's debug directory, with the store under it.
        params: The ``[debug]`` section.

    Returns:
        The files written, the index last.
    """
    made: list[tuple[str, Path]] = []
    for key in enabled(params):
        path = debug / PAGES[key]
        if key in DASHBOARDS:
            text = _dashboard(debug, DASHBOARDS[key], _title(key))
        elif key == "feedline_insertion":
            text = _insertion(debug)
        elif key == "local_sat":
            text = _local_sat(debug)
        else:
            text = _global_sat(debug)
        path.write_text(text, encoding="utf-8")
        made.append((_title(key), path))
    written = [path for _, path in made]
    if made or params.html:
        index = debug / INDEX
        index.write_text(_index(debug, made), encoding="utf-8")
        written.append(index)
    return written


def with_data(debug: Path) -> DebugParamsT:
    """A section that turns on every page the store under ``debug`` has records for.

    Returns:
        The section, with the index on.
    """
    present = set(store.kinds(debug))
    params = DebugParamsT()
    params.html = True
    for key, kinds in KINDS.items():
        if present.issuperset(kinds):
            setattr(params, _camel(key), True)
    return params


def all_pages() -> DebugParamsT:
    """A section with every page on, as ``-d`` asks for.

    Returns:
        The section.
    """
    params = DebugParamsT()
    params.html = True
    for key in PAGES:
        setattr(params, _camel(key), True)
    return params
