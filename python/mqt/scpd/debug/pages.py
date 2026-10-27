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
(:mod:`.serve`) with the coupler options as its session.

The local SAT page is the feedline dashboard: a card per feedline with the
local checks of the coupler insertion, the step checks of its chain search,
and the states the global SAT search checked on it. The global SAT page is the
global dashboard: the course of the search and every combination it checked.
:func:`local_sat_page` and :func:`global_sat_page` build both from the records,
for a page that stands alone (its graph and mover links are off) and for a live
coupler session (:mod:`.serve`), whose links open the graph of a step check or
a state and move a feedline onto its options.
"""

from __future__ import annotations

import html
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any

from ..flatbuffers.config.DebugParams import DebugParamsT
from . import serve, store

if TYPE_CHECKING:
    from collections.abc import Callable, Iterable, Sequence

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


def _page(title: str, sub: str, body: str, script: str = "", style: str = "") -> str:
    return (
        '<!doctype html>\n<meta charset="utf-8">\n'
        '<meta name="viewport" content="width=device-width, initial-scale=1">\n'
        f"<title>{html.escape(title)}</title>\n<style>{STYLE}{style}</style>\n"
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
        steps = list(store.read(debug, "step-check"))
        skips = list(store.read(debug, "step-skip"))
        session = {
            "couplers": options.get("couplers", []),
            "outcomes": serve.option_outcomes(steps, skips),
            "searched": bool(steps or skips),
            "last": {"action": "the coupler insertion", "lines": graph.get("summary", [])},
        }
    return serve.page_text(graph, session)


#: What a SAT dashboard without a session says in place of its graph and mover links.
STATIC_NOTE = "start `mqt-scpd debug --serve` (or SAT Graph Debug in ChipBuilder) to open graphs"

#: The look of the two SAT dashboards, on top of :data:`STYLE`.
SAT_STYLE = """
 [hidden] { display: none !important }
 nav.links a { display: inline-block; margin: 2px 14px 2px 0 }
 .note { padding: 6px 10px; border: 1px dashed color-mix(in srgb, CanvasText 30%, transparent);
   border-radius: 6px; max-width: 900px }
 .tiles { display: flex; flex-wrap: wrap; gap: 10px; margin: 10px 0 }
 .tile { min-width: 120px; padding: 8px 12px; border-radius: 8px;
   border: 1px solid color-mix(in srgb, CanvasText 15%, transparent) }
 .tile b { display: block; font-size: 20px; font-variant-numeric: tabular-nums }
 .tile span { font-size: 11px; opacity: .75 }
 .bar { position: sticky; top: 0; z-index: 2; background: Canvas; display: flex; flex-wrap: wrap; gap: 8px;
   align-items: center; padding: 8px 0; border-bottom: 1px solid color-mix(in srgb, CanvasText 15%, transparent) }
 .bar button { font: inherit; padding: 2px 10px; border-radius: 999px; cursor: pointer; color: inherit;
   background: transparent; border: 1px solid color-mix(in srgb, CanvasText 25%, transparent) }
 .bar button.on { background: CanvasText; color: Canvas }
 .bar input { font: inherit; padding: 3px 8px; min-width: 240px }
 .cards { display: grid; grid-template-columns: repeat(auto-fill, minmax(min(100%, 460px), 1fr)); gap: 12px;
   margin-top: 12px }
 .card { border: 1px solid color-mix(in srgb, CanvasText 15%, transparent); border-radius: 8px; padding: 8px 12px;
   min-width: 0 }
 .card > header { display: flex; gap: 8px; align-items: baseline; flex-wrap: wrap }
 .card h2 { margin: 0; font-size: 14px }
 .edges { font: 11px ui-monospace, monospace; opacity: .75 }
 .facts { margin: 4px 0; font-size: 12px; opacity: .85 }
 .badge { font-size: 11px; font-weight: 700; padding: 0 7px; border-radius: 999px; border: 1.5px solid currentColor;
   white-space: nowrap }
 .badge.small { font-size: 10px; font-weight: 600; border-width: 1px }
 .card h3 { margin: 10px 0 2px; font-size: 12px }
 .SAT { color: #009E73 } .UNSAT { color: #D55E00 } .none { color: GrayText }
 .wide { overflow-x: auto }
 details { margin-top: 6px } summary { cursor: pointer; font-size: 12px }
 a.off { opacity: .45; text-decoration: none; cursor: not-allowed }
 .hit td, .card.hit { outline: 2px solid #0072B2; outline-offset: -2px }
 .banner { padding: 8px 12px; border-radius: 8px; border: 1.5px solid currentColor; margin: 8px 0; font-weight: 600;
   max-width: 900px }
"""

#: The filter bar, the search box and the address of a SAT dashboard: an element of class ``item``
#: carries ``data-verdict`` and ``data-text``; ``#<id>`` in the address opens and marks that element.
#: The listeners sit on the document, so a dashboard that :data:`BUILDING_SCRIPT` swaps in later
#: needs none of its own; ``satRefilter`` applies the filter and the mark to it.
SAT_SCRIPT = """<script>
(() => {
  let filter = "all";
  function apply() {
    const find = document.getElementById("find");
    const shown = document.getElementById("shown");
    if (!find || !shown) return;
    const items = [...document.querySelectorAll(".item")];
    const query = find.value.trim().toLowerCase();
    let count = 0;
    for (const item of items) {
      const on = (filter === "all" || item.dataset.verdict === filter) &&
        (!query || item.dataset.text.includes(query));
      item.hidden = !on;
      if (on) count++;
    }
    shown.textContent = `${count} of ${items.length} shown`;
  }
  function choose(value) {
    filter = value;
    for (const b of document.querySelectorAll("[data-filter]")) b.classList.toggle("on", b.dataset.filter === value);
    apply();
  }
  document.addEventListener("click", (event) => {
    const button = event.target.closest("[data-filter]");
    if (button) choose(button.dataset.filter);
  });
  document.addEventListener("input", (event) => { if (event.target.id === "find") apply(); });
  function mark() {
    for (const old of document.querySelectorAll(".hit")) old.classList.remove("hit");
    const target = document.getElementById(decodeURIComponent(location.hash.slice(1)));
    if (target) target.classList.add("hit");
    return target;
  }
  function reveal() {
    const target = mark();
    if (!target) return;
    for (let d = target.closest("details"); d; d = d.parentElement.closest("details")) d.open = true;
    const item = target.closest(".item");
    const find = document.getElementById("find");
    if (item && item.hidden && find) { find.value = ""; choose("all"); }
    target.scrollIntoView({ block: "center" });
  }
  addEventListener("hashchange", reveal);
  window.satRefilter = () => { choose(filter); mark(); };
  choose("all");
  reveal();
})();
</script>
"""

#: What a SAT dashboard of a session still being built says in place of its graph and mover links.
BUILDING_NOTE = "the graph and mover links open when the session stands"

#: The look of the status bar of a session still being built, on top of :data:`SAT_STYLE`.
STATUS_STYLE = """
 .status { position: sticky; top: 0; z-index: 3; background: Canvas; padding: 8px 12px; margin: 0 0 10px;
   border: 1.5px solid #E69F00; border-radius: 8px; max-width: 1100px }
 .status.failed { border-color: #D55E00 }
 .status .line { display: flex; flex-wrap: wrap; gap: 6px 14px; align-items: center }
 .status progress { width: 220px }
 .status pre { max-height: 240px; overflow: auto; font-size: 11px; margin: 6px 0 0; white-space: pre-wrap }
"""

#: The script of a page whose session is still being built. Every 1.5 s it reads ``status.json``
#: and shows it in the status bar. When the number of records grows, it fetches the page again and
#: swaps the element ``#dashboard`` in place: the filter, the search text, the open ``details``
#: and the scroll position stay. When the session stands or failed, it loads the page once more.
BUILDING_SCRIPT = """<script>
(() => {
  const status = document.getElementById("status");
  if (!status || status.dataset.state !== "building") return;
  let records = +status.dataset.records;
  const set = (id, text) => { const e = document.getElementById(id); if (e) e.textContent = text; };
  function show(s) {
    set("status-task", s.task ? `${s.task}${s.detail ? `: ${s.detail}` : ""}` : "starting the Final stage");
    const bar = document.getElementById("status-bar");
    if (bar) { bar.max = Math.max(s.total, 1); bar.value = s.total ? s.done : 0; }
    set("status-count", s.total ? `${s.done} / ${s.total}` : "");
    set("status-seconds", `${Math.round(s.seconds)} s`);
    set("status-phase", s.phase ? `last step: ${s.phase}` : "");
    const kinds = Object.entries(s.records).map(([kind, n]) => `${n} ${kind}`).join(" · ");
    set("status-records", kinds ? `records so far: ${kinds}` : "no records yet");
    set("status-lines", s.lines.join("\\n"));
  }
  function key(details) {
    const owner = details.parentElement.closest("[id]");
    const all = owner ? [...owner.querySelectorAll("details")] : [];
    return `${owner ? owner.id : ""}:${all.indexOf(details)}`;
  }
  async function swap() {
    const text = await (await fetch(location.pathname + location.search, { cache: "no-store" })).text();
    const fresh = new DOMParser().parseFromString(text, "text/html").getElementById("dashboard");
    const board = document.getElementById("dashboard");
    if (!fresh || !board) return;
    const find = document.getElementById("find");
    const query = find ? find.value : "";
    const focused = find !== null && document.activeElement === find;
    const open = new Set([...board.querySelectorAll("details[open]")].map(key));
    const y = scrollY;
    board.replaceWith(document.adoptNode(fresh));
    for (const d of fresh.querySelectorAll("details")) if (open.has(key(d))) d.open = true;
    const box = document.getElementById("find");
    if (box) { box.value = query; if (focused) box.focus(); }
    if (window.satRefilter) window.satRefilter();
    scrollTo(0, y);
  }
  async function poll() {
    try {
      const s = await (await fetch("status.json", { cache: "no-store" })).json();
      if (!s.building) { location.reload(); return; }
      show(s);
      const count = Object.values(s.records).reduce((a, b) => a + b, 0);
      if (count !== records) { records = count; await swap(); }
    } catch (error) {
      set("status-task", `the server does not answer: ${error.message}`);
    }
    setTimeout(poll, 1500);
  }
  setTimeout(poll, 1500);
})();
</script>
"""


def status_bar(status: dict[str, Any]) -> str:
    """The bar on top of a page whose session is still being built or failed to build.

    Args:
        status: How far the session is, as ``GET /status.json`` of :mod:`.serve` gives it.

    Returns:
        The bar as HTML. :data:`BUILDING_SCRIPT` keeps it up to date.
    """
    lines = html.escape("\n".join(status.get("lines", [])))
    counts = status.get("records", {})
    kinds = " · ".join(f"{n} {kind}" for kind, n in counts.items())
    records = f"records so far: {kinds}" if kinds else "no records yet"
    seconds = f"{status.get('seconds', 0.0):.0f} s"
    if status.get("failed"):
        return (
            f'<div class="status failed" id="status" data-state="failed"><div class="line">'
            f"<b>the session could not be built</b><span>{html.escape(str(status.get('error', '')))}</span>"
            f"<span>{seconds}</span><span>{html.escape(records)}</span></div>"
            f"<details open><summary>the last lines of the stage</summary><pre>{lines}</pre></details></div>"
        )
    task = str(status.get("task", ""))
    detail = str(status.get("detail", ""))
    done, total = int(status.get("done", 0)), int(status.get("total", 0))
    phase = str(status.get("phase", ""))
    return (
        f'<div class="status" id="status" data-state="building" data-records="{sum(counts.values())}">'
        '<div class="line"><b>the session is still being built</b>'
        f'<span id="status-task">{html.escape(f"{task}: {detail}" if detail else task or "starting the Final stage")}'
        f'</span><progress id="status-bar" max="{max(total, 1)}" value="{done if total else 0}"></progress>'
        f'<span id="status-count">{f"{done} / {total}" if total else ""}</span>'
        f'<span id="status-seconds">{seconds}</span></div>'
        f'<div class="line sub"><span id="status-phase">{html.escape(f"last step: {phase}" if phase else "")}</span>'
        f'<span id="status-records">{html.escape(records)}</span></div>'
        f'<p class="note">The page fills in as the records come; {BUILDING_NOTE}.</p>'
        f'<details><summary>the last lines of the stage</summary><pre id="status-lines">{lines}</pre></details></div>'
    )


def building_page(status: dict[str, Any]) -> str:
    """The page a coupler server answers for a page that needs the session while it is still being built.

    Args:
        status: How far the session is, as ``GET /status.json`` of :mod:`.serve` gives it.

    Returns:
        The page as HTML; it loads itself again when the session stands.
    """
    title = "the session could not be built" if status.get("failed") else "the session is still being built"
    nav = '<nav class="links"><a href="sat">local SAT</a><a href="global">global SAT</a></nav>'
    return _page(title, "", status_bar(status) + nav, BUILDING_SCRIPT, SAT_STYLE + STATUS_STYLE)


def _while_building(body: str, status: dict[str, Any]) -> str:
    """The body of a SAT dashboard of a session still being built: the status bar, then the dashboard.

    The dashboard sits in the element ``#dashboard``, which :data:`BUILDING_SCRIPT` swaps. The off
    links say why they are off.

    Returns:
        The body as HTML.
    """
    body = body.replace(f'title="{html.escape(STATIC_NOTE)}"', f'title="{html.escape(BUILDING_NOTE)}"')
    return f'{status_bar(status)}<div id="dashboard">{body}</div>'


@dataclass
class SatRecords:
    """The records the two SAT dashboards read.

    Attributes:
        states: The ``chain-state`` records, in the order the search made them.
        steps: The ``step-check`` records of the chain search.
        skips: The ``step-skip`` records: the steps the chain search dropped without a step check.
        events: The ``global-sat`` events.
        couplers: The couplers of the ``coupler-options`` record, or empty.
    """

    states: list[dict[str, Any]] = field(default_factory=list)
    steps: list[dict[str, Any]] = field(default_factory=list)
    skips: list[dict[str, Any]] = field(default_factory=list)
    events: list[dict[str, Any]] = field(default_factory=list)
    couplers: list[dict[str, Any]] = field(default_factory=list)

    @classmethod
    def of(cls, read: Callable[[str], Iterable[dict[str, Any]]]) -> SatRecords:
        """The records ``read`` gives by kind.

        Args:
            read: The records of one kind, as :func:`.store.read` gives them for a run's store.

        Returns:
            The records.
        """
        options = next(iter(read("coupler-options")), None)
        return cls(
            states=list(read("chain-state")),
            steps=list(read("step-check")),
            skips=list(read("step-skip")),
            events=list(read("global-sat")),
            couplers=list(options.get("couplers", [])) if options else [],
        )

    def kept(self) -> dict[int, list[list[int]]]:
        """The option vectors of the states every chain keeps, in kept order, by chain.

        Returns:
            The vectors of each chain the ``states`` events name; state 0 is the committed state.
        """
        return {int(e["chain"]): e.get("options", []) for e in self.events if e.get("event") == "states"}

    def committed(self) -> dict[int, list[int]]:
        """The committed options of every chain the search checked, by chain.

        Returns:
            The options of the first committed ``chain-state`` record of each chain.
        """
        found: dict[int, list[int]] = {}
        for state in self.states:
            if state.get("committed"):
                found.setdefault(int(state["chain"]), list(state["options"]))
        return found

    def state(self, chain: int, options: Sequence[int]) -> dict[str, Any] | None:
        """The ``chain-state`` record of a chain on some options.

        Args:
            chain: The chain.
            options: The option of every waypoint.

        Returns:
            The first record of the chain on exactly these options, or None.
        """
        wanted = list(options)
        return next((s for s in self.states if s["chain"] == chain and s["options"] == wanted), None)

    def step(self, chain: int, step: int, options: Sequence[int]) -> tuple[dict[str, Any] | None, int]:
        """The ``step-check`` record of a chain at one step on some options.

        The chain search may check the same step on the same options more than once.

        Args:
            chain: The chain.
            step: The step: the index of the edge in the chain.
            options: The option of every waypoint, those after the step on 0.

        Returns:
            The first such record, or None, and how many records there are.
        """
        wanted = list(options)
        found = [s for s in self.steps if s["chain"] == chain and s["step"] == step and s["options"] == wanted]
        return (found[0] if found else None), len(found)

    def combination(self, index: int) -> dict[str, Any] | None:
        """The ``combination`` event of one index.

        Args:
            index: The combination's index.

        Returns:
            The event, or None.
        """
        return next((e for e in self.events if e.get("event") == "combination" and e["index"] == index), None)

    def result(self) -> dict[str, Any] | None:
        """The ``result`` event of the search.

        Returns:
            The event, or None when no search was recorded.
        """
        return next((e for e in self.events if e.get("event") == "result"), None)


@dataclass
class Live:
    """What a live coupler session adds to the SAT dashboards.

    Attributes:
        chains: The chains as they stand, as ``CouplerSession.chains`` lists them.
        verdict: The whole-chip check of the chip as it stands: ``verdict`` and ``lost``.
        checked: The local checks of the chains checked since the last move, by chain.
        committed: The committed options of every chain, by chain.
        links: More pages the session serves, as (address, text), for the navigation.
    """

    chains: list[dict[str, Any]]
    verdict: dict[str, Any]
    checked: dict[int, dict[str, Any]] = field(default_factory=dict)
    committed: dict[int, list[int]] = field(default_factory=dict)
    links: list[tuple[str, str]] = field(default_factory=list)


def recorded_verdict(state: dict[str, Any], check: str) -> str | None:
    """What a ``chain-state`` record says of one of its two checks.

    The search runs the ``alone`` check only when the ``standing`` check finds the state UNSAT.

    Args:
        state: The record.
        check: ``standing`` or ``alone``.

    Returns:
        ``SAT`` or ``UNSAT``, or None when the search did not run that check.
    """
    found = state["check"]
    if check == "standing":
        return "SAT" if found == "standing" else "UNSAT"
    if found == "standing":
        return None
    return "SAT" if found == "alone" else "UNSAT"


def search_verdict(result: dict[str, Any] | None, *, building: bool = False) -> tuple[str, str]:
    """What the global SAT search came to, for a banner.

    Args:
        result: The search's ``result`` event, or None.
        building: Whether the session is still being built, so that the search may yet run.

    Returns:
        ``SAT``, ``UNSAT`` or ``none``, and the sentence that says it.
    """
    if result is None:
        if building:
            return "none", "the global SAT search runs after the coupler insertion; nothing of it is recorded yet"
        return "none", "no global SAT search was recorded"
    seconds = f"{result.get('seconds', 0.0):.1f} s"
    if not result.get("searched"):
        why = str(result.get("why", ""))
        if why == "SAT after the commit":
            return "SAT", "the chip is SAT after the commit: the search had nothing to search"
        return "none", f"nothing searched: {why}"
    if result.get("sat"):
        return "SAT", f"SAT after {result['combinations']} combinations: {result.get('moved', '')}; {seconds}"
    return "UNSAT", (
        f"no SAT combination in {result['combinations']} checked: the chip stays as the commit left it; {seconds}"
    )


def _badge(verdict: str, text: str | None = None) -> str:
    return f'<span class="badge {html.escape(verdict)}">{html.escape(text or verdict)}</span>'


def _tiles(tiles: Iterable[tuple[str, str, str]]) -> str:
    return (
        '<div class="tiles">'
        + "".join(
            f'<div class="tile"><b class="{html.escape(cls)}">{html.escape(value)}</b><span>{html.escape(label)}</span>'
            "</div>"
            for value, label, cls in tiles
        )
        + "</div>"
    )


def _bar(counts: dict[str, int], placeholder: str) -> str:
    buttons = "".join(
        f'<button type="button" data-filter="{key}">{label} ({counts.get(key, 0)})</button>'
        for key, label in (("all", "All"), ("SAT", "SAT"), ("UNSAT", "UNSAT"))
    )
    return (
        f'<div class="bar">{buttons}<input type="search" id="find" placeholder="{html.escape(placeholder)}">'
        '<span id="shown" class="sub"></span></div>'
    )


def _link(live: Live | None, href: str, text: str, title: str, *, tab: bool = True) -> str:
    if live is None:
        return f'<a class="off" title="{html.escape(STATIC_NOTE)}">{html.escape(text)}</a>'
    target = ' target="_blank"' if tab else ""
    return f'<a href="{html.escape(href)}"{target} title="{html.escape(title)}">{html.escape(text)}</a>'


def _nav(live: Live | None, *, building: bool = False) -> str:
    if live is None and not building:
        pages = (("index.html", "debug pages"), ("local-sat.html", "local SAT"), ("global-sat.html", "global SAT"))
    else:
        pages = (
            ("./", "global mover (capacity graph)"),
            ("sat", "local SAT"),
            ("global", "global SAT"),
            *(live.links if live is not None else ()),
        )
    links = "".join(f'<a href="{href}">{html.escape(text)}</a>' for href, text in pages)
    # A session still being built says so in its status bar.
    note = (
        ""
        if live is not None or building
        else f'<p class="note">The graph and mover links are off: {html.escape(STATIC_NOTE)}.</p>'
    )
    return f'<nav class="links">{links}</nav>{note}'


def _names(names: Sequence[str], most: int = 6) -> str:
    text = ", ".join(names[:most])
    if len(names) > most:
        text += f" +{len(names) - most}"
    if len(names) > most:
        return f'<span title="{html.escape(", ".join(names))}">{html.escape(text)}</span>'
    return html.escape(text)


def _moves(moved: Iterable[dict[str, Any]]) -> str:
    return ", ".join(f"c{m['coupler']}: {m['from']}→{m['to']}" for m in moved)


def options_text(options: Sequence[int]) -> str:
    """The options of a chain as a query and the pages write them.

    Args:
        options: The option of every waypoint.

    Returns:
        The options, separated by commas.
    """
    return ",".join(str(o) for o in options)


def _edge_number(name: str) -> tuple[int, str]:
    digits = name.lstrip("f")
    return (int(digits), name) if digits.isdigit() else (1 << 30, name)


def _edges_by_chain(records: SatRecords, live: Live | None) -> dict[int, list[str]]:
    edges: dict[int, set[str]] = {}
    if live is not None:
        for chain in live.chains:
            edges.setdefault(int(chain["chain"]), set()).update(e for e in chain.get("edges", []) if e)
    for coupler in records.couplers:
        edges.setdefault(int(coupler["chain"]), set()).update(
            e for e in (coupler.get("edgeIn"), coupler.get("edgeOut")) if e
        )
    for step in records.steps:
        if "edge" in step:
            edges.setdefault(int(step["chain"]), set()).add(f"f{step['edge']}")
    return {chain: sorted(names, key=_edge_number) for chain, names in edges.items()}


#: The class of a cell that holds a number.
NUMERIC = ' class="n"'

#: The columns of a table of chain states.
STATE_HEAD = ("state", "segment", "moves", "check", "lost", "s", "")


def _state_rows(
    chain: int, states: list[dict[str, Any]], kept: dict[tuple[int, ...], int], live: Live | None
) -> list[str]:
    rows = []
    for state in states:
        number = 0 if state.get("committed") else kept.get(tuple(state["options"]))
        anchor = f' id="chain-{chain}-state-{number}"' if number is not None else ""
        options = options_text(state["options"])
        check = state["check"] if state["check"] != "none" else "standing"
        graph = _link(
            live,
            f"graph?chain={chain}&options={options}&check={check}",
            "graph ↗",
            f"the {check} check of feedline {chain} on options {options}",
        )
        mover = _link(
            live,
            f"mover?chain={chain}&options={options}",
            "mover ↗",
            f"put feedline {chain} on options {options} and keep it",
            tab=False,
        )
        segment = "commit" if state.get("committed") else str(state.get("segment"))
        cells = [
            "" if number is None else str(number),
            html.escape(segment),
            html.escape(_moves(state.get("moved", []))) or "—",
            html.escape(str(state["check"])),
            _names(state.get("lost", [])),
            f"{state.get('seconds', 0.0):.2f}",
            f"{graph} {mover}",
        ]
        rows.append(
            f'<tr{anchor} title="options {html.escape(options)}">'
            + "".join(f"<td{NUMERIC if at in {0, 5} else ''}>{cell}</td>" for at, cell in enumerate(cells))
            + "</tr>"
        )
    return rows


def _state_table(rows: list[str]) -> str:
    head = "".join(f"<th{NUMERIC if at in {0, 5} else ''}>{h}</th>" for at, h in enumerate(STATE_HEAD))
    return f'<div class="wide"><table><tr>{head}</tr>{"".join(rows)}</table></div>'


def _check_links(live: Live | None, chain: int, options: Sequence[int], what: str) -> str:
    """The links that check a feedline locally on some options and that put it there.

    Returns:
        The ``graph ↗``, ``alone ↗`` and ``mover ↗`` links.
    """
    text = options_text(options)
    graph = _link(
        live,
        f"graph?chain={chain}&options={text}&check=standing",
        "graph ↗",
        f"{what}: the local check of feedline {chain} on options {text}, the other feedlines standing",
    )
    alone = _link(
        live,
        f"graph?chain={chain}&options={text}&check=alone",
        "alone ↗",
        f"{what}: the local check of feedline {chain} on options {text}, the feedline alone",
    )
    mover = _link(
        live, f"mover?chain={chain}&options={text}", "mover ↗", f"put feedline {chain} on options {text}", tab=False
    )
    return f"{graph} {alone} {mover}"


#: The columns of a table of step checks.
STEP_HEAD = ("edge", "tried", "options", "verdict", "lost", "closed", "", "s", "")


def _tried(step: dict[str, Any]) -> str:
    """What a step check tried: the coupler at the far end of its edge and that coupler's option.

    Returns:
        ``c<coupler>: <option>``, or ``launcher (end)`` when the edge ends at a launcher.
    """
    layer = int(step["step"]) + 1
    couplers = step.get("couplers") or []
    options = step.get("options") or []
    coupler = couplers[layer] if layer < len(couplers) else None
    if coupler is None:
        return "launcher (end)"
    option = options[layer] if layer < len(options) else "?"
    return f"c{coupler}: {option}"


def _step_rows(chain: int, steps: list[dict[str, Any]], live: Live | None, waypoints: int | None) -> list[str]:
    rows = []
    for step in steps:
        options = options_text(step["options"])
        links = ""
        if waypoints is None or len(step["options"]) == waypoints:
            graph = _link(
                live,
                f"graph?step={step['step']}&chain={chain}&options={options}",
                "graph ↗",
                f"the step check of feedline {chain} at step {step['step']} on options {options}, rebuilt by the "
                "session",
            )
            mover = _link(
                live,
                f"mover?chain={chain}&options={options}",
                "mover ↗",
                f"put feedline {chain} on options {options} and keep it (the waypoints after the step on option 0)",
                tab=False,
            )
            links = f"{graph} {mover}"
        edge = f"f{step['edge']}" if "edge" in step else f"step {step['step']}"
        cells = [
            html.escape(edge),
            html.escape(_tried(step)),
            f"<code>{html.escape(options)}</code>",
            _badge(step["verdict"]),
            _names(step.get("lost", [])),
            _names(step.get("closed", [])),
            "refused" if step.get("refused") else "",
            f"{step.get('seconds', 0.0):.3f}",
            links,
        ]
        rows.append(
            f'<tr title="step {html.escape(str(step["step"]))}">'
            + "".join(f"<td{NUMERIC if at == 7 else ''}>{cell}</td>" for at, cell in enumerate(cells))
            + "</tr>"
        )
    return rows


def _step_table(rows: list[str]) -> str:
    head = "".join(f"<th{NUMERIC if at == 7 else ''}>{h}</th>" for at, h in enumerate(STEP_HEAD))
    return f'<div class="wide"><table><tr>{head}</tr>{"".join(rows)}</table></div>'


def _edge_count(chain: int, steps: list[dict[str, Any]], live: Live | None) -> int | None:
    """How many feedline edges a chain has: from the session's chains, else from its step checks.

    Returns:
        The number of edges, or None when nothing says it.
    """
    if live is not None:
        standing = next((c for c in live.chains if int(c["chain"]) == chain), None)
        if standing is not None and standing.get("edges"):
            return len(standing["edges"])
    if steps and steps[0].get("options"):
        return len(steps[0]["options"]) - 1
    return None


def local_verdict(steps: list[dict[str, Any]], edges: int | None) -> str:
    """What the local checks of the coupler insertion say of a feedline.

    The chain search checks every step it prices: the chain with its couplers up to the far end of
    one edge. The feedline is SAT when a step check of its last edge is SAT.

    Args:
        steps: The ``step-check`` records of the feedline.
        edges: How many feedline edges it has, or None when unknown: the last step seen counts.

    Returns:
        ``SAT``, ``UNSAT``, or ``none`` when the chain search made no step check of it.
    """
    if not steps:
        return "none"
    last = edges - 1 if edges is not None else max(int(s["step"]) for s in steps)
    return "SAT" if any(int(s["step"]) == last and s["verdict"] == "SAT" for s in steps) else "UNSAT"


def _search_badge(states: list[dict[str, Any]], result: dict[str, Any] | None) -> str:
    """The small badge with the global SAT search's outcome for one feedline.

    Returns:
        The badge as HTML.
    """
    sat = sum(1 for s in states if s["verdict"] == "SAT")
    if states:
        verdict, text = ("SAT", f"global search: SAT states {sat}") if sat else ("UNSAT", "global search: UNSAT")
    elif result is None:
        verdict, text = "none", "global search: not recorded"
    else:
        verdict, text = "none", "global search: not needed"
    return f'<span class="badge small {verdict}">{html.escape(text)}</span>'


def _step_facts(steps: list[dict[str, Any]]) -> str:
    sat = sum(1 for s in steps if s["verdict"] == "SAT")
    refused = sum(1 for s in steps if s.get("refused"))
    seconds = sum(s.get("seconds", 0.0) for s in steps)
    return (
        f"local checks of the coupler insertion: {len(steps)} options checked · {sat} SAT · {len(steps) - sat} UNSAT"
        f" · {refused} refused · {seconds:.2f} s"
    )


def _card(
    chain: int,
    edges: list[str],
    states: list[dict[str, Any]],
    steps: list[dict[str, Any]],
    kept: list[list[int]],
    live: Live | None,
    result: dict[str, Any] | None,
    *,
    building: bool = False,
) -> tuple[str, str]:
    verdict = local_verdict(steps, _edge_count(chain, steps, live))
    # While the session is being built, a chain search that has not reached its last edge with a SAT
    # step may still be running: it is no verdict yet.
    running = building and verdict == "UNSAT"
    if running:
        verdict = "none"
    numbers = {tuple(options): n for n, options in enumerate(kept)}
    text = [f"feedline {chain}", *edges]
    for state in states:
        text += [f"c{m['coupler']}" for m in state.get("moved", [])] + list(state.get("lost", []))
    for step in steps:
        text += [_tried(step), *step.get("lost", []), *step.get("closed", [])]
    label = "in progress" if running else "no local check" if verdict == "none" else None
    body = [
        (
            f"<header><h2>feedline {chain}</h2>{_badge(verdict, label)}{_search_badge(states, result)}"
            f'<span class="edges">{html.escape(" ".join(edges))}</span></header>'
        ),
        f'<p class="facts">{html.escape(_step_facts(steps) if steps else "the chain search made no step check")}</p>',
    ]
    standing = next((c for c in live.chains if int(c["chain"]) == chain), None) if live is not None else None
    if live is not None:
        now = []
        if standing is not None:
            on = standing["options"]
            moved = chain in live.committed and on != live.committed[chain]
            now.append(f"stands on {options_text(on)}" + (" (moved off the commit)" if moved else ""))
        checked = live.checked.get(chain)
        if checked is not None:
            now.append(
                f"checked since the last move: standing {checked['standing']['verdict']}, "
                f"alone {checked['alone']['verdict']}"
            )
        if now:
            links = "" if standing is None else " " + _check_links(live, chain, standing["options"], "now")
            body.append(f'<p class="facts">now: {html.escape(" · ".join(now))}{links}</p>')
        # The movers of the feedline as it stands: its own, and the whole chip's.
        local_mover = _link(
            live,
            f"feedline?chain={chain}#tab=couplers",
            "local mover ↗",
            f"move the couplers of feedline {chain} alone",
            tab=False,
        )
        global_mover = _link(
            live,
            "./#tab=couplers",
            "global mover ↗",
            "move the couplers of every feedline on the whole chip",
            tab=False,
        )
        body.append(f'<p class="facts">movers: {local_mover} {global_mover}</p>')
    if steps:
        # Per edge its SAT options; an edge with none shows its UNSAT options; the rest waits below.
        waypoints = len(standing["options"]) if standing is not None else None
        by_step: dict[int, list[dict[str, Any]]] = {}
        for step in steps:
            by_step.setdefault(int(step["step"]), []).append(step)
        shown: list[dict[str, Any]] = []
        rest: list[dict[str, Any]] = []
        for at in sorted(by_step):
            checks = by_step[at]
            sat = [s for s in checks if s["verdict"] == "SAT"]
            shown += sat or checks
            rest += [s for s in checks if s["verdict"] != "SAT"] if sat else []
        body.append(_step_table(_step_rows(chain, shown, live, waypoints)))
        if rest:
            body.append(
                f"<details><summary>{len(rest)} more step checks: the UNSAT options of edges with a SAT option"
                f"</summary>{_step_table(_step_rows(chain, rest, live, waypoints))}</details>"
            )
    body.append("<h3>States of the global SAT search</h3>")
    if states:
        sat_states = [s for s in states if s["verdict"] == "SAT"]
        committed = next((s for s in states if s.get("committed")), None)
        facts = [f"{len(states)} states checked · {len(sat_states)} SAT"]
        if committed is not None:
            facts.append(f"the commit: {committed['verdict']} ({committed['check']})")
        if kept:
            facts.append(f"{len(kept)} kept")
        facts.append(f"{sum(s.get('seconds', 0.0) for s in states):.2f} s")
        body.extend((
            f'<p class="facts">{html.escape(" · ".join(facts))}</p>',
            _state_table(_state_rows(chain, sat_states or states, numbers, live)),
        ))
        unsat = [s for s in states if s["verdict"] != "SAT"] if sat_states else []
        if unsat:
            body.append(
                f"<details><summary>{len(unsat)} UNSAT states</summary>"
                f"{_state_table(_state_rows(chain, unsat, numbers, live))}</details>"
            )
    else:
        body.append('<p class="facts">the global SAT search collected no state of this feedline</p>')
    search = html.escape(" ".join(text).lower())
    return verdict, (
        f'<section class="card item" id="chain-{chain}" data-verdict="{verdict}" data-text="{search}">'
        + "".join(body)
        + "</section>"
    )


def local_sat_page(records: SatRecords, live: Live | None = None, building: dict[str, Any] | None = None) -> str:
    """The feedline dashboard: one card per feedline with the local checks of the coupler insertion.

    The chain search of the coupler insertion checks every step it prices locally: the feedline
    alone, with its couplers up to the far end of one edge (a ``step-check`` record). A feedline is
    SAT when a step check of its last edge is SAT, UNSAT when it has step checks but none of its
    last edge is SAT. Its card lists per edge the SAT options, or the UNSAT options when the edge
    has no SAT option, with the other step checks below; then the states the global SAT search
    checked on it, with a smaller badge for the search's outcome.

    Args:
        records: The records of the run or of the session.
        live: What the session adds, or None for a page that stands alone (its links are off).
        building: How far a session still being built is, as ``GET /status.json`` of :mod:`.serve`
            gives it, or None. The page then shows the records so far under a status bar, links the
            session's pages and keeps the graph and mover links off.

    Returns:
        The page as HTML.
    """
    title = "local SAT · feedlines"
    if live is None and building is None and not records.states and not records.steps:
        return _page(title, "", "<p>no step check and no chain state was recorded</p>")
    edges = _edges_by_chain(records, live)
    chains = sorted({*edges, *(int(s["chain"]) for s in records.states), *(int(s["chain"]) for s in records.steps)})
    kept = records.kept()
    result = records.result()
    cards = []
    counts = {"all": 0, "SAT": 0, "UNSAT": 0}
    for chain in chains:
        verdict, card = _card(
            chain,
            edges.get(chain, []),
            [s for s in records.states if s["chain"] == chain],
            [s for s in records.steps if s["chain"] == chain],
            kept.get(chain, []),
            live,
            result,
            building=building is not None,
        )
        cards.append(card)
        counts["all"] += 1
        counts[verdict] = counts.get(verdict, 0) + 1
    verdict, said = search_verdict(result, building=building is not None)
    steps = records.steps
    sat_steps = sum(1 for s in steps if s["verdict"] == "SAT")
    tiles = [
        (f"{counts['SAT']} / {len(chains)}", "feedlines locally SAT / all", "SAT"),
        (str(len(steps)), "step checks", ""),
        (f"{100 * sat_steps / len(steps):.0f} %" if steps else "—", "of the step checks SAT", ""),
        (str(sum(1 for s in steps if s.get("refused"))), "steps refused", ""),
        (verdict if verdict != "none" else "—", "global SAT search", verdict),
    ]
    if live is not None:
        now = str(live.verdict.get("verdict", "?"))
        tiles.insert(0, (now, "the chip now (whole-chip check)", now))
    body = (
        _nav(live, building=building is not None)
        + f'<p class="banner {verdict}">{html.escape(said)}</p>'
        + _tiles(tiles)
        + _bar(counts, "feedline, edge, coupler or wire")
        + f'<div class="cards">{"".join(cards)}</div>'
    )
    sub = (
        f"{len(chains)} feedlines · a feedline is SAT when the chain search's step check of its last edge is SAT; "
        "a card lists per edge the SAT options, or the UNSAT options when the edge has none"
    )
    if building is not None:
        body = _while_building(body, building)
        return _page(title, sub, body, SAT_SCRIPT + BUILDING_SCRIPT, SAT_STYLE + STATUS_STYLE)
    return _page(title, sub, body, SAT_SCRIPT, SAT_STYLE)


def _picks(states: Sequence[int], feedlines: str) -> str:
    picked = [
        f'<a href="{feedlines}#chain-{chain}-state-{n}">feedline {chain}: state {n}</a>'
        for chain, n in enumerate(states)
        if n
    ]
    return ", ".join(picked) or "the commit"


def global_sat_page(records: SatRecords, live: Live | None = None, building: dict[str, Any] | None = None) -> str:
    """The global dashboard: the course of the global SAT search and every combination it checked.

    Args:
        records: The records of the run or of the session.
        live: What the session adds, or None for a page that stands alone (its links are off).
        building: How far a session still being built is, as for :func:`local_sat_page`, or None.

    Returns:
        The page as HTML.
    """
    title = "global SAT · combinations"
    if live is None and building is None and not records.events:
        return _page(title, "", "<p>no global SAT search was recorded</p>")
    served = live is not None or building is not None
    feedlines = "sat" if served else PAGES["local_sat"]
    verdict, said = search_verdict(records.result(), building=building is not None)
    parts = [_nav(live, building=building is not None), f'<p class="banner {verdict}">{html.escape(said)}</p>']
    start = next((e for e in records.events if e.get("event") == "start"), None)
    combinations = [e for e in records.events if e.get("event") == "combination"]
    kept = records.kept()
    result = records.result() or {}
    tiles = [
        (verdict if verdict != "none" else "—", "global SAT search", verdict),
        (str(len(combinations)), "combinations checked", ""),
        (str(sum(1 for c in combinations if c["verdict"] == "SAT")), "SAT combinations", "SAT"),
        (str(len(kept)), "feedlines with kept states", ""),
        (f"{result.get('seconds', 0.0):.1f}", "seconds", ""),
    ]
    if live is not None:
        now = str(live.verdict.get("verdict", "?"))
        tiles.insert(0, (now, "the chip now (whole-chip check)", now))
    parts.append(_tiles(tiles))
    if start is not None:
        parts.append(f"<p>after the commit the chip is UNSAT; lost: {_names(start.get('lost', []), 12)}</p>")
    if kept:
        rows = (
            [f'<a href="{feedlines}#chain-{chain}">feedline {chain}</a>', str(len(options))]
            for chain, options in sorted(kept.items())
        )
        parts.append("<h2>States kept per feedline</h2>" + _table(["feedline", "states"], rows, {1}))
    if combinations:
        counts = {
            "all": len(combinations),
            "SAT": sum(1 for c in combinations if c["verdict"] == "SAT"),
            "UNSAT": sum(1 for c in combinations if c["verdict"] != "SAT"),
        }
        rows = []
        for c in combinations:
            index = int(c["index"])
            text = " ".join(
                [str(index), str(c["stage"])]
                + [f"feedline {chain}" for chain, n in enumerate(c["states"]) if n]
                + list(c.get("lost", []))
            ).lower()
            graph = _link(live, f"graph?combination={index}", "graph ↗", f"the whole-chip check of combination {index}")
            cells = [
                str(index),
                html.escape(str(c["stage"]).rstrip(": ")),
                _picks(c["states"], feedlines),
                _badge(c["verdict"]),
                _names(c.get("lost", [])),
                graph,
            ]
            rows.append(
                f'<tr class="item" id="combination-{index}" data-verdict="{html.escape(c["verdict"])}" '
                f'data-text="{html.escape(text)}">'
                + "".join(f"<td{NUMERIC if at == 0 else ''}>{cell}</td>" for at, cell in enumerate(cells))
                + "</tr>"
            )
        head = "".join(
            f"<th{NUMERIC if at == 0 else ''}>{h}</th>"
            for at, h in enumerate(("#", "stage", "states", "verdict", "lost", ""))
        )
        parts.append(
            "<h2>Combinations</h2>"
            + _bar(counts, "combination, stage, feedline or wire")
            + f'<div class="wide"><table><tr>{head}</tr>{"".join(rows)}</table></div>'
        )
    elif verdict == "SAT" and not result.get("searched"):
        parts.append(f'<p>the <a href="{feedlines}">local SAT page</a> lists the step checks of the chain search</p>')
    sub = "a state number links to that state of its feedline on the local SAT page"
    if building is not None:
        body = _while_building("\n".join(parts), building)
        return _page(title, sub, body, SAT_SCRIPT + BUILDING_SCRIPT, SAT_STYLE + STATUS_STYLE)
    script = SAT_SCRIPT if combinations else ""
    return _page(title, sub, "\n".join(parts), script, SAT_STYLE)


def _local_sat(debug: Path) -> str:
    return local_sat_page(SatRecords.of(lambda kind: store.read(debug, kind)))


def _global_sat(debug: Path) -> str:
    return global_sat_page(SatRecords.of(lambda kind: store.read(debug, kind)))


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
