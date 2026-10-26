# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""The page of the capacity graph, alone or behind a live coupler session.

The Final stage records the capacity graph of its whole-chip check as data: the
chambers and walls as a raster, every edge of the graph with what it takes and
what the flow sends through it, and every outer wire with its two ports.
:func:`page_text` lays that data out on one HTML page with no outside
resources, from the template ``capacity.html`` beside this module. The page
draws the chambers and walls, the nodes and edges of the graph and a
``load/capacity`` badge on every edge, marks the target ports, and lists the
edges, the feedline edges and the wires in tables. A click on an edge, a
chamber, a feedline or a wire shows only the parts of the graph that belong to
it; the address can name one, as in ``#wire=14``, ``#edge=g20``,
``#chamber=c12`` or ``#feedline=f2``.

:func:`serve` puts the same page behind a local server that holds the Final
stage after its coupler insertion (``mqt-scpd debug --serve``): a tab of the
page then moves a coupler to another of its options, the server draws its two
feedline edges again and analyses the chip anew, and the page shows the new
graph. The server also answers the two SAT dashboards of :mod:`.pages` from the
records of the session's own run, with live links: the graph of a step check
of the chain search, of a chain state's local check or of a combination's
whole-chip check, and a move of a feedline onto the options of a step check or
a state.
"""

from __future__ import annotations

import html
import json
import re
import threading
from collections.abc import Callable
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from typing import TYPE_CHECKING, Any, Protocol
from urllib.parse import parse_qsl, urlsplit

from ..flatbuffers.config.DebugParams import DebugParamsT
from . import pages

if TYPE_CHECKING:
    from ..flatbuffers.config.Config import ConfigT

TEMPLATE = Path(__file__).with_name("capacity.html")
TITLE = re.compile(r"<title>.*?</title>", re.DOTALL)
DATA = re.compile(r'(<script id="graph-data" type="application/json">).*?(</script>)', re.DOTALL)
SESSION = re.compile(r'(<script id="session-data" type="application/json">).*?(</script>)', re.DOTALL)


def _embedded(value: object) -> str:
    # JSON has no "<" outside its strings, so writing every one as an escape
    # keeps the data from ending the script element it sits in.
    return json.dumps(value, separators=(",", ":")).replace("<", "\\u003c")


def page_text(
    graph: dict[str, Any], session: dict[str, Any] | None = None, banner: str | None = None, tone: str = "note"
) -> str:
    """The page for one capacity graph.

    Args:
        graph: The capacity graph, as the Final stage records it.
        session: What the coupler tab needs when a server holds the stage: the
            couplers and their options, and what the last change said. ``None``
            for a page that stands alone.
        banner: A line of HTML under the page's header that says which check
            judged the graph, or None.
        tone: The colour of the banner: ``SAT``, ``UNSAT``, ``mismatch`` or
            ``note``.

    Returns:
        The page as HTML.

    Raises:
        ValueError: When the data is not a capacity graph.
    """
    missing = [key for key in ("width", "height", "raster", "nodes", "edges", "wires") if key not in graph]
    if missing:
        msg = f"the data is not a capacity graph: no {', '.join(missing)}"
        raise ValueError(msg)
    title = f"<title>Capacity graph · {html.escape(str(graph.get('verdict', '?')))}</title>"
    text = TEMPLATE.read_text(encoding="utf-8")
    text = TITLE.sub(lambda _: title, text, count=1)
    text = DATA.sub(lambda found: found.group(1) + _embedded(graph) + found.group(2), text, count=1)
    if banner is not None:
        line = f'</header>\n    <div class="banner {html.escape(tone)}" id="banner">{banner}</div>'
        text = text.replace("</header>", line, 1)
    return SESSION.sub(lambda found: found.group(1) + _embedded(session) + found.group(2), text, count=1)


class Session(Protocol):
    """What the server needs of a coupler session (``pyscpd.CouplerSession``)."""

    def couplers(self) -> str:
        """The couplers and their options, as JSON."""
        ...

    def set_option(self, coupler: int, option: int) -> str:
        """Put a coupler on another option; what became of its edges, as JSON."""
        ...

    def graph(self) -> str:
        """The capacity graph of the chip as it stands, as JSON."""
        ...

    def chains(self) -> str:
        """Every chain as it stands: its couplers, options and feedline edges, as JSON."""
        ...

    def local_check(self, chain: int, options: list[int]) -> str:
        """The two local checks of a chain on other options, which then go back, as JSON."""
        ...

    def insertion_check(self) -> str:
        """The whole-chip check of the coupler insertion on the chip as it stands, as JSON."""
        ...

    def step_check(self, chain: int, step: int, options: list[int]) -> str:
        """The step check of the chain search at one edge of a chain, rebuilt on options, as JSON."""
        ...


#: The record kinds a coupler server keeps of its session's run.
KEPT = frozenset({"chain-state", "step-check", "global-sat", "coupler-options"})


class Recorder:
    """The debug callback of a session: keeps the records of some kinds in memory, by kind."""

    def __init__(self, kinds: frozenset[str] = KEPT) -> None:
        """Keep the records of ``kinds`` and drop every other record.

        Args:
            kinds: The kinds to keep.
        """
        self.kinds = kinds
        self.records: dict[str, list[dict[str, Any]]] = {}

    def __call__(self, kind: str, record: str) -> None:
        """Keep one record when its kind is kept.

        Args:
            kind: The record's kind.
            record: The record as JSON text.
        """
        if kind in self.kinds:
            self.records.setdefault(kind, []).append(json.loads(record))

    def read(self, kind: str) -> list[dict[str, Any]]:
        """The records of one kind, in the order they came.

        Returns:
            The records; empty when none came.
        """
        return self.records.get(kind, [])


def want_sat_records(config: ConfigT) -> None:
    """Turn on the ``[debug]`` keys whose records the SAT dashboards read.

    The Final stage records only the kinds the ``[debug]`` section asks for, so a session gives the
    records of its global SAT search only when ``global_sat`` is on, and the step checks of its
    chain search only when ``local_sat`` is on.

    Args:
        config: The configuration the session is built from; changed in place.
    """
    if config.debug is None:
        config.debug = DebugParamsT()
    config.debug.localSat = True
    config.debug.globalSat = True


@dataclass(frozen=True)
class Request:
    """A request to the server.

    Attributes:
        method: ``GET`` or ``POST``.
        path: The path, without the query.
        query: The query's keys and values.
        body: The body as it came.
    """

    method: str
    path: str
    query: dict[str, str] = field(default_factory=dict)
    body: bytes = b""

    def json(self) -> dict[str, Any]:
        """The body as a JSON object.

        Returns:
            The object; an empty one when the body is empty.

        Raises:
            TypeError: When the body is JSON but no object.
        """
        value = json.loads(self.body or b"{}")
        if not isinstance(value, dict):
            msg = "the body is no JSON object"
            raise TypeError(msg)
        return value

    def number(self, key: str) -> int:
        """A number of the query.

        Returns:
            The number.

        Raises:
            ValueError: When the query has no such key or its value is no number.
        """
        if key not in self.query:
            msg = f"the query names no {key}"
            raise ValueError(msg)
        return int(self.query[key])

    def numbers(self, key: str) -> list[int]:
        """A list of numbers of the query, separated by commas.

        Returns:
            The numbers.

        Raises:
            ValueError: When the query has no such key or a value is no number.
        """
        if key not in self.query:
            msg = f"the query names no {key}"
            raise ValueError(msg)
        return [int(n) for n in self.query[key].split(",") if n.strip()]


@dataclass(frozen=True)
class Answer:
    """The server's answer to a request.

    Attributes:
        status: The HTTP status.
        kind: The content type.
        body: The body.
        location: Where a redirect sends the browser, or None.
    """

    status: int
    kind: str
    body: str
    location: str | None = None

    @classmethod
    def page(cls, text: str) -> Answer:
        """An HTML page.

        Returns:
            The answer.
        """
        return cls(200, "text/html; charset=utf-8", text)

    @classmethod
    def data(cls, value: object, status: int = 200) -> Answer:
        """A JSON value.

        Returns:
            The answer.
        """
        return cls(status, "application/json", json.dumps(value))

    @classmethod
    def redirect(cls, location: str) -> Answer:
        """A redirect to another address (``303 See Other``).

        Returns:
            The answer.
        """
        return cls(303, "text/plain", f"see {location}", location)


#: A route answers a request, or answers None when the request is not its own; the server then
#: asks the next route of the same method and path.
Route = Callable[[Request], "Answer | None"]


def _verdicts(local: dict[str, Any]) -> dict[str, Any]:
    """A local check without its graphs.

    Returns:
        The chain, its options, its edges with no way and the verdict and lost wires of each check.
    """
    return {
        "chain": local["chain"],
        "options": local["options"],
        "undrawn": local.get("undrawn", []),
        "standing": {k: local["standing"][k] for k in ("verdict", "lost")},
        "alone": {k: local["alone"][k] for k in ("verdict", "lost")},
    }


def _joined(names: list[str]) -> str:
    return ", ".join(names) if names else "none"


class CouplerServer:
    """A coupler session behind a local web server.

    ``GET /`` answers the page of the capacity graph as the chip stands, with the coupler tab.
    ``GET /sat`` and ``GET /global`` answer the two SAT dashboards from the records of the
    session's run, with live links. ``GET /graph?step=S&chain=C&options=a,b,c`` answers the graph
    of the step check of the chain search at step S, rebuilt by the session;
    ``GET /graph?chain=C&options=a,b,c&check=standing`` the graph of one local check of the global
    SAT search; ``GET /graph?combination=K`` the graph of the whole-chip check of one combination of
    the global SAT search. All three put the couplers back. ``GET /mover?chain=C&
    options=a,b,c`` puts a feedline on the options and keeps it. ``POST /option``, ``POST
    /options``, ``POST /undo`` and ``POST /reset`` move couplers; after every move the server checks
    the moved feedlines locally and the chip as a whole. ``POST /check-all`` checks every feedline
    locally. ``GET /state.json``, ``GET /graph.json`` and ``GET /couplers.json`` answer data.

    :meth:`add_route` adds a route: another module serves its own pages through the same session.
    :meth:`add_link` links such a page from the navigation of the SAT dashboards.
    """

    def __init__(self, session: Session, lines: list[str], records: Recorder | None = None) -> None:
        """Hold a session and analyse the chip as it stands.

        Args:
            session: The session that holds the Final stage.
            lines: The list the session's progress callback appends to.
            records: The records of the session's run, or None when it recorded nothing.
        """
        self.session = session
        self.lines = lines
        self.records = pages.SatRecords.of(records.read) if records is not None else pages.SatRecords()
        listed = self._chains()
        #: The coupler of every waypoint, by chain (None at a fixed waypoint).
        self.waypoints: dict[int, list[int | None]] = {int(c["chain"]): list(c["couplers"]) for c in listed}
        self.chain_of: dict[int, int] = {
            coupler: chain for chain, couplers in self.waypoints.items() for coupler in couplers if coupler is not None
        }
        start = {int(c["chain"]): list(c["options"]) for c in listed}
        recorded = self.records.committed()
        #: The options of every chain as the commit left them: those of its committed
        #: ``chain-state`` record, or those it stood on when the session started.
        self.committed: dict[int, list[int]] = {chain: recorded.get(chain, options) for chain, options in start.items()}
        self.undo: list[dict[int, list[int]]] = []
        self.checked: dict[int, dict[str, Any]] = {}
        self.verdict = self._check_chip()
        self.graph, said = self._analyse()
        self.last: dict[str, Any] = {
            "action": "the coupler insertion",
            "lines": said,
            "verdicts": {"local": [], "global": self.verdict},
        }
        self.lock = threading.Lock()
        self.routes: dict[tuple[str, str], list[Route]] = {}
        self.links: list[tuple[str, str, Callable[[], bool]]] = []
        for path in ("/", "/index.html"):
            self.add_route("GET", path, lambda _: Answer.page(self.page()))
        self.add_route("GET", "/graph.json", lambda _: Answer.data(self.graph))
        self.add_route("GET", "/couplers.json", lambda _: Answer(200, "application/json", self.session.couplers()))
        self.add_route("GET", "/state.json", lambda _: Answer.data(self.state()))
        self.add_route("GET", "/sat", lambda _: Answer.page(pages.local_sat_page(self.records, self.live())))
        self.add_route("GET", "/global", lambda _: Answer.page(pages.global_sat_page(self.records, self.live())))
        self.add_route("GET", "/graph", self._step_graph)
        self.add_route("GET", "/graph", self._chain_graph)
        self.add_route("GET", "/graph", self._combination_graph)
        self.add_route("GET", "/mover", self._mover)
        self.add_route("POST", "/option", self._option)
        self.add_route("POST", "/options", self._options)
        self.add_route("POST", "/undo", lambda _: Answer.data(self.back()))
        self.add_route("POST", "/reset", lambda _: Answer.data(self.reset()))
        self.add_route("POST", "/check-all", lambda _: Answer.data(self.check_all()))

    def add_route(self, method: str, path: str, route: Route) -> None:
        """Answer requests of a method and a path with a route.

        Several routes may share a method and a path: the server asks them in the order they were
        added, and the first that answers something other than None answers the request. A route
        that raises ``ValueError``, ``KeyError`` or ``TypeError`` answers ``400`` with the error.

        Args:
            method: ``GET`` or ``POST``.
            path: The path, without the query, as ``/graph``.
            route: The route.
        """
        self.routes.setdefault((method, path), []).append(route)

    def add_link(self, href: str, text: str, shown: Callable[[], bool]) -> None:
        """Link a page of another module from the navigation of the SAT dashboards.

        Args:
            href: The address, relative to the dashboards, as ``bench/local``.
            text: What the link says.
            shown: Whether the page has something to show; asked on every request.
        """
        self.links.append((href, text, shown))

    def answer(self, request: Request) -> Answer:
        """Answer one request; one at a time, since a session is not safe across threads.

        Args:
            request: The request.

        Returns:
            The answer.
        """
        routes = self.routes.get((request.method, request.path))
        if not routes:
            return Answer(404, "text/plain", "not found")
        with self.lock:
            for route in routes:
                try:
                    found = route(request)
                except (ValueError, KeyError, TypeError) as error:
                    return Answer.data({"error": str(error)}, 400)
                if found is not None:
                    return found
        return Answer.data({"error": f"no route of {request.path} answers this query"}, 400)

    def _chains(self) -> list[dict[str, Any]]:
        return json.loads(self.session.chains())["chains"]

    def standing(self) -> dict[int, list[int]]:
        """The options every chain stands on.

        Returns:
            The option of every waypoint, by chain.
        """
        return {int(c["chain"]): list(c["options"]) for c in self._chains()}

    def _check_chip(self) -> dict[str, Any]:
        checked = json.loads(self.session.insertion_check())
        return {"verdict": checked["verdict"], "lost": checked.get("lost", []), "over": checked.get("over", [])}

    def _analyse(self) -> tuple[dict[str, Any], list[str]]:
        """Analyse the chip as it stands.

        Returns:
            The capacity graph, and the summary lines the analysis said.
        """
        start = len(self.lines)
        graph = json.loads(self.session.graph())
        said = [line for line in self.lines[start:] if "CAPACITY GRAPH" in line]
        return graph, said

    def live(self) -> pages.Live:
        """What the session adds to the SAT dashboards.

        Returns:
            The chains as they stand, the whole-chip verdict and the local checks since the last move.
        """
        links = [(href, text) for href, text, shown in self.links if shown()]
        return pages.Live(self._chains(), self.verdict, dict(self.checked), self.committed, links)

    def page(self) -> str:
        """The page as the chip stands.

        Returns:
            The page as HTML.
        """
        session = {
            "couplers": json.loads(self.session.couplers())["couplers"],
            "last": self.last,
            "live": True,
            "undo": len(self.undo),
        }
        return page_text(self.graph, session)

    def state(self) -> dict[str, Any]:
        """The state of the session.

        Returns:
            Per chain its options, its edges and whether it stands on the commit; the whole-chip
            verdict; the local checks of the chains checked since the last move; how many moves
            ``POST /undo`` can take back; and the last action.
        """
        chains = [
            {
                "chain": int(c["chain"]),
                "options": c["options"],
                "edges": c["edges"],
                "committed": c["options"] == self.committed.get(int(c["chain"])),
            }
            for c in self._chains()
        ]
        return {
            "chains": chains,
            "global": self.verdict,
            "local": list(self.checked.values()),
            "undo": len(self.undo),
            "last": self.last["action"],
        }

    def _apply(self, target: dict[int, list[int]]) -> tuple[list[dict[str, Any]], list[int]]:
        """Put chains on options; on an error, put them back.

        Args:
            target: The option of every waypoint, by chain.

        Returns:
            What became of the edges drawn again, and the couplers moved.

        Raises:
            ValueError: When a chain does not exist or the options do not fit it.
        """
        before = self.standing()
        for chain, options in target.items():
            if chain not in self.waypoints:
                msg = f"there is no feedline {chain}"
                raise ValueError(msg)
            if len(options) != len(self.waypoints[chain]):
                msg = f"feedline {chain} has {len(self.waypoints[chain])} waypoints, not {len(options)}"
                raise ValueError(msg)
        edges: list[dict[str, Any]] = []
        moved: list[int] = []
        try:
            for chain, options in target.items():
                self._put(chain, before[chain], options, edges, moved)
        except ValueError:
            self._apply(before)
            raise
        return edges, moved

    def _put(
        self, chain: int, now: list[int], options: list[int], edges: list[dict[str, Any]], moved: list[int]
    ) -> None:
        """Put the couplers of one chain that stand elsewhere on their options.

        Args:
            chain: The chain.
            now: The options the chain stands on.
            options: The options it is to stand on.
            edges: Where to add what became of the edges drawn again.
            moved: Where to add the couplers moved.
        """
        for coupler, standing, wanted in zip(self.waypoints[chain], now, options, strict=True):
            if coupler is not None and standing != wanted:
                edges += json.loads(self.session.set_option(coupler, wanted))["edges"]
                moved.append(coupler)

    def _moved(self, action: str, start: int, edges: list[dict[str, Any]], moved: list[int]) -> dict[str, Any]:
        """Check the moved chains locally and the chip as a whole, and analyse it anew.

        Args:
            action: What the move was.
            start: How many lines the session had said before the move.
            edges: What became of the edges drawn again.
            moved: The couplers moved.

        Returns:
            What the move came to: the action, the edges, the lines and the three verdicts.
        """
        said = [line for line in self.lines[start:] if "coupler session:" in line]
        standing = self.standing()
        chains = sorted({self.chain_of[c] for c in moved})
        local = [_verdicts(json.loads(self.session.local_check(c, standing[c]))) for c in chains]
        self.checked = {check["chain"]: check for check in local}
        self.verdict = self._check_chip()
        self.graph, analysed = self._analyse()
        self.last = {
            "action": action,
            "edges": edges,
            "lines": said + analysed,
            "verdicts": {"local": local, "global": self.verdict},
        }
        return self.last

    def moves(self, moves: list[tuple[int, int]], action: str | None = None) -> dict[str, Any]:
        """Put couplers on other options, keep them there, and check the chip anew.

        Args:
            moves: The couplers and their new options.
            action: What to call the move, or None for a name made from the moves.

        Returns:
            What the move came to.

        Raises:
            ValueError: When a coupler or an option does not exist; nothing moves then.
        """
        start = len(self.lines)
        before = self.standing()
        target = {chain: list(options) for chain, options in before.items()}
        for coupler, option in moves:
            if coupler not in self.chain_of:
                msg = f"there is no coupler {coupler}"
                raise ValueError(msg)
            chain = self.chain_of[coupler]
            target[chain][self.waypoints[chain].index(coupler)] = option
        edges, moved = self._apply({c: o for c, o in target.items() if o != before[c]})
        self.undo.append(before)
        named = ", ".join(f"coupler {c} on option {o}" for c, o in moves)
        return self._moved(action or named, start, edges, moved)

    def move(self, coupler: int, option: int) -> dict[str, Any]:
        """Put a coupler on another option and analyse the chip anew.

        Args:
            coupler: The coupler's index.
            option: The option's index.

        Returns:
            What became of the coupler's edges, what the analysis said and the three verdicts.
        """
        return self.moves([(coupler, option)])

    def mover(self, chain: int, options: list[int]) -> dict[str, Any]:
        """Put a feedline on options and keep it there.

        Args:
            chain: The feedline's chain.
            options: The option of every waypoint.

        Returns:
            What the move came to.
        """
        start = len(self.lines)
        before = self.standing()
        edges, moved = self._apply({chain: options})
        self.undo.append(before)
        return self._moved(f"feedline {chain} on options {pages.options_text(options)}", start, edges, moved)

    def back(self) -> dict[str, Any]:
        """Take the last move back.

        Returns:
            What taking it back came to.

        Raises:
            ValueError: When there is no move to take back.
        """
        if not self.undo:
            msg = "there is no move to take back"
            raise ValueError(msg)
        start = len(self.lines)
        edges, moved = self._apply(self.undo[-1])
        self.undo.pop()
        return self._moved("the last move taken back", start, edges, moved)

    def reset(self) -> dict[str, Any]:
        """Put every chain back on its committed options; the reset itself can be taken back.

        Returns:
            What the reset came to.
        """
        start = len(self.lines)
        before = self.standing()
        edges, moved = self._apply(self.committed)
        self.undo.append(before)
        return self._moved("every feedline back on the commit", start, edges, moved)

    def check_all(self) -> dict[str, Any]:
        """Check every feedline locally, as it stands.

        Returns:
            The last answer, with the local checks of every feedline.
        """
        standing = self.standing()
        local = [_verdicts(json.loads(self.session.local_check(c, standing[c]))) for c in sorted(standing)]
        self.checked = {check["chain"]: check for check in local}
        self.last = {
            **self.last,
            "action": "every feedline checked locally",
            "verdicts": {"local": local, "global": self.verdict},
        }
        return self.last

    def _chain_graph(self, request: Request) -> Answer | None:
        # A query with "bench" names a case of a SAT benchmark, which the bench routes answer; one
        # with "step" names a step check.
        if "chain" not in request.query or "bench" in request.query or "step" in request.query:
            return None
        chain = request.number("chain")
        options = request.numbers("options")
        check = request.query.get("check", "standing")
        if check not in {"standing", "alone"}:
            msg = f"there is no check {check}: standing or alone"
            raise ValueError(msg)
        local = json.loads(self.session.local_check(chain, options))
        judged = local[check]
        text = (
            f"feedline {chain} on options {pages.options_text(options)} · the {check} check: {judged['verdict']}"
            f" · lost: {_joined(judged.get('lost', []))}"
        )
        if local.get("undrawn"):
            text += f" · edges with no way: {_joined(local['undrawn'])}"
        tone = judged["verdict"]
        record = self.records.state(chain, options)
        if record is None:
            text += " · no chain-state record of these options"
        else:
            expected = pages.recorded_verdict(record, check)
            if expected is None:
                text += f" · the search did not run the {check} check on this state"
            elif expected != judged["verdict"]:
                text += f" · MISMATCH: the search recorded {expected}"
                tone = "mismatch"
            else:
                text += f" · matches the search's record ({expected})"
            standing = self.standing()
            off = [c for c, o in sorted(standing.items()) if c != chain and o != self.committed.get(c)]
            if off:
                named = ", ".join(map(str, off))
                stand = f"feedline {named} stands" if len(off) == 1 else f"feedlines {named} stand"
                text += f" · {stand} off the commit, which the search judged the state against"
        back = f'<a href="sat#chain-{chain}">local SAT</a> · <a href="global">global SAT</a> · '
        return Answer.page(page_text(judged["graph"], banner=back + html.escape(text), tone=tone))

    def _step_graph(self, request: Request) -> Answer | None:
        if "step" not in request.query or "bench" in request.query:
            return None
        step = request.number("step")
        chain = request.number("chain")
        options = request.numbers("options")
        checked = json.loads(self.session.step_check(chain, step, options))
        text = (
            f"the step check of feedline {chain} at edge f{checked['edge']}, step {step}, rebuilt by the session"
            f" · options {pages.options_text(checked['options'])} · {checked['verdict']}"
            f" · lost: {_joined(checked.get('lost', []))} · closed: {_joined(checked.get('closed', []))}"
        )
        if checked.get("undrawn"):
            text += f" · edges with no way: {_joined(checked['undrawn'])}"
        tone = checked["verdict"]
        record, count = self.records.step(chain, step, checked["options"])
        if record is None:
            text += " · no step-check record of this step on these options"
        else:
            first = f" (the first of {count} records)" if count > 1 else ""
            if record["verdict"] != checked["verdict"]:
                text += f" · MISMATCH: the chain search recorded {record['verdict']}{first}"
                tone = "mismatch"
            else:
                text += f" · matches the chain search's record ({record['verdict']}){first}"
        back = f'<a href="sat#chain-{chain}">local SAT</a> · <a href="global">global SAT</a> · '
        return Answer.page(page_text(checked["graph"], banner=back + html.escape(text), tone=tone))

    def _combination_graph(self, request: Request) -> Answer | None:
        if "combination" not in request.query:
            return None
        index = request.number("combination")
        event = self.records.combination(index)
        if event is None:
            msg = f"the global SAT search recorded no combination {index}"
            raise ValueError(msg)
        kept = self.records.kept()
        target: dict[int, list[int]] = {}
        for chain in self.waypoints:
            picked = event["states"][chain] if chain < len(event["states"]) else 0
            if picked == 0:
                target[chain] = self.committed[chain]
            elif chain not in kept or picked >= len(kept[chain]):
                msg = f"the search recorded no state {picked} of feedline {chain}"
                raise ValueError(msg)
            else:
                target[chain] = list(kept[chain][picked])
        before = self.standing()
        self._apply(target)
        try:
            checked = json.loads(self.session.insertion_check())
        finally:
            self._apply(before)
        text = (
            f"combination {index} ({str(event['stage']).rstrip(': ')}) · the whole-chip check: {checked['verdict']}"
            f" · lost: {_joined(checked.get('lost', []))} · edges over: {_joined(checked.get('over', []))}"
        )
        tone = checked["verdict"]
        if checked["verdict"] != event["verdict"]:
            text += f" · MISMATCH: the search recorded {event['verdict']}"
            tone = "mismatch"
        else:
            text += f" · matches the search's record ({event['verdict']})"
        back = f'<a href="global#combination-{index}">global SAT</a> · '
        return Answer.page(page_text(checked["graph"], banner=back + html.escape(text), tone=tone))

    def _mover(self, request: Request) -> Answer:
        chain = request.number("chain")
        self.mover(chain, request.numbers("options"))
        edges = next((c["edges"] for c in self._chains() if int(c["chain"]) == chain), [])
        first = next((e for e in edges if e), None)
        return Answer.redirect(f"./#feedline={first}&tab=couplers" if first else "./#tab=couplers")

    def _option(self, request: Request) -> Answer:
        asked = request.json()
        return Answer.data(self.move(int(asked["coupler"]), int(asked["option"])))

    def _options(self, request: Request) -> Answer:
        asked = request.json()
        moves = [(int(m["coupler"]), int(m["option"])) for m in asked["moves"]]
        if not moves:
            msg = "the moves are empty"
            raise ValueError(msg)
        return Answer.data(self.moves(moves))

    def handler(self) -> type[BaseHTTPRequestHandler]:
        """The request handler bound to this server.

        Returns:
            The handler class.
        """
        server = self

        class Handler(BaseHTTPRequestHandler):
            def _serve(self, method: str) -> None:
                split = urlsplit(self.path)
                body = b""
                if method == "POST":
                    body = self.rfile.read(int(self.headers.get("Content-Length", "0")))
                request = Request(method, split.path, dict(parse_qsl(split.query)), body)
                answer = server.answer(request)
                encoded = answer.body.encode("utf-8")
                self.send_response(answer.status)
                self.send_header("Content-Type", answer.kind)
                self.send_header("Content-Length", str(len(encoded)))
                if answer.location is not None:
                    self.send_header("Location", answer.location)
                self.end_headers()
                self.wfile.write(encoded)

            def do_GET(self) -> None:
                self._serve("GET")

            def do_POST(self) -> None:
                self._serve("POST")

            def log_request(self, code: int | str = "-", size: int | str = "-") -> None:
                # The stage's own lines are what the terminal is for; errors are still said.
                pass

        return Handler


def serve(server: CouplerServer, port: int) -> None:
    """Answer the page on ``http://127.0.0.1:<port>/`` until interrupted.

    The server answers one request at a time.

    Args:
        server: The coupler server.
        port: The local port.
    """
    httpd = HTTPServer(("127.0.0.1", port), server.handler())
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()
