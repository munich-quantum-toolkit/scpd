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
a state. The server answers from the first second: while the session is still
being built, ``status.json`` says how far it is and the two dashboards fill in
as the records come.
"""

from __future__ import annotations

import html
import json
import re
import threading
import time
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

    def add_option(self, coupler: int, x: int, y: int, offset: int = -1, port: int = 0) -> str:
        """Build an option of a coupler by hand, centred on a cell; what came of it, as JSON."""
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

    def local_reroute(self, chain: int, options: list[int]) -> str:
        """The rip-up and reroute of one feedline's wires alone on options, which then go back, as JSON."""
        ...


#: How many of the last lines of the stage ``GET /status.json`` gives.
TAIL = 200

#: The record kinds a coupler server keeps of its session's run.
KEPT = frozenset({"chain-state", "step-check", "step-skip", "global-sat", "coupler-options"})


def _outcome() -> dict[str, Any]:
    return {"tried": 0, "checks": 0, "SAT": 0, "UNSAT": 0, "refused": 0, "lost": [], "closed": [], "skipped": {}}


def option_outcomes(steps: list[dict[str, Any]], skips: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    """What the chain search made of every coupler option, from its step records.

    A step ``S`` of a chain lays the edge from waypoint ``S`` to waypoint ``S + 1``, so a record of
    it counts for the option at each of the two waypoints. The step adds the option at waypoint
    ``S + 1`` to the prefix: that option is *tried* there.

    Args:
        steps: The ``step-check`` records.
        skips: The ``step-skip`` records: the steps the search dropped without a step check.

    Returns:
        By coupler and then by option (both as text, as JSON keys are): ``tried``, the number of
        steps that added the option; ``checks``, the number of step checks of its two edges;
        ``SAT`` and ``UNSAT``, how many checks came to that; ``refused``, how many checks the
        capacity rule refused; ``lost`` and ``closed``, the wires the UNSAT checks lost and the
        wires the refused checks closed; ``skipped``, how many steps the search dropped, by reason;
        and ``example``, one check each of ``SAT``, ``UNSAT`` and ``refused`` as ``chain``,
        ``step`` and ``options``, for a link to its graph.
    """
    found: dict[str, dict[str, Any]] = {}

    def ends(record: dict[str, Any]) -> list[tuple[dict[str, Any], bool]]:
        step = int(record["step"])
        couplers = record.get("couplers") or []
        options = record.get("options") or []
        picked = []
        for layer in (step, step + 1):
            if layer < len(couplers) and layer < len(options) and couplers[layer] is not None:
                per = found.setdefault(str(couplers[layer]), {})
                picked.append((per.setdefault(str(options[layer]), _outcome()), layer == step + 1))
        return picked

    for record in steps:
        verdict = "refused" if record.get("refused") else str(record.get("verdict"))
        for outcome, tried in ends(record):
            outcome["tried"] += int(tried)
            outcome["checks"] += 1
            outcome["UNSAT" if record.get("verdict") == "UNSAT" else "SAT"] += 1
            outcome["refused"] += int(bool(record.get("refused")))
            outcome["lost"] = sorted({*outcome["lost"], *record.get("lost", [])})
            if record.get("refused"):
                outcome["closed"] = sorted({*outcome["closed"], *record.get("closed", [])})
            example = outcome.setdefault("example", {})
            example.setdefault(
                verdict, {"chain": record["chain"], "step": record["step"], "options": record["options"]}
            )
    for record in skips:
        for outcome, tried in ends(record):
            outcome["tried"] += int(tried)
            why = str(record.get("why", "dropped"))
            outcome["skipped"][why] = outcome["skipped"].get(why, 0) + 1
    return found


class Recorder:
    """The debug callback of a session: keeps the records of some kinds in memory, by kind.

    A session that is still being built hands its records over on a thread of its own while the
    server reads them, so every access takes a lock.
    """

    def __init__(self, kinds: frozenset[str] = KEPT) -> None:
        """Keep the records of ``kinds`` and drop every other record.

        Args:
            kinds: The kinds to keep.
        """
        self.kinds = kinds
        self.records: dict[str, list[dict[str, Any]]] = {}
        self._lock = threading.Lock()

    def __call__(self, kind: str, record: str) -> None:
        """Keep one record when its kind is kept.

        Args:
            kind: The record's kind.
            record: The record as JSON text.
        """
        if kind in self.kinds:
            parsed = json.loads(record)
            with self._lock:
                self.records.setdefault(kind, []).append(parsed)

    def read(self, kind: str) -> list[dict[str, Any]]:
        """The records of one kind, in the order they came.

        Returns:
            A copy of the records; empty when none came.
        """
        with self._lock:
            return list(self.records.get(kind, []))

    def counts(self) -> dict[str, int]:
        """How many records of every kind came.

        Returns:
            The number of records, by kind, of the kinds of which records came.
        """
        with self._lock:
            return {kind: len(kept) for kind, kept in self.records.items()}


class StoppedError(Exception):
    """Raised through the Final stage to stop a session still being built when the server ends."""


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


class StageProgress:
    """What the Final stage of a session reports while it runs: its lines, its last step and its progress.

    Once the server ends, every line and every progress update raises :class:`StoppedError`, which
    ends the stage.

    The cli's reporter hands everything to this object and not to the server. The session holds
    the reporter, and the server holds the session: a reporter that held the server would make a
    cycle that the garbage collector cannot see, since it runs through the session's C++ object.

    Attributes:
        lines: Every line the stage said, in order.
        stopping: Set when the server ends: the next line or progress update raises :class:`StoppedError`.
    """

    def __init__(self, lines: list[str]) -> None:
        """Start with no step and no progress.

        Args:
            lines: The list to append the stage's lines to.
        """
        self.lines = lines
        self.stopping = threading.Event()
        self._lock = threading.Lock()
        self._phase = ""
        self._task: tuple[str, str, int, int] = ("", "", 0, 0)

    def _go_on(self) -> None:
        if self.stopping.is_set():
            raise StoppedError

    def line(self, text: str) -> None:
        """Keep a line the stage said.

        Args:
            text: The line.
        """
        self._go_on()
        self.lines.append(text)

    def entered(self, label: str) -> None:
        """Note the step the stage reported last.

        Args:
            label: The label of the step's entry.
        """
        self._go_on()
        with self._lock:
            self._phase = label

    def progress(self, task: str, detail: str, done: int, total: int) -> None:
        """Note how far the stage's task is.

        Args:
            task: The task, such as ``coupler insertion``.
            detail: What of the task runs, such as ``chain 2``.
            done: How many items are done.
            total: How many items there are, or 0 when unknown.
        """
        self._go_on()
        with self._lock:
            self._task = (task, detail, done, total)

    def snapshot(self) -> dict[str, Any]:
        """The last step and the last progress update.

        Returns:
            ``phase``, ``task``, ``detail``, ``done``, ``total`` and ``lines``, the last :data:`TAIL` lines.
        """
        with self._lock:
            task, detail, done, total = self._task
            phase = self._phase
        return {
            "phase": phase,
            "task": task,
            "detail": detail,
            "done": done,
            "total": total,
            "lines": self.lines[-TAIL:],
        }


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


def _chain_asked(request: Request) -> int | None:
    """The feedline a request names in ``?chain=``, or None when it names none.

    Returns:
        The feedline.
    """
    return request.number("chain") if "chain" in request.query else None


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
    the global SAT search. All three put the couplers back. ``GET /`` is the global mover: the
    capacity graph of the whole chip with the couplers of every feedline. ``GET /feedline?chain=C``
    is the local mover of one feedline: the graph of its local SAT check as the chip stands (the
    feedline alone, as the chain search judges it), with its couplers only.
    ``GET /mover?chain=C&options=a,b,c`` puts a feedline on the options, keeps it, and opens its
    local mover (``&view=global`` the global one). ``POST /option``, ``POST
    /options``, ``POST /undo`` and ``POST /reset`` move couplers; after every move the server checks
    the moved feedlines locally and the chip as a whole. ``POST /place`` builds an option of a
    coupler by hand, centred on a cell of the chip, and moves the coupler onto it.
    ``POST /reset?chain=C`` puts one feedline back on the commit. ``POST /check-all`` checks every
    feedline locally, ``?chain=C`` one.
    ``GET /state.json``, ``GET /graph.json`` and ``GET /couplers.json`` answer data.

    :meth:`add_route` adds a route: another module serves its own pages through the same session.
    :meth:`add_link` links such a page from the navigation of the SAT dashboards.

    The server can answer before its session stands: :meth:`build` builds the session on a thread
    of its own and :meth:`attach` hands it over. Meanwhile ``GET /status.json`` says how far the
    session is, and ``GET /sat`` and ``GET /global`` show the records so far under a status bar,
    with the graph and mover links off; the page fills in as the records come. Every other route
    answers ``503``: an HTML page that loads itself again when the session stands, or JSON with
    the error. The cli's reporter tells :attr:`stage` how far the stage is.
    """

    def __init__(self, session: Session | None, lines: list[str], records: Recorder | None = None) -> None:
        """Hold a session and analyse the chip as it stands, or wait for a session still being built.

        Args:
            session: The session that holds the Final stage, or None when :meth:`attach` hands it
                over later.
            lines: The list the session's progress callback appends to.
            records: The recorder the session hands its records to, or None when it records nothing.
        """
        self.session: Session | None = None
        self.lines = lines
        self.recorder = records
        self.records = pages.SatRecords()
        #: The coupler of every waypoint, by chain (None at a fixed waypoint).
        self.waypoints: dict[int, list[int | None]] = {}
        self.chain_of: dict[int, int] = {}
        #: The options of every chain as the commit left them: those of its committed
        #: ``chain-state`` record, or those it stood on when the session started.
        self.committed: dict[int, list[int]] = {}
        self.undo: list[dict[int, list[int]]] = []
        self.checked: dict[int, dict[str, Any]] = {}
        self.verdict: dict[str, Any] = {}
        self.graph: dict[str, Any] = {}
        self.last: dict[str, Any] = {}
        #: Why the session could not be built, or None.
        self.error: str | None = None
        self.began = time.monotonic()
        self.ended: float | None = None
        #: What the stage reports while it runs; the cli's reporter feeds it.
        self.stage = StageProgress(lines)
        self.builder: threading.Thread | None = None
        # The lock of the error and the end, apart from the request lock, which attach holds while
        # the stage still reports.
        self._progress = threading.Lock()
        self.lock = threading.Lock()
        self.routes: dict[tuple[str, str], list[Route]] = {}
        self.unbuilt: set[tuple[str, str]] = set()
        self.links: list[tuple[str, str, Callable[[], bool]]] = []
        for path in ("/", "/index.html"):
            self.add_route("GET", path, lambda _: Answer.page(self.page()))
        self.add_route("GET", "/graph.json", lambda _: Answer.data(self.graph))
        self.add_route("GET", "/couplers.json", lambda _: Answer(200, "application/json", self._session().couplers()))
        self.add_route("GET", "/state.json", lambda _: Answer.data(self.state()))
        self.add_route("GET", "/status.json", lambda _: Answer.data(self.status()), building=True)
        self.add_route("GET", "/sat", lambda _: Answer.page(self._dashboard(pages.local_sat_page)), building=True)
        self.add_route("GET", "/global", lambda _: Answer.page(self._dashboard(pages.global_sat_page)), building=True)
        self.add_route("GET", "/graph", self._step_graph)
        self.add_route("GET", "/graph", self._chain_graph)
        self.add_route("GET", "/graph", self._combination_graph)
        self.add_route("GET", "/mover", self._mover)
        self.add_route("GET", "/feedline", self._feedline)
        self.add_route("POST", "/option", self._option)
        self.add_route("POST", "/options", self._options)
        self.add_route("POST", "/place", self._place)
        self.add_route("POST", "/undo", lambda _: Answer.data(self.back()))
        self.add_route("POST", "/reset", lambda request: Answer.data(self.reset(_chain_asked(request))))
        self.add_route("POST", "/check-all", lambda request: Answer.data(self.check_all(_chain_asked(request))))
        self.add_route("POST", "/reroute", self._reroute)
        if session is not None:
            self.attach(session)

    def add_route(self, method: str, path: str, route: Route, *, building: bool = False) -> None:
        """Answer requests of a method and a path with a route.

        Several routes may share a method and a path: the server asks them in the order they were
        added, and the first that answers something other than None answers the request. A route
        that raises ``ValueError``, ``KeyError`` or ``TypeError`` answers ``400`` with the error.

        Args:
            method: ``GET`` or ``POST``.
            path: The path, without the query, as ``/graph``.
            route: The route.
            building: Whether the route answers while the session is still being built: it then
                must not touch the session. The server answers the other routes ``503`` meanwhile.
        """
        self.routes.setdefault((method, path), []).append(route)
        if building:
            self.unbuilt.add((method, path))

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
            if self.session is None and (request.method, request.path) not in self.unbuilt:
                return self._not_built(request)
            for route in routes:
                try:
                    found = route(request)
                except (ValueError, KeyError, TypeError) as error:
                    return Answer.data({"error": str(error)}, 400)
                if found is not None:
                    return found
        return Answer.data({"error": f"no route of {request.path} answers this query"}, 400)

    def _not_built(self, request: Request) -> Answer:
        status = self.status()
        if request.method == "POST" or request.path.endswith(".json"):
            said = f"the session could not be built: {self.error}" if self.error else "the session is still being built"
            return Answer.data({"error": said}, 503)
        return Answer(503, "text/html; charset=utf-8", pages.building_page(status))

    def _session(self) -> Session:
        if self.session is None:
            msg = "the session is still being built"
            raise ValueError(msg)
        return self.session

    def _dashboard(self, page: Callable[..., str]) -> str:
        if self.session is not None:
            return page(self.records, self.live())
        records = pages.SatRecords.of(self.recorder.read) if self.recorder is not None else pages.SatRecords()
        return page(records, None, self.status())

    def status(self) -> dict[str, Any]:
        """How far the session is: ``GET /status.json``.

        Returns:
            ``building``, true until the session stands or failed; ``failed`` and ``error``, why
            it failed; ``phase``, the label of the last step the stage reported; ``task``,
            ``detail``, ``done`` and ``total``, its last progress update; ``seconds`` since the
            server began to wait for it, until it stood; ``records``, the number of records so far
            by kind; and ``lines``, the last :data:`TAIL` lines the stage said.
        """
        counts = self.recorder.counts() if self.recorder is not None else {}
        stage = self.stage.snapshot()
        with self._progress:
            ended = self.ended if self.ended is not None else time.monotonic()
            return {
                "building": self.session is None and self.error is None,
                "failed": self.error is not None,
                "error": self.error,
                **{key: stage[key] for key in ("phase", "task", "detail", "done", "total")},
                "seconds": round(ended - self.began, 1),
                "records": counts,
                "lines": stage["lines"],
            }

    def attach(self, session: Session) -> None:
        """Hold a session and analyse the chip as it stands; the routes that need it answer then.

        Args:
            session: The session that holds the Final stage.
        """
        self.stage.progress("the first check of the chip as it stands", "", 0, 0)
        with self.lock:
            self.session = session
            try:
                self._analyse_session()
            except BaseException:
                self.session = None
                raise
        with self._progress:
            self.ended = time.monotonic()

    def _analyse_session(self) -> None:
        self.records = pages.SatRecords.of(self.recorder.read) if self.recorder is not None else pages.SatRecords()
        listed = self._chains()
        self.waypoints = {int(c["chain"]): list(c["couplers"]) for c in listed}
        self.chain_of = {
            coupler: chain for chain, couplers in self.waypoints.items() for coupler in couplers if coupler is not None
        }
        start = {int(c["chain"]): list(c["options"]) for c in listed}
        recorded = self.records.committed()
        self.committed = {chain: recorded.get(chain, options) for chain, options in start.items()}
        self.verdict = self._check_chip()
        self.graph, said = self._analyse()
        self.last = {
            "action": "the coupler insertion",
            "lines": said,
            "verdicts": {"local": [], "global": self.verdict},
        }

    def fail(self, message: str) -> None:
        """Note that the session could not be built; the pages then show why.

        Args:
            message: Why.
        """
        with self._progress:
            self.error = message
            self.ended = time.monotonic()

    def build(self, make: Callable[[], Session]) -> threading.Thread:
        """Build the session on a thread of its own and attach it, or note why it failed.

        Args:
            make: Builds the session. It reports through :attr:`stage`, so that :meth:`stop`
                can end it.

        Returns:
            The thread, a daemon thread.
        """

        def run() -> None:
            try:
                self.attach(make())
            except StoppedError:
                return
            except Exception as error:  # ruff: ignore[blind-except] - the pages show why the session failed, whatever it was
                self.fail(str(error) or type(error).__name__)

        self.builder = threading.Thread(target=run, name="coupler session", daemon=True)
        self.builder.start()
        return self.builder

    def stop(self, timeout: float = 10.0) -> bool:
        """End a session still being built at the next line or progress update of its stage.

        Args:
            timeout: How many seconds to wait for the stage to end.

        Returns:
            True when no build runs any more.
        """
        self.stage.stopping.set()
        if self.builder is None:
            return True
        self.builder.join(timeout)
        return not self.builder.is_alive()

    def _chains(self) -> list[dict[str, Any]]:
        return json.loads(self._session().chains())["chains"]

    def standing(self) -> dict[int, list[int]]:
        """The options every chain stands on.

        Returns:
            The option of every waypoint, by chain.
        """
        return {int(c["chain"]): list(c["options"]) for c in self._chains()}

    def _check_chip(self) -> dict[str, Any]:
        checked = json.loads(self._session().insertion_check())
        return {"verdict": checked["verdict"], "lost": checked.get("lost", []), "over": checked.get("over", [])}

    def _analyse(self) -> tuple[dict[str, Any], list[str]]:
        """Analyse the chip as it stands.

        Returns:
            The capacity graph, and the summary lines the analysis said.
        """
        start = len(self.lines)
        graph = json.loads(self._session().graph())
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
            "couplers": json.loads(self._session().couplers())["couplers"],
            "outcomes": option_outcomes(self.records.steps, self.records.skips),
            "searched": bool(self.records.steps or self.records.skips),
            "last": self.last,
            "live": True,
            "undo": len(self.undo),
            "chains": sorted(self.standing()),
            "scope": None,
        }
        return page_text(self.graph, session, banner=self._mover_banner(None), tone="note")

    def _mover_banner(self, chain: int | None) -> str:
        """The line above a mover page: which mover it is, and the links to the others.

        Args:
            chain: The feedline of a local mover, or None for the global one.

        Returns:
            The line as HTML.
        """
        movers = []
        for other in sorted(self.standing()):
            mark = ' class="here"' if other == chain else ""
            movers.append(f'<a href="feedline?chain={other}#tab=couplers"{mark}>{other}</a>')
        if chain is None:
            here = "<b>global mover · the whole chip</b>"
            back = '<a href="sat">local SAT</a>'
        else:
            here = f'<b>local mover · feedline {chain}</b> · <a href="./#tab=couplers">global mover</a>'
            back = f'<a href="sat#chain-{chain}">local SAT</a>'
        return f'{here} · local movers: {" ".join(movers)} · {back} · <a href="global">global SAT</a>'

    def _feedline(self, request: Request) -> Answer:
        """The local mover of one feedline: its local SAT check as the chip stands, with its couplers.

        The graph is the one the local SAT check builds of the feedline alone, as the chain search
        judges it (user, 2026-10-27): the feedline's own edges, its terminal edges as wires with
        their port runs, its wires and the free wires beside them, and no other feedline. The
        banner gives the global SAT search's check with the other feedlines standing as well. The
        coupler tab lists the feedline's couplers only. A move here moves
        those couplers like a move on the global mover, and the page shows the feedline anew.

        Returns:
            The page.

        Raises:
            ValueError: When the feedline does not exist.
        """
        chain = request.number("chain")
        standing = self.standing()
        if chain not in standing:
            msg = f"there is no feedline {chain}"
            raise ValueError(msg)
        options = standing[chain]
        local = json.loads(self._session().local_check(chain, options))
        judged, alone = local["standing"], local["alone"]
        session = {
            "couplers": [c for c in json.loads(self._session().couplers())["couplers"] if int(c["chain"]) == chain],
            "outcomes": option_outcomes(self.records.steps, self.records.skips),
            "searched": bool(self.records.steps or self.records.skips),
            "last": self.last,
            "live": True,
            "undo": len(self.undo),
            "chains": sorted(standing),
            "scope": {"chain": chain},
        }
        committed = options == self.committed.get(chain)
        text = (
            f" · on options {pages.options_text(options)}"
            f" ({'the commit' if committed else 'moved off the commit'})"
            f" · local SAT check (the feedline alone) {alone['verdict']}"
            + (f" (lost {_joined(alone['lost'])})" if alone.get("lost") else "")
            + f" · the other feedlines standing {judged['verdict']}"
            + (f" (lost {_joined(judged['lost'])})" if judged.get("lost") else "")
        )
        if local.get("undrawn"):
            text += f" · edges with no way: {_joined(local['undrawn'])}"
        banner = self._mover_banner(chain) + html.escape(text)
        return Answer.page(page_text(alone["graph"], session, banner=banner, tone=alone["verdict"]))

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
                edges += json.loads(self._session().set_option(coupler, wanted))["edges"]
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
        local = [_verdicts(json.loads(self._session().local_check(c, standing[c]))) for c in chains]
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

    def place(self, coupler: int, x: int, y: int, offset: int | None = None, port: int | None = None) -> dict[str, Any]:
        """Build an option of a coupler by hand, put the coupler on it and check the chip anew.

        The pad's centre takes the cell (``x``, ``y``) and the resonator is cut at the cell of its
        way nearest it (``add_option``). An option that fits stays among the coupler's options;
        ``POST /undo`` puts the coupler back on the option it stood on.

        Args:
            coupler: The coupler's index.
            x: The column of the cell.
            y: The row of the cell.
            offset: The offset (0 to 7), or None for the first that fits.
            port: The port (1 or 2), or None for the first that fits.

        Returns:
            What the move came to, with ``placed``: what ``add_option`` answered. When no option
            fits, nothing moves: the answer is the last one, with ``placed`` and the action.

        Raises:
            ValueError: When the coupler, the cell, the offset or the port does not exist.
        """
        if coupler not in self.chain_of:
            msg = f"there is no coupler {coupler}"
            raise ValueError(msg)
        placed = json.loads(
            self._session().add_option(coupler, x, y, -1 if offset is None else offset, 0 if port is None else port)
        )
        if not placed["built"]:
            refusals = placed.get("tried", [])
            return {
                **self.last,
                "action": f"no option of coupler {coupler} fits at ({x}, {y}): {len(refusals)} candidates refused",
                "placed": placed,
            }
        action = (
            f"coupler {coupler} on option {placed['option']}, placed by hand at ({x}, {y})"
            f" · offset {placed['offset']} · port {2 if placed['secondPort'] else 1}"
        )
        return {**self.moves([(coupler, int(placed["option"]))], action), "placed": placed}

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

    def reset(self, chain: int | None = None) -> dict[str, Any]:
        """Put every chain, or one, back on its committed options; the reset can be taken back.

        Args:
            chain: The one feedline to put back, or None for every one.

        Returns:
            What the reset came to.

        Raises:
            ValueError: When the feedline does not exist.
        """
        start = len(self.lines)
        before = self.standing()
        if chain is not None and chain not in self.committed:
            msg = f"there is no feedline {chain}"
            raise ValueError(msg)
        target = self.committed if chain is None else {chain: self.committed[chain]}
        edges, moved = self._apply(target)
        self.undo.append(before)
        action = "every feedline back on the commit" if chain is None else f"feedline {chain} back on the commit"
        return self._moved(action, start, edges, moved)

    def check_all(self, chain: int | None = None) -> dict[str, Any]:
        """Check every feedline, or one, locally, as it stands.

        Args:
            chain: The one feedline to check, or None for every one.

        Returns:
            The last answer, with the local checks.

        Raises:
            ValueError: When the feedline does not exist.
        """
        standing = self.standing()
        if chain is not None and chain not in standing:
            msg = f"there is no feedline {chain}"
            raise ValueError(msg)
        chains = sorted(standing) if chain is None else [chain]
        local = [_verdicts(json.loads(self._session().local_check(c, standing[c]))) for c in chains]
        self.checked = {check["chain"]: check for check in local}
        self.last = {
            **self.last,
            "action": "every feedline checked locally" if chain is None else f"feedline {chain} checked locally",
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
        local = json.loads(self._session().local_check(chain, options))
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
        checked = json.loads(self._session().step_check(chain, step, options))
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
            checked = json.loads(self._session().insertion_check())
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
        if request.query.get("view") != "global":
            return Answer.redirect(f"feedline?chain={chain}#tab=couplers")
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

    def _reroute(self, request: Request) -> Answer:
        """Rip up and reroute one feedline's wires alone on the options it stands on.

        ``POST /reroute?chain=<feedline>``: the wires of the feedline and the free wires beside it
        are routed again under the feedline constraints, every other wire ignored, as the local
        benchmark reroutes a feedline (user, 2026-10-27), and the chip is put back as it was. The
        options are those the feedline stands on, so a move made by hand is rerouted as it stands.

        Returns:
            What the session's ``local_reroute`` answered.

        Raises:
            ValueError: When the request names no feedline or one that does not exist.
        """
        chain = _chain_asked(request)
        standing = self.standing()
        if chain is None or chain not in standing:
            msg = f"there is no feedline {chain}" if chain is not None else "name a feedline: ?chain=<feedline>"
            raise ValueError(msg)
        return Answer(200, "application/json", self._session().local_reroute(chain, standing[chain]))

    def _place(self, request: Request) -> Answer:
        asked = request.json()
        offset = asked.get("offset")
        port = asked.get("port")
        return Answer.data(
            self.place(
                int(asked["coupler"]),
                int(asked["x"]),
                int(asked["y"]),
                None if offset is None else int(offset),
                None if port is None else int(port),
            )
        )

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


def serve(server: CouplerServer, port: int) -> bool:
    """Answer the page on ``http://127.0.0.1:<port>/`` until interrupted.

    The server answers one request at a time. When it ends, it stops a session still being built.

    Args:
        server: The coupler server.
        port: The local port.

    Returns:
        True when no build of the session runs any more; False when the stage did not reach a
        line or a progress update in time, and its thread still runs.
    """
    httpd = HTTPServer(("127.0.0.1", port), server.handler())
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()
    return server.stop()
