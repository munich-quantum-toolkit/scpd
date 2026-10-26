# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""The SAT benchmarks: how well the capacity check of the coupler insertion predicts the feedline routing.

The **local** benchmark (:func:`local_cases`) has the coupler session search runs of options for
every segment of every chain, as the targeted repair does, and judge every feedline that a run may
touch twice: by the slot check of that feedline's window, and by a rip-up and re-route of its
wires. A case is one feedline of one run. The **global** benchmark (:func:`global_cases`) takes
whole-chip states, the chip as the insertion left it and states that move the couplers of one or
more chains onto runs that the local check called SAT, judges each by the whole-chip check and
routes it with the feedline routing in a forked child. A case is one state. Every case falls into
one of the four :data:`CLASSES`; ``docs/routing.md`` explains them.

:func:`page` lays a table of either benchmark out as one HTML page, in the look of the SAT
dashboards of :mod:`.pages`. :func:`add_routes` serves the pages from a live coupler session
(``mqt-scpd debug <run> --serve``), with links that put the chip on the state of a case and
answer the capacity graph of its check, or the graph with the wires drawn along the ways the
routing of the case laid. ``mqt-scpd sat-bench`` runs a benchmark on a run directory and writes
the table and the page under ``<run>/sat-bench``.
"""

from __future__ import annotations

import hashlib
import html
import json
import os
import random
import time
import traceback
from contextlib import contextmanager
from pathlib import Path
from typing import TYPE_CHECKING, Any, Protocol

from .. import pyscpd
from ..run import RunDirectory
from ..steps import STEPS
from . import pages
from .pages import _badge, _names, _page, _table, _tiles  # the helpers of the SAT dashboards
from .serve import Answer, Session, page_text

if TYPE_CHECKING:
    from collections.abc import Callable, Generator, Iterable, Sequence

    from ..flatbuffers.config.Config import ConfigT
    from .serve import CouplerServer, Request

#: The four classes of a case, in the order the core counts them.
CLASSES = ("satClean", "satFailing", "unsatClean", "unsatFailing")

#: What the pages call each class.
LABELS = {
    "satClean": "SAT and clean",
    "satFailing": "SAT but failing",
    "unsatClean": "UNSAT but clean",
    "unsatFailing": "UNSAT and failing",
}

#: The two benchmarks.
KINDS = ("local", "global")

#: The directory of a run that holds the tables and the pages of the benchmarks.
DIRECTORY = "sat-bench"

#: The runs of options per segment, the edges per segment, the whole-chip states beyond the
#: committed one and the seed of their choice, when nothing else is asked for.
RUNS_PER_SEGMENT = 8
SPAN = 3
CASES = 12
SEED = 2026


class BenchSession(Session, Protocol):
    """What the benchmarks need of a coupler session (``pyscpd.CouplerSession``) beyond the server's needs."""

    def compare_locally(self, runs: int, span: int) -> str:
        """Route the feedlines, then compare the slot check of every feedline with a rip-up, as JSON."""
        ...

    def route_feedlines(self) -> str:
        """Run the feedline routing on the chip as it stands, as JSON; it changes the state for good."""
        ...


def json_name(kind: str) -> str:
    """The file name of the table of a benchmark.

    Returns:
        ``sat-<kind>.json``.
    """
    return f"sat-{kind}.json"


def html_name(kind: str) -> str:
    """The file name of the page of a benchmark.

    Returns:
        ``sat-<kind>.html``.
    """
    return f"sat-{kind}.html"


def plan_to_the_insertion(config_path: Path, directory: Path) -> tuple[RunDirectory, ConfigT]:
    """Plan a chip through the Detail stage, so that a coupler session can start from its artifacts.

    Args:
        config_path: The ``config.toml`` of the chip.
        directory: The run directory to plan into.

    Returns:
        The run directory and its configuration.
    """
    run = RunDirectory(directory)
    config = run.prepare(config_path)
    chip_bytes = run.classified_chip(config)
    for step in STEPS[:-1]:
        run.run_stage(step, config, chip_bytes)
    return run, config


def fingerprint(directory: RunDirectory) -> str:
    """What the tables of a run depend on: its Detail artifact and its configuration.

    Returns:
        The first 16 hexadecimal digits of the SHA-256 of both files.
    """
    digest = hashlib.sha256(directory.artifact("detail").read_bytes())
    digest.update(directory.config.read_bytes())
    return digest.hexdigest()[:16]


def class_of(*, sat: bool, clean: bool) -> str:
    """The class of a case.

    Args:
        sat: Whether the check called the case SAT.
        clean: Whether the routing ended with no bad wire.

    Returns:
        One of :data:`CLASSES`.
    """
    return ("sat" if sat else "unsat") + ("Clean" if clean else "Failing")


def agreement(classes: dict[str, int]) -> float:
    """The share of the cases on which the check and the routing agree.

    Returns:
        ``(SAT and clean + UNSAT and failing) / all``, or 1.0 without a case.
    """
    total = sum(classes[name] for name in CLASSES)
    return 1.0 if total == 0 else (classes["satClean"] + classes["unsatFailing"]) / total


def summary(kind: str, name: str, table: dict[str, Any]) -> str:
    """The line that sums up one table.

    Args:
        kind: ``local`` or ``global``.
        name: The chip or the run.
        table: The table.

    Returns:
        The line.
    """
    classes = table["classes"]
    return (
        f"{kind} {name}: {table['cases']} cases · SAT and clean {classes['satClean']} · "
        f"SAT but failing {classes['satFailing']} · UNSAT but clean {classes['unsatClean']} · "
        f"UNSAT and failing {classes['unsatFailing']} · agreement {100 * agreement(classes):.1f} % · "
        f"{table['seconds']:.0f}s"
    )


def _quiet(_: str) -> None:
    pass


def _chosen(session: Session) -> dict[int, int]:
    return {int(c["index"]): int(c["chosen"]) for c in json.loads(session.couplers())["couplers"]}


#: The fields of a routing that name wires, which a table keeps.
ROUTED_LISTS = ("unrouted", "open", "crossing", "short", "long", "loops")


def _ways(routed: dict[str, Any]) -> dict[str, list[list[int]]]:
    return {str(w["name"]): w["path"] for w in routed.get("wires", [])}


def _compact(routed: dict[str, Any], base: dict[str, list[list[int]]]) -> dict[str, Any]:
    """A routing as a table keeps it: its bad count, the wires it names and the ways that differ from ``base``.

    Returns:
        ``bad``, the lists of :data:`ROUTED_LISTS` and ``ways``.
    """
    ways = {name: path for name, path in _ways(routed).items() if base.get(name) != path}
    return {"bad": int(routed["bad"]), **{key: list(routed.get(key, [])) for key in ROUTED_LISTS}, "ways": ways}


def ways_of(table: dict[str, Any], routed: dict[str, Any]) -> dict[str, list[list[int]]]:
    """Every wire's way in the routing of one case.

    Args:
        table: The table, whose ``ways`` hold the ways of its first case.
        routed: The case's ``routed`` entry, which keeps the ways that differ from those.

    Returns:
        The way of every wire, by name, as a list of the cells where it turns.
    """
    return {**table.get("ways", {}), **routed.get("ways", {})}


def local_cases(
    session: BenchSession, runs: int = RUNS_PER_SEGMENT, span: int = SPAN, say: Callable[[str], None] = _quiet
) -> dict[str, Any]:
    """Run the local benchmark on a session.

    The session routes its feedlines first (``compare_locally``), which changes its state for good;
    the comparison itself leaves the chip as it found it.

    Args:
        session: The coupler session.
        runs: The runs of options a segment contributes at most.
        span: The edges of a segment.
        say: Says a line of progress.

    Returns:
        The table: the four classes and the figures of the comparison; ``committed``, the option of
        every coupler before it; ``feedlines``, the edges of every chain; ``begun``; ``ways``, the
        ways of the first run's routing; and ``runs``, every run with its moves and its feedlines,
        each with its ``class``, and the routing it ended with.
    """
    began = time.monotonic()
    feedlines = [{"chain": int(c["chain"]), "edges": list(c["edges"])} for c in json.loads(session.chains())["chains"]]
    committed = _chosen(session)
    say(f"local: {runs} runs a segment of {span} edges; routing the feedlines and comparing every run ...")
    compared = json.loads(session.compare_locally(runs, span))
    classes = {name: int(compared["classes"][name]) for name in CLASSES}
    base: dict[str, list[list[int]]] | None = None
    for run in compared["runs"]:
        routed = run.pop("routed", {"bad": 0, "wires": []})
        run.pop("pads", None)
        if base is None:
            base = _ways(routed)
        run["routed"] = _compact(routed, base)
        for feedline in run["feedlines"]:
            feedline["class"] = class_of(sat=bool(feedline["sat"]), clean=bool(feedline["clean"]))
    return {
        "kind": "local",
        "runsPerSegment": runs,
        "span": span,
        "segments": compared["segments"],
        "cases": sum(classes.values()),
        "unjudged": compared["unjudged"],
        "classes": classes,
        "slotSeconds": compared["slotSeconds"],
        "rerouteSeconds": compared["rerouteSeconds"],
        "seconds": time.monotonic() - began,
        "committed": [{"coupler": c, "option": o} for c, o in sorted(committed.items())],
        "feedlines": feedlines,
        "begun": compared.get("begun", []),
        "ways": base or {},
        "runs": compared["runs"],
    }


def _cases_of(local: dict[str, Any], rng: random.Random, cases: int) -> list[list[tuple[int, int]]]:
    """Choose the moves of the global cases: runs the local slot check called SAT, one per chain at most.

    Returns:
        One list of (coupler, option) moves per case; the first case moves nothing.
    """
    by_chain: dict[int, list[list[tuple[int, int]]]] = {}
    for run in local["runs"]:
        if run["slot"]["verdict"] != "SAT" or not run["moves"]:
            continue
        by_chain.setdefault(int(run["chain"]), []).append([(int(m["coupler"]), int(m["option"])) for m in run["moves"]])
    chains = sorted(by_chain)
    chosen: list[list[tuple[int, int]]] = [[]]
    seen: set[tuple[tuple[int, int], ...]] = {()}
    tries = 0
    while len(chosen) < cases + 1 and chains and tries < 100 * cases:
        tries += 1
        picked = rng.sample(chains, rng.randint(1, len(chains)))
        moves = [move for chain in picked for move in rng.choice(by_chain[chain])]
        key = tuple(sorted(moves))
        if key not in seen:
            seen.add(key)
            chosen.append(moves)
    return chosen


def routed_in_child(session: BenchSession) -> dict[str, Any]:
    """Route the feedlines in a forked child, so that the session keeps its own state.

    Returns:
        What ``route_feedlines`` answered in the child.

    Raises:
        RuntimeError: When the platform cannot fork, or when the child failed; its traceback is on
            standard error.
    """
    if not hasattr(os, "fork"):
        msg = "the global SAT benchmark routes every case in a forked child, which this platform cannot make"
        raise RuntimeError(msg)
    reading, writing = os.pipe()
    pid = os.fork()
    if pid == 0:  # pragma: no cover - the child answers through the pipe and exits
        os.close(reading)
        code = 0
        try:
            with os.fdopen(writing, "wb") as answer:
                answer.write(session.route_feedlines().encode())
        except BaseException:  # ruff: ignore[blind-except]
            traceback.print_exc()
            code = 1
        finally:
            os._exit(code)
    os.close(writing)
    with os.fdopen(reading, "rb") as answer:
        text = answer.read()
    _, status = os.waitpid(pid, 0)
    if status != 0:
        msg = f"the routing child ended with status {status}; its traceback is above"
        raise RuntimeError(msg)
    return json.loads(text)


def global_cases(
    session: BenchSession,
    local: dict[str, Any],
    cases: int = CASES,
    *,
    seed: int = SEED,
    route_first: bool = False,
    say: Callable[[str], None] = _quiet,
    route: Callable[[BenchSession], dict[str, Any]] = routed_in_child,
) -> dict[str, Any]:
    """Run the global benchmark on a session.

    Every case puts the couplers on the committed options and then on its moves, takes the verdict
    of the whole-chip check (``insertion_check``) and routes the feedlines in a forked child. The
    chip goes back on the committed options at the end.

    Args:
        session: The coupler session, with its feedlines routed as ``compare_locally`` leaves them.
        local: The table of the local benchmark on the same run.
        cases: The states beyond the committed one.
        seed: The seed of the choice of the states.
        route_first: Route the feedlines of the session first, as ``compare_locally`` does; for a
            session that did not make ``local`` itself.
        say: Says a line of progress per case.
        route: Routes the feedlines of the chip as it stands and leaves the session as it was.

    Returns:
        The table: the four classes, ``committed``, ``ways`` (the ways of the first case's routing)
        and ``judged``, every case with its moves, the verdict, the lost wires and the edges over
        their capacity, the bad count, its class and its routing.
    """
    began = time.monotonic()
    if route_first:
        say("global: routing the feedlines, as the local benchmark does first ...")
        session.route_feedlines()
    committed = _chosen(session)
    classes = dict.fromkeys(CLASSES, 0)
    judged: list[dict[str, Any]] = []
    base: dict[str, list[list[int]]] | None = None
    # The cases are a fixed sample of the runs, not a secret.
    rng = random.Random(seed)  # ruff: ignore[suspicious-non-cryptographic-random-usage]
    chosen = _cases_of(local, rng, cases)
    for number, moves in enumerate(chosen):
        for coupler, option in committed.items():
            session.set_option(coupler, option)
        for coupler, option in moves:
            session.set_option(coupler, option)
        checked = json.loads(session.insertion_check())
        routed = route(session)
        if base is None:
            base = _ways(routed)
        bad = int(routed["bad"])
        name = class_of(sat=checked["verdict"] == "SAT", clean=bad == 0)
        classes[name] += 1
        judged.append({
            "case": number,
            "moves": [{"coupler": c, "option": o} for c, o in moves],
            "verdict": checked["verdict"],
            "lost": list(checked.get("lost", [])),
            "over": list(checked.get("over", [])),
            "bad": bad,
            "class": name,
            "routed": _compact(routed, base),
        })
        say(f"global case {number + 1}/{len(chosen)}: {len(moves)} moves · {checked['verdict']} · bad {bad}")
    for coupler, option in committed.items():
        session.set_option(coupler, option)
    return {
        "kind": "global",
        "runsPerSegment": local.get("runsPerSegment"),
        "span": local.get("span"),
        "seed": seed,
        "cases": len(judged),
        "classes": classes,
        "seconds": time.monotonic() - began,
        "committed": [{"coupler": c, "option": o} for c, o in sorted(committed.items())],
        "ways": base or {},
        "judged": judged,
    }


def read_table(path: Path) -> dict[str, Any] | None:
    """A table a benchmark wrote.

    Returns:
        The table, or None when the file is missing.
    """
    if not path.is_file():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def write_table(directory: Path, kind: str, table: dict[str, Any]) -> tuple[Path, Path]:
    """Write a table and its page, which stands alone.

    Args:
        directory: Where both go.
        kind: ``local`` or ``global``.
        table: The table.

    Returns:
        The JSON file and the HTML file.
    """
    directory.mkdir(parents=True, exist_ok=True)
    data = directory / json_name(kind)
    data.write_text(json.dumps(table, indent=1) + "\n", encoding="utf-8")
    shown = directory / html_name(kind)
    shown.write_text(page(kind, table, live=False), encoding="utf-8")
    return data, shown


def reusable(table: dict[str, Any] | None, runs: int, span: int, inputs: str) -> bool:
    """Whether a local table fits a global benchmark: the same runs, span and run inputs.

    Returns:
        True when the global benchmark can take its runs.
    """
    return (
        table is not None
        and table.get("kind") == "local"
        and table.get("runsPerSegment") == runs
        and table.get("span") == span
        and table.get("inputs") == inputs
    )


def run_bench(
    kind: str,
    session: BenchSession,
    output: Path,
    *,
    name: str,
    inputs: str,
    runs: int = RUNS_PER_SEGMENT,
    span: int = SPAN,
    cases: int = CASES,
    say: Callable[[str], None] = _quiet,
    route: Callable[[BenchSession], dict[str, Any]] = routed_in_child,
) -> dict[str, Any]:
    """Run one benchmark and write its table and page into ``output``.

    The global benchmark takes the local table in ``output`` when it was made with the same runs,
    span and inputs; otherwise it runs the local benchmark first and writes that table as well.

    Args:
        kind: ``local`` or ``global``.
        session: A fresh coupler session of the run.
        output: The directory of the tables and the pages.
        name: The name of the run, for the pages and the summary line.
        inputs: The :func:`fingerprint` of the run.
        runs: The runs of options a segment contributes at most.
        span: The edges of a segment.
        cases: The whole-chip states beyond the committed one.
        say: Says a line of progress.
        route: Routes the feedlines of a global case, as :func:`routed_in_child` does.

    Returns:
        The table.

    Raises:
        ValueError: When ``kind`` is no benchmark.
    """
    if kind not in KINDS:
        msg = f"there is no SAT benchmark {kind}: local or global"
        raise ValueError(msg)
    local = read_table(output / json_name("local"))
    fresh = kind == "local" or not reusable(local, runs, span, inputs)
    if fresh:
        local = {**local_cases(session, runs, span, say), "chip": name, "inputs": inputs}
        write_table(output, "local", local)
        say(summary("local", name, local))
    else:
        say(f"global: the runs of {output / json_name('local')}")
    if kind == "local":
        assert local is not None
        return local
    assert local is not None
    table = {
        **global_cases(session, local, cases, route_first=not fresh, say=say, route=route),
        "chip": name,
        "inputs": inputs,
    }
    write_table(output, "global", table)
    say(summary("global", name, table))
    return table


# The pages.

#: The look of the bench pages, on top of the SAT dashboards'.
STYLE = """
 .satClean { color: #009E73 } .satFailing { color: #D55E00 } .unsatClean { color: #B8860B }
 .unsatFailing { color: color-mix(in srgb, CanvasText 70%, transparent) }
 tr.satFailing td:first-child { box-shadow: inset 4px 0 #D55E00 }
 tr.unsatClean td:first-child { box-shadow: inset 4px 0 #E69F00 }
 .strip { display: flex; width: 260px; height: 12px; border-radius: 3px; overflow: hidden;
   background: color-mix(in srgb, CanvasText 10%, transparent) }
 .strip i { display: block; height: 100% }
 .strip i.satClean { background: #009E73 } .strip i.satFailing { background: #D55E00 }
 .strip i.unsatClean { background: #E69F00 }
 .strip i.unsatFailing { background: color-mix(in srgb, CanvasText 45%, transparent) }
 .legend i { display: inline-block; width: 10px; height: 10px; border-radius: 2px; margin: 0 4px 0 12px }
 .legend i.satClean { background: #009E73 } .legend i.satFailing { background: #D55E00 }
 .legend i.unsatClean { background: #E69F00 }
 .legend i.unsatFailing { background: color-mix(in srgb, CanvasText 45%, transparent) }
 .edges { font: 11px ui-monospace, monospace; opacity: .75 }
"""


def _nav(kind: str, *, live: bool) -> str:
    if live:
        links = (
            ("../", "capacity graph with the coupler tab"),
            ("../sat", "local SAT"),
            ("../global", "global SAT"),
            ("local", "local SAT bench"),
            ("global", "global SAT bench"),
        )
        note = ""
    else:
        links = ((html_name("local"), "local SAT bench"), (html_name("global"), "global SAT bench"))
        note = (
            f'<p class="note">The graph and routed links are off: {html.escape(pages.STATIC_NOTE)}; the session serves '
            f"this page at <code>/bench/{html.escape(kind)}</code>.</p>"
        )
    return '<nav class="links">' + "".join(f'<a href="{h}">{html.escape(t)}</a>' for h, t in links) + f"</nav>{note}"


def _link(href: str, text: str, title: str, *, live: bool) -> str:
    if not live:
        return f'<a class="off" title="{html.escape(pages.STATIC_NOTE)}">{html.escape(text)}</a>'
    return f'<a href="{html.escape(href)}" target="_blank" title="{html.escape(title)}">{html.escape(text)}</a>'


def _class_badge(name: str) -> str:
    return f'<span class="badge {html.escape(name)}">{html.escape(LABELS[name])}</span>'


def _class_tiles(table: dict[str, Any]) -> str:
    classes = table["classes"]
    tiles = [(str(classes[name]), LABELS[name], name) for name in CLASSES]
    tiles += [
        (f"{100 * agreement(classes):.1f} %", "agreement", ""),
        (str(table["cases"]), "cases", ""),
        (f"{table['seconds']:.0f}", "seconds", ""),
    ]
    return _tiles(tiles)


def _class_bar(counts: dict[str, int], placeholder: str) -> str:
    keys = [("all", "All"), *((name, LABELS[name]) for name in CLASSES)]
    buttons = "".join(
        f'<button type="button" data-filter="{key}">{html.escape(label)} ({counts.get(key, 0)})</button>'
        for key, label in keys
    )
    return (
        f'<div class="bar">{buttons}<input type="search" id="find" placeholder="{html.escape(placeholder)}">'
        '<span id="shown" class="sub"></span></div>'
    )


def _listed(names: Sequence[str]) -> str:
    return _names(list(names)) or "none"


def _from_to(table: dict[str, Any], moves: Iterable[dict[str, Any]]) -> list[tuple[int, int, int]]:
    committed = {int(m["coupler"]): int(m["option"]) for m in table.get("committed", [])}
    out = []
    for move in moves:
        coupler, option = int(move["coupler"]), int(move["option"])
        start = committed.get(coupler, option)
        if start != option:
            out.append((coupler, start, option))
    return out


def moves_text(table: dict[str, Any], moves: Iterable[dict[str, Any]]) -> str:
    """The moves of a case that leave the committed options, as the pages write them.

    Args:
        table: The table, whose ``committed`` gives the option every coupler starts on.
        moves: The case's moves, as ``{coupler, option}``.

    Returns:
        As ``c13: 0→1, c14: 2→0``, or ``—`` when no coupler leaves its committed option.
    """
    return ", ".join(f"c{c}: {a}→{b}" for c, a, b in _from_to(table, moves)) or "—"


def _rip(feedline: dict[str, Any]) -> str:
    rounds = len(feedline.get("rounds", []))
    said = "clean" if feedline["clean"] else f"failing: {feedline.get('failing') or '?'}"
    said += f" · {rounds} round{'' if rounds == 1 else 's'}"
    if feedline.get("reused"):
        said += " · as begun"
    title = f' title="{html.escape(feedline["blocked"])}"' if feedline.get("blocked") else ""
    tone = "good" if feedline["clean"] else "bad"
    return f'<span class="{tone}"{title}>{html.escape(said)}</span>'


def _local_rows(table: dict[str, Any], *, live: bool, items: bool) -> list[tuple[str, str]]:
    """The rows of the local table: one per feedline of a run, with its class.

    Returns:
        The class and the HTML of every row.
    """
    edges = {int(f["chain"]): f["edges"] for f in table.get("feedlines", [])}
    rows = []
    for run in table.get("runs", []):
        moves = moves_text(table, run["moves"])
        for feedline in run["feedlines"]:
            chain = int(feedline["chain"])
            name = feedline["class"]
            case = str(run["id"])
            query = f"bench=local&case={case}&chain={chain}"
            graph = _link(f"../graph?{query}", "graph ↗", f"the window slot check of feedline {chain}", live=live)
            routed = _link(f"../routed?{query}", "routed ↗", "the wires as the rip-up laid them", live=live)
            verdict = "SAT" if feedline["sat"] else "UNSAT"
            anchor = f"case-{case}-{chain}"
            first = (
                f'<a href="#{anchor}">feedline {chain}</a>' if not items else f"feedline {chain}"
            ) + f' <span class="edges">{html.escape(" ".join(edges.get(chain, [])))}</span>'
            cells = [
                first,
                html.escape(case),
                html.escape(str(run.get("segment", ""))),
                html.escape(moves),
                f"{_badge(verdict)} {_listed(feedline.get('lost', []))}",
                _rip(feedline),
                _class_badge(name),
                f"{float(feedline.get('seconds', 0.0)):.2f}",
                f"{graph} {routed}",
            ]
            text = " ".join([
                f"feedline {chain}",
                case,
                str(run.get("segment", "")),
                *edges.get(chain, []),
                moves,
                *feedline.get("lost", []),
                str(feedline.get("failing", "")),
                LABELS[name],
            ]).lower()
            body = "".join(f"<td{NUMERIC if at == 7 else ''}>{cell}</td>" for at, cell in enumerate(cells))
            if items:
                row = (
                    f'<tr class="item {name}" id="{anchor}" data-verdict="{name}" data-text="{html.escape(text)}">'
                    f"{body}</tr>"
                )
            else:
                row = f'<tr class="{name}">{body}</tr>'
            rows.append((name, row))
    return rows


LOCAL_HEAD = ("feedline", "run", "segment", "moves", "slot check · lost", "rip-up", "class", "s", "")


#: The class of a cell that holds a number.
NUMERIC = ' class="n"'


def _head(head: Sequence[str], numeric: set[int]) -> str:
    return "<tr>" + "".join(f"<th{NUMERIC if at in numeric else ''}>{h}</th>" for at, h in enumerate(head)) + "</tr>"


def _strip(counts: dict[str, int]) -> str:
    total = sum(counts.values()) or 1
    parts = "".join(
        f'<i class="{name}" style="width:{100 * counts[name] / total:.2f}%" title="{LABELS[name]}: {counts[name]}"></i>'
        for name in CLASSES
        if counts[name]
    )
    return f'<div class="strip">{parts}</div>'


def _legend() -> str:
    return '<p class="legend sub">' + "".join(f'<i class="{n}"></i>{html.escape(LABELS[n])}' for n in CLASSES) + "</p>"


def _per_feedline(table: dict[str, Any]) -> str:
    by_chain: dict[int, dict[str, int]] = {}
    for run in table.get("runs", []):
        for feedline in run["feedlines"]:
            counts = by_chain.setdefault(int(feedline["chain"]), dict.fromkeys(CLASSES, 0))
            counts[feedline["class"]] += 1
    if not by_chain:
        return ""
    rows = (
        [
            f"feedline {chain}",
            _strip(counts),
            f"{100 * agreement(counts):.0f} %",
            str(sum(counts.values())),
            str(counts["satFailing"]),
            str(counts["unsatClean"]),
        ]
        for chain, counts in sorted(by_chain.items())
    )
    head = ["feedline", "classes", "agreement", "cases", LABELS["satFailing"], LABELS["unsatClean"]]
    return "<h2>Agreement per feedline</h2>" + _legend() + _table(head, rows, {2, 3, 4, 5})


def _local_page(table: dict[str, Any], *, live: bool) -> str:
    title = f"local SAT bench · {table.get('chip', '')}".rstrip(" ·")
    order = {"satFailing": 0, "unsatClean": 1}
    mismatches = [
        row
        for _, row in sorted(
            ((name, row) for name, row in _local_rows(table, live=live, items=False) if name in order),
            key=lambda pair: order[pair[0]],
        )
    ]
    rows = _local_rows(table, live=live, items=True)
    counts = {"all": len(rows), **{name: sum(1 for n, _ in rows if n == name) for name in CLASSES}}
    parts = [_nav("local", live=live), _class_tiles(table)]
    begun = table.get("begun", [])
    if begun:
        said = "; ".join(
            f"feedline {b['chain']}: {'SAT' if b['sat'] else 'UNSAT'}, {'clean' if b['clean'] else 'failing'}"
            for b in begun
        )
        parts.append(f'<p class="facts">the chip as it began: {html.escape(said)}</p>')
    head = _head(LOCAL_HEAD, {7})
    if mismatches:
        parts.append(
            f'<h2>Mismatches ({len(mismatches)})</h2><div class="wide"><table>{head}{"".join(mismatches)}</table></div>'
        )
    else:
        parts.append("<h2>Mismatches</h2><p>none: the slot check and the rip-up agree on every case</p>")
    parts.extend((
        _per_feedline(table),
        "<h2>Every case</h2>"
        + _class_bar(counts, "feedline, run, edge, coupler or wire")
        + f'<div class="wide"><table>{head}{"".join(row for _, row in rows)}</table></div>',
    ))
    sub = (
        f"{table.get('runsPerSegment')} runs a segment of {table.get('span')} edges · {table.get('segments')} segments"
        f" · {table.get('unjudged', 0)} not judged · {float(table.get('slotSeconds', 0.0)):.0f} s in the slot check, "
        f"{float(table.get('rerouteSeconds', 0.0)):.0f} s rerouting · a case is one feedline of one run, judged by the "
        "slot check of its window and by a rip-up and re-route of its wires"
    )
    return _page(title, sub, "\n".join(parts), pages.SAT_SCRIPT, pages.SAT_STYLE + STYLE)


GLOBAL_HEAD = ("#", "moves", "whole-chip check", "lost · over", "bad", "class", "")


def _global_page(table: dict[str, Any], *, live: bool) -> str:
    title = f"global SAT bench · {table.get('chip', '')}".rstrip(" ·")
    rows = []
    judged = table.get("judged", [])
    for case in judged:
        number = int(case["case"])
        name = case["class"]
        moves = moves_text(table, case["moves"])
        query = f"bench=global&case={number}"
        graph = _link(f"../graph?{query}", "graph ↗", f"the whole-chip check of case {number}", live=live)
        routed = _link(f"../routed?{query}", "routed ↗", "the wires as the feedline routing laid them", live=live)
        lost = _listed(case.get("lost", []))
        over = _listed(case.get("over", []))
        bad = case.get("routed", {})
        failing = sorted({*bad.get("unrouted", []), *bad.get("open", []), *bad.get("crossing", [])})
        cells = [
            str(number),
            html.escape(moves if number else "the commit"),
            _badge(case["verdict"]),
            f"{lost} · {over}",
            f'<span title="{html.escape(", ".join(failing))}">{case["bad"]}</span>',
            _class_badge(name),
            f"{graph} {routed}",
        ]
        text = " ".join([str(number), moves, *case.get("lost", []), *case.get("over", []), *failing, LABELS[name]])
        body = "".join(f"<td{NUMERIC if at in {0, 4} else ''}>{cell}</td>" for at, cell in enumerate(cells))
        rows.append(
            f'<tr class="item {name}" id="case-{number}" data-verdict="{name}" data-text="{html.escape(text.lower())}">'
            f"{body}</tr>"
        )
    counts = {"all": len(judged), **{name: sum(1 for c in judged if c["class"] == name) for name in CLASSES}}
    parts = [
        _nav("global", live=live),
        _class_tiles(table),
        "<h2>Cases</h2>"
        + _class_bar(counts, "case, coupler or wire")
        + f'<div class="wide"><table>{_head(GLOBAL_HEAD, {0, 4})}{"".join(rows)}</table></div>',
    ]
    sub = (
        f"{table.get('cases')} whole-chip states from the runs of the local benchmark ({table.get('runsPerSegment')} "
        f"runs a segment of {table.get('span')} edges, seed {table.get('seed')}) · a case is judged by the whole-chip "
        "check of the insertion and routed by the feedline routing"
    )
    return _page(title, sub, "\n".join(parts), pages.SAT_SCRIPT, pages.SAT_STYLE + STYLE)


def page(kind: str, table: dict[str, Any], *, live: bool) -> str:
    """The page of a benchmark's table.

    Args:
        kind: ``local`` or ``global``.
        table: The table.
        live: Whether a coupler session serves the page; a page that stands alone shows its graph
            and routed links off.

    Returns:
        The page as HTML.

    Raises:
        ValueError: When ``kind`` is no benchmark.
    """
    if kind == "local":
        return _local_page(table, live=live)
    if kind == "global":
        return _global_page(table, live=live)
    msg = f"there is no SAT benchmark {kind}: local or global"
    raise ValueError(msg)


# The routes of a live session.


def window_check(
    graph: dict[str, Any], window: Iterable[str], slot_check: Callable[[str], str] = pyscpd.slot_check
) -> tuple[dict[str, Any], list[str]]:
    """The slot check of one feedline's window, as the local benchmark judges a feedline.

    The check routes the window's wires and every terminal edge alone, and a wire of the window
    that has no way or passes an edge over its capacity is lost.

    Args:
        graph: The capacity graph of the chip as it stands.
        window: The names of the wires of the window.
        slot_check: The slot check of a capacity graph, as ``pyscpd.slot_check``.

    Returns:
        What the slot check answered, and the lost wires of the window.
    """
    names = set(window)
    alone = {**graph, "wires": [w for w in graph["wires"] if w["name"] in names or w.get("terminal")]}
    checked = json.loads(slot_check(json.dumps(alone)))
    judged = checked.get("graph", {})
    crowded = {n for e in judged.get("edges", []) if e.get("overflow", 0) > 0 for n in e.get("through", [])}
    return checked, sorted((set(checked.get("lost", [])) | crowded) & names)


@contextmanager
def moved(session: Session, table: dict[str, Any], moves: Iterable[dict[str, Any]]) -> Generator[None]:
    """Put the chip on the state of a case for a block: the committed options of the table, then the moves.

    Every coupler goes back on the option it stood on before, also when the block raises.

    Args:
        session: The coupler session.
        table: The table, whose ``committed`` gives the option every coupler starts on.
        moves: The case's moves, as ``{coupler, option}``.

    Yields:
        Nothing; the chip stands on the case's state inside the block.
    """
    target = {int(m["coupler"]): int(m["option"]) for m in table.get("committed", [])}
    target.update({int(m["coupler"]): int(m["option"]) for m in moves})
    before = _chosen(session)
    changed = [c for c, o in target.items() if before.get(c, o) != o]
    try:
        for coupler in changed:
            session.set_option(coupler, target[coupler])
        yield
    finally:
        for coupler in reversed(changed):
            session.set_option(coupler, before[coupler])


def _missing(kind: str, path: Path) -> Answer:
    body = (
        f"<p>{html.escape(str(path))} does not exist. Run "
        f"<code>mqt-scpd sat-bench {html.escape(kind)} &lt;run&gt;</code> (or the SAT bench in ChipBuilder) first, "
        "then open this page again.</p>"
    )
    text = _page(f"{kind} SAT bench", "no table", body, "", pages.SAT_STYLE)
    return Answer(404, "text/html; charset=utf-8", text)


def _with_ways(graph: dict[str, Any], ways: dict[str, list[list[int]]]) -> tuple[dict[str, Any], int]:
    wires = []
    drawn = 0
    for wire in graph["wires"]:
        if wire["name"] in ways:
            wires.append({**wire, "path": ways[wire["name"]]})
            drawn += 1
        else:
            wires.append(wire)
    return {**graph, "wires": wires}, drawn


class Bench:
    """The routes of the bench pages behind a coupler server; :func:`add_routes` makes one."""

    def __init__(self, session: Session, directory: Path, slot_check: Callable[[str], str]) -> None:
        """Serve the tables of ``directory`` through ``session``.

        Args:
            session: The coupler session of the server.
            directory: The run's ``sat-bench`` directory.
            slot_check: The slot check of a capacity graph.
        """
        self.session = session
        self.directory = directory
        self.slot_check = slot_check
        self._cache: dict[str, tuple[float, dict[str, Any]]] = {}

    def table(self, kind: str) -> dict[str, Any] | None:
        """The table of a benchmark as the directory holds it now.

        Returns:
            The table, or None when the file is missing.

        Raises:
            ValueError: When ``kind`` is no benchmark.
        """
        if kind not in KINDS:
            msg = f"there is no SAT benchmark {kind}: local or global"
            raise ValueError(msg)
        path = self.directory / json_name(kind)
        if not path.is_file():
            return None
        stamp = path.stat().st_mtime
        cached = self._cache.get(kind)
        if cached is None or cached[0] != stamp:
            cached = (stamp, json.loads(path.read_text(encoding="utf-8")))
            self._cache[kind] = cached
        return cached[1]

    def _needed(self, kind: str) -> dict[str, Any]:
        table = self.table(kind)
        if table is None:
            msg = f"{self.directory / json_name(kind)} does not exist; run `mqt-scpd sat-bench {kind}` first"
            raise ValueError(msg)
        return table

    def page(self, kind: str) -> Answer:
        """The live page of a benchmark.

        Returns:
            The page, or a page that says how to make the table.
        """
        table = self.table(kind)
        if table is None:
            return _missing(kind, self.directory / json_name(kind))
        return Answer.page(page(kind, table, live=True))

    @staticmethod
    def _local_case(table: dict[str, Any], request: Request) -> tuple[dict[str, Any], dict[str, Any]]:
        case = request.query.get("case")
        chain = request.number("chain")
        run = next((r for r in table.get("runs", []) if str(r["id"]) == case), None)
        if run is None:
            msg = f"the local table has no run {case}"
            raise ValueError(msg)
        feedline = next((f for f in run["feedlines"] if int(f["chain"]) == chain), None)
        if feedline is None:
            msg = f"run {case} judged no feedline {chain}"
            raise ValueError(msg)
        return run, feedline

    @staticmethod
    def _global_case(table: dict[str, Any], request: Request) -> dict[str, Any]:
        number = request.number("case")
        case = next((c for c in table.get("judged", []) if int(c["case"]) == number), None)
        if case is None:
            msg = f"the global table has no case {number}"
            raise ValueError(msg)
        return case

    def graph(self, request: Request) -> Answer | None:
        """``GET /graph?bench=local&case=ID&chain=C`` or ``GET /graph?bench=global&case=K``.

        Puts the chip on the case's state, runs the check the benchmark ran, answers the capacity
        page of the graph it judged, and puts every coupler back.

        Returns:
            The page, or None when the query names no benchmark.
        """
        kind = request.query.get("bench")
        if kind is None:
            return None
        table = self._needed(kind)
        if kind == "local":
            return self._local_graph(table, request)
        return self._global_graph(table, request)

    def _local_graph(self, table: dict[str, Any], request: Request) -> Answer:
        run, feedline = self._local_case(table, request)
        chain = int(feedline["chain"])
        with moved(self.session, table, run["moves"]):
            checked, lost = window_check(json.loads(self.session.graph()), feedline["window"], self.slot_check)
        now = "UNSAT" if lost else "SAT"
        recorded = "SAT" if feedline["sat"] else "UNSAT"
        text = (
            f"run {run['id']} · feedline {chain} · moves {moves_text(table, run['moves'])} · the window slot check now:"
            f" {now} · lost: {', '.join(lost) or 'none'} · the benchmark: {recorded} · lost:"
            f" {', '.join(feedline.get('lost', [])) or 'none'}"
        )
        tone = now
        if now != recorded:
            text += " · MISMATCH with the benchmark"
            tone = "mismatch"
        text += " · rip-up: " + ("clean" if feedline["clean"] else f"failing {feedline.get('failing') or '?'}")
        text += f" ({LABELS[feedline['class']]})"
        note = (
            "The window slot check routes the feedline's window wires and every terminal edge alone, as the "
            "benchmark judged the feedline. The benchmark judged the chip after the feedline routing, with the "
            "run's edges as the segment search drew them; this page judges the chip after the insertion, with the "
            "edges drawn again for the moves."
        )
        back = f'<a href="bench/local#case-{html.escape(str(run["id"]))}-{chain}">local SAT bench</a> · '
        banner = back + html.escape(text) + f"<br><small>{html.escape(note)}</small>"
        return Answer.page(page_text(checked["graph"], banner=banner, tone=tone))

    def _global_graph(self, table: dict[str, Any], request: Request) -> Answer:
        case = self._global_case(table, request)
        number = int(case["case"])
        with moved(self.session, table, case["moves"]):
            checked = json.loads(self.session.insertion_check())
        text = (
            f"case {number} · moves {moves_text(table, case['moves'])} · the whole-chip check now: {checked['verdict']}"
            f" · lost: {', '.join(checked.get('lost', [])) or 'none'} · edges over:"
            f" {', '.join(checked.get('over', [])) or 'none'} · the benchmark: {case['verdict']}"
        )
        tone = checked["verdict"]
        if checked["verdict"] != case["verdict"]:
            text += " · MISMATCH with the benchmark"
            tone = "mismatch"
        text += f" · routed: bad {case['bad']} ({LABELS[case['class']]})"
        note = (
            "The benchmark checked the chip after the feedline routing; this page checks the chip after the insertion."
        )
        back = f'<a href="bench/global#case-{number}">global SAT bench</a> · '
        banner = back + html.escape(text) + f"<br><small>{html.escape(note)}</small>"
        return Answer.page(page_text(checked["graph"], banner=banner, tone=tone))

    def routed(self, request: Request) -> Answer | None:
        """``GET /routed?bench=local&case=ID&chain=C`` or ``GET /routed?bench=global&case=K``.

        Answers the capacity page of the case's state with every wire drawn along the way the
        routing of the case laid, and puts every coupler back.

        Returns:
            The page, or None when the query names no benchmark.
        """
        kind = request.query.get("bench")
        if kind is None:
            return None
        table = self._needed(kind)
        if kind == "local":
            run, feedline = self._local_case(table, request)
            chain = int(feedline["chain"])
            with moved(self.session, table, run["moves"]):
                graph = json.loads(self.session.graph())
            shown, drawn = _with_ways(graph, ways_of(table, run["routed"]))
            text = (
                f"run {run['id']} · feedline {chain} · wires as the rip-up laid them ({drawn} drawn) · rip-up of "
                f"feedline {chain}: "
                + ("clean" if feedline["clean"] else f"failing {feedline.get('failing') or '?'}")
                + f" · bad wires of the chip: {run['routed']['bad']}"
            )
            note = (
                f"The ways are those after the rip-up of feedline {run['chain']}, the run's own feedline, which the "
                "benchmark ripped up last; the wires outside its window keep the ways of the feedline routing."
            )
            if feedline.get("reused"):
                note += f" The result of feedline {chain} comes from the chip as it began."
            back = f'<a href="bench/local#case-{html.escape(str(run["id"]))}-{chain}">local SAT bench</a> · '
            tone = "SAT" if feedline["clean"] else "UNSAT"
        else:
            case = self._global_case(table, request)
            number = int(case["case"])
            with moved(self.session, table, case["moves"]):
                graph = json.loads(self.session.graph())
            shown, drawn = _with_ways(graph, ways_of(table, case["routed"]))
            text = (
                f"case {number} · wires as the feedline routing laid them ({drawn} drawn) · bad {case['bad']} "
                f"({LABELS[case['class']]})"
            )
            note = "The graph is the capacity graph of the case's state after the insertion."
            back = f'<a href="bench/global#case-{number}">global SAT bench</a> · '
            tone = "SAT" if case["bad"] == 0 else "UNSAT"
        banner = back + html.escape(text) + f"<br><small>{html.escape(note)}</small>"
        return Answer.page(page_text(shown, banner=banner, tone=tone))


def add_routes(server: CouplerServer, directory: Path, slot_check: Callable[[str], str] = pyscpd.slot_check) -> Bench:
    """Serve the bench pages of a run's ``sat-bench`` directory through a coupler server.

    ``GET /bench/local`` and ``GET /bench/global`` answer the pages with live links;
    ``GET /graph?bench=...`` and ``GET /routed?bench=...`` answer the graph of a case. The SAT
    dashboards link the bench pages whose tables exist.

    Args:
        server: The coupler server.
        directory: The run's ``sat-bench`` directory; its tables are read on every request.
        slot_check: The slot check of a capacity graph, as ``pyscpd.slot_check``.

    Returns:
        The routes.
    """
    bench = Bench(server.session, Path(directory), slot_check)
    for kind in KINDS:
        server.add_route("GET", f"/bench/{kind}", lambda _, kind=kind: bench.page(kind))
        server.add_link(
            f"bench/{kind}", f"{kind} SAT bench", lambda kind=kind: (bench.directory / json_name(kind)).is_file()
        )
    server.add_route("GET", "/graph", bench.graph)
    server.add_route("GET", "/routed", bench.routed)
    return bench
