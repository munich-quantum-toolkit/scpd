# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Lay the capacity graph of the coupler insertion out on a page of its own.

With ``SCPD_BOTTLENECKS=1`` and ``--debug`` the Final stage writes
``final-capacity-graph.json`` next to its pictures: the chambers and walls as a
raster, every edge of the graph with what it takes and what the flow sends
through it, and every outer wire with its two ports. This turns that file into
one HTML page with no outside resources, from the template
``capacityview.html`` beside this module. The page draws the chambers and
walls, the nodes and edges of the graph and a ``load/capacity`` badge on every
edge, marks the target ports, and lists the edges, the feedline edges and the
wires in tables. A click on an edge, a chamber, a feedline or a wire shows only
the parts of the graph that belong to it; the address can name one, as in
``#wire=14``, ``#edge=g20``, ``#chamber=c12`` or ``#feedline=f2``.

    python3 -m mqt.scpd.capacityview <final-capacity-graph.json> [-o out.html]

``serve`` puts the same page behind a local server that holds the Final stage
after its coupler insertion (``mqt-scpd couplers``): a tab of the page then
moves a coupler to another of its options, the server draws its two feedline
edges again and analyses the chip anew, and the page shows the new graph.
"""

from __future__ import annotations

import argparse
import html
import json
import re
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from typing import Any, Protocol

#: The name the Final stage gives the data, and the page built from it.
DATA_NAME = "final-capacity-graph.json"
PAGE_NAME = "final-capacity-graph.html"

TEMPLATE = Path(__file__).with_name("capacityview.html")
TITLE = re.compile(r"<title>.*?</title>", re.DOTALL)
DATA = re.compile(r'(<script id="graph-data" type="application/json">).*?(</script>)', re.DOTALL)
SESSION = re.compile(r'(<script id="session-data" type="application/json">).*?(</script>)', re.DOTALL)


def _embedded(value: object) -> str:
    # JSON has no "<" outside its strings, so writing every one as an escape
    # keeps the data from ending the script element it sits in.
    return json.dumps(value, separators=(",", ":")).replace("<", "\\u003c")


def page_text(graph: dict[str, Any], session: dict[str, Any] | None = None) -> str:
    """The page for one capacity graph.

    Args:
        graph: The capacity graph, as the Final stage writes it.
        session: What the coupler tab needs when a server holds the stage: the
            couplers and their options, and what the last change said. ``None``
            for a page that stands alone.

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
    return SESSION.sub(lambda found: found.group(1) + _embedded(session) + found.group(2), text, count=1)


def build(data: Path, page: Path) -> None:
    """Write the page for one capacity graph.

    Args:
        data: The ``final-capacity-graph.json`` the Final stage wrote.
        page: Where the page goes.

    Raises:
        ValueError: When the data is not a capacity graph.
    """
    try:
        text = page_text(json.loads(data.read_text(encoding="utf-8")))
    except ValueError as error:
        msg = f"{data}: {error}"
        raise ValueError(msg) from error
    page.write_text(text, encoding="utf-8")


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


class CouplerServer:
    """A coupler session behind a local web server.

    ``GET /`` answers the page of the capacity graph as the chip stands, with
    the coupler tab. ``POST /option`` with ``{"coupler": c, "option": o}``
    puts coupler ``c`` on option ``o``, draws its two feedline edges again
    and analyses the chip anew; it answers what became of the edges and
    every line the stage said meanwhile. ``GET /graph.json`` and
    ``GET /couplers.json`` answer the data alone.
    """

    def __init__(self, session: Session, lines: list[str]) -> None:
        """Hold a session and analyse the chip as it stands.

        Args:
            session: The session that holds the Final stage.
            lines: The list the session's progress callback appends to.
        """
        self.session = session
        self.lines = lines
        self.graph, said = self._analyse()
        self.last: dict[str, Any] = {"action": "the coupler insertion", "lines": said}

    def _analyse(self) -> tuple[dict[str, Any], list[str]]:
        """Analyse the chip as it stands.

        Returns:
            The capacity graph, and the summary lines the analysis said.
        """
        start = len(self.lines)
        graph = json.loads(self.session.graph())
        said = [line for line in self.lines[start:] if "CAPACITY GRAPH" in line]
        return graph, said

    def page(self) -> str:
        """The page as the chip stands.

        Returns:
            The page as HTML.
        """
        session = {"couplers": json.loads(self.session.couplers())["couplers"], "last": self.last}
        return page_text(self.graph, session)

    def move(self, coupler: int, option: int) -> dict[str, Any]:
        """Put a coupler on another option and analyse the chip anew.

        Args:
            coupler: The coupler's index.
            option: The option's index.

        Returns:
            What became of the coupler's edges and what the analysis said.
        """
        start = len(self.lines)
        edges = json.loads(self.session.set_option(coupler, option))
        moved = self.lines[start:]
        self.graph, said = self._analyse()
        self.last = {
            "action": f"coupler {coupler} on option {option}",
            "edges": edges["edges"],
            "lines": [line for line in moved if "[Coupler Session]" in line] + said,
        }
        return self.last

    def handler(self) -> type[BaseHTTPRequestHandler]:
        """The request handler bound to this server.

        Returns:
            The handler class.
        """
        server = self

        class Handler(BaseHTTPRequestHandler):
            def _answer(self, status: int, kind: str, body: str) -> None:
                encoded = body.encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", kind)
                self.send_header("Content-Length", str(len(encoded)))
                self.end_headers()
                self.wfile.write(encoded)

            def do_GET(self) -> None:
                if self.path.split("#")[0] in {"/", "/index.html"}:
                    self._answer(200, "text/html; charset=utf-8", server.page())
                elif self.path == "/graph.json":
                    self._answer(200, "application/json", json.dumps(server.graph))
                elif self.path == "/couplers.json":
                    self._answer(200, "application/json", server.session.couplers())
                else:
                    self._answer(404, "text/plain", "not found")

            def do_POST(self) -> None:
                if self.path != "/option":
                    self._answer(404, "text/plain", "not found")
                    return
                try:
                    asked = json.loads(self.rfile.read(int(self.headers.get("Content-Length", "0"))))
                    answer = server.move(int(asked["coupler"]), int(asked["option"]))
                except (ValueError, KeyError, TypeError) as error:
                    self._answer(400, "application/json", json.dumps({"error": str(error)}))
                    return
                self._answer(200, "application/json", json.dumps(answer))

            def log_request(self, code: int | str = "-", size: int | str = "-") -> None:
                # The stage's own lines are what the terminal is for; errors are still said.
                pass

        return Handler


def serve(server: CouplerServer, port: int) -> None:
    """Answer the page on ``http://127.0.0.1:<port>/`` until interrupted.

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


def main(argv: list[str] | None = None) -> int:
    """Build the page from the command line.

    Returns:
        The exit code.
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("data", type=Path, help=f"the {DATA_NAME} of a debug run")
    parser.add_argument("-o", "--output", type=Path, help=f"the page; {PAGE_NAME} beside the data by default")
    args = parser.parse_args(argv)
    build(args.data, args.output or args.data.with_name(PAGE_NAME))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
