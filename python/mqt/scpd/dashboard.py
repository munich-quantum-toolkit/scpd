#!/usr/bin/env python3
"""A table of what every wire did in every round of a pass, and why.

Reads a `plan --stage final -v 1 -d` log. Each attempt the sweep makes leaves
one line naming the wire, the round, the phase, the outcome and the picture it
drew, so the whole pass can be laid out as wires against rounds. A cell that
failed links to an interactive viewer of its own picture; one that succeeded
links to the picture itself.

    python3 tools_routing_dashboard.py <run.log> [--pass feedline] [-o out.html]
"""

from __future__ import annotations

import argparse
import html
import re
from pathlib import Path

from . import debugview

#: The marks a line carries. One attempt writes one line, and the pieces of
#: it are separated by a middle dot, so the line is read piece by piece rather
#: than by one pattern that has to hold all the shapes at once.
WIRE = re.compile(r"wire (\S+) · (.*)$")
FIG = re.compile(r"(\S+\.svg)\s*$")
ROUND = re.compile(r"round (\d+) \w+")
RELAX = re.compile(r"relax (\d+):")

#: How a cell is shown. The order is the order of severity.
LOOK = {
    "gave-up": ("#D55E00", "no way, not even after the relaxation"),
    "no-way": ("#E69F00", "this search found nothing"),
    "short": ("#CC79A7", "a way found, but not to be made its length"),
    "relaxed": ("#56B4E9", "only after a neighbour was let go of"),
    "found": ("#009E73", "a way found"),
    "kept": ("#cfd4d8", "kept its way, which holds"),
}


def classify(rest: str, relaxed: bool) -> str:
    """What an attempt came to, by the words its line carries."""
    if "keeps its way" in rest:
        return "kept"
    if "no way" in rest:
        return "no-way"
    if "no room for a meander" in rest or "too long for its target" in rest:
        return "short"
    if "found" in rest:
        return "relaxed" if relaxed else "found"
    return "kept"


def parse(log: Path, want: str) -> tuple[dict, list[str], int, dict]:
    cells: dict[tuple[str, int, int], dict] = {}
    order: list[str] = []
    rounds = 0
    inside = False
    at_round: dict[str, int] = {}
    kinds: dict[str, str] = {}
    fails: dict[str, list[str]] = {}
    # Every pass opens with the same line, so the one we want opens the block
    # and any other closes it. Without that the passes run into each other and
    # a table of the inner circuit holds the whole run.
    head = re.compile(r"([a-z]+(?: [a-z]+)*) routing: \d+ wires, up to (\d+) rounds")

    for raw in log.read_text(encoding="utf-8", errors="replace").splitlines():
        line = re.sub(r"^\[\w+\]\s+[\d.]+s\s+", "", raw).strip()
        if m := head.search(line):
            if m.group(1) == want:
                inside, rounds = True, int(m.group(2))
            else:
                inside = False
            continue
        if m := re.search(r"the ring the .* sweeps: (.*)", line):
            for token in m.group(1).split():
                name = re.sub(r"[\[(].*", "", token)
                kinds[name] = ("feedline" if "[" in token else
                               "resonator" if "(r)" in token else "plain")
            continue
        if m := re.search(r"final routing: unrouted: (.*)", line):
            for part in ("unrouted: " + m.group(1)).split(" · "):
                key, _, value = part.partition(": ")
                if value.strip() not in ("-", ""):
                    for wire in (v.strip() for v in value.split(",")):
                        fails.setdefault(wire, []).append(key)
            inside = False
            continue
        if not inside:
            continue
        found = WIRE.match(line)
        if not found:
            continue
        wire, rest = found.groups()
        fig = None
        if m := FIG.search(rest):
            fig = m.group(1)
            rest = rest[: m.start()].rstrip(" ·")
        if m := ROUND.search(rest):
            at_round[wire] = int(m.group(1))
        rnd = at_round.get(wire)
        if rnd is None or rnd >= rounds:
            continue
        if wire not in order:
            order.append(wire)
        # A round that gave up says so on a line of its own; it darkens the
        # cells of that round rather than making one.
        if "no way after" in rest:
            for slot in range(8):
                if (cell := cells.get((wire, rnd, slot))) and cell["state"] == "no-way":
                    cell["state"] = "gave-up"
            continue
        relax = RELAX.search(rest)
        slot = int(relax.group(1)) if relax else 0
        cells[(wire, rnd, slot)] = {
            "state": classify(rest, bool(relax)),
            "text": line,
            "fig": fig,
        }
    return cells, order, rounds, {"kinds": kinds, "fails": fails}


PAGE = """<!doctype html>
<meta charset="utf-8"><title>{title}</title>
<style>
 :root {{ color-scheme: light }}
 body {{ font:13px/1.4 ui-sans-serif,system-ui,sans-serif; margin:0; padding:18px 20px; color:#111 }}
 h1 {{ font-size:16px; margin:0 0 2px }} .sub {{ color:#555; margin:0 0 14px }}
 table {{ border-collapse:separate; border-spacing:1px; }}
 th {{ font-weight:600; font-size:11px; color:#444; padding:2px 4px; position:sticky; top:0; background:#fff }}
 th.r {{ border-bottom:2px solid #ddd }}
 td.w {{ font:11px ui-monospace,monospace; text-align:right; padding-right:7px; white-space:nowrap }}
 td.w.resonator {{ color:#0072B2; font-weight:600 }}
 td.w.feedline  {{ color:#D55E00; font-weight:600 }}
 a.c, span.c {{ display:block; width:17px; height:15px; border-radius:2px; background:#f4f4f4 }}
 a.c:hover {{ outline:2px solid #111 }}
 td.sum {{ font-size:11px; padding-left:10px; white-space:nowrap }}
 .badge {{ display:inline-block; padding:0 5px; border-radius:8px; color:#fff; margin-right:3px; font-size:10px }}
 .key {{ display:flex; gap:16px; flex-wrap:wrap; margin:0 0 14px; font-size:12px }}
 .key i {{ display:inline-block; width:13px; height:11px; border-radius:2px; margin-right:5px; vertical-align:-1px }}
 label.filt {{ font-size:12px; margin-left:14px }}
 .sep {{ width:7px }}
</style>
<h1>{title}</h1>
<p class="sub">{sub}</p>
<div class="key">{key}
  <label class="filt"><input type="checkbox" id="only"> only wires with fails</label>
</div>
{table}
<script>
document.getElementById('only').addEventListener('change', e => {{
  document.querySelectorAll('tr[data-fail="0"]').forEach(r =>
    r.style.display = e.target.checked ? 'none' : '');
}});
</script>
"""


def build(log: Path, want: str, out: Path, viewers: Path) -> None:
    cells, order, rounds, extra = parse(log, want)
    if not cells:
        raise SystemExit(f"no attempt of the {want} pass is in the log")
    kinds, fails = extra["kinds"], extra["fails"]
    slots = 1 + max((k[2] for k in cells), default=0)

    # A viewer only for what failed: a picture is as big as its figure, and
    # the ones worth opening are the ones that did not work.
    viewers.mkdir(parents=True, exist_ok=True)
    made: dict[str, str] = {}
    for cell in cells.values():
        fig = cell.get("fig")
        if not fig or cell["state"] in ("found", "kept"):
            continue
        source = Path(fig)
        if not source.exists():
            continue
        page = viewers / (source.stem + ".html")
        if not page.exists():
            debugview.build(source, page)
        made[fig] = page.relative_to(out.parent).as_posix()

    head = "<tr><th></th>"
    for rnd in range(rounds):
        head += f'<th class="r" colspan="{slots}">round {rnd}</th><th class="sep"></th>'
    head += "<th></th></tr><tr><th></th>"
    for _ in range(rounds):
        head += "<th>·</th>" + "".join(f"<th>{k}</th>" for k in range(1, slots))
        head += '<th class="sep"></th>'
    head += "<th></th></tr>"

    body = ""
    for wire in order:
        bad = fails.get(wire, [])
        body += f'<tr data-fail="{1 if bad else 0}">'
        body += f'<td class="w {kinds.get(wire,"plain")}">{html.escape(wire)}</td>'
        for rnd in range(rounds):
            for slot in range(slots):
                cell = cells.get((wire, rnd, slot))
                if not cell:
                    body += "<td><span class=\"c\"></span></td>"
                    continue
                colour = LOOK[cell["state"]][0]
                tip = html.escape(cell["text"])
                link = made.get(cell.get("fig") or "") or cell.get("fig")
                tag = (f'<a class="c" style="background:{colour}" href="{link}" '
                       f'target="_blank" title="{tip}"></a>') if link else (
                       f'<span class="c" style="background:{colour}" title="{tip}"></span>')
                body += f"<td>{tag}</td>"
            body += '<td class="sep"></td>'
        badges = "".join(
            f'<span class="badge" style="background:{LOOK["gave-up"][0]}">{html.escape(b)}</span>'
            for b in bad)
        body += f'<td class="sum">{badges}</td></tr>'

    key = "".join(f'<span><i style="background:{c}"></i>{html.escape(t)}</span>'
                  for c, t in LOOK.values())
    out.write_text(PAGE.format(
        title=f"{want} routing · {log.name}",
        sub=(f"{len(order)} wires · {rounds} rounds · {len(cells)} attempts · "
             f"{len(made)} failures to click. The '·' column is phase 1, "
             f"1–{slots-1} the relaxation levels. Blue is a resonator, orange a feedline edge."),
        key=key, table=f"<table>{head}{body}</table>"), encoding="utf-8")



if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("log", type=Path)
    ap.add_argument("--pass", dest="which", default="feedline")
    ap.add_argument("-o", "--output", type=Path)
    a = ap.parse_args()
    target = a.output or Path("routing-dashboard.html")
    build(a.log, a.which, target, target.parent / "viewers")
