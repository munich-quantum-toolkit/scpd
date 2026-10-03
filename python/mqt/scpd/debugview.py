#!/usr/bin/env python3
"""Wrap a Final-stage debug picture in a page whose layers can be switched off.

The pictures carry one class per thing on purpose — see `DEBUG_STYLE` in
`src/pipeline/FinalRouter.cpp` — so a layer is hidden by one CSS rule. This
adds the panel that writes those rules, and nothing else: the SVG goes in
untouched, so what the page shows is what the stage drew.

    python3 tools_debug_viewer.py <figure.svg> [-o out.html]
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

#: Every class the pictures use, grouped the way a reader thinks about them,
#: with the name the panel shows. A class the figure does not hold is left out
#: of the panel, so a search picture and an options picture each get their own.
GROUPS: list[tuple[str, list[tuple[str, str]]]] = [
    ("Walls — none of this is a price", [
        ("ob", "artwork, keepout and the ports' approach bands: closed"),
        ("co", "the band: what the search may enter"),
        ("fz", "the fence: the clearance of the fence wires, closed"),
        ("cbx", "where a coupler may sit"),
    ]),
    ("Static proximity — the obstacle halo", [
        (f"s{i}", f"level {i}" + (" (on the obstacle)" if i == 8 else "")) for i in range(8, 0, -1)
    ]),
    ("Wire proximity — the price", [
        (f"p{i}", f"level {i}" + (" (dearest)" if i == 8 else "")) for i in range(8, 0, -1)
    ]),
    ("The lane", [("lane", "the polygon: priced outside, free inside")]),
    ("Wires", [
        ("fd", "the way it found"),
        ("ow", "the way it had"),
        ("sd", "the Detail stage's seed"),
        ("nb", "ring neighbours"),
        ("rp", "let go of"),
        ("fw", "feedlines, drawn"),
        ("fwg", "a feedline with no way, never built"),
    ]),
    ("Ends and ports", [
        ("src", "source"), ("tgt", "target"), ("ar", "heading lines"),
        ("prt", "port"), ("prx", "port, exakte Lage"), ("pra", "port-Orientierung"),
        ("dst", "Abstand zum target"), ("prl", "port-Nummern"),
    ]),
    ("Couplers", [
        ("ocp", "gesetzter Couplers: Pad und Lead"), ("ocl", "its lead"),
        ("ocs", "dessen Feedline-port"), ("opt", "an option's pad"),
        ("opl", "an option's lead"), ("ops", "Option: port"),
    ]),
    ("Labels", [("lb", "text"), ("lg", "the legend box"), ("lgs", "swatch frames")]),
]

PAGE = """<!doctype html>
<meta charset="utf-8">
<title>{title}</title>
<style>
 :root {{ color-scheme: light }}
 body {{ margin:0; font:13px/1.45 ui-sans-serif,system-ui,sans-serif; color:#111;
         display:flex; height:100vh; overflow:hidden }}
 #panel {{ width:340px; flex:0 0 340px; overflow-y:auto; padding:14px 16px;
           border-right:1px solid #d8d8d8; background:#fafafa }}
 #stage {{ flex:1; overflow:auto; background:#fff }}
 #stage svg {{ display:block }}
 h1 {{ font-size:14px; margin:0 0 4px }}
 .sub {{ color:#555; margin:0 0 14px; font-size:12px }}
 fieldset {{ border:0; border-top:1px solid #e0e0e0; margin:0 0 10px; padding:10px 0 0 }}
 legend {{ font-weight:600; font-size:12px; padding:0 }}
 label {{ display:flex; gap:7px; align-items:flex-start; padding:2px 0; cursor:pointer }}
 label span {{ flex:1 }}
 .bulk {{ margin:4px 0 8px; display:flex; gap:6px }}
 button {{ font:inherit; padding:3px 9px; border:1px solid #bbb; background:#fff;
           border-radius:4px; cursor:pointer }}
 button:hover {{ background:#f0f0f0 }}
 #zoom {{ width:100% }}
 code {{ background:#eee; padding:0 3px; border-radius:3px }}
</style>
<div id="panel">
  <h1>{name}</h1>
  <p class="sub">{caption}</p>
  <div class="bulk"><button data-all="1">all on</button><button data-all="0">all off</button></div>
  <label><span>Zoom</span></label>
  <input id="zoom" type="range" min="400" max="6000" step="50" value="{width}">
  {fieldsets}
</div>
<div id="stage">{svg}</div>
<style id="hidden"></style>
<script>
const boxes = [...document.querySelectorAll('input[data-cls]')];
const sheet = document.getElementById('hidden');
function apply() {{
  sheet.textContent = boxes.filter(b => !b.checked)
      .map(b => '.' + b.dataset.cls + '{{display:none!important}}').join('');
}}
boxes.forEach(b => b.addEventListener('change', apply));
document.querySelectorAll('button[data-all]').forEach(b =>
  b.addEventListener('click', () => {{
    boxes.forEach(x => x.checked = b.dataset.all === '1'); apply();
  }}));
document.querySelectorAll('button[data-group]').forEach(b =>
  b.addEventListener('click', () => {{
    const on = b.dataset.state === '1';
    boxes.filter(x => x.dataset.group === b.dataset.group).forEach(x => x.checked = on);
    apply();
  }}));
const svg = document.querySelector('#stage svg');
document.getElementById('zoom').addEventListener('input', e => {{
  const w = +e.target.value, vb = svg.viewBox.baseVal;
  svg.setAttribute('width', w);
  svg.setAttribute('height', w * vb.height / vb.width);
}});
apply();
</script>
"""


def build(path: Path, out: Path) -> None:
    text = path.read_text(encoding="utf-8")
    present = set(re.findall(r'class="([A-Za-z0-9]+)"', text))
    # The page sizes the picture itself, so the file's own width must go.
    svg = re.sub(r'(<svg[^>]*?)\s+width="[^"]*"\s+height="[^"]*"', r"\1", text, count=1)
    width = 1400
    svg = svg.replace("<svg ", f'<svg width="{width}" ', 1)
    title = re.search(r'class="lb"[^>]*>([^<]{10,})', text)

    parts = []
    for group, entries in GROUPS:
        rows = [(cls, what) for cls, what in entries if cls in present]
        if not rows:
            continue
        key = re.sub(r"\W+", "", group)[:12]
        parts.append(
            f'<fieldset><legend>{group}</legend>'
            f'<div class="bulk">'
            f'<button data-group="{key}" data-state="1">on</button>'
            f'<button data-group="{key}" data-state="0">off</button></div>'
            + "".join(
                f'<label><input type="checkbox" checked data-cls="{cls}" '
                f'data-group="{key}"><span>{what} <code>{cls}</code></span></label>'
                for cls, what in rows
            )
            + "</fieldset>"
        )

    out.write_text(
        PAGE.format(
            title=path.name,
            name=path.name,
            caption=(title.group(1).strip() if title else "a picture of the Final stage"),
            width=width,
            fieldsets="".join(parts),
            svg=svg,
        ),
        encoding="utf-8",
    )



if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("svg", type=Path)
    ap.add_argument("-o", "--output", type=Path)
    a = ap.parse_args()
    build(a.svg, a.output or a.svg.with_suffix(".html"))
