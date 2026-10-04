#!/usr/bin/env python3
"""One row per chip of an arm: the outer routing's verdict, the feedline angle, the
length-point line, and bad after the feedline routing — the figures the length-point
clearance is judged by. Lengths are not read.

Usage: arm-table.py <arm> [<arm> ...]
"""
import os
import re
import sys

STRIP = re.compile(r"^\[final\] +[0-9.]+s  ")
CHIPS = ["4q", "9q", "17q", "21q", "33q", "45q", "57q", "69q"]


def read(arm, chip):
    path = f"artifacts/logs/{arm}/{chip}.log"
    if not os.path.exists(path):
        return None
    lines = [STRIP.sub("", l.rstrip("\n")) for l in open(path)]
    out = {}
    outer = [l for l in lines if l.startswith("outer routing:") and " drawn, " in l]
    m = re.search(r"(\d+) unrouted, (\d+) open", outer[0]) if outer else None
    out["outer"] = f"{m.group(1)}/{m.group(2)}" if m else "?"
    arrow = [l for l in lines if l.startswith("==> coupler insertion")]
    m = re.search(r"(\d+) feedline edges? NOT drawn, feedline angle cost (\d+)", arrow[0]) if arrow else None
    out["notdrawn"], out["angle"] = (m.group(1), m.group(2)) if m else ("?", "?")
    lenpt = [l for l in lines if l.startswith("outer routing: LENPT")]
    m = re.search(r"k=(\d+).*?min ([0-9.-]+) mean ([0-9.-]+) cells, (\d+) below k", lenpt[0]) if lenpt else None
    out["lenpt"] = f"k {m.group(1)}: min {float(m.group(2)):.0f} mean {float(m.group(3)):.0f} below {m.group(4)}" if m else "–"
    final = [l for l in lines if l.startswith("final routing:") and " drawn, " in l]
    m = re.search(r"(\d+) unrouted.*?(\d+) open.*?(\d+) crossing", final[-1]) if final else None
    if m:
        u, o, c = (int(g) for g in m.groups())
        out["bad"] = u + o + c
        out["final"] = f"{o}/{c} ({u + o + c})"
    else:
        out["bad"] = None
        out["final"] = "?"
    real = [l for l in lines if l.startswith("real ")]
    out["real"] = float(real[-1].split()[1]) if real else 0.0
    return out


for arm in sys.argv[1:]:
    print(f"===== {arm}")
    print("| chip | outer unrouted/open | edges not drawn | angle | length points | open/crossing (bad) | s |")
    print("|---|---|---|---|---|---|---|")
    total = 0
    secs = 0.0
    for chip in CHIPS:
        r = read(arm, chip)
        if r is None:
            print(f"| {chip} | – | | | | | |")
            continue
        total += r["bad"] or 0
        secs += r["real"]
        print(f"| {chip} | {r['outer']} | {r['notdrawn']} | {r['angle']} | {r['lenpt']} | {r['final']} | {r['real']:.0f} |")
    print(f"| sum | | | | | **{total}** | {secs:.0f} |")
