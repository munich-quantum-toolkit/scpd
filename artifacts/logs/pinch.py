#!/usr/bin/env python3
"""Measure a pinch between two ring wires against the two feedline edges around it.

Usage: pinch.py <run dir> <phase> <px> <py> <edgeA> <edgeB> [<wire> ...]

Prints the nearest cell of each edge to the point, the index of that cell
from the edge's start and end, and the shape of the two edges near it.
"""
import math
import sys

from mqt.scpd.artifacts import read_artifact

run, phase, px, py, ea, eb = sys.argv[1:7]
px, py, ea, eb = int(px), int(py), int(ea), int(eb)
wires = [int(w) for w in sys.argv[7:]]
data = open(f"{run}/06-final.fb", "rb").read()
artifact = read_artifact(data)
routing = artifact.output
snap = None
for s in routing.phases or []:
    if (s.name or "") == phase:
        snap = s
if snap is None:
    print("phases:", [s.name for s in routing.phases or []]); sys.exit(1)

def cells(w):
    return [(c.x, c.y, c.heading) for c in (w.path or [])]

def nearest(path, p):
    best = (1e9, -1)
    for i, (x, y, h) in enumerate(path):
        d = math.hypot(x - p[0], y - p[1])
        if d < best[0]:
            best = (d, i)
    return best

fa = cells(snap.feedlines[ea]); fb = cells(snap.feedlines[eb])
print(f"f{ea}: {len(fa)} cells from {fa[0]} to {fa[-1]}")
print(f"f{eb}: {len(fb)} cells from {fb[0]} to {fb[-1]}")
da, ia = nearest(fa, (px, py)); db, ib = nearest(fb, (px, py))
print(f"point ({px},{py}): f{ea} nearest {da:.1f} at index {ia} ({len(fa)-1-ia} from its end) {fa[ia]}; f{eb} nearest {db:.1f} at index {ib} ({len(fb)-1-ib} from its end) {fb[ib]}")
# min distance between the two edges, excluding the last/first 22 cells (the pad runs)
best = (1e9, 0, 0)
for i in range(max(0, len(fa) - 200), len(fa) - 22):
    for j in range(22, min(len(fb), 300)):
        d = math.hypot(fa[i][0]-fb[j][0], fa[i][1]-fb[j][1])
        if d < best[0]:
            best = (d, i, j)
print(f"min distance between f{ea} (last 200 less 22) and f{eb} (first 300 less 22): {best[0]:.1f} at f{ea}[{best[1]}] {fa[best[1]]} ({len(fa)-1-best[1]} from end) / f{eb}[{best[2]}] {fb[best[2]]}")
print(f"f{ea} tail (every 5th of last 90):", fa[-90::5])
print(f"f{eb} head (every 5th of first 160):", fb[:160:5])
for w in wires:
    ww = cells(snap.wires[w])
    d, i = nearest(ww, (px, py))
    print(f"wire {w}: {len(ww)} cells, nearest {d:.1f} at index {i}; around:", ww[max(0,i-30):i+31:6])
