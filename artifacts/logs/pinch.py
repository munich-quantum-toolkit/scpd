#!/usr/bin/env python3
# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Measure a pinch between two ring wires against the two feedline edges around it.

Usage: pinch.py <run dir> <phase> <px> <py> <edgeA> <edgeB> [<wire> ...]

Prints the nearest cell of each edge to the point, the index of that cell
from the edge's start and end, and the shape of the two edges near it.
"""

import math
import pathlib
import sys

from mqt.scpd.artifacts import read_artifact

run, phase, px, py, ea, eb = sys.argv[1:7]
px, py, ea, eb = int(px), int(py), int(ea), int(eb)
wires = [int(w) for w in sys.argv[7:]]
data = pathlib.Path(f"{run}/06-final.fb").open("rb").read()
artifact = read_artifact(data)
routing = artifact.output
snap = None
for s in routing.phases or []:
    if (s.name or "") == phase:
        snap = s
if snap is None:
    sys.exit(1)


def cells(w):
    return [(c.x, c.y, c.heading) for c in (w.path or [])]


def nearest(path, p):
    best = (1e9, -1)
    for i, (x, y, _h) in enumerate(path):
        d = math.hypot(x - p[0], y - p[1])
        if d < best[0]:
            best = (d, i)
    return best


fa = cells(snap.feedlines[ea])
fb = cells(snap.feedlines[eb])
da, ia = nearest(fa, (px, py))
db, ib = nearest(fb, (px, py))
# min distance between the two edges, excluding the last/first 22 cells (the pad runs)
best = (1e9, 0, 0)
for i in range(max(0, len(fa) - 200), len(fa) - 22):
    for j in range(22, min(len(fb), 300)):
        d = math.hypot(fa[i][0] - fb[j][0], fa[i][1] - fb[j][1])
        if d < best[0]:
            best = (d, i, j)
for w in wires:
    ww = cells(snap.wires[w])
    d, i = nearest(ww, (px, py))
