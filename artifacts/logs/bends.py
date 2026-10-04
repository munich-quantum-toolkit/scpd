#!/usr/bin/env python3
"""How soon each feedline edge bends after the pad, against the wires that must cross it.

Usage: bends.py <run dir> <log>

Reads the edges of the `couplers` phase and the `room R1` lines of the log
(which name the crossers of every edge between two couplers), and prints per
edge: the straight cells from its start to its first bend, the straight cells
from its last bend to its end, and the crossers. The stub at a coupler is the
pad run plus one, so a figure near that is an edge that bends as soon as it
may.
"""
import re, sys
from mqt.scpd.artifacts import read_artifact

run, log = sys.argv[1], sys.argv[2]
r = read_artifact(open(f"{run}/06-final.fb", "rb").read()).output
snap = [s for s in r.phases if (s.name or "") == "couplers"][0]
strip = re.compile(r"^\[final\] +[0-9.]+s +")
crossers = {}
for line in open(log):
    line = strip.sub("", line.strip())
    m = re.match(r"\[Coupler Insertion\] room R1 chain (\d+) edge (\d+)->(\d+) \(f(\d+)\): (\d+) plain wires? must cross it \(([^)]*)\)", line)
    if m:
        crossers[int(m.group(4))] = (int(m.group(1)), int(m.group(2)), int(m.group(5)), m.group(6))

def straights(path):
    cells = [(c.x, c.y, c.heading) for c in path]
    # cells listed twice at a bend: drop duplicates by place
    places = []
    for c in cells:
        if not places or (c[0], c[1]) != (places[-1][0], places[-1][1]):
            places.append(c)
    def step_along(a, b):
        dx, dy = b[0] - a[0], b[1] - a[1]
        v = {0: (0, -1), 1: (-1, -1), 2: (-1, 0), 3: (-1, 1), 4: (0, 1), 5: (1, 1), 6: (1, 0), 7: (1, -1)}[a[2]]
        return (dx, dy) == v
    head = 0
    while head + 1 < len(places) and step_along(places[head], places[head + 1]):
        head += 1
    tail = 0
    while tail + 1 < len(places) and step_along(places[-2 - tail], places[-1 - tail]) and places[-2 - tail][2] == places[-1][2]:
        tail += 1
    return head, tail, len(places)

rows = []
for f, (chain, at, m, ids) in sorted(crossers.items()):
    head, tail, n = straights(snap.feedlines[f].path or [])
    rows.append((f, chain, at, m, ids, head, tail, n))
print("edge chain at crossers | straight after its start (out of coupler at `at`) | straight before its end (into coupler at `at+1`) | cells")
for f, chain, at, m, ids, head, tail, n in rows:
    print(f"f{f:<3} {chain:<2} {at}->{at+1} m={m} ({ids}) | out {head:4} | in {tail:4} | {n}")
