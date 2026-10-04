#!/usr/bin/env python3
"""Pairs of feedline edges that share a cell, in the `couplers` phase.

Usage: fcross.py <run dir>
"""
import sys
from mqt.scpd.artifacts import read_artifact
r = read_artifact(open(f"{sys.argv[1]}/06-final.fb", "rb").read()).output
snap = [s for s in r.phases if (s.name or "") == "couplers"][0]
chain_of = {i: e.chain for i, e in enumerate(r.feedlineEdges or [])}
cells = [set((c.x, c.y) for c in (w.path or [])) for w in snap.feedlines]
pairs = []
for i in range(len(cells)):
    for j in range(i + 1, len(cells)):
        common = cells[i] & cells[j]
        if common:
            pairs.append((i, j, len(common), sorted(common)[:3]))
print(f"{len(pairs)} pairs of feedline edges share cells")
for i, j, n, some in pairs:
    print(f"  f{i} (chain {chain_of.get(i)}) / f{j} (chain {chain_of.get(j)}): {n} cells, e.g. {some}")
