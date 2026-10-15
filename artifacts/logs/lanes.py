#!/usr/bin/env python3
# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Per wire and kind: how many lane polygons a feedline pass built for it, how many cells of
its own way they priced as outside, and the longest jump from an end of the wire to its wall.
Reads the `lane <wire> · walls …` lines a -v 1 log carries since 2026-10-04.

Usage: lanes.py <log> [<log> ...]
"""

import collections
import pathlib
import re
import sys

for path in sys.argv[1:]:
    log = pathlib.Path(path).open(encoding="utf-8").read().splitlines()
    ring = {}
    for l in log:
        if "the ring the feedline pass sweeps:" in l:
            for tok in l.split("sweeps:")[1].split():
                m = re.match(r"(f?\d+)(\[.*?\]|\(r\))?", tok)
                if m:
                    ring[m.group(1)] = m.group(2) or "plain"
    stats = collections.defaultdict(lambda: [0, 0, 0, 0.0])
    kinds = collections.defaultdict(lambda: [0, 0, 0])
    for l in log:
        m = re.search(
            r"lane (\S+) · walls (\d+) / (\d+) cells, own (\d+) cells, (\d+) of them priced outside the lane, jumps (\d+) (\d+) (\d+) (\d+)",
            l,
        )
        if not m:
            continue
        w = m.group(1)
        kind = ring.get(w, "?")
        s = stats[w, kind]
        s[0] += 1
        s[1] += int(m.group(5))
        s[2] += int(m.group(4))
        s[3] = max(s[3], *(int(m.group(k)) for k in (6, 7, 8, 9)))
        k = kinds[kind]
        k[0] += 1
        k[1] += 1 if int(m.group(5)) > 0 else 0
        k[2] += int(m.group(5))
    for kind, k in sorted(kinds.items()):
        pass
    for (w, kind), s in sorted(stats.items(), key=lambda kv: -kv[1][1])[:12]:
        if s[1] == 0:
            break
