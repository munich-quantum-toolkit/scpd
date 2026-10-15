#!/usr/bin/env python3
# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Pairs of feedline edges that share a cell, in the `couplers` phase.

Usage: fcross.py <run dir>
"""

import pathlib
import sys

from mqt.scpd.artifacts import read_artifact

r = read_artifact(pathlib.Path(f"{sys.argv[1]}/06-final.fb").open("rb").read()).output
snap = next(s for s in r.phases if (s.name or "") == "couplers")
chain_of = {i: e.chain for i, e in enumerate(r.feedlineEdges or [])}
cells = [{(c.x, c.y) for c in (w.path or [])} for w in snap.feedlines]
pairs = []
for i in range(len(cells)):
    for j in range(i + 1, len(cells)):
        common = cells[i] & cells[j]
        if common:
            pairs.append((i, j, len(common), sorted(common)[:3]))
for i, j, _n, _some in pairs:
    pass
