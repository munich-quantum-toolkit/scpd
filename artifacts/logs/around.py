#!/usr/bin/env python3
# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Everything drawn within a radius of a point, in a phase of the final artifact.

Usage: around.py <run dir> <phase> <px> <py> <radius>
"""

import math
import pathlib
import sys

from mqt.scpd.artifacts import read_artifact

run, phase, px, py, radius = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4]), float(sys.argv[5])
r = read_artifact(pathlib.Path(f"{run}/06-final.fb").open("rb").read()).output
snap = next(s for s in r.phases if (s.name or "") == phase)


def near(name, w) -> None:
    best = None
    for i, c in enumerate(w.path or []):
        d = math.hypot(c.x - px, c.y - py)
        if best is None or d < best[0]:
            best = (d, i, (c.x, c.y, c.heading))
    best and best[0] <= radius


for i, w in enumerate(snap.wires):
    near(f"wire {i}", w)
for i, w in enumerate(snap.feedlines):
    near(f"f{i}", w)
for i, c in enumerate(snap.couplers or []):
    d = {k: getattr(c, k) for k in dir(c) if not k.startswith("_") and not callable(getattr(c, k))}
    s = str(d)
    print(f"  coupler {i}: {s[:300]}") if i < 1 else None
