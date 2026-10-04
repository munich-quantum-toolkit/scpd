#!/usr/bin/env python3
"""Everything drawn within a radius of a point, in a phase of the final artifact.

Usage: around.py <run dir> <phase> <px> <py> <radius>
"""
import math, sys
from mqt.scpd.artifacts import read_artifact
run, phase, px, py, radius = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4]), float(sys.argv[5])
r = read_artifact(open(f"{run}/06-final.fb", "rb").read()).output
snap = [s for s in r.phases if (s.name or "") == phase][0]
def near(name, w):
    best = None
    for i, c in enumerate(w.path or []):
        d = math.hypot(c.x - px, c.y - py)
        if best is None or d < best[0]:
            best = (d, i, (c.x, c.y, c.heading))
    if best and best[0] <= radius:
        print(f"  {name}: {best[0]:.1f} cells at index {best[1]} of {len(w.path)} {best[2]}")
for i, w in enumerate(snap.wires): near(f"wire {i}", w)
for i, w in enumerate(snap.feedlines): near(f"f{i}", w)
for i, c in enumerate(snap.couplers or []):
    d = {k: getattr(c, k) for k in dir(c) if not k.startswith('_') and not callable(getattr(c, k))}
    s = str(d)
    if 'x' in d or 'position' in d or 'centre' in d or 'center' in d:
        pass
    print(f"  coupler {i}: {s[:300]}") if i < 1 else None
