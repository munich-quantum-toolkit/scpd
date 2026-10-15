#!/usr/bin/env python3
# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Read the room-rule calibration lines of a run's log.

Usage: calibration.py <log> [<log> ...]

Prints, per log: the R1 edges R1 would refuse (and how many it was asked
about), the R2 coupler channels R2 would refuse, the chain-channel CHECK
line, the bridges CHECK line, the open figures and the insertion seconds.
"""

import pathlib
import re
import sys

STRIP = re.compile(r"^\[final\] +[0-9.]+s  ")


def read(path) -> None:
    lines = [STRIP.sub("", line.rstrip("\n")) for line in pathlib.Path(path).open(encoding="utf-8")]
    r1 = [l.strip() for l in lines if "room R1 chain" in l]
    r2 = [l.strip() for l in lines if "room R2 chain" in l]
    r1_refuse = [l for l in r1 if l.endswith("R1 would refuse")]
    r2_refuse = [l for l in r2 if "— R2 would refuse" in l]
    [
        l
        for l in r2
        if l.endswith("along-runners alone need") or re.search(r"along-runners alone need \d+ — would refuse$", l)
    ]
    [l for l in r2 if l.endswith("— silent")]
    r3 = [l.strip() for l in lines if "room R3 chain" in l]
    r3_near = []
    for l in r3:
        m = re.search(r"lies ([0-9.]+) cells", l)
        if m:
            r3_near.append((float(m.group(1)), l))
    r3_near.sort()
    checks = [l.strip() for l in lines if "CHECK" in l]
    [l.strip() for l in lines if l.strip().startswith("feedline routing round") and "tried" in l]
    [l.strip() for l in lines if r"failed: " in l and "feedline routing round" in l]
    final = [l.strip() for l in lines if l.strip().startswith("final routing:") and "drawn" in l]
    arrow = [l.strip() for l in lines if l.strip().startswith("==> coupler insertion")]
    summary = [l.strip() for l in lines if "room rules —" in l]
    for l in arrow + checks + summary:
        pass
    for l in final[:1]:
        pass
    for l in r1_refuse:
        pass
    for l in r2_refuse:
        pass
    for _d, l in r3_near:
        pass


for path in sys.argv[1:]:
    read(path)
