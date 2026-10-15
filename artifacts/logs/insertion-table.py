#!/usr/bin/env python3
# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""One row per chip of an arm, for the coupler insertion alone: its seconds, the feedline
angle, the edges not drawn, and how the chains ended — optimum / best within a weight /
best in the time given / out of time / no run joins. Reads artifacts/logs/<arm>/<chip>.log.

Usage: insertion-table.py <arm> [<arm> ...]
"""

import pathlib
import re
import sys

STRIP = re.compile(r"^\[final\] +[0-9.]+s  ")
CHIPS = ["4q", "9q", "17q", "21q", "33q", "45q", "57q", "69q"]
KINDS = [
    ("optimum", re.compile(r"chain \d+: optimum")),
    ("weighted", re.compile(r"chain \d+: best within weight")),
    ("timed", re.compile(r"chain \d+: best in the time given")),
    ("relaid", re.compile(r"chain \d+: best found, an end pair")),
    ("out", re.compile(r"chain \d+: out of time before any run")),
    ("none", re.compile(r"chain \d+: no run of options joins")),
]

for arm in sys.argv[1:]:
    for chip in CHIPS:
        path = f"artifacts/logs/{arm}/{chip}.log"
        if not pathlib.Path(path).exists():
            continue
        lines = [STRIP.sub("", l.rstrip("\n")) for l in pathlib.Path(path).open(encoding="utf-8")]
        arrow = [l for l in lines if l.startswith("==> coupler insertion")]
        m = (
            re.search(r"(\d+) feedline edges? NOT drawn, feedline angle cost (\d+), ([0-9.]+)s", arrow[0])
            if arrow
            else None
        )
        notdrawn, angle, secs = (m.group(1), m.group(2), m.group(3)) if m else ("?", "?", "?")
        counts = {k: 0 for k, _ in KINDS}
        routed = 0
        for l in lines:
            if not l.startswith("[Coupler Insertion] chain "):
                continue
            for k, rx in KINDS:
                if rx.search(l):
                    counts[k] += 1
            r = re.search(r"(\d+) steps routed", l)
            if r:
                routed += int(r.group(1))
