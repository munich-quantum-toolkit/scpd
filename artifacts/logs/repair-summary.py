#!/usr/bin/env python3
# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""The lines an arm of the targeted repair is judged by, per log.

Usage: repair-summary.py <log> [<log> ...]

Per log: the chip header, `==> coupler insertion`, the four guarantee CHECK lines, the
triage summary and every segment verdict, `==> targeted repair`, the last feedline round,
the `final routing:` line reduced to unrouted / open / crossing with bad = their sum (the
lengths are not read), and the seconds.
"""

import pathlib
import re
import sys

STRIP = re.compile(r"^\[final\] +[0-9.]+s  ")
GUARANTEES = ("CHECK feedline room", "CHECK coupler crossings", "CHECK resonator crossings", "CHECK feedline crossings")


def read(path) -> None:
    lines = [STRIP.sub("", line.rstrip("\n")) for line in pathlib.Path(path).open(encoding="utf-8")]
    header = [l for l in lines if l.startswith("### ")]
    arrow = [l for l in lines if l.startswith("==> coupler insertion")]
    checks = [l.strip() for l in lines if any(g in l for g in GUARANTEES)]
    [l for l in checks if "GREEN" not in l]
    triage = [
        l.strip()
        for l in lines
        if l.strip().startswith("targeted repair:")
        and (("segment" in l and "—" in l) or "fails →" in l or "fail →" in l or "budget" in l or "final sweep" in l)
    ]
    repair = [l for l in lines if l.startswith("==> targeted repair")]
    settings = [l for l in lines if l.startswith("targeted repair settings:")]
    [l for l in lines if l.startswith("feedline routing round") and "tried" in l]
    final = [l for l in lines if l.startswith("final routing:") and "drawn" in l]
    seconds = [l for l in lines if re.match(r"^(real|user) ", l)]
    outer = [l for l in lines if l.startswith("outer routing:") and " drawn, " in l]
    lenpt = [l for l in lines if l.startswith("outer routing: LENPT")]
    for l in header[:1]:
        pass
    # The outer routing's own verdict, lengths left out, and its length-point line.
    for l in outer[:1]:
        pass
    for l in lenpt[:1]:
        pass
    for l in arrow + checks:
        pass
    for l in settings + triage + repair:
        pass
    # The lengths are not the figure and are not read (user, 2026-10-04): only
    # unrouted, open and crossing, and their sum.
    for l in final[:1]:
        m = re.search(r"(\d+) of (\d+) drawn, (\d+) unrouted.*?(\d+) open.*?(\d+) crossing", l)
        if m:
            _drawn, _total, _unrouted, open_, _crossing = (int(g) for g in m.groups())
    for l in seconds:
        pass


for path in sys.argv[1:]:
    read(path)
