# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

import pathlib
import re
import sys

CHIPS = ["4q", "9q", "17q", "21q", "33q", "45q", "57q", "69q"]


def read(arm, chip):
    path = f"artifacts/logs/{arm}/{chip}.log"
    if not pathlib.Path(path).exists():
        return None
    text = pathlib.Path(path).open(encoding="utf-8").read()
    m = re.search(
        r"final routing: \d+ of \d+ drawn, (\d+) unrouted[^,]*, (\d+) open, (\d+) short(?:, (\d+) long)?(?:, (\d+) crossing)?",
        text,
    )
    if not m:
        return None
    u, o, c = int(m.group(1)), int(m.group(2)), int(m.group(5) or 0)
    sh, lo = int(m.group(3)), int(m.group(4) or 0)
    ids = re.search(
        r"final routing: unrouted: ([^·]*) · open: ([^·]*) · short: [^·]* · long: [^·]* · crossing: ([^·]*) ·", text
    )
    names = []
    if ids:
        for part in ids.groups():
            names += [x.strip() for x in part.split(",") if x.strip() and x.strip() != "-"]
    names = sorted(set(names), key=lambda n: (n[0].isalpha(), n))
    sec = re.search(r"^real (\d+)", text, re.MULTILINE)
    # The fifth phase on its own: the first and the last timestamp the
    # refinement prints, so the stage's own cost is separable from the sweep's.
    phase = None
    begun = re.search(r"^\[final\] +([0-9.]+)s +feedline refinement: ", text, re.MULTILINE)
    ended = [float(x) for x in re.findall(r"^\[final\] +([0-9.]+)s +feedline refinement round ", text, re.MULTILINE)]
    if begun and ended:
        phase = round(max(ended) - float(begun.group(1)), 1)
    return u, o, c, sh, lo, names, int(sec.group(1)) if sec else None, phase


arms = sys.argv[1:]
head = "| chip | " + " | ".join(f"{a}: bad (u/o/c) · short/long" for a in arms)
tot = [0] * len(arms)
totsl = [0] * len(arms)
for chip in CHIPS:
    cells = []
    wires = "-"
    sec = "-"
    phase = "-"
    for i, a in enumerate(arms):
        r = read(a, chip)
        if r is None:
            cells.append("–")
            continue
        u, o, c, sh, lo, names, s, ph = r
        tot[i] += u + o + c
        totsl[i] += sh + lo
        cells.append(f"**{u + o + c}** ({u}/{o}/{c}) · {sh}/{lo}")
        if a == arms[-1]:
            wires = ", ".join(names) or "none"
            sec = s
            phase = ph if ph is not None else "-"
