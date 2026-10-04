#!/usr/bin/env python3
"""The lines an arm of the targeted repair is judged by, per log.

Usage: repair-summary.py <log> [<log> ...]

Per log: the chip header, `==> coupler insertion`, the four guarantee CHECK lines, the
triage summary and every segment verdict, `==> targeted repair`, the last feedline round,
the `final routing:` line reduced to unrouted / open / crossing with bad = their sum (the
lengths are not read), and the seconds.
"""
import re
import sys

STRIP = re.compile(r"^\[final\] +[0-9.]+s  ")
GUARANTEES = ("CHECK feedline room", "CHECK coupler crossings", "CHECK resonator crossings",
              "CHECK feedline crossings")


def read(path):
    lines = [STRIP.sub("", line.rstrip("\n")) for line in open(path)]
    header = [l for l in lines if l.startswith("### ")]
    arrow = [l for l in lines if l.startswith("==> coupler insertion")]
    checks = [l.strip() for l in lines if any(g in l for g in GUARANTEES)]
    red = [l for l in checks if "GREEN" not in l]
    triage = [l.strip() for l in lines if l.strip().startswith("targeted repair:")
              and ("segment" in l and "—" in l or "fails →" in l or "fail →" in l
                   or "budget" in l or "final sweep" in l)]
    repair = [l for l in lines if l.startswith("==> targeted repair")]
    settings = [l for l in lines if l.startswith("targeted repair settings:")]
    rounds = [l for l in lines if l.startswith("feedline routing round") and "tried" in l]
    final = [l for l in lines if l.startswith("final routing:") and "drawn" in l]
    seconds = [l for l in lines if re.match(r"^(real|user) ", l)]
    outer = [l for l in lines if l.startswith("outer routing:") and " drawn, " in l]
    lenpt = [l for l in lines if l.startswith("outer routing: LENPT")]
    print(f"===== {path}")
    for l in header[:1]:
        print("  " + l)
    # The outer routing's own verdict, lengths left out, and its length-point line.
    for l in outer[:1]:
        print("  " + re.sub(r", \d+ short \| Fails: \d+$", "", l))
    for l in lenpt[:1]:
        print("  " + l)
    for l in arrow + checks:
        print("  " + l)
    if red:
        print(f"  !! {len(red)} CHECK line(s) not green")
    for l in settings + triage + repair:
        print("  " + l)
    if rounds:
        print("  " + re.sub(r", short \d+ \| Fails: \d+$", "", rounds[-1]))
    # The lengths are not the figure and are not read (user, 2026-10-04): only
    # unrouted, open and crossing, and their sum.
    for l in final[:1]:
        m = re.search(r"(\d+) of (\d+) drawn, (\d+) unrouted.*?(\d+) open.*?(\d+) crossing", l)
        if m:
            drawn, total, unrouted, open_, crossing = (int(g) for g in m.groups())
            print(f"  final routing: {drawn} of {total} drawn, {unrouted} unrouted, {open_} open, "
                  f"{crossing} crossing  => bad {unrouted + open_ + crossing}")
        else:
            print("  " + l)
    for l in seconds:
        print("  " + l)


for path in sys.argv[1:]:
    read(path)
