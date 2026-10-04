#!/usr/bin/env python3
"""Read the room-rule calibration lines of a run's log.

Usage: calibration.py <log> [<log> ...]

Prints, per log: the R1 edges R1 would refuse (and how many it was asked
about), the R2 coupler channels R2 would refuse, the chain-channel CHECK
line, the bridges CHECK line, the open figures and the insertion seconds.
"""
import re
import sys

STRIP = re.compile(r"^\[final\] +[0-9.]+s  ")


def read(path):
    lines = [STRIP.sub("", line.rstrip("\n")) for line in open(path)]
    r1 = [l.strip() for l in lines if "room R1 chain" in l]
    r2 = [l.strip() for l in lines if "room R2 chain" in l]
    r1_refuse = [l for l in r1 if l.endswith("R1 would refuse")]
    r2_refuse = [l for l in r2 if "— R2 would refuse" in l]
    r2_along = [l for l in r2 if l.endswith("along-runners alone need") or re.search(r"along-runners alone need \d+ — would refuse$", l)]
    r2_silent = [l for l in r2 if l.endswith("— silent")]
    r3 = [l.strip() for l in lines if "room R3 chain" in l]
    r3_near = []
    for l in r3:
        m = re.search(r"lies ([0-9.]+) cells", l)
        if m:
            r3_near.append((float(m.group(1)), l))
    r3_near.sort()
    checks = [l.strip() for l in lines if "CHECK" in l]
    rounds = [l.strip() for l in lines if l.strip().startswith("feedline routing round") and "tried" in l]
    failed = [l.strip() for l in lines if re.search(r"failed: ", l) and "feedline routing round" in l]
    final = [l.strip() for l in lines if l.strip().startswith("final routing:") and "drawn" in l]
    arrow = [l.strip() for l in lines if l.strip().startswith("==> coupler insertion")]
    summary = [l.strip() for l in lines if "room rules —" in l]
    print(f"===== {path}")
    for l in arrow + checks + summary:
        print("  " + l)
    if rounds:
        print("  " + rounds[-1])
    if failed:
        print("  " + failed[-1])
    for l in final[:1]:
        print("  " + l)
    print(f"  R1: {len(r1_refuse)} of {len(r1)} edges would be refused")
    for l in r1_refuse:
        print("    " + l)
    print(f"  R2: {len(r2_refuse)} of {len(r2)} coupler channels would be refused ({len(r2_silent)} silent, arms do not narrow); by along-runners alone {len(r2_along)}")
    for l in r2_refuse:
        print("    " + l)
    print(f"  R3: couplers with a plain wire within 0/5/10/19 cells: "
          f"{sum(1 for d,_ in r3_near if d<=0)}/{sum(1 for d,_ in r3_near if d<=5)}/"
          f"{sum(1 for d,_ in r3_near if d<=10)}/{sum(1 for d,_ in r3_near if d<=19)} of {len(r3)}")
    for d, l in r3_near:
        if d <= 10:
            print("    " + l)


for path in sys.argv[1:]:
    read(path)
