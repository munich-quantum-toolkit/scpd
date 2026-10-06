import re, sys, os
CHIPS = ["4q", "9q", "17q", "21q", "33q", "45q", "57q", "69q"]
def read(arm, chip):
    path = f"artifacts/logs/{arm}/{chip}.log"
    if not os.path.exists(path): return None
    text = open(path).read()
    m = re.search(r"final routing: \d+ of \d+ drawn, (\d+) unrouted[^,]*, (\d+) open, (\d+) short(?:, (\d+) long)?(?:, (\d+) crossing)?", text)
    if not m: return None
    u, o, c = int(m.group(1)), int(m.group(2)), int(m.group(5) or 0)
    sh, lo = int(m.group(3)), int(m.group(4) or 0)
    ids = re.search(r"final routing: unrouted: ([^·]*) · open: ([^·]*) · short: [^·]* · long: [^·]* · crossing: ([^·]*) ·", text)
    names = []
    if ids:
        for part in ids.groups():
            names += [x.strip() for x in part.split(",") if x.strip() and x.strip() != "-"]
    names = sorted(set(names), key=lambda n: (n[0].isalpha(), n))
    sec = re.search(r"^real (\d+)", text, re.M)
    # The fifth phase on its own: the first and the last timestamp the
    # refinement prints, so the stage's own cost is separable from the sweep's.
    phase = None
    begun = re.search(r"^\[final\] +([0-9.]+)s +feedline refinement: ", text, re.M)
    ended = [float(x) for x in re.findall(r"^\[final\] +([0-9.]+)s +feedline refinement round ", text, re.M)]
    if begun and ended:
        phase = round(max(ended) - float(begun.group(1)), 1)
    return u, o, c, sh, lo, names, int(sec.group(1)) if sec else None, phase
arms = sys.argv[1:]
head = "| chip | " + " | ".join(f"{a}: bad (u/o/c) · short/long" for a in arms)
print(head + " | wires (" + arms[-1] + ") | s | phase 5 s |")
print("|---|" + "---|" * (len(arms) + 3))
tot = [0] * len(arms); totsl = [0] * len(arms)
for chip in CHIPS:
    cells = []; wires = "-"; sec = "-"; phase = "-"
    for i, a in enumerate(arms):
        r = read(a, chip)
        if r is None: cells.append("–"); continue
        u, o, c, sh, lo, names, s, ph = r
        tot[i] += u + o + c; totsl[i] += sh + lo
        cells.append(f"**{u+o+c}** ({u}/{o}/{c}) · {sh}/{lo}")
        if a == arms[-1]:
            wires = ", ".join(names) or "none"; sec = s
            phase = ph if ph is not None else "-"
    print(f"| {chip} | " + " | ".join(cells) + f" | {wires} | {sec} | {phase} |")
print("| **sum** | " + " | ".join(f"**{t}** · {sl}" for t, sl in zip(tot, totsl)) + " | | | |")
