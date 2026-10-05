import re, sys, os
CHIPS = ["4q", "9q", "17q", "21q", "33q", "45q", "57q", "69q"]
def read(arm, chip):
    path = f"artifacts/logs/{arm}/{chip}.log"
    if not os.path.exists(path): return None
    text = open(path).read()
    m = re.search(r"final routing: \d+ of \d+ drawn, (\d+) unrouted[^,]*, (\d+) open, \d+ short(?:, \d+ long)?(?:, (\d+) crossing)?", text)
    if not m: return None
    u, o, c = int(m.group(1)), int(m.group(2)), int(m.group(3) or 0)
    ids = re.search(r"final routing: unrouted: ([^·]*) · open: ([^·]*) · short: [^·]* · long: [^·]* · crossing: ([^·]*) ·", text)
    names = []
    if ids:
        for part in ids.groups():
            names += [x.strip() for x in part.split(",") if x.strip() and x.strip() != "-"]
    names = sorted(set(names), key=lambda n: (n[0].isalpha(), n))
    sec = re.search(r"^real (\d+)", text, re.M)
    return u, o, c, names, int(sec.group(1)) if sec else None
arms = sys.argv[1:]
print("| chip | " + " | ".join(f"{a}: bad (unrouted/open/crossing)" for a in arms) + " | wires (" + arms[-1] + ") | s |")
print("|---|" + "---|" * (len(arms) + 2))
tot = [0] * len(arms)
for chip in CHIPS:
    cells = []; wires = "-"; sec = "-"
    for i, a in enumerate(arms):
        r = read(a, chip)
        if r is None: cells.append("–"); continue
        u, o, c, names, s = r
        tot[i] += u + o + c
        cells.append(f"**{u+o+c}** ({u}/{o}/{c})")
        if a == arms[-1]: wires = ", ".join(names) or "none"; sec = s
    print(f"| {chip} | " + " | ".join(cells) + f" | {wires} | {sec} |")
print("| **sum** | " + " | ".join(f"**{t}**" for t in tot) + " | | |")
