# The feedline routing

Written for whoever takes the feedline stage further. The stage draws
**every feedline edge of every chip** — 305 of 305 over the eight benchmarks
— and holds four guarantees: a feedline keeps the design rule from every
coupler lead, two edges meeting at a coupler do not cross, no feedline
crosses a resonator, and no feedline crosses a feedline of another chain.
Each is checked at the end of the insertion, the fourth again after the
repair; the first three are green on all eight chips, the fourth is red on
69q at the insertion (four pairs since the insertion closes the terminal
edges' coupler runs, six before) and green once the pass has redrawn the
terminal edges. The figure the stage is judged by is `bad = unrouted + open +
crossing` over every wire at the end of the stage; lengths are not read
(user, 2026-10-04). On 2026-10-05 evening it stands at **8 over the
eight chips**, every one a crossing wire at the halo's edge and none
open, from 20 at midday, 34 the evening before and 77 the morning before
that — see *Where it stands*.

Read [handover-cpw-coupler-insertion.md](handover-cpw-coupler-insertion.md)
for the coupler options, the geometry and the chain search this stands on,
and [handover-chain-astar.md](handover-chain-astar.md) for the prefix search
that settles a chain. This document is about what happens to the wires
afterwards, and about the obstacles the insertion builds for its own edges.

- Checkout: `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`
- Branch: `phase-4-routing-stages`. HEAD is **`454df86` 20 fails
  remain**, which carries everything up to midday 2026-10-05: the room
  rules, the crossing rule, the targeted repair, the bridge check, the
  length-point clearance, the verdict every wire carries in the artifact
  and the failing wires marked in the picture, the terminal edges' coupler
  runs closed in the feedline pass and the insertion, three fenced pairs,
  the halo at one clearance, `max_relaxation = 8` everywhere and `rounds =
  5` on 69q. **Uncommitted** on top of it, the user committing per phase,
  the afternoon and evening of 2026-10-05: the squeeze report and rule
  with its marks in the artifact and the pictures (*The squeeze report*);
  the chain A*'s step budget (`SCPD_CHAIN_STEP_BUDGET`, on) and three
  arms that did not help and are off — the length term, the learned bound,
  the obstacle-aware bound (handover-chain-astar, *What is open*).
- Benchmarks: `repair_trials = 0`, `stop_after = "feedlines"` and
  `max_relaxation = 8` in all eight `benchmarks/*/config.toml` (8 since
  2026-10-05, user; the 2026-10-04 figures were measured with the 8 set
  in the run directory's copy), so a run ends where this document ends.
  69q also says `rounds = 5` (user, 2026-10-05: one round fewer ended with
  fewer fails there).
- Run one chip with
  `.venv/bin/mqt-scpd plan -c benchmarks/45q/config.toml -o artifacts/45q --stage final -v 1`.
  `--stage final` resumes on the artifacts already in the run directory; it
  does **not** recompute the earlier stages, and it fails if they are missing.

## Where it stands

**2026-10-04 evening** (`artifacts/logs/full-r8`: the eight chips to the
feedline routing, no repair, `max_relaxation` 8, the switches at their
defaults — crossing rule on, bridge check on, length-point clearance k 40
hard with recovery, polygon and per-level halos in the outer routing),
against the morning's `base-ortho2` and the `bridge-check` arm between
them; `bad` = unrouted + open + crossing at the end of the stage:

| | 4q | 9q | 17q | 21q | 33q | 45q | 57q | 69q | all | s |
|---|---|---|---|---|---|---|---|---|---|---|
| base-ortho2, the rule on | 0 | 0 | 8 | 0 | 5 | 8 | 25 | 31 | 77 | 2287 |
| + bridge check | 0 | 0 | 8 | 0 | 4 | 8 | 23 | 28 | 71 | 2400 |
| + length-point clearance, relaxation 8 | 0 | 0 | 12 | 0 | 0 | 4 | 2 | 16 | **34** | 1276 |
| + terminal stub guard, 3 fenced pairs, halo reach 1 (2026-10-05, `guard-pairs3-halo1-all`) | 0 | 0 | 12 | 1 | 0 | 1 | 2 | 4 | **20** | 1075 |
| + squeeze reject (2026-10-05 afternoon, `squeeze-reject`) | 0 | 0 | 4 | 1 | 0 | 0 | 2 | 1 | **8** | 1021 |
| + the resonator-tail fix of the squeeze measurement and the step budget (`squeeze-reject2`, the state handed over) | 0 | 0 | 4 | 1 | 0 | 0 | 2 | 1 | **8** | 1017 |

**The feedline pass itself at that state**, chip by chip: every edge of
every chain drawn, no wire open, no edge squeezed, the crossing check
between chains green on all eight, and in the last round of the pass only
17q's wire 31 without a way.

| chip | edges drawn | wires drawn | open | crossing | crossing check | last round failed |
|---|---|---|---|---|---|---|
| 4q | 5 / 5 | 14 / 14 | 0 | 0 | green | – |
| 9q | 10 / 10 | 33 / 33 | 0 | 0 | green | – |
| 17q | 20 / 20 | 65 / 65 | 0 | 4 | green | 31 |
| 21q | 26 / 26 | 80 / 80 | 0 | 1 | green | – |
| 33q | 40 / 40 | 124 / 124 | 0 | 0 | green | – |
| 45q | 52 / 52 | 165 / 165 | 0 | 0 | green | – |
| 57q | 66 / 66 | 208 / 208 | 0 | 2 | green | – |
| 69q | 81 / 81 | 254 / 254 | 0 | 1 | green | – |

The eight that remain are one case: a plain wire crossing a chain edge at
10 cells from it (31 on 17q at 11) — the halo's edge of the orthogonal
crossing rule, where the search lets a wire cross straight and turn on
the last cell of the halo and the count reads that cell with its exit
heading. `SCPD_CROSSING_EXIT_HEADING` is the switch that makes the two
agree; it is off, measured on 2026-10-04 (crossing 22 → 7 then), and the
user's decision. The lengths are not read.

**2026-10-05 afternoon**: the squeeze rule (*The squeeze report*) takes
the eight chips to **8**, every one of them a crossing wire at 10 to 14
cells from an edge — 17q 14, 31, 33, 56; 21q 27; 57q 15, 171; 69q 15 —
and no wire open anywhere. The insertion refused 121 ways on 17q, 63 on
57q, 21 on 45q and 7 on 69q, none elsewhere, and no edge needed the
recovery; 17q's insertion took 9.5 s instead of 4.0, the others within a
second of before.

**2026-10-05**: the last row is the stage as it stands at the end of the
day — `SCPD_TERMINAL_STUB_GUARD`, `SCPD_FEEDLINE_FENCE_PAIRS` 3 and
`SCPD_HALO_REACH` 1, all defaults now, see *The terminal edges' coupler
runs* — measured with `run-arm.sh` on the installed binding, 69q on its
config (5 rounds, relaxation 8), the others at `MAX_RELAXATION=8` as
`full-r8` was. On the configs as committed (relaxation 5) 57q ends at 13
instead of 2: its outer routing needs the 8 (`outer57-*`), as before.
The wires: 17q 2, 10, 11, 14, 31, 33, 40, 56, f0, f1, f12; 21q 27
crossing; 45q 65 crossing; 57q 15, 171 crossing; 69q 183/184 open, 15 and
191 crossing. `artifacts/logs/bad-table.py <arm> …` prints this table.

Every outer routing ends at 0 unrouted / 0 open (17q through the recovery),
every edge is drawn, and 17q is the one chip that got worse. The per-chip
columns, the angle and the length-point figures are in *The length-point
clearance*; the targeted repair's arms (`research`, `research-exit`) were
measured on the morning's outer routing and are owed again on this one.

Two figures say how the pass stands, and they are not the same figure.

- **Open in the last round** is the `open N` of the pass's final round line
  — `feedline routing round 5 backward: tried 9, routed 5, unrouted 0,
  open 4, short 0 | Fails: 4` — and the `N failed: …` line after the pass
  names them: the wires whose own attempt found no way in that round and
  were put back as they were. **This is the figure the stage is judged by**
  (user, 2026-10-03). It counts one wire per conflict, the one that lost.
- **Open at the end of the stage** is what `failsOf` counts with every wire
  down: a wire whose way lies within the rule of another's. A conflict
  counts both its partners, so this figure comes in pairs and runs about
  twice the first. The `is open in the room of` lines say who, where, and
  how far from each end.

**The regime changed on 2026-10-04**: the orthogonal crossing rule is on by
default, and the stage is judged by `bad = unrouted + open + crossing` at the
end of the stage (user). Under it, before any repair
(`artifacts/logs/base-ortho2`, 2026-10-04, identical in every judged line to
`base-ortho` of 2026-10-03):

| rule on | 4q | 9q | 17q | 21q | 33q | 45q | 57q | 69q | all |
|---|---|---|---|---|---|---|---|---|---|
| open, last round | 0 | 0 | 4 | 0 | 3 | 2 | 9 | 10 | 28 |
| open, end | 0 | 0 | 4 | 0 | 4 | 6 | 17 | 24 | 55 |
| crossing a feedline | 0 | 0 | 4 | 0 | 1 | 2 | 8 | 7 | 22 |
| **bad** | 0 | 0 | 8 | 0 | 5 | 8 | 25 | 31 | **77** |
| fails, lengths included | 4 | 1 | 15 | 2 | 12 | 21 | 47 | 59 | 161 |

Twelve of the 22 crossing wires found a way in their last search and are
crossing by the count alone — see *The search and the count disagree at the
halo's edge* in [handover-targeted-repair.md](handover-targeted-repair.md).
What the repairs make of these figures, `bad` summed over the eight chips
(every arm 20 trials, 2026-10-04): the blind `repair` 73, the targeted
re-search 69, the re-search with the search's exit heading tested
(`SCPD_CROSSING_EXIT_HEADING=1`) 51, that rule alone 61 — tables and the
per-chip figures in the same document. The tables below are the rule-off
figures the stage was handed over on, kept as the control arm.

Measured over the eight benchmarks, no environment variable set, one run at
a time, 2026-10-03 — `SCPD_RESONATOR_STUB` off and `SCPD_FENCE_FIXED` on,
the defaults since *The resonator's exit*, and the rule **off**, which was
the default then. The five small chips have no open wire by either figure,
under any setting tried, so the tables show the three that do:

| open | 45q | 57q | 69q | all |
|---|---|---|---|---|
| **in the last round** | **2** | **1** | **4** | **7** |
| at the end of the stage | 6 | 3 | 9 | 18 |

In the last round: 45q 116 and 65, 57q 10, 69q 176, 19, 9 and f0 — a
terminal edge, whose partner 9 is what the end count sees instead. At the
end: 45q 65/66, 116/117, 133/136; 57q 9/10/11; 69q 9/10/f1, 19/20, 176/177,
203/206.

Every verdict per chip, the lengths included for completeness. They are the
meander's business — `SCPD_FEEDLINE_MEANDER` is off, so the pass routes and
does not try to make a resonator its length — and **not this phase's
figure** (user, 2026-10-03):

| Chip | edges drawn | open (last round) | open (end) | short | long | fails |
|---|---|---|---|---|---|---|
| 4q | 5 / 5 | 0 | 0 | 4 | 0 | 4 |
| 9q | 10 / 10 | 0 | 0 | 1 | 0 | 1 |
| 17q | 20 / 20 | 0 | 0 | 9 | 0 | 9 |
| 21q | 26 / 26 | 0 | 0 | 1 | 1 | 2 |
| 33q | 40 / 40 | 0 | 0 | 9 | 0 | 9 |
| 45q | 52 / 52 | 2 | 6 | 9 | 4 | 18 |
| 57q | 66 / 66 | 1 | 3 | 20 | 7 | 29 |
| 69q | 81 / 81 | 4 | 9 | 29 | 8 | 45 |
| **total** | **305 / 305** | **7** | **18** | **82** | **20** | **117** |

`fails` counts **wires** with any verdict against them; a wire can be open
*and* off its length.

What this was measured against, the same build with the two switches the
other way round, 2026-10-03 morning: in the last round 5 / 6 / 7 = **18**,
at the end 12 / 19 / 20 = **51**, fails 24 / 46 / 49, and on the small chips
4 / 2 / 7 / 3 / 11 fails with no open wire. Before that, 2026-10-02 midday:
63 open at the end, 169 fails, one edge with no way.

**What is left, by cause.** 45q 65/66, 133/136, 57q 9/10/11 and 69q 19/20,
176/177, 203/206 are pairs of plain wires trading one lane: each is drawn
through the other's released room and the other then finds nothing, round
after round, forward and back. 45q 116/117 is a resonator's way into its
qubit and the wire beside it sharing a pinch at the lattice. 69q's 9, 10 and
f1 are one case: the insertion laid 9's lead one cell from 10's way, 10 has
no room to leave once that lead is fenced, and 9 is *dead on arrival* in
every round — the repair's case (`repair_trials`), not the sweep's.

## The four checks

They run at the end of `insertCouplers` and print one line each. All of them
measure **what the corridor actually builds**, not a rule of their own — that
is the point of them, and it is what makes a green line mean something.
`checkClearance` in the DRC is the second opinion about the design rule, and
it has the whole chip to look at rather than this stage's fence.

```
coupler insertion: CHECK feedline room — 0 feedline/lead pairs closer than the rule; the check is GREEN
coupler insertion: CHECK coupler crossings — 0 pairs of edges meeting at a coupler cross; the check is GREEN
coupler insertion: CHECK resonator crossings — 0 feedlines cross a resonator; the check is GREEN
coupler insertion: CHECK feedline crossings — 0 pairs of edges of different chains cross; the check is GREEN
```

- **`checkFeedlineCrossings`** (2026-10-04) — every pair of drawn edges of
  **different** chains, `waysCross` prefiltered by the boxes the ways span.
  It is **red on 69q** with the four pairs *The room rules* found and
  `artifacts/logs/fcross.py` confirmed — `f32 (chain 4) x f33 (chain 5) ·
  f46 x f47 · f60 x f61 · f73 x f74` — and green on the other seven. The
  targeted repair reads the same pairs as fails of both edges.

- **`checkFeedlineRoom`** — every feedline against what the corridor fences,
  inflated by the rule in layout units (unrounded, as `conflictsIn` takes
  it), less the terminal slots. It follows `fenceTheLeadOnly`: with the
  switch on it measures leads, with it off whole resonators. A line that says
  `feedline/lead` and one that says `feedline/resonator` are answering
  different questions, and the line says which.
- **`checkCouplerCrossings`** — every pair of edges that meet at a coupler.
- **`checkResonatorCrossings`** — every feedline against every resonator past
  its lead.
- **`checkBridgers`** (2026-10-04), after `assignBridges` in phase 4: whether
  the edge every plain wire was told to cross is the one the insertion
  counted it against (`bridgersOf`, R0 of the room rules). Green on all eight
  chips. And `checkChainChannels` prints a fourth line at the end of the
  insertion, `CHECK chain channels`, which is a report and not a guarantee:
  see *The room rules* in [handover-cpw-coupler-insertion.md](handover-cpw-coupler-insertion.md).

Both crossing checks use `waysCross`, which does **not** only compare cells:
two diagonal steps through one unit square pass through each other without
ever standing on the same pixel, and a cell comparison calls such a pair
clear. It keys the diagonals by the square's lower-left corner and which way
the step leans, and finds both cases.

## The obstacles a chain edge is routed against

This is the heart of it. `corridorOfEdge` builds the corridor for one edge of
one chain, and what it closes decides everything the insertion then hands to
the feedline pass.

1. **The artwork**, from `scene_.components`. Not `blocked`: that carries two
   more things an edge has no reason to obey.
2. **A box** around the two endpoints with a wide margin, and everything
   outside it closed — a bound on the search, not a shape.
3. **Every coupler body**, copper only. The ports sit one cell outside the
   pad by construction, so closing it costs the edge nothing it needs.
4. **Every coupler lead**, inflated by the clearance. The lead is `option.arc`
   — the quarter turn and the straight after it — and it is the head of the
   resonator's way.
5. **Every resonator past its lead**, two cells wide: not crossable, no room
   kept.
6. **The launcher stubs** of every launcher but the one this edge runs to.
7. **This chain's own edges**, with two exceptions below.
8. **The terminal slots**, opened again, last. The only thing that opens.

The foreign leads and bodies live in one `grid::BitGrid`, `foreignRoom_`,
built once per chain solve and read in the pass the box already makes over the
grid. Building it per search would cost about twenty thousand stencil cells
per resonator — on 69q, twenty billion operations over an insertion. This is
why `fenceResonators` had never had a caller.

### The terminal slot

Everything within the clearance of a lead is closed, and that seals the
corridor at the port the edge has to dock on. A straight run is freed again at
each terminal and **nothing else is** — no coupling zone, no exemption around
an anchor. The prototype does exactly this,
`free_terminal_stub(..., 20)` at `FinalGrid.cpp:20160`.

The length is `max(SCPD_TERMINAL_SLOT, run + BEND_RADIUS)`. The 20 is the
prototype's, where it is the same number twice — it holds the router to 20 and
frees 20. Ours differs: the run at a coupler is the **pad's own length**,
because the feedline lies along the whole pad, and on 45q that is 21. A flat
slot of 20 left the twenty-first cell inside the lead's clearance and six
edges of 45q and fourteen of 57q came back with no way. Measured on 45q, slot
against edges drawn: 20 → 46 of 52, 25 → 52, and 30, 35, 40, 60, 100 all
bit-identical to 25. What is missing at 20 is a handful of cells to turn in,
which is what `BEND_RADIUS` is by name.

The artwork is **not** freed with the slot, where the prototype frees it
unconditionally. A forced run that lies on copper is an edge that cannot be
built, and opening it would hide that rather than mend it.

### The lead, and why not the whole resonator

`SCPD_EDGE_LEAD_ONLY` decides whether the clearance goes around the coupler's
lead or around the whole resonator. It is **on**, and the reason is one edge:

On 69q, edge f1 of chain 0 runs from (1835,4440) to (1904,4308). Its own
resonator crosses that strip at x≈1874 — a hundred and thirty cells from the
coupler, where the two have nothing to do with each other. With the whole
resonator fenced, chain 0 never brings one run of options to the end of the
chain in its ten seconds and the edge is lost. No slot reaches that, because a
slot runs along the terminal's heading and the room the edge wants there is
sideways.

Fencing the lead alone and leaving the rest open was measured and was much
worse — 100 open against 52, 199 fails against 160, and thirty pairs under the
rule. **Making the rest uncrossable is what resolves it**: a feedline may lie
beside a resonator far from any coupler, but may not run across it.

| | edges | open | fails |
|---|---|---|---|
| whole resonator fenced | 1 missing | 52 | 160 |
| lead only, rest open | all drawn | 100 | 199 |
| **lead + uncrossable rest** | **all drawn** | **51** | **146** |

The 43 open wires the lead fence cost came almost entirely from feedlines
running over resonators. Without the crossings almost none of that price is
left.

### The two exceptions for a chain's own edges

**A terminal edge is let off for the edges far from it, and gets no such
licence itself** (`SCPD_TERMINAL_EDGES_FENCE_ALL=0`, and the asymmetry is
deliberate). A terminal edge leaves a launcher on the launcher's heading and
cannot yield, so the rest of the chain works around it; for an edge four
waypoints along, fencing it is a wall where the two were never going to meet.
But when the terminal edge is itself being drawn, every edge of its chain is
in its way. The prototype goes the other way entirely and hard-fences the
terminal edges for everyone (`forbidden_paths_start_end`).

**The edges after this one stand open, except the one next to it**
(`SCPD_FENCE_LATER_EDGES=0`). The reason is in *The traps* below. The
neighbour is never skipped: it shares a coupler, and skipping it let the edge
leaving a coupler be drawn through the edge arriving at it.

## The resonator's exit

Written 2026-10-03. This is why 45q's wire 4 and 131, 57q's 11, 86, 91,
133, 149 and 152 and 69q's 9, 11, 99, 101, 106, 180 and 201 — fourteen of the
seventeen resonators that were open — found no way in any round at any
relaxation, with their own seed way open end to end in the corridor and a
strip twenty cells wide beside it.

**A resonator's search does not start at the tip of its lead.** The router
moves the source along its heading by the start stub before it searches
(`DubinsRouter::sanitize`; `straightStub` runs *forward* from the source,
`assemble` puts it in front of the way afterwards), and `applyOption` gave a
resonator a start stub of the rule's run — 11 cells — on top of the lead's
14. So the search began 25 cells after the turn, where the prototype's
begins after 14: its `params_resonators.start_straight_length = 14` is the
lead and the stub in one (`FinalGrid.cpp:10658`).

**Those eleven cells belonged to no way until the resonator had been drawn
once**, and nothing kept the neighbours out of them. The way the insertion
splices is the lead and the old tail, and the tail turns off five cells after
the tip; `fence` closes the clearance around a wire's **way**, and `fixPlaces`
charges the fixed places only while a wire is not down. A neighbour drawn
against the seed in round 0 therefore settled inside the stub's clearance:
45q's wire 3 stood 38.9 cells from 4's stub end at the start of the pass and
17.0 after its own round-0 search, 21.2 from 4's way all the while; 130 did
the same to 131 (38 to 18.0). From then on the first cell the search had to
step onto was closed, in every round, at every level, because the wire that
closed it stood on the other side from the wires the relaxation let go of —
or two slots away, where the second pair is fenced in every search.

**And a wire let go of gave its head away.** A released wire is fenced by
nothing, so a relaxed neighbour could be drawn through its lead: 132 on 45q
came within 6 cells of 131's resonator port, 13 having been the insertion's
figure. The prototype keeps exactly these cells hard for a ripped resonator
(`resonator_head_cells`, the arc and twenty cells beyond it,
`FinalGrid.cpp:12100`).

Two switches, each measured alone and together, open wires at the end of the
stage, the five small chips at none throughout:

| | 45q | 57q | 69q | all |
|---|---|---|---|---|
| the second run, ways fenced (before) | 12 | 19 | 20 | 51 |
| `SCPD_RESONATOR_STUB=0` | 8 | 11 | 15 | 34 |
| `SCPD_FENCE_FIXED=1` | 8 | 3 | 19 | 30 |
| **both, the defaults now** | **6** | **3** | **9** | **18** |

- **`SCPD_RESONATOR_STUB`**, off: a resonator drawn from its coupler has no
  start stub beyond its lead; the lead is already `max(14, straightStart)`
  cells and the search starts at its tip. `=1` restores the second run.
- **`SCPD_FENCE_FIXED`**, on: `fence` closes the clearance around a wire's
  fixed places — its lead, the run out of its source, the run into its target
  — as well as its way, and the relaxation keeps the fixed places of the
  wires it let go of closed (`fenceFixedPlaces`). `=0` fences the ways alone.

The same four arms counted in the last round, the figure the stage is
judged by:

| | 45q | 57q | 69q | all |
|---|---|---|---|---|
| before | 5 | 6 | 7 | 18 |
| `SCPD_RESONATOR_STUB=0` | 3 | 4 | 6 | 13 |
| `SCPD_FENCE_FIXED=1` | 3 | 1 | 8 | 12 |
| **both, the defaults now** | **2** | **1** | **4** | **7** |

The two need each other. Without the stub the search starts where the
neighbours were fenced, but a wire let go of still gives its head away: 131
on 45q finds a way in round 0 and loses it to 132's relaxation. With the
head fenced the run is still forced: 4 finds a way in round 0 and loses it to
5's. Together they close every resonator that was open at its head on 45q and
57q. Fails on the three large chips went 24/46/49 to 18/29/45; the one cost
is two short resonators on 17q (7 to 9), which the meander is for.

**The line that would have found this in minutes is now in the log.** Every
search that finds nothing says, at `-v 1`, whether it was over before it
began — `dead on arrival: the search starts at (809,3048) heading (0,-1)
after a stub of 11, and 3 of its first 4 cells are closed: (809,3047)
closed, within the clearance of 3 (way at 17.0 cells); …` — naming the
start cell, the cells a first move must reach, and every wire whose way or
fixed places lie within the clearance of each (`deadOnArrival`,
`whatIsNear`). It fires in the outer routing too, where the Detail stage's
seed of a neighbour can stand inside a launcher stub's clearance. The `in the
way` line cannot see any of this: it never fences the second pair of
neighbours, and the pictures draw a closed cell that no named fence explains
as plain white. And the feedline pass now prints a `feedline routing
settings:` line naming both switches and whether each came from the
environment.

### How this was found, so that it can be done again

Nothing of this is visible in the fail counts, and the pictures do not show
it either. What showed it, in order:

1. **The `is open in the room of` lines** at the end of the stage. Every
   open wire on 45q was one of a pair, and twelve of the seventeen open
   resonators over the three chips had their conflict within 24 cells of
   their source.
2. **The round table** (`dashboard-feedline.html`, or the same table printed
   from the log) — which wire failed in which round at which relaxation
   level. A wire that fails in every round at every level is not short of
   room, something stands that no relaxation lifts: 4 and 131 on 45q, 10 and
   133 on 57q, 9, 101, 106 and 180 on 69q.
3. **The seed state.** `06-final.fb` carries a snapshot per phase, and the
   `couplers` phase is the ring as the feedline pass starts on it; the
   `feedlines` phase is how it ends. Wire `k` of the ring is `wires[k]`,
   edge `fk` is `feedlines[k]`, a cell is `(x, y, heading)` with headings
   clockwise from south (0 south, 2 west, 4 north, 6 east). Distances between
   ways, and from a wire's head to its neighbours, come straight from that.
4. **The corridor itself.** A search picture draws every open cell as a
   `co` rectangle, cell `(x, y)` at SVG `(x, 3519 − y)` on a 3520 grid, so the
   corridor of any search can be read back cell by cell. For 4 and 131 the
   whole seed way was open and the strip beside it 16 to 34 cells wide — and
   the search still found nothing. That is what pointed at the start rather
   than the room.
5. **The router.** `sanitize` moves the source forward by the start stub,
   so the search begins 11 cells past the lead's tip; those cells, and the
   ones a first move has to reach, were inside wire 3's clearance — 17.0
   cells, where 3's seed had stood 38.9 away and 3's new way kept 21.2 from
   4's seed. The `dead on arrival` line now prints exactly this.

What was ruled out on the way, so nobody pays for it again: the `in the
way` line said `the neighbours`, which is true only because that line
fences neither neighbour of the second pair; the forward relaxation released
5, 6, 7, 8, 9 with 3 fenced, the backward one 3, 2, 1, 0, f0 with 5 fenced,
and a search with both 3 and 5 open found a way. The prototype's band (800
cells against our 798, and the whole chip from round 1 on), its four fences
in phase 1 and at every relaxation level, and its prices (`proxy_fact = 1`,
weaker than our halo and toll) are what ours are; the router has no budget,
so `no way` is `no way`. None of those was it.

## The switches

Everything is read from the environment, so a sweep costs a rebuild of
nothing. The feedline pass prints a `feedline routing settings:` line with
the two newest and where each value came from; the coupler insertion prints
its own. Defaults, with the ones the sessions of 2 and 3 October changed
marked:

| switch | default | |
|---|---|---|
| `SCPD_ROOM_REPORT` | on | new, 2026-10-04: the room-rule calibration lines and the `CHECK chain channels` line; report only |
| `SCPD_ROOM_PITCH` / `SCPD_ROOM_MARGIN` / `SCPD_ROOM_CHANNEL_REACH` / `SCPD_ROOM_CHANNEL_COUNT` | 20 / 10 / 80 / 1 | new, 2026-10-04: what the calibration measures with; no rule is live — see *The room rules* in [handover-cpw-coupler-insertion.md](handover-cpw-coupler-insertion.md) |
| `SCPD_RESONATOR_STUB` | **off** | new, 2026-10-03: no second straight run after the lead |
| `SCPD_FENCE_FIXED` | **on** | new, 2026-10-03: the fixed places keep their clearance in every fence |
| `SCPD_EDGE_LEAD_ONLY` | on | new |
| `SCPD_EDGE_ALL_RESONATORS` | on | new |
| `SCPD_EDGE_RESONATOR_CLEARANCE` | on | new |
| `SCPD_TERMINAL_SLOT` | 20, effective `max(20, run + BEND_RADIUS)` | new |
| `SCPD_FEEDLINE_LEAD_TRIM` | on | new |
| `SCPD_BODY_GUARD` | on | new |
| `SCPD_EDGE_SEES_CHAINS` | off | new |
| `SCPD_TERMINAL_EDGES_FENCE_ALL` | off | new |
| `SCPD_FENCE_LATER_EDGES` | off | new |
| `SCPD_LAUNCHER_LEAD` | 10.0 | new |
| `SCPD_CHAIN_KEEP_WAYS` | **off** | was on |
| `SCPD_ORTHO_CROSSING` | **on** | was off 2026-10-02 to 2026-10-04; the regime now, see [handover-targeted-repair.md](handover-targeted-repair.md) |
| `SCPD_BRIDGE_CHECK` | **on** | new, 2026-10-04: a way found that does not cross the wire's bridged edge is refused as no way — see *The bridge check* below |
| `SCPD_FENCE_TO_CONVENTIONAL` | off | new, 2026-10-05: the fence beyond the wire let go of walks on to the next conventional wire, a resonator not bounding the room; measured on 69q at 6 rounds / relaxation 8 and identical in every judged line to `full-r8` (47 searches fenced more, 4 outcomes moved inside the 180–184 pair) — not the cause of the round-5 loss (user) |
| `SCPD_FEEDLINE_FENCE_PAIRS` | **3** | new, 2026-10-05: how many pairs of ring neighbours the feedline pass fences, in phase 1 (`slot ± k`) and at every relaxation level (the k-th beyond the wire let go of and the k-th behind the wire drawn, released wires never); 2 is the prototype. 3 and 4 both take 69q from 10 to 5 with the terminal stub guard — see *The terminal edges' coupler runs*; 3 is the default (user) |
| `SCPD_TERMINAL_STUB_GUARD` / `SCPD_TERMINAL_STUB_EXTRA` | **on** / 5 | new, 2026-10-05: the run a terminal edge has to make at its coupler — into the first coupler, out of the last — plus 5 cells, inflated by the clearance, closed to every other wire's search in the feedline pass, drawn or not; the launcher end is the port band's. Terminal edges are fenced for nobody otherwise (user) — see *The terminal edges' coupler runs* |
| `SCPD_SQUEEZE_REPORT` / `SCPD_SQUEEZE_STEP` / `SCPD_SQUEEZE_REACH` | on / 5 / 300 | new, 2026-10-05: the squeeze report at the end of the insertion — see *The squeeze report* |
| `SCPD_CHAIN_BOUND` 3 / `SCPD_CHAIN_FAMILY_PATHS` | 1 / 20000 | new, 2026-10-05 night: the analytic bound around the artwork (families of ways exhausted against the obstacles), sound and measured useless on 17q and 69q — see *What is open* in handover-chain-astar.md |
| `SCPD_CHAIN_STEP_BUDGET` / `SCPD_CHAIN_LEARNED_BOUND` | on / off | new, 2026-10-05 evening: the chain A*'s step budget (exact, placeholders at the incumbent dropped, the edge search bounded in turns) and the learned pair bound (heuristic, measured useless) — see *What is open* in handover-chain-astar.md |
| `SCPD_CHAIN_LENGTH_WEIGHT` | 0 | new, 2026-10-05: cells of an edge's way added to the chain objective beside the 10000 per eighth turn. Measured at 1 (`artifacts/logs/length`): 17q insertion 8.7 → 20.4 s for `bad` 4 → 3, 69q 88 → 276 s, angle 180 → 246, an edge undrawn, `bad` 1 → 2 — the turn-only bounds stop pruning. Off; see *What is not done* |
| `SCPD_SQUEEZE_REJECT` / `SCPD_SQUEEZE_TOLERANCE` / `SCPD_SQUEEZE_RECOVER` | **on** / 0 / on | new, 2026-10-05: every edge search refuses a way that leaves too little room beside it; an edge the commit cannot draw under the rule is drawn once more without it. 17q 12 → 4, 69q 4 → 1, no open wire left — see *The squeeze report* |
| `SCPD_FEEDLINE_LANE` | on | new, 2026-10-04: the lane polygon of a relaxation priced in the feedline pass; `=0` leaves it out, measured once at the user's request — see *The lane polygon, measured* below |
| `SCPD_OUTER_LANE` | on | new, 2026-10-04: the lane polygon of the outer routing's relaxation; `=0` leaves it out and the released halos alone price the relaxation. Measured on 57q under the hard length-point clearance, see *The length-point clearance* |
| `SCPD_OUTER_PENALTY` | 0 | new, 2026-10-04: how the outer routing prices the wires it let go of — 0 a halo of one clearance more per level, as it always had; 1 the feedline pass's flat halo (`SCPD_HALO_REACH` clearances) and crossing toll; 2 not at all |
| `SCPD_LENPOINT_RECOVERY` | on | new, 2026-10-04: the outer routing ends on one more pass over its failing wires with the length-point bands off — the hard clearance's fallback, after the rounds rather than inside them |
| `SCPD_LENPOINT_K` / `SCPD_LENPOINT_PCT` / `SCPD_LENPOINT_REPORT` | 40 cells / 10 / on | new, 2026-10-04: the length-point clearance of the outer routing and the outer refinement, hard — see *The length-point clearance* below; `K=0` is off |
| `SCPD_CROSSING_EXIT_HEADING` | off | new, 2026-10-04: the orthogonal search tests a move's end cell with its exit heading, as the count reads it; measured, the user decides — see [handover-targeted-repair.md](handover-targeted-repair.md) |
| `SCPD_REPAIR_SEARCH`, `SCPD_RESEARCH_K` / `GROW` / `SECONDS` / `ROUNDS` / `JOGS` / `SEES_CHAINS` / `FINAL_SWEEP` | on, 2 / 1 / 20 / 2 / on / on / off | new, 2026-10-04: the targeted repair that replaces `repair`; its own `targeted repair settings:` line names them |
| `SCPD_RESEARCH_VERBOSE` / `SCPD_PROBE_ONLY_UNSETTLED` | off / unset | new, 2026-10-04: the repair's local passes speak; a test seam for `Pass::onlyUnsettled` — see [handover-targeted-repair.md](handover-targeted-repair.md) |
| `SCPD_EDGE_BEND_FACTOR` | 1.0 | may now go below 1 |
| `SCPD_FEEDLINE_MEANDER` | off | |
| `SCPD_FEEDLINE_PROTOTYPE` | on | |
| `SCPD_FEEDLINE_FORCE_REROUTE` | on | |
| `SCPD_LANE_TRIM` | on | |
| `SCPD_PRICE_ADD` | on | |
| `SCPD_PRICE_RELEASED` | on, `SCPD_RELEASED_FACTOR` 1 | |
| `SCPD_PRICE_APPROACHES` | on (outer routing only) | |
| `SCPD_PORT_BANDS` | on | |
| `SCPD_CROSS_TOLL` | 150, width 2 | |
| `SCPD_HALO_REACH` | **1**, `SCPD_HALO_DECAY` 0 (decay off) | was 3 from 2026-10-01 to 2026-10-05. Measured once on 69q with the guard and three pairs (`artifacts/dev/69q-halo1`): `bad` 5 → 4 — 190/191 close, 183/184 stay open, 191 and now 15 cross an edge at 10 cells, the halo's-edge case; 4 wires either way, 550 s against 576. The user made 1 the default on that run; with the decay off it is the prototype's disc of one clearance |

`RESONATOR_COPPER` (2) and `COUPLER_LEAD_STRAIGHT` (14) are constants, not
switches. Three settings lines say what is in force and where each value
came from: `outer routing settings:` before the outer routing (length
point, recovery, polygon, released halos), `feedline routing settings:` at
every feedline pass (stub, fixed places, crossing rule, exit heading, bridge
check, lane polygon, the terminal edges' coupler runs and the cells beyond
them) and `targeted repair settings:` before the repair. The fence pairs
and the halo reach have no line; the `fence …` of every relaxation search
names the pairs, and the reach shows only in the pictures.

## The bridge check

Written 2026-10-04 (user). `assignBridges` tells every plain wire between
two couplers which chain edge it may cross — its bridge — and
`constrainByFeedlines` fences every other edge but the terminal ones. That
leaves a search two ways home: across the bridge at a right angle, or round
the chain's end through a terminal edge, which fences nobody, and so onto
the wrong side of the feedline without crossing anything the rule sees. On
33q wires 9 and 11 came home that way, and the lane pairs 9–12 then never
settled: a wire on the wrong side stands in the following rounds, and every
wire drawn after it is drawn against it.

`SCPD_BRIDGE_CHECK` (on) refuses such a way in `attempt`: a found way that
does not cross the wire's bridge (`waysCross` against the bridge's way, in
phase 1 and at every relaxation level) is cleared as if the search had
found nothing, so the wire fails where it stands rather than poisoning the
rip-up. The pass says how often at its end — `feedline routing: N ways
refused for not crossing the wire's bridge` — and names each refusal at
`-v 1`. A wire without a bridge, and a wire whose bridge is not drawn, is
not held to it. Measured over the eight chips with `stop_after =
"feedlines"` and no repair (`artifacts/logs/bridge-check` against
`base-ortho2`, 2026-10-04), open / crossing at the end of the stage and
`bad` = their sum (unrouted is 0 everywhere), the ways refused, and open in
the last round:

| chip | base-ortho2 | bridge-check | refused | open, last round | CPU s |
|---|---|---|---|---|---|
| 4q | 0 / 0 (0) | 0 / 0 (0) | 0 | 0 → 0 | 2 → 2 |
| 9q | 0 / 0 (0) | 0 / 0 (0) | 0 | 0 → 0 | 10 → 11 |
| 17q | 4 / 4 (8) | 4 / 4 (8) | 20 | 4 → 4 | 35 → 32 |
| 21q | 0 / 0 (0) | 0 / 0 (0) | 4 | 0 → 0 | 53 → 48 |
| 33q | 4 / 1 (5) | 2 / 2 (4) | 6 | 3 → 1 | 123 → 85 |
| 45q | 6 / 2 (8) | 7 / 1 (8) | 9 | 2 → 3 | 449 → 490 |
| 57q | 17 / 8 (25) | 15 / 8 (23) | 39 | 9 → 7 | 609 → 483 |
| 69q | 24 / 7 (31) | 22 / 6 (28) | 25 | 10 → 10 | 938 → 837 |
| **sum** | **77** | **71** | 103 | 28 → 25 | |

No chip is worse; 33q's bundle 9–12 shrinks to 12/13 (12 keeps its outer way
and now counts as crossing f2), 45q's 132 fails instead of taking its way
round f46 and ends open instead of crossing, 57q and 69q lose two open
wires each. On 17q the twenty refusals are the lane pairs 18/19 and 32/33
at relaxation levels 2 to 5, round after round; the figures do not move,
but the forward rounds no longer carry the follow-on fails f5, f10, 20 and
34 that stood behind those detours. The pass is faster where it refuses
much: a refused way is a search not followed by the rounds it would have
cost.

## The length-point clearance

Written 2026-10-04 (user). The coupler insertion puts a resonator's pad
where what is left of the way to the qubit is the target length
(`couplerPlace`), and the pad and the lead stick out beside the way there.
A plain wire the outer routing laid too close to that place is what the
insertion and the feedline pass fail on afterwards — the room rules found
exactly that after the fact (R3, the lane pairs leaning on a pad). The
prototype's answer is the RRR-Lenpoint-Constraint of its outer routing
(`FinalGrid.cpp:6193-6600`, its refinement `:7620-7760`;
`plan-resonator-lenpoint.md` is the analysis). This is that mechanism, for
the **outer routing and the outer refinement only** — under the feedline
constraints the couplers stand and the lead and pad are fenced themselves —
and **hard**: a wire that cannot be routed with the clearance fails; the
prototype's second try without it is deliberately not built (user).

- `lengthMarksOf(wire)`: on the way a resonator has now, the cell where what
  is left to the target end is the target length less the anchor gap — the
  prototype's `meander_length − anchor`, plainly, nothing derived (user) —
  and the band of cells within `SCPD_LENPOINT_PCT` of it, measured as the
  insertion measures (`reconstructSegments` on a drawn way, the cells'
  polyline on a seeded one). The pad the insertion then centres lies the
  lead's length nearer the target end, inside the band. The band, not the
  point: the point moves with every reroute and every meander.
- `closeLengthBands(wire, others, pass)`, called after every `fence` of
  `attempt` (phase 1: both ring neighbours; each relaxation level: the
  neighbour on the far side only — the wire let go of at that level is
  fenced by its way alone, its band goes with it, as the prototype's does
  since its stamp lands before the corridor buffer is reset) and of
  `refine`, outside the feedline passes: a fenced neighbour that is a
  resonator has its band inflated by `SCPD_LENPOINT_K` instead of the
  clearance; a resonator being routed has the stretch of each fenced
  neighbour that runs alongside its own band — its two marks projected onto
  the neighbour's way — inflated by the same radius; **one zone per
  neighbour** (user): a resonator neighbour gets its band and nothing else,
  a plain neighbour the alongside stretch only — a plain wire beside a
  routed resonator used to carry both and showed two inflated zones in the
  pictures. Cells within the larger of `k + 2` and the meeting radius of the
  routed wire's own **fixed places** — its ends, the run out of its source, the run into its target —
  are left out, the two wires' meeting too. The prototype guards the two
  ends alone and falls back to a search without the constraint; here it is
  hard, and the first version with the ends alone left 17q's wires 2 and 31
  dead on arrival in every round: the length point of the resonator beside
  them lies 24 cells from their launcher stub, and a band cell 30 cells from
  the source, inflated by `k`, closed the cells the search has to step onto
  behind the stub. With the fixed places guarded both route again in some
  rounds; what the hard constraint still costs the outer routing is in the
  table below.
- `k` is the prototype's flat **40** cells (user: as the prototype has it).
  A first version derived it from the pad — `couplerHeight + clearance +
  BEND_RADIUS`, 27 on the benchmarks — and that arm is kept for 4q–45q as
  `artifacts/logs/lenpoint-k27`. The `outer routing settings:` line says
  which is in force.
- `outer routing: LENPT k=… band=±…%: N length points, nearest other wire
  min … mean … cells, M below k; histogram <20:… 20-25:… 25-30:… 30-40:…
  40-60:… 60-100:… >=100:…` ends the outer refinement — the prototype's
  `[LENPT]` report at fixed edges, so that arms compare — with one line per
  resonator at `-v 1`.

Measured over the eight chips with `stop_after = "feedlines"` and no repair
(`artifacts/logs/lenpoint40` against `bridge-check`, the build it was added
to, 2026-10-04). Three figures per chip, as the user asked: the outer
routing's own verdict (unrouted / open after the refinement), the
insertion's feedline angle cost with the edges it could not draw, and
`bad` = unrouted + open + crossing after the feedline routing with no
repair; beside them the length-point line's nearest-wire figures and the
seconds of the whole stage.

| chip | outer unrouted/open | edges not drawn | angle (before) | LENPT min / mean / below k | bad (before) | s (before) |
|---|---|---|---|---|---|---|
| 4q | 0 / 0 | 0 | 12 (8) | 80 / 81 / 0 | 0 (0) | 2 (2) |
| 9q | 0 / 0 | 0 | 20 (20) | 90 / 97 / 0 | 0 (0) | 10 (11) |
| 17q | **1 / 1** | 0 | 49 (49) | 48 / 63 / 0 | 12 (8) | 24 (33) |
| 21q | 0 / 0 | 0 | 50 (50) | 42 / 70 / 0 | 0 (0) | 51 (49) |
| 33q | 0 / 0 | 0 | 76 (74) | 50 / 70 / 0 | **0** (4) | 57 (86) |
| 45q | 0 / 0 | 0 | 109 (111) | 44 / 61 / 0 | **5** (8) | 157 (882) |
| 57q | **0 / 23** | **1** | 146 (144) | 1 / 52 / 8 | 23 (23) | 372 (486) |
| 69q | 0 / 0 | 0 | 176 (176) | 38 / 57 / 1 | **16** (28) | 682 (852) |
| **sum** | | 1 | | | **56** (71) | 1354 (2400) |

What it says:

- **The outer routing is no longer at 0 fails everywhere.** On 17q the
  hard clearance leaves resonator 30 unrouted and one wire open: 30's own
  band and its neighbours' leave it no lane. On 57q the outer routing ends
  with 23 open wires (35 unrouted after round 0, 23 open after five rounds
  — the pairs that trade a lane inside a band), and the insertion then loses
  edge f4 of chain 0. Six chips keep 0 / 0.
- **Where the outer routing holds, the feedline pass gains**: 33q to 0, 45q
  from 8 to 5, 69q from 28 to 16 — the length points there have no wire
  closer than 38 cells, where the baseline had plain wires at 1 to 10 cells
  from leads (R3). 57q's 23 is unchanged in sum but has an edge missing.
- **The angle** moves by the layout, not by the rule: 4q 8 → 12, 33q 74 → 76,
  45q 111 → 109, 57q 144 → 146, the rest equal.
- **The stage is faster** wherever the ring has room: 1354 s against 2400 s
  over the eight, 45q 157 s against 882 s — fewer relaxations in the
  feedline pass once the pads have their room.

A first version derived `k` from the pad (27 cells) and subtracted the lead
from the figure (`artifacts/logs/lenpoint-k27`, 4q–45q): outer routing 0 / 2
on 17q and 0 / 0 elsewhere, 33q to 0, but 21q from 0 to 4 and 45q from 8
to 10. The user set `k = 40` and the plain band instead.

**57q's outer routing, the evening's matrix** (`artifacts/logs/outer-*`
and `outer57-*`, `stop_after = "outer"`, k 40 hard; "band" is the band of
the wire let go of at a relaxation level):

| pricing of the relaxation | band | relaxation | outer routing |
|---|---|---|---|
| polygon + halos per level (as it was) | stamped | 5 | 23 open |
| halos per level alone | stamped | 5 | 12 open |
| polygon + flat halo and toll | stamped | 5 | 19 open |
| flat halo and toll alone | stamped | 5 | 35 open |
| halos per level alone | free | 5 | **0** |
| halos per level alone | free | 8 | **0** |
| polygon alone, no halos | free | 5 | 28 open, 1 unrouted |
| polygon + halos per level | free | 5 | 18 open, 1 unrouted |
| polygon + halos per level, one zone per neighbour | free | 5 | 18 open, 1 unrouted |
| **polygon + halos per level, one zone per neighbour** | free | **8** | **0**, min 40 cells |

Two things close 57q: letting the band of the wire let go of go with it
(stamped at its own level it kept the released lane shut), and either the
polygon left out at relaxation 5 or the polygon kept at relaxation 8. The
user chose the original pricing — polygon and per-level halos — with
relaxation 8 and one zone per neighbour; `artifacts/logs/outer-orig-r8` is
the eight chips under it (seven at 0 / 0, 17q 1 / 1), `outer-noband` the
eight chips under the halos alone at relaxation 5 (six at 0, 17q 1 / 1,
21q 0 / 3).

**17q closes with the recovery** (`SCPD_LENPOINT_RECOVERY`, on): resonator
30 cannot route under its own band — the alongside stamp inflates both
plain neighbours by `k` and the lane between them is too narrow — so the
outer routing ends on one more pass over its failing wires with the bands
off (`recoverWithoutBands`, before the refinement). On 17q that pass draws
the three wires the rounds left (`tried 3, routed 3`), the outer routing
ends at 58 of 58 with 0 / 0, and the nearest wire to any length point is
still 41 cells (`artifacts/logs/outer17-recovery`). The hard constraint
holds inside the rounds; the recovery is its fallback after them, where the
prototype has it inside (its second `process_wire` without the constraint).
`artifacts/logs/outer-recovery-r8` is the eight chips under the whole
setting.

**The whole stage under that setting** (`artifacts/logs/full-r8`, the eight
chips to `stop_after = "feedlines"`, no repair, `max_relaxation` 8 in the
run directory's copy — the benchmark configs still say 5 — against
`bridge-check`, the same build before the length-point work):

| chip | outer unrouted / open | edges not drawn | angle (before) | unrouted / open / crossing | bad (before) | s (before) |
|---|---|---|---|---|---|---|
| 4q | 0 / 0 | 0 | 12 (8) | 0 / 0 / 0 | 0 (0) | 2 (2) |
| 9q | 0 / 0 | 0 | 20 (20) | 0 / 0 / 0 | 0 (0) | 10 (11) |
| 17q | 0 / 0 after the recovery | 0 | 49 (49) | 0 / 6 / 6 | 12 (8) | 36 (33) |
| 21q | 0 / 0 | 0 | 50 (50) | 0 / 0 / 0 | 0 (0) | 51 (49) |
| 33q | 0 / 0 | 0 | 76 (74) | 0 / 0 / 0 | **0** (4) | 58 (86) |
| 45q | 0 / 0 | 0 | 109 (111) | 0 / 3 / 1 | **4** (8) | 240 (882) |
| 57q | 0 / 0 | 0 | 132 (144) | 0 / 0 / 2 | **2** (23) | 211 (486) |
| 69q | 0 / 0 | 0 | 178 (176) | 0 / 15 / 1 | **16** (28) | 669 (852) |
| **sum** | | 0 | | | **34** (71) | 1276 (2400) |

Every outer routing at 0, every edge drawn, `bad` 71 → 34, the stage half
the time. 17q is the one chip that loses — its feedline pass ends with
three open edges (f0, f1, f12) and six crossing wires where the baseline had
four and four — and 69q's fifteen open include f5 and f6. The lengths are
not read (user).

## The terminal edges' coupler runs

Written 2026-10-05 (user). A terminal edge runs from a launcher to the
first coupler of its chain or from the last coupler to a launcher, and the
prototype fences it for nobody; `constrainByFeedlines` follows it. Its
launcher end is walled by the port band in the obstacle mask. Its coupler
end is not a port of the chip and had no wall at all: the run the edge has
to arrive on at the first coupler, and the run it has to leave the last
coupler on, were open to every wire that is not a ring neighbour of the
edge, and a wire drawn across them leaves the edge no way when it is drawn
again. 69q's f5 and f6 ended open there, 17q's f0, f1 and f12 do not
(their conflicts lie 69 and 77 cells from the coupler, and f12's is a
resonator that finds no way in any round and keeps its seed).

`SCPD_TERMINAL_STUB_GUARD` (on) closes that run — `endStub` into the first
coupler, `startStub` out of the last — plus `SCPD_TERMINAL_STUB_EXTRA` (5)
cells beyond it, inflated by the clearance, in every search of the
feedline pass but the edge's own, drawn or not (`guardTerminalStub`, from
`constrainByFeedlines`); the meeting at the coupler stays open as
`closeRoomOf` leaves it. The insertion closes the same runs of every chain
already settled in the corridor of every later edge search
(`guardSettledTerminalRuns`, from `corridorOfEdge`), whatever
`SCPD_EDGE_SEES_CHAINS` says. Measured on 69q with the benchmark config
(5 rounds, relaxation 8, band 209), no repair, `bad` = unrouted + open +
crossing at the end of the stage:

| 69q | insertion pairs crossing | bad | open | crossing | s |
|---|---|---|---|---|---|
| no guard (`artifacts/dev/69q-stubguard-off`) | 6 | 14 | 13 | 1 | 645 |
| no guard, 6 rounds (`full-r8`) | 6 | 16 | 15 | 1 | 669 |
| guard in the feedline pass (`artifacts/dev/69q-stubguard`) | 6 | **10** | 9 | 1 | 884 (`-d`) |
| guard in the pass and the insertion (`artifacts/dev/69q-stubguard-ins`) | 4 | **10** | 9 | 1 | 912 (`-d`) |

f5 and f6 end drawn and clear in both guard arms, and the `CHECK feedline
crossings` line after the pass is green. What is left is plain-wire pairs
alone: 20/22, 183/184, 190/191, 203/206/208, with 191 crossing f65. The
insertion guard takes two of the six crossing pairs of the insertion away
(f32/f33, f73/f74) and changes nothing at the end; f5 and f6 still cross
at the insertion, away from the coupler end, which the guard does not
reach. 17q is identical in every judged line with the guard in the pass
(the insertion guard was not run there). `artifacts/69q` holds the
feedline-pass-guard run since 2026-10-05 (`06-final.fb`, `run.log`,
`debug/`), at the user's request.

**More ring neighbours fenced** (`SCPD_FEEDLINE_FENCE_PAIRS`, user,
2026-10-05), on top of the guard in the pass and the insertion, same
config, no `-d`:

| 69q | bad | open | crossing | last round failed | s |
|---|---|---|---|---|---|
| 2 pairs (the prototype) | 10 | 9 | 1 | 20, f6, 184, 191 | 912 |
| 3 pairs (`artifacts/dev/69q-pairs3`) | **5** | 4 | 1 | 184, 191 | 576 |
| 4 pairs (`artifacts/dev/69q-pairs4`) | **5** | 4 | 1 | 184, 191 | 565 |

Three and four end on the same wires in every round — 183/184 and
190/191 open, 191 crossing f65 — and the pass is a third faster, because
round 0 fails 10 instead of 17 and the later rounds have less to redo.
On 17q three pairs change nothing (`bad` 12). **3 is the default** (user,
2026-10-05); `=2` is the prototype's fence, kept as the control arm.

Two things tried the same day and found not to be the cause of the round
count trading fails (6 rounds 16, 5 rounds 14): the fence beyond the wire
let go of walked on to the next conventional wire
(`SCPD_FENCE_TO_CONVENTIONAL`, identical in every judged line), and the
band raised from 11 to 16 clearances (`artifacts/dev/69q-corridor16`,
`bad` 16 at 5 rounds).

## The squeeze report

Written 2026-10-05 (user). An experiment, not a rule: the idea is that
what is left open at the end of the stage is edges that leave the wires
beside them too little room, and that a bottleneck detection at the end of
the insertion could find those edges before the feedline pass runs into
them — where the repair's trials cost the runtime. `reportSqueeze` runs
after the four checks and the room report, prints one line per edge at
`-v 1` and a `coupler insertion: SQUEEZE — N of M edges …` summary, and
marks the edges in the artifact: `FinalWire.verdict` carries `Squeezed`
(bit 64), `note` the figures and `marks` the two ends of the worst line,
on every snapshot from the `couplers` phase on. `plot --stage final
--phase couplers` draws them dashed magenta (`l-squeezed`), the measured
line solid with a dot on the wall, the note in the tooltip and `squeezed
(N)` in the legend; `render` puts both on layer 30 `final.squeezed`.

The measurement: from every `SCPD_SQUEEZE_STEP` (5) cells of an edge's
way, its two runs at the couplers skipped, a straight line to either side
at a right angle to the edge's heading, until the first cell of the
artwork **inside the chip** — a qubit or a tunable coupler — which is the
wall. Not a launcher pad: those lie outside the rectangle the launcher
cells span, and a line that leaves that rectangle measures nothing (the
first version measured against them and drew most of its lines to the
border, user). Not a CPW coupler body the insertion put there either
(user). `SCPD_SQUEEZE_REACH` (300) cells without a wall measure nothing.
Every distinct wire whose copper the line crosses (`field.owner`, the two
axis neighbours of a diagonal step included) has to pass through that
channel; `k` of them need `clearance + k · pitch` cells (19 + 20k on the
benchmarks), and a line shorter than that is short by the difference. The
worst line of an edge is its figure. `SCPD_SQUEEZE_REPORT=0` turns the
report off.

What it found, against the fails of the same run (the day's defaults,
`artifacts/logs/squeeze-inner`; the pictures are
`artifacts/<chip>/<chip>-final-couplers.svg` and `-final.svg`):

| chip | edges marked | fail sites on a marked edge | fail sites not marked | marked without a fail |
|---|---|---|---|---|
| 17q | 3 of 20: f1, f12, f18 | f1 (11 in 21 cells, short by 18: f0, f1, 11 open), f12 (40 in 16, short by 23: 40 open and crossing), f18 (1 and 2 in 44, short by 15: 2 crossing) | f2 (14), f8 (31), f9 (33), f17 (56), all crossing at 10 cells | none |
| 69q | 1 of 81: f63 | f63 (179–184, six wires in 132 cells to the artwork, short by 7: 183/184 open) | f3 (15), f65 (191), both crossing at 10 cells | none |

Every marked edge carries a fail and every open wire of the two chips
lies on a marked edge; what the report does not see is the crossing
fails at 10 cells from an edge, the halo's-edge case of
`SCPD_CROSSING_EXIT_HEADING`, which is not a question of room. The
version that took launcher pads and coupler bodies as walls marked 9 of
20 on 17q and 5 of 81 on 69q, four and four of them without a fail
(`artifacts/logs/squeeze`, `squeeze-marks`).

**The rule** (`SCPD_SQUEEZE_REJECT`, on, user, 2026-10-05 afternoon):
every edge search — the prefix search, the memo fill, the commit, the
targeted repair's re-search, all through `routeEdge` — measures the way
it found with `measureSqueeze` and clears it as no way when the shortfall
is above `SCPD_SQUEEZE_TOLERANCE` (0), as the bridge check clears a way
that misses its bridge; a way the greedy chose is held to it at the
commit as well (`refuseSqueezed`). An edge the commit then cannot draw
is drawn once more with the rule suspended (`SCPD_SQUEEZE_RECOVER`, on)
and counted — an edge not drawn is worse than one that is squeezed. The
`SQUEEZE` line says how many ways were refused and how many edges came
through the recovery. Measured against the report-only run of the same
day (`artifacts/logs/squeeze-reject` against `squeeze-inner`):

| chip | ways refused | recovered | insertion s | angle | `bad` before → after | left |
|---|---|---|---|---|---|---|
| 17q | 121 | 0 | 4.0 → 9.5 | 49 → 55 | 12 → **4** | 14, 31, 33, 56 crossing at 10–14 cells |
| 69q | 7 | 0 | 90 → 88 | 180 → 180 | 4 → **1** | 15 crossing at 10 cells |

No open wire is left on either chip, and no edge is squeezed at the end;
69q's feedline pass ends after round 3 with nothing failing, the whole
run 515 s against 569. The cost is the chain search on 17q, where the
rule refused 121 ways and the insertion took 9.5 s instead of 4.0 and
the angle cost went 49 → 55; on 69q the time-boxed chains expanded two to
five prefixes more and ended on the same costs. What is left is the
crossing fails at the halo's edge, which no room rule reaches.

**A defect the user found in that arm** (2026-10-05): 17q's chain 1,
whose edges were squeezed by nothing in the report-only run, changed its
options under the rule and paid 100000 → 140000. The refused ways named
wires 26 and 30 — chain 1's **own resonators**. A resonator whose coupler
is not applied yet still holds its whole outer way in the field, the
tail past the anchor included, and `measureSqueeze` read that tail as a
wire in the channel. It now counts such a resonator only where the cell
lies on `wayOfResonator`, the way as the chosen option cuts it
(`squeeze-reject2`): chain 1 is back at 100000 on the options it had,
17q refuses 81 ways instead of 121, the angle is 51, `bad` stays 4. The
eight-chip table of `squeeze-reject2` is in *Where it stands*.

## The lane polygon, measured

Asked for by the user on 2026-10-04: what the feedline pass loses without
the lane polygon of its relaxation — `fillLane` over `laneOf`, the
prototype's `compute_corridor_polygon_proximity`, everything outside the
lane between the two neighbours' ways priced. `SCPD_FEEDLINE_LANE=0` leaves
it out and keeps the halos and the toll of the wires let go of
(`artifacts/logs/nolane` against `lenpoint40`, the same build otherwise,
eight chips, no repair):

| chip | with the polygon: unrouted / open / crossing (bad) | without (bad) | CPU s with → without |
|---|---|---|---|
| 4q – 33q | as with | identical | ≈ |
| 45q | 0 / 3 / 2 (5) | 0 / 3 / 2 (5) | 156 → 168 |
| 57q | 1 / 16 / 6 (23) | 1 / 16 / 6 (23) | 370 → 390 |
| 69q | 0 / 9 / 7 (16) | **0 / 14 / 6 (20)** | 675 → 750 |
| sum | 56 | 60 | |

Seven chips end on the same figures (the searches differ, the keylines are
not identical, the verdicts are); 69q loses five open wires to it and gains
one crossing. The polygon stays on.

## The traps

Five real defects were found on 2026-10-02 and three more on 2026-10-03.
Each was invisible in the numbers until something else was loosened, which
is the reason to write them down.

**The proximity field was detached.** `routeEdge` attaches a field of zeroes
to the router and never attached the real one back. The coupler insertion runs
before the feedline pass, so from its first edge onward *every* later search
priced against zeroes. This is why four price sweeps, the crossing toll at
every value and width, and the band widening had all done nothing. The fix is
one line in `search()`: say which field this search is priced by every time,
rather than once.

**The coupler pads did not exist during the option search.** `bodies_` is
cleared at the start of the insertion and filled by `applyOption`, which runs
at the **commit**. So the search solved against a chip with no pads on it at
all, while the commit builds on one with every pad standing. Now `foreignRoom_`
carries the foreign pads and this chain's are stamped per edge. Measured: it
changes not one decision on any of the eight chips — the pads never lay on a
way the search wanted. The defect was real, its consequences were not.

**Kept ways were never re-tested.** `SCPD_CHAIN_KEEP_WAYS` was on, and its own
comment says what that gives up: *"the way the search found is taken as it is,
crossings and all"*. `checkFeedlineRoom` caught it on 69q — edge f73 of chain
10, kept as the search found it, ran inside the rule of resonator 218, ten
cells from that resonator's own coupler. One pair on eight chips. Routed again
instead, the check is green and the stage ends on the same numbers.

**The search and the commit did not ask the same question.** The prefix search
prices edge `k` against edges `0..k-1` and nothing else, because the later ones
do not exist yet when it is priced. The commit routes edge `k` against all of
them — the later ones stand on the ways the search found, and `edgeWayOf`
hands them out. So a run of options the search **proved complete** can be one
the commit cannot build, and `stateFaults` is the count of exactly that. Seen
on 45q under `SCPD_EDGE_LEAD_ONLY`: chain 7 settled, `stateFaults` said one
edge would not survive, and f47 — the third of seven — came back with no way.
In the search it was fenced against edge 1 alone; at the commit against 1, 3,
4 and 5. `SCPD_FENCE_LATER_EDGES=0` makes the commit reproduce the search.

This one is worth understanding properly, because it looks like a
contradiction and is not: **loosening the obstacles cannot make a feasible
solution infeasible**. On 45q both fences reach the same chain cost of 140000.
What changes is *which* of the equally-priced runs the search keeps — with
more options open it walks the space differently and keeps one that only
exists through the end-pair reorder, and that one is not commit-feasible. The
objective the search optimises is not "survives the commit".

**The coupler box left the launchers one cell too little.** It was drawn on
the stub itself, so a coupler could sit with its feedline port one cell outside
a stub's clearance. `SCPD_LAUNCHER_LEAD` adds ten layout units to the straight
run the box keeps clear. On 17q that is one cell and it closed both open
wires; on the other seven it changes nothing, and a sweep over the lead
(0 → 2 open, 10 → 0, 30 → 2, 60 → 0, 100 → 0, 200 → 4) shows it is not
monotone. It moves where a coupler may sit, so it reshuffles the chain rather
than mending a mechanism. Keep the expectations low.

**A search does not start at its source.** `DubinsRouter::sanitize` moves
the source forward along its heading by the start stub, and `straightStub`
runs *forward* from the source too — the stub is ahead of the source, not
behind it. A resonator's search therefore began 25 cells after its turn, 14
of lead and 11 of stub, where the prototype's begins after 14. Whoever reads
"the source is open" has looked at the wrong cell; the `dead on arrival`
line looks at the right one (2026-10-03).

**A fence that covers the way covers less than the wire needs.** `fence`
closes the clearance around `other->way`, and a wire's fixed places — lead,
start stub, end stub — are in its way only once a search has drawn it. The
way the insertion splices turns off five cells after the lead's tip, so the
stub's eleven cells were in no way, charged in no field (`fixPlaces` charges
the fixed places only while a wire is not down), and open to every
neighbour. `SCPD_FENCE_FIXED` closes them (2026-10-03).

**The `in the way` line fences neither neighbour of the second pair**, in
any of its five attempts, and puts the corridor back without them too. On
every resonator that was dead on arrival it said `the neighbours`, and on
131 in round 0 it said all four at once — both verdicts were the same ±2
fence it never built. Read it as "something in the corridor the line does
not model", and read `dead on arrival` first. The pictures are no help
there either: only `frame_.fence` wires get the grey band, the ±2 fences,
the feedline fences and the coupler bodies are closed but drawn as plain
white, and a fence wire is drawn in the feedlines' orange (`fw`). And the
`IN ITS OWN STUB` flag of the `is open in the room of` line fires at the
target end as well, where a ring wire has no stub (2026-10-03).

## Negative results

Written down so nobody pays for them twice.

- **Raising the wire proximity** — the general one, the released-wire factor,
  and the crossing toll at every value and width — does nothing, and the
  minima sit at the defaults. All of it was measured **before** the detached
  field was found, so it is worth one re-measurement before trusting it.
- **Lowering the bend penalty** of the outer routing made things worse at
  every factor tried.
- **Fencing the whole resonator with no coupling zone** makes 33q unsolvable:
  0 of 7 chains settled, 7 of 40 edges drawn, and the search never brings one
  run of options to the end of a chain. The clearance band is 19 cells and the
  slot cannot carry an edge out of it.
- **Longer terminal slots** do not help past the first few cells, and they
  make the room check red: 26 → green, 40 → one pair, 60 → two.
- **More search time** for a chain was never measured. On 69q chain 0 runs out
  of its ten seconds without one complete run of options, so
  `SCPD_CHAIN_ASTAR_SECONDS=30` is the obvious next thing to try.
- **The fixed places fenced, the second stub kept** does nearly nothing on
  69q — 19 open at the end against 20, 8 in the last round against 7 — and
  takes 57q from 19 to 3. **The stub dropped, the ways fenced alone** opens a
  new pair on 69q (175/176) and leaves resonator 9 on its seed, which touches
  f1. Neither half stands alone; measured 2026-10-03, table in *The
  resonator's exit*.
- **The band, the number of fences and the prices were compared with the
  prototype's, not swept**, when the fourteen resonators were traced: band
  798 against its 800, the same ±1 and ±2 at every level, its `proxy_fact`
  of 1 weaker than our halo and toll. They were not the cause. The
  re-measurement of the prices the first bullet asks for is still owed.

## Where the pieces are

- `src/pipeline/FinalRouter.cpp` — all of it. `corridorOfEdge` is the
  obstacle construction, `ensureForeignRoom` the grid, `checkFeedlineRoom`,
  `checkCouplerCrossings`, `checkResonatorCrossings` and
  `checkFeedlineCrossings` the four checks, `waysCross` the crossing test,
  `repairFeedlines` → `researchSegments` / `blameFails` / `researchSegment` /
  `tryRun` the targeted repair and `Pass::onlyUnsettled` the local pass it
  sweeps with, `applyOption` the lead trim and the resonator's stub
  (`resonatorStub`), `fence` and `closeRoomOf` the fence of ways and fixed
  places (`fenceFixed`), `fenceFixedPlaces` the heads of the wires let go
  of, `deadOnArrival` and `whatIsNear` the line that says a search was over
  before it began. Of 2026-10-04 evening: `refuseTheDetour` in `attempt` is
  the bridge check; `lengthMarksOf`, `closeBandCells`, `closeLengthBands`,
  `recoverWithoutBands`, `sayLengthPoints` and `sayOuterSettings` the
  length-point clearance; `priceLane` carries `outerLane`, `outerPenalty`
  and `feedlineLane`. Of 2026-10-05: `guardTerminalStub` (from
  `constrainByFeedlines`) and `guardSettledTerminalRuns` (from
  `corridorOfEdge`) are the terminal edges' coupler runs;
  `feedlineFencePairs` the pairs fenced in phase 1 and the relaxation of
  `attempt`, where the walk of `fenceToConventional` follows them;
  `Fails::verdicts`, `wireOf` and `snapshotOf` the verdict in the
  artifact; `measureSqueeze`, `refuseSqueezed` (from `routeEdge` and the
  commit), `reportSqueeze` and `Driver::Squeeze` the squeeze rule and
  report, its marks carried by the same `snapshotOf`.
- `artifacts/logs/` — the arms and the tools that read them: `run-arm.sh
  <arm> [ENV=…]` (the eight chips one at a time on the installed binding;
  `REPAIR_TRIALS=`, `STOP_AFTER=`, `MAX_RELAXATION=` edit the run
  directory's config copy, `CHIPS=` restricts the chips), `dev-sync.sh` /
  `dev-run.sh` / `dev-mqt-scpd.py` (the same on the build tree's binding
  through an overlay, so a build can be tried while an arm runs),
  `keylines.sh` (the lines an arm is judged by, for identity), `arm-table.py`
  (outer verdict, angle, length points, `bad` per chip), `repair-summary.py`
  (the repair's lines), `calibration.py`, `fcross.py`, `pinch.py`. The arms
  of 2026-10-04 are `base-ortho2`, `repair-old`, `research`,
  `research-exit`, `exit-base`, `bridge-check`, `nolane`, `lenpoint-k27`,
  `lenpoint40`, `outer-*`, `outer57-*`, `outer-recovery-r8`, `full-r8`;
  of 2026-10-05 `guard-pairs3-halo1` (the eight chips on the day's
  defaults, four of them at relaxation 5), `guard-pairs3-halo1-r8` (17q,
  21q, 45q, 57q at 8) and `guard-pairs3-halo1-all` (the two joined, the
  table in *Where it stands*), with `bad-table.py <arm> …` printing
  `bad` and the wires per chip. The single-chip arms of the day are run
  directories under `artifacts/dev/69q-*` (`stubguard`, `stubguard-off`,
  `stubguard-ins`, `pairs3`, `pairs4`, `halo1`, `corridor16`), each with
  its `run.log` and, where `-d` was on, its `debug/`.
- `plan-coupler-repair-search.md` and `plan-resonator-lenpoint.md` — the
  two plans of 2026-10-04, each with the analysis it rests on.
- **The failing wires in the picture** (2026-10-05, uncommitted): the end
  state of `06-final.fb` carries `FinalWire.verdict`, the bits of
  `FinalVerdict` (unrouted, open, crossing, short, long, meeting itself) as
  `failsOf` counted them over every wire for the `final routing:` line; the
  phase snapshots carry zero. `mqt-scpd plot --stage final` draws every
  wire with an unrouted, open or crossing bit in red over the picture
  (`l-failing`), names it and its verdicts in a tooltip (`wire 40: open,
  crossing`, the ids of the log) and counts them in the legend; `render`
  puts them on layer 29 `final.failing`. The lengths are carried and named,
  not marked. A `06-final.fb` written before this carries no verdict and
  marks nothing — the stage has to run again on the new binding.
- `python/mqt/scpd/debugview.py` — wraps a debug SVG in an HTML page with
  per-class layer toggles. In the tree since `8941853`.
- `python/mqt/scpd/dashboard.py` — parses a `-v 1 -d` log into a table of
  which wire failed in which round and relaxation, with the failures linking
  to the viewers. In the tree since `8941853`. Written with `-d`, into
  `<run>/debug/dashboard-{feedline,outer,inner}.html`; see *The dashboards*.
- The log lines to read first, all at `-v 1`: `the ring the feedline pass
  sweeps` (the order, resonators marked `(r)`, terminal edges `[F]`/`[L]`),
  `feedline routing settings`, the `wire … · round … · normal` and
  `relax N` lines with `dead on arrival` where it applies, the round lines,
  the `N failed:` lines after the pass, and `is open in the room of`.
- The prototype is
  `/Users/michaelfeldmeier/Documents/GitHub/FridgeCAD/include/fiction/layout/FinalGrid.cpp`.
  `run_final_routing_feedline_parallel` is the pass this one mirrors;
  `free_terminal_stub` at `:20160` and the coupler edge parameters at `:4963`
  are the two places this document quotes.

## What is not done

- **What is left is eight crossing wires at the halo's edge** and nothing
  else — 17q 14, 31, 33, 56; 21q 27; 57q 15, 171; 69q 15 — see the table
  in *Where it stands*. The squeeze rule took 17q from 12 to 4 (its three
  open edges f0, f1, f12 and the open pairs are gone) and 69q from 4 to 1;
  the one lever on the eight is `SCPD_CROSSING_EXIT_HEADING`, the user's
  decision. 17q's outer routing is at 0 only through the recovery; what
  the recovered resonator 30 does to the pass has not been read.
- **The relaxation is 8 in every config now** (user, 2026-10-05) and 69q
  runs five rounds; `rounds` on the other seven is still 6. 57q at
  relaxation 5 ends at 13, so the 8 is not optional there.
- **The targeted repair has not run on any of the day's outer routings.**
  Its arms (`research`, `research-exit`, 20 tests a chip) stand on the
  2026-10-04 morning's: 69 and 51 against the 77 of then. On today's 20 it
  is owed again, as are the user's three decisions — the exit-heading
  default, the strict window criterion, and the sweep over k / grow /
  trials / the 20 s clock per segment. Note that the repair's local pass
  fences three pairs now, as the feedline pass does.
- **`SCPD_HALO_REACH` 1 rests on one 69q run** (`artifacts/dev/69q-halo1`):
  5 → 4 there, and the eight-chip table was run at 1 from the start, so
  what the reach alone does on the other chips is not separated out. The
  re-measurement of the prices that *Negative results* asks for is still
  owed; with reach 1 the halo is the prototype's disc, so the one price
  that is ours now is the toll.
- **69q's length point at 20 cells.** One resonator ends the outer routing
  with a wire 20 cells from its length point (the `LENPT` line, 1 below k):
  a wire the recovery or the refinement laid there, or a band that
  collapsed onto its point. Not read.
- **Feedlines of different chains cross each other on 69q at the end of the
  insertion** — f5/f6, f45/f47, f46/f47, f60/f61 since the insertion closes
  the terminal edges' coupler runs (f32/f33 and f73/f74 went with that),
  `CHECK feedline crossings` red there — and the pass redraws the edges so
  that the second line, after the repair, is green. `SCPD_EDGE_SEES_CHAINS`
  is off, so the insertion itself never routes an edge against another
  chain; f5 and f6 cross away from the coupler end, which the guard does
  not reach.
- **Lengths are not read** (user): `SCPD_FEEDLINE_MEANDER` is off and the
  short and long columns are nobody's figure in this phase.
- **The orthogonal crossing rule is on** (`SCPD_ORTHO_CROSSING`), and the
  DRC's `checkOrthogonality` leaves the terminal edges out of its mask as
  the router does (`CheckedWire::terminal`). The search and the count still
  disagree at the halo's edge unless `SCPD_CROSSING_EXIT_HEADING` is on —
  twelve of the morning's 22 crossing wires were crossing by the count
  alone; measured, off, the user's decision
  ([handover-targeted-repair.md](handover-targeted-repair.md)).
- **The bend penalty of a chain edge** has never been measured below 1.0. Ours
  is the outer routing's own figure, 7125 on 17q and 27000 on 69q, where the
  prototype routes its coupler edges at a flat 4000. The clamp now allows it;
  the sweep has not been run.
- **The chain objective counts only turns.** Ten thousand an eighth turn, and
  length counts for nothing — a detour is free. A length term was tried on
  2026-10-05 (`SCPD_CHAIN_LENGTH_WEIGHT` 1, `artifacts/logs/length`): with
  the bounds still counting turns alone the exact search lost its pruning
  — 17q's chain 0 252 prefixes instead of 43, 69q's insertion 276 s
  instead of 88 with four chains out of their clock, a worse angle and an
  edge undrawn. Off. A length term needs a bound that counts length too
  (the analytic Dubins bound has the length of its own curves, which would
  be the place) before it can be afforded.
- **What is left at the end of the stage** are lane-trading pairs of plain
  wires, which the one-sided relaxation never frees together (it releases
  along the sweep only, as the prototype's does — a two-sided release would
  be ours, and has not been measured): 69q's 183/184, 17q's. The targeted
  repair is the mechanism built for these. The rest of the day's 20 is
  crossing wires at 10 cells from an edge — 21q 27, 45q 65, 57q 15 and 171,
  69q 15 and 191 — the halo's-edge case of `SCPD_CROSSING_EXIT_HEADING`.
- **`whatBlocks` costs five searches** per failed phase 1 at `-v 1`, and its
  `in the way` verdict leaves the second pair of neighbours out. The
  `dead on arrival` line is the cheaper and sharper of the two, and it does
  not know the length-point bands: a cell they close reads `within the
  clearance of nothing named`.
- **`ctest`**: see the note at the end of this document for what ran on
  2026-10-05 and what did not. The terminal stub guard, the fence pairs and
  the halo reach have no unit test of their own; their measurements are in
  *The terminal edges' coupler runs*.

## The dashboards

Generated for all eight chips on 2026-10-03 under the defaults above, each
with `plan … --stage final -v 1 -d`:

| Chip | pictures | run | size | dashboards |
|---|---|---|---|---|
| 4q | 98 | seconds | 16 MB | feedline, outer |
| 9q | 250 | ½ min | 177 MB | feedline, outer |
| 17q | 795 | 1 min | 597 MB | feedline, outer, inner |
| 21q | 780 | 2 min | 1.2 GB | feedline, outer, inner |
| 33q | 1203 | 3 min | 1.1 GB | feedline, outer, inner |
| 45q | 1983 | 8 min | 4.4 GB | feedline, outer, inner |
| 57q | 3373 | 7 min | 6.1 GB | feedline, outer, inner |
| 69q | 4663 | 13 min | 9.5 GB | feedline, outer, inner |

Some 23 GB in all. They live in `artifacts/<chip>/debug/`: `dashboard-feedline.html`,
`dashboard-outer.html` and, where the inner circuit had anything to attempt,
`dashboard-inner.html`; `run.log` is the log they were built from, and
`viewers/` holds one layered page per **failed** search, which is what the
red cells of a table link to. A green cell links to the picture itself.

Three things to know before regenerating one. The table needs `-v 1`, not
`-v` alone, or the attempts are not reported and there is nothing to lay
out. The CLI does **not** clear the directory, and the picture counter
starts at 1 on every run, so a second `-d` run over the first leaves the
pictures of the longer run lying beside the new ones; remove
`artifacts/<chip>/debug` first. And the targeted repair's local passes have
no dashboard and draw no pictures: they are silenced, and with
`SCPD_RESEARCH_VERBOSE=1` their lines land in the log, where the rounds of
every candidate start at 0 again. And the pictures are
what makes a run slow — 45q takes 3 minutes without them and 8 with — so
run the chips one at a time for the three whose chain search reaches its
budget (57q, 69q), and in parallel only the small ones.

## Tests, 2026-10-05

`cmake --build --preset release` and
`ctest --test-dir build/release -E 'EveryChip/(Final|SpacedFinal)' -j 6`,
run last at the end of the day after the family bound: **394 of 397 passed**,
the same three Detail-stage tests failing as on 2026-10-04 (below). The Python
suite, `.venv/bin/python -m pytest test/python/unit`: **143 of 145**. The
two that fail are not the day's: `test_the_detail_routing_carries_a_drawn_wire_per_connection`
is the 9q Detail defect the three C++ tests fail on (wire 0 starts where
it is not fed), and `test_the_final_routing_carries_a_snapshot_of_every_phase`
expects the `refined` phase of a run that the benchmark configs stop after
`feedlines` since 2026-10-04 — the test or the config has to give. New on
2026-10-05: `FinalRouter.StartsOnTheDetailWaysAndReportsItsFails` checks
the verdicts of the end state against the `Fails:` line and the phase
snapshots for none; `test_the_final_routing_marks_the_wires_the_stage_left_failing`,
`test_the_failing_wires_are_marked_over_the_picture`,
`test_the_failing_wires_get_a_layer_of_their_own` and
`test_the_final_verdict_is_a_bit_set` cover the verdict, the picture, the
GDS layer and the schema. The terminal stub guard, the fence pairs and
the halo reach have no unit test; their measurements are in *The terminal
edges' coupler runs*.

## Tests, 2026-10-04

`cmake --build --preset release` and
`ctest --test-dir build/release -E 'EveryChip/(Final|SpacedFinal)' -j 6`:
**393 of 396 passed** in about 190 s, run last after the length-point
recovery. The two suites left out are the full Final stage on every chip —
hours of work; `test/pipeline/Benchmarks.hpp` now reads `repair_trials`
from the benchmark configs, so they would run the stage as the CLI runs it,
but they were not run.

The three that fail are the Detail stage's, which nothing here touches:
`EveryChip/BenchmarkDetail.DrawsCopperTheRulesAllow/9q`, `…/57q` and
`EveryChip/SpacedDetail.KeepTheWireSpacing/9q` (`wire 0 starts at (61,
1244) and is fed at (730.7, 38222.2)`).
`FinalRouter.StartsOnTheDetailWaysAndReportsItsFails`, which failed on
2026-10-03, passes again since the tests run the benchmark's
`repair_trials`: the stage and the design-rule check now agree about the
nine-qubit chip.

New on 2026-10-04: `ChainSearch.ARefusedCheapestRunYieldsTheNextCheapest`,
`AcceptingEveryRunChangesNothing`, `RefusingEveryRunExhaustsTheChain`,
`TheTrialBoundStopsTheRefusals`, `TheClockIsReadAfterARefusal`;
`FinalRouter.OnlyUnsettledRedrawsOneWire`;
`FeedlineOrthogonality.ATerminalEdgeIsNotInTheRule`;
`DubinsRouter.TheExitHeadingCheckMakesTheSearchAgreeWithTheCount`. The
bridge check, the length-point clearance and the outer routing's pricing
switches have no unit test of their own; their measurements are above.
