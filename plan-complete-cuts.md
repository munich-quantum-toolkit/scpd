# Plan: complete cuts for the capacity check

Written for the user and for whoever rebuilds the bottleneck detection of the
capacity check. It reviews how the check finds its bottlenecks today, measures
where that goes wrong on the stored cases of round 31, and proposes a detection
that needs no selection rules: every Delaunay edge between two walls is a cut.

Read [handover-capacity-graph-sat-check.md](handover-capacity-graph-sat-check.md)
first. This plan uses its terms (gate, chamber, stretch, demand, the four
counts) and its experiment folder.

- Checkout `scpd-phase-4`, branch `phase-4-routing-stages`, HEAD `ce1d5b4`
  (2026-10-13), everything uncommitted. Nothing in `src/` changed for this plan.
- The probes of this plan are in the experiment folder
  `artifacts/logs/sat-vs-feedline-2026-10-13/`: `cutaudit.py` and
  `cutproto.py`. They read the stored cases only and need no rebuilt package:
  `uv run --no-project --with numpy --with scipy python -I cutproto.py round31/results 33q --per-kind`.

## Summary

1. The check picks a few lines out of the free space (the gates) by local rules
   and floods the chambers between them. 15 rules, most of them from the review
   rounds, decide which lines exist. Nothing guarantees that every narrow place
   has a line, that the lines close the chambers, or that a line has two sides.
2. Measured on the stored cases: 20 to 43 % of all gates border more than two
   chambers, 30 to 60 % of the chambers have fewer than 50 cells, and every gap
   between two static walls (artwork pockets in front of target ports, gaps
   between port runs) has no gate at all.
3. One rule, ⌊L / clearance⌋, measures every gate. The artwork and the border
   already hold the router's keepout; a feedline edge, a port run and a pad
   are inflated by half a clearance. A gate with an end on the artwork holds
   half a wire to one wire more than the rule gives. Two of the three overfull
   gates of 17q cases 17 and 18 (UNSAT0) are such gates.
4. The proposal: triangulate the free space (Delaunay over the wall boundary
   cells, the dual of the Voronoi diagram `grid::medialAxis` already builds).
   Every triangle edge between two free triangles is a straight segment from
   wall to wall, so it is a valid cut. Triangles joined over wide edges are the
   chambers. Edges never cross, each has exactly two sides, and every free cell
   lies in one triangle. The capacity of an edge depends on the kinds of its two
   walls.
5. A Python prototype of this check on the stored cases of round 31 gives the
   counts in *The prototype on round 31*. It needs none of the profile, mouth,
   slot, wedge, crossing or filter rules. On 17q it removes both UNSAT0 cases.
   On 33q it turns one UNSATbad into SATbad (case 14, the class 2 wires 18 and
   19). The gaps between static walls change no verdict on the stored cases.

## How the check finds its bottlenecks today

`capacityPass` (`src/pipeline/FinalRouter.cpp`) with `SCPD_CAPACITY_FAST=1`,
the default:

| Step | Code | What it decides |
| --- | --- | --- |
| walls | `wallsOf` | one label per cell: border, artwork, pad, port run flank, feedline edge (line plus a disc of half a clearance) |
| profile cuts | `grid::profileCuts` | walks every dynamic wall; a gap is a cut where it has a persistent local minimum (`minimumRise`), cut at both ends of the narrowing (`narrowingAtItsMouths`); static walls only through the nearest cell of two fields; cuts at slot mouths; wedges dropped unless a neck; one cut per narrowing (`sameNarrowing`); crossing cuts dropped (`noCrossingCuts`) |
| filter | `couplerCuts` | drops every cut with no end on a feedline edge, a pad, a lead or a terminal run |
| cut at walls | `cutGatesAtWalls` | splits a gate whose raster line crosses a wall |
| mouths | `mouthGates`, `mouthLinesOf` | lines between the run tips of two chains, cut at walls, with lead exceptions |
| slots | `slotGates` | a gate from tip to tip of every terminal run, capacity 1 for its edge |
| uncrossed | `dropCrossingGates` | drops the lower-ranked of two gates that meet in the middle |
| capacity | `wiresThroughGap` | ⌊L / clearance⌋ for every gate; own-run passage, slot and mouth exceptions in `capacityProblemOf` |
| chambers | `grid::chambersOf` | a flood that no step across a gate line passes; `beside` lists every chamber that touches a line |

Each rule closed one case the user found in a review round. The rules interact:
a mouth line needs the cut at walls, the cut at walls makes pieces that can
cross other gates, the uncrossed rule then drops one of them, and the dropped
one can leave a way round the other.

## What is wrong, measured

All numbers come from the 100 stored cases of round 31 (`cutaudit.py`, and a
count over the `caseNN-graph.json` files). Per chip, averages per case:

| Chip | Gates | Gates beside > 2 chambers | Aside | Chambers | Chambers < 50 cells |
| --- | ---: | ---: | ---: | ---: | ---: |
| 4q | 102 | 39 | 0.5 | 109 | 15 |
| 9q | 135 | 58 | 1.5 | 183 | 57 |
| 17q | 314 | 116 | 4.4 | 428 | 129 |
| 21q | 153 | 44 | 12.3 | 259 | 124 |
| 33q | 211 | 46 | 3.4 | 465 | 281 |

### 1. A gate beside more than two chambers

`grid::chambersOf` puts a chamber beside a gate when it touches the gate's line,
also at its ends. A flow edge joins all chambers beside it. Where gates fan out
from one tip, or one gate ends near another, a chamber touches a gate it does
not lie across. A demand can then pass from one chamber to another on the same
side of the gate, through the gate's node, and pay the gate instead of the gate
it actually crosses. This is the trap the handover names ("a gate beside more
than two chambers lets a wire hop"). It is not a rare case: a third of the
gates.

### 2. Lines that leave a way round them

An aside gate has one chamber on both sides: the flood found a way round it.
Every such gate is a narrowing whose partner line is missing, for example the
second gate beside an island. 21q has 12 per case.

### 3. Gaps between static walls have no gate

`couplerCuts` keeps only cuts with an end on a wall the insertion moves. The
reason was that the insertion cannot change the other gaps. The gaps still
limit the routing. `cutaudit.py` compares, for every demand, the max flow
between its two terminals in the check's graph and in a graph of all Delaunay
cuts under the same capacity rule. Where the check allows 8 or more wires and
the complete cuts 3 or fewer, the narrowest complete cut lies between two
static walls in 85 to 92 % of the demands (17q: 847 of 1041, 33q: 1196 of
1306). Example, 33q case 01: the pocket in front of the target port of wire 6
opens through a 37.5-cell gap between two parts of the artwork. The check has
one chamber on both sides of it.

### 4. One capacity rule for every wall kind

`wiresThroughGap` gives ⌊L / clearance⌋, with L the distance between the two
wall cells. That is right between two inflated walls: each keeps the wire's
middle line half a clearance off. The artwork and the border are
`scene.blocked`, which already holds the router's keepout: a wire's middle
line may run in the next free cell. So a gap of L cells holds

```text
k = ⌊(L − m_a − m_b) / clearance⌋ + 1,   m = 1 cell (artwork, border)
                                          m = clearance / 2 (feedline edge, port run, pad)
```

which is ⌊L / clearance⌋ between two inflated walls and half a wire to one
wire more at the artwork. Evidence:

- 17q cases 17 and 18 (UNSAT0, routed with 0 bad): of the three overfull
  gates, `f18 · artwork near Qb15.port0` (35.5 cells) and
  `f18 · artwork near Qb15.port1` (37.0 cells) hold 1 by the current rule and 2
  by the rule above. The third, `0 at its target · f18`, lies on f18, which the
  feedline routing lays again.
- 33q: wires 65 and 66 reach their bridge-pair ports through one 34-cell gap
  between two parts of the artwork. The routing lays both; the current rule
  gives 1, the rule above 2.
- Of the gates with an end on the artwork or the border, the rule above widens
  654 of 1502 on 17q (370 from 1 to 2 wires), 374 of 713 on 4q.
- Away from a wire's own ends, no routed wire crosses a cut between two
  inflated walls more often than ⌊L / clearance⌋ allows, except at the runs of
  terminal edges (17q `f5 at its coupler`, 21q `f10 at its coupler`), which
  the feedline routing lays again.

The pad's margin is an assumption (inflated). The tests below have to fix it.

### 5. Terminals inside the artwork

The terminal of a wire at a bridge-pair port (`Coupler*.port1` to `port4`) lies
inside the artwork in the raster: the router's approach of twice the straight
length runs in a narrow notch of the keepout. The check gives such a terminal
the nearest chamber (`chambersOfPort`). Any cut-based detection has to do the
same and let the wire pass its own notch.

### Not part of the detection

Classes 1 (too few crossing places) and 2 (the terminal edges' paths are no
obstacle) of the handover do not depend on how the gates are found. A better
detection does not fix them; they stay open.

## The proposal: complete cuts

### Idea

Every straight segment between two wall points that runs through free space is
a cut: each wire across it needs a clearance of it (Leiserson and Maley, the cut
condition of homotopic routing). The check needs the cuts that limit something,
and it needs them to divide the free space without gaps. A triangulation of the
free space gives both:

- The Delaunay triangulation of the wall boundary cells holds the nearest
  neighbour and the Gabriel graph of the boundary, so every narrowing has an
  edge close to its narrowest segment, whatever its shape.
- Its edges do not cross. Each edge has two triangles. Every free cell lies in
  one triangle. So there is no wedge, no aside line, no hop and no crossing
  rule.
- A slot is a strip of triangles between its two flanks; its mouth is the edge
  from tip to tip. A mouth between two chains is the set of edges between the
  two coupler runs. An island in a corridor gives two edges, one on each side.
  None of them needs its own rule.

### Steps

1. **Sites.** The wall cells with a free 4-neighbour, labelled with their wall
   (`Walls::of`). Optionally every second cell along a straight outline.
2. **Triangulation.** `boost::polygon::construct_voronoi` over the sites, as
   `grid::medialAxis` does; every Voronoi vertex is a triangle (a polygon where
   more than three sites lie on one circle), every primary finite edge between
   the cells of sites a and b is the triangle edge ab. No new dependency.
3. **Free triangles.** A triangle is free when its centroid cell is free. A
   triangle edge between two free triangles is a cut when its raster line runs
   free between its two ends; otherwise nothing crosses it.
4. **Capacity.** k = ⌊(L − m_a − m_b) / clearance⌋ + 1 by the kinds of its two
   end walls. For the owner of a port run whose flank is an end, the flank's
   margin is 0 (the own-run passage). A terminal inside the artwork lets its
   wire pass every cut within its approach.
5. **Chambers.** Union-find over the cuts that take at least W wires (W above
   the most wires any cut can be asked for; 12 in the prototype): the merged
   triangles are the chambers. A cut between two triangles of one chamber is
   dropped: a way round it is wide. Optional and exact: a chain of triangles
   with two neighbours each is one edge of the smallest capacity on it, and a
   dead end with no terminal is dropped.
6. **Feedline crossings.** The stretches stay as they are (`crossingStretches`).
   Each crossing point joins the two chambers at the ends of its crossing line,
   capacity 1, open to the stretch's users.
7. **Demands and router.** Unchanged: `capacityProblemOf` builds the same
   demands, `checkInTurn` routes them. Every flow edge has exactly two
   chambers.

`profileCuts`, `couplerCuts`, `cutGatesAtWalls`, `mouthGates`,
`mouthLinesOf`, `slotGates`, `dropCrossingGates`, `uncrossedBottlenecks` and the
flood of `chambersOf` are not needed by this check. The report path
(`SCPD_BOTTLENECKS`) can keep them until the user removes them.

### What it does not make exact

- **The cut condition on triangle edges is necessary, not sufficient.** A
  segment between two wall points that is not a triangle edge can be narrower
  than the edges it spans. This happens where a third wall point lies close to
  the segment, and the error is at most a factor √2 on that segment. If the
  stored cases show it, add the segments from each site to the sites two
  triangles away.
- **The router is no free-form router.** Its headings are multiples of 45°: a
  wire crosses a cut at up to 22.5° off the cut's normal and needs up to 8 %
  more of it. Its bends have a radius and its terminals a straight length. A
  corrected capacity ⌊(L − m_a − m_b) · cos φ / clearance⌋ + 1, with φ the angle
  between the cut's normal and the nearest heading, covers the first; the rest
  is no capacity question.
- **The demand router is a heuristic.** An integral multi-commodity flow is
  NP-hard. `checkInTurn` with rip-up stays; negotiated congestion (as in the
  prototype, and as PathFinder measured on 2026-10-08) does not depend on the
  wire order and costs little on the merged graph.

## The prototype on round 31

`cutproto.py` builds the graph above from each stored `caseNN-graph.json` (the
raster, the strokes for the owners of the port runs, the stretches with their
crossing points, the wires as demands) and routes the demands by negotiated
congestion, 25 rounds at most. Each crossing point is a portal of capacity 1
between the chambers at the two ends of its crossing line. A plain wire crosses
each prescribed edge exactly once (a search over one layer per set of edges
crossed so far), and no other. SAT when every demand has a way and no cut is
over. The prototype takes the stretches and the covered ports from the stored
graph, so class 1 stays as it is. It does not hold the resonators to their
length and has no detour factor.

Three findings from building it:

- A stretch joined at one cell per side puts every crossing in whatever pocket
  that cell lies in. The crossings have to be joined point by point.
- Without the prescription a plain wire passes round the open end of a chain
  instead of crossing its edge, and three 21q cases turn SAT.
- The terminal of a bridge-pair port lies inside the artwork; the wire has to
  pass the cuts of its own approach notch (40 cells round the port in the
  prototype).

Counts SAT0 / SATbad / UNSAT0 / UNSATbad on the stored cases:

| Chip | Check of round 31 | Prototype, all cuts | Prototype, static gaps left out |
| --- | --- | --- | --- |
| 4q | 13 / 0 / 0 / 10 | 13 / 0 / 0 / 10 | 13 / 0 / 0 / 10 |
| 9q | 13 / 0 / 0 / 0 | 13 / 0 / 0 / 0 | 13 / 0 / 0 / 0 |
| 17q | 12 / 0 / 2 / 11 | **14 / 0 / 0 / 11** | 14 / 0 / 0 / 11 |
| 21q | 0 / 0 / 1 / 13 | 0 / 0 / 1 / 13 | 0 / 0 / 1 / 13 |
| 33q | 5 / 8 / 5 / 7 | 5 / 9 / 5 / 6 | 5 / 9 / 5 / 6 |

The prototype removes both UNSAT0 cases of 17q (capacity per wall kind) and
agrees with the check elsewhere. 33q case 14 turns from UNSATbad to SATbad:
its fails are 18 and 19, the class 2 pair. The 21q UNSAT0 (`case13`) and the
33q UNSAT0 cases stay: they depend on the crossing places (class 1), which the
prototype takes from the stored graph. The probes ran on 2026-10-15 as round
32 (`run_round32.sh`, `round32/index.html`).

## Plan

1. **Decide** (user): the margins per wall kind, the pad's above all; whether
   the gaps between static walls count (the prototype's variant
   `--static-off` drops them as `couplerCuts` does).
2. **Grid tests first** (`test/grid/test_complete_cuts.cpp`, new suite
   `CompleteCuts`), each red before the code:
   - a straight corridor of width L between two walls of a kind holds the
     capacity of the rule, for each pair of kinds;
   - an island in a corridor: two cuts, one on each side, and no way round
     either;
   - a pocket: its mouth is one cut, and a terminal in it lies in its own
     chamber;
   - a slot between two thin flanks: the chamber of the terminal is reached
     through the edge from tip to tip alone;
   - no two cuts cross, every cut has two chambers, every free cell lies in
     one chamber;
   - a thin wall one cell thick is crossed by no cut.
3. **The graph** in `src/grid/CompleteCuts.cpp` (sites, triangulation, free
   triangles, cuts with their two end walls, chambers by union-find), returning
   the same `grid::Chambers` and a list of cuts with two chambers each.
4. **The check**: `capacityPass` under a new switch `SCPD_CAPACITY_CUTS`
   (`delaunay`; the profile cuts stay the default until the four counts say
   otherwise), and the capacity per wall kind in `capacityProblemOf`.
5. **Pipeline tests** (`CouplerSession`): the properties of the handover that
   still hold (every chain checked alone first, the terminal edges as wires,
   no terminal edge lost at a mouth), and new ones on the JSON: every gate has
   two chambers, no gate crosses another, every free cell of the raster lies
   in a chamber.
6. **Measure**: `recheck.py round31 <chip>` with the switch, against the table
   above; then round 32 on 4q to 33q, and 45q to 69q, which have not run since
   round 16.
7. **Cost**: `construct_voronoi` took 97 ms on 17q on the cloud machine
   (handover-capacity-check.md §9); the profile cuts take 12 to 24 ms. The
   prototype's graph has about 33 000 cuts on 17q and 61 000 on 33q before
   merging. If the check of a chain-search step becomes too slow, rebuild only
   the triangles the step's new walls touch.

## Open questions for the user

1. The margin of a pad: inflated like a feedline edge, or keepout already in
   it like the artwork?
2. The gaps between static walls: real limits of the check (the prototype's
   default), or left out as now?
3. The class 2 proposal of the handover (a terminal edge's routed path as a
   wall) is independent of this plan and still open.
