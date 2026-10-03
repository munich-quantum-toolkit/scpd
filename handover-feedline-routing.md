# The feedline routing

Written for whoever takes the feedline stage further. The stage now draws
**every feedline edge of every chip** — 305 of 305 over the eight benchmarks
— and holds three guarantees it did not hold before: a feedline keeps the
design rule from every coupler lead, two edges meeting at a coupler do not
cross, and no feedline crosses a resonator. Each is checked at the end of the
insertion and each is green on all eight chips.

Read [handover-cpw-coupler-insertion.md](handover-cpw-coupler-insertion.md)
for the coupler options, the geometry and the chain search this stands on,
and [handover-chain-astar.md](handover-chain-astar.md) for the prefix search
that settles a chain. This document is about what happens to the wires
afterwards, and about the obstacles the insertion builds for its own edges.

- Checkout: `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`
- Branch: `phase-2-grid-router`. HEAD is **`435ca78` Coupler insertion done
  now feedline routing**. Everything below is **uncommitted** on top of it —
  the user commits per phase.
- Benchmarks: `repair_trials = 0` and `stop_after = "feedlines"` in all eight
  `benchmarks/*/config.toml`, so a run ends where this document ends.
- Run one chip with
  `.venv/bin/mqt-scpd plan -c benchmarks/45q/config.toml -o artifacts/45q --stage final -v 1`.
  `--stage final` resumes on the artifacts already in the run directory; it
  does **not** recompute the earlier stages, and it fails if they are missing.

## Where it stands

Measured over the eight benchmarks, no environment variable set:

| Chip | edges drawn | open | short | long | fails |
|---|---|---|---|---|---|
| 4q | 5 / 5 | 0 | 4 | 0 | 4 |
| 9q | 10 / 10 | 0 | 1 | 1 | 2 |
| 17q | 20 / 20 | 0 | 7 | 0 | 7 |
| 21q | 26 / 26 | 0 | 0 | 3 | 3 |
| 33q | 40 / 40 | 0 | 9 | 2 | 11 |
| 45q | 52 / 52 | 12 | 8 | 6 | 24 |
| 57q | 66 / 66 | 19 | 18 | 13 | 46 |
| 69q | 81 / 81 | 20 | 25 | 11 | 49 |
| **total** | **305 / 305** | **51** | **72** | **36** | **146** |

`fails` counts **wires**, the other columns count verdicts, and thirteen
wires are open *and* off their length — which is why the columns sum to 159
and the total says 146.

Two things to read off it. The open wires sit **only** on 45q, 57q and 69q;
the five smaller chips have none, so the rip-up there is clean and what is
left is length alone. And **108 of the 146 are length fails**, which is the
meander: `SCPD_FEEDLINE_MEANDER` is off, so the pass routes and does not try
to make a resonator its length. Three quarters of the fails are a switched-off
step, not a routing problem. Nobody has measured what turning it back on
catches.

The ground this is measured against is the state of 2026-10-02 midday: 63
open, 169 fails, one edge with no way.

## The three checks

They run at the end of `insertCouplers` and print one line each. All three
measure **what the corridor actually builds**, not a rule of their own — that
is the point of them, and it is what makes a green line mean something.
`checkClearance` in the DRC is the second opinion about the design rule, and
it has the whole chip to look at rather than this stage's fence.

```
coupler insertion: CHECK feedline room — 0 feedline/lead pairs closer than the rule; the check is GREEN
coupler insertion: CHECK coupler crossings — 0 pairs of edges meeting at a coupler cross; the check is GREEN
coupler insertion: CHECK resonator crossings — 0 feedlines cross a resonator; the check is GREEN
```

- **`checkFeedlineRoom`** — every feedline against what the corridor fences,
  inflated by the rule in layout units (unrounded, as `conflictsIn` takes
  it), less the terminal slots. It follows `fenceTheLeadOnly`: with the
  switch on it measures leads, with it off whole resonators. A line that says
  `feedline/lead` and one that says `feedline/resonator` are answering
  different questions, and the line says which.
- **`checkCouplerCrossings`** — every pair of edges that meet at a coupler.
- **`checkResonatorCrossings`** — every feedline against every resonator past
  its lead.

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

## The switches

Everything is read from the environment, so a sweep costs a rebuild of
nothing. Defaults, with the ones this session changed marked:

| switch | default | |
|---|---|---|
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
| `SCPD_ORTHO_CROSSING` | **off** | was on |
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
| `SCPD_HALO_REACH` | 3, `SCPD_HALO_DECAY` 0 (decay off) | |

`RESONATOR_COPPER` (2) and `COUPLER_LEAD_STRAIGHT` (14) are constants, not
switches.

## The traps

Five real defects were found this session. Each was invisible in the numbers
until something else was loosened, which is the reason to write them down.

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

## Where the pieces are

- `src/pipeline/FinalRouter.cpp` — all of it. `corridorOfEdge` is the
  obstacle construction, `ensureForeignRoom` the grid, `checkFeedlineRoom`,
  `checkCouplerCrossings` and `checkResonatorCrossings` the three checks,
  `waysCross` the crossing test, `applyOption` the lead trim.
- `python/mqt/scpd/debugview.py` — wraps a debug SVG in an HTML page with
  per-class layer toggles. **Untracked.**
- `python/mqt/scpd/dashboard.py` — parses a `-v 1 -d` log into a table of
  which wire failed in which round and relaxation, with the failures linking
  to the viewers. **Untracked.** Written with `-d`, into
  `<run>/debug/dashboard-{feedline,outer,inner}.html`.
- The prototype is
  `/Users/michaelfeldmeier/Documents/GitHub/FridgeCAD/include/fiction/layout/FinalGrid.cpp`.
  `run_final_routing_feedline_parallel` is the pass this one mirrors;
  `free_terminal_stub` at `:20160` and the coupler edge parameters at `:4963`
  are the two places this document quotes.

## What is not done

- **The meander is off**, and 108 of the 146 fails are lengths. Nobody has
  measured what `SCPD_FEEDLINE_MEANDER=1` catches.
- **The orthogonal crossing rule is off** (`SCPD_ORTHO_CROSSING`), set aside
  so the rip-up could be read without it. Measured over the eight chips it
  accounted for 35 of 207 fails when it was last on, and most of those were
  wires grazing the ten-cell halo of an edge rather than crossing one. It is
  the right rule and should come back.
- **The bend penalty of a chain edge** has never been measured below 1.0. Ours
  is the outer routing's own figure, 7125 on 17q and 27000 on 69q, where the
  prototype routes its coupler edges at a flat 4000. The clamp now allows it;
  the sweep has not been run.
- **The chain objective counts only turns.** Ten thousand an eighth turn, and
  length counts for nothing — a detour is free. Lowering that number changes
  nothing, because it is the only term; loosening it would mean adding a
  length term.
- **`ctest` has not run to completion** since 2026-10-02 midday. It was killed
  mid-run to free the machine for a sweep and never restarted. Run it before
  committing.
