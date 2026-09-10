# Phase 4, step 3 — the Final stage, phases 1 and 2

What was built, what was measured, what is deliberately different from the
prototype, and what is still open. Step 2 is
[summary-detail-routing.md](summary-detail-routing.md); the briefing this
answers is [handover-final-routing.md](handover-final-routing.md), and what is
left of the stage is [handover-final-couplers.md](handover-final-couplers.md).

- Checkout: `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`
- Branch: `phase-4-routing-stages`. **Nothing is committed.**
- Prototype: `/Users/michaelfeldmeier/Documents/GitHub/FridgeCAD` (`0c5d6d9`)

## What this step delivers

The **Final** stage and the first two of its five phases: every wire of the
plan drawn again as curvature-constrained copper over the Dubins primitives of
one bend radius, on a grid whose cell is about ten layout units. The run
directory fills six of its seven artifacts:

```text
01-capacity.fb … 05-detail.fb  06-final.fb
```

`FinalRouting` carries the router grid, one wire per connection of the
assignment in its order, one per connection of the global stage, and a snapshot
of every phase. A wire that was not drawn carries no cells, so a reader counts
the failures rather than being handed a list that can disagree with the paths
beside it.

## The model

The prototype's five phases, in its order:

```text
1 inner routing      the inner circuit, inside its unit cells        built
2 outer routing      the ring, against the inner circuit and itself  built
3 coupler insertion  the CPW couplers and their ports                open
4 feedline routing   the launcher-to-launcher chains                 open
5 feedline refinement                                                open
```

**One driver runs every routing phase.** The prototype writes the loop out four
times — once for the inner circuit, once for the ring, once for the wires under
the feedline constraints and once for the refinement — and the four differ only
in their parameters. Its shape, from `FinalGrid.cpp:2778`:

- sweep the wire list, alternating direction, skipping the wires already
  routed;
- offer each wire a way inside a band around the way it has;
- when it finds none, let go of the wires beside it along the sweep, one at a
  time, and price the way outside the lane between its two ring neighbours;
- roll everything back when nothing came of it.

What the relaxation steers by is the prototype's own corridor polygon
(`compute_corridor_polygon_proximity`, `FinalGrid.cpp:15748`): the closed shape
that runs from the wire's source along the way of its ring predecessor, back to
its target, and along the way of its successor. Everything outside it is
priced, nothing is forbidden. On top of it the wires further along the sweep
are priced at growing distances, so a wire that was let go of is pushed away
rather than walked over.

## The four corrections

Each is the answer to a defect the prototype's own pictures show.

**1. The clearance holds against every wire.** The prototype keeps a wire clear
of the two beside it in the ring, per search, and knows nothing of the other two
hundred. Cutting the disc for every wire per search would cost the whole chip's
copper per attempt, so the grid carries how many wires guard each cell instead:
a wire charges its own room when it is put down and gives it up when it is taken
off. The rule against two hundred wires then costs what the rule against two
cost, and letting a wire go is exactly "its room stops standing in the way, its
copper stays".

**2. A wire is judged against the full field, not against the one its search was
given.** Everything let go of for a search stands in the way again before the
wire is put down, and a wire counts as settled only when the way it has holds
the rule against every other wire. A way that exists only because a neighbour
was lifted leaves the wire open and the next round tries it again. This is the
single most valuable change: without it a wire keeps room it took from a
neighbour, the neighbour cannot get it back, and no later round undoes it. On
the 21-qubit chip it took ten violations to one.

Two figures make it work. The verdict is taken in **layout units**, against the
rule of 185, not against the nineteen whole cells the search keeps — otherwise
the rounds chase encounters nothing will ever report. And a wire whose search
fails but whose way was legal all along is **settled**, not open: returning
false there kept eleven wires of the 17-qubit chip in the sweep for every round,
each costing up to twenty searches and moving nothing.

A new way is taken when it is **not worse** than the one the wire had, counted
the same way. Not strictly better: the corridor stage measured that a strict
test blocks the lateral moves the next round needs.

**3. The relaxation runs both ways along the sweep.** The prototype only ever
goes one way, so a wire blocked by the wire behind it has no move at all — the
sweep reverses every round, but so does which side is behind.

**4. Everything outside the ring of sources is blocked.** Every wire starts on
the ring of launcher slots, a rectangle set in from the chip outline, and the
strip beyond it is free space a wire can slip through to come back in somewhere
else — around the sources of the wires beside it rather than past them. Where
that edge is, is derived from where the launchers are; the prototype gives it
as 40 cells, which on the 17-qubit chip is 45.

**5. The straight run out of a source is a fixed place.** The router adds it to
the way after it has searched, so no search ever sees it, and a wire drawn
across another wire's run is a violation the search could not have refused.
They are charged before the first wire is drawn.

Beside those, two mechanisms the prototype does not have: a **targeted rip**
that routes the wire once with the room of the others ignored and lets go of
every wire whose room lies over that way — the wire in the way of this one need
not be a ring neighbour — and a **rescue pass** after the rounds, in which a
wire that still has no way at all takes the one it finds without the rule
rather than none. A connection that is not drawn cannot be repaired later; a
connection drawn too close to another is a finding the check reports.

## What the figures say

At the shipped defaults, all eight chips:

Measured over the eight benchmarks. The first six columns come from one run of
the whole set; the two chips marked ● were measured again after the last three
corrections and are the current figures. **Everything else in the table is one
build older**, so the counts for the six unmarked chips are an upper bound on
what the tree produces now, not a measurement of it — re-run the loop at the
end of [user_final.md](user_final.md) to replace them.

| chip | connections | drawn | inner | cells | findings | routed |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 4q    |  12 |  12 |   0/0 |   1198 |  0 |   1 s |
| 9q    |  30 |  30 |   6/6 |  11574 |  0 |   3 s |
| 17q ● |  58 |  58 | 14/14 |  18170 |  2 |  10 s |
| 21q ● |  70 |  70 |   8/8 |  53026 |  3 |  72 s |
| 33q   | 110 | 110 |   8/8 | 104433 |  8 |  81 s |
| 45q   | 150 | 150 |   7/7 | 164656 | 33 | 296 s |
| 57q   | 190 | 190 | 11/11 | 232533 | 10 | 514 s |
| 69q   | 230 | 228 | 11/11 | 406490 | 44 | 977 s |

**Every connection is drawn on seven of the eight chips**, and 228 of 230 on the
largest. `findings` is what `mqt-scpd drc` reports: pairs of wires closer than
`min_wire_spacing`, after the two exemptions below. On the 4- and 9-qubit chips
there are none.

The artifact is 0.1 MB on the smallest chip and 22 MB on the largest, because
it carries a full cell path per wire per phase; the SVG of one phase is 0.04 to
1.3 MB, well inside the 10 MB budget, and the GDS 0.03 to 1.2 MB.

### Where the remaining findings come from

Two shapes, and they want different answers.

- **A near miss.** Two wires 176 to 184 layout units apart against a rule of
  185. The search cannot produce one — it keeps nineteen whole cells, which is
  189.5 — so they come from the straight runs the router adds to a way *after*
  it has searched, and from the stubs of the wires the rescue draws.
- **A short.** Two wires one cell apart. These came from a wire taking a way
  through a neighbour's room in the first round, when it had no way of its own
  to compare against; refusing that took the 21-qubit chip's three-wire bundle
  out. What is left of them is on the chips that were not measured again.

**The stage's own counter and the checker disagree**, and the checker is right.
`mqt-scpd plan -v` ends the 17-qubit run with "0 too close" while
`mqt-scpd drc` reports two pairs at 176 and 178 layout units. Both make the same
two exemptions and both measure in layout units, so one of the two reads
something the other does not; it is the first thing to find out.


## The rule, and the two encounters it forgives

`min_wire_spacing` is 185 layout units. The router grid's cell is 9.91 to 10.00
units on every benchmark, so the rule spans 19 cells and what the router keeps
is 19 whole cells — about 189.5 units, a little more than the rule asks for. The
**check** asks for the rule itself, in layout units, because that is the
physical contract.

Two encounters are not violations, and each is one a working design makes on
purpose. Both are the prototype's own, from `verify_min_clearance`:

- **A junction.** Two terminals within one wire spacing of each other are one
  meeting, and everything within one and a half spacings of it belongs to that
  meeting.
- **Two wires that end on one component.** The ports of a component sit closer
  together than the wire spacing and each of the two has to be reached, so the
  approaches converge and no arrangement holds them apart.

The router makes the same two exemptions, in their narrow form: a wire may
enter a cell another wire only guards when the cell belongs to a junction of
its own **and** every wire guarding it ends on a component this wire also ends
on. The exemption is of the clearance, never of the copper — two wires never
share a cell.

## Every figure is normalised to the grid

Nothing here is a cell count standing for a distance.

| Prototype | Here |
| --- | --- |
| `outer_min_dist_wires = 19` | `cells_for(min_wire_spacing, router)` |
| `outer_straight_start = 9` | `cells_for(min_straight_length, router)`, which is 10–11 |
| `outer_expansion = 200` cells | `corridor_spacings`, in wire spacings |
| `x_y_obstacle_boundary = 40` cells | the rectangle the launcher slots stand on |
| `outer_static_proximity_radius = 10` cells | `obstacle_penalty_reach`, in layout units |
| the keepout, plus `FG_OBSTACLE_INFLATE` | `min_obstacle_spacing`, and no environment override |
| `outer_bend_penalty = 8000…30000`, hand-tuned per chip | `bend_penalty_norm` × the grid's extent, one figure for every chip |

## The design-rule check

`MQT::ScpdDrc` gains the rules that read a view in router cells — wire
clearance, wire loop and obstacle clearance — each written once. The stage's own
tests call them, so what the stage is judged by and what `mqt-scpd drc` reports
cannot drift apart, and `drc.json` says where every finding is and how far the
two wires are.

The wire-loop rule is `routing::pathSelfIntersects`, the router's own test. A
revisit within four steps is the micro backtrack the router emits at every
heading change — hundreds per layout, none of them a loop, because the bend
radius is five cells.

The obstacle rule is checked and not searched: the keepout is baked into the
mask before any search runs, as an exact distance in layout units from the cell
to the polygon edge rather than a dilation of the finished raster.

## The pictures

Five phases, five pictures, and one GDS that shows all five:

```bash
mqt-scpd plot   -c benchmarks/17q/config.toml --stage final --phase outer \
                --run-dir artifacts/17q -o artifacts/17q/17q-final-outer.svg
mqt-scpd render -c benchmarks/17q/config.toml --stage final \
                --run-dir artifacts/17q -o artifacts/17q/17q-final.gds
mqt-scpd drc    artifacts/17q
```

A phase that is not built yet draws nothing. The GDS puts each phase on a layer
of its own, layers 24 to 28, beside the clearance band on layer 23, where a
boolean finds an overlap exactly.

## The files

| file | what |
| --- | --- |
| `schemas/artifacts.fbs` | `FinalWire`, `FinalPhase`, and `FinalRouting` with `grid`, `wires`, `inner`, `feedlines`, `phases` |
| `schemas/config.fbs` | `FinalParams`, `GridParams.router_cell_size` |
| `include/mqt-scpd/pipeline/Stages.hpp` | `IFinalRouter` |
| `include/mqt-scpd/pipeline/FinalRouter.hpp`, `src/pipeline/FinalRouter.cpp` | the stage, the scene, the field, the driver |
| `include/mqt-scpd/drc/Rules.hpp`, `src/drc/Rules.cpp` | the rules in the cell view, and `drc.json` |
| `src/pipeline/Registry.cpp`, `src/io/Artifacts.cpp`, `bindings/bindings.cpp` | wiring |
| `python/mqt/scpd/{run,planning,plot,cli,drc}.py`, `export/klayout.py` | the stage, the phases, the layers, the report |
| `test/pipeline/test_final_router.cpp`, `test/drc/test_rules.cpp` | the invariants over all eight chips |

## What is still open

1. **Three of the five phases.** [handover-final-couplers.md](handover-final-couplers.md)
   says what each has to do and what the first two learned.
2. **The meander is not built.** `meander_length` is read and converted and
   nothing uses it, so a resonator's way is as long as its route makes it.
3. **Two chips do not hold the rule everywhere.** Every wire is drawn on all
   eight; what is left is in `drc.json` of each run.
4. **Nothing is compared against the prototype's own output**, only against its
   formulation as read and the counts in its checked-in run logs.
