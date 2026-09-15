# Phase 4, step 3 — the Final stage, phases 1 and 2

What was built, what was measured, what is deliberately different from the
prototype, and what is still open. Step 2 is
[summary-detail-routing.md](summary-detail-routing.md); the briefing this
answers is [handover-final-routing.md](handover-final-routing.md), and what is
left of the stage is [handover-final-couplers.md](handover-final-couplers.md).

- Checkout: `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`
- Branch: `phase-4-routing-stages`. **Nothing is committed.**
- Prototype: `/Users/michaelfeldmeier/Documents/GitHub/FridgeCAD` (`0c5d6d9`)
- Newest first: [the meander](#the-meander), then
  [the seeded sweep, and what a fail is](#the-seeded-sweep-and-what-a-fail-is)

## What this step delivers

The **Final** stage and the first two of its five phases, with the meander
that makes every resonator as long as the coupler insertion needs: every wire
of the plan drawn again as curvature-constrained copper over the Dubins
primitives of one bend radius, on a grid whose cell is about ten layout
units. The run directory fills six of its seven artifacts:

```text
01-capacity.fb … 05-detail.fb  06-final.fb
```

`FinalRouting` carries the router grid, one wire per connection of the
assignment in its order, one per connection of the global stage, and a snapshot
of every phase. A wire that was not drawn carries no cells, so a reader counts
the failures rather than being handed a list that can disagree with the paths
beside it.

## The meander

The last piece of the outer routing, built after it reached no fail on every
chip: a resonator's way is made `meander_length` long before the coupler is
spliced into it. The prototype's `meander_insertion`
(`dubin_router_opt.hpp:2182`) and `meander_insertion_proximity` (`:2325`),
called from `process_wire` of `run_final_routing_parralel`
(`FinalGrid.cpp:7017` and `:7201`), are `routing::insertMeander`
(`include/mqt-scpd/routing/MeanderInsertion.hpp`) here, called from
`Driver::lengthen`.

**What the prototype does, read from its code.** Sample the way and measure
it. If it is long enough, done. Else walk the cells of its straight runs in
steps of a hundredth of their count, from the fourth cell on, and for every
pair of them: turn each end onto the axis across the pair by one or two moves
of the primitives — every single move that does, and the cheapest pair of
moves per exit heading — and between the turned ends build one rectangular
loop: out along the heading, a quarter turn, across to the other end, a
quarter turn, back. Its depth is what the missing length makes it, rounded up
so that the length is not undershot. That loop is the one shape its
`compute_meander_between_points` ever accepts (`nMeanders == 1`); the even
meanders it computes for two ends on the same heading are never taken. The
loop and its turns replace the cells between the pair, and every cell of the
piece is tested against the corridor grid of the search that found the way —
the band less the fence — and against nothing else. Phase 1 takes the first
fit. The relaxation scores every fit by the static and the wire price summed
over the path plus ten per direction change and takes the cheapest. A
deficit under fifty cells is made fifty cells more. A resonator whose meander
cannot be placed counts as not routed, so the relaxation goes on, and the
plain way is what the neighbours see.

**What was built the same, and where it differs.** The pairs of straight
cells, the loop and its length formula, the corridor test, the first fit and
the priced fit, the margin, the four-cell start margin and the 25-cell leg
spacing (the prototype's `min_straight_length`, which is not the design rule
of that name) are the prototype's. Seven things are not:

- **The pairs are tried from the qubit end, the smallest span first**, so
  that the smallest loop goes as near the qubit as it fits and walks back
  toward the source only where nothing fits. The prototype walks from the
  source and takes the first pair that fits, which put every loop right
  after the feed point: on the 4-qubit chip up against the launcher pads,
  and on 17q within thirty cells of the ring. That end is where the coupler
  insertion cuts the way back to `target_resonator_length` — a loop there is
  cut in two, and the coupler lands on a leg of it — and where the feedline
  runs, so a loop there is in its way. The prototype's own pictures show its
  loops near the coupler because its feedline phase routes every resonator
  again from the coupler and places the loop again from there; that phase
  is not built here, so the loop is put where it will survive the cut. On
  4q the four loops now sit before the qubits; on 17q, where the fan-in
  before a qubit leaves no room for a loop 26 to 51 cells deep, they sit
  on the middle of the way.
- The two ends are turned by a direct walk over the primitives, forward from
  the first cell and backward into the second, each cell tagged with the move
  that leaves it, as the router tags its own paths. The prototype reflects a
  forward move through the second cell and then looks the tag up by exit
  heading, which names the wrong move wherever a heading has two arcs to
  the same exit.
- The loop's room is checked from its geometry: its far edge stays two cells
  short of the box, and neither leg is negative. The prototype's checks are
  written per heading, and one of them misses the loop's depth by the offset
  between the two ends.
- The rendered length of the spliced path is measured, and where the
  rendering falls short of the requirement by a fraction of a cell per bend,
  the loop is built again that much deeper. The prototype trusts the formula.
- The spliced path is tested for meeting itself, by the router's own test. A
  loop that reaches across the way's own later run is a self-crossing the
  check reports; the prototype tests nothing there.
- The priced fit scores a candidate by its piece, with the price and the
  bends of the rest summed up front, and splices only the best. The prototype
  splices every fit and, through a stale index, scores a path that is not
  the candidate's. Ten times faster on a 150-cell run, and the same answer.
- The priced variant's early exit for a "length difference too small" —
  forty cells, four hundred layout units, accepted as long enough — is not
  ported. A way is long enough when it is as long as required, in both
  variants.

The box is the router grid. The prototype insets it by seven percent of the
grid on each side and needs zero on one chip; here everything outside the
ring of sources is blocked in the mask already, and the corridor carries the
mask.

**Where it hooks in.** In `Driver::attempt`, after the phase-1 search and
after every relaxation level, a resonator's way found is lengthened before it
is taken; a way without room for its meander is set aside and counts as no
way, so the relaxation goes on. When nothing comes of it the wire keeps that
plain way, drawn and marked too short, and is tried again in the next round;
the round summary counts it as `short`, not `open`. `Driver::refine`
lengthens the wider way and keeps the old one when there is no room.
`Driver::failsOf` measures every drawn resonator's way with the sampler and
counts one shorter than required as short, whatever the sweep believed. The
length a resonator has to reach is `meander_length` less the run from the
cell its way ends on to the port itself (`Wire::anchorGap`), which the wire
covers without cells of its own: the prototype's `target_anchor_offset`.

**The artifact carries the length**, and so does the log. `FinalWire.length`
is the rendered length of the way in layout units, so that the test and any
reader can check a resonator against `meander_length` without the
primitives: the cells alone overstate an arc by two thirds, and nothing a
hundred units short of the figure could be told from them. With `-v` the
stage ends on one line per resonator — the way, the run from its last cell
to the port, the two together against `meander_length` — and one line over
all of them.

**The corners of a loop are quarter turns**, as the prototype builds them,
and a quarter turn's swept cells zigzag between the two rows the arc runs
between, so a plot of the cells shows a sawtooth at every corner. The
router's own quarter turns show the same sawtooth (eleven of them on 17q
outside the resonators); it is the raster of the primitive, not the loop,
and a picture of the exact curves is the Finalize stage's.

### Measured at six rounds, five relaxations, no refinement

Every benchmark carries the prototype's own `meander_length`: 250 cells on
4q, 400 on 21q and 600 from 33q up, the default of 300 on 9q and 17q — in
layout units about ten times that.

| chip | wires | resonators | needed a meander | searches without room | **Fails** | `drc` pairs | final stage |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 4q  |  12 |  4 |  4 |  0 | **0** | 0 | 0.04 s |
| 9q  |  36 |  9 |  0 |  0 | **0** | 0 |  0.4 s |
| 17q |  72 | 17 |  5 |  4 | **0** | 0 |  0.9 s |
| 21q |  78 | 21 |  0 |  0 | **0** | 0 |  3.4 s |
| 33q | 118 | 33 |  0 |  0 | **0** | 0 |  7.4 s |
| 45q | 157 | 45 |  0 |  0 | **0** | 0 |  9.0 s |
| 57q | 201 | 57 |  0 |  0 | **0** | 0 |   29 s |
| 69q | 241 | 69 |  0 |  0 | **0** | 0 |   40 s |

"Needed a meander" is the number of resonators whose way carries one when the
stage ends; the rest were long enough as drawn. So at the prototype's figures
only the two smallest chips exercise the insertion at all: from 21q up every
resonator is longer than its `meander_length` as routed — 6.4 to 27 thousand
layout units on chips whose figure is 4 to 6 thousand — because the ring
feeds a resonator far from its qubit. The final stage takes what it took
before the meander, to the tenth of a second.

**Measured under load**, at figures the benchmarks do not ship, so that the
insertion is seen doing work:

| chip | `meander_length` | resonators with a meander | insertions | searches without room | drawn | open | short | **Fails** | `drc` pairs | final stage |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 33q | 14000 | 33 of 33 | 49 | 28 | 118 of 118 |  0 | 0 | **0** |  0 | 16 s |
| 17q |  6000 | 16 of 17 | 52 | 78 |  71 of 72 | 16 | 0 |   17 | 10 | 10 s |

On 33q at more than twice its figure every resonator gets a loop and the
chip still routes clean, and the stage takes eight seconds more than
without. On 17q at twice its figure — a loop 130 cells deep for every
resonator on the densest of the eight grids — 16 wires end within the
rule of another and one finds no way, but **no resonator is short**: every
way drawn is as long as required. What that setting shows is a loop taking
room that only the two ring neighbours guard, as the search itself may, at a
density where the rounds do not resolve it; it is the prototype's model,
measured on the outer routing before.

### What the lengths are

The rendered length of the resonators' ways in the artifact, in layout units
and before the run to the port is added:

| chip | `meander_length` | shortest | median | longest |
| --- | ---: | ---: | ---: | ---: |
| 4q  | 2500 |  2407 |  2420 |  2429 |
| 9q  | 3000 |  3914 |  4822 |  5037 |
| 17q | 3000 |  2898 |  3366 |  5433 |
| 21q | 4000 |  6422 |  8088 |  9951 |
| 33q | 6000 |  7777 | 10758 | 12955 |
| 45q | 6000 |  8436 | 12372 | 16051 |
| 57q | 6000 |  7866 | 14034 | 19655 |
| 69q | 6000 | 12912 | 20269 | 27110 |

A shortest way under the figure, as on 4q and 17q, is the gap: the way ends
on the cell beyond the port's band, about a hundred units from the port, and
what the stage requires of the way is the figure less that gap.

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

## The search fenced as the prototype's

The user's last instruction of this step: build `attempt()` exactly as the
prototype's `run_final_routing` has it — phase 1 with both ring neighbours as
obstacles inflated by the wire clearance, phase 2 the relaxation with the
proximity penalty on the wires ripped up and the corridor polygon, and
nothing else. So the three corrections below that changed the *search* are
gone again, and what a search sees is now:

- **Phase 1.** The band around the way the wire has (`buildCorridor`, the
  prototype's `expand_path`: a walk over the cells that are not artwork, which
  knows nothing of other wires), less the ways of the two ring neighbours
  inflated by the clearance (`fence`, the prototype's `mark_obstacles` with
  `min_dist_wires`). Nothing priced.
- **Phase 2.** For each level up to `max_relaxation`, along the sweep only:
  the wire one further ahead is let go of — its way is no obstacle at all,
  because it is drawn again afterwards — and the search is fenced by the last
  wire let go of and the neighbour on the other side. Everything outside the
  lane between the two ring neighbours is priced (`priceLane`, the
  prototype's `compute_corridor_polygon_proximity`), and the wires let go of
  are priced at growing distances (`compute_proximity_grid`), so the way is
  pushed away from where they run rather than drawn over it.
- **A way found is taken.** When none is, the wires let go of go back to
  what they were. No verdict against the field, no relaxation against the
  sweep, no targeted rip, no rescue.

**One price on top of the prototype's**, asked for after the crossings above
were understood: in the relaxation, the approaches of the wires around the
search — the straight run out of each source and the run into each target,
shaped as the port band is, `min_straight_length` long and two wire
clearances wide — cost ten times the wire price (`priceApproaches`). A wire let go of is
crossable, but those two runs are the places it cannot be drawn anywhere else,
and a relaxed way through one of them took it for good.

**The target sat two cells too far out.** `digTargetBeyondBand` stepped once
before its first test, so the cell it returned lay two cells beyond the last
strip of the port's band, and the band itself ran to `floor(L / cell)`
strips, the rule's whole length. The straight run into a port was therefore
120 layout units along an axis and 127 along a diagonal against a rule of
100, every one of the 58 ring wires of 17q alike (16 axial at 12 cells, 28
diagonal at 9 steps, 14 bridging at 22). The prototype has the same walk, one
strip shorter, so its targets sit at 110 and 113. Now the band ends on the
cell before the first cell whose centre lies the rule's length from the
port's own position, and the target is that cell: 100 to 110 units along an
axis, 100 to 114 along a diagonal. It is what took the 17- and 21-qubit chips
to no fail at all; 33q did not move.

What stays from the corrections is the **count**: the field carries how many
wires guard each cell, and the fails of a pass are counted on it against
every other wire, by the check's own test. The field no longer fences any
search. The junction exemption stays in its narrow form — a fence wire that
shares a junction with this one leaves the meeting open, its copper closed —
because without it two wires whose ports sit closer than the rule cannot both
reach them.

### Measured at six rounds, five relaxations, no refinement

| chip | wires | drawn | unrouted | open | **Fails** | pairs | of them crossings | final stage |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 4q  |  12 |  12 | 0 |  0 |  **0** |  0 | 0 | 0.04 s |
| 9q  |  36 |  36 | 0 |  0 |  **0** |  0 | 0 |  0.4 s |
| 17q |  72 |  72 | 0 |  3 |  **3** |  2 | 0 |  1.8 s |
| 21q |  78 |  78 | 0 |  7 |  **7** |  5 | 2 |  6.3 s |
| 33q | 118 | 117 | 1 | 16 | **17** | 12 | 9 |  8.6 s |

Against the field-fenced search at the same setting (3 / 9 / 16 fails, with
42 of 17q and 69 of 21q unrouted): both of those wires are drawn now, 21q has
fewer fails, 33q one more, and every pass runs in half the time because nearly
everything settles in round 0 — 33q ends round 0 with 6 fails where the
field-fenced search ended it with 53.

**What the fence of two cannot see.** Eleven of the twenty pairs are at zero
distance: two wires holding the same cells. On 33q wire 47 runs over 43, 44
and 45, and 74 and 75 over 71, 72 and 73 — wires two to four places away in
the ring, which no fence of this wire ever names, and which the relaxation
lets it cross on purpose. The prototype's rerouting of the wires it crossed
is fenced by *their* neighbours, and where that leaves them no way they keep
the way they had, crossing and all. This is the defect the field-fenced
search was built against, and it is the price of the prototype's model. What
the two searches share is that neither holds the rule everywhere at this
setting; where they differ is in kind — near misses and shorts on one side,
crossings on the other.

## The four corrections

Each is the answer to a defect the prototype's own pictures show. **The first
three no longer describe the search** — see the section above — and stay here
as the record of what was built and measured; the count they left behind is
what the `Fails:` lines report.

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

## The seeded sweep, and what a fail is

Two changes the user asked for after the first write-up, and one bug they
uncovered.

**The sweep starts from the Detail stage's ways.** The sweep is a rip-up and
re-route, and a rip-up needs something to rip. Before, every wire began with
nothing — the Detail way was only the band a search was allowed in — so a
round saw empty space where the wires it had not reached yet would run, and a
wire whose search failed had no way at all. Now every pass begins by putting
each of its wires down on the Detail way, joined on the router grid by
eight-connected steps with the straight stub out of the source spliced in
front (`Driver::seed`, `seededWay`), copper and clearance charged like any
other way. A wire is lifted, offered a way of its own, and put back on the
way it had when it finds none; the baseline of the verdict is that way, seed
or its own. This is the prototype's start: its `global_paths` begin as the
detailed routing's paths, and a wire that fails keeps its entry.

A seed is not a way this stage drew — it is not curvature-constrained and it
knows nothing of this grid's keepout — so a wire still on its seed when the
rounds end is **unrouted**, and the artifact carries no cells for it, as the
prototype clears the path of every wire it could not route.

**Every stage ends on a `Fails:` line.** A fail is a wire without a way of its
own (*unrouted*) or a wire whose way comes within the rule of another
(*open*), counted with every wire down and by the design-rule check's own
test. The Corridor and Detail stages end on the same line; the Final stage
prints it per pass and once more over every wire:

```text
[corridor]     0.03s  58 of 58 connections have a way through the partitions, 0 unrouted | Fails: 0
[detail]       0.29s  72 of 72 wires drawn, 0 unrouted, 0 within the rule of another | Fails: 0
[final]        0.09s  outer routing: 58 of 58 wires start on the way the Detail stage drew
[final]        0.81s  outer routing round 0 forward : tried 58, routed 48, unrouted 2, open 14 | Fails: 16
[final]        3.52s  outer routing: 57 of 58 drawn, 1 unrouted, 2 open | Fails: 3
[final]        3.52s  final routing: 71 of 72 drawn, 1 unrouted, 2 open | Fails: 3
```

**The verdict never fired.** `Tuning::spacing` — the rule in cells, unrounded,
that `conflictsIn` judges a committed way by — was declared and never set. At
zero, a way conflicted only where another wire owned the very same cell,
which never happens, so every way found was taken, every drawn wire counted
as settled the moment it was drawn, and the "0 too close" the stage reported
against the checker's two findings was this. It is set now, as
`min_wire_spacing` over the smaller cell side, which is the unit
`checkClearance` measures in. Everything below is measured with it live.

### Measured at six rounds, five relaxations, no refinement — with the search still fenced by the field

These figures are of the seeded sweep with the search as the corrections
below had it, fenced by every wire's room; the prototype-shaped search that
replaced it is measured in the next section. The user's setting for this
measurement, carried by every benchmark's
`[stages.final]`: `rounds = 6`, `max_relaxation = 5`,
`refinement_rounds = 0`. Fails are wires; pairs are what `mqt-scpd drc`
reports for rule 1, and a short is a pair one cell apart.

| chip | wires | drawn | unrouted | open | **Fails** | pairs | of them shorts | final stage |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 4q  |  12 |  12 | 0 |  0 |  **0** |  0 | 0 |  0.04 s |
| 9q  |  36 |  36 | 0 |  0 |  **0** |  0 | 0 |  0.3 s |
| 17q |  72 |  71 | 1 |  2 |  **3** |  1 | 0 |  3.5 s |
| 21q |  78 |  77 | 1 |  8 |  **9** |  4 | 3 | 12.8 s |
| 33q | 118 | 118 | 0 | 16 | **16** | 10 | 5 | 15.9 s |

The same three chips with the sweep as it was before — no seeds, the verdict
dead — at the same setting: 17q 1 unrouted and no pair, 21q 1 unrouted and
one pair (a short), 33q none unrouted and six pairs (four shorts). So at
this setting the seeded sweep leaves *more* open wires than the unseeded one,
and the same two wires unrouted: connection 42 of 17q and 69 of 21q are left
by both, and were drawn only at thirty rounds and ten relaxations.

Where the difference comes from, read off the rounds: the canvas is full
from the first search on, so a wire in a bundle at the Detail stage's pitch
(153–182 layout units against 185) finds no free channel and relaxes; the
search then runs a cell from the copper of the neighbour it was let past,
because a bend costs 71 to 150 cells of length and a hug saves two. Taken,
the hug displaces the neighbour, which is the negotiation that resolves most
of the bundle by the last round — and leaves a short wherever it does not.

**Measured and not kept**, so nobody builds them twice:

| mechanism | 17q / 21q / 33q fails | shorts |
| --- | --- | --- |
| the verdict as shipped: a new way is taken when it conflicts in no more cells than the way it had | **3 / 9 / 16** | 0 / 3 / 5 |
| judged by depth instead — the sum over conflicting cells of how far inside the rule the nearest copper lies, so a hug is never taken over a near miss | 5 / 7 / 25 | 0 / 1 / 4 |
| depth, and the top price of 127 on the room of every wire let go of, so the search leaves it whenever it can | 5 / 8 / 29 | 0 / 2 / 7 |

Depth keeps the hugs out of the rounds, but the wire then stays on its seed
until the rescue, which takes any way it finds — the shorts move from the
rounds to the rescue and fewer bundles get negotiated. The uniform price does
not tell a hug from a graze, because `stampDisc` writes one value over the
whole disc, exactly as the prototype's `compute_proximity_grid` does.

**What it costs.** The seeded sweep makes far more searches per round than
the unseeded one, because far more wires stay unsettled: at the schema's
defaults of thirty rounds and ten relaxations, the 45-qubit test case took
109 minutes where the stage took 296 s before. The tests therefore read
`rounds`, `max_relaxation` and `refinement_rounds` from each benchmark's
`config.toml` (`test/pipeline/Benchmarks.hpp`), as they read every other
per-chip figure, and run at the setting above.

## What the figures say

**Before the seeded sweep**, at the schema's defaults, all eight chips:

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

**The stage's own counter and the checker disagreed**, and the checker was
right: the counter's rule was never set (see *The verdict never fired* above).
With it set, the `Fails:` line and `mqt-scpd drc` count the same encounters —
17q ends on 2 open wires and the check reports the one pair they make.


## The rule, and the two encounters it forgives

`min_wire_spacing` is 185 layout units. The router grid's cell is 9.91 to 10.00
units on every benchmark, so the rule spans 19 cells and what the router keeps
is 19 whole cells — about 189.5 units, a little more than the rule asks for. The
**check** asks for the rule itself, in layout units, because that is the
physical contract.

One encounter is not a violation, and it is one a working design makes on
purpose, the prototype's own geometric test from `verify_min_clearance`:

- **A junction.** Two terminals within one wire spacing of each other are one
  meeting, and everything within one and a half spacings of it belongs to that
  meeting.

**A second exemption was taken out on the user's instruction.** Two wires
that end on one component used to be forgiven near their ports as well, on
the assumption that a component's ports sit closer together than the wire
spacing. The two ports of a qubit do not: on 17q, Qb2.port0 and Qb2.port1
lie 909 layout units apart, and wire 44, bound for port0, ran 140 units
from wire 43's approach to port1 over thirty cells — forgiven by the check,
by the count and by the fence alike, because all three keyed the exemption on
the component. The prototype forgives such a pair outright, over its whole
length (`wires_connected`). Now a junction is geometry alone, in all four
places (`junctionsOf`, `meetAt`, `couldMeet`, and the count), and the
component bookkeeping that served the shortcut — `Field::guardedOnlyBy`,
`Wire::components`, `markJunctions` — is gone with it. The exemption is of the
clearance, never of the copper — two wires never share a cell.

**With it, every chip routes clean.** At six rounds, five relaxations and no
refinement, all eight benchmarks end on `Fails: 0` — every connection drawn
and `mqt-scpd drc` without a pair — where the same setting left 33q at 18
fails an hour earlier: the forgiven pass-bys were what the relaxation cascades
grew from. The final stage takes 0.04 s on 4q, 1.9 s on 17q, 9.6 s on 45q,
30 s on 57q and 41 s on 69q.

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
clearance and wire loop — each written once. The stage's own
tests call them, so what the stage is judged by and what `mqt-scpd drc` reports
cannot drift apart, and `drc.json` says where every finding is and how far the
two wires are.

The wire-loop rule is `routing::pathSelfIntersects`, the router's own test. A
revisit within four steps is the micro backtrack the router emits at every
heading change — hundreds per layout, none of them a loop, because the bend
radius is five cells.

The keepout is searched and, for now, not checked: it is baked into the
router's mask before any search runs, as an exact distance in layout units
from the cell to the polygon edge, with the approach of every port exempted so
that a wire can leave its port at all. The check's rule 4 rasterized the same
keepout *without* the exemptions and therefore reported every stub out of a
port (8 on 4q, 21 on 9q, 40 on 17q, all at zero distance); the user took it
out until the check's raster makes the same exemptions.

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
| `include/mqt-scpd/pipeline/FinalRouter.hpp`, `src/pipeline/FinalRouter.cpp` | the stage, the scene, the field, the driver, the lengthening of a resonator, and the debug pictures of the grid and of every search (`plan -d`) |
| `include/mqt-scpd/routing/MeanderInsertion.hpp`, `src/routing/MeanderInsertion.cpp` | `insertMeander`: the prototype's meander insertion, first fit or cheapest by a price |
| `include/mqt-scpd/routing/PathGeometry.hpp` | `renderedLength`, the length of a way as the sampler renders it |
| `src/pipeline/DebugSvg.hpp` | the painter of those pictures: fields as merged runs of rectangles, ways as polylines, one CSS class per layer |
| `include/mqt-scpd/drc/Rules.hpp`, `src/drc/Rules.cpp` | the rules in the cell view, and `drc.json` |
| `src/pipeline/Registry.cpp`, `src/io/Artifacts.cpp`, `bindings/bindings.cpp` | wiring |
| `python/mqt/scpd/{run,planning,plot,cli,drc}.py`, `export/klayout.py` | the stage, the phases, the layers, the report |
| `test/pipeline/test_final_router.cpp`, `test/drc/test_rules.cpp` | the invariants over all eight chips, every resonator's length among them |
| `test/routing/test_meander_insertion.cpp` | the meander on a straight run, by heading, side, box and price |

## What is still open

1. **Three of the five phases.** [handover-final-couplers.md](handover-final-couplers.md)
   says what each has to do and what the first two learned.
2. **The meander is one loop, and it is not checked against the wire it is
   part of.** One rectangular detour is the prototype's one shape; a loop
   whose legs come within the rule of the wire's own other runs breaks no
   rule of the check, which compares two wires, and the insertion refuses
   only a loop that meets the way. The 25-cell leg spacing keeps the two
   legs apart, and nothing keeps the loop from the rest of its own wire.
3. **Three of the five measured chips do not hold the rule everywhere, and two
   leave a wire unrouted** at six rounds and five relaxations; what is left
   is in `drc.json` of each run and in the `Fails:` lines. Connections 42 of
   17q and 69 of 21q find no way at this setting with or without seeds.
4. **The shorts come from the relaxation.** A search let past a neighbour
   runs a cell from its copper to save two bends. A price that tells a hug
   from a graze — decaying from the copper to the rim of the room, unlike the
   uniform disc — has not been measured.
5. **Rule 4, obstacle clearance, is out of the check** until its raster
   exempts the port approaches as the router's mask does (`sceneOf`,
   `keepoutExemptions`). The keepout itself is still searched.
6. **Nothing is compared against the prototype's own output**, only against its
   formulation as read and the counts in its checked-in run logs.
