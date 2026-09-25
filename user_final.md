# The routing stages — where the code is and how to run them

A guide to the three routing stages the pipeline runs — **Corridor**, **Detail**
and **Final**, the last with its five phases up to the couplers, the feedlines
and their repair — with what every piece of them is for, where it lives, and
every command needed to run them, draw them and check them; and, first, what
the three planning stages before them leave for them.

```text
01-capacity.fb  02-global.fb  03-assign.fb  04-corridor.fb  05-detail.fb  06-final.fb
└ Capacity      └ Global      └ Assignment  └ Corridor      └ Detail      └ Final
                              (+ chains)                                  (inner, outer, couplers, feedlines, refined)
```

Each stage draws the same wires at a finer grain than the one before it: the
Corridor stage says which partitions a wire runs through, the Detail stage
draws it cell by cell, and the Final stage draws it again as
curvature-constrained copper.

- Corridor: [summary-stage-4.md](summary-stage-4.md)
- Detail: [summary-detail-routing.md](summary-detail-routing.md)
- Final: [summary-final-routing.md](summary-final-routing.md), and
  [handover-final-couplers.md](handover-final-couplers.md) for where the
  couplers and the feedlines stand and what to do next

---

## 0. Building it

```bash
cd /Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4

# C++ only
cmake --preset release            # needed once after a new source file
cmake --build --preset release

# the Python extension. `uv sync` does NOT rebuild it when only C++ changed.
uv pip install --python .venv/bin/python --no-build-isolation --no-deps \
                --reinstall-package mqt-scpd -e .

# after any change to schemas/*.fbs
uvx nox -s schemas
```

## 1. The planning stages before

The three routing stages start from a plan that three stages before them
make. They are documented in their own summaries; what a routing run needs
to know about them is here.

| Stage | Artifact | What it decides | Summary |
| --- | --- | --- | --- |
| Capacity (`watershed`) | `01-capacity.fb` | the partitions of the free space, their borders and bottlenecks with a wire budget each, the launcher slots, the capacity chains | [summary-stage-3.md](summary-stage-3.md) |
| Global (`hanan-milp`) | `02-global.fb` | the inner circuit on the lattice each capacity chain induces, and the outer port ring the assignment consumes | [summary-stage-3.md](summary-stage-3.md) |
| Assignment (`ordered-milp`) | `03-assign.fb` | which launcher feeds which ring node, where every node is fed from, and **the feedline chains**: which resonators one feedline drives, from one launcher to the next | [task-assignment-stage.md](task-assignment-stage.md) |

The chains are what the Final stage's couplers and feedlines are routed
along. The model gives every resonator degree two on the ring — a chord to
the resonator on either side, or a chord and an end, an end being a launcher
or one of the `feedline_terminations` permitted — and `Assignment.chains`
carries every run of resonators joined by chords, in ring order, with the
launcher slot of its first and its last resonator (`start`, `end`; absent at a
termination). `Assigner::readBackChains` in
[src/pipeline/Assigner.cpp](src/pipeline/Assigner.cpp) reads them off the
solved model. A resonator is fed on the segment from its own launcher to the
next one along, so the chain of a launcher runs from that launcher past its
resonators' feeds to the next launcher.

```bash
c=17q
# the three planning stages, then everything after them
.venv/bin/mqt-scpd plan -c benchmarks/$c/config.toml -o artifacts/$c --stage assign
# a picture of each: the partitions and chains, the lattice, the ring with each node's chord
for s in capacity global assign; do
  .venv/bin/mqt-scpd plot   -c benchmarks/$c/config.toml --stage $s \
      --run-dir artifacts/$c -o artifacts/$c/$c-$s.svg
  .venv/bin/mqt-scpd render -c benchmarks/$c/config.toml --stage $s \
      --run-dir artifacts/$c -o artifacts/$c/$c-$s.gds
done
# the chains, read by hand
.venv/bin/mqt-scpd inspect artifacts/$c/03-assign.fb | grep -A 40 '"chains"'
```

Their knobs, in `config.toml`:

```toml
[design_rules]
max_feedline_utilization = 5   # how many wires one corridor between two launchers may carry
feedline_terminations    = 1   # how many chains may end at a resonator instead of a launcher

[grid]
capacity_cells_x  = 12         # the capacity grid, and the launcher slots set in from the outline
launcher_offset_x = 5
launcher_offset_y = 5

[stages.assignment]
launcher_target = 7            # how many launchers the assignment activates; per chip, no default

[stages.global]
internal_bridges = false       # whether an inner wire may cross a coupler through its own bridge
```

`--stage <name>` runs only that stage and resumes what is before it; `plan`
without `--stage` runs `capacity`, `global`, `assign`, `corridor`, `detail`
and `final` in that order.

---

## 2. The Corridor stage

### 2.1 What it does

Routes every assigned connection through the partitions of the free space,
coarsely: from the point the assignment feeds it, across a border at a time, to
the cell its target port is reached at. It decides **which** corridors a wire
runs through and **where** it crosses from one into the next, and it is the
stage that makes the wire budgets binding — a border is crossed at one of a
fixed set of slots one wire spacing apart, and no two wires take the same one.

Only the connections of the assignment are routed here. The inner circuit paid
for the free space it crosses while it was solved, so it goes to the Detail
router directly.

### 2.2 The code

| Where | What it is for |
| --- | --- |
| [include/mqt-scpd/pipeline/Stages.hpp](include/mqt-scpd/pipeline/Stages.hpp) | `ICorridorRouter` |
| [include/mqt-scpd/pipeline/CorridorRouter.hpp](include/mqt-scpd/pipeline/CorridorRouter.hpp) | `makePartitionAStarRouter()` |
| [src/pipeline/CorridorRouter.cpp](src/pipeline/CorridorRouter.cpp) | the stage, about 1200 lines |

| Symbol | Line | What it is for |
| --- | ---: | --- |
| `Place`, `Slot` | 54 | a point on a border in layout units, and the crossing slot it is |
| `Request` | 78 | one connection: where it is fed, where it ends, which partitions it may use |
| `Chord` | 123 | a straight line between two places — what the crossing tests are made of |
| `onGrid` | 140 | snaps a border sample to half a cell. Every geometric predicate downstream is exact only because of it |
| `crosses`, `overlaps`, `touches`, `meets` | 157 | whether two chords cross, run along each other, or share an end |
| `Corridors` | 241 | the graph of partitions and slots, and the search over it |
| `Corridors::route` | 347 | one wire's way through the partitions |
| `Corridors::commit` / `withdraw` | 485 | claim a slot, or give it back |
| `Corridors::blockersOf` | 512 | the wires holding the slots this one wanted |
| `Corridors::sound` | 620 | whether a route crosses another wire, runs over a pinned place, or turns straight back |
| `sweepsOf` | 704 | `rounds` and `max_relaxation` from the configuration |
| `PartitionAStarRouter::run` | 779 | the sweeps, the rip-up and the artifact |

### 2.3 Running it

```bash
c=17q
.venv/bin/mqt-scpd plan   -c benchmarks/$c/config.toml -o artifacts/$c --stage corridor -v
.venv/bin/mqt-scpd plot   -c benchmarks/$c/config.toml --stage corridor \
    --run-dir artifacts/$c -o artifacts/$c/$c-corridor.svg
.venv/bin/mqt-scpd render -c benchmarks/$c/config.toml --stage corridor \
    --run-dir artifacts/$c -o artifacts/$c/$c-corridor.gds
.venv/bin/mqt-scpd inspect artifacts/$c/04-corridor.fb | head -40
```

GDS layers: 10 `plan.partition`, 11 `plan.border`, 19 `plan.corridor`,
20 `plan.slot` — every place a wire may cross a border, taken or not.

### 2.4 Its knobs

```toml
[stages.corridor]
rounds         = 12   # sweeps over the connection list
max_relaxation = 30   # neighbours a failed connection may rip up
```

Not a knob: where the slots are. `[stages.capacity] crossing_pitch` decides how
finely a border is divided, and it is a planning pitch and **not** a clearance
rule.

### 2.5 Its tests

[test/pipeline/test_corridor_router.cpp](test/pipeline/test_corridor_router.cpp)
— one wire per crossing place, no two wires cross inside a partition, no wire
runs over a place another is pinned to, no crossing outside the ring the ports
feed from, no wire turns straight back, and two runs give the same answer.

```bash
./build/release/test/pipeline/mqt-scpd-pipeline-test --gtest_filter='*Corridor*:Corridors.*'
```

---

## 3. The Detail stage

### 3.1 What it does

Draws every wire cell by cell on the detail grid — 750 × 750 cells on the
17-qubit chip — eight-connected, and holds the design rule between **every**
pair of wires. The corridor stage's route is a seed and not a constraint: the
last pass is free to move a wire off it entirely.

Four passes, in the prototype's order: route inside each partition, join the
pieces in corridor order, draw every wire again from end to end with rip-up and
relaxation, and finally seed the inner circuit over whatever free space is left.

### 3.2 The code

| Where | What it is for |
| --- | --- |
| [include/mqt-scpd/pipeline/Stages.hpp](include/mqt-scpd/pipeline/Stages.hpp) | `IDetailRouter` |
| [include/mqt-scpd/pipeline/DetailRouter.hpp](include/mqt-scpd/pipeline/DetailRouter.hpp) | `makePixelAStarRouter()` |
| [src/pipeline/DetailRouter.cpp](src/pipeline/DetailRouter.cpp) | the stage, about 2000 lines |

| Symbol | Line | What it is for |
| --- | ---: | --- |
| `Tuning`, `tuningOf` | 101 | the knobs, converted onto the grid |
| `Rule`, `ruleOf` | 144 | the design rule as offsets on this grid: `ceil(spacing / cell) - 1` cells |
| `Query` | 180 | what one search may enter and what it is charged for |
| `Canvas` | 212 | the grid: what is blocked, which partition holds a cell, which wire owns it, how many wires guard it — and the one A\* every pass uses |
| `Canvas::search` | 309 | that search |
| `Canvas::armClearance` | 420 | switches the clearance field on, once the seed is drawn |
| `Canvas::guard` / `unguard` | 471 | charge and discharge a cell's clearance |
| `cellAt` | 741 | the one conversion of a plan's place to a cell: `floor(onGrid(toCell(p)))`, never a rounding |
| `labelsOf` | 761 | the partition labels back on the grid, filled from the outlines rather than grown again |
| `PixelAStarRouter::run` | 779 | the four passes |
| `drawInsidePartitions` | 1047 | pass 1: every corridor piece inside its own partition |
| `placeWhatIsLeft` | 1213 | the rescue of pass 1 |
| `joinPieces` | 1258 | pass 2: a concatenation in corridor order, not a search |
| `drawWhatIsLeftWhole` | 1301 | draws a wire the pieces could not join, in one search |
| `seedInnerCircuit` | 1945 | the inner circuit, over what the ring left |
| `crossBoundaryRouting` | 1548 | pass 3: the rounds |
| `reroute` | 1584 | one wire's turn: the band, then the relaxation ahead and behind |
| `letGo` / `takeUpAgain` | 1653 | ripping a wire is discharging its clearance and leaving its copper |
| `recount` | 1493 | how many cells of each wire lie within the rule of another, counted with the wire off the field |
| `artifactOf` | 1990 | the artifact |

### 3.3 Running it

```bash
c=17q
.venv/bin/mqt-scpd plan   -c benchmarks/$c/config.toml -o artifacts/$c --stage detail -v
.venv/bin/mqt-scpd plot   -c benchmarks/$c/config.toml --stage detail \
    --run-dir artifacts/$c -o artifacts/$c/$c-detail.svg
.venv/bin/mqt-scpd render -c benchmarks/$c/config.toml --stage detail \
    --run-dir artifacts/$c -o artifacts/$c/$c-detail.gds
```

GDS layers: 21 `plan.wire`, 22 `plan.inner-wire`, 23 `plan.clearance` — a band
one clearance wide around every wire, so two wires closer than the rule are two
bands whose overlap a boolean finds exactly.

### 3.4 Its knobs

```toml
[stages.detail]
corridor_spacings      = 4      # the band around a wire's own way, in wire spacings
rounds                 = 30     # sweeps over the wire list
max_relaxation         = 30     # neighbours a failed wire may let go of
obstacle_penalty_reach = 185.0  # layout units
obstacle_penalty       = 10     # a price per step
orderings              = 16     # orders the wires of one partition are tried in
```

Not a knob: the clearance. It is `cells_for(min_wire_spacing, detail) - 1`,
which comes to 4 to 9 cells on the eight chips because their grids differ.

### 3.5 Its tests

[test/pipeline/test_detail_router.cpp](test/pipeline/test_detail_router.cpp) —
every wire eight-connected and free of obstacles, no wire meets itself, no two
share a cell or cross between cells, first and last cell are the feed and the
target, **no two wires within the design rule**, and two runs give the same
answer.

```bash
./build/release/test/pipeline/mqt-scpd-pipeline-test --gtest_filter='*Detail*'
```

---

## 4. The Final stage

### 4.1 What it does

The Detail stage leaves a **centre line on a cell grid**. The Final stage draws
each of those wires again as curvature-constrained copper over the Dubins
primitives of one bend radius, on a grid whose cell is about ten layout units —
1425 × 1425 cells on the 17-qubit chip, 3016 × 3016 on the 69-qubit one — and
then finishes what the plan left open: the coupler that carries a resonator's
`ResonatorSource` port, the feedline chains from launcher to launcher that
drive the couplers, and their refinement.

It runs five phases and leaves a snapshot after each:

| # | Phase | What it leaves |
| --- | --- | --- |
| 1 | `inner` | the inner circuit, inside its unit cells |
| 2 | `outer` | the ring, against the inner circuit and itself; every resonator at least `meander_length` long, with a meander where it was shorter |
| 3 | `couplers` | every resonator cut back to its coupler, where the way left to the qubit port is `target_resonator_length`; the coupler bodies; every edge of every feedline chain drawn |
| 4 | `feedlines` | every wire drawn again under the feedline constraints, the resonators from their couplers and made the target length exactly; then the repair, which turns couplers while fails are left |
| 5 | `refined` | the refinement of the feedline routing: `feedline_refinement_rounds` rounds (**zero by default now**, so the phase does nothing until the knob is turned up) in which every wire of the ring and every edge at a launcher is drawn again against the centring price, under the feedline constraints; a wire that finds no wider way keeps the one it had |

**Where the chains come from.** The Assignment stage's model gives every
resonator degree two on the ring — a chord to the resonator on either side, or
a chord and an end, an end being a launcher or a permitted termination — and a
run of resonators joined by chords is what one feedline drives.
`Assignment.chains` carries each of them in ring order with the launcher slot
of its first and its last resonator; the Final stage routes each chain
launcher → coupler → … → coupler → launcher. A chain that ends at a
termination simply stops at its last coupler.

**Where the coupler sits, and how it is shaped.** The resonator's way is cut
where the way left to the qubit port is the target length, and a dogleg is put
in front: from the anchor the way runs `coupler_length` straight on the heading
across the coupler's orientation, turns a quarter onto the orientation, runs
the straight start and joins what is left. The body spans that coupling run,
`coupler_height` across on the side away from the turn, and the feedline runs
along its far edge — along the ring, where the chains run. Which orientation,
mirrored or not, with the feedline along the run or against it, with or
without a second dogleg: that is the option set, 64 per resonator, and a greedy
search per chain takes for every coupler the option whose two edges to its
chain neighbours turn least. See
[decision 0032](docs/design/decisions/0032-the-coupler-couples-along-the-ring.md).

Input: `05-detail.fb` plus the chip and the three planning artifacts.
Output: `06-final.fb` — the wires, the inner wires, the feedline edges with
what each runs between (`feedline_edges`), the couplers with the port each
creates, and the five snapshots — and `drc.json` when the check is run.

---

### 4.2 The code, piece by piece

#### The stage

| Where | What it is for |
| --- | --- |
| [include/mqt-scpd/pipeline/Stages.hpp](include/mqt-scpd/pipeline/Stages.hpp) | `IFinalRouter`, the stage interface. One call does coupler placement **and** feedline routing, because the two are one fixpoint |
| [include/mqt-scpd/pipeline/FinalRouter.hpp](include/mqt-scpd/pipeline/FinalRouter.hpp) | `makeDubinsFinalRouter()`, the one implementation this build ships |
| [src/pipeline/FinalRouter.cpp](src/pipeline/FinalRouter.cpp) | the whole stage, about 5000 lines |
| [include/mqt-scpd/routing/MeanderInsertion.hpp](include/mqt-scpd/routing/MeanderInsertion.hpp), [src/routing/MeanderInsertion.cpp](src/routing/MeanderInsertion.cpp) | `insertMeander`, the prototype's meander insertion: one loop spliced into a straight run of a path, first fit or cheapest by a price; with `MeanderOptions.exact` the prototype's strict insertion, which aims at the length exactly within a tolerance and tries the widest free strip along every pair |
| [include/mqt-scpd/routing/CouplerInsertion.hpp](include/mqt-scpd/routing/CouplerInsertion.hpp), [src/routing/CouplerInsertion.cpp](src/routing/CouplerInsertion.cpp) | `spliceCouplerDogleg`: cuts a path where the rest is the target length and puts the dogleg in front; `leadStraight` is the coupling run before the turn |
| [include/mqt-scpd/routing/CrossingConstraints.hpp](include/mqt-scpd/routing/CrossingConstraints.hpp), [src/routing/CrossingConstraints.cpp](src/routing/CrossingConstraints.cpp) | the crossing rule: a cell within ten of a feedline's straight run may be entered at a right angle to it only, a cell around its bends or its first and last ten cells never. Split out of the router so that the search and the design-rule check run one test |
| [src/pipeline/Assigner.cpp](src/pipeline/Assigner.cpp) | `readBackChains` (line 420): the chains, read off the model's chord, launcher and termination variables |

Inside `FinalRouter.cpp`, in the order it reads:

| Symbol | Line | What it is for |
| --- | ---: | --- |
| `Tuning` | 90 | every knob, already converted onto the grid. Nothing here is a cell count standing for a distance; the target length and its tolerance, the coupler body, `repair_trials` and `stop_after` are here too |
| `tuningOf` | 155 | the one place a design rule or a configured length becomes cells |
| `Scene` | 239 | the router grid, the mask it searches on, the target cell and arrival heading of every port, and the cell every launcher stands on |
| `blockOutsideTheSources` | 259 | blocks everything between the chip outline and the rectangle the launcher slots stand on, so no wire slips **around** the sources |
| `sceneOf` | 289 | builds that scene: obstacle raster with the keepout baked in, one band per port, one target dug beyond each band, the launcher cells freed |
| `Stencil`, `stencilOf` | 435 | the offsets of one clearance disc, and what enters it on each step |
| `Field` | 495 | two fields per cell: which wire holds the copper, and how many wires keep their clearance over it. A wire charges its room when it is put down and gives it up when it is taken off |
| `Wire` | 624 | one connection: its two ends, the way it has, whether it is drawn, placed, routed, too short or too long; for a resonator the run from its last cell to the port (`anchorGap`) and, once its coupler is in, the head of its way (`arc`) and its coupler; for a feedline edge which edge it is, whether it ends at a launcher (`terminal`) and its stubs; for a conventional wire the one edge it may cross (`bridged`); `ripped` when a neighbour let go of it |
| `Pass` | 705 | what one run of the driver does: rounds, relaxation, band, stub, and whether the feedline constraints apply |
| `Driver` | 895 | the one rip-up-and-reroute loop. The prototype writes it out four times |
| `Driver::needsLength`, `requiredLength`, `aimAtTarget` | 976 | how long a resonator's way has to be: at least `meander_length` less `anchorGap` in the outer routing, the target length exactly once the coupler is in |
| `Driver::lengthen` | 1039 | lengthens a resonator's way with `insertMeander`: the meander may enter what the search could enter; first fit in phase 1, cheapest by the price field in the relaxation; the strict insertion, in the widest free strip along the pair, once the coupler is in |
| `Driver::sweep` | 1155 | the rounds, over wires that start on the way they have. Ends when no wire fails, or after four rounds without progress, on the pass's `Fails:` line |
| `Driver::failsOf` | 1305 | the fails of a set of wires, counted with every wire down: unrouted, open, short, long, crossing a feedline and meeting itself, each wire once. The wire list is taken as it is at the time, because the coupler insertion appends the edges of the chains to it |
| `Driver::sayResonatorLengths` | 1395 | one `-v` line per resonator at the end of the stage, against the target and its tolerance |
| `Driver::refine` | 1479 | routes every wire again against a centring price; under the feedline constraints in phase 5 |
| `CouplerOption`, `Coupler`, `Waypoint`, `Edge` | 1552 | one way a coupler can sit, a coupler with all of them, a point of a chain, an edge of a chain |
| `Driver::insertCouplers` | 1698 | **phase 3**: the approaches of every port, the options of every resonator, the chains from the assignment, the greedy search per chain, the commit, the edges, and the `coupler insertion:` line |
| `Driver::optionsOf`, `makeOption` | 1926 | the 64 options of a resonator and the geometry of one: the cut, the head, the body, the feedline ports, and what refuses it — off the grid, on artwork, on a body, in a port's approach, longer than the target allows, or a spliced way that meets itself |
| `Driver::edgeWayOf`, `fenceCommittedEdges` | 2133 | the way an edge has now, and the rule that every committed edge stands in the way of the edges routed after it, inflated by the clearance — the two edges that meet at a coupler keep no clearance from each other but never cross |
| `Driver::fenceResonators`, `corridorOfEdge`, `routeEdge` | 2198 | the search of one edge: a band of 200 cells around the chord, the bodies, every resonator inflated by the clearance (the two couplers' own within their reach excepted), the approaches of every port and the committed edges closed, the room of every wire priced, the stubs sealed |
| `Driver::angleCostOf`, `localCost`, `heuristicCost` | 2409 | what an option costs: its two edges routed and priced by how much they turn, plus their lengths, their difference, a second dogleg and the room it takes; the beeline estimate that pre-screens the options |
| `Driver::optimizeChain` | 2582 | the greedy local search: up to five passes, every coupler takes the cheapest of its options |
| `Driver::applyOption`, `sayCoupler` | 2678 | put an option in place — the resonator cut, its search from the turn's end, the body an obstacle — and the `coupler N for resonator M` line |
| `Driver::ringWithEdges`, `assignBridges` | 2738 | the sweep of phase 4: the edge into a coupler before its resonator, the edge out after it; and which edge a conventional wire may cross |
| `Driver::beginPass`, `constrainByFeedlines` | 2795 | what a pass under the feedline constraints starts with and how a search is fenced by the edges and priced around them |
| `Driver::snapshot`, `restore`, `turnCoupler`, `unsettledBy`, `repair` | 2876 | **the repair**: a trial is a snapshot, a coupler turned to another option with its two edges drawn again, a one-round sweep over what that unsettles, and a count; the turn that leaves strictly fewer fails is kept |
| `Driver::fixPlaces` | 3167 | charges the places no wire may be moved off: its target, the straight run out of its source, and for a resonator drawn from its coupler its head |
| `Driver::seed` | 3202 | puts every wire of a pass down on the way it has |
| `Driver::attempt` | 3283 | one wire's turn, the prototype's two phases; under the feedline constraints a wire whose way already holds every rule is settled without a search, unless a neighbour let go of it |
| `Driver::whatBlocks` | 3461 | for `-v 1`: what stands in the way of a wire that found nothing — the neighbours, the feedlines, the crossing rule, the bodies — by searching again with each left out |
| `Driver::isLegal` | 3518 | whether the way a wire has holds the rule against every other wire, the crossing rule, and its length |
| `Driver::conflictsOf`, `meetAt`, `sharedCoupler`, `couplerReach` | 3548 | how many cells of a way lie within the rule of another wire; where two wires meet, the rule does not bind — at a junction, and between a resonator and the edges of its own coupler within the coupler's reach |
| `Driver::search` | 3686 | the call into the shared `DubinsRouter`: `routeOrthogonal` under the feedline constraints, with the crossing rule lifted around the wire's own coupler; the head put in front of a resonator's way |
| `Driver::buildCorridor` | 3760 | the cells a search may enter: the free space within the band around the way the wire has, less the coupler bodies |
| `Driver::fence` | 3707 | closes the ways of the fence wires, inflated by the clearance; the meeting of two wires stays open |
| `Driver::priceLane` / `priceApproaches` / `priceRoom` | 3823 | the lane price of the relaxation, the price on the approaches around a search, the centring price of the refinement |
| `Driver::drawSearch` | 4321 | the picture of one search, the failing edge searches of the insertion included |
| `wiresOf` | 4619 | reads the plan into wires |
| `snapshotOf` | 4749 | what one phase drew, each wire with its length, the edges and the couplers from phase 3 on |
| `portOf`, `couplersOf` | 4805 | the port a waypoint stands for, and the couplers as the artifact carries them: connection, created port, centre, rotation along the coupling run, length, height |
| `DubinsFinalRouter::run` | 4863 | the five phases and the artifact |

#### What the stage reuses

| Where | What |
| --- | --- |
| [include/mqt-scpd/routing/DubinsRouter.hpp](include/mqt-scpd/routing/DubinsRouter.hpp) | the curvature-constrained A\*, its corridor mask and its two proximity prices; `routeOrthogonal` for the crossing rule, `setCrossingExemption` for the room around a wire's own coupler, `freeStripAlong` for the strict meander's box |
| [include/mqt-scpd/routing/PathGeometry.hpp](include/mqt-scpd/routing/PathGeometry.hpp) | `samplePath`, `renderedLength`, `reconstructSegments` |
| [include/mqt-scpd/routing/SelfIntersection.hpp](include/mqt-scpd/routing/SelfIntersection.hpp) | whether a path meets itself; the design-rule check calls the same function |
| [include/mqt-scpd/grid/GridMetrics.hpp](include/mqt-scpd/grid/GridMetrics.hpp) | `routerGrid`, and `cellsFor` — the one conversion from a rule to cells |
| [include/mqt-scpd/grid/Rasterize.hpp](include/mqt-scpd/grid/Rasterize.hpp) | the obstacle raster with the keepout baked in |
| [include/mqt-scpd/grid/PortBands.hpp](include/mqt-scpd/grid/PortBands.hpp) | the band around a port and the target dug beyond it |

#### The design-rule check

| Where | What |
| --- | --- |
| [include/mqt-scpd/drc/Rules.hpp](include/mqt-scpd/drc/Rules.hpp) | `CellView` — what a rule sees: every wire with the ports at its ends, the couplers with their reach — `viewOfFinal`, which builds it from a final routing, and `checkCells` |
| [src/drc/Rules.cpp](src/drc/Rules.cpp) | rule 1 wire clearance (an edge of a chain is left out against the wires that cross it, binds against the resonators and against the other edges, and is left alone near a coupler it shares), rule 2 feedline orthogonality (line 275, the router's own test, on no edge of a chain), rule 3 wire loop, and the writer of `drc.json`. Rule 4, obstacle clearance, is out for now |
| [python/mqt/scpd/drc.py](python/mqt/scpd/drc.py) | reads `drc.json` back and counts it; only the active findings decide the exit code |

#### Schema, wiring and rendering

| Where | What |
| --- | --- |
| [schemas/artifacts.fbs](schemas/artifacts.fbs) | `FeedlineChain` and `Assignment.chains`; `FinalWire`, `FinalPhase`, `FinalRouting` with `feedlines`, `feedline_edges` and `couplers` |
| [schemas/design.fbs](schemas/design.fbs) | `CpwCoupler`: the connection it completes, the port it creates, centre, rotation, length, height |
| [schemas/config.fbs](schemas/config.fbs) | `FinalParams`, `repair_trials` and `stop_after` among them, and `GridParams.router_cell_size` |
| [src/pipeline/Registry.cpp](src/pipeline/Registry.cpp) | `finalRouters()`, default `"dubins"` |
| [src/io/Artifacts.cpp](src/io/Artifacts.cpp) | the artifact is refused unless every wire steps by at most one cell, every chain names ring nodes that exist, and the edges describe the feedlines |
| [bindings/bindings.cpp](bindings/bindings.cpp) | `route_final(...)` and `check_final(...)`, which reads the same view as the tests |
| [python/mqt/scpd/run.py](python/mqt/scpd/run.py) | the stage in the run directory |
| [python/mqt/scpd/planning.py](python/mqt/scpd/planning.py) | `_final` — the artifact as the shapes a picture draws, per phase, the coupler bodies among them |
| [python/mqt/scpd/plot.py](python/mqt/scpd/plot.py) | the `l-coupler` layer |
| [python/mqt/scpd/export/klayout.py](python/mqt/scpd/export/klayout.py) | one GDS layer per phase, 24 to 28, beside the clearance band on 23 |
| [python/mqt/scpd/cli.py](python/mqt/scpd/cli.py) | `--phase` for `plot` and `render`, and the `drc` command |

#### The tests

| Where | What |
| --- | --- |
| [test/pipeline/test_final_router.cpp](test/pipeline/test_final_router.cpp) | over all eight chips, one run per chip shared by every test: every connection drawn, no wire meets itself, every wire starts at its feed and a resonator at its coupler, every resonator the target length within tolerance, every resonator has a coupler with its port, every edge drawn and crossed at right angles only, consecutive edges of a chain meet in exactly one cell, no two wires within the rule, two runs give the same answer, the five snapshots, the `Fails:` line and the pictures |
| [test/routing/test_meander_insertion.cpp](test/routing/test_meander_insertion.cpp) | the meander on a straight run |
| [test/routing/test_coupler_insertion.cpp](test/routing/test_coupler_insertion.cpp) | the dogleg and the cut |
| [test/drc/test_rules.cpp](test/drc/test_rules.cpp) | what each rule finds and what it forgives, on wires built by hand: a right-angle crossing of a feedline, a run beside one, and a resonator beside its own coupler |
| [test/python/unit/test_planning.py](test/python/unit/test_planning.py) | the five phases, the coupler phase drawing the couplers and the edges |

---

### 4.3 Running it

#### One chip, end to end

```bash
c=17q

# every stage from scratch
.venv/bin/mqt-scpd plan -c benchmarks/$c/config.toml -o artifacts/$c

# or just the Final stage, resuming the run that is already there
.venv/bin/mqt-scpd plan -c benchmarks/$c/config.toml -o artifacts/$c --stage final

# with -v it says what it is doing while it does it
.venv/bin/mqt-scpd plan -c benchmarks/$c/config.toml -o artifacts/$c --stage final -v
```

The first two phases read as before:

```text
[final]     0.01s  grid 1425x1425 cells of 9.97 layout units | clearance 19 cells for a rule of 18.55 | stub 11 cells | band 209 cells | bend 7125 | wire price 4 | obstacle price 1 over 11 cells | resonators 301 cells long
[final]     0.02s  inner routing: 14 of 14 wires start on the way the Detail stage drew
[final]     0.05s  inner routing: 14 of 14 drawn, 0 unrouted, 0 open, 0 short | Fails: 0
[final]     0.06s  outer routing: 58 of 58 wires start on the way the Detail stage drew
[final]     0.92s  outer routing round 0 forward : tried 58, routed 56, unrouted 2, open 0, short 0 | Fails: 2
...
[final]     0.90s  outer routing: 58 of 58 drawn, 0 unrouted, 0 open, 0 short | Fails: 0
```

Then the coupler insertion, which ends on one line: how many couplers on how
many resonators, the chains, the edges drawn, how many of the couplers stand
on a diagonal, the total angle cost of the edges (the prototype's
`[ANGLECOST]`), how many greedy passes improved something, how many edges were
weighed without the clearance to their neighbour, and the time:

```text
[final]    23.99s  coupler insertion: 17 couplers on 17 resonators, 4 chains, 20 of 20 edges drawn, feedline angle cost 96, 1 greedy passes, 23.1s
```

Then the feedline routing, the inner circuit under the same constraints, the
repair, and the stage's own end:

```text
[final]    23.99s  feedline routing: 65 wires, up to 6 rounds, 5 relaxations each way
[final]    27.12s  feedline routing round 0 forward : tried 65, routed 55, unrouted 0, open 54, short 1 | Fails: 55
[final]    29.34s  feedline routing round 1 backward: tried 55, routed 46, unrouted 0, open 50, short 3 | Fails: 53
...
[final]    38.81s  feedline routing: 65 of 65 drawn, 0 unrouted, 12 open, 2 short, 1 long, 4 crossing a feedline | Fails: 14
[final]    38.81s  inner routing under the feedlines: 14 of 14 drawn, 0 unrouted, 0 open, 0 short, 0 long, 0 crossing a feedline | Fails: 0
[final]    94.30s  feedline repair: coupler 6 turned to option 7 after 34 trials | Fails: 13
[final]   129.92s  feedline repair: coupler 8 turned to option 1 after 68 trials | Fails: 12
[final]   151.56s  resonator 1 to Qb15.port0: 2342 units of way + 113 to the port = 2456 units, meets 2500 within 100
...
[final]   151.56s  feedline refinement: 65 wires, 5 rounds
[final]   157.02s  feedline refinement round 0 forward : moved 9 of 65, 10 too close to another
...
[final]   151.56s  resonators: 17 in all, 2456 to 2874 units, 1 off 2500 by more than 100
[final]   151.56s  final routing: 91 of 92 drawn, 1 unrouted (1 feedline edges), 10 open, 0 short, 1 long, 2 crossing a feedline | Fails: 11
```

A run with `stop_after` set says so on one line of its own, just before the
resonator lengths: `the stage stops after couplers, as stop_after asks`.

What each word counts:

| Word | What it counts |
| --- | --- |
| `tried` / `routed` | wires this round offered a way, and of those the ones that found one |
| `unrouted` | wires with no way of their own: in the outer routing, still on the Detail stage's seed; in the feedline routing, an edge of a chain without a way. `(N feedline edges)` says how many of the unrouted are edges |
| `open` | wires with a way of their own that is not settled: in a round, let go of by a wire that relaxed past it and not drawn again yet; at the end of a pass, too close to another wire |
| `short` | resonators shorter than they have to be: `meander_length` in the outer routing, the target length less the tolerance once the coupler is in |
| `long` | resonators longer than the target length and the tolerance allow, once the coupler is in — nothing spliced in makes a way shorter, so the wire has to find a shorter way or fail |
| `crossing a feedline` | wires that cross an edge between two couplers other than at a right angle, or enter the room around its bends — the crossing rule, by the router's own test |
| `meeting itself` | wires whose way crosses or touches itself, rule 3 of the check; a search never returns one, so it is a resonator that kept a way it never drew. Printed only when there is one |
| `open`, for a feedline edge | an edge within the rule of a resonator other than its own coupler's, or within the rule of another edge they do not share a coupler with. A feedline keeps the clearance from every resonator and from every other feedline; the conventional and inner wires cross it instead, at a right angle |
| `coupler insertion:` | see above |
| `N on a diagonal` | couplers whose body lies on a diagonal rather than along an axis. It was zero on every chip until the body was measured in cells of its own orientation |
| `N weighed without the clearance to their neighbour` | how often the greedy had to weigh an edge against a neighbour's copper alone, because the full clearance left it no way. The fallback keeps an edge that would otherwise go undrawn, and it makes the clearance depend on the data — **quote this number whenever you quote a fail count**, or two runs are not comparable |
| `feedline repair: coupler N turned to option M after T trials` | a turn the repair kept, and the fails it left; `none with fewer than N fails` when a wave brought nothing; `nothing left to try` when every option of every candidate was tried |
| `resonator N to <port>` | the rendered length of its way, the run from the way's last cell to the port, the two together, and whether that meets the target within the tolerance, falls short of it or is over it |
| `Fails:` | every kind above, each wire counted once. The pass's own last line and the stage's `final routing:` line count with every wire down, by the test `mqt-scpd drc` makes |
| `moved` | wires the refinement found a wider way for and took: a way found is taken only when it is not worse than the way the wire had, against every other wire, and crosses no feedline other than at a right angle; `-v 1` says `the way found is worse … kept its way` otherwise |

`-v 1` says one line more per thing looked at, indented. In the insertion,
per resonator how many options fit, per chain every turn the greedy took,
per edge whether it was drawn, and per coupler where it sits and what is left
of its resonator:

```text
[final]     0.93s    coupler for resonator 1: 11 options
[final]     1.41s    chain 0 coupler of resonator 10: option 0 -> 2, cost 82442
[final]    23.98s    feedline edge 72 of chain 0: 171 cells
[final]    23.98s    edge of chain 0 from 2 to 3: no way; source (1329, 792) heading 6 -> (1340, 792) free, target (1308, 437) heading 6 -> (1287, 437) free | waypoint 2 way (1329,789)..(1159,755) 233 cells, ...
[final]    23.99s    coupler 0 for resonator 1 to Qb15.port0: anchor (1037, 1323), heading 0, feedline against the run, 2342 units of way + 113 to the port = 2456, target 2500
```

`cost` is the option's local cost: ten thousand per eighth of a turn of its
two edges, plus their lengths, the difference between them, twenty thousand
for a second dogleg, and a hundred per cell of another wire's room under the
body. An edge that finds no way says where its two ends are and whether each
lies inside the search's band.

In the feedline routing, every wire says what it did — kept its way, found
one, or found none and what stood in its way, which the stage finds out by
searching again with one constraint left out at a time:

```text
[final]    24.01s    wire 1 · round 0 forward: keeps its way, which holds
[final]    27.30s    wire 5 · round 0 forward · normal: no way, relaxing · in the way: the neighbours
[final]    27.35s    wire 5 · relax 1: let go of 6, fence 6 and 4 · no way
[final]    28.02s    wire 11 · relax 2: let go of 13, fence 13 and 10 · found 254 cells · too long for its target: 260 of 240 cells
```

`in the way:` names the neighbours, the feedlines, the crossing rule, the
coupler bodies — any of them alone would have let the wire through — or
`several together`, or `the band itself`. In the repair, every trial says
what it came to:

```text
[final]    40.44s    repair trial 1: coupler 2 option 10: 17 fails against 15
[final]    40.48s    repair trial 2: coupler 2 option 0: its edges find no way
```

`-v` works on the Corridor and the Detail stage too, and each ends on a
`Fails:` line of its own.

> **Trap.** `--stage` reads the configuration **from the run directory**, not
> from the file named on the command line. After editing `benchmarks/$c/config.toml`,
> copy it over first: `cp benchmarks/$c/config.toml artifacts/$c/config.toml`.

#### Stopping after one phase

The Final stage runs five phases, and `stop_after` names the last one to run.
What follows it is not run at all, so a stop costs what the phases before it
cost and nothing more:

```toml
[stages.final]
stop_after = "couplers"
```

It takes `inner`, `outer`, `couplers`, `feedlines` or `refined`, the five
names `--phase` takes, and is empty by default, which runs the whole stage.
The stage still says what it came to, still counts its fails and still writes
its artifact, so a stopped run is drawn, rendered and checked like any other.
Asking a stopped run for a phase it does not hold is an error rather than a
picture of the wrong thing.

| Stop after | What has run | What it is good for |
| --- | --- | --- |
| `inner` | the inner circuit | the unit cells alone |
| `outer` | the ring on top of it | the seeded sweep, before any coupler |
| `couplers` | the insertion and the chain edges | where the couplers went and what the edges cost |
| `feedlines` | the sweep under the feedline constraints and the repair | the routing without the refinement |
| `refined` | everything | the same as no stop |

The two slow parts have knobs of their own, and switching them off is not the
same as stopping before them: `repair_trials = 0` leaves out the repair but
still runs the refinement, and `feedline_refinement_rounds = 0` leaves out the
refinement alone.

What a stop saves, on the 4-qubit chip, is the whole cost of the phases after
it:

| `stop_after` | Final stage |
| --- | ---: |
| `outer` | 0.04 s |
| `couplers` | 3.9 s |
| `feedlines` | 3.8 s |
| `refined`, or unset | 5.5 s |

#### Debug pictures of the Final stage

```bash
.venv/bin/mqt-scpd plan -c benchmarks/$c/config.toml -o artifacts/$c -d
```

`-d` makes the Final stage hand out one SVG per thing it looks at, written
into `artifacts/$c/debug/`:

| File | What it shows |
| --- | --- |
| `final-grid.svg` | the whole router grid before anything is drawn: the artwork with its keepout, the obstacle halo the search pays for, every port as the chip carries it, every wire's seed, and every source and target with its heading |
| `final-00017-outer-r0f-w1-relax1.svg` | one search of the outer routing, taken right after it: picture 17, the `outer` pass, round 0 forward, wire 1, relaxation level 1 (`normal` is phase 1, `refine` the refinement) |
| `final-00103-edge-r0f-wf0-edge.svg` | one edge search of the coupler insertion **that found nothing**, with the chain and the two waypoints in the title: the band, the bodies and the committed edges closed, the room of the wires priced |
| `final-00511-feedline-r0f-w5-normal.svg` | one search of the feedline routing: the fences of the feedlines are closed cells, the price around them is the red band |

The wire is the ring index, `i3` the third wire of the inner circuit, `f5` the
sixth edge of the feedline chains.

#### The five pictures and the GDS

```bash
c=17q
for p in inner outer couplers feedlines refined; do
  .venv/bin/mqt-scpd plot -c benchmarks/$c/config.toml --stage final --phase $p \
      --run-dir artifacts/$c -o artifacts/$c/$c-final-$p.svg
done

# the end state, without naming a phase
.venv/bin/mqt-scpd plot -c benchmarks/$c/config.toml --stage final \
    --run-dir artifacts/$c -o artifacts/$c/$c-final.svg

# one GDS with every phase on a layer of its own
.venv/bin/mqt-scpd render -c benchmarks/$c/config.toml --stage final \
    --run-dir artifacts/$c -o artifacts/$c/$c-final.gds
```

From the `couplers` phase on, a picture draws the coupler bodies (orange
rectangles along the coupling run) and the feedline edges (with the inner
wires, in blue). A run stopped early with `stop_after` carries only the
phases it ran; asking it for one of the others is an error that names the
ones it holds, rather than a picture of the end state under the wrong name.
In KLayout the layers are:

| Layer | Name |
| ---: | --- |
| 1 | the chip artwork |
| 2 | the ports, datatype = the role |
| 23 | `plan.clearance` — a band one clearance wide around every wire |
| 24 | `final.inner-routing` |
| 25 | `final.outer-routing` |
| 26 | `final.coupler-insertion` — the resonators cut back to their couplers, and the edges |
| 27 | `final.feedline-routing` — every wire under the feedline constraints, after the repair |
| 28 | `final.feedline-refinement` |

#### Checking the design rules

```bash
.venv/bin/mqt-scpd drc artifacts/17q
```

Writes `artifacts/17q/drc.json`, prints how many findings it holds, and exits
nonzero when an active rule found something. Rule 1 names the two wires, the
place and the distance; rule 2 the wire that crosses a feedline other than at
a right angle and where; rule 3 the wire that meets itself. Wires are numbered
as the artifact holds them: the ring first, then the inner circuit, then the
feedline edges. Under rule 1 an edge of a chain is compared with the
resonators and with the other edges, and left out against the conventional and
inner wires, which cross it on purpose; it is counted in `feedlines_skipped`.
The exemption near a coupler covers any two wires that share it, so a
resonator and its own two edges, and those two edges with each other, are free
within the coupler's reach. An edge at a launcher is an edge like any other
here: it used to be checked against every wire, which walled off the plane
from the chip edge to the first coupler. Rule 2 binds on no edge at all: the
search draws every edge free of the crossing rule and fences it by every other
edge instead. Rule 2 reads a feedline's straight runs off its
cells, as the router does, so the stage's `Fails:` line and the check agree.

#### All eight chips

```bash
for c in 4q 9q 17q 21q 33q 45q 57q 69q; do
  cp benchmarks/$c/config.toml artifacts/$c/config.toml
  .venv/bin/mqt-scpd plan   -c benchmarks/$c/config.toml -o artifacts/$c --stage final -v
  for p in inner outer couplers feedlines refined; do
    .venv/bin/mqt-scpd plot -c benchmarks/$c/config.toml --stage final --phase $p \
        --run-dir artifacts/$c -o artifacts/$c/$c-final-$p.svg
  done
  .venv/bin/mqt-scpd render -c benchmarks/$c/config.toml --stage final \
      --run-dir artifacts/$c -o artifacts/$c/$c-final.gds
  .venv/bin/mqt-scpd drc artifacts/$c
done
```

The Final stage takes seconds on 4q, half a minute on 9q, three minutes on 17q,
six on 21q, twenty on 33q, half an hour on 45q and forty minutes on 57q at the
shipped setting; most of that is the repair.

#### The tests

```bash
cmake --build --preset release && ctest --preset release
uv run --no-sync pytest test/python/unit

# just this stage, one chip; every test of a chip shares one run of it
./build/release/test/pipeline/mqt-scpd-pipeline-test --gtest_filter='*Final.*/9q'
./build/release/test/drc/mqt-scpd-drc-test
./build/release/test/routing/mqt-scpd-routing-test --gtest_filter='CouplerInsertion.*:MeanderInsertion.*'
```

#### Reading an artifact by hand

```bash
.venv/bin/mqt-scpd inspect artifacts/17q/06-final.fb | head -40
.venv/bin/mqt-scpd inspect artifacts/17q/03-assign.fb | grep -A 30 chains
```

---

### 4.4 Its knobs

Everything is a length in layout units or a price. A shipped configuration
carries only what differs from the defaults; every benchmark carries a
`[stages.final]` section with `rounds = 6`, `max_relaxation = 5` and
`refinement_rounds = 0`, the setting the seeded sweep was measured at; the
feedline refinement of phase 5 runs its default of five rounds.

```toml
[design_rules]
target_resonator_length    = 2500.0   # where every resonator is cut and what it is made
resonator_length_tolerance = 100.0    # how far from it a resonator may be

[grid]
router_cell_size = 10.0        # how wide a router cell may be, in layout units

[stages.final]
corridor_spacings       = 11   # the band around a wire's own way, in wire spacings
inner_corridor_spacings = 11   # the same, for the inner circuit
rounds                  = 30   # sweeps over the wire list, in the outer and the feedline routing
inner_rounds            = 4
max_relaxation          = 10   # neighbours a failed wire may let go of, along the sweep
refinement_rounds       = 2    # rounds of the centring price over the ring after the outer routing; the benchmarks carry 0
feedline_refinement_rounds = 0 # the same after the feedline routing, under the feedline constraints: phase 5
meander_length          = 3000.0   # layout units; how long the outer routing makes every resonator, so that the cut point exists. The benchmarks carry the prototype's figure per chip: 2500 on 4q, 4000 on 21q, 6000 from 33q up
bend_penalty_norm       = 2.5      # × the sum of the grid's two extents
wire_proximity_penalty_norm   = 0.00125
static_proximity_penalty_norm = 0.00033
obstacle_penalty_reach  = 100.0    # layout units
coupler_length          = 200.0    # layout units; the coupling run, and the body along it
coupler_height          = 26.0     # layout units; the body across the run, the feedline along its far edge
repair_trials           = 100      # how many times the repair may turn a coupler; 0 switches it off
stop_after              = ""       # the last phase to run: inner, outer, couplers, feedlines, refined; empty runs them all
```

**Not knobs, and deliberately so.** The clearance between two wires is
`cells_for(min_wire_spacing, router)` — 19 cells, about 189.5 layout units
against a rule of 185. The straight run out of a source, and the straight
start of a resonator after its turn, is `cells_for(min_straight_length, router)`,
10 or 11 cells. The obstacle keepout is `min_obstacle_spacing`. The edge of
the routable space is the rectangle the launcher slots stand on. The crossing
rule reaches ten cells from a feedline's straight run, as the prototype's
does. All of them come from `[design_rules]`, from the chip or from the
prototype, and none of them can be set.

---

## 5. When something is wrong

0. **Run it again with `-v`, or `-v 1 -d`.** Every round says how many wires
   it tried, how many settled, how many are unrouted, open, short, long or
   crossing a feedline, and every stage ends on a `Fails:` line that counts
   them all. At level 1 every search says what it came to, which wires it
   let go of, what a resonator's lengthening came to, where its picture is,
   and — under the feedline constraints — what stood in its way
   (`in the way: the neighbours`, `the feedlines`, `the crossing rule`,
   `the coupler bodies`, `several together`, `the band itself`). Every
   coupler says where it sits, every edge whether it was drawn, every
   repair trial what it came to.
1. **`mqt-scpd drc <run>`** then. It says which two wires, where, and by how
   much.
2. **The picture of the phase.** `plot --stage final --phase outer` draws every
   wire with a band of the clearance the router keeps, so a pair that is too
   close is two bands whose overlap is darker; `--phase couplers` shows
   where every coupler sits and how the edges run, before any wire moved
   for them, `--phase feedlines` what the routing under the feedline
   constraints and the repair made of it.
3. **The GDS.** Layer 23 carries the same band; a boolean on it finds every
   overlap exactly, and layers 24 to 28 show which phase drew what.
4. **`inspect`** to read the artifact itself, when a number has to be checked
   rather than looked at.

A wire that was not drawn carries no cells, and its connection is named in
`FinalRouting.unresolved`; an edge that was not drawn is an empty entry in
`FinalRouting.feedlines`, and `feedline_edges` says what it should have run
between.

---

## 6. Everything, for every chip

One pass over all eight benchmarks: every stage, every picture, every check.

```bash
cd /Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4
for c in 4q 9q 17q 21q 33q 45q 57q 69q; do
  .venv/bin/mqt-scpd plan -c benchmarks/$c/config.toml -o artifacts/$c
  for s in corridor detail final; do
    .venv/bin/mqt-scpd plot   -c benchmarks/$c/config.toml --stage $s \
        --run-dir artifacts/$c -o artifacts/$c/$c-$s.svg
    .venv/bin/mqt-scpd render -c benchmarks/$c/config.toml --stage $s \
        --run-dir artifacts/$c -o artifacts/$c/$c-$s.gds
  done
  for p in inner outer couplers feedlines refined; do
    .venv/bin/mqt-scpd plot -c benchmarks/$c/config.toml --stage final --phase $p \
        --run-dir artifacts/$c -o artifacts/$c/$c-final-$p.svg
  done
  .venv/bin/mqt-scpd drc artifacts/$c
done
```

The whole run is minutes on the small chips and tens of minutes on the largest.
`plan` without `--stage` runs every stage from the beginning; with `--stage` it
resumes and runs only that one, reading the configuration **from the run
directory**.
