# The routing stages — where the code is and how to run them

A guide to the three routing stages the pipeline runs — **Corridor**, **Detail**
and **Final** — with what every piece of them is for, where it lives, and every
command needed to run them, draw them and check them.

```text
01-capacity.fb  02-global.fb  03-assign.fb  04-corridor.fb  05-detail.fb  06-final.fb
                                            └ Corridor      └ Detail      └ Final
```

Each stage draws the same wires at a finer grain than the one before it: the
Corridor stage says which partitions a wire runs through, the Detail stage
draws it cell by cell, and the Final stage draws it again as
curvature-constrained copper.

- Corridor: [summary-stage-4.md](summary-stage-4.md)
- Detail: [summary-detail-routing.md](summary-detail-routing.md)
- Final: [summary-final-routing.md](summary-final-routing.md), and
  [handover-final-couplers.md](handover-final-couplers.md) for the three phases
  that are still open

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

## 1. The Corridor stage

### 1.1 What it does

Routes every assigned connection through the partitions of the free space,
coarsely: from the point the assignment feeds it, across a border at a time, to
the cell its target port is reached at. It decides **which** corridors a wire
runs through and **where** it crosses from one into the next, and it is the
stage that makes the wire budgets binding — a border is crossed at one of a
fixed set of slots one wire spacing apart, and no two wires take the same one.

Only the connections of the assignment are routed here. The inner circuit paid
for the free space it crosses while it was solved, so it goes to the Detail
router directly.

### 1.2 The code

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

### 1.3 Running it

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

### 1.4 Its knobs

```toml
[stages.corridor]
rounds         = 12   # sweeps over the connection list
max_relaxation = 30   # neighbours a failed connection may rip up
```

Not a knob: where the slots are. `[stages.capacity] crossing_pitch` decides how
finely a border is divided, and it is a planning pitch and **not** a clearance
rule.

### 1.5 Its tests

[test/pipeline/test_corridor_router.cpp](test/pipeline/test_corridor_router.cpp)
— one wire per crossing place, no two wires cross inside a partition, no wire
runs over a place another is pinned to, no crossing outside the ring the ports
feed from, no wire turns straight back, and two runs give the same answer.

```bash
./build/release/test/pipeline/mqt-scpd-pipeline-test --gtest_filter='*Corridor*:Corridors.*'
```

---

## 2. The Detail stage

### 2.1 What it does

Draws every wire cell by cell on the detail grid — 750 × 750 cells on the
17-qubit chip — eight-connected, and holds the design rule between **every**
pair of wires. The corridor stage's route is a seed and not a constraint: the
last pass is free to move a wire off it entirely.

Four passes, in the prototype's order: route inside each partition, join the
pieces in corridor order, draw every wire again from end to end with rip-up and
relaxation, and finally seed the inner circuit over whatever free space is left.

### 2.2 The code

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

### 2.3 Running it

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

### 2.4 Its knobs

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

### 2.5 Its tests

[test/pipeline/test_detail_router.cpp](test/pipeline/test_detail_router.cpp) —
every wire eight-connected and free of obstacles, no wire meets itself, no two
share a cell or cross between cells, first and last cell are the feed and the
target, **no two wires within the design rule**, and two runs give the same
answer.

```bash
./build/release/test/pipeline/mqt-scpd-pipeline-test --gtest_filter='*Detail*'
```

---

## 3. The Final stage

### 3.1 What it does

The Detail stage leaves a **centre line on a cell grid**. The Final stage draws
each of those wires again as curvature-constrained copper over the Dubins
primitives of one bend radius, on a grid whose cell is about ten layout units —
1425 × 1425 cells on the 17-qubit chip, 3016 × 3016 on the 69-qubit one.

It runs five phases and leaves a snapshot after each:

| # | Phase | State |
| --- | --- | --- |
| 1 | `inner` — the inner circuit, inside its unit cells | built |
| 2 | `outer` — the ring, against the inner circuit and itself, every resonator lengthened to `meander_length` | built |
| 3 | `couplers` — the CPW couplers and the ports they create | open |
| 4 | `feedlines` — the launcher-to-launcher chains | open |
| 5 | `refined` — the refinement of those chains | open |

A phase that changes nothing leaves the state of the phase before it, so the
last three snapshots are the outer routing again until they are built.

Input: `05-detail.fb` plus the chip and the three planning artifacts.
Output: `06-final.fb`, and `drc.json` when the check is run.

---

### 3.2 The code, piece by piece

#### The stage

| Where | What it is for |
| --- | --- |
| [include/mqt-scpd/pipeline/Stages.hpp](include/mqt-scpd/pipeline/Stages.hpp) | `IFinalRouter`, the stage interface. One call does coupler placement **and** feedline routing, because the two are one fixpoint |
| [include/mqt-scpd/pipeline/FinalRouter.hpp](include/mqt-scpd/pipeline/FinalRouter.hpp) | `makeDubinsFinalRouter()`, the one implementation this build ships |
| [src/pipeline/FinalRouter.cpp](src/pipeline/FinalRouter.cpp) | the whole stage, about 2900 lines |
| [include/mqt-scpd/routing/MeanderInsertion.hpp](include/mqt-scpd/routing/MeanderInsertion.hpp), [src/routing/MeanderInsertion.cpp](src/routing/MeanderInsertion.cpp) | `insertMeander`, the prototype's meander insertion: one loop spliced into a straight run of a path, first fit or cheapest by a price |

Inside `FinalRouter.cpp`, in the order it reads:

| Symbol | Line | What it is for |
| --- | ---: | --- |
| `Tuning` | 85 | every knob, already converted onto the grid. Nothing here is a cell count standing for a distance |
| `tuningOf` | 135 | the one place a design rule or a configured length becomes cells |
| `Scene` | 201 | the router grid, the mask it searches on, the target cell and arrival heading of every port |
| `blockOutsideTheSources` | 224 | blocks everything between the chip outline and the rectangle the launcher slots stand on, so no wire slips **around** the sources |
| `sceneOf` | 254 | builds that scene: obstacle raster with the keepout baked in, one band per port, one target dug beyond each band |
| `Stencil`, `stencilOf` | 382 | the offsets of one clearance disc, and what enters it on each step. Charging a disc per wire cell costs thirty times as much |
| `Field` | 435 | **the heart of the correction.** Two fields per cell: which wire holds the copper, and how many wires keep their clearance over it. A wire charges its room when it is put down and gives it up when it is taken off |
| `Wire` | 564 | one connection: its two ends, the way it has, whether it is drawn, placed, routed, too short; for a resonator the run from its last cell to the port, `anchorGap` |
| `Pass` | 613 | what one run of the driver does: rounds, relaxation, band, stub |
| `Driver` | 803 | the one rip-up-and-reroute loop. The prototype writes it out four times |
| `Driver::needsLength`, `requiredLength`, `lengthOf`, `lengthInUnits` | 874 | whether a wire has to reach a length, how long a resonator's way has to be in cells (`meander_length` less `anchorGap`), and the rendered length of a way in cells and in layout units |
| `Driver::lengthen` | 920 | lengthens a resonator's way with `insertMeander`: the meander may enter what the search could enter — the band less the fence — first fit in phase 1, cheapest by the search's price field in the relaxation and the refinement. Returns whether the length was reached and the words for the line about it |
| `Driver::sweep` | 1022 | the rounds, over wires that start on the Detail stage's ways. Ends when no wire fails, or after four rounds without progress, on the pass's `Fails:` line |
| `Driver::failsOf` | 1154 | the fails of a set of wires, counted with every wire down: unrouted, open and short, each wire once |
| `Driver::sayResonatorLengths` | 1190 | one `-v` line per resonator at the end of the stage: the way, the run to the port, the two together against `meander_length`, and one line over all of them |
| `Driver::refine` | 1258 | routes every wire again against a centring price, to widen the room around it; a resonator is lengthened again and keeps its way when the wider one has no room |
| `Driver::fixPlaces` | 1350 | charges the places no wire may be moved off — its target **and the straight run out of its source**, which no search ever sees |
| `Driver::seed` | 1377 | puts every wire of a pass down on the way the Detail stage drew, joined on this grid with the straight stub in front |
| `Driver::attempt` | 1458 | one wire's turn, the prototype's two phases: the band fenced by both ring neighbours inflated by the clearance, then the relaxation along the sweep with the lane price and the discs around the wires let go of. A way found is taken; a resonator's way found is lengthened first, and one without room for its meander is no way. Nothing else |
| `Driver::conflictsOf` | 1606 | how many cells of a way lie within the rule of another wire, counted with the wire off the canvas — what the fails are counted by, not what the search keeps |
| `Driver::search` | 1701 | the call into the shared `DubinsRouter` |
| `Driver::buildCorridor` | 1724 | the cells a search may enter: the free space within the band around the way the wire has — the prototype's `expand_path`, which knows nothing of other wires |
| `Driver::fence` | 1792 | closes the ways of the fence wires, inflated by the clearance — the prototype's `mark_obstacles` with `min_dist_wires`; the meeting of two wires that share a junction stays open |
| `Driver::priceLane` / `fillLane` | 1908 | the prototype's corridor polygon: everything outside the lane between the two ring neighbours is priced, nothing is forbidden; the wires let go of are priced at growing distances |
| `Driver::priceApproaches` | 1951 | not in the prototype: the straight run out of the source and the run into the target of every wire around the search, in the port band's geometry but two clearances wide, at ten times the wire price, so that a relaxed search does not take a place a wire let go of has to come back to |
| `Driver::priceRoom` | 2101 | the centring price of the refinement: free in the middle of a channel, full price at its wall |
| `Driver::drawSearch` | 2406 | the picture of one search, with the meander and what the lengthening said for a resonator |
| `wiresOf` | 2680 | reads the plan into wires: feed point, target cell, headings, the Detail stage's way as a seed, the gap to the port for a resonator |
| `snapshotOf` | 2798 | what one phase drew, each wire with its length |
| `DubinsFinalRouter::run` | 2824 | the five phases and the artifact |

#### What the stage reuses

Written in phase 2 of the project; **no new search was written**.

| Where | What |
| --- | --- |
| [include/mqt-scpd/routing/DubinsRouter.hpp](include/mqt-scpd/routing/DubinsRouter.hpp) | the curvature-constrained A\*, its corridor mask and its two proximity prices |
| [include/mqt-scpd/routing/CouplerInsertion.hpp](include/mqt-scpd/routing/CouplerInsertion.hpp) | `spliceCouplerDogleg` — phase 3 will need it |
| [include/mqt-scpd/routing/PathGeometry.hpp](include/mqt-scpd/routing/PathGeometry.hpp) | `samplePath` and `renderedLength` — how long a way is, as the prototype's `sample_path` measures it |
| [include/mqt-scpd/routing/SelfIntersection.hpp](include/mqt-scpd/routing/SelfIntersection.hpp) | whether a path meets itself; the design-rule check calls the same function |
| [include/mqt-scpd/grid/GridMetrics.hpp](include/mqt-scpd/grid/GridMetrics.hpp) | `routerGrid`, and `cellsFor` — the one conversion from a rule to cells |
| [include/mqt-scpd/grid/Rasterize.hpp](include/mqt-scpd/grid/Rasterize.hpp) | the obstacle raster with the keepout baked in |
| [include/mqt-scpd/grid/PortBands.hpp](include/mqt-scpd/grid/PortBands.hpp) | the band around a port and the target dug beyond it |

#### The design-rule check

| Where | What |
| --- | --- |
| [include/mqt-scpd/drc/Rules.hpp](include/mqt-scpd/drc/Rules.hpp) | `CellView` — what a rule sees — and `checkCells` |
| [src/drc/Rules.cpp](src/drc/Rules.cpp) | rule 1 wire clearance, rule 3 wire loop, and the writer of `drc.json`. Rule 4, obstacle clearance, is out for now: its raster does not exempt the port approaches the router's mask exempts, so it reported every stub out of a port |
| [python/mqt/scpd/drc.py](python/mqt/scpd/drc.py) | reads `drc.json` back and counts it; only the active findings decide the exit code |

#### Schema, wiring and rendering

| Where | What |
| --- | --- |
| [schemas/artifacts.fbs](schemas/artifacts.fbs) | `FinalWire`, `FinalPhase`, `FinalRouting` |
| [schemas/config.fbs](schemas/config.fbs) | `FinalParams`, and `GridParams.router_cell_size` |
| [src/pipeline/Registry.cpp](src/pipeline/Registry.cpp) | `finalRouters()`, default `"dubins"` |
| [src/io/Artifacts.cpp](src/io/Artifacts.cpp) | the artifact is refused unless every wire steps by at most one cell |
| [bindings/bindings.cpp](bindings/bindings.cpp) | `route_final(...)` and `check_final(...)` |
| [python/mqt/scpd/run.py](python/mqt/scpd/run.py) | the stage in the run directory |
| [python/mqt/scpd/planning.py](python/mqt/scpd/planning.py) | `_final` — the artifact as the shapes a picture draws, per phase |
| [python/mqt/scpd/export/klayout.py](python/mqt/scpd/export/klayout.py) | one GDS layer per phase, 24 to 28, beside the clearance band on 23 |
| [python/mqt/scpd/cli.py](python/mqt/scpd/cli.py) | `--phase` for `plot` and `render`, and the `drc` command |

#### The tests

| Where | What |
| --- | --- |
| [test/pipeline/test_final_router.cpp](test/pipeline/test_final_router.cpp) | over all eight chips: every connection drawn, no wire meets itself or the artwork, every wire starts at its feed, **no two wires within the design rule**, **every resonator at least `meander_length` long**, two runs give the same answer |
| [test/routing/test_meander_insertion.cpp](test/routing/test_meander_insertion.cpp) | the meander on a straight run: the length is reached, the path stays well formed, the loop stays where it may and in its box, the priced insertion takes the cheaper side |
| [test/drc/test_rules.cpp](test/drc/test_rules.cpp) | what each rule finds and what it forgives, on wires built by hand |
| [test/python/unit/test_planning.py](test/python/unit/test_planning.py) | the five phases and the picture of one of them |
| [test/python/unit/test_drc.py](test/python/unit/test_drc.py) | reading a report back |

---

### 3.3 Running it

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

```text
[final]     0.01s  grid 1425x1425 cells of 9.97 layout units | clearance 19 cells for a rule of 18.55 | stub 11 cells | band 209 cells | bend 7125 | wire price 4 | obstacle price 1 over 11 cells | resonators 301 cells long
[final]     0.02s  inner routing: 14 of 14 wires start on the way the Detail stage drew
[final]     0.02s  inner routing: 14 wires, up to 4 rounds, 5 relaxations each way
[final]     0.05s  inner routing: 14 of 14 drawn, 0 unrouted, 0 open, 0 short | Fails: 0
[final]     0.06s  outer routing: 58 of 58 wires start on the way the Detail stage drew
[final]     0.06s  outer routing: 58 wires, up to 6 rounds, 5 relaxations each way
[final]     0.92s  outer routing round 0 forward : tried 58, routed 56, unrouted 2, open 0, short 0 | Fails: 2
[final]     0.98s  outer routing round 1 backward: tried 6, routed 5, unrouted 0, open 1, short 0 | Fails: 1
...
[final]     0.90s  outer routing round 4 forward : tried 1, routed 1, unrouted 0, open 0, short 0 | Fails: 0
[final]     0.90s  outer routing: 58 of 58 drawn, 0 unrouted, 0 open, 0 short | Fails: 0
[final]     0.94s  resonator 1 to Qb15.port0: 3395 units of way + 113 to the port = 3508 units, meets 3000
[final]     0.94s  resonator 5 to Qb14.port0: 5106 units of way + 113 to the port = 5219 units, meets 3000
...
[final]     0.94s  resonators: 17 in all, 3007 to 5640 units, 0 short of 3000
[final]     0.90s  final routing: 72 of 72 drawn, 0 unrouted, 0 open, 0 short | Fails: 0
```

Every pass starts with every wire on the way the Detail stage drew, put down
with its copper and its clearance, so that the sweep is a rip-up and
re-route over a complete routing and no wire is ever without a way. What
each line says:

| Word | What it counts |
| --- | --- |
| `tried` | wires this round offered a way |
| `routed` | of those, the ones that found a way |
| `unrouted` | wires with no way of their own: the stage has found none, and the wire still stands on the Detail stage's way. A seed is not curvature-constrained and knows nothing of this grid's keepout, so the artifact carries no cells for a wire left on it |
| `open` | wires with a way of their own that is not settled: in a round, let go of by a wire that relaxed past it and not drawn again yet, so the next round looks at them again; at the end of a pass, too close to another wire |
| `short` | resonators whose way is shorter than `meander_length` asks: drawn, but without room for the meander that would make them long enough. In a round, the ones that kept such a way in this round; at the end of a pass, measured by the sampler on every resonator |
| `resonator N to <port>` | one line per resonator at the end of the stage: the rendered length of its way, the run from the way's last cell to the port itself, the two together, and whether that meets `meander_length` or falls short of it. `resonators:` sums them up |
| `Fails:` | `unrouted` plus `open` plus `short`, each wire counted once. A round with none ends the pass. The pass's own last line and the stage's `final routing:` line count the same things with every wire down, by the test `mqt-scpd drc` makes and by the length the artifact carries, so what they say is what the check will find |
| `moved` | wires the refinement found a wider way for |

`-v 1` says one line more per search, indented, and with `-d` it names the
picture of that search:

```text
[final]     0.11s    wire 1 · relax 2: let go of 3, fence 3 and 0 · found 348 cells · meander: 267 → 340 cells for 289 · artifacts/17q/debug/final-00018-outer-r0f-w1-relax2.svg
[final]     0.27s    wire 10 · round 0 forward · normal: found 288 cells · long enough: 338 of 281 cells
[final]     0.33s    wire 15 · round 0 forward · normal: found 167 cells · no room for a meander: 182 of 290 cells, 191508 placements tried, relaxing
[final]     0.36s    wire 15 · relax 1: let go of 16, fence 16 and 14 · found 167 cells · no room for a meander: 182 of 290 cells, 191508 placements tried
[final]     0.42s    wire 15 · relax 2: let go of 17, fence 17 and 14 · found 305 cells · meander: 182 → 292 cells for 290
...
[final]     3.99s    wire 42 · round 5 backward: no way after 5 relaxations; 41, 40, 39, 38, 37 back as they were; keeps a way too short for its meander
[final]     4.34s  outer routing: 57 of 58 drawn, 1 unrouted, 4 open, 1 short | Fails: 6
[final]     4.34s  outer routing: unrouted: 42 · open: 30, 31, 36, 37 · short: 45
[final]     4.34s  outer routing round 0 forward : 48 found a way in phase 1, 5 after relaxation, 5 failed: 30, 36, 42, 45, 46, 1 kept a way too short for its meander
[final]     4.34s  outer routing round 1 backward: 17 found a way in phase 1, 3 after relaxation, 3 failed: 30, 36, 42
```

So a pass ends on which wires are unrouted, which open and which short, and
on what every round came to: how many wires found a way in phase 1, how many
only after letting neighbours go, and which found none in that round. A
resonator's search says what its lengthening came to: `meander: before →
after cells for required`, `long enough`, or `no room for a meander` with how
many placements were tried.

`-v` works on the Corridor and the Detail stage too, and each ends on a
`Fails:` line of its own:

```text
[corridor]     0.00s  30 connections over 91 partitions, up to 12 rounds, 30 relaxations
[corridor]     0.00s  round 0 forward : routed 30 of 30
[corridor]     0.00s  30 of 30 connections have a way through the partitions, 0 unrouted | Fails: 0
[detail]     0.00s  grid 360x360 cells | 30 connections of the ring | 6 of the inner circuit
[detail]     0.00s  pass 1, inside the partitions: 0 pieces left undrawn
[detail]     0.00s  the rule is 4 cells of this grid, which is 158 layout units against a rule of 185
[detail]     0.02s  round 0 forward : 1 of 36 wires still to settle
[detail]     0.02s  36 of 36 wires drawn, 0 unrouted, 0 within the rule of another | Fails: 0
```

> **Trap.** `--stage` reads the configuration **from the run directory**, not
> from the file named on the command line. After editing `benchmarks/$c/config.toml`,
> copy it over first: `cp benchmarks/$c/config.toml artifacts/$c/config.toml`.

#### Debug pictures of the Final stage

```bash
.venv/bin/mqt-scpd plan -c benchmarks/$c/config.toml -o artifacts/$c -d
```

`-d` makes the Final stage hand out one SVG per thing it looks at, written
into `artifacts/$c/debug/`:

| File | What it shows |
| --- | --- |
| `final-grid.svg` | the whole router grid before anything is drawn: the artwork with its keepout (dark), the obstacle halo the search pays for (yellow, darker is dearer), every port as the chip carries it (teal square on the port's own cell, line = orientation, `p<n>` = its index, the label on hover), every wire's seed from the Detail stage (thin grey), and every source (blue) and target (red, the cell beyond the port's band) with the heading it leaves or arrives on as a line and the wire's number beside it |
| `final-00017-outer-r0f-w1-relax1.svg` | one search, taken right after it so it shows exactly what the router was given: picture 17 of the run, the `outer` pass, round 0 `f`orward (`b` backward), wire 1, phase 2 at relaxation level 1 (`normal` is phase 1, `refine` the refinement). Cropped to the band's box. For a resonator the way found carries its meander, and the title says what the lengthening came to |

In a search picture: every port in the crop as the chip carries it (teal
square on its cell, dot at its exact position, index beside it, label on
hover) and, dashed, the distance from that exact position to the cell the
wire is routed to beyond the port's band, in layout units and cells; the band
the search may enter (green), the wire price
outside the lane and around the wires let go of (red, darker is dearer), the
fence — the clearance around the fence wires, closed — as an orange band
around their copper, the ring neighbours (blue), the wires let go of (purple,
dashed), the way the wire had (grey, dashed), the way it found (green, bold)
or `no way` in the title, and its two ends. The title line names the fence,
the wires let go of, the lane's two neighbours and the box; the second one
the setting. Every fill is translucent, so what lies under it stays visible,
and every layer is one CSS class, so one can be switched off in the file.

The wire is the ring index, `i3` the third wire of the inner circuit. Count on
one picture per search: 13 on the 4-qubit chip, 37 on the 9-qubit, 166 on
the 17-qubit (19 MB); a chip that relaxes a lot makes many hundred.

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

In KLayout the layers are:

| Layer | Name |
| ---: | --- |
| 1 | the chip artwork |
| 2 | the ports, datatype = the role |
| 23 | `plan.clearance` — a band one clearance wide around every wire. Two wires closer than the rule are two of these that overlap, which a boolean finds exactly |
| 24 | `final.inner-routing` |
| 25 | `final.outer-routing` |
| 26 | `final.coupler-insertion` |
| 27 | `final.feedline-routing` |
| 28 | `final.feedline-refinement` |

#### Checking the design rules

```bash
.venv/bin/mqt-scpd drc artifacts/17q
```

Writes `artifacts/17q/drc.json`, prints how many findings it holds, and exits
nonzero when an active rule found something. Each finding names the two wires,
the place in layout units, the distance measured and the limit.

#### All eight chips

```bash
for c in 4q 9q 17q 21q 33q 45q 57q 69q; do
  cp benchmarks/$c/config.toml artifacts/$c/config.toml
  .venv/bin/mqt-scpd plan   -c benchmarks/$c/config.toml -o artifacts/$c --stage final
  for p in inner outer couplers feedlines refined; do
    .venv/bin/mqt-scpd plot -c benchmarks/$c/config.toml --stage final --phase $p \
        --run-dir artifacts/$c -o artifacts/$c/$c-final-$p.svg
  done
  .venv/bin/mqt-scpd render -c benchmarks/$c/config.toml --stage final \
      --run-dir artifacts/$c -o artifacts/$c/$c-final.gds
  .venv/bin/mqt-scpd drc artifacts/$c
done
```

#### The tests

```bash
cmake --build --preset release && ctest --preset release
uv run --no-sync pytest test/python/unit

# just this stage, one chip
./build/release/test/pipeline/mqt-scpd-pipeline-test --gtest_filter='*Final*17q*'
./build/release/test/drc/mqt-scpd-drc-test
```

#### Reading an artifact by hand

```bash
.venv/bin/mqt-scpd inspect artifacts/17q/06-final.fb | head -40
```

---

### 3.4 Its knobs

Everything is a length in layout units or a price. A shipped configuration
carries only what differs from the defaults; every benchmark carries a
`[stages.final]` section with `rounds = 6`, `max_relaxation = 5` and
`refinement_rounds = 0`, the setting the seeded sweep was measured at.

```toml
[grid]
router_cell_size = 10.0        # how wide a router cell may be, in layout units

[stages.final]
corridor_spacings       = 11   # the band around a wire's own way, in wire spacings
inner_corridor_spacings = 11   # the same, for the inner circuit
rounds                  = 30   # sweeps over the wire list
inner_rounds            = 4
max_relaxation          = 10   # neighbours a failed wire may let go of, along the sweep
refinement_rounds       = 2
meander_length          = 3000.0   # layout units; every resonator's way is made this long. The benchmarks carry the prototype's figure per chip: 2500 on 4q, 4000 on 21q, 6000 from 33q up
bend_penalty_norm       = 2.5      # × the sum of the grid's two extents
wire_proximity_penalty_norm   = 0.00125
static_proximity_penalty_norm = 0.00033
obstacle_penalty_reach  = 100.0    # layout units
coupler_length          = 200.0    # layout units; phase 3 will use it
coupler_height          = 26.0
```

**Not knobs, and deliberately so.** The clearance between two wires is
`cells_for(min_wire_spacing, router)` — 19 cells, about 189.5 layout units
against a rule of 185. The straight run out of a source is
`cells_for(min_straight_length, router)`, 10 or 11 cells. The obstacle keepout
is `min_obstacle_spacing`. The edge of the routable space is the rectangle the
launcher slots stand on. All four come from `[design_rules]` or from the chip,
and none of them can be set.

---

## 4. When something is wrong

0. **Run it again with `-v`, or `-v 1 -d`.** Every round says how many wires
   it tried, how many settled, how many are unrouted, open and short, and
   every stage ends on a `Fails:` line that counts all three. At level 1
   every search says what it came to, which wires it let go of, what a
   resonator's lengthening came to, and where its picture is.
1. **`mqt-scpd drc <run>`** then. It says which two wires, where, and by how
   much.
2. **The picture of the phase.** `plot --stage final --phase outer` draws every
   wire with a band of the clearance the router keeps, so a pair that is too
   close is two bands whose overlap is darker.
3. **The GDS.** Layer 23 carries the same band; a boolean on it finds every
   overlap exactly, and layers 24 to 28 show which phase drew what.
4. **`inspect`** to read the artifact itself, when a number has to be checked
   rather than looked at.

A wire that was not drawn carries no cells, and its connection is named in
`FinalRouting.unresolved`.

---

## 5. Everything, for every chip

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
