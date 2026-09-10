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
| 2 | `outer` — the ring, against the inner circuit and itself | built |
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
| [src/pipeline/FinalRouter.cpp](src/pipeline/FinalRouter.cpp) | the whole stage, about 1800 lines |

Inside `FinalRouter.cpp`, in the order it reads:

| Symbol | Line | What it is for |
| --- | ---: | --- |
| `Tuning` | 79 | every knob, already converted onto the grid. Nothing here is a cell count standing for a distance |
| `tuningOf` | 114 | the one place a design rule or a configured length becomes cells |
| `Scene` | 165 | the router grid, the mask it searches on, the target cell and arrival heading of every port |
| `blockOutsideTheSources` | 188 | blocks everything between the chip outline and the rectangle the launcher slots stand on, so no wire slips **around** the sources |
| `sceneOf` | 218 | builds that scene: obstacle raster with the keepout baked in, one band per port, one target dug beyond each band |
| `Stencil`, `stencilOf` | 318 | the offsets of one clearance disc, and what enters it on each step. Charging a disc per wire cell costs thirty times as much |
| `Field` | 373 | **the heart of the correction.** Two fields per cell: which wire holds the copper, and how many wires keep their clearance over it. A wire charges its room when it is put down and gives it up when it is taken off |
| `Field::guardedOnlyBy` | 398 | whether every wire guarding a cell ends on one of two components — what makes the junction exemption narrow |
| `Wire` | 541 | one connection: its two ends, the way it has, whether it is drawn, placed, routed |
| `Pass` | 589 | what one run of the driver does: rounds, relaxation, band, stub |
| `Driver` | 608 | the one rip-up-and-reroute loop. The prototype writes it out four times |
| `Driver::sweep` | 697 | the rounds. Ends when every wire is routed, or after four rounds without progress, then a rescue pass |
| `Driver::refine` | 766 | routes every wire again against a centring price, to widen the room around it |
| `Driver::fixPlaces` | 802 | charges the places no wire may be moved off — its target **and the straight run out of its source**, which no search ever sees |
| `Driver::markJunctions` | 827 | where two wires meet and the rule therefore does not bind |
| `Driver::attempt` | 901 | one wire's turn: band, relaxation both ways, targeted rip, verdict |
| `Driver::conflictsOf` | 1029 | the verdict: how many cells of a way lie in another wire's room, counted with the wire off the canvas |
| `Driver::blockersOf` | 1056 | the wires actually in the way of a way |
| `Driver::search` | 1082 | the call into the shared `DubinsRouter` |
| `Driver::buildCorridor` | 1099 | the cells a search may enter: free space within the band, less every cell another wire holds or guards |
| `Driver::priceLane` / `fillLane` | 1196 | the prototype's corridor polygon: everything outside the lane between the two ring neighbours is priced, nothing is forbidden |
| `Driver::priceGuarded` | 1342 | a price on every guarded cell, for the search that has to ignore the rule |
| `Driver::priceRoom` | 1369 | the centring price of the refinement: free in the middle of a channel, full price at its wall |
| `wiresOf` | 1591 | reads the plan into wires: feed point, target cell, headings, components, the Detail stage's way as a seed |
| `snapshotOf` | 1697 | what one phase drew |
| `DubinsFinalRouter::run` | 1722 | the five phases and the artifact |

#### What the stage reuses

Written in phase 2 of the project; **no new search was written**.

| Where | What |
| --- | --- |
| [include/mqt-scpd/routing/DubinsRouter.hpp](include/mqt-scpd/routing/DubinsRouter.hpp) | the curvature-constrained A\*, its corridor mask and its two proximity prices |
| [include/mqt-scpd/routing/CouplerInsertion.hpp](include/mqt-scpd/routing/CouplerInsertion.hpp) | `spliceCouplerDogleg` — phase 3 will need it |
| [include/mqt-scpd/routing/SelfIntersection.hpp](include/mqt-scpd/routing/SelfIntersection.hpp) | whether a path meets itself; the design-rule check calls the same function |
| [include/mqt-scpd/grid/GridMetrics.hpp](include/mqt-scpd/grid/GridMetrics.hpp) | `routerGrid`, and `cellsFor` — the one conversion from a rule to cells |
| [include/mqt-scpd/grid/Rasterize.hpp](include/mqt-scpd/grid/Rasterize.hpp) | the obstacle raster with the keepout baked in |
| [include/mqt-scpd/grid/PortBands.hpp](include/mqt-scpd/grid/PortBands.hpp) | the band around a port and the target dug beyond it |

#### The design-rule check

| Where | What |
| --- | --- |
| [include/mqt-scpd/drc/Rules.hpp](include/mqt-scpd/drc/Rules.hpp) | `CellView` — what a rule sees — and `checkCells` |
| [src/drc/Rules.cpp](src/drc/Rules.cpp) | rule 1 wire clearance, rule 3 wire loop, rule 4 obstacle clearance, and the writer of `drc.json` |
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
| [test/pipeline/test_final_router.cpp](test/pipeline/test_final_router.cpp) | over all eight chips: every connection drawn, no wire meets itself or the artwork, every wire starts at its feed, **no two wires within the design rule**, two runs give the same answer |
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
[final]     0.01s  grid 1425x1425 cells of 9.97 layout units | clearance 19 cells | stub 11 cells | band 209 cells | bend 7125 | wire price 4 | obstacle price 1 over 11 cells
[final]     0.02s  inner routing: 14 wires, up to 4 rounds, 10 relaxations each way
[final]     0.06s  inner routing round 0 forward : tried 14, routed 14, still open 0, undrawn 0
[final]     0.06s  inner routing: 14 of 14 drawn, 0 of them too close to another wire
[final]     0.09s  outer routing: 58 wires, up to 30 rounds, 10 relaxations each way
[final]     2.02s  outer routing round 0 forward : tried 58, routed 57, still open 14, undrawn 1
[final]     3.06s  outer routing round 1 backward: tried 24, routed 23, still open 12, undrawn 1
...
[final]     6.72s  outer routing: 4 rounds without progress, stopping
[final]     6.72s  outer routing: wire 42 has no way; taking one without the rule
[final]     7.40s  outer routing: 58 of 58 drawn, 0 of them too close to another wire
[final]     7.40s  refinement: 58 wires, 2 rounds
[final]     8.44s  refinement round 0 forward : moved 54 of 58
```

What each line says:

| Word | What it counts |
| --- | --- |
| `tried` | wires this round offered a way |
| `routed` | of those, the ones that came out settled — a way that holds the rule against **every** other wire |
| `still open` | wires not settled at the end of the round, so the next round looks at them again. A wire that another wire had to let go of is open too, because the way it has was drawn when that other one was not there |
| `undrawn` | wires with no way at all. These are the ones the rescue takes without the rule |
| `too close` | wires whose committed way comes within `min_wire_spacing` of another, counted with every wire down and by the same test `mqt-scpd drc` uses |
| `moved` | wires the refinement found a wider way for |

`-v` works on the Corridor and the Detail stage too:

```text
[corridor]     0.00s  30 connections over 91 partitions, up to 12 rounds, 30 relaxations
[corridor]     0.00s  round 0 forward : routed 30 of 30
[detail]     0.00s  grid 360x360 cells | 30 connections of the ring | 6 of the inner circuit
[detail]     0.00s  pass 1, inside the partitions: 0 pieces left undrawn
[detail]     0.00s  the rule is 4 cells of this grid, which is 158 layout units against a rule of 185
[detail]     0.02s  round 0 forward : 1 of 36 wires still to settle
[detail]     0.02s  36 of 36 wires drawn, 0 of them within the rule of another
```

> **Trap.** `--stage` reads the configuration **from the run directory**, not
> from the file named on the command line. After editing `benchmarks/$c/config.toml`,
> copy it over first: `cp benchmarks/$c/config.toml artifacts/$c/config.toml`.

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
carries only what differs from the defaults, so none of the benchmarks has a
`[stages.final]` section at all.

```toml
[grid]
router_cell_size = 10.0        # how wide a router cell may be, in layout units

[stages.final]
corridor_spacings       = 11   # the band around a wire's own way, in wire spacings
inner_corridor_spacings = 11   # the same, for the inner circuit
rounds                  = 30   # sweeps over the wire list
inner_rounds            = 4
max_relaxation          = 10   # neighbours a failed wire may let go of, each way
refinement_rounds       = 2
meander_length          = 3000.0   # layout units; not built yet
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

0. **Run it again with `-v`.** Every round says how many wires it tried, how
   many settled, how many are still open and how many have no way at all.
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
