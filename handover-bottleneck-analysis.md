# The bottleneck analysis after the coupler insertion

Written for whoever takes the bottleneck analysis further. Its next use, as the
user set it, is to find the feedline edges that will make the feedline routing
fail **before** that routing runs.

The analysis looks at the chip the coupler insertion leaves behind. It finds
every narrow place between the obstacles, cuts the free space into partitions at
those places, and checks with an integer flow whether every outer wire can get
from the partition of its source terminal to the partition of its target
terminal within the capacities. It is **report only**: it changes nothing the
stage does, and with `SCPD_BOTTLENECKS` off (the default) it does not run.

On 17q it now says **UNSAT, short by 19 wires on 19 edges**. Most of the 19 are
gaps between a wire's own port run and a feedline, and the open question in
*What is open* asks whether those gaps are too strict for their own wire.

Read [handover-cpw-coupler-insertion.md](handover-cpw-coupler-insertion.md) for
the couplers and [handover-feedline-routing.md](handover-feedline-routing.md)
for the feedline pass whose failures this analysis is meant to predict.

- Checkout: `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`, branch
  `phase-4-routing-stages`.
- HEAD is `712cf7a` (2026-10-07 20:27, the user's commit). It holds the first
  version of the analysis: the walls, the bottleneck search, the chambers and
  the capacity flow. Its message ("Add exact eighth turns …") belongs to another
  task and does not describe the content.
- **Uncommitted** on top of it; the user commits per phase:
  - the JSON data and the HTML page;
  - the port runs with their slots;
  - the slot rules in the bottleneck search;
  - the removal of the feedline case in the report.

  The files are `src/pipeline/FinalRouter.cpp`, `src/grid/Bottlenecks.cpp`,
  `include/mqt-scpd/grid/Bottlenecks.hpp`, `python/mqt/scpd/cli.py`, the new
  `python/mqt/scpd/capacityview.py` and `capacityview.html`, the tests and
  `CHANGELOG.md`.

## How to run it

The page and the data need `-d`. The 17q config says `stop_after = "couplers"`,
so a run ends right after the analysis.

```zsh
cd /Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4

# after a C++ change: build the binding and copy it into the overlay
cmake --build build/cp311-abi3-macosx_15_0_arm64/Release --target mqt-scpd-bindings
OVERLAY=py-cg zsh artifacts/logs/dev-sync.sh

SCPD_DEV_OVERLAY=artifacts/dev/py-cg \
SCPD_BOTTLENECKS=1 SCPD_LAUNCHER_FENCE_TURN=1 SCPD_SEARCH_PICTURES=0 \
.venv/bin/python artifacts/logs/dev-mqt-scpd.py plan \
  -c benchmarks/17q/config.toml -o artifacts/dev/17q-cg --stage final -v 1 -d
```

- The overlay is needed because the binding in `.venv` is installed, not
  rebuilt. Without the overlay, reinstall first and then run
  `.venv/bin/mqt-scpd plan …` with the same switches:

  ```zsh
  uv sync --inexact --no-dev --no-build-isolation-package mqt-scpd \
    --reinstall-package mqt-scpd
  ```

- `--stage final` resumes on the artifacts already in `artifacts/dev/17q-cg`.
- `SCPD_LAUNCHER_FENCE_TURN=1` and `SCPD_SEARCH_PICTURES=0` are the user's
  settings for this work. The first changes the insertion; the second keeps a
  debug run from drawing a picture per search.
- To rebuild only the page from existing data (after a change to the template):
  `uv run --no-sync python -m mqt.scpd.capacityview artifacts/dev/17q-cg/debug/final-capacity-graph.json`.

Outputs in `artifacts/dev/17q-cg/debug/`:

| File                        | What it is                                                       |
| --------------------------- | ---------------------------------------------------------------- |
| `final-bottlenecks.svg`     | the walls, the medial axis, every cut by how many wires it holds |
| `final-capacity-graph.svg`  | the chambers, the graph, the load of each edge                   |
| `final-capacity-graph.json` | the graph as data (`writeCapacityGraph`)                         |
| `final-capacity-graph.html` | the page built from the JSON (`mqt.scpd.capacityview`)           |

The log lines start with `coupler insertion: BOTTLENECKS —`,
`coupler insertion: CAPACITY GRAPH —` and `coupler insertion: CAPACITY GRAPH
SAT|UNSAT|UNKNOWN —`. With `-v 1`, every cut adds a `[Bottleneck]` line and
every wire a `[Capacity]` line with its way.

## The steps

All of it is in `src/pipeline/FinalRouter.cpp` unless a file is named. The entry
is `reportBottlenecks(wires)`, called at the end of the coupler insertion after
`reportSqueeze`.

### 1. The walls — `wallsAfterInsertion`

One mask of obstacles, each cell labelled with the wall it belongs to
(`Walls::of`, `Walls::list`). A later wall takes only free cells.

| Kind        | What                                                                                                                          |
| ----------- | ----------------------------------------------------------------------------------------------------------------------------- |
| border      | everything outside the rectangle the launcher cells span, and the outermost ring of cells                                     |
| artwork     | the qubits and couplers inside that rectangle, as **one** wall; `endName` names a cell of it by the nearest non-launcher port |
| coupler pad | the body of every chosen coupler option                                                                                       |
| stub        | the port runs, two walls each (below)                                                                                         |
| feedline    | every drawn feedline edge, its line plus a disc of half the clearance (9.5 cells on 17q)                                      |

**Port runs** (user, 2026-10-07). A port run is the copper a wire has no choice
about at one end:

- a plain wire's source: the rule's straight length (`min_straight_length`, 100
  units) from the point the assignment feeds it, on its heading (`sourceRunOf`);
- a resonator's source: its lead as it stands — the coupling run and the quarter
  turn off the coupler, to where its search starts (`wire.arc` and
  `objective.source`) — with nothing added;
- every target: the rule's straight length from the target port itself
  (`targetRunOf`).

The router's target cell already lies a band of `min_straight_length` past the
port, and outer wires get no end stub. Before 2026-10-07 evening the analysis
added a second straight length from that cell; the run was about 200 units for
normal ports and 310 for bridge-pair ports. It is now measured from the port.
`Tuning::straightLength` is the rule in cells, not rounded, so a diagonal run is
also 100 units.

Each run keeps a band of half the clearance to either side of its middle line,
with square ends: a cell whose nearest point on the line is an end, seen from
beyond, is not in it. Since 2026-10-08 (user) the run is only the two sides of
that band: two walls one cell thick, named `"<wire> at its source, left"` /
`", right"` (or `"lead of <wire>, …"`, `"… at its target, …"`), one either side
of the line. A wall cell is a cell of the band with a neighbour, diagonal ones
included, outside the band to the side or beyond the end at the port. So each
wall is 4-connected and lies where the band's edge lies, and a gap between two
runs keeps the length it had when the run was a solid band. The end at the port
is closed, each wall taking its half of it; the end at the terminal is open. A
run of no length keeps no room.

Why the end at the port is closed: the port's own copper lies there, and on 17q
a row of free cells lies between every launcher run and the border. With that
end open, the slot joined the room behind the run through that row: 73 of the 81
cuts from a launcher run to a feedline had a way round them, 85 cuts in all, and
the terminal lay in the big chamber outside instead of the one in front of it.

**The slot** (user, 2026-10-07). The free cells between the two walls are the
slot the terminal lies in; they go into `Walls::slots`, less any cell a later
obstacle takes. The terminal cell (`Walls::terminals`, per wire: source,
target) is the cell on the line one cell short of the end, inside the slot.

Why the slot: the medial axis runs into the space between the two walls, and
the narrowing in front of a terminal parts into one cut from the end of each
wall. The two cuts close the room in front of the run's open end, with the slot,
as the terminal's own partition. Without the slot rules, the clearance from a
wall end to the fork in front of the open end often rises less than the minimum
rise, and neither cut is kept (`ASlotBetweenTwoThinWallsIsCutAtBothWallEnds`).
No cut crosses a slot cell, so no cut joins the two walls of one run.

### 2. The medial axis

`grid::squaredDistanceTransform`, `grid::medialAxis` (a Voronoi diagram over the
wall boundary cells; edges between sites of one wall are dropped) and
`grid::rasterizeMedialAxis`. About 0.12 s on 17q.

### 3. The cuts — `grid::findBottlenecks` (`src/grid/Bottlenecks.cpp`)

The options the analysis passes:

| Option                    | Value                                            | Meaning                                                                 |
| ------------------------- | ------------------------------------------------ | ----------------------------------------------------------------------- |
| `maximumSquaredClearance` | ⌊95²⌋ + 1                                        | only gaps up to `SCPD_BOTTLENECK_WIRES` (10) wires: 10 × 19 = 190 cells |
| `sameNarrowing`           | `roomPitch()` × cell (20 cells, about 200 units) | cuts closer than this are one narrowing                                 |
| `minimumRise`             | `SCPD_BOTTLENECK_RISE` (2 cells)                 | persistence: the clearance must rise this much on both sides            |
| `slots`                   | `Walls::slots`                                   | the slot rules below                                                    |
| `wallOf`                  | `Walls::of`                                      | the wall label of every cell                                            |

The search:

1. **Candidates.** A cell of the axis with two neighbours is a candidate when
   its clearance is a local minimum along the axis or an edge of a plateau of
   one. It must also be below the limit.
2. **Persistence** (`risesTowards`). Walking the axis away from a candidate on
   both sides, the clearance must rise by `minimumRise` before it falls below
   the candidate's. A walk that reaches an ordinary fork or an end fails.
   **A fork with an arm into a slot counts as the rise.** Every cell of a
   stretch at one clearance passes and is taken to the middle of the stretch
   (`middleOfStretch`, the lower index of the two middle cells of an even
   stretch), so a stretch gives one cut.
   - There used to be a tie rule: a cell of the same clearance and a lower index
     counted as below. It killed one half of the narrowing in front of a slot,
     the half whose walk to the slot fork ran to lower indices, so a launcher
     with a feedline in front kept one cut on one side only. It was removed on
     2026-10-08.
3. **The cut** (`cutAt`). From the candidate, each of the two shores (the free
   cells around it on either side of the axis) is walked downhill in the
   distance transform to a wall. The two wall cells are the ends of the cut. No
   cut may cross a target or a slot cell.
4. **Arm cuts at slot forks** (2026-10-08). Every fork with an arm into a slot:
   each of its other arms, up to the next fork or end, is cut at its narrowest
   cell, unless a candidate of step 2 lies on it. If the fork itself is the
   narrowest, the first valid cut next to the fork is taken. This closes the two
   cases where step 2 finds nothing:
   - the side is narrowest at the slot fork, so the clearance only rises along
     the arm;
   - the side lies between the slot fork and another fork, such as the axis
     around the stub's corner, so the walk of step 2 fails at the far fork.
5. **One narrowing per place** (`narrowestOfEveryPlateau`). Two cuts are one
   narrowing and the narrowest is kept when they share a wall cell and their
   other ends are closer than `sameNarrowing` **and** on the same wall. They are
   also one narrowing when both lie between the same two walls and both pairs of
   ends are that close. The wall check keeps the two cuts beside a slot apart,
   since they end on different flanks.

The grid library knows nothing of terminals. `slots` and `wallOf` are empty for
the capacity stage (`WatershedPlanner`), which also runs without a minimum rise,
so the capacity stage is unchanged by all of this.

### 4. Measuring a cut

- Its length `L` in cells, between the two wall cells.
- It **holds ⌊L / clearance⌋ wires** (`pipeline::wiresThroughGap`,
  `CapacityFlow.cpp`), whatever the walls are. The walls are already inflated by
  half the clearance, so this is the rule for every kind.
  - There used to be a case for feedlines: half a clearance deducted per
    feedline end, then inflation instead, then a report that listed and drew
    feedline cuts apart. The user removed the last of it on 2026-10-07: every
    cut is logged, drawn and labelled the same way.
- The wires whose current way crosses its line (`crossing`, from the field's
  owner of each cell). These ways are the Detail stage's or the insertion's, and
  the feedline pass moves them, so this count says where to look and not what
  will fail.

### 5. The partitions — `grid::chambersOf` (`src/grid/Chambers.cpp`)

A cut is a line that blocks the steps crossing it (`bottleneckMoves`). It takes
no free space of its own. A chamber — the user calls it a **partition**; the
code and the page say chamber or partition for the same thing — is a group of
free cells a walk reaches without crossing a cut, with no diagonal step between
two wall cells. `Chambers::beside` gives the chambers on either side of each
cut. A cut with one chamber on both sides separates nothing and becomes no edge
(`aside` in the JSON).

### 6. The capacity graph — `reportCapacityGraph`

- **Nodes**: the chambers.
- **Bottleneck edges**: every cut between two or more chambers, capacity
  `holds`, open to every wire. A cut between three chambers is one edge.
- **Crossing stretches** (`crossingStretches`): where a plain wire may cross a
  feedline edge, by the rule the feedline pass holds it to. A cell of a drawn
  middle edge is a place to cross when the edge runs straight there
  (`SCPD_ORTHO_CROSSING`, on) and the line across it at a right angle,
  `CROSSING_REACH + 1` (11) cells to each side, runs into no wall but the edge
  itself and is one `CrossingConstraints::allowed` lets a wire run. Consecutive
  places with the same two chambers are one stretch. A stretch of `length` cells
  takes ⌊length / pitch⌋ + 1 wires (pitch 20) and is open only to the plain
  wires that bridge that edge (`bridgers_`, user 2026-10-07: a plain wire
  crosses only its prescribed edge). Terminal edges get no stretch, and no
  resonator crosses a feedline.
- **Demands**: one per outer wire, from the chamber its source terminal lies in
  to the chamber of its target terminal (`portsOf`, `chambersOfPort`). A
  terminal in the slot lies in the slot's chamber. `besideOwnStubs`, which used
  to grow a port's chambers across the cuts that end on the wire's own stub, is
  gone: the slot partition replaces it. A terminal that lies in no chamber makes
  the verdict UNSAT.
- **The check** — `pipeline::checkCapacity`
  (`include/mqt-scpd/pipeline/CapacityFlow.hpp`, HiGHS):
  - an integer multi-commodity flow; every edge is a node of its own, joined
    both ways to its chambers, and the units entering it are its load;
  - the load may exceed the capacity by an integer overflow;
  - the objective is the least total overflow first, then the fewest arcs (one
    overflow unit costs more than all arcs together);
  - demands with no way at all are found by a walk that ignores the capacities
    and left out of the model;
  - the time limit is `SCPD_CAPACITY_SECONDS`, 30.
- **Verdict**:
  - SAT: every wire has a way within the capacities.
  - UNSAT: a wire has no way, a terminal lies in no chamber, or the least
    overflow is proven above zero.
  - UNKNOWN: the time limit stopped the solver with overflow left.

  The check is necessary, not sufficient: the flow knows nothing of the order of
  wires inside a chamber, of turns or of lengths.

### 7. The data and the page

`writeCapacityGraph` writes the JSON (debug runs only). Cells are grid cells, y
up. Its fields:

- `raster`: run-length pairs over the grid, row by row; 0 is a cell on a cut
  line, 1–5 a wall by `wallKinds`, 6 + c a cell of chamber c.
- `strokes`: the middle lines of port runs (on the left flank's name) and
  feedline edges.
- `slots`: the slot cells, every free cell between the two walls of a port run.
- `nodes`: one per chamber, with its cell count.
- `edges`: chambers, capacity, load, overflow, length, `through`, `users`; a
  bottleneck also has `ends` and `crossingNow`, a stretch has `feedline`.
- `aside`: the cuts with one chamber on both sides.
- `wires`: per outer wire, `source` and `target` with label, port position,
  heading, terminal `cell` and chambers; `way` as edge indices; `status`; `path`
  as the corners of the current way.
- `verdict`, `summary`, `straightLength`, `clearance`, `pitch`.

`plan -d` builds the page from it when the stage wrote the JSON in this run
(`cli._capacity_view`). It never builds from data an earlier run left behind.
`python/mqt/scpd/capacityview.py` fills the template `capacityview.html`, which
is prettier-formatted and has no outside resources.

What the page shows:

- **Views**: Chip, Graph or Both.
  - The chip view draws the chambers and walls on a canvas, with the edges,
    `load/capacity` badges, nodes, slots, target ports labelled with their
    partition ("38 · c33") and a dashed line from each target port to its
    partition's node.
  - The graph view lays the chambers out by a force model seeded with their chip
    positions, with the ports as leaves; chambers with neither an edge nor a
    port are left out.
- **Tables**: edges (by severity, then spare), feedline edges, wires,
  partitions.
- **Selection**: a click on an edge, node, port or row selects it in both views.
  The address can name a selection: `#wire=14`, `#edge=g20`, `#chamber=c12`,
  `#feedline=f2`, with `&theme=light` and `&view=graph`.

## Switches, variables and fixed values

Every switch is an environment variable read by `envFlag`, `envWhole` or
`envReal` in `src/pipeline/FinalRouter.cpp`. A flag is on for `1`, off for `0`.
Most are read once per process (`static`); `SCPD_BOTTLENECKS` is read at every
insertion.

### The analysis

| Variable                    | Default                       | Range        | What it does                                                                                                                                    |
| --------------------------- | ----------------------------- | ------------ | ----------------------------------------------------------------------------------------------------------------------------------------------- |
| `SCPD_BOTTLENECKS`          | off                           | flag         | runs the whole analysis at the end of the coupler insertion (`bottleneckReport`)                                                                |
| `SCPD_BOTTLENECK_WIRES`     | 10                            | 1–100        | the widest gap searched, in wires: the limit is `wires × clearance` cells, passed as `maximumSquaredClearance` = ⌊(wires × clearance / 2)²⌋ + 1 |
| `SCPD_BOTTLENECK_RISE`      | 2.0                           | 0–1000 cells | `minimumRise`: how far the clearance must rise on both sides of a candidate; 0 keeps every local minimum and every cell of a plateau            |
| `SCPD_CAPACITY_SECONDS`     | 30                            | 1–3600 s     | time limit of `checkCapacity`; a stop with overflow left gives UNKNOWN                                                                          |
| `SCPD_BOTTLENECK_ALL`       | off                           | flag         | on: every bottleneck cuts the chambers; off: only those with an end on a feedline edge, a coupler pad or a resonator's lead (`couplerCuts`)     |
| `SCPD_CAPACITY_FLOW`        | off                           | flag         | on: the integer flow `checkCapacity`; off: every wire alone, `checkEachAlone`                                                                   |
| `SCPD_CAPACITY_CHAIN`       | off                           | flag         | on: the check after every settled chain, and the search again of a chain that closes a wire's way (`checkChainCapacity`)                        |
| `SCPD_CAPACITY_CHAIN_TRIES` | 3                             | 0–20         | how often one chain is searched again                                                                                                           |
| `SCPD_CAPACITY_BENCH`       | 0                             | 0–1000       | measurement only: repeats the analysis this often, leaves every feedline edge out once, times windows (`benchCapacityCheck`)                    |
| `SCPD_CAPACITY_DUMP`        | none                          | directory    | measurement only: `SCPD_CAPACITY_BENCH` writes every flow problem there as JSON                                                                 |
| `SCPD_ROOM_PITCH`           | 0 = clearance + 1 (20 on 17q) | cells        | `roomPitch()`: the pitch of two crossings on a stretch (capacity ⌊length / pitch⌋ + 1), and `sameNarrowing` = pitch × cell side                 |
| `SCPD_ORTHO_CROSSING`       | on                            | flag         | a stretch only where the feedline edge runs straight, under the crossing rule (`CrossingConstraints`)                                           |
| `SCPD_FEEDLINE_PROTOTYPE`   | on                            | flag         | terminal edges are left out of the crossing rule; no edge of a stretch is terminal either way                                                   |
| `SCPD_SEARCH_PICTURES`      | on                            | flag         | off: a `-d` run draws only the whole-chip pictures, not one per search; set to 0 for this work                                                  |

### Switches that shape the chip the analysis sees

These change the coupler insertion, so they change the walls. Only the first is
set for this work; the others are at their defaults.

| Variable                     | Default                          | What it changes                                                                                                                                                                                                            |
| ---------------------------- | -------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `SCPD_LAUNCHER_FENCE_TURN`   | off (**set to 1 for this work**) | the fence in front of a launcher reaches as far as a quarter turn needs (`launcherTurnRun`); the code says it means something mainly beside `SCPD_COUPLER_BOX_TURN`, and alone it removes wire 31's dead-on-arrival on 17q |
| `SCPD_COUPLER_BOX_TURN`      | off                              | the same reach for the box a coupler must keep clear of a launcher                                                                                                                                                         |
| `SCPD_EDGE_IN_BOX`           | on                               | everything outside the box is closed to a feedline edge's search; the first and last edge of a chain are exempt                                                                                                            |
| `SCPD_SQUEEZE_REPORT`        | on                               | the squeeze report that runs just before this analysis; it marks edges `Squeezed` in the artifact, report only                                                                                                             |
| `SCPD_SQUEEZE_REJECT`        | on                               | the chain search looks only for options whose edges all leave the room the squeeze report asks for                                                                                                                         |
| `SCPD_SQUEEZE_STEP`          | 5                                | every how many cells of an edge the room beside it is measured                                                                                                                                                             |
| `SCPD_SQUEEZE_REACH`         | 300                              | how far from an edge an obstacle still counts as the wall of its channel, cells                                                                                                                                            |
| `SCPD_SQUEEZE_TOLERANCE`     | 0                                | cells of shortfall the squeeze rule lets through                                                                                                                                                                           |
| `SCPD_SQUEEZE_RECOVER`       | on                               | an edge the commit could not draw under the squeeze rule is drawn once more with the rule suspended                                                                                                                        |
| `SCPD_FEEDLINE_LEAD_TRIM`    | on                               | the resonator's lead is cut back by the lead margin and those cells go back to the search, so `wire.arc`, the resonator's port run, ends earlier                                                                           |
| `SCPD_COUPLER_LEAD_MARGIN`   | 5                                | that margin, cells                                                                                                                                                                                                         |
| `SCPD_RESONATOR_STUB`        | off                              | off: a resonator gets no second straight run after its lead; `=1` restores it. The analysis takes the lead alone either way                                                                                                |
| `SCPD_COUPLER_STUBS`         | on                               | the straight runs at a coupler are measured along the heading they run on, not along an axis                                                                                                                               |
| `SCPD_FENCE_FIXED`           | on                               | the fixed places of the wires are fenced too, not only their ways; `=0` fences the ways alone                                                                                                                              |
| `SCPD_BRIDGE_CHECK`          | on                               | a found way that does not cross the wire's bridged edge is refused                                                                                                                                                         |
| `SCPD_CROSSING_EXIT_HEADING` | off                              | the crossing test also checks the exit heading                                                                                                                                                                             |

### The dev scripts

| Variable           | Where                            | What                                                                                                        |
| ------------------ | -------------------------------- | ----------------------------------------------------------------------------------------------------------- |
| `OVERLAY`          | `artifacts/logs/dev-sync.sh`     | the overlay to copy the built binding into, `artifacts/dev/$OVERLAY` (default `py`; this work uses `py-cg`) |
| `SCPD_DEV_OVERLAY` | `artifacts/logs/dev-mqt-scpd.py` | the overlay put first on `sys.path` (default `artifacts/dev/py`)                                            |

The dev scripts and `artifacts/dev/` are not part of the package; the overlay
keeps a dev run off the binding installed in `.venv`.

### The command line

| Flag             | What                                                                                                   |
| ---------------- | ------------------------------------------------------------------------------------------------------ |
| `-d` / `--debug` | writes the pictures, the JSON and the page into `<run>/debug`; without it there is no data and no page |
| `-v 1`           | one line per search and per cut/wire (`[Bottleneck]`, `[Capacity]`); the dashboards need it            |
| `--stage final`  | runs the Final stage alone on the artifacts already in the run directory                               |

The config of the chip (`benchmarks/17q/config.toml`) says
`stop_after = "couplers"`, so the stage stops after the insertion and this
analysis.

### Fixed values in the code

| Name                     | Value                                                          | Where                      | What                                                                                                                           |
| ------------------------ | -------------------------------------------------------------- | -------------------------- | ------------------------------------------------------------------------------------------------------------------------------ |
| `Tuning::clearance`      | ⌈`min_wire_spacing` / cell⌉, 19 cells on 17q                   | `tuningOf`                 | the wire clearance in cells                                                                                                    |
| `wallInflation()`        | clearance / 2 (9.5)                                            | `FinalRouter.cpp`          | how far a port run's two walls lie from its middle line, and the radius of a feedline's disc                                   |
| `Tuning::straightLength` | `min_straight_length` / cell, not rounded (about 10.03 on 17q) | `tuningOf`                 | the length of a straight port run                                                                                              |
| `CROSSING_REACH`         | 10                                                             | `FinalRouter.cpp`          | the crossing rule's reach; a stretch needs `CROSSING_REACH + 1` free cells to each side                                        |
| `SAME_WALL_STEPS`        | 4                                                              | `src/grid/Voronoi.cpp`     | two boundary sites closer than this along the boundary are one wall to the medial axis                                         |
| arm reach at a fork      | ⌈√clearance(fork)⌉ + 4                                         | `src/grid/Bottlenecks.cpp` | how far an arm is followed to see whether it runs into a slot                                                                  |

## Tests

- `test/grid/test_bottlenecks.cpp`: the search, including five slot tests, each
  run in all four orientations of the stub:
  - `ATerminalSlotPartsTheNarrowingInFrontOfIt`
  - `ASideNarrowestAtTheSlotIsCutBesideIt` — fails without the arm cuts;
  - `AStubOnABorderIsCutOnBothSidesOfItsSlot`
  - `TheTwoCutsBesideASlotCloseItsTerminalsChamber`
  - `ASlotBetweenTwoThinWallsIsCutAtBothWallEnds` — a port run as it is drawn
    now, two thin walls; fails without the slot.

  The old tie rule fails the first of them. No test reproduces the case of a
  side between the slot fork and a corner fork; it was seen only on 17q
  (launcher 27).
- `test/grid/test_chambers.cpp`: the chambers.
- `test/pipeline/test_capacity_flow.cpp`: the flow and `wiresThroughGap`.
- `test/pipeline/test_final_router.cpp`:
  - `DrawsTheBottlenecksAfterTheCouplerInsertion`
  - `DrawsTheCapacityGraphAfterTheBottlenecks` (4q) — it also checks the JSON:
    the raster covers the grid, every edge's load is the number of ways through
    it, every port run is the rule's straight length, every terminal lies in a
    slot, every slot cell is free, a port run's walls are one cell thick and no
    cut joins the two walls of one run.
- `test/python/unit/test_capacityview.py`: the page and the CLI hook.

Last state:

- grid tests 73 of 73;
- the pipeline subset of `FinalRouter.Draws*`, `*Capacity*` and `*Watershed*`,
  13 of 13;
- Python tests 22 of 22;
- `ctest -E EveryChip`: 370 of 371. Only
  `FinalRouter.OnlyUnsettledRedrawsOneWire` fails, which failed before this work
  too.

The `EveryChip` benchmark tests were not run to the end since the slot changes.

## Changed on 2026-10-08 (user)

Only the cuts with an end on a feedline edge, a coupler pad or a resonator's
lead cut the chambers now, and every wire is checked alone instead of by the
flow. The check after every settled chain and the page that moves couplers by
hand were built on top. All of it is in
[handover-capacity-check.md](handover-capacity-check.md).

The table below is the state before these changes.

## Where it stands on 17q

Each row is a change the user asked for, run with the switches above.

| State                                                        | Cuts | Chambers | Verdict | Over       |
| ------------------------------------------------------------ | ---- | -------- | ------- | ---------- |
| domed port runs, target runs doubled (2026-10-07 afternoon)  | 75   | 126      | SAT     | 0 (6 full) |
| port runs as 100-unit rectangles from the port               | 64   | 160      | UNSAT   | 3          |
| slots; resonator lead as it stands; `besideOwnStubs` removed | 95   | 190      | UNSAT   | 8          |
| tie rule removed                                             | 108  | 199      | UNSAT   | 8          |
| arm cuts at slot forks, same-walls merge                     | 214  | 321      | UNSAT   | 19         |

Logs: `artifacts/logs/cg/17q-*.log`; the last row is `17q-arms.log`.

Every launcher with a feedline near it now has one cut on each side of its
terminal. The exceptions are 8 and 37, which have nothing within the 10-wire
limit on one side.

The 19 edges with overflow in the last run:

- **Artwork pockets in front of target ports**:
  - g33: 22 (`Coupler8_12.port3`) and 23 (`Qb8.port0`), both terminals in one
    pocket whose exit holds 1;
  - g72: 51 and 52, the same pattern;
  - g73: 47, a 5-cell gap between `Qb6.port1` and `Qb6.port0` that holds 0.

  Before 2026-10-07 evening the doubled target runs put these terminals outside
  the pockets.
- **A wire's own source run against a feedline, holding 0**: 2 (f17), 9 (f0), 12
  (f2), 14 (f2), 27 (f8), 38 (f11), 41 (f13), 56 (f17); and 31/32 at f9, holding
  1 for two wires.
- **Two target runs side by side, holding 0**: 13 (beside 12), 33 (beside 32),
  37 (beside 36), 8 (beside 7), 42 (beside 41), 4 (beside 3).
- **A resonator lead against a feedline, holding 0**: 11 (f1).

## What is open

1. **The own stub** (asked of the user on 2026-10-08, no answer yet). A wire
   needs no clearance to its own port run. A cut between a wire's own flank and
   a feedline is therefore half a clearance wider, for that wire, than ⌊L /
   clearance⌋ assumes. Most of the 19 overflows are such cuts.
   - The old `besideOwnStubs` handled this by joining the chambers across those
     cuts, which is what the slot partition now forbids.
   - The proposal: such a cut holds ⌊(L + clearance/2) / clearance⌋ for its own
     wire and ⌊L / clearance⌋ for the others. `checkCapacity` then needs a
     capacity per user, or a width per demand, in place of one integer per edge.
2. **Bridge-pair ports.** The router's band out of a bridge-pair port
   (`Coupler*.port1`–`port4`) is 2 × `min_straight_length` (`bridging` in the
   scene); the analysis counts 100 for every port, as the user asked. Whether
   the routing's 200 is wanted is the user's to say.
3. **Many small chambers.** 321 chambers on 17q. Many have neither an edge nor a
   port, mostly one-cell pockets between the new walls. They do not change the
   flow, and the graph view hides them. The arm cuts at slot forks reach every
   arm up to the next fork, up to the 10-wire limit, so open areas around
   targets get cuts too.
4. **The prediction itself is not measured yet.** The next step the user set —
   which feedline edges will fail — needs the verdict compared with
   `bad = unrouted + open + crossing` of the same chip routed to
   `stop_after = "feedlines"`. Only `crossing` and `[Capacity] over` lines give
   the wire names to compare.
5. **Other chips.** Everything above was measured on 17q only, by the user's
   instruction.

## Pitfalls

- `uvx nox -s lint` runs `prek run --all-files`. It rewrites about 130 clean
  files: clang-format reflows all of `FinalRouter.cpp`, rumdl reflows
  `CHANGELOG.md`, ruff turns every `noqa` into `ruff: ignore`, and typos touches
  old docs. Revert those. Instead run
  `SKIP=clang-format .nox/lint/bin/prek run --files <own files>`, and
  clang-format with `--lines` on the changed hunks only.
- In zsh a variable holding several `--lines=` flags stays one word.
  `clang-format -i "$R" file` then formats the **whole** file. Build an array
  from the `git diff -U0` hunks.
- Running the hook's ruff by hand applies its fixes (`fix = true`), also to
  neighbouring files.
- The remaining known lint findings are not from this work: ruff on
  `from . import dashboard` in `cli.py`, ty on `artifacts/logs/*.py`, and 48
  rumdl findings in `CHANGELOG.md`, the same as in HEAD.
- Edit the HTML template in its unformatted form if you edit it a lot, then let
  prettier format it through prek. Check the page in a browser afterwards;
  headless Chrome with `--screenshot` and a `#…` selection in the address works.
