# Phase 3 — handover

Written 2026-09-09 for whoever picks this up next. It says what is here, what
was verified and how, what is deliberately different from the prototype, and
what is still open. Everything is **uncommitted**: the user commits per phase.

- Checkout: `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-3`
- Branch: `phase-3-solver-and-planning-stages`, based on `a126097`
- Prototype for comparison: `/Users/michaelfeldmeier/Documents/GitHub/FridgeCAD`
  (`FridgeCAD-0fails` is the same through the planning steps and differs only
  in the FinalGrid extras)

## What phase 3 delivers

`MQT::ScpdMilp` (solver-neutral model, linked-in HiGHS, MPS emission, a
bring-your-own-key gurobipy backend over an MPS round trip, `SCPD_SOLVER`),
`MQT::ScpdPipeline` (stage interfaces, name registry, and the **Capacity**,
**Global** and **Assignment** stages), the capacity layer of `MQT::ScpdGrid`,
the resumable run directory behind `mqt-scpd plan`, and rendering of the
planning stages through the existing `plot` (SVG) and `render` (GDSII/OASIS)
commands.

The pipeline order is **Capacity → Global → Assignment**
([decision 0026](docs/design/decisions/0026-global-runs-before-the-assignment.md)).
Artifacts are `01-capacity.fb`, `02-global.fb`, `03-assign.fb`.

## What the latest session did

The Assignment stage. It had not been run since the ring moved, and reading it
against the prototype showed it wrong in ways its one test could not see. Three
rules now hold on **all eight** chips and each is checked in ctest:

1. **Every ring node is reached.** `readBack` emitted a connection only for an
   anchor whose launcher binary was set, so the artifact named
   `launcher_target` ports of the ring — 2 of 12 on 4Q, 24 of 230 on 69Q — and
   said nothing about the rest. It now writes one connection per ring node: a
   resonator as `ResonatorSource` to `ResonatorTarget` with no source port, a
   conventional port as `FeedlineSource` to `FeedlineTarget` fed from its own
   launcher.
2. **A launcher feeds conventional ports, at most one each.** Only a chain
   *end* was moved off its launcher, so 45 of 70 resonators still stood on one
   on 69Q, and a launcher slot carried up to five ring nodes on 57Q. The
   prototype's `"zero"` pass (`OrderedAssignmentGraph.cpp:792-843`) is now
   ported in full: every resonator
   a launcher was given moves onto the segment to the **next launcher along**,
   at `1/(n+1) … n/(n+1)` of the way, in ring order. Every ring node of every
   chip now has a feed point of its own — 230 distinct points on 69Q.
3. **The ring keeps its cyclic order.** Measured, not assumed: it failed on 9Q,
   17Q, 45Q, 57Q and 69Q, and the read-back fixes alone did not repair it.

**The third rule needed a model change, and it is the interesting one.** The
ordering potential ran over the *ring's* length, so the walk could turn the
launcher ring twice, and two nodes a whole launcher ring apart then shared a
launcher: 17 launchers carried two conventional ports each on 69Q and 15 on 57Q.
The potential's range is now **one turn of the launcher ring**, one short of the
launcher count. That is not a cap on the ring — it is rule 2 itself: a chip
whose ring carries more conventional ports than it has launchers has no
assignment, and the model now says so by being infeasible instead of quietly
doubling up.

The walk is forced to fall once per conventional port and once per resonator
that takes a launcher, so the range has to hold that many steps. All eight chips
hold, three of them with nothing to spare:

| chip | 4q | 9q | 17q | 21q | 33q | 45q | 57q | 69q |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| forced steps | 9 | 23 | 47 | 58 | 90 | 119 | 150 | 184 |
| range | 15 | 23 | 47 | 71 | 95 | 119 | 151 | 191 |

This reverses the first note under "Two Assignment-stage changes" further down,
and the same table says why that note was right when it was written: 17Q's ring
carried 59 ports then and needed 49 steps of 47. The ring has since lost a port
and now needs 47 of 47.

**One more defect, found by reading and confirmed by measuring.** The anchor
counter in the flow constraint started at the first resonator the walk *meets*,
which is the second anchor whenever the ring opens on a conventional port. Every
launcher constraint was then shifted by one anchor. Four of the eight chips open
on a conventional port: 9Q, 17Q, 21Q and 33Q. The anchor is now looked up
through `RingEdges::positionOf` rather than counted.

The prototype's endpoint refinement (`OrderedAssignmentGraph.cpp:957-996`, tags
`"first"` and `"second"`) is **not** ported. It moves a chain end off a
launcher, which the `"zero"` pass now does for every resonator, and a second
interpolation would put that end back on a point another node already uses. See
the amendment to
[decision 0029](docs/design/decisions/0029-a-feedline-end-is-fed-between-launchers.md).

`AssignmentInputs::edgeWeight` is gone. It was filled with ones and read
nowhere; the chord weights are worked out in `ringEdgesOf`.

### The tests

`TEST_P(BenchmarkAssignment, FollowsTheRulesOfTheRing)` runs capacity, global
and assign once per chip and then checks the three rules, so every chip is its
own ctest entry with its own failure message. Five of the eight had no fixture.
A `readScalar` beside the existing `readArray` now reads `capacity_cells_x`,
`capacity_cells_y`, both launcher offsets, `launcher_target`,
`max_feedline_utilization` and `feedline_terminations` out of the shipped
`config.toml`, so no fixture copies a per-chip figure by hand. The role patterns
stay fixture arguments: the 4-qubit chip names its parts `Q1` and `C12` where
the other seven use `Qb1` and `Coupler1_2`, and that difference is what the
patterns are configuration for.

Reading the scalars also corrected the 17-qubit fixture, which held
`capacity_cells_y` at zero where the chip ships 25. That chip's grid is square
either way, so no figure moved.

`AssignsTheFourQubitChip` keeps what is chip-specific about 4Q — its ring is the
configured sequence, because it has no inner circuit to extend it — and calls
the same rule-1 check as the parameterized test.

### What the figures did

| chip | assign | objective | crossings | connections | feed points |
| --- | --- | --- | --- | --- | --- |
| 4q | 0.1s | 5.07 → **5.02** | 5 | 2 → **12** | 10 → **12** |
| 9q | 0.1s | 15.24 → **15.27** | 15 | 3 → **30** | 25 → **30** |
| 17q | 0.3s | 27.67 → **27.84** | 27 | 7 → **58** | 46 → **58** |
| 21q | 0.4s | 34.06 → **34.14** | 34 | 10 → **70** | 64 → **70** |
| 33q | 2.8s | 58.61 → **58.72** | 58 | 14 → **110** | 98 → **110** |
| 45q | 2.1s | 86.56 → **86.71** | 86 | 15 → **150** | 120 → **150** |
| 57q | 3.9s | 111.66 → **115.40** | 111 → **115** | 18 → **190** | 135 → **190** |
| 69q | 31.8s | 137.48 → **141.85** | 137 → **141** | 24 → **230** | 169 → **230** |

The integer part is the crossing count and the hundredths are the proximity
tie-break. Six chips keep their crossing count and pay a hundredth or two of
tie-break for the tighter potential; 57Q and 69Q pay four crossings each. That
is the price of one launcher per conventional port, and it is what the user
asked for. 4Q moves the other way and now reads **5.02 exactly**, which is the
prototype's own figure; the 0.05 gap recorded under the deviations is closed.

The assign stage costs 0.1 to 32 seconds per chip, 41 seconds over all eight
against 96 seconds for capacity. ctest goes from 11 seconds to about 130.

## What the session before it did

Three corrections to the Global stage. Capacity was left alone and is
**byte-identical** on all eight chips.

1. **The bridge pairing is declared, not measured.** A new port role
   `bridge_pair` names the ports a wire crosses a component at, and one
   `[[ports.bridge_pairs]]` rule per crossing pairs them by a capture both
   sides share. The geometric rule — opposite orientation, shortest distance —
   is gone from `design::componentsOf`. The two declarations are checked
   against each other at load, so they cannot drift apart. See
   [decision 0030](docs/design/decisions/0030-bridge-pairs-are-declared.md).
2. **An internal bridge is shut by default.** A crossing whose far end the
   outer ring does not name lets a wire enter a component the assignment never
   sees; both of its ports now carry no flow at all.
   `[stages.global] internal_bridges = true` grants it back.
3. **A target draws one wire over every lattice at once.** The demand was on
   the lattice node, so a target beside a chamber border — a node of two
   lattices — was asked for a wire in each. The second wire took supply the
   chip has, and the target it was taken from was then reported unreached. On
   17Q four targets sit in two lattices; the stage built 14 wires for 11 served
   targets and reported 3 unreached, where 13 wires now serve 13. Same
   correction the bridge coupling already had, in the one place it had not been
   made.
4. **The SVG carries the wire budget of every gate**, in the capacity and the
   global picture alike. The global picture reads the run's own
   `01-capacity.fb` for the gate layer.
5. **A gate is hidden only when the chamber cannot see it at all.** The test
   asked that *every* cell beside a gate see it unobstructed, which is true of
   no gate that has a neighbour: a gate is a line of cells and the sight line
   from one of its own ends to its middle runs almost along it. A chamber that
   several corridors meet therefore lost all of its gates, the chain ended at
   its own target, and the pruning took the gate leading in as well — the free
   space beyond a branching corridor fell out of the plan. The prototype has
   the same defect and its own comment contradicts its code
   (`CapacityGrid.cpp:4857`, "Ein einziger valider Beobachtungspunkt auf der
   Frontier reicht", then `break` on the first blocked one). This is what left
   `Coupler12_13.port0` unserved on 17Q.
6. **One narrowing is one gate.** The saddle search reports a candidate
   wherever the clearance stops falling, so a narrow place whose clearance is
   flat over a few cells came back as several lines that share a wall and end
   a cell or two apart. A chamber that two of them led out of was credited
   with twice the room the chip has. `findBottlenecks` now merges candidates
   that share a wall and end within one **wire pitch** of each other, keeping
   the narrowest; the pitch is the length in which the count of wires can
   change at all. Gates fall from 24 to 15 on 4Q and from 216 to 185 on 69Q,
   and no chamber has two gates across one narrowing on any of the eight.
7. **The plan reports what the ports' own approaches keep clear**, in
   `CapacityPlan.port_keepout`, and both renderers draw it on a layer of its
   own. The port bands and the launcher sweeps are obstacles the chip input
   does not carry, so a picture drawn from the chip alone showed corridors
   wider than they are. The rings are traced by the same `extractPartitions`
   the partitioning uses, so a band along a diagonal comes out as the
   staircase the grid actually blocks. It decides nothing — a gate's budget
   still reads `scene.reserved` — and it costs the 69-qubit capacity stage
   about ten seconds for the second trace.

The pairing change is behaviour-preserving: with `internal_bridges = true`
every one of the eight chips reproduces the table further down exactly. What
the switch costs:

| chip | connections | unserved targets | ring | objective |
| --- | --- | --- | --- | --- |
| 4q | 0 | 0 | 12 | 0.00 |
| 9q | 6 | 0 | 30 | 9896.87 |
| 17q | 17 → **13** | 1 → **1** | 59 → **57** | 27931.03 → **24066.39** |
| 21q | 9 → **8** | 0 | 70 | 12470.96 → **12890.07** |
| 33q | 10 → **8** | 0 | 110 | 12157.51 → **12506.07** |
| 45q | 7 | 0 | 150 | 11891.47 |
| 57q | 11 | 0 | 190 | 13667.56 |
| 69q | 13 → **11** | 0 | 231 → **230** | 17647.40 → **13667.56** |

The two objectives that **rise** are the honest reading — the same targets,
reached by longer wires now that the shortcut through an inner coupler is gone.
The two that fall do so because fewer wires are built at all.

**Every inner target of every chip is served.** The gate visibility fix is what
closed the last one: it gave `Coupler16_17.port3` a chain that leads somewhere,
so the bridge that surfaces there is no longer shut for want of a launcher, and
`Coupler12_13.port0` has supply.

The two gate fixes pull in opposite directions and both change the capacity
plan on every chip. Keeping the gates of a branching chamber adds gates; taking
one line per narrowing removes them. The pruned count ends at 15 of 352 on 4Q
against 12 before, and 185 of 2452 on 69Q against 78.

The Assignment stage was not run in that session and its artifacts were not
regenerated; `03-assign.fb` is absent from every run directory.

## What the session before that did

The stages already ran on all eight benchmarks, but nothing had ever been
checked against the prototype's own output — only against reference objective
values, which is too coarse to say where a divergence begins. The plan was to
align the capacity stage on 9Q in five ordered steps: initial bottlenecks,
pruning, chain structure, Hanan grids, inner assignment.

### Result on 9Q, against `FridgeCAD/src/simple_9q_layout.cpp`

| | prototype | ours |
| --- | --- | --- |
| capacity chains / targets | 38 / 78 | 38 / 78, **all 38 target sets identical** |
| Hanan grids | 4 (49/49/64/64 nodes, 66/61/81/80 edges) | 4, **identical node sets and identical port sets per grid** |
| inner connections | 6 | 6: **4 identical**, 1 exact tie, 1 strictly cheaper on our side |
| outer ring / resonators | 30 / 10 | 30 / 10, **resonators identical** |

### The four defects that were in the way

Each produced a silently wrong answer rather than a failure.

1. **A bridge port needs twice the forward approach.** The prototype selects
   those ports with `label[0]=='C' && label.back() in {1,2,3,4}`. The rule in
   the model is that a port paired across its component carries the wire *on*
   and so has to clear the component's own artwork. Without it the target lands
   on the artwork edge in a one-cell pocket: 40 of 78 targets were walled in,
   and the chains fragmented into 70 instead of 38. The pairing now lives once
   in `design::componentsOf` (`include/mqt-scpd/design/Bridges.hpp`), and
   `pipeline::innerCircuitOf` reads the same topology instead of repeating it.
2. **The bridge flow coupling has to span lattices.** A bridge's two ports sit
   in different chambers and therefore in different Hanan grids; requiring both
   in one grid skipped exactly the bridges the constraint was written for, and
   a wire could start at a bridge port out of nothing.
3. **Only *internal* bridges are coupled that way.** An external bridge's far
   end is on the ring and on no lattice, so coupling it forces the near end's
   outflow to zero and no wire can leave the inner region at all.
4. **The ring keeps a gated port when the circuit surfaced there** — which is
   the *inner* end of the bridge being a source, not the ring port itself. The
   resonator list **grows** by the outer end; it does not substitute.

### The model part that was missing entirely

A lattice edge says a wire *may* run somewhere; it does not say the corridor
has room. The capacity chains were computed, written and drawn, but the model
that decides where a wire goes never read them. They are now a second, integer
flow beside the binary flow on the lattices
([decision 0028](docs/design/decisions/0028-the-inner-circuit-pays-for-free-space.md)):
launchers supply, gates pass at most their capacity, targets draw one wire,
and where a target is the outer end of a bridge it draws exactly what the inner
circuit sends out there. **A demand exists only where a supply can reach it** —
27 of 17Q's 73 chains have no launcher or are cut off by a shut gate, and
insisting their targets draw a wire made the whole Global stage infeasible.

### Two Assignment-stage changes

- **The ordering potential runs over the ring's length, not the launcher
  count.** It falls once per conventional port, so the old bound capped how
  many ports a ring could carry: 17Q needs 49 steps out of 47 slots and came
  out infeasible once the ring was corrected. The read-back already took the
  slot modulo the launcher count, so the potential is simply given the ring's
  own length. **Reversed by the latest session**: 17Q's ring lost a port and
  needs 47 steps of 47, and the ring's own length let the walk turn the
  launcher ring twice.
- **A resonator that ends a feedline is fed between two launchers**
  ([decision 0029](docs/design/decisions/0029-a-feedline-end-is-fed-between-launchers.md)),
  at the midpoint between its own launcher and the one its neighbour on the
  open side was given — what `CapacityGrid::add_coupler_launcher_ports` does.
  The Assignment artifact carries this per ring node in `feeds` as a **point**,
  not a port reference: the slot is not a port of the chip. Both renderers draw
  the chord to it.

## Deliberate deviations from the prototype

These are measured and written down, not reproduced. Do not "fix" them without
reading the record first.

- **The prototype's inner-circuit objective is half-priced.** It sums
  `weight * var[u][v]` over an edge list emitted with `u < v` only, so *every
  arc running the other way is free*. That is why it feeds `Coupler5_8.port0`
  from `Coupler6_9.port4` (798 under its objective, 2438 by true rectilinear
  length) where we use `Coupler8_9.port4` (1333 and 1368). We price both
  directions. The other differing connection is an exact tie: `Qb5.port1` costs
  2844.9 from either `Coupler7_8.port2` or `Coupler4_7.port4`.
- **The grid frame is a node convention, the prototype's is a raster one**;
  they differ by half a cell. On 9Q this moves 28 of 102 port cells by one cell
  and 19 of 78 targets by one or two. Written up in
  `include/mqt-scpd/grid/GridMetrics.hpp`, which also records that the *other*
  three-cell difference once attributed to the frame was the missing bridge
  band and is fixed.
- **`bottleneck_clearance` is a length**, not the prototype's
  `saddle_edt_limit = 150` squared cells, which admitted 235 layout units on 4Q
  and 484 on 9Q ([decision 0019](docs/design/decisions/0019-design-rules-in-layout-units.md)).
  We get 102 initial bottlenecks against 131, and 27 pruned against 22. That is
  why the chain *trees* differ in how many gates lie between a target and its
  launcher, even though all 38 target sets match. The tree comparison itself
  predates the gate visibility fix and has not been re-run.
- **The port band reaches back three times the forward reach for every port.**
  The prototype used six for a qubit's `port1`, selected by a label test; that
  band covers artwork the raster already blocks
  ([decision 0018](docs/design/decisions/0018-port-roles-unassigned-and-assigned.md)).
- **The Hanan mask skips polygon 0 only.** The prototype's
  `is_point_in_polygon` also skips polygon 1 and every polygon with fewer than
  ten vertices, which silently exempts real artwork.
- **The assignment objective's integer part is the crossing count**; the
  hundredths are the proximity tie-break at `PROXIMITY_WEIGHT = 0.01`.
  Comparing the totals without splitting them apart is what made earlier
  readings look worse than they were. 4Q now reads 5.02, the prototype's own
  figure. Giving the potential a launcher ring of slack costs 45Q minutes
  instead of seconds; it was tried and dropped.
- **A bridge whose far end the ring does not name is shut**, where the
  prototype crosses it. It has no switch for this; we do, and it is off
  ([decision 0030](docs/design/decisions/0030-bridge-pairs-are-declared.md)).
  Turning it on reproduces the prototype's behaviour on all eight chips.

## Verification

All green as of this writing.

```bash
cmake --build build/dev && ctest --test-dir build/dev     # 262 tests, 130s
uvx nox -s tests                                          # 120 tests x 4 Python versions
uvx nox -s lint                                           # see the trap below
```

`uvx nox -s lint` **rewrites 71 files this change never touched**, and it did so
before this change as well. The hook revisions are pinned, so this is the
committed tree not matching them rather than a tool that moved: `.clang-format`
is `BasedOnStyle: LLVM`, which wraps at 80 columns, and the C++ in the
repository is written at 100. `rumdl` reflows most of the Markdown the same way,
and `ruff` reports four errors in `python/mqt/scpd/run.py`,
`solvers/gurobipy_backend.py` and `test/python/unit/{test_planning,test_run}.py`.
The files this change touches are left in the style of their neighbours, and the
rewrite was reverted. Formatting the tree is a change of its own and would bury
this one.

One of those rewrites is **wrong** and has to be reverted whatever is decided
about the rest: `typos` reads the German quotation from `CapacityGrid.cpp:4857`
further up this file and turns "Ein einziger" into "In einziger". A verbatim
quotation is not a typo. Adding the word to a `typos` allow-list would settle
it.

`uvx nox -s schemas -- --check` reports "stale" locally only because the
regenerated files are uncommitted, exactly as in phases 1 and 2.

The eight benchmarks, all three stages:

```bash
mqt-scpd plan -c benchmarks/<chip>/config.toml -o artifacts/<chip> --stage capacity
mqt-scpd plan -c benchmarks/<chip>/config.toml -o artifacts/<chip> --stage global
mqt-scpd plan -c benchmarks/<chip>/config.toml -o artifacts/<chip> --stage assign
```

`--stage X` runs **only** that stage and resumes the run; there is no "up to
X" flag.

| chip | capacity | global | assign | gates | chains | lattices | conns | targets | unserved | ring | resonators | global objective | assign objective |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 4q | 0s | 0s | 0s | 15 | 20 | 0 | 0 | 0 | 0 | 12 | 4 | 0.00 | 5.02 |
| 9q | 0s | 0s | 0s | 27 | 38 | 4 | 6 | 6 | 0 | 30 | 10 | 9896.87 | 15.27 |
| 17q | 1s | 0s | 0s | 62 | 69 | 8 | 14 | 14 | 0 | 58 | 20 | 23419.67 | 27.84 |
| 21q | 2s | 0s | 0s | 58 | 97 | 6 | 8 | 8 | 0 | 70 | 22 | 12890.07 | 34.14 |
| 33q | 8s | 0s | 3s | 90 | 155 | 6 | 8 | 8 | 0 | 110 | 34 | 13576.07 | 58.72 |
| 45q | 11s | 0s | 2s | 122 | 219 | 5 | 7 | 7 | 0 | 150 | 46 | 11891.47 | 86.71 |
| 57q | 24s | 0s | 4s | 138 | 281 | 9 | 11 | 11 | 0 | 190 | 58 | 13667.56 | 115.40 |
| 69q | 49s | 0s | 32s | 185 | 329 | 9 | 11 | 11 | 0 | 230 | 70 | 13667.56 | 141.85 |

`conns` equals `targets` on every chip: one wire per target, nothing unserved.
57Q and 69Q agreeing to eleven digits is not a mistake — the two chips repeat
one unit cell, so their inner circuits are the same eleven wires. Capacity is
byte-identical to the previous session on all eight.

The assignment carries one connection per ring node on every chip, every ring
node has a feed point of its own, no launcher slot carries a resonator or two
conventional ports, and the launcher slots along each ring rise exactly once.

Rendering, which is what the user looks at. All three stages of all eight chips
sit beside the run they came from, as `artifacts/<chip>/<chip>-<stage>.svg` and
`.gds`:

```bash
mqt-scpd plot   -c benchmarks/9q/config.toml --stage capacity --run-dir artifacts/9q -o artifacts/9q/9q-capacity.svg
mqt-scpd render -c benchmarks/9q/config.toml --stage global   --run-dir artifacts/9q -o artifacts/9q/9q-global.gds
```

Every gate carries its wire budget as a number in the SVG, and the port
keep-out is drawn under everything else. The global picture reads the run's
`01-capacity.fb` for both layers, so it shows the gates the circuit had to pay
for and the space the ports had already taken.

## The golden harness

It lives in the session scratchpad, **never in the FridgeCAD tree**, and is a
development instrument: no golden output is pinned in the test suite, so new
tests assert invariants (every bottleneck the plan carries is named by some
chain) rather than recorded coordinates.

```
<scratchpad>/golden/
  golden_9q.cpp    links FridgeCAD's libfridgecad_lib.a, reproduces
                   simple_9q_layout.cpp through buildModelInnerCircuit with no
                   SVG export; writes capacity.fcdb/, setup.json, hanan.json,
                   inner.json
  mine_9q.cpp      our side: grid, targets, masks, bottlenecks, chains,
                   lattices, inner circuit
  build_mine.sh    builds mine_9q against build/dev's static libraries
  compare.py       the six-step comparison
```

Run: `./build_mine.sh && ./mine_9q out && python3 compare.py out`.

Gurobi 13.0.1 at `/Library/gurobi1301/macos_universal2` is used **for the
golden data only**; our side keeps the linked-in HiGHS.

## Traps that cost hours

- **`uv sync` does not rebuild the extension when only C++ changes.** It
  restores a cached archive keyed by the package version, which does not move
  while the tree is uncommitted. Deleting `.nox/tests-3-13`, `build/<tag>` or
  running `uv cache clean mqt-scpd` does *not* help. Two things work:

  ```bash
  uv pip install --python .nox/tests-3-13/bin/python \
      --no-build-isolation --no-deps --reinstall-package mqt-scpd -e .
  ```

  or, before `uvx nox -s tests`, drop the cached archives of this package:

  ```bash
  for d in ~/.cache/uv/archive-v0/*/; do
    [ -f "$d/mqt/scpd/pyscpd.abi3.so" ] && rm -rf "$d"
  done
  ```

  Always confirm behaviourally; a fresh `.so` timestamp only means an archive
  was extracted.
- **HiGHS reads a row-wise `HighsSparseMatrix` as empty.** `passModel` assumes
  a column-wise start vector in places and every constraint then silently does
  not bind. The backend transposes into column-major itself.
- **Boost pulls in `BUILD_SHARED_LIBS` as a cache option**, which turned the
  whole project and HiGHS into shared libraries. `ExternalDependencies.cmake`
  forces it OFF before `FetchContent_MakeAvailable`.
- **A gate line must be four-connected.** A Bresenham line between diagonally
  offset cells is eight-connected and an eight-connected walk steps through it.
- **`flood` must take a visited seed.** The prototype skips visited
  *neighbours* only; skipping visited seeds leaves every chamber beyond a gate
  empty, so no gate survives pruning.
- **`uvx nox -s lint` reformats the whole repository.** Revert everything the
  change does not own before handing back, and do it with a file list a shell
  will actually split; `for f in $LIST` in zsh does not word-split and reverts
  the change itself. This cost an hour.
- A per-step binary "lap" variable in the assignment potential works but
  hardens the MILP badly (45Q went from 25s to over four minutes) and, left
  free, collapses every objective to a whole number.

## Open, in rough priority order

1. **Some chains still end at their own target**, on every chip but 17Q: 4 of
   20 on 4Q, 1 of 38 on 9Q, up to 14 of 329 on 69Q, and on all but 4Q the
   target is a port of the **outer ring**. A ring port whose free space has no
   way out is either real geometry or one more instance of what the gate
   visibility fix addressed; it costs nothing today, because every target is
   served, but it seals the bridge that surfaces there. `Coupler1_4.port1` on
   9Q is the smallest case to look at.
2. **The 9Q comparison against the prototype is stale.** Its global result is
   unchanged, but the gate count moved from 28 to 31 and the chain trees with
   it, so the chain-tree figure under the deviations has not been re-measured.
3. **The assignment's crossing count on 57Q and 69Q** rose by four when the
   ordering potential was held to one turn of the launcher ring. Whether a
   formulation exists that keeps one launcher per conventional port *and* the
   old crossing count has not been looked into.
4. **Only 9Q has been compared against the prototype.** The harness is 9Q-only;
   4Q additionally has a known grid difference (the prototype rounds
   `D_HEIGHT` independently and gets 339 where we get 330).
5. **The chain trees that differ from the prototype's** follow from the
   bottleneck set. If exact tree parity is ever wanted, that is where to start
   — but it means revisiting `bottleneck_clearance`, a deliberate deviation.
6. **The assignment has never been compared against the prototype's own
   output**, only against its formulation as read. The golden harness stops at
   the inner circuit.
7. **The two-backend agreement is untested**: gurobipy is not installed here.
8. **Detail, Final and Geometry stages** are phase 4.

## Before committing

- `artifacts/.gitignore` keeps a run directory out of the repository: every
  `.fb`, `.svg`, `.gds`, `.gds2` and `.oas`, and the `00-chip.json` and
  `config.toml` that `plan` copies in. It lives there rather than in the root
  `.gitignore`, which an external template owns and which must not be edited.
- **64 files under `artifacts/` are still tracked** from an earlier commit, and
  an ignore rule does not reach a tracked file. Untrack them with
  `git rm -r --cached artifacts/` (the working tree is left alone) or delete the
  directory; until then they keep turning up in every diff.
- `17q-global.gds` is **tracked at the repository root**. It is a stray render
  and belongs with the others; the user's own copies live in
  `/Users/michaelfeldmeier/Documents/GitHub/scpd/`.
- CHANGELOG entries guess pull request `#114`; re-check when the PR is opened.
- Decisions added so far: **0028**, **0029** and **0030**, all listed in
  `docs/design/decisions/README.md`.
