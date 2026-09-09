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

## What the session before it did

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
  own length.
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
  hundredths are the proximity tie-break at `PROXIMITY_WEIGHT = 0.01`. 4Q comes
  out at 5.07 against the prototype's 5.02 — the same five crossings. Comparing
  the totals without splitting them apart is what made earlier readings look
  worse than they were. Giving the potential a launcher ring of slack improves
  the tie-break by 0.05 and costs 45Q minutes instead of seconds; it was tried
  and dropped.
- **A bridge whose far end the ring does not name is shut**, where the
  prototype crosses it. It has no switch for this; we do, and it is off
  ([decision 0030](docs/design/decisions/0030-bridge-pairs-are-declared.md)).
  Turning it on reproduces the prototype's behaviour on all eight chips.

## Verification

All green as of this writing.

```bash
cmake --build build/dev && ctest --test-dir build/dev     # 254 tests
uvx nox -s tests                                          # 120 tests x 4 Python versions
uvx nox -s lint
```

`uvx nox -s schemas -- --check` reports "stale" locally only because the
regenerated files are uncommitted, exactly as in phases 1 and 2.

The eight benchmarks, capacity and global only. Assignment was not re-run:

```bash
mqt-scpd plan -c benchmarks/<chip>/config.toml -o artifacts/<chip> --stage capacity
mqt-scpd plan -c benchmarks/<chip>/config.toml -o artifacts/<chip> --stage global
```

`--stage X` runs **only** that stage and resumes the run; there is no "up to
X" flag.

| chip | capacity | global | gates | chains | lattices | conns | targets | unserved | ring | resonators | objective |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 4q | 0s | 0s | 15 | 20 | 0 | 0 | 0 | 0 | 12 | 4 | 0.00 |
| 9q | 1s | 0s | 27 | 38 | 4 | 6 | 6 | 0 | 30 | 10 | 9896.87 |
| 17q | 1s | 0s | 62 | 69 | 8 | 14 | 14 | 0 | 58 | 20 | 23419.67 |
| 21q | 2s | 0s | 58 | 97 | 6 | 8 | 8 | 0 | 70 | 22 | 12890.07 |
| 33q | 7s | 0s | 90 | 155 | 6 | 8 | 8 | 0 | 110 | 34 | 13576.07 |
| 45q | 12s | 0s | 122 | 219 | 5 | 7 | 7 | 0 | 150 | 46 | 11891.47 |
| 57q | 22s | 1s | 138 | 281 | 9 | 11 | 11 | 0 | 190 | 58 | 13667.56 |
| 69q | 47s | 0s | 185 | 329 | 9 | 11 | 11 | 0 | 230 | 70 | 13667.56 |

`conns` equals `targets` on every chip: one wire per target, nothing unserved.
57Q and 69Q agreeing to eleven digits is not a mistake — the two chips repeat
one unit cell, so their inner circuits are the same eleven wires.

Rendering, which is what the user looks at. Both stages of all eight chips sit
beside the run they came from, as `artifacts/<chip>/<chip>-<stage>.svg` and
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
3. **The Assignment stage has not been run since any of this.** The ring moved
   on 17Q and 69Q, so its numbers will move.
4. **Only 9Q has been compared against the prototype.** The harness is 9Q-only;
   4Q additionally has a known grid difference (the prototype rounds
   `D_HEIGHT` independently and gets 339 where we get 330).
5. **The chain trees that differ from the prototype's** follow from the
   bottleneck set. If exact tree parity is ever wanted, that is where to start
   — but it means revisiting `bottleneck_clearance`, a deliberate deviation.
6. **The outer assignment stage** was deferred by the user until the inner one
   was right. It is now right on 9Q, so this is the next thing.
7. **The two-backend agreement is untested**: gurobipy is not installed here.
8. **Detail, Final and Geometry stages** are phase 4.

## Before committing

- Delete `artifacts/` — it is **not** gitignored, and it now also carries the
  SVG and GDSII renderings of both stages.
- Remove stray `*.gds` / `*.svg` / `run9/` at the repo root if a render was run
  there; the user's own copies live in
  `/Users/michaelfeldmeier/Documents/GitHub/scpd/`.
- CHANGELOG entries guess pull request `#114`; re-check when the PR is opened.
- Decisions added so far: **0028**, **0029** and **0030**, all listed in
  `docs/design/decisions/README.md`.
