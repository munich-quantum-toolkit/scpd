# Phase 4, step 2 — detail routing

Written for whoever routes the pixels. Step 1 (the Corridor stage) is done and
verified; this says what you inherit, what the job is, and what step 1 learned
the hard way. Read [summary-stage-4.md](summary-stage-4.md) first for the stage
you are building on; this file does not repeat it.

- Checkout: `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`
- Branch: `phase-4-routing-stages`, based on `8ef300a`. **Nothing is committed —
  the user commits.** Leave your work in the tree and report what you verified.
- Prototype: `/Users/michaelfeldmeier/Documents/GitHub/FridgeCAD` (`0c5d6d9`),
  read only. It is the specification; it is also wrong in places (below).
- The design plan for both steps is
  `/Users/michaelfeldmeier/Documents/GitHub/scpd-design-plan-backup/docs`.

## Where the pipeline stands

```text
01-capacity.fb  02-global.fb  03-assign.fb  04-corridor.fb   written
05-detail.fb    06-final.fb   07-geometry.fb                  not written
```

`DetailRouting` in [schemas/artifacts.fbs](schemas/artifacts.fbs) is still the
empty placeholder `table DetailRouting {}`. Filling it is yours.

## What you inherit

`04-corridor.fb` carries, for every connection of `03-assign.fb`, in that same
order and with no key to look up:

| field | meaning |
| --- | --- |
| `Corridor.source` | the point the assignment feeds the wire at |
| `Corridor.target` | the cell its target port is reached at |
| `Corridor.partitions` | the partitions it runs through, in order |
| `Corridor.crossings` | where it crosses from each partition to the next — one fewer than `partitions` |
| `CorridorRouting.slots` | every place a wire may cross, per border, plus one entry marked `pocket` for the ways out of the port pockets |

A corridor with no partitions was not routed. On the eight benchmark chips
every connection has one, so if you see an empty corridor, something upstream
broke — check before you route around it.

**What the plan guarantees**, all of it checked in ctest and again against the
written artifacts:

1. One wire per crossing place.
2. No two wires cross inside a partition, and none runs along another.
3. No wire runs over a point another wire is *pinned* to (a feed point or a
   target cell).
4. No wire crosses a border outside the ring the ports feed from.
5. No wire turns straight back into the partition it just left.
6. No wire crosses itself or re-uses a place.

**What it does not guarantee, and you must not assume:**

- **That two crossings on one border are far enough apart for your wires.** The
  crossing pitch is a *planning* figure (`[stages.capacity] crossing_pitch`,
  165 units, the prototype's `CON_MIN_WIRE_DIST − 20`). How far two wires end up
  apart is your answer and the DRC's verdict.
- **That a chord fits.** A chord is a straight line between two places; the
  partition it crosses is not convex, and your path will be longer than it.
- **That the inner connections are in there.** The connections of
  `02-global.fb` never entered the corridor stage — they carry their capacity
  story in the Hanan model (decision 0028) and come to you unrouted. The
  prototype does the same.
- **That a partition is one connected blob.** It is what the watershed made of
  it.

## The job

Three passes, as in `DetailedGrid::run`
(`include/fiction/layout/DetailedGrid.cpp:2165`), minus its reconstruction
step, which you do not need — the corridor already knows which wire is which.

**0. Get the labels back.** The A\* is confined to one partition, and a label
grid is derived state that is in no artifact. `grid::rasterizePartitions`
([include/mqt-scpd/grid/Partitions.hpp:137](include/mqt-scpd/grid/Partitions.hpp#L137))
already fills the cells back in from the outlines; the corridor stage uses it
(`labelsOf` in [src/pipeline/CorridorRouter.cpp](src/pipeline/CorridorRouter.cpp)).
Do not run the watershed again — on 69Q it is most of the capacity stage's
runtime.

**1. Route inside each partition** (`detailed_pre_routing`, `DetailedGrid.cpp:399`).
Every corridor piece that crosses a partition, 8-connected A\* on the detail
grid, costs `{10, 14}`, octile heuristic. Only the *inner* cells of a path are
occupied, so two wires may share an endpoint on a border. Diagonal corners are
not cut. Up to `MAX_VARS = 16` orderings are tried (`DetailedGrid.cpp:454`,
sorted by start cell, then `std::next_permutation`); the ordering that routes
the most pieces wins.

**2. Join them up.** A concatenation in corridor order — not a search. The
prototype has to recover which fragment belongs to which wire by comparing
endpoints (`reconstruct_wires`, `DetailedGrid.cpp:832`) and loses a cell at each
end of every wire doing it. You have the identity; keep it.

**3. Re-route across borders** (`detailed_cross_boundary_routing`,
`DetailedGrid.cpp:1445`). `rounds = 8`, alternating forward and backward, wires
that are already good are skipped. Per wire: the allowed cells are a Chebyshev
box of half-width `K = 40` around its own path, minus discs of radius `J` around
the paths of its two ring neighbours.

**4. The inner connections** last, against the ring wires as hard blockers.

### Parameters, and which of them are real

| name | value | where | note |
| --- | --- | --- | --- |
| `K` corridor half width | 40 | `DetailedGrid.hpp:53` | grid-dependent: 11 % of the 9Q grid width, 2.7 % of 69Q. Port it, measure it, and if a chip fails on it express it relative to the grid |
| `J` blockade radius | derived | `DetailedGrid.cpp:1455` | `ceil(CON_MIN_WIRE_DIST × pixel_per_unit) − 1`. **Derive it. Never read the literal.** |
| `J` again | `6` | `DetailedGrid.cpp:1770` | the same quantity read as a literal default in `run_inner_routing_requests` — this is the inconsistency `data-model.md` names |
| `rounds` | 8 | `DetailedGrid.hpp:65` | |
| `max_relaxation` | 10 | `DetailedGrid.hpp:67` | |
| `orderings` | 16 | `DetailedGrid.cpp:454` | |
| obstacle penalty | radius 6, value 10 | `DetailedGrid.hpp:61,63` | linear falloff |
| `params.I` | — | `DetailedGrid.cpp:1772` | read, never used. Dead |

Put them in `[stages.detail]` of `schemas/config.fbs` next to `CorridorParams`,
and parse them in `python/mqt/scpd/config.py` — both stages' parameters are
already there to copy from.

## Lessons from step 1

Ordered by how much time each one cost.

**1. The prototype is the specification, and it is not all live.** Read the call
graph, not only the function. Step 1 lost an afternoon to `digTargetBeyondBand`,
which is documented "for the router grid" and is called by *nothing*; the
capacity grid uses a variant that opens no channel, and five wires sat in sealed
pockets because of it. In your part: `params.I` is dead, the capacity guard in
the coarse router is commented out, and there is a
`for (back_count = 0; back_count <= 0 && !success; ++back_count)` in the middle
of your own cross-boundary re-routing (`DetailedGrid.cpp:1587`) whose body runs
exactly once — the comment above it describes an escalation that does not
happen. Grep for the caller, and read the loop bounds, before you port a
behaviour.

**2. The prototype's own checks have holes, and two of them mattered.** Its
`check_edge_crossing` tests *proper* crossings only, and the occupancy it keeps
beside that is a set of pixels — never the lines between them. So a wire may
stop on top of another wire's endpoint and nothing objects. Step 1 added two
rules the prototype does not have:

- no wire over another wire's **pin** (11 such tunnels on four chips without it),
- no crossing outside the **ring** the ports feed from (27 without it — the
  prototype does clamp its border pixels to the launcher box,
  `CapacityGrid.cpp:7342`, and step 1 had simply not ported it).

Expect the same class of hole in `DetailedGrid`. The one already visible: the
launcher clamp exists there too (`astar_in_corridor`, `DetailedGrid.cpp:1128`) —
port it rather than trusting the blocked border strips of the scene, and say so
if you measure that it is not needed.

**3. Write the check before you believe the result.** Every defect step 1 found
was found by a verification test, never by looking at a picture — and the two
the *user* found were also expressible as tests within minutes. Write each
invariant twice: once in ctest over all eight chips, and once as a small script
that reads the written artifact through `mqt-scpd inspect`. The second one has
caught a stale build more than once. For detail routing the invariants are
already known:

- a wire's cells are 8-connected and free of obstacles,
- a wire lies, in order, in the partitions its corridor names,
- no wire self-intersects (`routing::pathSelfIntersects`,
  [include/mqt-scpd/routing/SelfIntersection.hpp:116](include/mqt-scpd/routing/SelfIntersection.hpp#L116)),
- two wires share no cell except on a partition border,
- first and last cell are the feed and the target — the place the prototype is
  off by one,
- two runs of the same input give the same answer.

**4. Every number in a doc must have been measured, and re-measured when the
code changed.** Step 1 wrote "the sweep alone leaves thirteen" early on; by the
end it was fifteen, and the sentence had to be re-measured to stay true. If you
change a mechanism, re-run the measurement behind every claim you inherit.

**5. Rounding: snap to the grid you came from.** Border samples live in layout
units; reading them back with `toCell` gives 937.4999999999999 for what was
937.5, the determinant is then not exactly zero, and two collinear points look
separate. `onGrid` in [src/pipeline/CorridorRouter.cpp](src/pipeline/CorridorRouter.cpp)
snaps to half-cells; every geometric predicate downstream is exact only because
of it. Your paths are whole cells, which is easier — but the corridor endpoints
you read are half-cells.

**6. Exclusivity is per *place*, not per entry.** Where three partitions meet,
one point is a slot of each of the three borders and still one place on the
chip. The same trap waits wherever you index by "the thing" instead of by "where
the thing is".

**7. Do not build machinery before you have measured the failure.** Step 1 built
a displacement chain that shifts a whole fan of wires one rung along a border —
correct, terminating, and worth nothing: with the pin rule in place it changed
not one byte of any artifact, and it was taken out again. Two hours. Measure the
one failing case first, then decide.

**8. What did work, and transfers.** Rip-up needs *both* the prototype's sweep
over ring neighbours (a big hole) and a targeted rip of the wires actually in
the way (one or two per failure): 15 unrouted for the sweep alone, 32 for the
targeted rip alone, 0 together. A place must remember having been wanted, or two
wires trade it for ever. And a best route that turns out to be a short must not
end the search — set its places aside and search again.

**9. Determinism is a stage contract**, because a resumed run must equal an
uninterrupted one. The prototype spreads its pre-routing over
`hardware_concurrency()` threads and gets an order-dependent fragment list. Stay
single-threaded; concurrency is phase 6.

**10. The build traps, all of which have bitten:**

- `uv sync` does **not** rebuild the extension when only C++ changed. Use
  `uv pip install --python .venv/bin/python --no-build-isolation --no-deps --reinstall-package mqt-scpd -e .`
  and check *behaviour*, not the `.so` timestamp — a fresh timestamp only means
  an archive was unpacked.
- `clang-format` reflows what you just wrote, so a patch anchored on your own
  formatting fails the next time. Re-read the file before patching it again.
- `uvx nox -s lint` reformats ~66 files this branch never touched, and rewrites
  every `# noqa: <code>` into `# ruff: ignore[<name>]`. Revert everything you do
  not own; map those comments back in the files you do.
- `uvx nox -s stubs` emits invalid Python for the `global` stage (`global: bytes`).
  `python/mqt/scpd/pyscpd.pyi` is maintained by hand.
- Long runs belong in the background; the 69Q corridor test alone is 74 s and the
  whole corridor suite 260 s.

**11. The picture is a deliverable, and it has a budget.** 10 MB per SVG
(`pipeline.md`). A 69Q detail wire is a few thousand cells, so draw each polyline
with its collinear runs collapsed — that pulls an 8-connected path down to its
bends. Check the size of all eight before you report.

**12. How this work is run.** Implement → test → generate SVG *and* GDS for all
eight chips → report → wait for feedback. Reply to the user in German; write
everything in the repository in English. **Do not commit.**

## The files you will touch

| file | what |
| --- | --- |
| `schemas/artifacts.fbs` | fill `DetailRouting`: `grid`, `wires`, `inner`; paths as `geometry.DCoord` |
| `schemas/config.fbs` | `DetailParams`, and `detail` in `StageParams` |
| `include/mqt-scpd/pipeline/Stages.hpp` | `IDetailRouter::run(chip, capacity, global, assignment, corridor, config)` |
| `include/mqt-scpd/pipeline/DetailRouter.hpp`, `src/pipeline/DetailRouter.cpp` | the stage; `makePixelAStarRouter()` |
| `src/pipeline/Registry.{hpp,cpp}` | `detailRouters()`, `selectedDetailRouter()`, default `"pixel-astar"` |
| `src/io/Artifacts.cpp` | a `case` in `validate` |
| `bindings/bindings.cpp` | `route_detail(...)`, an entry in `algorithms()` |
| `python/mqt/scpd/{run,planning,plot,artifacts,inspection,config}.py`, `export/klayout.py` | the stage, the read-back, a layer, a colour |
| `test/pipeline/test_detail_router.cpp` | the invariants above, over all eight chips |

Every one of these has a Corridor twin committed to the tree already — copy the
shape rather than inventing one. `test/pipeline/Benchmarks.hpp` builds the
fixtures; `include/mqt-scpd/pipeline/CapacityPlanner.hpp` has `buildScene`, which
the routing stages are meant to rebuild from (its comment says so). No CMake
change is needed — `AddMQTScpdLibrary.cmake` globs.

Reusable, and already tested: `grid::BitGrid`, `routing::BucketQueue`,
`routing::pathSelfIntersects`, `grid::GridMetrics::refined`/`cellsFor`,
`grid::rasterizePartitions`, `grid::orientationStep`. A pixel A\* is the one
genuinely new algorithm.

## Verification

```bash
cmake --build --preset release && ctest --preset release   # 287 tests today
uv run --no-sync pytest test/python/unit                   # 125 tests today
uvx nox -s schemas                                         # after any .fbs change
uvx nox -s stubs                                           # after any bindings change
```

Then all eight chips, with the pictures:

```bash
for c in 4q 9q 17q 21q 33q 45q 57q 69q; do
  mqt-scpd plan   -c benchmarks/$c/config.toml -o artifacts/$c --stage detail
  mqt-scpd plot   -c benchmarks/$c/config.toml --stage detail --run-dir artifacts/$c \
                  -o artifacts/$c/$c-detail.svg
  mqt-scpd render -c benchmarks/$c/config.toml --stage detail --run-dir artifacts/$c \
                  -o artifacts/$c/$c-detail.gds
done
```

Report per chip: unrouted connections, runtime, SVG size. The bar is the
prototype's own table in `FridgeCAD/README.md`: **`DetailFail` 0 on all eight**,
at 0.08 s to 46.9 s. Step 1 met its half of that bar (968 of 968 corridors,
0.1–11.6 s against the prototype's 0.35–11.5 s).

## Open, inherited

1. **The crossing pitch decides which chip is hard.** At the design rule's 185
   the 45-qubit chip lost a connection; at the prototype's 165 it does not. One
   number for every chip, on purpose. If detail routing turns out to want more
   room than the pitch leaves, that is the number to argue about — with a
   measurement, not a per-chip override.
2. **`digTargetBeyondBand` is still called by nothing.** Step 1 adds a port's way
   out of its own pocket to the partition graph instead. You build your own mask
   and should dig it there; then the two ways of saying it want reconciling.
3. **Nothing has been compared against the prototype's own output**, only against
   its formulation as read and its published failure counts. A run of FridgeCAD
   on 9Q, side by side, would be worth more than another test.
4. **The obstacle keepout is not in these stages.** It lives in
   `FinalGrid::rasterize_obstacles` (`FinalGrid.cpp:316`), and
   `RasterOptions.keepout` in `include/mqt-scpd/grid/Rasterize.hpp` is ready for
   it. Corridor and detail share the capacity scene's mask.
