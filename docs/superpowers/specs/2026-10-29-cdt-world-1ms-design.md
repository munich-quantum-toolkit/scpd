# CDT world: one millisecond per capacity pass

Status: done 2026-10-30. Phase 1 (see "Results") is the shipped state; D10 was tried and dropped (see "Phase 2: chambers from the world"); `capacity_cdt` and `capacity_cdt_world` are `true` by default since 2026-10-30 (schema, Python config, docs, CHANGELOG). Code: `src/grid/CdtWorld.cpp`,
`worldGatesOf` / `capacityPass` in `src/pipeline/FinalRouter.cpp`, the replay
benchmark `test/grid/test_cdt_replay.cpp`, the CDT patch `cmake/PatchCdt.cmake`.

## Goal

The coupler insertion judges every step of a chain's search with a capacity
pass: raster the walls, take the gates (bottlenecks) from the CDT world, label
the chambers, build the flow problem, check it. With `capacity_cdt_world = true`
the gates step alone takes 2.5–6.5 ms per pass (33q_dense, 33q), the whole pass
5–9 ms. A chain's search runs thousands of passes (33q_dense chain 3: 4245), so
the insertion phase of 33q_dense takes 2 min and of 45q_dense over 3 min.

Target: **1 ms per pass** for the gates step (the world), measured as the
pass-weighted mean over the chains of a chip, on 4q, 9q, 17q, 21q, 33q, 45q,
33q_dense and 45q_dense; the whole pass as close to 1 ms as the downstream
steps allow. The gates must stay what they are: a state of the same objects
gives the same gates however it was built (the existing invariant, checked by
`fromScratch`), and the SAT verdicts of the benchmark chips do not change.

## Where the time goes (33q_dense, 6977 passes, pass-weighted mean, ms)

| part | ms | notes |
|---|---|---|
| collect (router) | 0.33 | walk the drawn raster, `std::map` per cell |
| key and add (router) | 0.52 | hash walls, `CdtWorld::add` of new objects (0.1–2.1 ms each) |
| copy | 0.32 | copy the parent state (triangulation, hash sets, `vidOf` of every point of the world) |
| insert | 1.02 | `ensurePairs` over all objects, insert ~600–1000 vertices and their constraints |
| patch and classify | 0.59 | diff all triangles, classify ~1500–3000 of them (`wallCell` loops over every object of the state) |
| candidates and basins | 0.47 | evaluate the edges of the patch, basins of the touched channels |
| measure, chambers, output | 1.12 | wall cells near each gate end (36 cells × every object), relabel, sort |
| **world total** | **4.3** | |
| couplerCuts + fallback passes | 0.8 | 819 of 6977 passes refused by CDT, fall back to the windowed `cdtGatesCached` |
| walls (raster) | 0.7 | |
| chambers (raster) | 1.3 | `chambersOfCached` |
| stretches and ports, check | 1.0 | |
| **pass total** | **8.4** | |

The pass inserts 11–15 objects (the chain's own pads, runs and edges) into the
pinned state (147 objects: everything every pass of the chain had) every time,
although consecutive passes of a search differ in one or two objects.

## Brainstormed optimisations

Ordered by expected gain per risk. "Exact" means the gates do not change.

### A. Infrastructure first

1. **Replay dump and micro-benchmark.** `worldGatesOf` writes, under an
   environment variable, the world's inputs (grid, static walls, options), every
   object the first time it is added, and the object ids of every pass of a
   chain. A gtest (`CdtWorld.DISABLED_Replay`) rebuilds the world and replays
   the passes in-process: per-pass times by lap, and the gates of every pass
   checked against `fromScratch` of the same objects. This makes every change
   below measurable in seconds and profileable with Instruments or `perf`.

### B. Exact: insert less

2. **Reuse the nearest cached state instead of the pinned one.** `CdtWorld::state`
   already picks the cached state whose objects are the largest subset. The
   router should ask for that (with a cache of 32–64 states) so that a sibling
   step inserts only the pad and the edge that differ from the prefix state:
   1–3 objects, 100–200 vertices, instead of 12 objects and 600–1000 vertices.
   Expected: insert, patch, classify, candidates and measure shrink 3–5×.
   Risk: memory (a state is ~2–3 MB), the cache lookup (`includes` over 64
   entries, trivial).
3. **`ensurePairs` only over the objects near the fresh one** (a bucket index of
   object boxes), not over all 2500 objects of the world with a hash lookup
   each. Part of "insert" today.
4. **Cheaper copy.** `vidOf` is sized by every point of the world (hundreds of
   thousands) and copied per pass; keep it per state as the points of the
   present layers only (or a per-layer base index). Drop `present` to a bitset.
   Move the dead candidates out when they grow. Measure what the hash
   containers of CDT (`fixedEdges`, `pieceToOriginals`) cost to copy; if they
   dominate, a patched CDT with flat containers is the fallback (phase D).

### C. Exact: do each thing faster

5. **Spatial index of the objects per world** (squares of 32 cells → the objects
   whose box meets the square): `wallsOfCell`, `wallCell`, `insidePolygon` and
   `wallCellNear` then look at a handful of objects, not at all 160 of the
   state. Used in classify, measure and the Steiner groups.
6. **Router collect/key without `std::map`:** label-indexed vectors, one pass
   over the drawn words; hash the cells as they come. Expected 0.3 → 0.05 ms.
7. **Avoid the O(n) scans per pass** where they are not needed (the chamber
   reset loop, `hit`, `inC` allocations): keep scratch vectors in the world,
   reset by the touched lists.
8. **Fewer points per object:** outlines are simplified with `options.simplify`;
   long feedline edges (7000 cells) still carry many corners and foot points.
   Check how many points an object has and whether `simplify` can grow without
   changing a gate (the gates are measured on cells, not on the outline). Not
   exact a priori; validate with the replay oracle.

### D. Beyond exact: the pass around the world

9. **Fix the refusals** ("Intersection of constraint edges ... computed split
   vertex is invalid", 12 % of the passes of 33q_dense). Each refused pass
   costs the windowed fallback (~5 ms) and gives gates from another method.
   Reproduce with the replay dump; likely cause: a foot laid a hair off a piece
   that another object's constraint then crosses at an angle CDT cannot split.
10. **Chambers from the world.** `chambersOfCached` relabels the raster each
    pass (1.3 ms); the world knows the chamber of every free triangle and
    `chamberAt` locates one cell by a walk. `capacityProblemOf` needs chambers
    at the ports and stretches only. Changes the verdicts where a raster
    chamber and a triangle chamber disagree; validate with sat-bench.
11. **No raster at all** for a world pass: the router knows the objects it drew
    (wires and pads); key them by wire and path instead of by cells, raster an
    object once when it is first seen. Removes "walls" (0.7 ms) and "collect".
    Needs `couplerCuts`, `capacityProblemOf` and the slot check to live without
    `walls.mask` in a world pass; the largest refactor.
12. **Copy-free children** (undo log instead of copying the parent): only if 2–4
    leave copy as the floor above 1 ms.
13. **Parallel sibling passes**: the trellis could evaluate the options of one
    coupler in parallel on copies; last resort, touches the search.

## Plan

Phase 0: baseline over the eight chips (`runs/_cdt-bench/bench.py` in the
planar repo), A1. Phase 1: B2, B3, C6, C5, B4, C7 — each measured on the replay
bench, gates checked against `fromScratch`, then the eight-chip bench. Phase 2:
D9, then D10 with sat-bench agreement. Phase 3 (if needed): C8, D11, D12.

## Testing

- `CdtWorld.DISABLED_Replay` checks every pass of the replay against
  `fromScratch` (gates, cuts, capacity, beside) and prints the lap times.
- The existing `test_cdt_world.cpp` tests (order, cache, one-by-one versus all
  at once) stay green.
- `bench.py run --label <x>` on the eight chips: ms per pass, refused passes,
  SAT/UNSAT verdict of the insertion phase against the baseline.
- ChipBuilder: `python -m chipbuilder build --check` and `pytest tests` stay
  green (the inputs are not touched).

## Results (2026-10-30)

Replay of 33q_dense (6979 passes of the coupler insertion, gates checked
against `fromScratch` every 20th pass, all 196 grid tests green):

| step | before | after | what changed |
|---|---|---|---|
| world per pass, mean | 3.29 ms | 1.11 ms (median 0.94) | everything below |
| refused passes | 819 (12 %) | 0 | A: constraint edges inserted once per pass (two layers that overlap share pieces); B: `Snapper`, a CDT callback (`CDT_ENABLE_CALLBACK_HANDLER`) that puts the vertex CDT adds at a crossing exactly onto the row or column of the constraint being inserted, so the rest of it meets the vertices on its line exactly |
| objects inserted per pass | 15.6 | 4.4 | the cache: largest cached subset, else the common part with the sibling that shares most, built and kept as a state of its own (`cached`); the router asks `state(ids)` with a cache of 64 |
| measure | 0.85 | 0.04 | objects by 8-cell squares: world-level lists (reach 3) for `relate`, per-state linked lists of the present objects for `wallsOfCell`, `wallCell`, `classify` |
| classify | 0.45 | 0.06 | same |
| candidates + basins | 0.45 | 0.18 | `groupsAt` cached per vertex while a state is built (epoch stamps, nothing copied) |
| insert | 1.04 | 0.43 | fewer objects; `LastPointLocator` + `VertexInsertionOrder::AsProvided` (CDT inserted incrementally in random order with a KD-tree nearest query per point); `ensurePairs` over the present objects only |
| copy | 0.11–0.32 | 0.075 | `CDT::EdgeUSet` replaced by the flat open-addressing `FlatEdgeSet` (`cmake/PatchCdt.cmake`, FetchContent PATCH_COMMAND; the KD-tree is gone from the state too) |
| `add` per object | 0.65–2.0 ms | ~0.6 ms | chunked stamps and crack index, point squares of 8 cells, base corners indexed; the add laps were mislabelled before (off by one) |

Eight-chip bench (`bench.py`, pass-weighted ms per pass, world / whole pass),
baseline → final (`flat`): 4q 1.8/3.1 → 1.3/2.9; 9q 2.4/4.3 → 1.8/3.6; 17q
2.6/4.6 → 1.4/3.3; 21q 2.9/5.5 → 1.2/3.5; 33q 2.4/9.2 → 1.7/5.0; 45q 4.6/12.7
→ 2.4/7.0; 33q_dense 4.5/8.7 → 1.7/4.9; 45q_dense 4.5/10.1 → 1.5/5.4. The
gates step including fallbacks: 1.8–7.6 → 1.3–2.4 ms. The rerun `final`
(2026-10-30, after D10 was removed again) repeats these within noise: world
1.2–2.4, whole pass 2.8–7.1 ms. SAT/UNSAT verdicts of every chip, pass
counts, drawn edges and angle costs unchanged against the baseline; one
step of 33q_dense chain 4 judges SAT where the baseline judged UNSAT (2075
of 2091 against 2074), where the baseline's gates came from the windowed
fallback after a CDT refusal. No pass refused. What the router adds
over the replay's 1.1 ms: collecting the drawn cells (0.3 ms, inherent to
the raster), keying and adding new objects (0.1–0.3 ms), the cache (~0.1).

Rejected: CDT's `minDistToConstraintEdge` tolerance (1e-10) removes the
refusals too but makes the triangulation depend on the insertion order (two
canonicality tests fail).

## Decision point for phase 2

The world is near 1 ms; the whole pass is 5–8 ms because of the raster steps
around it: `wallsOf` (0.7 ms), `chambersOfCached` (1.3 ms), stretches and
ports (0.6 ms), the check (0.4 ms). Getting the pass itself to 1 ms needs
D10/D11: chambers from the world's triangles instead of the raster, and a
pass that does not raster the walls at all. Both change what the check sees
in degenerate places (a cell under a non-free triangle, chambers that differ
at a gate's end), so the verdicts may move; that is the user's call, to be
validated with sat-bench.

## Phase 2: chambers from the world (D10, 2026-10-30, dropped)

The user asked for D10 to be tried. What the pass code showed first:

- Every step of the search takes the terminal edges as wires
  (`stepStateOf` sets `terminalsAsWires`), so every pass has, besides the
  world's cutting gates, the slot gates of its terminal runs (two per pass on
  33q_dense; no mouth gates were ever closed there), and `dropCrossingGates`
  drops one to three world gates per pass whose raster lines cross a slot
  gate's or each other. `couplerCuts` drops nothing: the world's `cuts` is
  the same predicate.
- The slot check reads the raster cell by cell through `SlotGraph::raster`,
  but only whether a cell is free; the chamber numbers it would need for
  `capacity_chamber_fill` are a scan over every cell, inactive in the
  benchmark configurations.
- The raster labelling (`chambersOfCached`) costs 1.3 ms a pass on
  33q_dense, 1.0 of it the labelling itself (`labelChambers`), 0.12 the fill
  of the 16 MB cell table; rasterising every triangle of the world costs
  3.7 ms. A lookup of one cell in the world (`chamberAt`) costs 13 ns for
  cells in a row (the walk starts from the last triangle) and ~100 ns at
  random.
- Raster and world agree on 99.99 % of sampled free cells (same or
  different chamber), but at the gates they differ: on 6–8 % of the cutting
  gates one says the gate parts two chambers and the other not (tiny raster
  pockets of a few cells beside a gate's end; the raster's look-through,
  which adds the chamber behind a touching gate's line as a third side), and
  a quarter of the gates name other chambers beside them. So the verdicts
  may move; the eight-chip bench says whether they do.

The design, behind `MQT_SCPD_CDT_CHAMBERS` (an environment variable for
the trial; a configuration option needs a schema field):

1. The gates step hands the state it built to the pass (`cdtWorldLast_`);
   `dropCrossingGates` records where every kept gate stood before the drop.
2. `worldChambersOf` takes the world's chambers by rank. The two chambers
   beside every world gate the pass dropped are joined (a union-find over
   the ranks). The refused steps of the pass's gates are brought up to date
   as before (`chamberCacheUpdate`, the first half of `chambersOfCached`),
   and every slot or mouth gate is flooded from both sides of its line, a
   cell from each side in turn, over the steps the raster labelling allows
   (`floodPocket`): the side that runs out first is a pocket and a chamber
   of its own; the sides meeting means the gate parts nothing; both sides
   past a budget of 32 768 cells means no answer, and the pass falls back
   to the raster.
3. The chambers are numbered: the world's that stand, by rank, then the
   pockets. The chambers beside a world gate are its two triangles' (mapped
   through the joins); beside a slot gate, and beside a world gate within
   two cells of a pocket, they are read off the free cells of the gate's
   moves.
4. The consumers of the labels per cell — the stretches of the feedline
   edges, the chambers of the ports, the terminal cells of the slot rule —
   ask a `ChamberLookup`: the dense table when the raster labelled, else
   the pockets (sorted) and then the world (`chamberAt` with the last
   triangle as the hint). The slot check's view runs without chamber codes
   (every free cell reads as passable, which is all it uses). The reports
   and the slot graph by codes materialise the table (`denseChambers`).

### Result and decision

On 33q_dense the chambers step fell from 1.0–1.3 to 0.4 ms a pass and the
whole pass from 4.9 to 3.6 ms, but the verdicts moved: the chip came out
UNSAT, every chain's search ran 3000–5500 passes (26 672 in all against
6977), and in the passes whose verdict differed the world lost the same
wires every time. The comparison pass by pass (both labellings computed,
`MQT_SCPD_CDT_CHAMBERS=diag`) showed why: the world judges more strictly
than the raster. Where several gates fan out from one flank tip, the
raster's look-through lets each gate open onto every chamber of the fan (a
gate with six chambers beside it), while a world gate joins exactly the two
chambers of its triangles; and 4–6 % of the gates part nothing in the
raster (their cell line runs through wall cells, or leaks round an end)
but do in the world. Emulating the look-through in the world (the cells
beside a gate's line read through other lines, as `labelChambers` does)
did not bring the verdicts back (chain 0: 338 of 5551 passes differed).

The chips are SAT under the raster's semantics and are routed after it, so
the user dropped the approach (2026-10-30). The router is back at the
phase-1 state; the D10 version of `FinalRouter.cpp` is kept outside the
repository. What stays: the measurement `CdtWorld.DISABLED_ReplayChambers`
(raster against world chambers on a replay) and `chamberCacheUpdate`, the
first half of `chambersOfCached` as a function of its own.
