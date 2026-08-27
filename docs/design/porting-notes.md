# Porting notes

A map of the FridgeCAD prototype, for contributors carrying out the rewrite.
**Delete this document once the port is complete.**

The prototype is roughly 64,000 lines of C++. Around 15,000 to 18,000 of that is
real algorithmic content. The rest is duplication, dead code, hand-rolled SVG,
and commented-out history. The single most useful thing to know before starting
is which is which.

## Size ledger

| File                         |  Lines |   Code | What it actually is                                                                  |
| ---------------------------- | -----: | -----: | ------------------------------------------------------------------------------------ |
| `FinalGrid.cpp`              | 19,413 | 14,491 | Six variants of one routing driver, one genuine coupler optimizer, ~5,250 dead lines |
| `dubin_router_opt.hpp`       | 10,392 |  7,632 | The A* that is actually used; ~5,000 lines shared with its twin                      |
| `CapacityGrid.cpp`           |  8,968 |  7,002 | Watershed and distance transform, plus 928 lines of SVG and ~1,100 duplicated        |
| `dubin_router.hpp`           |  6,883 |  5,188 | The superseded twin. Not ported                                                      |
| `QubitLayout.cpp`            |  4,061 |  2,874 | Chip model, plus a 1,771-line curve-fitting engine hidden in an anonymous namespace  |
| `DetailedGrid.cpp`           |  2,172 |  1,542 | Per-partition A* and wire stitching                                                  |
| `move_primitives.hpp`        |  2,097 |    834 | Lines 1–1096 are an older copy of the same class, fully commented out                |
| `OrderedAssignmentGraph.cpp` |  1,455 |  1,230 | The ring assignment model, plus 137 lines of SVG and a 202-line dead twin            |
| `QubitLayoutOptimizer.cpp`   |    949 |    772 | The inner-circuit model. The only place Gurobi is properly encapsulated              |

## Where each prototype class goes

| Prototype                                                           | Destination                       | Notes                                              |
| ------------------------------------------------------------------- | --------------------------------- | -------------------------------------------------- |
| `QubitLayout` — geometry, ports                                     | `MQT::ScpdDesign`                 | Entity model replaces label-string semantics       |
| `QubitLayout` — JSON load                                           | `MQT::ScpdIO`                     | Schema-validated                                   |
| `QubitLayout` — SVG export, five overloads                          | deleted                           | KLayout renders; `plot.py` for debug views         |
| `QubitLayout::align_routed_paths` and its 1,771-line helper block   | `IFinalizer`                      | Becomes a stage in its own right                   |
| `QubitLayout::verify_min_clearance`                                 | `MQT::ScpdIO`                     | One checker, not two                               |
| `QubitLayout::report_qor`                                           | deleted                           | Metrics become structured output                   |
| `CapacityGrid` — rasterize, EDT, watershed                          | `MQT::ScpdGrid`                   | Genuine algorithms, port them                      |
| `CapacityGrid` — chains, budgets, routing requests                  | `ICapacityPlanner`                |                                                    |
| `CapacityGrid::svg_overlay` (928 lines)                             | deleted                           |                                                    |
| `CapacityGrid::solve_capacity_graph`                                | deleted                           | Dead, and the only reason Gurobi reaches this file |
| `CapacityGrid::compute_capacity_chain*` non-optimized pair          | deleted                           | Superseded by the `_optimized` twins               |
| `OrderedAssignmentGraph`                                            | `IAssigner`                       | Precompute the geometric inputs first              |
| `OrderedAssignmentGraph::compute_min_overlap_assignment`            | deleted                           | Dead reference formulation                         |
| `HananGrid`                                                         | `IGlobalRouter`                   | Smallest and most reusable class in the prototype  |
| `QubitLayoutOptimizer`                                              | `IGlobalRouter` + `MQT::ScpdMilp` | Model building splits from solver access           |
| `DetailedGrid`                                                      | `IDetailRouter`                   |                                                    |
| `FinalGrid` — the parallel `run_final_routing` and feedline drivers | `IFinalRouter`                    | Collapse six into one                              |
| `FinalGrid` — `run_optimized_cpw_coupler_insertion`                 | `IFinalRouter`                    | 2,126 lines of genuine algorithm. Port it          |
| `FinalGrid` — dead methods                                          | deleted                           | ~5,250 lines, listed below                         |
| `dubin_router_opt.hpp`                                              | `MQT::ScpdRouting`                | Port this one                                      |
| `dubin_router.hpp`                                                  | deleted                           |                                                    |
| `move_primitives.hpp` lines 1097–2097                               | `MQT::ScpdRouting`                | The live class only                                |
| `src/simple_*q_layout.cpp`, ten files                               | deleted                           | Replaced by the CLI and recipes                    |
| `src/simple_outing.cpp`                                             | `test/routing/`                   | Becomes a GoogleTest                               |
| `mini_project/`, `lib/`                                             | deleted                           | Unrelated demo, and a stale build tree             |

## Dead code inventory

Confirmed zero call sites. Do not port any of it.

- `FinalGrid`: `run_final_routing` (757 lines), `run_final_routing_feedline`
  (1,412) and its only caller `run_final_routing_feedline_choices` (609),
  `run_cpw_coupler_insertion` (60), `run_feedline_routing` (690),
  `run_bridge_insertion` (262), `run_bridge_smoothing` (618),
  `finalize_coupler_positions` (751), `debug_svg` and `debug_proxy_svg` (44).
- `CapacityGrid`: `solve_capacity_graph` (140), `computeManhattanDT` (107),
  `rasterizeVoronoi_new` (92), `computePreciseEDTVoronoi` (161), and the
  non-optimized chain pair (534).
- `OrderedAssignmentGraph`: `compute_min_overlap_assignment` (202),
  `add_circular_consecutive_constraint` (20).
- `QubitLayout`: `export_netlist_to_svg` (45).
- `QubitLayoutOptimizer`: seven `GRBVar` member maps that the solver never
  touches; the real model builds its variables in locals.

## Traps

Things that will cost a day if discovered late.

**The two routers are a fork, not a refactor.** They share about 70
identically-named methods. The comments record that a `route_orthogonal`
shadowing bug was fixed in the copy and deliberately left in the original. Port
`_opt`; read the other only to understand intent.

**`FeedlineRouterCtx` documents a dangling reference.** The router's
`primitives` is a reference member bound to a caller stack local, which dangles
in the reused and threaded cases. The prototype works around it in the caller
rather than fixing the router. Fix the router.

**The assignment model is not separable as written.** It calls
`CapacityGrid::route_target_to_nearest_launcher` during model construction and
mutates the capacity grid after solving. Precompute those lookups before
attempting any solver abstraction.

**Threading correctness is by interference guard, not by locking.** Threads work
on disjoint blocks of the wire ring with a guard radius sized so two active
threads never read the same region. Dynamic work stealing was tried and reverted
because rip-up-and-reroute needs the same sweep that ripped a neighbour to
re-route it. Preserve the reasoning even if the mechanism changes.

**Hard-coded paths and labels are compiled into the library.** Five drivers load
their configuration from absolute paths into another machine's checkout. Every
driver passes a Gurobi licence path that does not resolve. Several per-benchmark
debug probes survive as live branches, including one that tests for a
4-qubit-era label inside a 69-qubit code path, and four copies of a loop that
assigns an uninitialized variable that is then never read.

**`HananGrid::is_point_in_polygon` silently skips geometry.** It ignores the
first two polygons, assuming they are the chip outline, and any polygon with
fewer than ten vertices. On the 69-qubit chip the first polygon has four
vertices. Understand what obstacle testing actually covered before treating the
prototype's results as ground truth.

**Configuration is spread across four mechanisms**: two parameter structs, 38
`getenv` sites, roughly fifteen constants hard-coded at call sites, and
per-benchmark hand tuning. The header records that one penalty shipped as values
between 8,000 and 30,000 across benchmarks.

## Reference numbers

From the prototype's own benchmark table, for the 69-qubit chip. These are the
targets to reproduce.

| Metric                    | Value                                |
| ------------------------- | ------------------------------------ |
| Final unresolved wires    | 0                                    |
| Total feedline angle cost | 188                                  |
| Wall-clock runtime        | 456.1 s                              |
| Of which Dubins routing   | 355.7 s, about 78 percent            |
| Assignment objective      | 132.75                               |
| Peak memory               | roughly 7.5 GB, four router contexts |

Per-stage timings and the full eight-benchmark table are in the prototype's
`README.md`.
