# Porting notes

A map of the FridgeCAD prototype, for contributors carrying out the rewrite.
**Delete this document once the port is complete.**

Written against the `FridgeCAD-0fails` snapshot. The prototype is a moving
target: it reaches zero unresolved wires on all eight benchmarks, and several
things earlier drafts of this document described as defects have since been
fixed upstream. Where that has happened it is called out, because the fixes are
the parts most worth carrying over.

The prototype is roughly 68,000 lines of C++. Around 15,000 to 18,000 of that is
real algorithmic content. The rest is duplication, dead code, hand-rolled SVG,
and commented-out history. The single most useful thing to know before starting
is which is which.

## Size ledger

| File                         |  Lines | What it actually is                                                                  |
| ---------------------------- | -----: | ------------------------------------------------------------------------------------ |
| `FinalGrid.cpp`              | 22,859 | Four live routing drivers and a coupler optimizer; 6,248 lines dead or withdrawn     |
| `dubin_router_opt.hpp`       | 10,438 | The A* that is actually used; ~5,000 lines shared with its twin                      |
| `CapacityGrid.cpp`           |  9,537 | Watershed and distance transform, plus 377 lines of SVG and 532 superseded           |
| `dubin_router.hpp`           |  6,949 | The superseded twin. Not ported                                                      |
| `QubitLayout.cpp`            |  4,061 | Chip model, SVG, JSON, and the curve-fitting engine behind `align_routed_paths`      |
| `DetailedGrid.cpp`           |  2,276 | Per-partition A* and wire stitching                                                  |
| `move_primitives.hpp`        |  2,097 | Lines 1–1073 are an older copy of the same class, fully commented out                |
| `OrderedAssignmentGraph.cpp` |  1,455 | The ring assignment model, plus 112 lines of SVG and a 202-line dead twin            |
| `FinalGrid.hpp`              |  1,329 | Declarations plus the parameter struct, which is where the tuning history lives      |
| `QubitLayoutOptimizer.cpp`   |  1,109 | The inner-circuit model. The only place Gurobi is properly encapsulated              |

## Where each prototype class goes

| Prototype                                                             | Destination                       | Notes                                                    |
| --------------------------------------------------------------------- | --------------------------------- | -------------------------------------------------------- |
| `QubitLayout` — geometry, ports                                       | `MQT::ScpdDesign`                 | Labels resolved to indices once; roles from patterns     |
| `QubitLayout` — JSON load                                             | `MQT::ScpdIO`                     | Same format, validated. `nets`/`sampleSpacing` rejected  |
| `QubitLayout` — SVG export, five overloads                            | deleted                           | KLayout renders; `plot.py` for debug views               |
| `QubitLayout` — clockwise port ordering (see below)                   | deleted                           | The port ring is input now                               |
| `QubitLayout::align_routed_paths` and its curve-fitting helper block  | `IFinalizer`                      | Becomes a stage in its own right                         |
| `QubitLayout::Coupler`, `::Bridge`                                    | `MQT::ScpdDesign`                 | The exact physical geometry; the router keeps only cells |
| `QubitLayout::verify_min_clearance`, `MinClearanceReport`             | `MQT::ScpdDrc`                    | Rule 1, layout-space view. One rule, not two checkers    |
| `FinalGrid::verify_min_clearance` and its exemption logic             | `MQT::ScpdDrc`                    | Rule 1, cell-space view. The exemptions are the value    |
| `FinalGrid::verify_crossing_orthogonality`                            | `MQT::ScpdDrc`                    | Rule 2. Keep the router's own test, not a re-derivation  |
| `QubitLayout::compute_qor_report`, `QorReport`                        | `MQT::ScpdIO`                     | Becomes the metrics schema; `report_qor` printing is not |
| `CapacityGrid` — rasterize, EDT, watershed                            | `MQT::ScpdGrid`                   | Genuine algorithms, port them                            |
| `CapacityGrid` — chains, budgets, routing requests                    | `ICapacityPlanner`                |                                                          |
| `CapacityGrid::svg_overlay`                                           | deleted                           |                                                          |
| `CapacityGrid::solve_capacity_graph`                                  | deleted                           | Dead, and the only reason Gurobi reaches this file       |
| `CapacityGrid::compute_capacity_chain*` non-optimized pair            | deleted                           | Superseded by the `_optimized` twins                     |
| `OrderedAssignmentGraph`                                              | `IAssigner`                       | Precompute the geometric inputs first                    |
| `OrderedAssignmentGraph::NodeKind`                                    | `UnassignedRole`                  | Stops being a second vocabulary                          |
| `OrderedAssignmentGraph::compute_min_overlap_assignment`              | deleted                           | Dead reference formulation                               |
| `OrderedAssignmentGraph::cycle_svg`, `write_cycle_svg`                | deleted                           | The ring view is not carried over                        |
| `HananGrid`                                                           | `IGlobalRouter`                   | Smallest and most reusable class in the prototype        |
| `QubitLayoutOptimizer`                                                | `IGlobalRouter` + `MQT::ScpdMilp` | Model building splits from solver access                 |
| `DetailedGrid`                                                        | `IDetailRouter`                   |                                                          |
| `FinalGrid` — the four live rip-up-and-reroute drivers                | `IFinalRouter`                    | 4,969 lines. Collapse into one parameterized driver      |
| `FinalGrid::run_optimized_cpw_coupler_insertion`                      | `IFinalRouter`                    | 2,283 lines of genuine algorithm. Port it as it is       |
| `FinalGrid::CPWCoupler`, `::CPWBridge`                                | `IFinalRouter`                    | Rasterized footprint only, derived via `cells_for`       |
| `FinalGrid` — dead methods                                            | deleted                           | Listed below                                             |
| `dubin_router_opt.hpp`                                                | `MQT::ScpdRouting`                | Port this one                                            |
| `dubin_router.hpp`                                                    | deleted                           |                                                          |
| `move_primitives.hpp` lines 1074–2097                                 | `MQT::ScpdRouting`                | The live class only                                      |
| `src/simple_*q_layout.cpp`, ten files                                 | deleted                           | Replaced by the CLI and one `config.toml` per chip       |
| `src/simple_outing.cpp`                                               | `test/routing/`                   | Becomes a GoogleTest                                     |
| `mini_project/`, `lib/`                                               | deleted                           | Unrelated demo, and a stale build tree                   |

## Dead code inventory

Confirmed zero call sites, or explicitly withdrawn. Do not port any of it.

- `FinalGrid`, the serial trio: `run_final_routing` (718 lines),
  `run_final_routing_feedline` (1,378) and `run_final_routing_feedline_choices`
  (603). A grep shows three call sites for the middle one — all of them inside
  `run_final_routing_feedline_choices`, which itself has none, so the trio is
  dead transitively rather than obviously.
- `FinalGrid`, otherwise: `run_cpw_coupler_insertion` (59),
  `run_feedline_routing` (689), `run_bridge_insertion` (261),
  `run_bridge_smoothing` (616), `finalize_coupler_positions` (750), `debug_svg`
  and `debug_proxy_svg` (35). 2,410 lines with no call sites at all.
- `FinalGrid::run_final_routing_resonator_refinement_parallel` (1,139 lines) —
  **a failed experiment, not dead code.** It is fully implemented and reachable,
  and it is disabled upstream (`ref_res_refinement_rounds = 0`) because the
  results did not hold up. Do not port it, and do not port its four `ref_res_*`
  parameters or the hotspot-repulsor block that feeds it.
- `CapacityGrid`: `solve_capacity_graph`, `computeManhattanDT`,
  `rasterizeVoronoi_new`, `computePreciseEDTVoronoi`, and the non-optimized
  chain pair.
- `OrderedAssignmentGraph`: `compute_min_overlap_assignment`,
  `add_circular_consecutive_constraint`.
- `QubitLayout`: `export_netlist_to_svg`.
- `QubitLayoutOptimizer`: seven `GRBVar` member maps that the solver never
  touches; the real model builds its variables in locals.

## The port ring subsystem is deleted, not ported

Because `all_outer` and `fixed_outer` are configuration
([decision 0020](decisions/0020-legacy-routing-config-as-input.md)), everything
that existed to derive them goes: `ordered_qubit_port_labels`,
`qubit_ports_ordered_clockwise` (the qubit→coupler→qubit Hamiltonian walk),
`boundary_arc_length`, `project_param_on_segment`, `is_key_qubit_port_label`,
`parse_coupler_component_name`.

This is worth stating separately because it is also the *only* consumer of a
coupler's qubit pair. Delete it and nothing in the pipeline needs to know that
`Coupler13_14` joins `Qb13` and `Qb14`, which is what makes dropping the entity
model safe.

One further label parser goes with it. `FinalGrid::verify_min_clearance` decides
whether two wires share a component by splitting a port label at its trailing
`.port<digits>`, and carries a documented special case for the synthetic
launcher labels built in `CapacityGrid.hpp` — `CouplerLauncher1_Chip.port25zero`
embeds *another* port's name, so splitting it would collapse every launcher on
one pad into a single bogus component. Under the role model that whole branch is
a field comparison, so rule 1 arrives in `MQT::ScpdDrc` shorter than it is here.

`ordered_launcher_port_labels` — a plain clockwise walk over launcher ports — is
kept and ported. Configuration may override it with an explicit list.

## What the prototype already fixed

Do not "improve" these during the port; reproduce them.

**The three grids agree on the clearance.** `CON_MIN_WIRE_DIST` is 185.0 in
`CapacityGrid`, `DetailedGrid` and `FinalGrid` alike. Earlier drafts of this
document claimed 185 / 70 / 185; that is no longer true and the argument built
on it was wrong.

**The detail grid already derives its blockade radius.**
`detailed_cross_boundary_routing` computes
`J = ceil(min_wire_dist · px_per_unit) − 1` and ignores `params.J`. The literal
`params.J = 6` survives at exactly one site, `DetailedGrid.cpp:1769` in
`run_inner_routing_requests`. That one site is the inconsistency the port fixes,
using the identical expression.

**The routing penalties are grid-normalized.** The three penalties that shipped
as per-layout hand-tuned values — bend 8,000 to 30,000, wire proximity 4 to 15 —
are now derived from `norm · (F_WIDTH_ + F_HEIGHT_)` with one triple,
`2.5 / 0.00125 / 0.00033`, valid on every benchmark. Port the normalized form.
The absolute fields exist only as the mechanism.

**`align_routed_paths` is a real stage.** It fits each wire to analytic form
from the router's own bend/straight bookkeeping rather than re-deriving corners
from the sampled polyline, and it instantiates couplers at true physical size
and bridges at every feedline crossing. This is the specification for
`IFinalizer`.

**QoR and clearance reporting are already structured.** `QorReport`,
`MinClearanceReport`, `ClearanceViolation` and `CrossingOrthogonalityReport` are
value types with documented fields. Take them as the metrics and `DrcReport`
schemas; delete only the `printf`-style rendering.

## What the prototype does not check

Six of DRCPolice's eight rules have no prototype implementation at all:
`wire-loop`, `obstacle-clearance`, `component-overlap`, `min-straight-length`,
`resonator-length` and `min-bend-radius`. Only wire clearance and crossing
orthogonality exist.

Two of those gaps are worth stating precisely, because the prototype looks like
it covers them and does not:

**Obstacle clearance is marked, never verified.** Every routing stage stamps a
`CON_MIN_OBSTACLE_DIST` keepout around each polygon before searching, so a wire
cannot be routed into one. Nothing re-checks the committed result — and the
committed result is not always what the search produced, which is precisely the
argument `verify_crossing_orthogonality` was written to make about feedlines.

**`lengthSatisfied` is a stub, not a check.** It is assigned `false` at all
eight sites that write it and is never computed. The prototype's own KLayout
README records that it reads `False` for all 92 nets of the 17Q run and is
therefore useless as a highlight, and `klayout/fridgecad/extract.py` documents
that it deliberately ignores the field and substitutes a median-deviation
heuristic instead. Do not port the field; implement the rule.

## Traps

Things that will cost a day if discovered late.

**The two routers are a fork, not a refactor.** They share about 70
identically-named methods. The comments record that a `route_orthogonal`
shadowing bug was fixed in the copy and deliberately left in the original. Port
`_opt`; read the other only to understand intent.

**Coupler placement and feedline routing are one fixpoint.**
`run_final_routing_feedline_choices_parallel` re-orients an already-placed
coupler, selecting from the option set the insertion pass stored, when that is
what lets a chain route. Splitting the stage into "place couplers, then route
feedlines" loses the mechanism that reaches zero failures.

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

**Hard-coded paths and labels are compiled into the library.** Several drivers
load their configuration from absolute paths into another machine's checkout,
and every driver passes a Gurobi licence path that does not resolve. Several
per-benchmark debug probes survive as live branches, including one that tests
for a 4-qubit-era label inside a 69-qubit code path.

**`HananGrid::is_point_in_polygon` silently skips geometry.** It ignores the
first two polygons, assuming they are the chip outline, and any polygon with
fewer than ten vertices. On the 69-qubit chip the first polygon has four
vertices. Understand what obstacle testing actually covered before treating the
prototype's results as ground truth.

**Configuration is spread across four mechanisms**: two parameter structs — some
of whose fields are `const`, making them non-assignable — **72** `getenv` sites,
roughly fifteen call-site constants, and per-benchmark hand tuning across ten
driver programs.

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

Per-benchmark assignment inputs, which become `config.toml` values:

| Chip | `launcher_target` | `feedline_terminations` | `max_feedline_utilization` |
| ---- | ----------------: | ----------------------: | -------------------------: |
| 4Q   |                 2 |                       0 |                          4 |
| 9Q   |                 3 |                       1 |                          5 |
| 17Q  |                 7 |                       1 |                          5 |
| 21Q  |                10 |                       0 |                          5 |
| 33Q  |                14 |                       0 |                          5 |
| 45Q  |                15 |                       1 |                          6 |
| 57Q  |                18 |                       0 |                          7 |
| 69Q  |                24 |                       0 |                          6 |

Per-stage timings and the full eight-benchmark table are in the prototype's
`README.md`.
