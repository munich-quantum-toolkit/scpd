# Porting notes

A map of the FridgeCAD prototype, for contributors carrying out the rewrite.
**Delete this document once the port is complete.**

Written against the `FridgeCAD-0fails` snapshot at commit `42bccf4`, dated
2026-09-02. The prototype is a moving target: it reaches zero unresolved wires
on all eight benchmarks, and several things earlier drafts of this document
described as defects have since been fixed upstream. Where that has happened it
is called out, because the fixes are the parts most worth carrying over.

The prototype is roughly 69,800 lines of C++. Around 15,000 to 18,000 of that is
real algorithmic content. The rest is duplication, dead code, hand-rolled SVG,
and commented-out history. The single most useful thing to know before starting
is which is which.

Ten `simple_<N>q_layout.cpp` drivers exist. Eight are benchmarks: 4, 9, 17, 21,
33, 45, 57 and 69 qubits. The 28-qubit and 39-qubit drivers are experiments and
are out of scope. This document names them only where they explain a mechanism
the eight benchmarks also use.

## Size ledger

| File                             |  Lines | What it actually is                                                              |
| -------------------------------- | -----: | -------------------------------------------------------------------------------- |
| `FinalGrid.cpp`                  | 23,413 | Four live routing drivers and a coupler optimizer; 6,248 lines dead or withdrawn |
| `dubin_router_opt.hpp`           | 10,532 | The A* that is actually used; ~5,000 lines shared with its twin                  |
| `CapacityGrid.cpp`               |  9,537 | Watershed and distance transform, plus 377 lines of SVG and 532 superseded       |
| `dubin_router.hpp`               |  6,949 | The superseded twin. Not ported                                                  |
| `QubitLayout.cpp`                |  4,709 | Chip model, SVG, JSON, the curve-fitting engine, and the outer-boundary walk     |
| `DetailedGrid.cpp`               |  2,362 | Per-partition A* and wire stitching                                              |
| `verify_port_order_expected.inc` |  2,333 | Generated. The port sequences the benchmarks used to carry as literals           |
| `move_primitives.hpp`            |  2,097 | Lines 1–1073 are an older copy of the same class, fully commented out            |
| `FinalGrid.hpp`                  |  1,499 | Declarations plus the parameter struct, which is where the tuning history lives  |
| `OrderedAssignmentGraph.cpp`     |  1,455 | The ring assignment model, plus 112 lines of SVG and a 202-line dead twin        |
| `QubitLayoutOptimizer.cpp`       |  1,109 | The inner-circuit model. The only place Gurobi is properly encapsulated          |
| `path_self_intersection.hpp`     |    293 | Self-crossing detection. Shared by the router and the end-of-stage check         |
| `verify_port_order.cpp`          |    275 | Checks the derived port sequences against the frozen literals                    |

## Where each prototype class goes

| Prototype                                                            | Destination                         | Notes                                                    |
| -------------------------------------------------------------------- | ----------------------------------- | -------------------------------------------------------- |
| `QubitLayout` — geometry, ports                                      | `MQT::ScpdDesign`                   | Labels resolved to indices once; roles from patterns     |
| `QubitLayout` — JSON load                                            | `MQT::ScpdIO`                       | Same format, validated. `nets`/`sampleSpacing` rejected  |
| `QubitLayout` — SVG export, five overloads                           | deleted                             | KLayout renders; `plot.py` for debug views               |
| `QubitLayout::ordered_qubit_port_labels` and its helpers (see below) | deleted                             | The qubit-to-coupler Hamiltonian walk. Superseded        |
| `QubitLayout::outer_boundary_walk` and its two flatteners            | `MQT::ScpdDesign`                   | Port it. Derives the outer port ring from geometry       |
| `QubitLayout::align_routed_paths` and its curve-fitting helper block | `IFinalizer`                        | Becomes a stage in its own right                         |
| `QubitLayout::Coupler`, `::Bridge`                                   | `MQT::ScpdDesign`                   | The exact physical geometry; the router keeps only cells |
| `QubitLayout::verify_min_clearance`, `MinClearanceReport`            | `MQT::ScpdDrc`                      | Rule 1, layout-space view. One rule, not two checkers    |
| `FinalGrid::verify_min_clearance` and its exemption logic            | `MQT::ScpdDrc`                      | Rule 1, cell-space view. The exemptions are the value    |
| `FinalGrid::verify_crossing_orthogonality`                           | `MQT::ScpdDrc`                      | Rule 2. Keep the router's own test, not a re-derivation  |
| `FinalGrid::verify_no_path_loops`, `PathLoopReport`                  | `MQT::ScpdDrc`                      | Rule 3, now active. See decision 0024                    |
| `path_self_intersection.hpp`                                         | `MQT::ScpdRouting` + `MQT::ScpdDrc` | One implementation, two callers. Keep it that way        |
| `QubitLayout::compute_qor_report`, `QorReport`                       | `MQT::ScpdIO`                       | Becomes the metrics schema; `report_qor` printing is not |
| `CapacityGrid` — rasterize, EDT, watershed                           | `MQT::ScpdGrid`                     | Genuine algorithms, port them                            |
| `CapacityGrid` — chains, budgets, routing requests                   | `ICapacityPlanner`                  |                                                          |
| `CapacityGrid::svg_overlay`                                          | deleted                             |                                                          |
| `CapacityGrid::solve_capacity_graph`                                 | deleted                             | Dead, and the only reason Gurobi reaches this file       |
| `CapacityGrid::compute_capacity_chain*` non-optimized pair           | deleted                             | Superseded by the `_optimized` twins                     |
| `OrderedAssignmentGraph`                                             | `IAssigner`                         | Precompute the geometric inputs first                    |
| `OrderedAssignmentGraph::NodeKind`                                   | `UnassignedRole`                    | Stops being a second vocabulary                          |
| `OrderedAssignmentGraph::compute_min_overlap_assignment`             | deleted                             | Dead reference formulation                               |
| `OrderedAssignmentGraph::cycle_svg`, `write_cycle_svg`               | deleted                             | The ring view is not carried over                        |
| `HananGrid`                                                          | `IGlobalRouter`                     | Smallest and most reusable class in the prototype        |
| `QubitLayoutOptimizer`                                               | `IGlobalRouter` + `MQT::ScpdMilp`   | Model building splits from solver access                 |
| `DetailedGrid`                                                       | `IDetailRouter`                     |                                                          |
| `FinalGrid` — the four live rip-up-and-reroute drivers               | `IFinalRouter`                      | 4,969 lines. Collapse into one parameterized driver      |
| `FinalGrid::run_optimized_cpw_coupler_insertion`                     | `IFinalRouter`                      | 2,283 lines of genuine algorithm. Port it as it is       |
| `FinalGrid::CPWCoupler`, `::CPWBridge`                               | `IFinalRouter`                      | Rasterized footprint only, derived via `cells_for`       |
| `FinalGrid` — dead methods                                           | deleted                             | Listed below                                             |
| `dubin_router_opt.hpp`                                               | `MQT::ScpdRouting`                  | Port this one                                            |
| `dubin_router.hpp`                                                   | deleted                             |                                                          |
| `move_primitives.hpp` lines 1074–2097                                | `MQT::ScpdRouting`                  | The live class only                                      |
| `src/simple_*q_layout.cpp`, ten files                                | deleted                             | Eight become one `config.toml` each; 28Q and 39Q do not  |
| `src/verify_port_order.cpp` and its generated `.inc`                 | `test/design/`                      | Becomes the GoogleTest that pins the walk                |
| `scripts/gen_expected_port_orders.py`                                | `test/design/`                      | Regenerates that fixture from a driver's literals        |
| `src/simple_outing.cpp`                                              | `test/routing/`                     | Becomes a GoogleTest                                     |
| `mini_project/`, `lib/`                                              | deleted                             | Unrelated demo, and a stale build tree                   |

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

## The port ring: one subsystem goes, another is ported

Two different mechanisms in the prototype produce an ordered port sequence, and
the port keeps only the second one.

**Deleted.** The clockwise qubit-port ordering that decision 0008 relied on:
`ordered_qubit_port_labels`, `qubit_ports_ordered_clockwise` (the
qubit-to-coupler-to-qubit Hamiltonian walk), `boundary_arc_length`,
`project_param_on_segment`, `is_key_qubit_port_label` and
`parse_coupler_component_name`. Roughly 350 lines.

**Ported.** `outer_boundary_walk` and the two functions that flatten it,
`ordered_outer_port_labels` and `ordered_fixed_outer_port_labels`. Roughly 740
lines, added upstream in commit `42bccf4`. All ten drivers now call them instead
of carrying the sequence as a literal. `ordered_launcher_port_labels`, a plain
clockwise walk over launcher ports, is ported as before.

The walk is a face traversal of the planar graph whose vertices are the qubits
and whose edges are the couplers. It starts at the leftmost-lowest qubit and
repeatedly leaves a qubit through the coupler that keeps the exterior on its
left. It is not a simple ring: where the lattice has a notch, the traversal
re-enters a qubit it has already visited, and every visit contributes its own
ports.

**The walk reads geometry, not names.** A qubit's routing ports are the ones
left after removing its coupler-mating cross. Which of them face outward is
decided per visit, from the angular sector between the direction back to the
previous qubit and the direction on to the next one, swept clockwise. A coupler
contributes every port strictly left of its own mating-end-to-mating-end axis.
The only name test in the whole traversal is a leading `Q` or `C` prefix, which
selects what takes part.

That matters for the data model. The walk never recovers a coupler's qubit pair,
so porting it does not reopen the entity model that
[decision 0018](decisions/0018-port-roles-unassigned-and-assigned.md) removed.
See [decision 0023](decisions/0023-geometric-port-ring-detection.md) for how the
two mechanisms are selected in `config.toml`.

One further label parser goes with the deleted subsystem.
`FinalGrid::verify_min_clearance` decides whether two wires share a component by
splitting a port label at its trailing `.port<digits>`, and carries a documented
special case for the synthetic launcher labels built in `CapacityGrid.hpp` —
`CouplerLauncher1_Chip.port25zero` embeds *another* port's name, so splitting it
would collapse every launcher on one pad into a single bogus component. Under
the role model that whole branch is a field comparison, so rule 1 arrives in
`MQT::ScpdDrc` shorter than it is here.

### Two per-chip facts about the walk

**17Q enters the cycle at `Qb15`.** The other seven benchmarks pass no start
component and take the walk's own start. The sequence is a closed cycle, so the
start only rotates it — but that rotation changes the model, because the stage
consumes the sequence in order.

**Drive lines exist only on 28Q.** A link whose two mating ends land on the same
qubit is a drive line, not a coupler. The walk sweeps its free end in beside its
host qubit's own ports, which is the role `Qb*.port1` plays on the other
layouts. The `include_drive_lines` parameter is ported for completeness, but no
in-scope benchmark exercises it.

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

**The outer port ring is derived from geometry.** Commit `42bccf4` replaced the
hand-written sequences in all ten drivers with `outer_boundary_walk`, described
above. Port the walk. Keep the frozen literals as a test fixture, not as input.

**Obstacle clearance is enforced by construction.** `rasterize_obstacles` now
blocks every cell whose world position lies within `CON_MIN_OBSTACLE_DIST` of a
polygon edge, so whatever stays free satisfies the rule for every router, every
rip-up path and every feedline, without any search knowing about it.

Three details decide whether a reimplementation is faithful:

- The distance is measured **exactly**, as a point-to-segment distance in world
  coordinates. It is not a pixel dilation of the finished mask. The area fill
  tests the pixel centre at `(x+0.5, y+0.5)` while paths, ports and launchers go
  through `llround()`, so a dilation would carry that half pixel — 5 world units
  — as a systematic error and would widen the Bresenham seam a second time.
- Polygon 0, the chip outline, is skipped.
- The seam runs in the constructor, before `add_fixed_port_obstacles()`. It
  therefore surrounds the layout polygons only, never the port extensions drawn
  later, or every port exit would be blocked. `fg_port_extension_units()` states
  the extension length once so the two callers cannot disagree.

**Self-intersecting paths are detected and refused.** `dubin_router_opt`
searches over `(cell, heading)` states, so a route may legally return to a cell
it already occupied on a different octant. In copper that is a short, and no
other check can see it: `verify_min_clearance` only ever compares two different
wires. `path_self_intersection.hpp` closes the gap, and the router and
`FinalGrid::verify_no_path_loops` share it, so the two cannot drift apart.

It catches two shapes. A **revisit** is the same cell twice. A
**diagonal crossing** is two diagonal steps that cross inside one 2×2 block
while all four cells stay distinct, detected by keying each diagonal step on its
block and its orientation. The second shape is invisible to the obvious test and
is confirmed on real 69Q geometry: the legs `(3701,4306)->(3702,4305)` and
`(3702,4306)->(3701,4305)` cross while sharing no cell.

Reproduce the suppression exactly. The router emits a state sequence, so every
heading change re-emits the cell it turns on, giving 800 to 1,300 harmless
`A->B->A` micro-backtracks per layout. They are suppressed by a window,
`kPathLoopSpurWindow = 4`, not by collapsing spurs off a stack. A stack collapse
looks right and is wrong: it pops any `A->B->A` and so unwinds a long exact
retrace one cell at a time, removing the defect it is meant to find. The
upstream unit test for this file measures that.

## What the prototype does not check

Five of DRCPolice's eight rules have no prototype implementation at all:
`obstacle-clearance`, `resonator-length`, `component-overlap`,
`min-straight-length` and `min-bend-radius`. Wire clearance, crossing
orthogonality and wire loops exist.

One gap needs stating precisely, because the prototype now looks as though it
covers it and still does not:

**Obstacle clearance is enforced, never verified.** The keepout described above
is a hard constraint on the search, and it is the mechanism to port. Nothing
re-checks the committed result, and the committed result is not always what the
search produced — which is precisely the argument
`verify_crossing_orthogonality` was written to make about feedlines. Port the
enforcement, then write the rule as well.

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

**The walk's entry point reaches the solver.** `all_outer` is a closed cycle, so
a different start component rotates it rather than changing its content — but
that order is consumed in order by `OrderedAssignmentGraph` and
`buildModelInnerCircuit`, so the rotation changes the model. `fixed_outer` is
different: its only consumers, `CapacityGrid::add_fixed_port_obstacles` and the
`FinalGrid` equivalent, funnel it straight into an `unordered_set`, so for that
list a rotation means nothing. `verify_port_order.cpp` checks the first for byte
identity and the second for set identity, and the port must keep that
distinction.

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
for a 4-qubit-era label inside a 69-qubit code path. The generated
`verify_port_order_expected.inc` has the same problem: three of its ten fixtures
name absolute paths into a different checkout.

**`HananGrid::is_point_in_polygon` silently skips geometry.** It ignores the
first two polygons, assuming they are the chip outline, and any polygon with
fewer than ten vertices. On the 69-qubit chip the first polygon has four
vertices. Understand what obstacle testing actually covered before treating the
prototype's results as ground truth.

**Configuration is spread across four mechanisms**: two parameter structs — some
of whose fields are `const`, making them non-assignable — **75** `getenv` sites,
roughly fifteen call-site constants, and per-benchmark hand tuning across ten
driver programs.

## Reference numbers

These are the targets to reproduce. They come from the eight in-scope benchmarks
in `scripts/logs/automated_ports/SUMMARY.log`, measured at commit `42bccf4` with
the port ring derived from geometry — `port_detection = "auto"` in the port's
own terms.

For the 69-qubit chip:

| Metric                    | Value                                |
| ------------------------- | ------------------------------------ |
| Final unresolved wires    | 0                                    |
| Total feedline angle cost | 196                                  |
| Wall-clock runtime        | 366.5 s                              |
| Of which Dubins routing   | 320.0 s, about 87 percent            |
| Assignment objective      | 132.68                               |
| Peak memory               | roughly 7.5 GB, four router contexts |

The previously published figures for the same chip were 456.1 s, angle cost 188
and objective 132.75, measured before the port walk, the obstacle keepout and
the loop protection landed. The runtime improved and the angle cost got worse.
The delta is not attributed to any one of those three changes, which is why
[the roadmap](roadmap.md) records it as a risk rather than a result.

Runtime and angle cost across the eight benchmarks:

| Chip | Final fails | Angle cost | Runtime | Assignment objective |
| ---- | ----------: | ---------: | ------: | -------------------: |
| 4Q   |           0 |         12 |   3.9 s |                 5.02 |
| 9Q   |           0 |         18 |  10.0 s |                15.32 |
| 17Q  |           0 |         36 |  10.1 s |                27.86 |
| 21Q  |           0 |         56 |  28.5 s |                34.10 |
| 33Q  |           0 |         72 |  53.3 s |                59.87 |
| 45Q  |           0 |        114 | 154.3 s |                86.67 |
| 57Q  |           0 |        134 | 152.5 s |               108.82 |
| 69Q  |           0 |        196 | 366.5 s |               132.68 |

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

Per-stage timings and the full table are in the prototype's `README.md` and in
`scripts/logs/automated_ports/SUMMARY.log`.
