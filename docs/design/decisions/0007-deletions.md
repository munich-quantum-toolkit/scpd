# 0007 — What is deleted rather than ported

- **Status:** Accepted
- **Date:** 2026-08-27

## Context

The prototype is roughly 68,000 lines, of which about 15,000 to 18,000 is real
algorithmic content. Porting everything would carry the duplication forward.

## Decision

Delete outright. The table below accounts for about 19,600 lines; a further
~4,900 are absorbed when the four routing drivers become one. It is not an
exhaustive inventory — commented-out history is scattered more widely than any
table can usefully list — but every row is measured, not estimated.

| Item                                                |  Lines | Replacement                                                           |
| --------------------------------------------------- | -----: | --------------------------------------------------------------------- |
| `dubin_router.hpp`, keeping the optimized twin      |  6,949 | One router behind the routing interfaces                              |
| The serial `run_final_routing*` trio                |  2,699 | The parallel drivers, collapsed into one                              |
| The four live rip-up-and-reroute drivers, merged    |  4,969 | One parameterized driver                                              |
| The resonator-refinement pass, a failed experiment  |  1,139 | Nothing. It is disabled upstream                                      |
| Other dead methods in `FinalGrid.cpp`               |  2,410 | Nothing                                                               |
| Hand-rolled SVG across five classes                 |  1,476 | KLayout, plus `plot.py` for debug views                               |
| The clockwise port-ordering subsystem               |   ~350 | `all_outer` / `fixed_outer` in `config.toml`                          |
| Ten `simple_<N>q_layout.cpp` drivers                | ~2,600 | The CLI and one `config.toml` per chip                                |
| 72 `getenv` sites and their branches                |   ~200 | `config.toml`; `FG_CLEARANCE_DEBUG` alone survives as `--drc-verbose` |
| Component-name parsing in the clearance checker     |    ~40 | A `PortRef` field comparison                                          |
| Duplicate capacity-chain computation                |    532 | The `_optimized` twin only                                            |
| Commented-out class copy in `move_primitives.hpp`   |  1,073 | Nothing                                                               |
| Dead Gurobi in `CapacityGrid.cpp`                   |    140 | Also removes Gurobi from that translation unit                        |
| `mini_project/`, `lib/`, committed logs and results | ~18 MB | Nothing                                                               |

Each deletion has a named replacement or is confirmed to have zero call sites.
The full inventory is in [porting notes](../porting-notes.md).

## Alternatives considered

**Port everything and clean up later.** Rejected: the duplication is the problem
being solved, and "later" does not arrive.

**Keep the SVG export for debugging.** Rejected as written — 1,476 lines spread
over five classes, with inline colour literals and no shared transform, one of
which emits 17.8 MB for a four-qubit chip. The need is real, so it is met by a
Python plotter reading the run's artifacts, plus KLayout for publication views.
See
[decision 0021](0021-debug-rendering-in-python.md).

**Keep the `getenv` overrides for sweeps.** They exist because rebuilding to try
a value is slow, which is a real problem. Rejected anyway: 72 of them, each an
undocumented branch that can silently change a benchmark result. A config file
and a rebuild-free `--set key=value` override serve the same need without hiding
state in the environment.

One of the 72 is kept, as a flag rather than a variable. `FG_CLEARANCE_DEBUG`
lists every wire pair the clearance check *exempted*, with the closest
non-exempt approach each pair still has — the way to confirm that an exemption
is not quietly swallowing a real violation. That is a diagnostic for a rule with
three exemption classes, not a tuning knob, so it becomes `--drc-verbose`.

## Consequences

- The console quality report and the regular-expression log scraping in the
  benchmark harness both go. Metrics become structured output.
- Deleting the unoptimized router removes the only consumer of some shared
  namespace-scope types; they move into the surviving router.
- Deleting the port-ordering subsystem removes the only consumer of a coupler's
  parsed qubit pair, which is what lets the entity model go too
  ([decision 0018](0018-port-roles-unassigned-and-assigned.md)).
