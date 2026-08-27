# 0007 — What is deleted rather than ported

- **Status:** Accepted
- **Date:** 2026-08-27

## Context

The prototype is roughly 64,000 lines, of which about 15,000 to 18,000 is real
algorithmic content. Porting everything would carry the duplication forward.

## Decision

Delete outright, roughly 40,000 lines.

| Item                                                |  Lines | Replacement                                    |
| --------------------------------------------------- | -----: | ---------------------------------------------- |
| `dubin_router.hpp`, keeping the optimized twin      |  6,883 | One router behind the routing interfaces       |
| Five of six `run_final_routing*` variants           | ~6,500 | One parameterized driver                       |
| Dead methods in `FinalGrid.cpp`                     | ~5,250 | Nothing                                        |
| Six SVG renderers                                   | ~2,150 | KLayout, plus `plot.py` for debug views        |
| Ten `simple_<N>q_layout.cpp` drivers                | ~2,600 | The CLI and per-chip recipes                   |
| Duplicate capacity-chain computation                | ~1,100 | The `_optimized` twin only                     |
| Commented-out class copy in `move_primitives.hpp`   |  1,096 | Nothing                                        |
| Dead Gurobi in `CapacityGrid.cpp`                   |    140 | Also removes Gurobi from that translation unit |
| `mini_project/`, `lib/`, committed logs and results | ~18 MB | Nothing                                        |

Each deletion has a named replacement or is confirmed to have zero call sites.
The full inventory is in [porting notes](../porting-notes.md).

## Alternatives considered

**Port everything and clean up later.** Rejected: the duplication is the problem
being solved, and "later" does not arrive.

**Keep the SVG export for debugging.** Rejected as written — six unrelated
renderers with inline colour literals and no shared transform. The need is real,
so it is met by a small Python plotter over the geometry IR plus KLayout for
publication views.

## Consequences

- The console quality report and the regular-expression log scraping in the
  benchmark harness both go. Metrics become structured output.
- Deleting the unoptimized router removes the only consumer of some shared
  namespace-scope types; they move into the surviving router.
