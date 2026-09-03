# 0024 — Wire loop is an active rule

- **Status:** Accepted
- **Date:** 2026-09-02
- **Amends:** [0022](0022-drc-in-the-core.md)

## Context

Decision 0022 split DRCPolice's eight rules four and four. Wire clearance,
feedline orthogonality, obstacle clearance and resonator length were active;
wire loops, component overlap, minimum straight length and minimum bend radius
were advisory. The argument for the advisory half was maturity: six of the eight
rules were new code with no measured behaviour on any benchmark, and declaring
them enforced before anyone had seen what they report would either block the
port on false positives or get them switched off wholesale.

That argument no longer applies to wire loops. Commit `8a188ec`, dated
2026-09-01, added `path_self_intersection.hpp` to the prototype and wired it
into two places: `dubin_router_opt`, which now refuses to commit a self-crossing
path, and `FinalGrid::verify_no_path_loops`, which runs after all four routing
passes. The rule has measured behaviour on all eight benchmarks, and the
prototype reaches zero unresolved wires with it enabled.

The defect it catches is not cosmetic. `dubin_router_opt` searches over
`(cell, heading)` states, so a route may legally return to a cell it already
occupied on a different octant. In copper that is a short. Nothing else can see
it: `verify_min_clearance` only ever compares two different wires.

## Decision

`wire-loop` joins the active set. DRCPolice ships **five active** rules — wire
clearance, feedline orthogonality, wire loop, obstacle clearance and resonator
length — and **three advisory** — component overlap, minimum straight length and
minimum bend radius.

It is ported as one implementation with two callers, matching upstream:
`MQT::ScpdRouting` uses it to reject a candidate path, and `MQT::ScpdDrc` uses
it to check a committed one. Both detected shapes are ported — a repeated cell,
and two diagonal steps crossing inside one 2×2 block where all four cells stay
distinct — as is the spur window that suppresses the router's state re-emission
artefact.

Phase 4 gains it as an exit criterion.

## Alternatives considered

**Keep it advisory and port it in phase 4 anyway.** Safe, and it would still get
the implementation into the tree on time. Rejected: an advisory rule is a
promise to measure something nobody has measured, and this one has been measured
upstream across eight benchmarks. Leaving it advisory would describe the port as
less certain than the prototype it is porting.

**Two implementations, one per module.** Rejected for the reason upstream gives
in the file's own header: the router and the checker must not be able to
disagree about what a loop is.

## Consequences

- Phase 4's exit criterion becomes five rules rather than four, and phase 5's
  measurement work drops to three.
- `path_self_intersection.hpp` is linked by both `MQT::ScpdRouting` and
  `MQT::ScpdDrc`. The dependency graph already permits it: `drc` depends on
  `grid`, `design` and `geometry`, and `routing` depends on `grid`, so the
  shared code sits below both.
- The `[drc]` table loses `drc.rules.wire-loop`. An active rule is not
  switchable.
- Decision 0022's count of "two of the eight rules that matter, implemented
  twice over" is now three, implemented once each in the port.
