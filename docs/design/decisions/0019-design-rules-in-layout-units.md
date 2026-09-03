# 0019 — Design rules are physical; grids convert

- **Status:** Accepted
- **Date:** 2026-09-01

## Context

The prototype's three grid classes each declare their own clearance constants.
They agree — `CON_MIN_WIRE_DIST` is 185.0 in all three — but the value actually
enforced during final routing is written as `outer_min_dist_wires = 19`, a cell
count, with nothing recording where 19 came from.

It came from 185. The final grid's cell size is 9.91–10.00 layout units on every
benchmark, so `ceil(185 / cell)` is 18.5–18.7 → 19 on all eight. The literal was
correct; nothing in the code said why, so nothing could have caught it drifting.

The same conversion is done correctly in one place and not in another: the
detail router computes its blockade radius from `CON_MIN_WIRE_DIST` in
`detailed_cross_boundary_routing`, and uses a literal `6` in
`run_inner_routing_requests`.

Two further constants are the same rule under two names. `CON_PORT_EXTENSION`
(100.0) is the straight run out of a port; the minimum straight run between two
curvature changes is the same physical requirement at the same value.

## Decision

Every design rule is a **length in layout units**, held once in `DesignRules`. A
stage that works in grid space obtains a cell count through one function, and
only through that function:

```cpp
/// Cells on `g` that span at least `distance` layout units.
[[nodiscard]] uint32_t cells_for(double distance, const GridMetrics& g);
```

No stage declares a clearance constant, and no clearance appears as a literal
cell count anywhere.

Three changes to the rule set itself:

- `port_extension` is **deleted**; its call sites read `min_straight_length`.
- `min_straight_length` is added, at 100.0.
- `max_feedline_utilization` and `feedline_terminations` are added, because both
  are bounds on what the chip can physically carry rather than solver knobs.

The rules named `min_straight_length` and the router's
`meander_insertion_params::min_straight_length` are **not** fused. The latter is
a different quantity — the straight a meander needs to fit — and keeps its own
name so the coincidence of wording does not become a coincidence of value.

## Alternatives considered

**Keep the literals and document them.** Zero behavioural risk. Rejected: it
preserves exactly the situation where `19` is right for reasons no one can
check, and leaves the one genuinely inconsistent site inconsistent.

**Convert once at load into a per-stage cell-count struct.** Avoids repeated
conversion. Rejected as premature: the conversion is one division, it happens a
handful of times per run, and a struct of pre-converted counts is one more thing
that can be stale.

**Rule per grid, authored separately.** What the prototype effectively has.
Rejected on the argument this decision exists to make.

## Consequences

- `cells_for` reproduces every literal it replaces: 19 for wire clearance on all
  eight benchmarks, 20 × 3 cells for the coupler footprint (which is the
  physical 200 × 26 at the final cell size), 6 × 6 for the bridge. That
  agreement is the evidence the derivation is right rather than plausible.
- **Two values become chip-dependent and therefore change behaviour.** The
  detail-grid blockade goes from a fixed 6 to 4–9, and the straight-start stub
  from a fixed 9 to 10–11. This is the point: one literal cannot be correct on
  eight differently-scaled grids. Both are validated against the zero-fail
  baseline in phase 4.
- If a chip regresses, it gets a documented per-chip override. Reverting to the
  literal would restore the defect.
- Coupler and bridge dimensions become configurable, and their rasterized
  footprints stop being independently authored numbers that happened to agree
  with the physical ones.
