# 0021 — Per-stage rendering reads artifacts, in Python

- **Status:** Accepted
- **Date:** 2026-09-01

## Context

The prototype's most useful development instrument is its per-stage SVG
snapshot: `layout.svg`, `capacity_*.svg`, `detyailed_routed_*.svg`,
`final_routed_*.svg`, `aligned_*.svg`. Watching those change is how a routing
change is judged before the benchmark table is available.

The port needs the same instrument, and needs it early — phases 2 through 4
build the grid, the routers and the final stage, and without a picture their
only feedback is a fail count.

But the prototype pays a real price for it. 1,476 lines of hand-rolled SVG are
spread across `QubitLayout`, `CapacityGrid`, `DetailedGrid`, `FinalGrid` and
`OrderedAssignmentGraph`; `export_to_svg` exists in five overloads whose
signatures differ only by which grid pointer they take; and the pixel-field
views stamp one element per pixel, so the 4-qubit capacity snapshot is
**17.8 MB**. `ARCHITECTURE.md` lists "no SVG writer of our own" among the things
deliberately not built.

## Decision

`mqt-scpd plot run/ --stage <name>` renders SVG
**in Python, from the run directory's artifacts**, and nothing in the C++ core
writes SVG.

| `--stage`  | Reads            | Renders                                               |
| ---------- | ---------------- | ----------------------------------------------------- |
| `layout`   | `00-chip.json`   | obstacles, ports coloured by `UnassignedRole`         |
| `capacity` | `01-capacity.fb` | + partitions, bottlenecks, budgets, routed chains     |
| `detail`   | `04-detail.fb`   | + pixel paths                                         |
| `final`    | `05-final.fb`    | + Dubins paths, couplers, bridges                     |
| `aligned`  | `06-geometry.fb` | fitted analytic wires, real coupler/bridge footprints |

Two constraints are part of the decision, not implementation detail:

- **Pixel fields are one downsampled raster**, never one element per pixel. The
  budget is 2 MB per snapshot on every benchmark, and it is an acceptance
  criterion for phase 1.
- **`plot` reads artifacts only.** It has no access to pipeline internals, which
  is what lets it run against a half-finished run directory.

The prototype's assignment-ring view is not carried over;
`OrderedAssignmentGraph::cycle_svg` is not ported.

## Alternatives considered

**Port the C++ SVG writers.** A C++-only build would produce the same snapshots
with no Python. Rejected: it re-adds hand-rolled SVG to the core, contradicts a
stated non-goal, and reproduces the five-overload shape whose only variation is
which grid it was handed.

**Defer plotting to phase 5, with the rest of the tooling.** Rejected on
sequencing. The phases that most need a picture are 2 through 4, and building it
after them means building it after it would have helped.

**Render through KLayout instead.** KLayout already renders the exported
geometry, and does it well. Rejected as insufficient: it can only show what has
been exported, and the intermediate stages — partitions, budgets, pixel paths —
are not GDS geometry at all. KLayout remains the renderer for the final layout.

## Consequences

- "No SVG writer of our own" holds in the place that matters — the core — and is
  now specific rather than absolute.
- Debug rendering costs nothing at routing time and cannot regress routing
  behaviour, because it is not in the routing path.
- A snapshot can be regenerated from an old run directory without re-running the
  pipeline, which the prototype cannot do.
- Rendering requires Python. A pure C++ consumer of the library gets artifacts
  and metrics, not pictures. That is consistent with
  [decision 0002](0002-cli-is-the-product.md).
