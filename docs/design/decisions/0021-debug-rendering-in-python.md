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

| `--stage`  | Reads            | Renders                                                    |
| ---------- | ---------------- | ---------------------------------------------------------- |
| `layout`   | `00-chip.json`   | obstacles, ports coloured by `UnassignedRole`              |
| `capacity` | `01-capacity.fb` | + partitions, bottlenecks, budgets, routed chains          |
| `assign`   | `02-assign.fb`   | + each connection as a chord, source and target role apart |
| `global`   | `03-global.fb`   | + the Hanan lattice and the selected inner-circuit edges   |
| `detail`   | `04-detail.fb`   | + pixel paths                                              |
| `final`    | `05-final.fb`    | + Dubins paths, couplers, bridges                          |
| `aligned`  | `06-geometry.fb` | fitted analytic wires, real coupler/bridge footprints      |

`mqt-scpd render --stage <name>` takes the same names and writes the same
content as GDSII or OASIS. The chip artwork keeps the layers it has; every
planning element goes on a named layer of its own, so KLayout can switch the
overlay off. `render` with no `--stage` renders the unrouted chip, unchanged.

Two constraints are part of the decision, not implementation detail:

- **Pixel fields are one downsampled raster**, never one element per pixel. The
  budget is 10 MB per snapshot on every benchmark, and it is an acceptance
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

**Render through KLayout instead**, and drop the SVG path. Rejected as
insufficient on its own: KLayout can only show what has been exported, and the
export adapter is a stage of the pipeline rather than a view onto it. SVG stays
the primary debug view for that reason.

> **Amended 2026-09-08.** The two are no longer exclusive. `render` gained the
> same `--stage` names as `plot`, so the planning stages can also be written to
> GDS or OASIS, each element on a named layer beside the chip artwork. The
> rejection above stands for what it actually argued — SVG is not replaced, and
> the core still writes neither format — but "the intermediate stages are not
> GDS geometry at all" was too strong. A partition is a polygon and a capacity
> chain is a path; refusing to emit them cost the one view in which they can be
> measured against the artwork they have to respect, which is KLayout's. What
> genuinely does not survive the trip is the pixel field, and it is rendered as
> the partition polygons it induces rather than as pixels. The planning layers
> carry no manufacturing intent and are not part of any exported design.

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
