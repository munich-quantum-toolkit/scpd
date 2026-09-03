# 0015 — Keep eight-way headings, fix memory during the port

- **Status:** Accepted
- **Date:** 2026-08-27

## Context

The prototype hardwires eight-way octilinear routing. Headings are `uint8`
values `0..7`, packed into the A* state index and into flat primitive tables as
`(ang << 10) | path_id`, which caps path identifiers at 1024.

It also allocates densely. On the largest benchmark the capacity grid holds
around 500 MB, each router context up to 1.87 GB, and four run concurrently for
roughly 7.5 GB.

## Decision

Keep eight-way headings for the first release, but replace scattered literals
with a named constant and route grid access through one abstraction, so the
assumption is greppable rather than implicit.

Fix the memory model during the port: one shared read-only obstacle grid across
threads, per-thread scratch only, bit-packed masks where a byte per cell was
used. Target about 2 GB on the largest benchmark.

## Alternatives considered

**Port both as they are.** Honours the port-first rule most literally, but
leaves memory as the cap on thread count, which blocks the phase-6 performance
work.

**Abstract the grid over angular resolution and storage**, so sixteen-way or
sparse representation becomes a later swap. Most future-proof, but templating
the hot path risks the performance the benchmark bar depends on, and no research
requirement for it exists yet.

## Consequences

- Sixteen-way routing remains out of reach without touching the router. Accepted
  knowingly; the constant makes the blast radius visible.
- Memory work is sequenced before the threading work that depends on it.
- The router's documented dangling-reference defect — `primitives` is a
  reference member bound to a caller stack local — is fixed here rather than
  worked around in callers as the prototype does.
