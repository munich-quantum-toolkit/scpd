# 0010 — Benchmark chips are generated, not committed

- **Status:** Accepted
- **Date:** 2026-08-27

## Context

Eight benchmark chips form the regression baseline. In the prototype these are
large: the 69-qubit configuration is 34 MB, holding 13,316 fully expanded
obstacle polygons.

## Decision

Do not commit the expanded chip descriptions. Ship a lattice generator and a
short recipe per benchmark, and reconstruct each chip on demand.

```bash
mqt-scpd gen benchmarks/recipes/69q.toml -o 69q.json
```

## Alternatives considered

**Commit all eight.** Simplest and fully reproducible offline. Was rejected on
size — though note that the instance-based schema in
[decision 0016](0016-artifact-formats.md) shrinks these to tens of kilobytes, so
this alternative is no longer costly and the two approaches now coexist
comfortably.

**Small chips in-repository, large ones fetched.** Adds fetch logic and a
network dependency in continuous integration for little gain.

## Consequences

- The repository stays small and new topologies are cheap to define.
- **The baseline depends on the generator staying stable.** This is the main
  risk of the decision. Mitigated by converting the prototype's configurations
  first and committing the converted chips for the four smaller benchmarks as
  reference inputs, then validating the generator against them.
