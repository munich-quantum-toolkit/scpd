# 0010 — Benchmark chips are generated, not committed

- **Status:** Superseded by [0020](0020-legacy-routing-config-as-input.md)
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

## Why this was superseded

That risk is the reason. [Decision 0020](0020-legacy-routing-config-as-input.md)
drops the generator entirely: the benchmark chips are the prototype's own
inputs, byte for byte, so a quality difference against the prototype is a
difference in the port rather than a difference in the geometry. The 4-qubit and
9-qubit inputs are committed; the larger ones are referenced by path.

A generator remains worth having once the routing baseline is established, at
which point there is something to validate it against.
