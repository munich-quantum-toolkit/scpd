# 0020 — The prototype's routing config is the chip input

- **Status:** Accepted
- **Date:** 2026-09-01
- **Amended by:** [0023](0023-geometric-port-ring-detection.md)

## Context

Decision 0008 split the input into a new versioned chip description and a TOML
routing configuration, with a `convert` subcommand to migrate the prototype's
files. Decision 0010 then chose not to commit the converted chips at all, but to
regenerate them from lattice recipes.

Together those put three pieces of machinery — a new schema, a converter and a
generator — in front of the first routed wire, and made the quality baseline
depend on the generator reproducing benchmark geometry faithfully. That was
recorded as the project's largest risk.

Decision 0008 also asserted that the hand-entered ring of outer ports "is
**not** input; it is derived from geometry by the port-ordering pass", and
called forcing that distinction into the open "much of the value of the split".

## Decision

The chip input is the prototype's `routing_config_*.json`, unchanged. Of its
four top-level keys the loader reads two:

```json
{ "obstacles": [ { "polygon": [[x, y], ...] }, ... ],
  "ports":     { "Qb1.port0": { "center": [x, y], "orientation": 90.0 }, ... } }
```

`sampleSpacing` is unused and `nets` is empty in every benchmark; a file
carrying a non-empty `nets` is **rejected**, rather than silently ignored as the
prototype does.

Everything else moves to `config.toml`: the role patterns, the design rules, the
grid geometry, stage tuning — and **both ordered port sequences**, `all_outer`
and `fixed_outer`, verbatim.

There is no `gen`, no `convert`, and no second chip format. The 4-qubit and
9-qubit inputs (0.8 MB and 2.5 MB) are committed as CI fixtures; the six larger
ones are referenced by path from their config.

The FlatBuffers schemas still define the in-memory model and the `01-`…`06-`
artifacts. They simply no longer parse the chip text, so `ScpdIO` links the
header-only runtime rather than the parser library.

## Alternatives considered

**Decision 0008 and 0010 as written.** The right long-term shape, and the
instance-based schema's size win is real: 34 MB of expanded polygons becomes
tens of kilobytes. Rejected for the first release because it must all land
before anything routes, and because the generator becomes load-bearing for the
quality baseline before there is a baseline to check it against.

**New schema, but converted rather than generated.** Removes the generator risk
and keeps the size win. Rejected as still front-loading a schema and a converter
whose only purpose is to feed stages that do not exist yet. It stays available
as follow-up work once routing quality is established.

**Commit all eight inputs.** ~114 MB in the repository. Rejected on size; the
two smallest are enough for continuous integration.

## Consequences

- **Decision 0008's central consequence is reversed.** The port ring *is* input.
  That is a real loss of principle, bought for a real gain: it deletes the
  entire clockwise-ordering subsystem — the qubit→coupler→qubit Hamiltonian
  walk, boundary arc-length projection, the key-port filter — and with it the
  only consumer of a coupler's parsed qubit pair, which is what makes dropping
  the entity model safe
  ([decision 0018](0018-port-roles-unassigned-and-assigned.md)).
  [Decision 0023](0023-geometric-port-ring-detection.md) softens this to
  *may be* input: a different, purely geometric traversal can derive the ring
  instead, and it recovers no qubit pair either, so the gain above survives
  under both modes.
- The risk changes shape rather than disappearing: `all_outer` is about 330
  hand-maintained entries on the 69-qubit chip. It is a smaller risk than the
  generator, because the sequences are copied verbatim from the prototype's own
  drivers where they were already hand-maintained, and because load-time
  validation checks every label against the chip and the role patterns.
- The routing baseline is directly comparable: the port consumes byte-identical
  inputs to the prototype, so a quality difference is a difference in the port.
- The 69-qubit input stays 34 MB and its 13,316 expanded polygons stay expanded.
  The memory win from a cell library is deferred, not abandoned, and is gated on
  the routing quality baseline existing first.
- Two of the prototype's four JSON keys were ignored on load. Rejecting rather
  than ignoring them is how that stops being a thing the format quietly permits.
