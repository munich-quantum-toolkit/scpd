# 0009 — Standalone, no ecosystem coupling in the first release

- **Status:** Accepted
- **Date:** 2026-08-27

## Context

MQT Core ships QDMI device descriptions for superconducting devices. It is
natural to ask whether MQT SCPD should consume them.

Those descriptions are *logical*: qubits, coupling maps, fidelities. This tool's
input is *physical*: polygons, port positions, orientations, design rules. They
describe different things about the same chip.

## Decision

MQT SCPD owns its own physical chip schema and takes no dependency on MQT Core
or QDMI in the first release.

## Alternatives considered

**Depend on MQT Core and reuse its device types.** Tightest integration, but it
couples build and release cadence and bends a logical type toward a physical use
it was not designed for.

**Ship a bridge alongside the standalone schema**, importing a coupling map and
generating a placed chip through a lattice generator. Genuinely useful and
architecturally clean. Deferred rather than rejected: it is additive and can
arrive whenever it is wanted.

## Consequences

- No cross-repository version constraints.
- A QDMI bridge needs a lattice generator to turn a coupling map into placed
  geometry, and the first release has none —
  [decision 0020](0020-legacy-routing-config-as-input.md) drops it. So the
  bridge is further away than this decision originally assumed. It is not
  foreclosed: the generator and the bridge become one piece of follow-up work
  rather than two, once the routing baseline exists to validate a generator
  against.
