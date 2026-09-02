# 0004 — Planar now, layer axis later

- **Status:** Accepted
- **Date:** 2026-08-27

## Context

Flip-chip and multi-layer architectures are a stated future direction. The
prototype is planar throughout: there is no layer concept, and air bridges are
modelled as two-dimensional markers with an octant.

## Decision

Keep the data model two-dimensional for the first release. Isolate coordinates
and geometry behind named types so that a layer axis can be added later without
rewriting algorithm code.

Do not add a layer field, a via model, or inter-layer edges in the routing graph
now.

## Alternatives considered

**A full layer stack in the data model from the start**, with every shape
carrying a layer and vias as first-class entities. Attractive because it maps
onto GDSII layer and datatype directly, and retrofitting is painful. Rejected
because no algorithm would use it in the first release, and the minimalism rules
are explicit that flexibility without a current use does not get built.

**A three-dimensional routing graph.** Rejected: over-engineering ahead of the
research.

## Consequences

- Adding layers later means touching the geometry types and the router state,
  which is real work, accepted knowingly.
- The named coordinate types make the assumption visible rather than implicit,
  which is the part that actually reduces the retrofit cost.
