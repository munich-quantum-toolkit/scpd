# 0027 — A port's component is declared, not parsed

- **Status:** Accepted
- **Date:** 2026-09-08
- **Amends:** [0018](0018-port-roles-unassigned-and-assigned.md)

## Context

[Decision 0018](0018-port-roles-unassigned-and-assigned.md) removed the entity
model that the prototype recovered from label strings, on the argument that
"the one relationship those string parsers existed to recover, a coupler's
qubit pair, is needed by nothing that survives".

The qubit pair is indeed needed by nothing. Grouping a component's ports is
not the same relationship, and the Global stage cannot be written without it:

- An inner wire crosses a coupler's artwork and leaves through the port on the
  opposite side. Which two ports those are is a property of the component they
  belong to.
- What the inner circuit has to reach is a qubit's own routing ports. A qubit
  port and a coupler port both classify as `Conventional`, so the role does not
  separate them.

The prototype answers both by parsing: it splits a label at its dot, tests the
prefix for `Coupler`, and hard-codes that ports 1 and 2 bridge and so do 3 and
4. That is exactly what decision 0018 exists to end.

## Decision

A port carries the **component** it belongs to, as a string, filled at load
from a configured pattern with one capture group:

```toml
[ports.patterns]
component = '^([^.]+)\.port\d+$'
```

The pattern is optional and is written per chip beside the role patterns,
because which part of a label names the component is a property of the chip and
not of the tool. `mqt-scpd doctor` prints the resulting grouping, so a pattern
that captures nothing is visible in a second rather than as a global stage with
no bridges.

Everything else follows from the grouping and from geometry, with no further
name test:

- A component that carries a `Resonator` port is a **qubit**; every other
  component with routable ports is a **coupler**.
- A coupler's ports **bridge** in pairs whose orientations are opposite. Which
  of a pair the outer ring carries is what makes that one the ring's side of
  the bridge.

## Alternatives considered

**Parse the label in the Global stage.** Rejected on decision 0018: it is the
same parser in a new place, and it would carry the same special case for the
synthetic launcher labels.

**Derive the grouping from geometry alone**, by clustering ports that sit on
one piece of artwork. Rejected: it needs a distance threshold that no rule
supplies, and it would fail exactly where two components touch, which is
everywhere on these chips.

**Add the full entity model back**, with a component table and typed
references. Rejected as more than any stage asks for. The grouping is a name;
nothing needs a component to be an object.

## Consequences

- Decision 0018's rule is unchanged in substance: a label is looked up, never
  parsed. What changed is that the configuration now declares one more thing
  about a label, in the same way it already declared the roles.
- A chip whose planning stages need no grouping does not have to state one; the
  pattern is optional and an absent one leaves every component empty.
- The eight benchmark configurations all use the same expression, and each
  writes it out, exactly as each writes out its own `launcher` pattern even
  though all eight agree on that too.

> **Amended 2026-09-09.** Which two ports of a component pair is declared as
> well, by one rule per crossing, and no longer measured from their
> orientations and the distance between them. The grouping this record adds is
> unchanged and is what a rule pairs within. See
> [decision 0030](0030-bridge-pairs-are-declared.md).
