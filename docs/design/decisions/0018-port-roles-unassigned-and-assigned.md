# 0018 — Port roles are declared, in two stages

- **Status:** Accepted
- **Date:** 2026-09-01

## Context

The prototype had no notion of what a port *is*. Semantics were recovered by
string matching at each point of use, and the tests disagreed with each other:
one function accepted any label starting with `Q` (which also matches `Qb`),
another required `Qb` (which does not match the 4-qubit chip's `Q1.port0` at
all), and a third hard-coded which port indices counted as "key" ports. A
coupler's qubit pair was recovered by parsing digits out of `Coupler13_14`.

An earlier plan replaced this with a full entity model: `EntityId`, `Qubit` and
`Coupler` tables, stored `qubitA`/`qubitB` relationships, and a converter to
build them from the prototype's inputs. That removes the same defects, but it
front-loads a schema and a converter before any wire is routed.

Separately, the prototype used two different vocabularies for the same
distinction — `is_resonator` as a `bool` on a routing request, and `NodeKind` on
an assignment graph node — and neither could express a port's role *after* the
assignment was solved.

## Decision

Roles are **declared in configuration**, and there are **two enums** with a
defined transition.

```fbs
/// What a port is, before anything is decided. Set at load, from patterns.
enum UnassignedRole : ubyte { Launcher, Resonator, Conventional }

/// What a port does in the solved design. Set by the Assignment stage.
enum AssignedRole : ubyte {
  FeedlineSource,     FeedlineTarget,
  ResonatorSource,    ResonatorTarget,
  ConventionalSource, ConventionalTarget,
}
```

`UnassignedRole` comes from one regular expression per role in `config.toml`:

```toml
[ports.patterns]
launcher     = '^Chip\.port\d+$'
resonator    = '^Qb?\d+\.port0$'
conventional = '^(Qb?\d+\.port1|Coupler\d+_\d+\.port[0-4])$'
```

Every port must match exactly one. Matching none, or more than one, is a load
error naming the label and the patterns involved.

`AssignedRole` is a property of a connection endpoint, not of a port, and is set
by the Assignment stage. `ResonatorSource` is the constraining case: a resonator
runs from the CPW coupler that taps the feedline to the qubit's readout port, so
its source port does not exist in the input at all and is materialized by
coupler insertion in the Final stage.

No entity model, no stored qubit-coupler relationships, no converter.

## Alternatives considered

**The full entity model.** Strictly better typing, and the right long-term
shape. Rejected for the first release: it requires a new chip schema and a
converter to populate it, and the only relationship it would store — a coupler's
qubit pair — is needed by exactly one subsystem, which
[decision 0020](0020-legacy-routing-config-as-input.md) deletes.

**One combined enum.** Fewer types. Rejected: it cannot represent a port before
the assignment runs without a `Unknown` member that every consumer must handle,
which is the same defect in a new place.

**Keep prefix matching, but in one function.** Cheapest possible change.
Rejected: it leaves the semantics in C++ where a new chip's naming convention
means a code change, and the 4Q/69Q naming split already proves that happens.

## Consequences

- A chip's naming convention is data. The 4-qubit chip's `Q1.port0` and the
  69-qubit chip's `Qb1.port0` are two config files, not a prefix test that has
  to satisfy both.
- Misclassification becomes a load error instead of silently wrong routing.
  `mqt-scpd doctor` prints the classification table, so a bad regex costs a
  second rather than a 456-second run.
- **`Chip::ports` grows during a run**, when coupler insertion creates
  `ResonatorSource` ports. Ports are therefore appended and never reordered or
  removed, so a `PortRef` stays valid for the life of a run.
- A regular expression is a weaker guarantee than a type. It is checked at one
  place, at load, against the whole port list — which is the strongest form the
  check can take when the labels are the only names the input has.
