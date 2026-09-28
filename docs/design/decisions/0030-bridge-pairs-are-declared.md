# 0030 — A bridge pair is declared, and an internal one is shut

- **Status:** Accepted
- **Date:** 2026-09-09
- **Amends:** [0027](0027-components-are-declared.md),
  [0028](0028-the-inner-circuit-pays-for-free-space.md)

## Context

A wire crosses a coupler's artwork between two of its ports. Which two those
are decided where every inner wire could go, and until now it was **measured**:
[decision 0027](0027-components-are-declared.md) grouped a component's ports
from a configured pattern and then paired them by geometry — orientations
within five degrees of opposite, shortest distance first.

That rule picks the right two ports on all eight benchmark chips. It is still
the wrong kind of statement. It says what the artwork happens to be, not what
the crossing is meant to be, and nothing ever checked the two against each
other. The prototype's own rule is a hard-coded `port1`–`port2` and
`port3`–`port4` per coupler, so the two descriptions of one fact lived in two
places, in two forms, and agreed by luck.

The second question the pairing decides is which crossings a wire may use. The
prototype splits them by whether the outer ring names the coupler
(`QubitLayout.cpp:3948-4015`): a coupler on the ring gets **external** bridges,
whose far end the assignment sees, and every other coupler gets **internal**
ones, which the model couples like any other crossing
(`QubitLayoutOptimizer.cpp:219-262`). It has no way to say otherwise.

An internal crossing is a wire that enters a component the outer ring never
reaches and leaves on the far side. Nothing downstream can say where it goes
from there: the assignment works on the ring, and the ring does not name either
of those ports.

## Decision

**The pairing is declared, in two parts.** A port that a wire crosses at rather
than ends at has its own role, from its own pattern:

```toml
[ports.patterns]
conventional = '^(Qb\d+\.port1|Coupler\d+_\d+\.port0)$'
bridge_pair  = '^Coupler\d+_\d+\.port[1-4]$'
```

and which two of those ports pair is one rule per crossing, each side capturing
the component the two must share:

```toml
[[ports.bridge_pairs]]
first  = '^(Coupler\d+_\d+)\.port1$'
second = '^(Coupler\d+_\d+)\.port2$'
```

Both parts are optional together, because a chip whose components carry no
crossing declares neither. Where they are given they must agree: a port the
pattern names and no rule pairs is a load problem, and so is a port a rule
pairs whose role is something else. That check is what keeps the two from
drifting apart, and it is why the redundancy is worth its keep — the role is
what the grid, the ring and the lattices read, and the rules are what the
Global stage reads.

**An internal bridge is shut by default.** `[stages.global] internal_bridges`
grants the crossing and is `false` unless a run says otherwise. Shut means the
two ports carry no flow at all, in either direction, on any lattice — not
merely that the two sides are left uncoupled. A wire that ended at one of them
would end inside a component the ring never reaches, which is the same defect
in a quieter form. The nodes stay in the lattice, so the picture still shows
the ports.

## Consequences

The pairing is behaviour-preserving. The capacity artifact of every one of the
eight benchmarks comes out **byte-identical** to the measured pairing, and with
`internal_bridges = true` the Global stage reproduces its earlier result on all
eight exactly. The declaration therefore selects the same ports the geometry
did, which is what makes the check meaningful rather than a new rule with a new
answer.

Shutting the crossing costs four chips something, and two of them a wire:

| chip | connections | targets unserved | ring | objective |
| --- | --- | --- | --- | --- |
| 17q | 17 → 14 | 1 → 3 | 59 → 58 | 27931.03 → 23607.71 |
| 21q | 9 → 8 | 0 → 0 | 70 | 12470.96 → **12890.07** |
| 33q | 10 → 8 | 0 → 0 | 110 | 12157.51 → **12506.07** |
| 69q | 13 → 11 | 0 → 1 | 231 → 230 | 17647.40 → 12833.60 |

4Q, 9Q, 45Q and 57Q are unchanged: their circuits never wanted a crossing.

The two objectives that **rise** are the honest reading of the change — the
same targets are served by longer wires, because the shortcut through the inner
coupler is gone. The two that fall do so because fewer wires are built at all;
a target the stage cannot reach is reported rather than routed, which is what
[decision 0028](0028-the-inner-circuit-pays-for-free-space.md) made a variable
instead of an infeasibility.

## Alternatives considered

**Keep the geometric pairing and check it against the rules.** Half the cost
and none of the benefit: the pairing would still be a measurement, and a chip
whose artwork does not read the way the rule expects would still pair silently
wrong before the check ran.

**One list of pattern pairs, with no role.** Fewer keys. Rejected: the role is
what `isRoutable`, the port bands, the ring validation and the doctor table all
read, and deriving it from the rules would put the two sides of one fact back
into one place at the cost of every consumer having to run the rules to learn
what a port is.

**Leave an internal crossing uncoupled rather than shut.** Cheaper to write.
Rejected: it lets a wire end at a bridge port, which is a wire ending inside a
component the ring does not name — the defect the switch exists to remove,
still there and harder to see.

**Drop internal bridges from the model entirely.** Rejected: the prototype uses
them and the comparison against it has to stay possible, so the switch is a
configuration key rather than deleted code.
