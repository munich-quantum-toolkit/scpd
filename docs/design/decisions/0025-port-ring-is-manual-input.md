# 0025 — The port ring is manual input

- **Status:** Accepted
- **Date:** 2026-09-06
- **Supersedes:** [0023](0023-geometric-port-ring-detection.md)

## Context

Decision 0023 kept the hand-written outer port sequences as the default and
added `detection = "auto"`, under which a port of the prototype's outer-boundary
walk derives `all_outer` and `fixed_outer` from the chip geometry. It also gave
`mqt-scpd doctor` the job of comparing a configured sequence against the walk.

Phase 1 ported the walk and measured both modes on all eight benchmark chips.
The walk reproduces the drivers' sequences on 4Q, 33Q, 45Q and 57Q, up to a
rotation on 17Q, and up to an open sub-chain of the fixed sequence on 9Q. On
21Q and 69Q it produces different sequences. Those two inputs are the ones whose
components carry no mating ports — two ports per qubit and five per coupler —
so the walk has neither a qubit center nor a coupler axis to read and falls
back to centroids and diagonals. The prototype reached the same result: its
last commit before the port, `0708436` of 2026-09-05, reverted every driver to
hand-written sequences.

## Decision

The outer port ring is configuration, and only configuration.
`[ports.sequences]` with `all_outer` and `fixed_outer` is required in every
`config.toml`. The keys `detection` and `start_component` do not exist, and the
walk is not part of the tool.

What stays is the validation that decision 0020 asked for: every label in a
sequence is a routable port of the chip, no label appears twice, and
`fixed_outer ⊆ all_outer`. `mqt-scpd doctor` prints the configured ring next to
the classification table.

## Alternatives considered

**Keep automatic detection as an opt-in mode.** What decision 0023 chose.
Rejected: a mode that is wrong on two of eight benchmarks, and whose result
cannot be told from a right one without the sequence it was meant to replace,
is not a mode anyone can select. The sequences have to ship either way.

**Keep the walk as the doctor's cross-check only.** Rejected: it is several
hundred lines of geometry to check an input that the validation already checks
label by label, and where the two disagree the doctor cannot say which side is
right.

**Extend the walk to inputs without mating ports.** Rejected: it needs a
definition of a qubit's center and a coupler's axis that those inputs do not
carry, so it would reintroduce the guessing from names that decision 0018
removed.

## Consequences

- Decision 0020's first consequence stands again in full: the ring is input,
  about 330 entries on the 69-qubit chip, validated at load and printed by the
  doctor.
- The 17-qubit ring enters the cycle at `Qb15` because its sequence does; no
  key carries that rotation separately.
- The acceptance figures that were measured with the derived ring are to be
  re-measured with the shipped sequences before they serve as a baseline.
- The `Mating` role of decision 0018 stays. It names the mating ports the inputs
  carry, so that the loader classifies every port instead of dropping some.
- `MQT::ScpdGeometry` keeps its vector and bounding-box helpers; the grid of
  phase 2 is their first consumer.
