# 0023 — Port ring detection is geometric, and opt-in

- **Status:** Accepted
- **Date:** 2026-09-02
- **Amends:** [0020](0020-legacy-routing-config-as-input.md)

## Context

Decision 0020 made the outer port ring configuration. It deleted the prototype's
clockwise-ordering subsystem and copied `all_outer` and `fixed_outer` verbatim
into `config.toml`, and it recorded the cost plainly: about 330 hand-maintained
entries on the 69-qubit chip, where a typo is a worse assignment rather than a
compile error.

The prototype has since built a replacement. Commit `42bccf4`, dated 2026-09-02,
added `QubitLayout::outer_boundary_walk` and two functions that flatten it,
`ordered_outer_port_labels` and `ordered_fixed_outer_port_labels`. All ten
drivers now call them, and `src/verify_port_order.cpp` checks the derived
sequences against the literals they replaced.

The walk is a face traversal of the planar graph whose vertices are the qubits
and whose edges are the couplers. It decides which ports face outward from the
angular sector between the previous and the next qubit, swept clockwise. Its
only name test is a leading `Q` or `C`, which selects what takes part.

Two facts make this more than a convenience. It removes the largest recorded
risk in the roadmap. And it does not reintroduce the entity model, because it
never recovers a coupler's qubit pair — which is what
[decision 0018](0018-port-roles-unassigned-and-assigned.md) removed and what
decision 0020 relied on staying removed.

## Decision

`config.toml` gains `[ports].detection`, with two values.

`"manual"` is the default. The configuration supplies `all_outer` and
`fixed_outer`, exactly as decision 0020 describes.

`"auto"` derives both sequences from the chip geometry.
`[ports].start_component` selects where the closed cycle is entered; empty takes
the walk's own start. One benchmark sets it: 17Q enters at `Qb15`.

Supplying a sequence under `"auto"` is an error, not an override. `detection` is
written out in every shipped configuration even when it holds the default, for
the same reason `[design_rules]` is.

The frozen literals survive as a test fixture rather than as input. The
prototype's `verify_port_order.cpp` and its generated `.inc` become a GoogleTest
in `test/design/`.

## Alternatives considered

**`"auto"` everywhere, with no manual mode.** The cleaner shape, and where this
should end up. Rejected for now on evidence: the only measured run of the
current prototype uses the derived ring, and against the previously published
figures for the same chip its angle cost is worse — 196 against 188 — while its
runtime is better. Three upstream changes separate those two runs and the
difference is attributed to none of them. Defaulting to `"auto"` would build an
unexplained quality change into the baseline the port is judged against.

**`"auto"` by default, with the literals also committed as a cross-check.**
Checks the derivation on every run. Rejected on size: about 330 lines per
configuration, reintroducing the maintenance burden the walk exists to remove.

**Port the walk but never expose it.** Rejected. The walk is the thing that
retires risk 1, and a mechanism nobody can select does not retire anything.

## Consequences

- Decision 0020's first consequence softens. The port ring *may* be input; it is
  no longer true that it must be.
- Risk 1 in [the roadmap](../roadmap.md) shrinks but survives, because manual
  remains the default. `mqt-scpd doctor` gains the job of diffing a configured
  sequence against the walk whenever both are available, and of reporting a
  rotation as a rotation.
- A new risk replaces what risk 1 gives up: the two modes are not proven to
  agree. Phase 1 measures both on all eight benchmarks.
- The order of `all_outer` reaches the solver, so `start_component` is a real
  input rather than a display choice. `fixed_outer` reaches only
  `add_fixed_port_obstacles`, which funnels it into an unordered set, so a
  rotation there means nothing. The validation treats the two differently.
- Decision 0018 is unaffected. The walk parses no component names, so nothing in
  the port needs to know which qubits a coupler joins.
