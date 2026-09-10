<!-- Entries in each category are sorted by merge time, with the latest PRs appearing first. -->

# Changelog

All notable changes to this project will be documented in this file.

The format is based on a mixture of [Keep a Changelog] and [Common Changelog].
This project adheres to [Semantic Versioning], with the exception that minor
releases may include breaking changes.

## [Unreleased]

### Added

- ✨ Add the Detail stage: every wire of the plan is drawn cell by cell on the
  detail grid, eight-connected, inside the partitions its corridor names, and no
  two wires share a cell ([#116]) ([**@FeldmeierMichael**])
- 🐛 Refuse the one crossing that shares no cell: a diagonal step past a corner
  both of whose cells belong to one wire. The prototype's occupancy is a set of
  pixels and never records the lines between them, so two eight-connected paths
  can cross there without either noticing ([#116]) ([**@FeldmeierMichael**])
- ✨ Draw a wire the per-partition pieces could not join in one search from the
  point it is fed at to its target. A piece has to begin and end exactly at the
  crossings the plan names, and where two wires both have to pass a narrow place
  neither order fits; what is binding is which partitions the wire runs through,
  not where along a border it crosses ([#116]) ([**@FeldmeierMichael**])
- ✨ Derive the clearance the detail stage keeps between two wires from
  `min_wire_spacing` on the detail grid. The prototype carries the same quantity
  twice, computed in one pass and as the literal 6 in the other, and the two
  disagree on every benchmark ([#116]) ([**@FeldmeierMichael**])
- ✨ Render the Detail stage: `plot --stage detail` draws every wire as the
  polyline of its bends, and `render --stage detail` writes the same on layers
  of its own ([#116]) ([**@FeldmeierMichael**])
- ✨ Draw the clearance a wire is entitled to: a band around every routed wire,
  translucent in the SVG and a path on a layer of its own in the GDS, so that two
  wires closer than the clearance are two bands that overlap. The band is
  `min_wire_spacing` as the router converts it to whole cells, not the rule
  itself, because a router that works in cells cannot keep a distance the cells
  do not divide ([#116]) ([**@FeldmeierMichael**])
- ✨ Hold the wire spacing between **every** pair of wires the Detail stage
  draws, the inner circuit included, rather than between a wire and the two
  beside it in the ring. The prototype cuts its clearance out of the search mask
  and can only afford to do it for two neighbours; here the canvas carries a
  field of how many wire cells lie within the rule of each cell, so the rule
  against 240 wires costs what the rule against two cost. Over the eight
  benchmarks this takes the places where the rule does not hold from 457 to zero
  ([#116]) ([**@FeldmeierMichael**])
- 🐛 Keep a wire's two fixed places charged with their clearance while the wire
  is off the canvas. The point the assignment feeds a wire at and the cell of
  its target port are where it has to be, and a wire drawn while another one was
  lifted could settle within a wire spacing of where that other one had to
  return to — a violation no later round can undo, because neither wire can move
  the place ([#116]) ([**@FeldmeierMichael**])
- 🐛 Count how close a wire runs to another with the wire itself off the
  clearance field. Its own two ends are within the rule of the cells beside
  them, so a wire that was charged before the count was always too close to
  itself, and no wire was ever finished ([#116]) ([**@FeldmeierMichael**])
- ✨ Treat the Corridor stage's crossings as a seed and not as a constraint: a
  wire is drawn again from the point it is fed at to its target and crosses
  where it can. The crossings a plan names sit as little as 13 layout units
  apart on the benchmark chips, and no arrangement that runs through them can
  hold a rule of 185 ([#116]) ([**@FeldmeierMichael**])
- ✨ Let go of the wires behind a wire as well as ahead of it when its re-route
  fails. The prototype's `back_count` loop describes the escalation and its
  `back_count <= 0` runs the body exactly once; running it is what takes the
  last three places on the 57-qubit chip ([#116]) ([**@FeldmeierMichael**])
- ✨ Relax the corridor a wire is re-routed in, letting go of the wire ahead of
  it one place further along each time and steering the search with the price of
  leaving the room between its two neighbours rather than forbidding it. Without
  it the clearance around those two covers the way most wires have, and they keep
  the corners their pieces met at ([#116]) ([**@FeldmeierMichael**])
- ✅ Check, over every benchmark chip, that every connection is drawn and that no
  two wires come within the design rule of each other — the inner circuit
  included. A wire that runs too close counts exactly as a wire that was never
  drawn ([#116]) ([**@FeldmeierMichael**])
- ✨ Add the Corridor stage: every assigned connection is routed through the
  partitions before any pixel is drawn, crossing a border only at a slot of its
  own and never meeting another wire inside a partition — neither crossing it
  nor running along it ([#115]) ([**@FeldmeierMichael**])
- ✨ Confine the coarse routing to the ring the ports feed from: a border is a
  place to cross only strictly inside the rectangle the feed points span, so no
  wire leaves through the ring and comes back in behind another port ([#115])
  ([**@FeldmeierMichael**])
- 🐛 Refuse a wire that runs over a point another wire is pinned to: the point
  it is fed at, or the cell of its target port. Neither can be moved aside, and
  the crossing test cannot see the case at all — a chord that stops on another
  one never reaches its far side — so a wire could run down a whole line of
  feed points with no crossing reported ([#115]) ([**@FeldmeierMichael**])
- ✨ Render the Corridor stage: `plot --stage corridor` draws every wire's way
  through the partitions and every crossing slot, taken or free, and
  `render --stage corridor` writes the same on layers of its own ([#115])
  ([**@FeldmeierMichael**])
- ✨ Fill the cells of a partition back in from its outlines, so a routing stage
  can work inside one without growing the watershed a second time ([#115])
  ([**@FeldmeierMichael**])
- 🐛 Report the wire budget of a partition border as the number of wires it can
  carry. It was the length of the border in detail cells, which is what its own
  comment already denied ([#115]) ([**@FeldmeierMichael**])
- ✨ Let a target that no partition border reaches leave along its own port's
  approach. The capacity grid does not open the band it stamps, so a port the
  band and the artwork close around sat in a pocket no wire could leave
  ([#115]) ([**@FeldmeierMichael**])
- ✨ Add `[stages.capacity] crossing_pitch`: how finely a partition border is
  divided into places a wire may cross. It is a planning figure and not a
  clearance rule ([#115]) ([**@FeldmeierMichael**])

### Changed

- ♻️ Quote every figure of the Detail stage against the grid rather than in
  cells of it. `[stages.detail] corridor_half_width`, 40 cells, becomes
  `corridor_spacings`, 4 wire spacings; `obstacle_penalty_radius`, 6 cells,
  becomes `obstacle_penalty_reach`, 185 layout units. A cell is 19 layout units
  on the 17-qubit grid and 40 on the 9-qubit one, so the same cell count stood
  for two different distances ([#116]) ([**@FeldmeierMichael**])
- ♻️ Raise `[stages.detail] rounds` to 30 and `max_relaxation` to 30, from the
  prototype's 8 and 10. Its figures are enough for the clearance it holds — to
  two wires — and holding it against every wire takes more sweeps to settle: at
  8 and 10 the eight benchmarks leave 46 places where the rule does not hold
  ([#116]) ([**@FeldmeierMichael**])
- ✨ Add `mqt-scpd plan`, the resumable run directory, and
  `mqt-scpd list-algorithms` ([#114]) ([**@FeldmeierMichael**])
- ✨ Render the planning stages through the existing commands:
  `plot --stage capacity|global|assign` draws them over the chip as SVG, and
  `render --stage` writes the same content to GDSII or OASIS on layers of its
  own ([#114]) ([**@FeldmeierMichael**])
- ✨ Add `MQT::ScpdPipeline`: the stage interfaces, the name registry, and the
  Capacity, Global and Assignment stages ([#114]) ([**@FeldmeierMichael**])
- ✨ Add `MQT::ScpdMilp`: the solver-neutral model, the linked-in HiGHS backend,
  MPS emission and the runtime backend selection ([#114])
  ([**@FeldmeierMichael**])
- ✨ Add the bring-your-own-licence Gurobi backend, reached through `gurobipy`
  over an MPS round trip ([#114]) ([**@FeldmeierMichael**])
- ✨ Add the capacity layer of `MQT::ScpdGrid`: the medial axis over
  Boost.Polygon, bottleneck detection, partition extraction and the per-cell
  wire budgets ([#114]) ([**@FeldmeierMichael**])
- ✨ Prune the bottlenecks to the gates the capacity chains cross, and derive
  the partitions from the chambers those gates carve out ([#114])
  ([**@FeldmeierMichael**])
- ✨ Keep the inner circuit's Hanan lattice off the chip artwork ([#114])
  ([**@FeldmeierMichael**])
- ✨ Make the inner circuit pay for the free space it crosses: each capacity
  chain constrains the flow, so a wire may surface at an outer port only where
  the chain behind it carries one to a launcher ([#114])
  ([**@FeldmeierMichael**])
- ✨ Declare which two ports of a component a wire crosses between: the new
  `bridge_pair` role names the ports, and one `[[ports.bridge_pairs]]` rule per
  crossing pairs them. The two declarations are checked against each other at
  load ([#114]) ([**@FeldmeierMichael**])
- 🐛 Give an inner target one wire over every lattice at once, instead of one
  per lattice that carries it. A target beside a chamber border is a node of two
  lattices, and the second wire took the supply another target then had to do
  without ([#114]) ([**@FeldmeierMichael**])
- ✨ Shut a bridge whose far end the outer ring does not name, so no inner wire
  crosses a component the assignment never sees.
  `[stages.global] internal_bridges` grants the crossing ([#114])
  ([**@FeldmeierMichael**])
- ✨ Draw the wire budget of every gate beside it in the SVG, in the capacity
  and the global picture alike ([#114]) ([**@FeldmeierMichael**])
- 🐛 Report one gate per narrowing. The saddle search returns a line at every
  cell of a plateau of equal clearance, so one narrow place came back several
  times over, and a chamber that two of those lines led out of was credited with
  twice the room the chip has ([#114]) ([**@FeldmeierMichael**])
- 🐛 Keep the gates of a chamber that several corridors meet. A gate counted as
  hidden unless *every* cell beside it saw it unobstructed, which is true of no
  gate that has a neighbour, so such a chamber lost all of its gates and the
  free space beyond it fell out of the plan ([#114]) ([**@FeldmeierMichael**])
- ✨ Report what the ports' own approaches keep clear in the capacity plan, and
  draw it on a layer of its own. The port bands and the launcher sweeps are
  obstacles the chip input does not carry, so no picture could show them
  ([#114]) ([**@FeldmeierMichael**])
- 🐛 Carry a port's component through `mqt-scpd inspect`, which dropped it on
  the way to JSON and back ([#114]) ([**@FeldmeierMichael**])
- ✨ Give a bridge port twice the forward approach, worked out from the
  component's own port pairs rather than from its label ([#114])
  ([**@FeldmeierMichael**])
- 🐛 Give every port of the outer ring a connection in the assignment. Only the
  resonators that ended a feedline carried one, so the artifact named
  `launcher_target` ports of the ring and said nothing about the rest ([#114])
  ([**@FeldmeierMichael**])
- 🐛 Feed every resonator from a point between two launchers, not only one that
  ends a feedline. What stands on a launcher slot is now a conventional port and
  nothing else; the resonators a launcher was given divide the segment to the
  next launcher along evenly, in ring order ([#114]) ([**@FeldmeierMichael**])
- 🐛 Run the assignment's ordering potential over one turn of the launcher ring.
  Over the ring's own length the walk could turn twice, which put two
  conventional ports on one launcher on five of the eight benchmark chips and
  broke the cyclic order of the ring ([#114]) ([**@FeldmeierMichael**])
- 🐛 Charge a resonator's ordering step to its own launcher. The step was
  charged to the next resonator along whenever the ring opened on a conventional
  port, which is four of the eight benchmark chips ([#114])
  ([**@FeldmeierMichael**])
- ✨ Carry where each ring node is fed from in the assignment as `feeds`, so
  both renderers draw the chord to the point the wire starts at ([#114])
  ([**@FeldmeierMichael**])
- ✨ Add the component a port belongs to, declared by one more configured
  pattern, and print the grouping in `mqt-scpd doctor` ([#114])
  ([**@FeldmeierMichael**])
- ✨ Add `MQT::ScpdRouting`: the curvature-constrained A\* over Dubins
  primitives, the path geometry and its sampler, the self-intersection guard and
  the coupler dogleg insertion ([#113]) ([**@FeldmeierMichael**])
- ✨ Add `MQT::ScpdGrid`: the grid metrics and the rule-to-cell conversion, the
  obstacle rasterization with the design-rule keepout, the distance transform,
  the watershed partitioning and the port keep-out bands ([#113])
  ([**@FeldmeierMichael**])
- ✨ Add the three grid coordinate types of the data model to the geometry
  schema ([#113]) ([**@FeldmeierMichael**])
- ✨ Add the `mqt-scpd` command line with `doctor`, `plot`, `render` and
  `inspect`: the `config.toml` and chip loaders, the port-role classification,
  the layout SVG, the KLayout adapter and the JSON view of the artifacts
  ([#105]) ([**@FeldmeierMichael**])
- ✨ Add the eight benchmark configurations and their chip inputs ([#105])
  ([**@FeldmeierMichael**])
- 👷 Fail CI when the committed schema-generated code is stale ([#98])
  ([**@FeldmeierMichael**])
- ✨ Add semantic validation of the data model in the core and a checked
  artifact read and write layer in C++ and Python ([#98])
  ([**@FeldmeierMichael**])
- ✨ Add the FlatBuffers schemas of the data model, the committed C++ and Python
  code generated from them, and the `nox -s schemas` session ([#98])
  ([**@FeldmeierMichael**])
- 🏗️ Split the core into the eight per-module CMake targets of the architecture
  ([#98]) ([**@FeldmeierMichael**])
- 🐍 Start building CPython 3.15 wheels ([#67]) ([**@denialhaag**])
- ✨ Set up the repository ([#1]) ([**@denialhaag**])

### Changed

- 💥 Run the Global stage before the Assignment stage, and number the artifacts
  `02-global.fb` and `03-assign.fb` accordingly; which outer port an inner wire
  surfaces at is what the assignment's ring is made of ([#114])
  ([**@FeldmeierMichael**])
- ♻️ Build the grid and stage sections of a configuration whether or not the
  file carries them, so that an absent section is the defaults rather than
  nothing for the stage that reads it ([#114]) ([**@FeldmeierMichael**])
- ♻️ Make `[ports.sequences]` required and drop the `detection` and
  `start_component` keys of the configuration schema; the outer port ring is
  configuration only ([#105]) ([**@FeldmeierMichael**])
- 💥 Drop support for x86 macOS and stop publishing the respective wheels
  ([#89]) ([**@denialhaag**])
- ⬆️ Raise the macOS deployment target to 13.3 to enable `std::format` in libc++
  ([#89]) ([**@denialhaag**])
- 💥 Require Python 3.11 or newer ([#89]) ([**@denialhaag**])
- ⬆️ Update `nanobind` to version 3.0.1 ([#83]) ([**@denialhaag**])

<!-- Version links -->

[unreleased]: https://github.com/munich-quantum-toolkit/scpd

<!-- PR links -->

[#116]: https://github.com/munich-quantum-toolkit/scpd/pull/116
[#115]: https://github.com/munich-quantum-toolkit/scpd/pull/115
[#114]: https://github.com/munich-quantum-toolkit/scpd/pull/114
[#113]: https://github.com/munich-quantum-toolkit/scpd/pull/113
[#105]: https://github.com/munich-quantum-toolkit/scpd/pull/105
[#98]: https://github.com/munich-quantum-toolkit/scpd/pull/98
[#89]: https://github.com/munich-quantum-toolkit/scpd/pull/89
[#83]: https://github.com/munich-quantum-toolkit/scpd/pull/83
[#67]: https://github.com/munich-quantum-toolkit/scpd/pull/67
[#1]: https://github.com/munich-quantum-toolkit/scpd/pull/1

<!-- Contributor -->

[**@denialhaag**]: https://github.com/denialhaag
[**@FeldmeierMichael**]: https://github.com/FeldmeierMichael

<!-- General links -->

[Keep a Changelog]: https://keepachangelog.com/en/1.1.0/
[Common Changelog]: https://common-changelog.org
[Semantic Versioning]: https://semver.org/spec/v2.0.0.html
