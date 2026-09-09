<!-- Entries in each category are sorted by merge time, with the latest PRs appearing first. -->

# Changelog

All notable changes to this project will be documented in this file.

The format is based on a mixture of [Keep a Changelog] and [Common Changelog].
This project adheres to [Semantic Versioning], with the exception that minor
releases may include breaking changes.

## [Unreleased]

### Added

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
