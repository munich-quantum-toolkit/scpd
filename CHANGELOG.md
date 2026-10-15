<!-- Entries in each category are sorted by merge time, with the latest PRs appearing first. -->

# Changelog

All notable changes to this project will be documented in this file.

The format is based on a mixture of [Keep a Changelog] and [Common Changelog].
This project adheres to [Semantic Versioning], with the exception that minor
releases may include breaking changes.

## [Unreleased]

### Added

- 📝 Add a configuration reference that lists every key of `config.toml` with
  its default and meaning ([#140]) ([**@FeldmeierMichael**])
- ✨ Add `mqt-scpd plan`, which runs the planning steps into a run directory:
  `--stop-after` and `[run] stop_after` end a run after a step, `--stage` runs
  one step again on the copies the run carries, and `-v`, `-vv` and `-vvv` show
  the steps, the items and the log of HiGHS. A run prints one block per stage
  with a live progress line on a terminal, in the format of
  `docs/terminal_output.md`. Add `list-algorithms`, and `plot` and `render` with
  `--stage` and `--run-dir` for the planning stages; the assign and corridor
  pictures show the feedline chains ([#140]) ([**@FeldmeierMichael**])
- 🎨 Draw `plot` pictures like a mask in a layout tool: hatched metal with an
  inner contour on a light substrate, a millimetre grid, rulers, corner marks, a
  scale bar and a caption. Every port is one red marker, feedlines run in
  horizontal and vertical steps in one color, and corridors are solid dark
  lines. The capacity picture shows the partition borders without the gates, and
  no picture shows the port keep-out or the capacity chains. A picture opened on
  its own has a panel that shows and hides each layer. `plot` and `render` draw
  a partition border along its cell edges instead of through them in the order
  the grid was scanned, and `plot` straightens the staircase of a slanted
  border. The capacity and corridor pictures show the blocked cells of the grid
  in dark grey under the artwork, without strips of free cells too thin to
  block, and leave out the borders of the partitions that the artwork closes in.
  A corridor runs on into its target port along the direction of the port, with
  rounded turns. Both pictures shade every partition in a light grey that
  differs from its neighbours. The assign picture lies over the same plan, draws
  its chords as thick black dashes and leaves out the ring of its ports ([#140])
  ([**@FeldmeierMichael**])
- ✨ Add the planning stages: the Capacity stage partitions the free space at
  its bottlenecks, one at the narrowest stretch of each corridor, and budgets
  the wires of every capacity chain, the Global stage solves the inner circuit
  as a mixed-integer program on Hanan lattices, with every inner wire leaving
  through a bridge to the ring, the Assignment stage assigns the ring ports to
  launchers and feedline chains with an integer program and feeds no two of them
  at one point, and the Corridor stage routes every connection through the
  partitions over the border slots inside the launcher ring, which are the slots
  the Capacity stage budgets. They write the artifacts `01-capacity.fb` to
  `04-corridor.fb`, and refuse a detail grid of more than 25,000,000 cells
  ([#140]) ([**@FeldmeierMichael**])
- ✨ Add `MQT::ScpdMilp`: a linear model, the HiGHS backend with a progress and
  log observer, MPS output, and the choice of the backend through
  `[stages.solver] backend` or `SCPD_SOLVER`: HiGHS by default, and Gurobi
  through an installed and licensed `gurobipy` when the backend is `gurobi`.
  `[stages.solver] threads` sets the threads of a solve, and both backends
  report the bound, the gap and the node count of a solve ([#140])
  ([**@FeldmeierMichael**])
- ➕ Add the C++ dependencies HiGHS 1.11.0 and Boost.Polygon 1.89.0, both
  fetched at configure time and linked static, and the Python dependency `rich`
  ([#140]) ([**@FeldmeierMichael**])
- ✨ Add bridge pairs and components to the configuration: the `bridge_pair` and
  `component` patterns, the `[[ports.bridge_pairs]]` rules, the stage sections
  and `[run]` ([#140]) ([**@FeldmeierMichael**])
- ✨ Add the `--chip` option of `doctor`, `plot`, `render` and `plan`; without
  it, these commands read the `config.toml` of a run directory with the chip
  copy `00-chip.json` beside it ([#140]) ([**@FeldmeierMichael**])
- ✨ Add `MQT::ScpdRouting`: a curvature-constrained A* over eight-way Dubins
  move primitives, with a distance-field heuristic, a bucket queue whose memory
  is that of its largest search, one shared read-only obstacle mask and
  per-thread scratch, an orthogonal search for feedline crossings, the path
  geometry decoder, turns with exact grid endpoints and tangents at the minimum
  bend radius, the self-intersection check and the coupler dogleg with
  complete-candidate validation. Crossing-mask rebuilds preserve the previous
  state on allocation failure ([#134]) ([**@FeldmeierMichael**],
  [**@marcelwa**])
- ✨ Add `MQT::ScpdGrid`: grid metrics and the rule-to-cell conversion
  `cellsFor`, bit grids, obstacle rasterization with the keepout, the exact
  distance transform and the watershed, and add the grid coordinate types
  `DCoord` and `RCoord` to the geometry schema ([#134])
  ([**@FeldmeierMichael**])
- 📝 Document how to run MQT SCPD on the benchmark chips of
  [planar-superconducting-pd](https://github.com/cda-tum/planar-superconducting-pd),
  and check every chip against it in CI ([#111]) ([**@marcelwa**])
- ✨ Add the `mqt-scpd` command line with `doctor`, `plot`, `render` and
  `inspect`: the `config.toml` and chip loaders, the port-role classification,
  the layout SVG, the KLayout adapter and the JSON view of the artifacts, which
  the core renders from the schema's own type tables ([#111])
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

- 🎨 Start the result line of `plot`, `render` and `inspect` with `✓` and an
  error of every command except `doctor` with `✗`, show the `bridge_pair`
  pattern and the reachable solvers in `doctor`, and check there, as `plan` does
  before its first stage, what would stop a run of the configured steps ([#140])
  ([**@FeldmeierMichael**])
- 🔧 Build the module libraries, the tests and the Python bindings with
  floating-point contraction off (`-ffp-contract=off`, and
  `/clang:-ffp-contract=off` for clang-cl), also inside a parent project that
  defines `MQT::ProjectOptions`, so that a sum of products rounds the same on
  every compiler and target; MSVC does not contract since Visual Studio 2022,
  and the configuration warns for an older MSVC ([#134])
  ([**@FeldmeierMichael**])
- ♻️ Make `[ports.sequences]` required and drop the `detection` and
  `start_component` keys of the configuration schema; the outer port ring is
  configuration only ([#111]) ([**@FeldmeierMichael**])
- 💥 Drop support for x86 macOS and stop publishing the respective wheels
  ([#89]) ([**@denialhaag**])
- ⬆️ Raise the macOS deployment target to 13.3 to enable `std::format` in libc++
  ([#89]) ([**@denialhaag**])
- 💥 Require Python 3.11 or newer ([#89]) ([**@denialhaag**])
- ⬆️ Update `nanobind` to version 3.0.1 ([#83]) ([**@denialhaag**])

<!-- Version links -->

[unreleased]: https://github.com/munich-quantum-toolkit/scpd

<!-- PR links -->

[#140]: https://github.com/munich-quantum-toolkit/scpd/pull/140
[#134]: https://github.com/munich-quantum-toolkit/scpd/pull/134
[#111]: https://github.com/munich-quantum-toolkit/scpd/pull/111
[#98]: https://github.com/munich-quantum-toolkit/scpd/pull/98
[#89]: https://github.com/munich-quantum-toolkit/scpd/pull/89
[#83]: https://github.com/munich-quantum-toolkit/scpd/pull/83
[#67]: https://github.com/munich-quantum-toolkit/scpd/pull/67
[#1]: https://github.com/munich-quantum-toolkit/scpd/pull/1

<!-- Contributor -->

[**@denialhaag**]: https://github.com/denialhaag
[**@FeldmeierMichael**]: https://github.com/FeldmeierMichael
[**@marcelwa**]: https://github.com/marcelwa

<!-- General links -->

[Keep a Changelog]: https://keepachangelog.com/en/1.1.0/
[Common Changelog]: https://common-changelog.org
[Semantic Versioning]: https://semver.org/spec/v2.0.0.html
