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
- ✨ Add `mqt-scpd plan`, which runs the planning steps into a run directory
  ([#140]) ([**@FeldmeierMichael**]):
  - `--stop-after` and `[run] stop_after` end a run after a step.
  - `--stage` runs one step again on the copies that the run carries. It refuses
    the step when the chip or a configuration section that an earlier step read
    has changed since that step.
  - `-v`, `-vv` and `-vvv` show the steps, the items and the log of HiGHS.
  - A run prints one block per stage in the format of `docs/terminal_output.md`.
    On a terminal, each block has a live progress line, which `--no-progress`
    turns off.
- ✨ Add `list-algorithms`, and `--stage` and `--run-dir` of `plot` and `render`
  for the planning stages ([#140]) ([**@FeldmeierMichael**]):
  - `plot` and `render` refuse an artifact whose inputs have changed since its
    step, and `inspect` prints such an artifact after a warning.
  - The assign and corridor pictures show the feedline chains.
- 🎨 Draw `plot` pictures like a mask in a layout tool ([#140])
  ([**@FeldmeierMichael**]):
  - Metal is hatched and has an inner contour on a light substrate. A picture
    has a millimetre grid, rulers, corner marks, a scale bar and a caption.
  - Every port is one red marker. Feedlines run in horizontal and vertical steps
    in one color. Corridors are solid dark lines that run on into their target
    port along its direction, with rounded turns.
  - A picture opened on its own has a panel that shows and hides each layer.
  - The capacity picture shows the partition borders. No picture shows the
    gates, the port keep-out or the capacity chains.
  - `plot` and `render` draw a partition border along its cell edges. `plot`
    straightens the staircase of a slanted border.
  - Every planning picture shows the blocked cells of the grid in dark grey
    under the artwork, without strips of free cells too thin to block. The
    capacity, assign and corridor pictures leave out the borders of the
    partitions that the artwork closes in, and shade every partition in a light
    grey that differs from its neighbours.
  - The assign and corridor pictures show the inner wires on the lattice edges
    that the global step selected.
  - The assign picture draws its chords as thick black dashes and leaves out the
    ring of its ports.
- ✨ Add the planning stages. The Capacity stage partitions the free space at
  its bottlenecks, one at the narrowest stretch of each corridor along a grid
  axis or a diagonal, and budgets the wires of every capacity chain. The Global
  stage solves the inner circuit as a mixed-integer program on Hanan lattices.
  Every inner wire leaves through a bridge to the ring. The Global stage counts
  a ring port as a fail when the gates behind it have no room for it, or when
  its chain does not carry it to a launcher. The Assignment stage assigns the
  ring ports to launchers and feedline chains with an integer program, and feeds
  no two of them at one point. The Corridor stage routes every connection
  through the partitions over the border slots inside the launcher ring, which
  are the slots that the Capacity stage budgets. It keeps the crossings of two
  wires one crossing pitch apart. The stages write the artifacts
  `01-capacity.fb` to `04-corridor.fb`. They refuse a detail grid of more than
  25,000,000 cells, and a chip on which the targets of two ports lie on one
  detail cell ([#140]) ([**@FeldmeierMichael**])
- ✨ Add `MQT::ScpdMilp`: a linear model, the HiGHS backend with a progress and
  log observer, MPS output, and the choice of the backend through
  `[stages.solver] backend` or `SCPD_SOLVER`: HiGHS by default, and Gurobi
  through an installed and licensed `gurobipy` when the backend is `gurobi`.
  `[stages.solver] threads` sets the threads of a solve, and both backends
  report the bound, the gap and the node count of a solve ([#140])
  ([**@FeldmeierMichael**])
- ➕ Add the C++ dependencies HiGHS 1.11.0, linked static, and the header-only
  Boost.Polygon and Boost.Config of Boost 1.89.0, all fetched at configure time,
  and the Python dependency `rich`. The wheel carries the licences of HiGHS,
  FlatBuffers and nlohmann/json, which its extension links in ([#140])
  ([**@FeldmeierMichael**])
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

- 🔥 Stop reading `[grid] launcher_offset_x` and `launcher_offset_y`: the
  capacity step blocks no border of its grid, so the launchers on the edge stay
  free. A configuration that sets them still loads, and `doctor` and `plan` warn
  about each key ([#140]) ([**@FeldmeierMichael**])
- 🎨 Change the output of the commands ([#140]) ([**@FeldmeierMichael**]):
  - `plot`, `render` and `inspect` start their result line with `✓`.
  - Every command except `doctor` starts the line of an input or stage error
    with `✗`.
  - `doctor` shows the `bridge_pair` pattern and the solvers it can reach.
  - `doctor` checks what would stop a run of the steps up to `[run] stop_after`,
    as `plan` does before its first stage. Without that key, `doctor` checks all
    four steps and fails a configuration without `launcher_target`.
  - `doctor` warns about each removed key on a line that starts with `WARNING:`.
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
