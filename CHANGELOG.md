<!-- Entries in each category are sorted by merge time, with the latest PRs appearing first. -->

# Changelog

All notable changes to this project will be documented in this file.

The format is based on a mixture of [Keep a Changelog] and [Common Changelog].
This project adheres to [Semantic Versioning], with the exception that minor
releases may include breaking changes.

## [Unreleased]

### Added

- ✨ Add Corridor as the default final stage. It routes connections through
  partition borders and writes `04-corridor.fb`, which `plot` and `render` can
  draw. The plan reports a failure if any connection has no route. Corridor
  rejects Capacity artifacts whose partition outlines cover blocked cells or
  whose border samples do not join the stated partitions ([#168])
  ([**@FeldmeierMichael**], [**@marcelwa**])
- 📝 Add a reference of every `config.toml` key ([#166])
  ([**@FeldmeierMichael**])
- ✨ Add `mqt-scpd plan`, which runs the planning steps into a run directory and
  can stop after a step or run one step again ([#166]) ([**@FeldmeierMichael**])
- ✨ Add `list-algorithms`, and draw the planning stages of a run directory with
  `plot` and `render` ([#166]) ([**@FeldmeierMichael**])
- 🎨 Draw `plot` pictures like a mask in a layout tool ([#166])
  ([**@FeldmeierMichael**])
- ✨ Add the Capacity stage, which partitions the free space at its bottlenecks
  and budgets the wires of every capacity chain ([#166])
  ([**@FeldmeierMichael**])
- ➕ Add Boost.Polygon, Boost.Config and `rich` as dependencies ([#166])
  ([**@FeldmeierMichael**])
- ✨ Add bridge pairs, components, the stage sections and `[run]` to the
  configuration ([#166]) ([**@FeldmeierMichael**])
- ✨ Add the `--chip` option of `doctor`, `plot`, `render` and `plan` ([#166])
  ([**@FeldmeierMichael**])
- ✨ Add Global routing on Hanan lattices and Assignment of ring ports to
  launchers and feedline chains ([#167]) ([**@FeldmeierMichael**],
  [**@marcelwa**])
- ➕ Add HiGHS 1.16.0, the `gurobi` installation extra, and solver configuration
  ([#167]) ([**@FeldmeierMichael**], [**@marcelwa**])
- 🎨 Draw Global and Assignment artifacts with the stored Capacity raster
  ([#167]) ([**@FeldmeierMichael**], [**@marcelwa**])
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

- ⬆️ Require KLayout 0.30.13 or later in the 0.30 series for Python 3.15 wheels
  on Linux and macOS ([#167]) ([**@marcelwa**])
- ⚡ Use sliding windows to enforce the feedline utilization limit in Assignment
  ([#167]) ([**@FeldmeierMichael**], [**@marcelwa**])
- ⬆️ Require Python 3.11.5 or newer ([#178]) ([**@denialhaag**])
- 🔥 Stop reading `[grid] launcher_offset_x` and `launcher_offset_y` ([#166])
  ([**@FeldmeierMichael**])
- 🎨 Change the output and the exit codes of the commands to the format of
  `docs/terminal_output.md` ([#166]) ([**@FeldmeierMichael**])
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

### Fixed

- 🐛 Preserve completed runs on planning failures and verify artifact provenance
  when resuming or drawing a run ([#166]) ([**@FeldmeierMichael**],
  [**@marcelwa**])
- 🐛 Store the actual Capacity obstacle raster in `SCP2` artifacts and use the
  same artifact validation for inspection and drawing ([#166])
  ([**@FeldmeierMichael**], [**@marcelwa**])
- 🐛 Preserve port-keepout holes in GDS/OASIS exports ([#166])
  ([**@FeldmeierMichael**], [**@marcelwa**])
- 🐛 Check launcher slot order and keep plain ring ports first at capacity gates
  ([#167]) ([**@FeldmeierMichael**], [**@marcelwa**])

<!-- Version links -->

[unreleased]: https://github.com/munich-quantum-toolkit/scpd

<!-- PR links -->

[#178]: https://github.com/munich-quantum-toolkit/scpd/pull/178
[#168]: https://github.com/munich-quantum-toolkit/scpd/pull/168
[#167]: https://github.com/munich-quantum-toolkit/scpd/pull/167
[#166]: https://github.com/munich-quantum-toolkit/scpd/pull/166
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
