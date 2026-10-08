<!-- Entries in each category are sorted by merge time, with the latest PRs appearing first. -->

# Changelog

All notable changes to this project will be documented in this file.

The format is based on a mixture of [Keep a Changelog] and [Common Changelog].
This project adheres to [Semantic Versioning], with the exception that minor
releases may include breaking changes.

## [Unreleased]

### Added

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
