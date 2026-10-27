<!-- Entries in each category are sorted by merge time, with the latest PRs appearing first. -->

# Changelog

All notable changes to this project will be documented in this file.

The format is based on a mixture of [Keep a Changelog] and [Common Changelog].
This project adheres to [Semantic Versioning], with the exception that minor
releases may include breaking changes.

## [Unreleased]

### Added

- ✨ Add the button "Rip up and reroute feedline C" to the coupler tab of a
  feedline on the coupler server: `POST /reroute?chain=C` reroutes the
  feedline's wires and the free wires beside it alone on the options it stands
  on, as the local benchmark does, puts the chip back and draws the ways
  (`CouplerSession.local_reroute`) ([#XXX]) ([**@FeldmeierMichael**])
- ✨ Add `[stages.final] chain_end_blacklist` and `chain_slot_blacklist`, on by
  default: under the capacity rule the chain search blacklists the pairs of
  options at an end of a chain whose refused step closed only wires of that end,
  and the pairs whose edge got fewer crossing slots than wires must cross it,
  and drops a later step on a blacklisted pair unrouted and unchecked ([#XXX])
  ([**@FeldmeierMichael**])
- ✨ Add `[stages.final] coupler_copy_cells`: eight copies of every plain
  coupler option, moved by that many cells on each axis, each with its lead
  joined to the nearest cell of the resonator's way and refused when the
  resonator is then too long or too short; they replace the jogs and open as the
  jogs do ([#XXX]) ([**@FeldmeierMichael**])
- ✨ Add `[stages.final] coupler_line_options`: options whose feedline ports lie
  on a line of their chain, the chord of their neighbours or the line that fits
  the whole chain, turned to the nearest of the eight headings ([#XXX])
  ([**@FeldmeierMichael**])
- ✨ Add `[stages.final] chain_search`: `bidirectional` searches the options of
  a chain from its last waypoint as well and lays the run of a prefix and a
  suffix that meet again from the front; `astar`, the default, searches from the
  front alone ([#XXX]) ([**@FeldmeierMichael**])
- ✨ Place a coupler option by hand on the coupler server: `POST /place` and the
  "Place an option" button of the coupler tab build an option centred on the
  cell clicked, cut the resonator at its nearest cell, put the coupler on it and
  check the feedline and the chip again (`CouplerSession.add_option`) ([#XXX])
  ([**@FeldmeierMichael**])
- ✨ Add `[stages.final] coupler_option_extension`: centred coupler options,
  with the pad's centre on the resonator's way at the place nearest the target
  length and at each listed number of cells further along it, toward the qubit,
  for every allowed offset; `0,20` adds them at the place and twenty cells
  further on, and empty adds none. The plain options stay as they are, and
  `coupler_max_shortfall` does not refuse the centred ones. The chain search
  opens a coupler's centred options only when no run reaches past it with its
  jogs open, or when `chain_rule_refusals` stops it ([#XXX])
  ([**@FeldmeierMichael**])
- ✨ Add `[stages.final] jog_options`: the jogs a coupler option may take in
  front of the pad, as a comma list of lengths in cells, each tried turned to
  either side; empty takes the one jog of 20 cells, as before ([#XXX])
  ([**@FeldmeierMichael**])
- ✨ Add `[stages.final] meander_relaxation` (on by default): a resonator whose
  way has no room for its meander goes into the relaxation, as a wire with no
  way does, and keeps its short way when no level finds one long enough ([#XXX])
  ([**@FeldmeierMichael**])
- ✨ Add the routing stages ([#XXX]) ([**@FeldmeierMichael**]):
  - The Detail stage draws every corridor as a path of cells on the detail grid,
    with an eight-connected search inside the corridor.
  - The Final stage draws every wire as curvature-constrained copper on the
    router grid, inserts the couplers, routes the feedline chains and refines
    them, in five phases: `pre-feedline-routing`, `pre-feedline-refinement`,
    `feedline-insertion`, `feedline-constrained-routing` and
    `feedline-constrained-refinement`. Each phase leaves a snapshot in the
    artifact, and `[run] stop_after` and `--stop-after` take a phase as well as
    a step.
  - The coupler insertion checks every chain of feedline edges against a
    capacity graph of the free space, with the wires of its feedline and the
    free wires beside them, judges the whole chip by the crossing slots of its
    feedline edges, and searches a globally satisfiable state over the chains
    when the chip is not. `[stages.final] capacity_check` turns the check off.
  - Every resonator reaches its target length in every phase: the meander takes
    several loops where one does not fit, the cheapest placement, and a loop
    along the other diagonal only as the last resort; before the couplers it
    stays inside the coupler box and the feedline edges keep out of it; the
    coupler leaves `coupler_bias` (0.98) times the target; and a resonator
    through a bridge carries its length beyond the bridge.
  - `[stages.detail]` and `[stages.final]` hold the parameters of the two
    stages; `docs/routing.md` describes the stages and `docs/configuration.md`
    every key.
  - Both stages report as the planning stages do: the Detail stage one entry per
    round and its result, the Final stage one entry per phase and a result that
    counts every wire, with the wires without a usable way as its fails and the
    resonators off their length as figures beside them;
    `docs/terminal_output.md` shows the two blocks.
- ✨ Add `[debug]`, a configuration section that names the debug pages a run
  writes under `<run>/debug`, and `mqt-scpd plan -d`, which turns every page on
  ([#XXX]) ([**@FeldmeierMichael**]):
  - A dashboard per routing phase, the coupler options with the capacity graph,
    every step check of the chain search and the course of the global SAT
    search, linked from `index.html`. The Final stage records what the pages
    need as JSON, one record at a time, and writes no file itself.
  - `mqt-scpd debug <run>` builds the pages again from the records, and
    `mqt-scpd debug <run> --serve` serves the capacity graph on a local page
    whose tab moves a coupler to another of its options.
  - `global_sat` records every chain state that the global SAT search checks
    locally, the committed state of each chain first, with its options, its
    moved couplers and the check that found it SAT.
  - The coupler session takes `debug` and records its own run to the end of the
    coupler insertion; `chains()` lists every chain, and `local_check()` runs
    the two local checks of the search on a chain with its couplers on given
    options.
  - The local SAT page is a dashboard of the feedlines, a card per feedline. A
    feedline is SAT when a step check of the chain search at its last edge is
    SAT. Its card lists per edge the SAT options, or the UNSAT options of an
    edge with none, each with a link to the step check that the coupler
    session's `step_check()` rebuilds (`/graph?step=`), and below them the
    states the global search checked. The global SAT page lists every
    combination the search checked, and each state number links to its row on
    the local SAT page.
  - `mqt-scpd debug <run> --serve` serves both dashboards at `/sat` and
    `/global`, from the session's own run: a link opens the graph of a state's
    local check or a combination's whole-chip check and says when the verdict
    differs from the record, and another puts a feedline on the state's options.
    After every move the server checks the moved feedlines locally and the chip
    as a whole; the coupler tab shows the three verdicts and can undo, reset to
    the commit and check every feedline.
  - The coupler tab lists every option of a coupler with why the chain search
    closed it and what its step checks found, and every candidate the option
    generator refused, with the reason; `couplers()` and the `coupler-options`
    record carry the reasons, and a `step-skip` record names the steps the chain
    search dropped without a step check.
  - `mqt-scpd debug <run> --serve` answers from the first second: it builds the
    session on a thread of its own, `/status.json` says how far the stage is,
    and `/sat` and `/global` show the records so far under a progress bar and
    fill in as the records come.
- ✅ Add the local and the global SAT benchmark, which measure how well the
  capacity check of the coupler insertion predicts the feedline routing; the
  coupler session's `compare_locally` is their entry, and
  `uvx nox -s sat_benchmarks` runs them ([#XXX]) ([**@FeldmeierMichael**])
- ✨ Add `mqt-scpd sat-bench {local|global} <run>`, which runs a SAT benchmark
  on a run directory and writes its table and page to `<run>/sat-bench`; the
  global benchmark takes the local table of the same runs, and
  `mqt-scpd debug <run> --serve` serves both pages at `/bench/local` and
  `/bench/global`, with links to the check and the routed ways of every case
  ([#XXX]) ([**@FeldmeierMichael**])
- ✨ Add `mqt-scpd drc`, which checks a finished run against the design rules
  and writes `drc.json` beside the artifacts ([#XXX]) ([**@FeldmeierMichael**])
- 🎨 Draw the routing stages with `plot` and `render` ([#XXX])
  ([**@FeldmeierMichael**]):
  - Every wire is a coplanar waveguide in the colors of the figures: the plain
    wires and the feedline edges gray and the resonators light, with the room
    every wire keeps as a translucent band and the couplers as pads.
  - Every wire of the Final stage runs into its ports, over the straight
    approach that the stage keeps clear in front of each port.
  - In the two phases that route under the feedline constraints, and in the end
    state, an air bridge stands where a wire crosses a feedline edge between two
    couplers: a translucent deck on two posts, so that the feedline under it
    stays visible.
  - The wires the Final stage left failing and the resonators off their length
    are marked on layers of their own and counted in the legend.
  - `--phase` draws the snapshot of one phase of the Final stage; the GDS
    carries every phase on a layer of its own.
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
    has a millimeter grid, rulers, corner marks, a scale bar and a caption.
  - Every port is one red marker. Feedlines run in horizontal and vertical steps
    in one color. Corridors are solid dark lines that run on into their target
    port along its direction, with rounded turns.
  - A picture opened on its own has a panel that shows and hides each layer.
  - The capacity picture shows the partition borders. No picture shows the
    gates, the port keep-out or the capacity chains.
  - `plot` and `render` draw a partition border along its cell edges. `plot`
    straightens the staircase of a slanted border.
  - Every planning picture shows the blocked cells of the grid in dark gray
    under the artwork, without strips of free cells too thin to block. The
    capacity, assign and corridor pictures leave out the borders of the
    partitions that the artwork closes in, and shade every partition in a light
    gray that differs from its neighbors.
  - The assign and corridor pictures show the inner wires on the lattice edges
    that the global step selected.
  - The assign picture draws its chords as thick black dashes and leaves out the
    ring of its ports.
- ✨ Add the planning stages. The Capacity stage partitions the free space at
  its bottlenecks, one at the narrowest stretch of each corridor along a grid
  axis or a diagonal, and budgets the wires of every capacity chain. The Global
  stage solves the inner circuit as a mixed-integer program on Hanan lattices.
  Every inner wire leaves through a bridge to the ring. The Global stage counts
  as a fail an inner port that no wire of the circuit reaches, a ring port that
  the gates of its chain have no room for, and a ring port without a bridge that
  its chain does not carry to a launcher. The Assignment stage assigns the ring
  ports to launchers and feedline chains with an integer program, and feeds no
  two of them at one point. The Corridor stage routes every connection through
  the partitions over the border slots inside the launcher ring, which are the
  slots that the Capacity stage budgets. It keeps the crossings of two wires one
  crossing pitch apart. The stages write the artifacts `01-capacity.fb` to
  `04-corridor.fb`. They refuse a detail grid of more than 25,000,000 cells, a
  chip on which the targets of two ports lie on one detail cell, and two ports
  of one capacity chain that can carry an inner wire where the points
  `[design_rules] min_straight_length` in front of them coincide ([#140])
  ([**@FeldmeierMichael**])
- ✨ Add `MQT::ScpdMilp`: a linear model, the HiGHS backend with a progress and
  log observer, MPS output, and the choice of the backend through
  `[stages.solver] backend` or `SCPD_SOLVER`: HiGHS by default, and Gurobi
  through an installed and licensed `gurobipy` when the backend is `gurobi`.
  `[stages.solver] threads` sets the threads of a solve, and both backends
  report the bound, the gap and the node count of a solve ([#140])
  ([**@FeldmeierMichael**])
- ➕ Add the C++ dependencies HiGHS 1.11.0, linked static, and the header-only
  Boost.Polygon and Boost.Config of Boost 1.89.0, all fetched at configure time,
  and the Python dependency `rich`. The wheel carries the licenses of HiGHS, its
  LP file reader, FlatBuffers and nlohmann/json, which its extension links in,
  and its license expression names Apache-2.0, BSL-1.0 and Zlib next to MIT
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

- 🔧 The `standing` local check of a chain and the window check of the local SAT
  benchmark no longer route the terminal edges of the other chains: they only
  name the mouths between the chains (`SlotGraph::Wire::standing`); in the
  `standing` check they are walls, inflated by the wire clearance as every
  feedline edge is (`InsertionState::terminalChain`); the `alone` check routes
  the chain's terminal edges as wires with their port runs as well, where they
  were thin walls, and so do the step checks of the chain search and the check
  of each chain alone of the whole-chip check ([#XXX]) ([**@FeldmeierMichael**])
- 🔧 Run the local rip-up of the targeted repair and of the local SAT comparison
  for four rounds by default: `[stages.final] research_rounds` is `4` ([#XXX])
  ([**@FeldmeierMichael**])
- 🔧 Turn the targeted repair of the Final stage off by default:
  `[stages.final] repair_trials` is `0` unless a configuration sets it ([#XXX])
  ([**@FeldmeierMichael**])
- 🔥 Stop reading `[grid] launcher_offset_x` and `launcher_offset_y`: the
  capacity step blocks no border of its grid, so the launchers on the edge stay
  free. A configuration that sets them still loads, and `doctor` and `plan` warn
  about each key ([#140]) ([**@FeldmeierMichael**])
- 🎨 Change the output of the commands ([#140]) ([**@FeldmeierMichael**]):
  - `plot`, `render` and `inspect -o` start their result line with `✓`.
  - Every command except `doctor` starts the line of an input or stage error
    with `✗`.
  - `doctor` shows the `bridge_pair` pattern and the solvers it can reach.
  - `doctor` checks what would stop a run of the steps up to `[run] stop_after`,
    as `plan` does before its first stage. Without that key, `doctor` checks all
    four steps and fails a configuration without `launcher_target`.
  - `doctor` warns about each removed key on a line that starts with `WARNING:`.
  - Every command exits with code 130 on Ctrl-C. It stops without a message and
    exits with code 1 when the reader of its output goes away, as with `| head`.
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

- 🐛 Put the terminal edges of a state of the insertion at the ports of the
  options the state names, not of the options the couplers hold: a step of the
  chain search judged the run of its end edge at the wrong port and passed steps
  that close wires; and name the wires a step loses from the state's wires, not
  out of range of the chip's ([#XXX]) ([**@FeldmeierMichael**])
- 🐛 Close the slot of a terminal edge's port run at both flanks when the
  terminal edges are wires: such a run is no static wall, so it had no mouth,
  and a flank whose open end no profile reached had no gate to the wall ahead of
  it ([#XXX]) ([**@FeldmeierMichael**])
- 🐛 Route a chip whose configuration leaves out `[grid]` through the Final
  stage and the coupler session with the grid defaults, as the planning stages
  do, instead of stopping at the Final stage ([#XXX]) ([**@FeldmeierMichael**])
- 🐛 Handle self-addition and self-subtraction of linear expressions, preserve
  port-keepout holes in GDS/OASIS exports, and check launcher order
  independently of the chip's position ([#140]) ([**@FeldmeierMichael**],
  [**@marcelwa**])

<!-- Version links -->

[unreleased]: https://github.com/munich-quantum-toolkit/scpd

<!-- PR links -->

[#XXX]: https://github.com/munich-quantum-toolkit/scpd/pull/XXX
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
