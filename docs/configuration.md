# Configuration Reference

A `config.toml` says how MQT SCPD plans a chip. This page lists every key with
its default and its meaning. A key without a default is required in its section.
Every length is in layout units, the µm of the chip input. A section or key that
MQT SCPD does not know is an error.

`mqt-scpd doctor -c config.toml` checks a configuration and its chip before a
run.

## `[chip]`

| Key     | Default | Meaning                                                    |
| ------- | ------- | ---------------------------------------------------------- |
| `input` | —       | The chip input, as a path relative to the `config.toml`    |

`--chip` replaces `input` for one command. A `config.toml` with a `00-chip.json`
beside it, as in a run directory, reads that file instead.

## `[ports.patterns]`

Each pattern is an ECMAScript regular expression that has to match the whole
port label. Every port must match exactly one of the role patterns `launcher`,
`resonator`, `conventional` and `bridge_pair`.

| Key            | Default | Meaning                                                                                     |
| -------------- | ------- | ------------------------------------------------------------------------------------------- |
| `launcher`     | —       | The ports at the edge of the chip where a feedline starts or ends                           |
| `resonator`    | —       | The ports that a resonator wire connects to a feedline                                      |
| `conventional` | —       | The ports that a regular wire connects                                                      |
| `bridge_pair`  | `""`    | The ports where a wire crosses a component instead of ending at it; empty for none          |
| `component`    | `""`    | The component a port belongs to, as one capture group over the label; empty for no grouping |

A chip with `bridge_pair` ports needs the `component` pattern, and the
`component` pattern has to match every `bridge_pair` port.

## `[ports.sequences]`

| Key           | Default | Meaning                                                                         |
| ------------- | ------- | ------------------------------------------------------------------------------- |
| `all_outer`   | —       | The outer port ring, in the order the assignment walks it                       |
| `fixed_outer` | —       | The ports of `all_outer` that a run pins; the order does not matter             |

## `[[ports.bridge_pairs]]`

Each rule pairs the two `bridge_pair` ports that a wire crosses a component
between. Both sides are regular expressions with exactly one capture group. Two
ports pair when they belong to one component and their captures are equal. The
rules apply in the order the file lists them.

| Key      | Default | Meaning                           |
| -------- | ------- | --------------------------------- |
| `first`  | —       | The pattern of one port of a pair |
| `second` | —       | The pattern of the other port     |

## `[design_rules]`

Every design rule is required. Every length must be positive.

| Key                          | Default | Meaning                                                                                              |
| ---------------------------- | ------- | ---------------------------------------------------------------------------------------------------- |
| `min_wire_spacing`           | —       | The smallest distance between two wires                                                              |
| `min_obstacle_spacing`       | —       | The smallest distance between a wire and the artwork                                                 |
| `min_bend_radius`            | —       | The smallest radius of a turn of a wire                                                              |
| `min_straight_length`        | —       | The straight length of a wire in front of a port                                                     |
| `target_resonator_length`    | —       | The length a resonator wire should have                                                              |
| `resonator_length_tolerance` | —       | How far a resonator wire may differ from `target_resonator_length`                                   |
| `max_feedline_utilization`   | —       | The most resonators that one feedline chain feeds; at least one                                      |
| `feedline_terminations`      | —       | How many feedline chain ends stop at a termination instead of a launcher                             |

The planning steps of this release do not read `min_bend_radius`,
`target_resonator_length` and `resonator_length_tolerance`. The routing stages
of a later release read them.

## `[grid]`

The capacity step cuts the chip into a coarse capacity grid, and each capacity
cell into a finer detail grid.

| Key                | Default | Meaning                                                                                                                                        |
| ------------------ | ------- | ---------------------------------------------------------------------------------------------------------------------------------------------- |
| `capacity_cells_x` | `50`    | The cells of the capacity grid across the width of the chip; at least two                                                                      |
| `capacity_cells_y` | `0`     | The cells of the capacity grid across the height of the chip; `0` takes the aspect ratio, any other value is at least two                      |
| `detail_factor`    | `30`    | The detail cells per capacity cell along each axis; at least one                                                                               |
| `router_cell_size` | `10.0`  | The size of a router cell in layout units; every capacity cell is divided into as many router cells as it holds steps of this size, rounded up |

No stage reads `launcher_offset_x` and `launcher_offset_y` any more. The loader
still accepts them, and `doctor` and `plan` warn about each one; a later version
refuses them.

The detail grid has `capacity_cells_x` × `detail_factor` cells across and
`capacity_cells_y` × `detail_factor` cells down; with `capacity_cells_y` at `0`,
the aspect ratio of the chip sets the capacity cells down. The detail grid may
have at most 25,000,000 cells, for example 5000 × 5000. The capacity step holds
about 120 bytes per detail cell, so the limit stands for about 3 GB of memory.
The finest benchmark chip, 69q, plans on 1500 × 1500 cells. `plan` and `doctor`
refuse a larger grid before the first stage of a run through the capacity,
assignment or corridor step, and name the grid that the configuration asks for.

## `[stages.capacity]`

| Key                    | Default | Meaning                                                                                                                                                                                                 |
| ---------------------- | ------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `planner`              | `""`    | The implementation of the step; empty takes `watershed`                                                                                                                                                 |
| `bottleneck_clearance` | `1.5`   | A place is a bottleneck only where its free radius is below this many wire pitches; a pitch is `min_wire_spacing` plus `min_obstacle_spacing`; positive                                                 |
| `crossing_pitch`       | `165.0` | The distance between two places on one partition border where a wire may cross it, and the least distance between the crossings of two wires; it decides how many wires a border lets through; positive |

`crossing_pitch` applies to crossings only. A wire may cross a border closer
than one pitch to the target cell or the feed of another wire.

## `[stages.global]`

| Key                | Default | Meaning                                                                                                    |
| ------------------ | ------- | ---------------------------------------------------------------------------------------------------------- |
| `router`           | `""`    | The implementation of the step; empty takes `hanan-milp`                                                   |
| `internal_bridges` | `false` | Whether an inner wire may cross a component that the outer ring does not reach, through its own bridges    |

Every inner wire starts at the inner end of a bridge whose other end is on the
ring, and surfaces at that ring end. With `internal_bridges`, the wire may cross
internal bridges on the way. No ring port starts an inner wire, because the
assignment gives every ring port a wire of its own. An inner port that no such
wire can reach is a fail of the global step. An inner resonator that is a fail
gets no feed.

The lattice lines of a port run through the point
`[design_rules] min_straight_length` in front of it. The step refuses two ports
of one capacity chain that can carry an inner wire when these points coincide,
since both would meet the lattice of their chain at one node. A port without an
inner wire, such as a ring port, may share its point with any port. `doctor`
does not check this; the step refuses such a chip when it runs.

The wires of the ring ports behind a gate of a capacity chain pass that gate on
their way to a launcher, and so do the inner wires that surface there. When the
gates have no room for all of them, the step solves the circuit again and lets
ring ports go without room. Each such ring port is a fail of the global step,
and `-vv` names it. A port lies on one chain, and a gate on several chains holds
the wires of all of them. The step checks a gate with some room only where it
solves an inner circuit, and only on the chains without inner ports. Elsewhere,
a gate that has room for some of its ring ports but not for all gives no fail. A
ring port that no bridge joins always needs a way to a launcher: where its chain
has no launcher, or a gate without room cuts it off, it is a fail too, also on a
chip without inner ports. The ring end of a bridge may be cut off, because it
needs a way only where an inner wire surfaces there.

## `[stages.assignment]`

| Key               | Default | Meaning                                                                                                                    |
| ----------------- | ------- | -------------------------------------------------------------------------------------------------------------------------- |
| `assigner`        | `""`    | The implementation of the step; empty takes `ordered-milp`                                                                 |
| `launcher_target` | `0`     | How many feedline chain ends stop at a launcher; `0` leaves it unset, and `plan` refuses a run that reaches the assignment |

`launcher_target` plus `[design_rules] feedline_terminations` must be even,
since every feedline chain has two ends. A ring with resonators sets four more
limits, which the assignment refuses to break:

- the conventional ports of the ring plus `launcher_target` are at most the
  launcher ports of the chip;
- `launcher_target` and `feedline_terminations` are each at most the resonators
  of the ring;
- `feedline_terminations` is at most `launcher_target`, since every feedline
  chain has a launcher at one end at least;
- the resonators of the ring are at most the feedline chains,
  (`launcher_target` + `feedline_terminations`) / 2, times
  `[design_rules] max_feedline_utilization`.

The global step decides whether a ring port at a bridge stays in the ring and
whether it is a resonator. `plan` and `doctor` refuse before the first stage a
configuration that breaks a limit for every such decision.

The assignment also refuses a solution in which two feeds lie closer than
`[design_rules] min_wire_spacing`, such as two conventional ports at launcher
slots in one cell. It refuses two feeds that round onto one cell of the detail
grid as well, since the Corridor step starts each wire at that cell; a detail
cell narrower than about 0.7 wire spacings rules this out.

## `[stages.solver]`

| Key            | Default | Meaning                                                                                       |
| -------------- | ------- | --------------------------------------------------------------------------------------------- |
| `backend`      | `""`    | `highs` or `gurobi`; empty takes `highs`. The environment variable `SCPD_SOLVER` overrides it |
| `time_limit`   | `0.0`   | The seconds one solve may take; `0.0` sets no limit; not negative                             |
| `relative_gap` | `0.0`   | The relative gap at which a solve may stop; `0.0` keeps the gap of the backend; not negative  |
| `threads`      | `0`     | The threads one solve may use; `0` keeps the default of the backend; at most `1024`           |

A solve that the report calls `optimal` is optimal within the relative gap: its
objective and the best bound the solver proved differ by at most that fraction
of the objective. With `relative_gap` at `0.0`, the gap is the default of the
backend, which is `1e-4` for HiGHS (`mip_rel_gap`) and for Gurobi (`MIPGap`). A
solve also stops when the objective and the bound differ by less than the
absolute gap of the backend: `1e-6` for HiGHS (`mip_abs_gap`) and `1e-10` for
Gurobi (`MIPGapAbs`). For an objective near zero, the absolute gap decides.

HiGHS and Gurobi can report different optimal solutions: their objectives can
differ by up to the relative gap, and between solutions of equal objective each
backend can choose another one. A later stage then works on another input and
can reach another objective.

A solve that stops at `time_limit` keeps the best solution it has found by then.
That solution depends on the speed of the machine, so the same inputs can give
different artifacts. Leave `time_limit` at `0.0` where a run has to repeat. Such
a solution of the Global stage can hold wires that reach no target: a wire
between the inner ends of two ring bridges, a closed loop through internal
bridges, or a closed loop over crossings. The Global artifact and its objective
leave these wires out.

With `threads` at `0`, HiGHS uses half the hardware threads of the machine, and
Gurobi chooses the count itself (its parameter `Threads`). The count changes the
order of the search, so the time of a solve, its node count and, between optimal
solutions of equal objective, its answer can differ from one machine to the
next. A fixed `threads` makes runs on different machines comparable. The limit
of `1024` is the largest value that Gurobi accepts.

## `[stages.corridor]`

| Key              | Default | Meaning                                                                                                                                                                |
| ---------------- | ------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `router`         | `""`    | The implementation of the step; empty takes `partition-astar`                                                                                                          |
| `rounds`         | `12`    | The most sweeps over the connections, in turn forward and backward; at least `1`                                                                                       |
| `max_relaxation` | `4`     | How many neighboring connections along a sweep a connection without a way may rip up, and how many wires in its way it may move aside; the stage limits it (see below) |

The later sweeps let a connection without a way take room that another
connection has not claimed yet. A sweep can take a way from a connection that
had one, so the fails of the rounds can go up as well as down. The stage keeps
the plan of the round with the fewest fails, and goes back to it when the last
round ends with more. The fails of a round count every connection without a way
at its end, also one that the round ripped up after its sweep had passed it and
so did not place again. After the rounds, one last pass offers every connection
without a way the free slots of that plan, and rips up nothing; the stage block
shows it as `free slots` when it places a connection. Within a sweep, a
connection without a way also raises the price of the slots in its way that
another wire holds, so that the next sweeps leave them to it. The step that
moves the wires in its way aside raises the price, so with `max_relaxation` at
`0`, no price rises.

The stage limits `max_relaxation` to a value below half the number of
connections. A connection that may rip up all the others takes their room, they
find no way back, and every round repeats this. With two or more connections,
the limit is at least `1`, so that two connections that want one slot can still
move each other aside. With the default of `4`, the eight benchmark chips route
every wire within two rounds. `2` is the smallest value that routes every wire
there, in up to four rounds. A value above `4` routes them in two rounds as
well, but each round takes longer.

## `[stages.detail]`

The Detail stage draws every corridor of the Corridor stage as a path of cells
on the detail grid, with an eight-connected A\* search inside the corridor.
Every length is in layout units unless the key says `cells`. The wire clearance
of the stage is `min_wire_spacing` converted onto the grid, and no key sets it.
{doc}`routing` describes the stage.

| Key                      | Default | Meaning                                                                                                                 |
| ------------------------ | ------- | ----------------------------------------------------------------------------------------------------------------------- |
| `router`                 | `""`    | The implementation of the stage; empty takes `pixel-astar`                                                              |
| `corridor_spacings`      | `4`     | How far to either side of its own way a wire may move when it is drawn again across the borders, in wire spacings       |
| `rounds`                 | `30`    | The most sweeps over the wires, in turn forward and backward                                                            |
| `max_relaxation`         | `30`    | How many neighbouring wires a wire without a way may let go of, along the sweep first and then against it               |
| `obstacle_penalty_reach` | `185.0` | How far the price of running beside an obstacle reaches; `0.0` turns the price off                                      |
| `obstacle_penalty`       | `10`    | What a step right beside an obstacle costs, against the 10 a step along an axis costs; it falls to nothing at the reach |
| `orderings`              | `16`    | How many orders the pieces of one partition are tried in; `1` keeps the given order                                     |

## `[stages.final]`

The Final stage draws every wire as curvature-constrained copper on the router
grid, inserts the couplers, routes the feedline chains and refines them. A cell
is a router cell, whose size `[grid] router_cell_size` sets. {doc}`routing`
describes the five phases, the coupler insertion with its checks and what each
key does.

| Key                             | Default              | Meaning                                                                                                                                                |
| ------------------------------- | -------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `router`                        | `""`                 | The implementation of the stage; empty takes `dubins`                                                                                                  |
| `corridor_spacings`             | `11`                 | The band to either side of its way a ring wire may move in, in wire spacings                                                                           |
| `inner_corridor_spacings`       | `11`                 | The same for the inner circuit                                                                                                                         |
| `rounds`                        | `30`                 | The most sweeps of the outer routing and the feedline routing                                                                                          |
| `inner_rounds`                  | `4`                  | The most sweeps of the inner routing                                                                                                                   |
| `max_relaxation`                | `10`                 | How many neighbouring wires a wire without a way may let go of                                                                                         |
| `refinement_rounds`             | `2`                  | The rounds of the pre-feedline refinement; `0` skips it                                                                                                |
| `feedline_refinement_rounds`    | `2`                  | The rounds of the feedline-constrained refinement; `0` skips it                                                                                        |
| `refinement_corridor_spacings`  | `6`                  | The band of both refinements, in wire spacings; `0` takes `corridor_spacings`                                                                          |
| `bend_penalty_norm`             | `2.5`                | What one eighth turn costs, in hundredths of a cell per unit of the grid's width plus height                                                           |
| `wire_proximity_penalty_norm`   | `0.00125`            | What running beside another wire costs, on the same scale                                                                                              |
| `static_proximity_penalty_norm` | `0.00033`            | What running beside an obstacle costs, on the same scale                                                                                               |
| `obstacle_penalty_reach`        | `100.0`              | How far the price of running beside an obstacle reaches; `0.0` turns the price off                                                                     |
| `coupler_length`                | `200.0`              | The run of the pad the resonator couples along                                                                                                         |
| `coupler_height`                | `26.0`               | How far the feedline lies from the resonator across the pad                                                                                            |
| `repair_trials`                 | `100`                | How many legality tests the targeted repair may run per chip; `0` turns the repair off                                                                 |
| `coupler_bias`                  | `1.0`                | Where the coupler cuts the way, as the share of the target length left to run from the coupler to the qubit                                            |
| `coupler_max_shortfall`         | `0.3`                | How far below the target length a coupler may leave the resonator, as a share of it                                                                    |
| `coupler_box_margin_cells`      | `5`                  | What a coupler keeps off every side of the coupler box, in cells                                                                                       |
| `coupler_shift_cells`           | `20`                 | How far an option that leaves the box is moved into it before the place is given up, in cells                                                          |
| `coupler_lead_margin_cells`     | `5`                  | The straight run of the lead beyond `min_straight_length`, in cells                                                                                    |
| `launcher_lead`                 | `10.0`               | How much further than its straight run a launcher keeps the coupler box away                                                                           |
| `coupler_offsets`               | `""`                 | The orientations of an option, as a comma list of eighth turns `0` to `7` off the nearest launcher's heading; empty takes every offset but `3` and `4` |
| `edge_launcher_margin_cells`    | `6`                  | What a feedline edge keeps closed in front of every launcher but its own, beyond the launcher's run, in cells                                          |
| `terminal_slot_cells`           | `20`                 | The straight slot a feedline edge keeps open at a port, in cells                                                                                       |
| `chain_end_edge_weight`         | `0.1`                | The weight of an edge at a launcher in the chain search, against `1` for every other edge                                                              |
| `edge_bend_factor`              | `1.0`                | The bend price of a feedline edge, as a factor on `bend_penalty_norm`                                                                                  |
| `chain_length_weight`           | `0`                  | The price of one cell of an edge's length in the chain search; `0` prices length not at all                                                            |
| `capacity_check`                | `true`               | Whether the insertion runs the step checks, the whole-chip check and the global SAT search; off, the chains are settled by cost alone                  |
| `length_point_radius_cells`     | `40`                 | What a wire keeps from the band where a resonator reaches its target length, in cells; `0` turns it off                                                |
| `length_point_band_percent`     | `10.0`               | The half-width of that band, as a percentage of the target length                                                                                      |
| `sweep_length_band`             | `5.0`                | The band of lengths the feedline sweep accepts, as a multiple of `resonator_length_tolerance`                                                          |
| `meander_leg_spacing_cells`     | `25`                 | What the two legs of a meander loop keep between them, in cells                                                                                        |
| `meander_start_margin_cells`    | `-1`                 | How many cells at the start of a resonator the meander leaves alone; negative takes the coupler's run plus four                                        |
| `capacity_margin`               | `1.06`               | The factor on the clearance a gate counts with, from `capacity_margin_from_wires` wires on                                                             |
| `capacity_margin_from_wires`    | `3`                  | The fewest wires the factor applies to                                                                                                                 |
| `resonator_passage`             | `0.8`                | The narrowest gate a resonator passes at its own port run, in clearances                                                                               |
| `crossing_reach_cells`          | `10`                 | The band beside a feedline edge where a wire may cross only at a right angle, in cells                                                                 |
| `crossing_spacing`              | `1.4142135623730951` | The spacing of the crossing points of a stretch, in clearances                                                                                         |
| `slot_pin_cells`                | `10`                 | The cells at each end of an edge where no slot lies                                                                                                    |
| `escape_straight_cells`         | `5`                  | The straight run a slot's escape may take, in cells                                                                                                    |
| `escape_stub_margin_cells`      | `4`                  | What an escape keeps from the walls of a port run, in cells                                                                                            |
| `bottleneck_wires`              | `10`                 | The widest gap, in wires, that is still a bottleneck                                                                                                   |
| `bottleneck_rise`               | `2.0`                | The least rise of a gap profile on both sides of a minimum, in cells                                                                                   |
| `room_margin_cells`             | `10`                 | The dead length at both ends of a straight run in the room report, in cells                                                                            |
| `room_channel_reach_cells`      | `80`                 | The reach of the channel rule in the room report, in cells                                                                                             |
| `detour_factor`                 | `1.5`                | How much longer than its shortest way a demand may run through the graph                                                                               |
| `terminal_stub_extra_cells`     | `5`                  | How far the coupler run of a terminal edge is guarded past the pad, in cells                                                                           |
| `global_sat_runs`               | `8`                  | The runs of options the global SAT search collects per segment                                                                                         |
| `global_sat_states`             | `8`                  | The locally SAT states it keeps per chain outside the focus                                                                                            |
| `global_sat_tries`              | `200`                | The whole-chip combinations it tries                                                                                                                   |
| `chain_rule_refusals`           | `1500`               | How many steps the capacity rule may refuse before a run of options joins a chain; then the chain is searched again without the rule                   |
| `research_ring_members`         | `2`                  | How many ring members beside each window wire the repair draws again                                                                                   |
| `research_rounds`               | `2`                  | The rounds of the repair's local pass                                                                                                                  |
| `fence_pairs`                   | `3`                  | How many ring neighbours to either side a wire is fenced against in the feedline routing and the refinements                                           |
| `halo_reach`                    | `1`                  | The halo of a wire let go of, in clearances                                                                                                            |
| `rise_reach`                    | `3`                  | Where the rising price past the halo reaches its peak, in clearances                                                                                   |
| `rise_peak`                     | `3`                  | The peak of the rising price, as a factor on the flat price                                                                                            |

## `[debug]`

The pages a run writes under `<run>/debug`, each off unless asked for;
`mqt-scpd plan -d` turns every page on. The Final stage records what the pages
need and nothing else, and the pages are built from the records when the run
ends and again by `mqt-scpd debug <run>`. {doc}`routing` describes the pages.

| Key                                         | Default | Meaning                                                                                                                              |
| ------------------------------------------- | ------- | ------------------------------------------------------------------------------------------------------------------------------------ |
| `html`                                      | `false` | Write `index.html`, which links every page below and lists what each phase came to                                                   |
| `pre_feedline_routing_dashboard`            | `false` | A table of the pre-feedline routing: a row per wire, a column per round and relaxation level; a click draws the way the search found |
| `pre_feedline_refinement_dashboard`         | `false` | The same table for the pre-feedline refinement                                                                                       |
| `feedline_insertion`                        | `false` | The coupler options and the capacity graph of the whole-chip check, on the page the graph is laid out on                             |
| `local_sat`                                 | `false` | Every step check of the chain search: the chain, the step, the options, the verdict and the wires it closes                          |
| `global_sat`                                | `false` | The global SAT search: the states collected per chain, every combination with its verdict, and the result                            |
| `feedline_constrained_routing_dashboard`    | `false` | The table for the feedline-constrained routing                                                                                       |
| `feedline_constrained_refinement_dashboard` | `false` | The table for the feedline-constrained refinement                                                                                    |

## `[run]`

| Key          | Default | Meaning                                                                                                                                                                                                                                                                                  |
| ------------ | ------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `stop_after` | `""`    | The last step of a run: `capacity`, `global`, `assign`, `corridor`, `detail` or `final`, or a phase of the Final stage (`pre-feedline-routing`, `pre-feedline-refinement`, `feedline-insertion`, `feedline-constrained-routing`, `feedline-constrained-refinement`); empty runs them all |

`mqt-scpd list-algorithms` names the implementations that a build ships for each
step. `--stop-after` replaces `stop_after` for one run.
