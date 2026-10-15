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

| Key                 | Default | Meaning                                                                                    |
| ------------------- | ------- | ------------------------------------------------------------------------------------------ |
| `capacity_cells_x`  | `50`    | The cells of the capacity grid across the width of the chip; at least one                  |
| `capacity_cells_y`  | `0`     | The cells of the capacity grid across the height of the chip; `0` takes the aspect ratio   |
| `detail_factor`     | `30`    | The detail cells per capacity cell along each axis; at least one                           |

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
internal bridges on the way. No ring port supplies an inner wire, because the
assignment gives every ring port a wire of its own. An inner port that no such
wire can reach is a fail of the global step. An inner resonator that is a fail
gets no feed.

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
backend, which is `1e-4` for HiGHS (`mip_rel_gap`) and for Gurobi (`MIPGap`).

HiGHS and Gurobi reach the same optimum, but between optimal solutions of equal
objective each can choose another one. A later stage then works on another input
and can reach another objective.

A solve that stops at `time_limit` keeps the best solution it has found by then.
That solution depends on the speed of the machine, so the same inputs can give
different artifacts. Leave `time_limit` at `0.0` where a run has to repeat.

With `threads` at `0`, HiGHS uses half the hardware threads of the machine, and
Gurobi chooses the count itself (its parameter `Threads`). The count changes the
order of the search, so the time of a solve, its node count and, between optimal
solutions of equal objective, its answer can differ from one machine to the
next. A fixed `threads` makes runs on different machines comparable. The limit
of `1024` is the largest value that Gurobi accepts.

## `[stages.corridor]`

| Key              | Default | Meaning                                                                                                                                 |
| ---------------- | ------- | --------------------------------------------------------------------------------------------------------------------------------------- |
| `router`         | `""`    | The implementation of the step; empty takes `partition-astar`                                                                           |
| `rounds`         | `12`    | The most sweeps over the connections, in turn forward and backward                                                                      |
| `max_relaxation` | `30`    | How many neighbouring connections along the sweep a connection without a way may rip up, and then how many wires in its way it may move |

A connection that has a way keeps it. The later sweeps let a connection without
a way take room that another connection has not claimed yet. After the sweeps,
two passes run, also with `rounds` at `0`. The rescue pass gives each connection
still without a way free room where it finds some, and takes nothing from
anyone. The exchange pass lets such a connection move the wires in its way, and
keeps the change only when fewer connections are left without a way.

## `[run]`

| Key          | Default | Meaning                                                                                   |
| ------------ | ------- | ----------------------------------------------------------------------------------------- |
| `stop_after` | `""`    | The last step of a run: `capacity`, `global`, `assign` or `corridor`; empty runs them all |

`mqt-scpd list-algorithms` names the implementations that a build ships for each
step. `--stop-after` replaces `stop_after` for one run.
