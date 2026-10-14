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
| `launcher_offset_x` | `15`    | The detail cells along the left and right edges of the grid that stay blocked              |
| `launcher_offset_y` | `15`    | The detail cells along the top and bottom edges of the grid that stay blocked              |
| `detail_factor`     | `30`    | The detail cells per capacity cell along each axis; at least one                           |

A launcher that sits on the edge of the chip needs `launcher_offset_x` and
`launcher_offset_y` small enough that the blocked border does not cover it.

## `[stages.capacity]`

| Key                    | Default | Meaning                                                                                                                                                 |
| ---------------------- | ------- | ------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `planner`              | `""`    | The implementation of the step; empty takes `watershed`                                                                                                 |
| `bottleneck_clearance` | `1.5`   | A place is a bottleneck only where its free radius is below this many wire pitches; a pitch is `min_wire_spacing` plus `min_obstacle_spacing`; positive |
| `crossing_pitch`       | `165.0` | The distance between two places where a wire may cross a partition border; it decides how many wires a border lets through; positive                    |

## `[stages.global]`

| Key                | Default | Meaning                                                                                                    |
| ------------------ | ------- | ---------------------------------------------------------------------------------------------------------- |
| `router`           | `""`    | The implementation of the step; empty takes `hanan-milp`                                                   |
| `internal_bridges` | `false` | Whether an inner wire may cross a component that the outer ring does not reach, through its own bridges    |

## `[stages.assignment]`

| Key               | Default | Meaning                                                                                                                    |
| ----------------- | ------- | -------------------------------------------------------------------------------------------------------------------------- |
| `assigner`        | `""`    | The implementation of the step; empty takes `ordered-milp`                                                                 |
| `launcher_target` | `0`     | How many feedline chain ends stop at a launcher; `0` leaves it unset, and `plan` refuses a run that reaches the assignment |

`launcher_target` plus `[design_rules] feedline_terminations` must be even,
since every feedline chain has two ends.

## `[stages.solver]`

| Key            | Default | Meaning                                                                                       |
| -------------- | ------- | --------------------------------------------------------------------------------------------- |
| `backend`      | `""`    | `highs` or `gurobi`; empty takes `highs`. The environment variable `SCPD_SOLVER` overrides it |
| `time_limit`   | `0.0`   | The seconds one solve may take; `0.0` sets no limit; not negative                             |
| `relative_gap` | `0.0`   | The relative gap at which a solve may stop; `0.0` keeps the gap of the backend; not negative  |

## `[stages.corridor]`

| Key              | Default | Meaning                                                                                       |
| ---------------- | ------- | --------------------------------------------------------------------------------------------- |
| `router`         | `""`    | The implementation of the step; empty takes `partition-astar`                                 |
| `rounds`         | `12`    | The most sweeps over the connections, in turn forward and backward                            |
| `max_relaxation` | `30`    | How many neighbouring connections a connection without a way may rip up, along the sweep      |

A connection that has a way keeps it. The later sweeps let a connection without
a way take room that another connection has not claimed yet.

## `[run]`

| Key          | Default | Meaning                                                                                   |
| ------------ | ------- | ----------------------------------------------------------------------------------------- |
| `stop_after` | `""`    | The last step of a run: `capacity`, `global`, `assign` or `corridor`; empty runs them all |

`mqt-scpd list-algorithms` names the implementations that a build ships for each
step. `--stop-after` replaces `stop_after` for one run.
