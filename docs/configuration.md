# Configuration Reference

Run `mqt-scpd doctor -c config.toml` to check a chip and its configuration.
Lengths are in µm. A dash in the default column marks a required key. MQT SCPD
rejects unknown sections and keys.

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

The `component` pattern must match every `bridge_pair` port. Resonator,
conventional, and bridge ports must face a multiple of 45° so their approaches
follow a grid axis or diagonal. `plan` and `doctor` check this before Capacity.

## `[ports.sequences]`

| Key           | Default | Meaning                                                                         |
| ------------- | ------- | ------------------------------------------------------------------------------- |
| `all_outer`   | —       | The outer port ring, in the order the assignment walks it                       |
| `fixed_outer` | —       | The ports of `all_outer` that a run pins; the order does not matter             |

## `[[ports.bridge_pairs]]`

Each rule pairs the two `bridge_pair` ports that a wire crosses a component
between. Both sides are regular expressions with exactly one capture group. Two
ports pair when they belong to one component and their captures are equal. Each
component and capture must match one port per side of its rule. The pairs follow
the rule order in the file.

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

| Key                | Default | Meaning                                                                                                                   |
| ------------------ | ------- | ------------------------------------------------------------------------------------------------------------------------- |
| `capacity_cells_x` | `50`    | The cells of the capacity grid across the width of the chip; at least two                                                 |
| `capacity_cells_y` | `0`     | The cells of the capacity grid across the height of the chip; `0` takes the aspect ratio, any other value is at least two |
| `detail_factor`    | `30`    | The detail cells per capacity cell along each axis; at least one                                                          |

Remove the deprecated `launcher_offset_x` and `launcher_offset_y` keys. `doctor`
and `plan` warn when these keys are present.

Multiply each capacity grid dimension by `detail_factor` to get the detail grid.
The limit is 25,000,000 detail cells. Capacity uses about 120 bytes per cell, or
about 3 GB at the limit. The 69q benchmark uses 1500 × 1500 cells.

The longer detail cell step may exceed the shorter one by up to 10%. Use
`capacity_cells_y = 0` to derive the height from the chip aspect ratio. Increase
`capacity_cells_x` if rounding still puts the grid outside this limit.

## `[stages.capacity]`

| Key                    | Default | Meaning                                                                                                                                                 |
| ---------------------- | ------- | ------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `planner`              | `""`    | The implementation of the step; empty takes `watershed`                                                                                                 |
| `bottleneck_clearance` | `1.5`   | A place is a bottleneck only where its free radius is below this many wire pitches; a pitch is `min_wire_spacing` plus `min_obstacle_spacing`; positive |
| `crossing_pitch`       | `165.0` | Minimum spacing between border crossings of different wires; positive                                                                                   |

Capacity counts the crossing slots on each partition border. Corridor uses the
same slots and enforces the crossing pitch across different borders. A crossing
can therefore block a slot on a neighboring border.

## `[stages.global]`

| Key                | Default | Meaning                                                                                                    |
| ------------------ | ------- | ---------------------------------------------------------------------------------------------------------- |
| `router`           | `""`    | The implementation of the step; empty takes `hanan-milp`                                                   |
| `internal_bridges` | `false` | Whether an inner wire may cross a component that the outer ring does not reach, through its own bridges    |

Each inner wire starts at the inner end of a ring bridge and surfaces at its
ring end. It can cross other bridges when `internal_bridges` is enabled. Global
reports unreachable inner ports as fails. An unreachable inner resonator gets no
feed.

For each port in a lattice, Global uses a point `min_straight_length` in front
of the port. Two ports that can carry inner wires on the same capacity chain
must have distinct points. A port without an inner wire may share a point. The
`doctor` preflight leaves this check to Global.

A gate's budget is shared by all chains that cross it. Ring ports and inner
wires that surface on the ring both use that budget. When the budget falls
short, Global can solve a second model that leaves some ring ports out. Global
counts these ports as fails and lists them at `-vv`.

Global checks gates with room for some ring ports only when it solves an inner
circuit, and only on chains without inner ports. A ring port without a bridge
always needs a path to a launcher, even on a chip without inner ports. A ring
bridge needs a path when an inner wire surfaces there.

## `[stages.assignment]`

| Key               | Default | Meaning                                                                                                                    |
| ----------------- | ------- | -------------------------------------------------------------------------------------------------------------------------- |
| `assigner`        | `""`    | The implementation of the step; empty takes `ordered-milp`                                                                 |
| `launcher_target` | `0`     | How many feedline chain ends stop at a launcher; `0` leaves it unset, and `plan` refuses a run that reaches the assignment |

For a ring with resonators, `launcher_target` and `feedline_terminations` count
the ends of its feedline chains. Apply these rules.

- The total is even. The number of chains equals half the total.
- Conventional ring ports and launcher ends together fit on the chip's
  launchers.
- `launcher_target` and `feedline_terminations` are each at most the number of
  resonators on the ring.
- Terminations cannot outnumber launcher ends. Each chain needs a launcher.
- The number of resonators must fit within the number of chains times
  `max_feedline_utilization`.

Global decides which bridge ports stay on the ring and which count as
resonators. `plan` and `doctor` reject configurations that violate a limit for
every possible ring. Assignment also requires feed points to respect
`min_wire_spacing` and map to distinct detail cells. Refine the detail grid if
distinct feeds round to the same cell. Corridor uses those cells as its wire
starts.

## `[stages.solver]`

| Key            | Default | Meaning                                                                                       |
| -------------- | ------- | --------------------------------------------------------------------------------------------- |
| `backend`      | `""`    | `highs` or `gurobi`; empty takes `highs`. The environment variable `SCPD_SOLVER` overrides it |
| `time_limit`   | `0.0`   | The seconds one solve may take; `0.0` sets no limit; not negative                             |
| `relative_gap` | `0.0`   | The relative gap at which a solve may stop; `0.0` keeps the gap of the backend; not negative  |
| `threads`      | `0`     | The threads one solve may use; `0` keeps the default of the backend; at most `1024`           |

Install the Gurobi backend with `uv pip install "mqt-scpd[gurobi]"`. Gurobi has
no Windows ARM64 wheel. The bundled license allows up to 2,000 variables and
2,000 constraints per linear model. Larger models need a full license. Run
`mqt-scpd doctor` with the configuration to check the backend before planning.

`optimal` means the gap between the best solution and the best bound is within
the solver's tolerance. Both backends default to a relative gap of `1e-4`
(`mip_rel_gap` in HiGHS and `MIPGap` in Gurobi). Their absolute gaps are `1e-6`
(`mip_abs_gap`) and `1e-10` (`MIPGapAbs`), respectively. The absolute gap
matters near zero. Backends can pick different solutions within the tolerance or
break ties differently. That can change the next stage's inputs.

At `time_limit`, the solver returns its best feasible solution if it found one.
The limit applies to each model. Global can solve two models when port capacity
falls short, and reports when it used a solution from an early stop. Its gap
applies to a model objective that includes penalties for missing ports. Its
reported wire length counts only wires that reach a target.

With `threads = 0`, the bundled HiGHS uses the physical cores available to the
process, and Gurobi selects its own count. Thread counts and time limits can
change the solution. Use a fixed thread count and leave `time_limit` at `0.0`
for more comparable runs.

HiGHS supports concurrent solves from separate calling threads. A callback
cannot start a nested solve. To change the thread count on a calling thread, let
any other active solves in MQT SCPD finish first.

## `[stages.corridor]`

| Key              | Default | Meaning                                                       |
| ---------------- | ------- | ------------------------------------------------------------- |
| `router`         | `""`    | The implementation of the step; empty takes `partition-astar` |
| `rounds`         | `12`    | Maximum number of routing sweeps; at least one                |
| `max_relaxation` | `4`     | Maximum number of connections to remove per reroute attempt   |

Corridor alternates forward and backward sweeps. It can remove routes to make
room and raises the cost of occupied slots for later sweeps. The stage keeps the
round with the fewest unrouted connections, then tries the remaining connections
using free slots only. It reports a `free slots` entry if that pass routes a
connection.

With fewer than two connections, Corridor uses zero relaxation. Otherwise, it
caps `max_relaxation` at half the number of other connections, rounded down,
with a minimum cap of one. Set `max_relaxation = 0` to disable route removal and
slot cost increases.

Corridor checks wire intersections with straight segments inside each partition.
This check is exact for convex partitions. Routing can leave a connection
unrouted even if a feasible route exists.

## `[run]`

| Key          | Default | Meaning                                                                                   |
| ------------ | ------- | ----------------------------------------------------------------------------------------- |
| `stop_after` | `""`    | The last step of a run: `capacity`, `global`, `assign` or `corridor`; empty runs them all |

`mqt-scpd list-algorithms` names the implementations that a build ships for each
step. `--stop-after` replaces `stop_after` for one run.
