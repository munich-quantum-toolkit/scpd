# Benchmark chips

One directory per benchmark chip. Each holds the chip's `config.toml` and, for
the two smallest chips, its input. The inputs are the prototype's own
`routing_config.json` files, unchanged, which is what makes the routing
baseline directly comparable; see
[decision 0020](../docs/design/decisions/0020-legacy-routing-config-as-input.md).

| Chip  | Qubits | Prototype checkout | Input file                              |   Size | Committed |
| ----- | -----: | ------------------ | --------------------------------------- | -----: | --------- |
| `4q`  |      4 | FridgeCAD          | `4Q_layout/routing_config_4q.json`      | 0.8 MB | yes       |
| `9q`  |      9 | FridgeCAD          | `9Q_layout/routing_config_9q.json`      | 2.4 MB | yes       |
| `17q` |     17 | FridgeCAD          | `src/routing_config.json`               | 6.4 MB | no        |
| `21q` |     21 | FridgeCAD-0fails   | `21Q_layout/routing_config_21q.json`    | 5.3 MB | no        |
| `33q` |     33 | FridgeCAD-0fails   | `33Q_layout/routing_config_33q.json`    |  16 MB | no        |
| `45q` |     45 | FridgeCAD-0fails   | `45Q_layout/routing_config_45q.json`    |  22 MB | no        |
| `57q` |     57 | FridgeCAD-0fails   | `57Q_layout/routing_config_57q.json`    |  27 MB | no        |
| `69q` |     69 | FridgeCAD-0fails   | `69Q_layout/routing_config_69q_new.json` |  17 MB | no        |

The table names the file each prototype driver loads at its last commit
(`0708436`, 2026-09-05). Every configuration names its input as
`routing_config.json` next to it. To work with one of the six larger chips,
copy its file there; `.gitignore` keeps the copies out of the repository.

## What the configurations carry

**Role patterns.** Two naming schemes exist: the 4-qubit chip names its
components `Q1` and `C12`, every other chip `Qb1` and `Coupler1_2`. Five
inputs, `9q`, `17q`, `33q`, `45q` and `57q`, carry the coupler-mating ports
`Qb*.port2` to `port5` and `Coupler*.port5` to `port6` besides the routable
ports; their configurations classify those as `mating`. The `21q` and `69q`
inputs carry routable ports only.

**Port sequences.** `all_outer` and `fixed_outer` are the literals of the
prototype's drivers, `all_outer_ports` and `fixed_outer_ports`, copied
verbatim. The 4-qubit driver carries one list, `all_ports`, and passes it as
both. The 9-qubit driver's fixed literal omits `Qb4` and `Coupler1_4`, which
its own `all_outer` lists. Loading checks every label against the chip; see
[decision 0025](../docs/design/decisions/0025-port-ring-is-manual-input.md).

**Design rules.** The five lengths are the same on every chip: 185, 25, 50 and
100 layout units, a resonator target of 2500 with a tolerance of 100. The two
counts come from each driver's assignment call:

| Chip  | `max_feedline_utilization` | `feedline_terminations` |
| ----- | -------------------------: | ----------------------: |
| `4q`  |                          4 |                       0 |
| `9q`  |                          5 |                       1 |
| `17q` |                          5 |                       1 |
| `21q` |                          5 |                       0 |
| `33q` |                          5 |                       0 |
| `45q` |                          6 |                       1 |
| `57q` |                          7 |                       0 |
| `69q` |                          6 |                       0 |

**Grid.** The capacity grid's cell count and launcher offset come from each
driver's `CapacityGrid` constructor call. The 17-qubit chip is the one that
sets a cell count in both directions and an offset of 20; the 69-qubit chip
runs on the defaults and therefore has no `[grid]` section.

## What the drivers do that no key carries yet

These values arrive with the stages that read them.

| Chip  | Assignment `launcher_target` | Capacity grid detail factor |
| ----- | ---------------------------: | --------------------------: |
| `4q`  |                            2 |                          30 |
| `9q`  |                            3 |                          30 |
| `17q` |                            7 |                          20 |
| `21q` |                           10 |                          30 |
| `33q` |                           14 |                          30 |
| `45q` |                           15 |                          30 |
| `57q` |                           18 |                          30 |
| `69q` |                           24 |                          30 |

The drivers do not agree on what they pin as fixed ports. The 4-qubit driver
pins `all_outer` in the capacity stage and nothing in the final stage. The
9-qubit driver pins every routable port in the capacity stage and nothing in
the final stage; its `fixed_outer_ports` literal is a dead variable there. The
17-qubit driver pins `fixed_outer` in both stages. The five larger drivers pin
nothing in either stage, although each carries a `fixed_outer_ports` literal.
The phase that ports the capacity stage decides how a run reproduces this; the
configurations record the literals so that the decision has its input.

The 17-qubit driver's `all_outer` enters the ring at `Qb15`; the sequence
itself carries that rotation, and the assignment consumes it in order.

## Checks

```bash
mqt-scpd doctor -c benchmarks/9q/config.toml
mqt-scpd plot -c benchmarks/9q/config.toml --stage layout -o 9q.svg
mqt-scpd render -c benchmarks/9q/config.toml -o 9q.gds
```
