# Benchmark chips

One directory per benchmark chip, each holding the chip's `config.toml` and its
input `routing_config.json`. The inputs are the prototype's files in the same
format; see
[decision 0020](../docs/design/decisions/0020-legacy-routing-config-as-input.md).
On 2026-09-08 they were reduced to the routable ports and a consolidated
obstacle set, so every port of every chip matches one of the three role
patterns.

## What the configurations carry

**Role patterns.** Two naming schemes exist: the 4-qubit chip names its
components `Q1` and `C12` with three ports per coupler, every other chip `Qb1`
and `Coupler1_2` with five. Every qubit carries the readout port `port0` and the
drive port `port1`.

**Port sequences.** `all_outer` and `fixed_outer` are the literals of the
prototype's drivers, `all_outer_ports` and `fixed_outer_ports`, copied verbatim.
The 4-qubit driver carries one list, `all_ports`, and passes it as both. The
9-qubit driver's fixed literal omits `Qb4` and `Coupler1_4`, which its own
`all_outer` lists. Loading checks every label against the chip; see
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
driver's `CapacityGrid` constructor call. The 17-qubit chip is the one that sets
a cell count in both directions and an offset of 20; the 69-qubit chip runs on
the defaults and therefore has no `[grid]` section.

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
9-qubit driver pins every routable port in the capacity stage and nothing in the
final stage; its `fixed_outer_ports` literal is a dead variable there. The
17-qubit driver pins `fixed_outer` in both stages. The five larger drivers pin
nothing in either stage, although each carries a `fixed_outer_ports` literal.
The phase that ports the capacity stage decides how a run reproduces this; the
configurations record the literals so that the decision has its input.

The 17-qubit driver's `all_outer` enters the ring at `Qb15`; the sequence itself
carries that rotation, and the assignment consumes it in order.

## Checks

```bash
mqt-scpd doctor -c benchmarks/9q/config.toml
mqt-scpd plot -c benchmarks/9q/config.toml --stage layout -o 9q.svg
mqt-scpd render -c benchmarks/9q/config.toml -o 9q.gds
```
