# Benchmark chips

This directory holds the eight chips that MQT SCPD is developed and measured
against. They range from 4 to 69 qubits and cover the shapes the router has to
handle, from a single square of four qubits to a lattice with notched arms.

Each chip lives in its own directory and consists of two files:

| File                  | What it is                                                                   |
| --------------------- | ---------------------------------------------------------------------------- |
| `routing_config.json` | The chip itself: the obstacle polygons and the named ports, in layout units  |
| `config.toml`         | How to route it: the port roles, the outer port ring, the design rules       |

## Trying a chip out

```bash
mqt-scpd doctor -c benchmarks/9q/config.toml                        # check it
mqt-scpd plot -c benchmarks/9q/config.toml --stage layout -o 9q.svg # look at it
mqt-scpd render -c benchmarks/9q/config.toml -o 9q.gds              # export it
```

`doctor` is the one to run first. It loads the chip, prints how many ports fell
into each role and which outer port ring the run would use, and it fails with a
message naming the offending key when the configuration and the chip do not fit
together.

## The chips

| Chip  | Qubits | Obstacles | Ports | Outer ring | Pinned ports |
| ----- | -----: | --------: | ----: | ---------: | -----------: |
| `4q`  |      4 |        28 |    36 |         12 |           12 |
| `9q`  |      9 |        49 |   102 |         40 |           21 |
| `17q` |     17 |        95 |   202 |         76 |           44 |
| `21q` |     21 |       133 |   254 |        110 |           62 |
| `33q` |     33 |       193 |   382 |        182 |          102 |
| `45q` |     45 |       253 |   510 |        255 |          143 |
| `57q` |     57 |       313 |   646 |        323 |          179 |
| `69q` |     69 |       385 |   790 |        395 |          219 |

All eight come from the FridgeCAD prototype and keep its file format, so that
routing results can be compared with it directly. They were reduced to the ports
a wire can actually end at, plus a consolidated obstacle set.

## Reading a configuration

A configuration carries only what differs from the defaults, with one exception:
the design rules are always written out in full, because they are the physical
contract a routing result is judged against.

**`[ports.patterns]`** gives one regular expression per role, and every port of
the chip must match exactly one of them:

```toml
launcher     = '^Chip\.port\d+$'
resonator    = '^Qb\d+\.port0$'
conventional = '^(Qb\d+\.port1|Coupler\d+_\d+\.port[0-4])$'
```

Two naming schemes are in use. The 4-qubit chip names its components `Q1` and
`C12` and gives each coupler three ports; the other seven use `Qb1` and
`Coupler1_2` with five. On every chip a qubit carries its readout port as
`port0` and its drive port as `port1`.

**`[ports.sequences]`** carries the ring of outer ports. `all_outer` is the ring
the assignment walks, in order, and the point at which it starts matters: the
17-qubit chip enters at `Qb15`, and its sequence is written that way.
`fixed_outer` is the subset a run pins as an obstacle, and it is only ever read
as a set. Both sequences come from the prototype's drivers unchanged. The
9-qubit list is the one oddity: its `fixed_outer` omits `Qb4` and `Coupler1_4`,
which its own `all_outer` does list. Loading checks every label against the
chip, so a typo is a message rather than a worse routing result.

**`[design_rules]`** is the same on every chip except for two counts: a minimum
wire spacing of 185, an obstacle spacing of 25, a bend radius of 50, a straight
length of 100, and a resonator target of 2500 with a tolerance of 100, all in
layout units. What differs per chip is how many wires a routing corridor may
carry and how many extra endpoints a feedline may terminate at:

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

**`[grid]`** sizes the coarse capacity grid. Most chips set only the number of
columns; the 17-qubit chip also sets the rows and a larger launcher offset, and
the 69-qubit chip runs on the defaults and therefore has no `[grid]` section at
all.

## Adding a chip

1. Put the chip's `routing_config.json` in a new directory next to the others.
2. Write a `config.toml` beside it. Copy the one whose naming scheme matches and
   adjust the patterns, the sequences and the two feedline counts.
3. Run `mqt-scpd doctor -c benchmarks/<chip>/config.toml` and fix what it names.
4. Run `mqt-scpd plot` and look at the result before routing anything.

## Values the configurations do not carry yet

Two per-chip numbers of the prototype have no key in `config.toml`, because the
stage that reads them is not ported yet. They are recorded here so they are not
lost:

| Chip  | Assignment launcher target | Capacity grid detail factor |
| ----- | -------------------------: | --------------------------: |
| `4q`  |                          2 |                          30 |
| `9q`  |                          3 |                          30 |
| `17q` |                          7 |                          20 |
| `21q` |                         10 |                          30 |
| `33q` |                         14 |                          30 |
| `45q` |                         15 |                          30 |
| `57q` |                         18 |                          30 |
| `69q` |                         24 |                          30 |

The prototype's drivers also disagree about which ports they pin. The 4-qubit
driver pins the whole outer ring in the capacity stage and nothing later; the
9-qubit driver pins every routable port there; the 17-qubit driver pins
`fixed_outer` in both stages; the five largest pin nothing at all, although each
carries a `fixed_outer` list. The configurations keep every list, so that the
phase which ports the capacity stage can decide what to do with them.
