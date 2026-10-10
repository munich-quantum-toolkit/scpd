# Benchmark Chips

The eight benchmark chips of the paper
_Physical Design Automation for Planar Superconducting Quantum Chips_ range from
4 to 69 qubits. They are published together with the paper's layouts and
quality-of-results data in
[planar-superconducting-pd](https://github.com/cda-tum/planar-superconducting-pd),
not in this repository:

| Path                         | Content                                                           |
| ---------------------------- | ----------------------------------------------------------------- |
| `inputs/<chip>/`             | The chip inputs that MQT SCPD reads, one directory per chip       |
| `layouts/unrouted/{gds,svg}` | The chips before routing                                          |
| `layouts/routed/{gds,svg}`   | The chips as the paper's flow routed them                         |
| `qor/qor.csv`                | The quality-of-results metrics of Table I of the paper            |

The chips are named `4q`, `9q`, `17q`, `21q`, `33q`, `45q`, `57q`, and `69q`.

## Getting the Chips

Install MQT SCPD with the KLayout extra, which `render` needs, and clone the
data repository:

```console
uv pip install "mqt.scpd[klayout]"
git clone --depth 1 https://github.com/cda-tum/planar-superconducting-pd.git
cd planar-superconducting-pd
```

The CI of MQT SCPD checks every chip against a pinned commit of that repository,
which follows its `main` branch as the data changes.

## Running a Chip

Every chip directory holds two files. `routing_config.json` is the chip itself:
the obstacle polygons and the named ports a wire can start or end at, in µm.
`config.toml` says how to route it: one regular expression per port role, the
ring of outer ports, and the design rules of Table I. Every command takes the
`config.toml`, which names its chip input:

```console
mqt-scpd doctor -c inputs/33q/config.toml               # check the chip and its configuration
mqt-scpd plot -c inputs/33q/config.toml -o 33q.svg      # draw the unrouted chip
mqt-scpd render -c inputs/33q/config.toml -o 33q.gds    # write it as GDSII
```

Run `doctor` first. It prints how many ports fell into each role and the outer
port ring the run would use, and when the configuration and the chip do not fit
together, it names the offending port or key and exits with a nonzero status.

## Planning a Chip

`mqt-scpd plan` runs the planning steps on a chip. The steps need a few keys
that the configurations of the data repository do not carry yet: the
`launcher_target` of the assignment, the `component` pattern, and the bridge
rules of the couplers. This repository carries a configuration with these keys
for every benchmark chip in `benchmarks/<chip>/config.toml`. Each takes the chip
input of the data repository through `--chip`:

```console
mqt-scpd plan -c benchmarks/33q/config.toml \
  --chip planar-superconducting-pd/inputs/33q/routing_config.json -o runs/33q
```

A run writes one artifact per step into its directory, in this order:

| Step     | Artifact         | What the step does                                                                         |
| -------- | ---------------- | ------------------------------------------------------------------------------------------ |
| capacity | `01-capacity.fb` | Partitions the free space at its bottlenecks and budgets the wires of every capacity chain |
| global   | `02-global.fb`   | Solves the inner circuit and the port ring that the assignment works on                    |
| assign   | `03-assign.fb`   | Assigns the ring ports to launchers and feedline chains                                    |
| corridor | `04-corridor.fb` | Routes every connection through the partitions                                             |

The global step runs before the assignment, because the inner circuit decides at
which coupler ports its wires surface, and the assignment works on the ring that
those ports extend. The run directory also keeps a copy of the configuration and
of the chip input, `config.toml` and `00-chip.json`.

`--stop-after <step>`, or `stop_after` in the `[run]` section of the
configuration, ends a run after a step. `--stage <step>` runs one step again on
the copies a run carries and deletes the artifacts of every later step:

```console
mqt-scpd plan -o runs/33q --stage corridor
```

`plot` and `render` draw a planning step of a run over the chip:

```console
mqt-scpd plot --run-dir runs/33q --stage corridor -o 33q-corridor.svg
mqt-scpd render --run-dir runs/33q --stage capacity -o 33q-capacity.gds
```

The pictures of the assign and corridor steps also show every feedline chain. A
chain runs from its first launcher through the feeds of its resonators to its
last launcher. In SVG each chain has a color of its own and names its launchers
and resonators under the pointer; in GDS the chains are on layer 32,
`plan.feedline`. A chain that ends at a termination ends at the feed of its
resonator there, with a square mark.

{doc}`terminal_output` describes what a run prints and what `-v`, `-vv` and
`-vvv` add.

## Comparing with the Published Layouts

`render` writes the obstacles to layer 1/0, as the published layouts do, so the
file it writes can be laid over `layouts/unrouted/gds/33q_unrouted.gds` in
KLayout. The obstacles match exactly; the benchmark job checks that for every
chip. The port labels go to layer 10, with the port's role as the datatype,
which keeps them clear of the wire, coupler, bridge, and clearance layers 2 to 5
of the routed layouts. A planning step goes to the layers from 20 on, one per
kind of shape, such as `plan.partition` on layer 21 and `plan.corridor` on layer
30.

## What the Current Release Does

MQT SCPD is being ported, one stage at a time, from the research prototype that
produced the results of the paper. The current release reads, checks, draws, and
exports the chip inputs, and it plans them: it partitions the free space, solves
the inner circuit, assigns the ports to launchers, and routes every connection
through the partitions. It does not draw the wires yet. Until those stages
arrive, the routed layouts and the metrics of Table I in
`planar-superconducting-pd` come from the prototype, and this page grows with
each stage that MQT SCPD takes over.
