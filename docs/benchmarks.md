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

The CI of MQT SCPD checks every chip against a pinned commit of that repository.
The `Update benchmark data` workflow checks its `main` branch every Sunday and
opens a pull request when the pinned commit changes. These updates require
review before merging. The workflow can also be run manually.

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

`mqt-scpd plan` runs the planning steps on a chip:

```console
mqt-scpd plan -c inputs/33q/config.toml -o runs/33q
```

A run writes one artifact per step into its directory:

| Step     | Artifact         | What the step does                                                                         |
| -------- | ---------------- | ------------------------------------------------------------------------------------------ |
| capacity | `01-capacity.fb` | Partitions the free space at its bottlenecks and budgets the wires of every capacity chain |

The run directory also keeps a copy of the configuration and of the chip input,
`config.toml` and `00-chip.json`. Every command that takes `-c` reads that
`config.toml` with the chip copy beside it, so
`mqt-scpd doctor -c runs/33q/config.toml` checks the inputs of the run.

A new run starts afresh and deletes the artifacts of the run it replaces. `plan`
therefore takes a directory that does not exist, an empty directory or a run
directory, which holds `00-chip.json`, and refuses any other directory.

`--stop-after <step>`, or `stop_after` in the `[run]` section of the
configuration, ends a run after a step. `--stage <step>` runs one step again on
the copies a run carries and deletes the artifacts of every later step:

```console
mqt-scpd plan -o runs/33q --stage capacity
```

Beside each artifact a step writes a record of its inputs, for example
`01-capacity.inputs.json`. The record holds a digest of the chip copy and of
each section of `config.toml` that the step reads. `--stage` refuses a step when
the chip copy or such a section has changed since an earlier step wrote its
artifact, because the step would read an artifact that the current configuration
does not make. The message names the section and the step to run again, in the
block of the refused step; running that step deletes the artifacts of every
later step. A step may run again after a change to a section that only it reads,
such as `[stages.capacity]` for the capacity step.

`plot` and `render` draw a planning step of a run over the chip:

```console
mqt-scpd plot --run-dir runs/33q --stage capacity -o 33q-capacity.svg
mqt-scpd render --run-dir runs/33q --stage capacity -o 33q-capacity.gds
```

An SVG shows the chip like a layout tool shows a mask: hatched metal on a light
substrate, a millimeter grid, rulers and a scale bar. Every port is a red
marker. The capacity picture shows the partition borders and the cells the grid
blocked, in dark gray under the artwork: the artwork as the grid rasterizes it,
and the approach of every port. It shows no gates. It leaves out the borders of
partitions that the artwork closes in on every side, such as the free space
inside a ring of qubits. Every other partition is shaded in a light gray that
differs from its neighbors. Opened on its own in a browser, the picture has a
**Layers** panel in its top left corner that shows and hides each layer, and it
leaves out the finest detail when zoomed out.

{doc}`terminal_output` describes what a run prints and what `-v` and `-vv` add.

## Comparing with the Published Layouts

`render` writes the obstacles to layer 1/0, as the published layouts do, so the
file it writes can be laid over `layouts/unrouted/gds/33q_unrouted.gds` in
KLayout. The obstacles match exactly; the benchmark job checks that for every
chip. The port labels go to layer 10, with the port's role as the datatype,
which keeps them clear of the wire, coupler, bridge, and clearance layers 2 to 5
of the routed layouts. A planning step goes to the layers from 20 on, one per
kind of shape, such as `plan.partition` on layer 21 and `plan.chain` on layer
25. The cells the grid blocked are on layer 33, `plan.blocked`, and the wire
budget of each bottleneck is a text on layer 34, `plan.budget`.

## What the Current Release Does

MQT SCPD is being ported, one stage at a time, from the research prototype that
produced the results of the paper. The current release reads, checks, draws, and
exports the chip inputs, and it partitions their free space. It does not assign
the ports or route the wires yet. Until the routing stages arrive, the routed
layouts and the metrics of Table I in `planar-superconducting-pd` come from the
prototype, and this page grows with each stage that MQT SCPD takes over.
