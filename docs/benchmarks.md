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

`mqt-scpd plan` runs the planning steps on a chip. The steps need a few keys
beyond those of `plot` and `render`: the `launcher_target` of the assignment,
the `component` pattern, and the bridge rules of the couplers. The configuration
of every benchmark chip carries them. `doctor` checks the configuration for the
steps up to `[run] stop_after`, which are all three steps when the key is
missing, so a configuration without `launcher_target` passes `doctor` only with
`stop_after = "global"` or an earlier step:

```console
mqt-scpd plan -c inputs/33q/config.toml -o runs/33q
```

A run writes one artifact per step into its directory, in this order:

| Step     | Artifact         | What the step does                                                                         |
| -------- | ---------------- | ------------------------------------------------------------------------------------------ |
| capacity | `01-capacity.fb` | Partitions the free space at its bottlenecks and budgets the wires of every capacity chain |
| global   | `02-global.fb`   | Solves the inner circuit and the port ring that the assignment works on                    |
| assign   | `03-assign.fb`   | Assigns the ring ports to launchers and feedline chains                                    |

The global step runs before the assignment, because the inner circuit decides at
which coupler ports its wires surface, and the assignment works on the ring that
those ports extend. The run directory also keeps a copy of the configuration and
of the chip input, `config.toml` and `00-chip.json`. Every command that takes
`-c` reads that `config.toml` with the chip copy beside it, so
`mqt-scpd doctor -c runs/33q/config.toml` checks the inputs of the run.

`plan` accepts a new directory, an empty directory, or a directory with a valid
`.mqt-scpd-run` marker. It reads a single snapshot of the configuration and chip
input. Once the requested stages finish, it saves those exact inputs with their
artifacts and replaces any previous run outputs. A planning failure leaves the
previous run intact. If a writer stops while replacing files, the next planning
command restores the previous files before using the run.

`--stop-after <step>`, or `stop_after` in the `[run]` section of the
configuration, ends a run after a step. `--stage <step>` runs one step again on
the copies a run carries. A successful rerun deletes the artifacts of every
later step:

```console
mqt-scpd plan -o runs/33q --stage assign
```

Beside each artifact a step writes a record of its inputs, for example
`01-capacity.inputs.json`. The record holds digests of the classified chip, the
configuration sections the step reads, the artifacts it reads, and the artifact
it writes. Resuming or drawing a run checks these digests before using an
artifact. The message names the section and the step to run again, in the block
of the refused step; a successful rerun of that step deletes the artifacts of
every later step. A step may run again after a change to a section that only it
reads, such as `[stages.assignment]` for the assign step. `SCPD_SOLVER` chooses
the backend for one invocation and is outside the input record. A step that
resumes with a different backend can write a different artifact.

`plot` and `render` draw a planning step of a run over the chip:

```console
mqt-scpd plot --run-dir runs/33q --stage assign -o 33q-assign.svg
mqt-scpd render --run-dir runs/33q --stage capacity -o 33q-capacity.gds
```

The picture of the assign step also shows every feedline chain. A chain runs
from its first launcher through the feeds of its resonators to its last
launcher. In SVG every chain runs in horizontal and vertical steps in one color
and names its launchers and resonators under the pointer; in GDS the chains are
on layer 32, `plan.feedline`. A chain that ends at a termination ends at the
feed of its resonator there, with a square mark.

An SVG shows the chip like a layout tool shows a mask: hatched metal on a light
substrate, a millimeter grid, rulers and a scale bar. Every port is a red
marker. The global picture shows the Hanan lattices and, in green, the lattice
edges that the inner circuit selected. The capacity picture shows the partition
borders. The assign picture shows each ring port joined to its feed by a black
dashed chord, over light borders and with the green inner wires of the global
step. All three show the cells the grid blocked, in dark gray under the artwork:
this raster is the one stored in the Capacity artifact. Port approach polygons
also include reservations reopened while placing targets. The SVG leaves out the
gates. The capacity and assign pictures leave out the borders of partitions that
the artwork closes in on every side, such as the free space inside a ring of
qubits. Every other partition is shaded in a light gray that differs from its
neighbors. Opened on its own in a browser, the picture has a **Layers** panel in
its top left corner that shows and hides each layer, and it leaves out the
finest detail when zoomed out.

{doc}`terminal_output` describes what a run prints and what `-v`, `-vv` and
`-vvv` add.

## Comparing with the Published Layouts

`render` writes the obstacles to layer 1/0, as the published layouts do, so the
file it writes can be laid over `layouts/unrouted/gds/33q_unrouted.gds` in
KLayout. The obstacles match exactly; the benchmark job checks that for every
chip. The port labels go to layer 10, with the port's role as the datatype,
which keeps them clear of the wire, coupler, bridge, and clearance layers 2 to 5
of the routed layouts. A planning step goes to the layers from 20 on, one per
kind of shape, such as `plan.partition` on layer 21 and `plan.assignment` on
layer 28. The cells the grid blocked are on layer 33, `plan.blocked`, and the
wire budget of each bottleneck is a text on layer 34, `plan.budget`.

## What the Current Release Does

MQT SCPD is being ported, one stage at a time, from the research prototype that
produced the results of the paper. The current release reads, checks, draws, and
exports the chip inputs, and it plans them: it partitions the free space, solves
the inner circuit, and assigns the ports to launchers. It does not route the
wires yet. Until the routing stages arrive, the routed layouts and the metrics
of Table I in `planar-superconducting-pd` come from the prototype, and this page
grows with each stage that MQT SCPD takes over.
