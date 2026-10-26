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
beyond those of `plot` and `render`: the `launcher_target` of the assignment,
the `component` pattern, and the bridge rules of the couplers. The configuration
of every benchmark chip carries them. `doctor` checks the configuration for the
steps up to `[run] stop_after`, which are all six steps when the key is missing,
so a configuration without `launcher_target` passes `doctor` only with
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
| corridor | `04-corridor.fb` | Routes every connection through the partitions                                             |

The global step runs before the assignment, because the inner circuit decides at
which coupler ports its wires surface, and the assignment works on the ring that
those ports extend. The run directory also keeps a copy of the configuration and
of the chip input, `config.toml` and `00-chip.json`. Every command that takes
`-c` reads that `config.toml` with the chip copy beside it, so
`mqt-scpd doctor -c runs/33q/config.toml` checks the inputs of the run.

`--stop-after <step>`, or `stop_after` in the `[run]` section of the
configuration, ends a run after a step. `--stage <step>` runs one step again on
the copies a run carries and deletes the artifacts of every later step:

```console
mqt-scpd plan -o runs/33q --stage corridor
```

Beside each artifact a step writes a record of its inputs, for example
`01-capacity.inputs.json`. The record holds a digest of the chip copy and of
each section of `config.toml` that the step reads. `SCPD_SOLVER` is not an
input: it chooses the backend of one invocation, and a step that resumes under
another backend can give another artifact. `--stage` refuses a step when the
chip copy or such a section has changed since an earlier step wrote its
artifact, because the step would read an artifact that the current configuration
does not make. The message names the section and the step to run again, in the
block of the refused step; running that step deletes the artifacts of every
later step:

```console
🧭 corridor
   ✗ [design_rules] in config.toml changed after the capacity step wrote 01-capacity.fb; run the capacity step again

✗ stopped at corridor after 0.00s
```

A step may run again after a change to a section that only it reads, such as
`[stages.corridor]` for the corridor step.

`plot` and `render` draw a planning step of a run over the chip:

```console
mqt-scpd plot --run-dir runs/33q --stage corridor -o 33q-corridor.svg
mqt-scpd render --run-dir runs/33q --stage capacity -o 33q-capacity.gds
```

The pictures of the assign and corridor steps also show every feedline chain. A
chain runs from its first launcher through the feeds of its resonators to its
last launcher. In SVG every chain runs in horizontal and vertical steps in one
color and names its launchers and resonators under the pointer; in GDS the
chains are on layer 32, `plan.feedline`. A chain that ends at a termination ends
at the feed of its resonator there, with a square mark.

An SVG shows the chip like a layout tool shows a mask: hatched metal on a light
substrate, a millimeter grid, rulers and a scale bar. Every port is a red
marker. The global picture shows the Hanan lattices and, in green, the lattice
edges that the inner circuit selected. The capacity picture shows the partition
borders. The assign picture shows each ring port joined to its feed by a black
dashed chord, and the corridor picture the corridors as dark lines, both over
light borders and with the green inner wires of the global step. All four show
the cells the grid blocked, in dark gray under the artwork: the artwork as the
grid rasterizes it, and the approach of every port. No picture shows the gates.
The capacity, assign and corridor pictures leave out the borders of partitions
that the artwork closes in on every side, such as the free space inside a ring
of qubits. Every other partition is shaded in a light gray that differs from its
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
kind of shape, such as `plan.partition` on layer 21 and `plan.corridor` on layer
30. Layer 30 also carries the approach of each corridor to its target port. The
cells the grid blocked are on layer 33, `plan.blocked`, and the wire budget of
each bottleneck is a text on layer 34, `plan.budget`.

## What the Current Release Does

MQT SCPD is being ported, one stage at a time, from the research prototype that
produced the results of the paper. The current release reads, checks, draws, and
exports the chip inputs, plans them, and routes them: it partitions the free
space, solves the inner circuit, assigns the ports to launchers, routes every
connection through the partitions, draws every wire as copper, inserts the
couplers and routes the feedline chains. `docs/routing.md` describes the two
routing stages. The routed layouts and the metrics of Table I in
`planar-superconducting-pd` still come from the prototype, and this page grows
as MQT SCPD takes them over.

## SAT Benchmarks

Two benchmarks measure how well the capacity check of the coupler insertion
predicts the feedline routing; `docs/routing.md` explains their four classes.
They run on the chips in `benchmarks/`, or on a clone named by
`MQT_SCPD_BENCHMARKS`, and only when asked for, because they take minutes on the
small chips and hours on the large ones:

```console
uvx nox -s sat_benchmarks                                   # 4q, 9q and 17q
MQT_SCPD_SAT_CHIPS="4q 9q" MQT_SCPD_SAT_REPORT=sat uvx nox -s sat_benchmarks
```

`test_sat_local.py` plans a chip through the coupler insertion, then has the
coupler session search up to eight runs of options for every segment of three
edges of every chain and judge every feedline a run touches twice: by the slot
check of that feedline alone, and by ripping its wires up and routing them
again. `test_sat_global.py` builds whole-chip states from the runs the local
check called SAT, judges each by the whole-chip check and routes it. Both assert
that the check is never optimistic, write one JSON table and one page per chip
to `MQT_SCPD_SAT_REPORT` (the working directory by default), and print one line
per chip with the four classes and the agreement.

`mqt-scpd sat-bench` runs the same measurement on one run directory that holds
the artifacts through `detail`:

```console
mqt-scpd plan -c inputs/17q/config.toml -o runs/17q --stop-after detail
mqt-scpd sat-bench local runs/17q --runs 8 --span 3
mqt-scpd sat-bench global runs/17q --cases 12
```

`--runs` is the number of runs of options per segment (8), `--span` the edges of
a segment (3) and `--cases` the whole-chip states beyond the committed one (12).
The command prints one line per stage, one line per global case and the summary
line of the tests. It writes `sat-local.json` and `sat-local.html`, or
`sat-global.json` and `sat-global.html`, to `<run>/sat-bench`, or to the
directory that `-o` names. The global benchmark takes the runs of
`sat-local.json` when that table has the same runs, span and run inputs, and
otherwise runs the local benchmark first. It routes every case in a forked
child, one after the other, so it needs a platform with `fork`. On 17q with
`--runs 2`, the local benchmark takes about 40 s and the global benchmark with
`--cases 3` about 20 s.

The local page has a tile per class, the agreement, the cases and the seconds.
The mismatches come first: SAT but failing in red on top, then UNSAT but clean
in amber. A row gives the feedline and its edges, the run, the segment, the
moved couplers (`c13: 0→1`), the verdict and the lost wires of the slot check,
the result of the rip-up (clean, or the failing wires, and the rounds) and the
seconds. A strip per feedline shows its classes, and every case follows in a
table with a filter per class and a search box. The global page has the same
tiles and a row per case with its moves, the verdict of the whole-chip check,
the lost wires and the edges over their capacity, the bad count and the class.

The pages that the command writes stand alone, and their `graph ↗` and
`routed ↗` links are off. `mqt-scpd debug <run> --serve` serves the same pages
at `/bench/local` and `/bench/global` from `<run>/sat-bench`, with the links on:
`graph ↗` runs the check of the case on the session and `routed ↗` draws the
wires along the ways that the routing of the case laid. `docs/routing.md` lists
the routes.
