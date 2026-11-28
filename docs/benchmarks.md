# Benchmark Chips

Download the eight benchmark chips (4 to 69 qubits) from
[planar-superconducting-pd](https://github.com/cda-tum/planar-superconducting-pd).
The data accompanies
_Physical Design Automation for Planar Superconducting Quantum Chips_.

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

MQT SCPD tests all eight chips against the data revision pinned by its benchmark
workflow. The `Update benchmark data` workflow opens a PR for changes to that
pin.

## Running a Chip

Each chip's `config.toml` names its `routing_config.json` input and defines its
port roles, outer ring, and design rules. The JSON stores obstacle polygons and
port coordinates in µm. Run `doctor` first to check both files and see the
classified ports and outer ring.

```console
mqt-scpd doctor -c inputs/33q/config.toml               # check the chip and its configuration
mqt-scpd plot -c inputs/33q/config.toml -o 33q.svg      # draw the unrouted chip
mqt-scpd render -c inputs/33q/config.toml -o 33q.gds    # write it as GDSII
```

## Planning a Chip

The benchmark configurations already include the launcher target, component
pattern, and coupler bridge rules needed to run through Detail.

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
| detail   | `05-detail.fb`   | Draws every wire on the detail grid and checks clearance between routed cells              |

Global determines which bridge ports join the outer ring. Assignment uses that
ring. The run saves its inputs as `config.toml` and `00-chip.json`. Check those
copies with `mqt-scpd doctor -c runs/33q/config.toml`.

Use `--stop-after <step>` or `[run] stop_after` to stop after a stage. `doctor`
checks the stages through the configured stop point. To rerun one stage with the
saved inputs, use `--stage`. A successful rerun removes outputs from later
stages.

```console
mqt-scpd plan -o runs/33q --stage corridor
```

Each stage records its input and artifact digests in a file such as
`01-capacity.inputs.json`. If `plan`, `plot`, or `render` reports stale inputs,
rerun the stage named in the message. For example, changing only
`[stages.assignment]` requires rerunning Assignment. The `SCPD_SOLVER` override
is outside these records and can change a rerun's result.

## Viewing a Run

Use `plot` for SVG or `render` for GDSII or OASIS.

```console
mqt-scpd plot --run-dir runs/33q --stage corridor -o 33q-corridor.svg
mqt-scpd plot --run-dir runs/33q --stage detail -o 33q-detail.svg
mqt-scpd render --run-dir runs/33q --stage capacity -o 33q-capacity.gds
```

Capacity shows partition borders. Global shows the Hanan lattices and selected
inner wires. Assignment adds the feed points and feedline chains. Dashed lines
connect ring ports to their feeds. A square marks a chain termination. All views
use the blocked cells saved by Capacity. Corridor shows routes from the exact
feed points to the target cells, with an illustrative approach to each port. It
also shows the available crossing slots. Detail draws every wire as a coplanar
waveguide over a band as wide as the clearance that the stage keeps. The bands
illustrate spacing along continuous paths. Detail counts conflicts between
discrete path cells. On rectangular cells the band width uses the smaller grid
step. In a browser, use the SVG's **Layers** panel to show or hide individual
layers.

SVGs omit gates. The Capacity, Assignment, Corridor, and Detail views also omit
the borders of fully enclosed partitions.

See {doc}`terminal_output` for destinations, failure recovery, and verbosity.

## Comparing with the Published Layouts

`render` writes obstacles to layer 1/0. Overlay the output on
`layouts/unrouted/gds/33q_unrouted.gds` in KLayout. The benchmark job checks
that the obstacles match for every chip. Port labels use layer 10, with the port
role as the datatype. Planning layers start at 20. Key layers include
`plan.partition` (21), `plan.assignment` (28), `plan.corridor` (30), `plan.slot`
(31), `plan.feedline` (32), `plan.blocked` (33), and `plan.budget` (34). Layer
30 contains the coarse routes and separate paths that illustrate the approaches
to their target ports. Detail writes `route.wire` (40), `route.resonator` (41),
`route.inner-wire` (42), and `route.clearance` (44).

The published routed layouts and Table I metrics come from the research
prototype.
