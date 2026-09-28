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

## Comparing with the Published Layouts

`render` writes the obstacles to layer 1/0, as the published layouts do, so the
file it writes can be laid over `layouts/unrouted/gds/33q_unrouted.gds` in
KLayout. The obstacles match exactly; the benchmark job checks that for every
chip. The port labels go to layer 10, with the port's role as the datatype,
which keeps them clear of the wire, coupler, bridge, and clearance layers 2 to 5
of the routed layouts.

## What the Current Release Does

MQT SCPD is being ported, one stage at a time, from the research prototype that
produced the results of the paper. The current release reads, checks, draws, and
exports the chip inputs; it does not route them yet. Until the routing stages
arrive, the routed layouts and the metrics of Table I in
`planar-superconducting-pd` come from the prototype, and this page grows with
each stage that MQT SCPD takes over.
