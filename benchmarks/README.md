# Planning Configurations of the Benchmark Chips

The eight benchmark chips live in
[planar-superconducting-pd](https://github.com/cda-tum/planar-superconducting-pd),
together with one `config.toml` per chip for the commands that read and draw a
chip. The planning stages need more keys than those files carry, so this
directory holds the configuration each chip is planned with. The chip input
itself stays in the data repository, and `--chip` names it:

```console
git clone --depth 1 https://github.com/cda-tum/planar-superconducting-pd.git
mqt-scpd plan -c benchmarks/17q/config.toml \
  --chip planar-superconducting-pd/inputs/17q/routing_config.json -o runs/17q
```

Each file is the configuration of the data repository with these keys added or
changed. The benchmark tests check that no other key differs.

| Key                                             | Why the planning stages need it                                                                                                                                                |
| ----------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `[ports.patterns] conventional`, `bridge_pair`  | A coupler port where a wire crosses the coupler is a bridge port, not a conventional one. Only `port0` of a coupler ends a wire.                                               |
| `[ports.patterns] component`                    | The Global stage groups the ports of one component: the inner circuit has to reach the ports of a qubit, and a wire may cross a coupler between two of its ports.              |
| `[[ports.bridge_pairs]]`                        | Which two bridge ports of one coupler a wire crosses between: `port1` with `port2`, and `port3` with `port4`.                                                                  |
| `[grid] launcher_offset_x`, `launcher_offset_y` | The capacity grid blocks a border of this many detail cells. The launcher slot sits on the launcher port, so the border must not cover it; on most chips it is therefore zero. |
| `[stages.assignment] launcher_target`           | How many launchers the assignment activates. A figure per chip with no default, taken from the prototype's driver of the same benchmark.                                       |

| Chip | `launcher_target` | Launcher offset |
| ---- | ----------------: | --------------: |
| 4q   |                 2 |               7 |
| 9q   |                 3 |               5 |
| 17q  |                 7 |               0 |
| 21q  |                10 |               0 |
| 33q  |                14 |               0 |
| 45q  |                15 |               0 |
| 57q  |                18 |    15 (default) |
| 69q  |                24 |               0 |

The design rules are those of Table I of the paper, the same as in the data
repository. None of the planning stages reads the resonator length.
