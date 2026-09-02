# MQT SCPD Architecture

This document is the authority on module boundaries and dependency direction for
MQT SCPD. It describes the target architecture for the rewrite of the FridgeCAD
prototype into this repository.

Companion documents:

- [Data model](docs/design/data-model.md) — entities, coordinate systems
- [Pipeline](docs/design/pipeline.md) — stage contracts, artifacts, resume
- [Roadmap](docs/design/roadmap.md) — phased plan with acceptance criteria
- [Decisions](docs/design/decisions/) — one record per architectural decision
- [Porting notes](docs/design/porting-notes.md) — map of the prototype

Diagrams are Mermaid and render on GitHub. These design documents are excluded
from the Sphinx build; they target contributors, not users.

## What the tool does

MQT SCPD takes a description of a superconducting quantum chip — qubits,
couplers, launchers and obstacle geometry — and produces manufacturable
coplanar-waveguide routing: every resonator connected to a launcher, every wire
curvature-constrained, and every design rule satisfied. The output is GDSII or
OASIS.

## Guiding principles

1. **The C++ core is pure.** It never opens a file, never writes to `stdout`,
   and never links a layout library. It consumes typed inputs and returns typed
   outputs. Python owns argument parsing, file orchestration, and rendering.
2. **Stages are values in, values out.** Every stage entry point is `const` and
   takes its inputs by `const&`. No stage mutates another stage's state.
3. **One vocabulary per concept.** One point type per coordinate system, one
   design-rule struct, one path representation. The prototype had eight point
   types, and the same clearance appeared as a physical constant in two stages
   and as an unexplained cell count in a third.
4. **Design rules are physical, and converted once.** Every rule is a length in
   layout units. A stage that works in grid space obtains its cell count from
   `cells_for`, never from a literal. No stage declares its own constant.
5. **Derived state is never an artifact.** Grids and router containers are
   rebuilt deterministically from the chip and the configuration.
6. **Minimum viable structure.** Abstractions exist where a second
   implementation is already planned or a dependency must be swappable. Nowhere
   else.

## Level 1 — System context

```mermaid
graph LR
  user([Researcher])
  cli[mqt-scpd CLI<br/>Python]
  core[MQT::Scpd*<br/>C++ core]
  kl[KLayout<br/>GDS / OASIS / render]
  gf[GDSFactory<br/>optional extra]
  fab([Fabrication])

  user -->|routing_config.json<br/>config.toml| cli
  cli -->|nanobind| core
  core -->|geometry IR + metrics| cli
  cli --> kl
  cli -.-> gf
  kl --> fab
  gf --> fab
```

The CLI is the product. The Python API exists and is importable, but it is
documented as internal and unstable. See
[decision 0002](docs/design/decisions/0002-cli-is-the-product.md).

## Level 2 — Pipeline and artifacts

```mermaid
flowchart TD
  chip[routing_config.json<br/>obstacles and ports]
  cfg[config.toml<br/>roles, rules, tuning]

  subgraph core["C++ core"]
    cap[Capacity<br/>watershed / EDT / budgets]
    asg[Assignment<br/>resonator to launcher, MILP]
    glb[Global<br/>Hanan inner circuit, MILP]
    det[Detail<br/>A* on pixel grid]
    fin[Final<br/>Dubins routing, CPW, feedlines]
    geo[Finalize<br/>curve fit, meander, bridges]
  end

  a1[01-capacity.fb]
  a2[02-assign.fb]
  a3[03-global.fb]
  a4[04-detail.fb]
  a5[05-final.fb]
  a6[06-geometry.fb<br/>analytic line and arc]
  met[metrics.json<br/>log.jsonl]
  drc[drc.json<br/>DRCPolice findings]
  gds[(out.gds / out.oas)]

  svg[["mqt-scpd plot<br/>per-stage SVG"]]

  chip --> cap --> a1 --> asg --> a2 --> glb --> a3 --> det --> a4 --> fin --> a5 --> geo --> a6
  cfg -.-> cap & asg & glb & det & fin & geo
  core --> met
  a5 & a6 -->|DRCPolice| drc
  a6 -->|KLayout adapter| gds
  a1 & a4 & a5 & a6 -.-> svg
  drc -.-> svg
```

Each stage is independently runnable and resumable, and every artifact can be
rendered to SVG without re-running the pipeline. Full contracts are in
[the pipeline document](docs/design/pipeline.md).

**The geometry IR carries analytic segments** — lines and circular arcs with
exact centre and radius — not sampled points. The router already produces Dubins
paths; sampling early and reconstructing arc structure afterwards loses
precision and wastes work. Polygonization happens once, in the export adapter,
at a configured tolerance.

## Level 3 — Modules

Eight libraries, following the per-module pattern of MQT Core rather than a
single flat target. Public headers live under `include/mqt-scpd/<module>/` and
sources mirror them under `src/<module>/`.

```mermaid
graph TD
  geometry[MQT::ScpdGeometry<br/>Point, Polygon, Path, arcs, units]
  design[MQT::ScpdDesign<br/>Chip, ports, roles, port ring, DesignRules, Config]
  grid[MQT::ScpdGrid<br/>ObstacleGrid, EDT, watershed, cells_for]
  routing[MQT::ScpdRouting<br/>Dubins A*, move primitives]
  milp[MQT::ScpdMilp<br/>Model, HiGHS backend, MPS emit]
  drc[MQT::ScpdDrc<br/>DrcPolice, eight rules, DrcReport]
  pipeline[MQT::ScpdPipeline<br/>stage interfaces, registry, six stages]
  io[MQT::ScpdIO<br/>artifacts, metrics, report writing]

  design --> geometry
  grid --> geometry
  grid --> design
  routing --> grid
  drc --> grid
  drc --> design
  drc --> geometry
  io --> design
  io --> geometry
  io --> drc
  pipeline --> routing
  pipeline --> milp
  pipeline --> drc
  pipeline --> io
```

The dependency graph is acyclic. Nothing depends on `pipeline`; it is the top.

| Target              | Alias               | Responsibility                                                                                                            |
| ------------------- | ------------------- | ------------------------------------------------------------------------------------------------------------------------- |
| `mqt-scpd-geometry` | `MQT::ScpdGeometry` | Point, polygon, path with line and arc segments, transforms, unit handling                                                |
| `mqt-scpd-design`   | `MQT::ScpdDesign`   | Chip ports and obstacles, the two port-role enums, the outer-boundary walk, design rules, the run configuration           |
| `mqt-scpd-grid`     | `MQT::ScpdGrid`     | Rasterization, distance transform, watershed, packed obstacle grids, rule-to-cell conversion                              |
| `mqt-scpd-routing`  | `MQT::ScpdRouting`  | Curvature-constrained A* over Dubins primitives                                                                           |
| `mqt-scpd-milp`     | `MQT::ScpdMilp`     | Solver-neutral model assembly, HiGHS backend, MPS emission for the BYOK path                                              |
| `mqt-scpd-drc`      | `MQT::ScpdDrc`      | DRCPolice: the eight design rules — five active, three advisory — checked in both the router and layout coordinate spaces |
| `mqt-scpd-pipeline` | `MQT::ScpdPipeline` | Stage interfaces, the implementation registry, and the six stage implementations                                          |
| `mqt-scpd-io`       | `MQT::ScpdIO`       | Artifact read and write, metrics, serialization of the DRC report                                                         |

Each is declared through `cmake/AddMQTScpdLibrary.cmake`, adapted from MQT
Core's `AddMQTCoreLibrary.cmake`: `FILE_SET HEADERS`, `generate_export_header`,
the `MQT::` export namespace, and a per-module `test/<module>/CMakeLists.txt`
that globs its own tests.

### Stage interfaces

```mermaid
classDiagram
  class ICapacityPlanner {
    <<interface>>
    +run(Chip, Config) CapacityPlan
  }
  class IAssigner {
    <<interface>>
    +run(Chip, CapacityPlan, Config) Assignment
  }
  class IGlobalRouter {
    <<interface>>
    +run(Chip, Assignment, Config) GlobalRouting
  }
  class IDetailRouter {
    <<interface>>
    +run(Chip, GlobalRouting, Config) DetailRouting
  }
  class IFinalRouter {
    <<interface>>
    +run(Chip, DetailRouting, Config) FinalRouting
  }
  class IFinalizer {
    <<interface>>
    +run(Chip, FinalRouting, Config) Geometry
  }
  class Registry~T~ {
    +add(name, factory)
    +make(name) unique_ptr~T~
    +names() vector~string~
  }

  ICapacityPlanner <|.. WatershedPlanner
  IAssigner <|.. OrderedMilpAssigner
  IGlobalRouter <|.. HananMilpRouter
  IDetailRouter <|.. AStarDetailRouter
  IFinalRouter <|.. DubinsFinalRouter
  IFinalizer <|.. CurveFitFinalizer
```

Every `run` is `const` and takes inputs by `const&`. That single constraint
removes three prototype defects at once: the chip is no longer copied by value
into three stages, no stage writes into another's queue mid-solve, and the
signatures are already shaped for the threading work in phase 2.

The registry is a `map<string, factory>` — roughly thirty lines. It exists
because algorithm swapping is a stated requirement of a research tool, not on
speculation. There is no plugin loading and no configuration language.

## Level 4 — A run

```mermaid
sequenceDiagram
  participant U as User
  participant CLI as mqt-scpd (Python)
  participant B as pyscpd (nanobind)
  participant C as C++ core
  participant S as Solver backend
  participant K as KLayout

  U->>CLI: mqt-scpd route -c 69q/config.toml -o run/
  CLI->>CLI: load routing_config.json, validate config, classify port roles
  CLI->>B: route(chip, config, run_dir)
  loop each stage
    B->>C: stage.run(inputs, config)
    opt MILP stage
      C->>S: model (in-process HiGHS, or MPS to gurobipy)
      S-->>C: solution
    end
    opt after Final and after Finalize
      C->>C: DrcPolice::check(view, rules)
      C-->>B: DrcReport appended to drc.json
    end
    C-->>C: spdlog to log.jsonl sink
    C-->>B: typed stage output
    B->>CLI: artifact written, metrics appended
    CLI->>U: progress line
  end
  B-->>CLI: Geometry (analytic segments)
  CLI->>K: polygonize at tolerance, write GDS or OASIS
  CLI->>U: table of fails, angle cost, runtime, DRC
  CLI->>U: exit nonzero if any active DRC rule was violated
```

## Repository layout

```text
mqt-scpd/
  schemas/*.fbs                          the data model, single source of truth
  include/mqt-scpd/<module>/*.hpp        public headers
  include/mqt-scpd/generated/            committed, via nox -s schemas
  src/<module>/                          mirrors include/
  bindings/bindings.cpp                  minimal: run pipeline, read metrics
  python/mqt/scpd/
    cli.py                               argparse subcommands
    solvers/gurobipy_backend.py          BYOK Gurobi via MPS
    export/klayout.py                    GDS and OASIS
    export/gdsfactory.py                 optional extra
    report.py                            tables from metrics.json and drc.json
    plot.py                              per-stage SVG from the artifacts
  test/<module>/*.cpp                    GoogleTest, per module
  test/python/{unit,property,integration}/
  benchmarks/<n>q/config.toml            one per benchmark chip
  benchmarks/<n>q/routing_config.json    4Q and 9Q only; larger ones by path
  docs/design/                           these documents
```

## Dependencies

| Layer         | Dependency    | Acquisition                      | Why                                                                              |
| ------------- | ------------- | -------------------------------- | -------------------------------------------------------------------------------- |
| C++           | FlatBuffers   | FetchContent                     | Schema-generated data model and stage artifacts. Runtime only, no parser library |
| C++           | nlohmann/json | FetchContent                     | The chip input, metrics and logs                                                 |
| C++           | spdlog        | FetchContent                     | Structured logging; replaces scattered `std::cout`                               |
| C++           | HiGHS         | FetchContent                     | Default solver; makes an unlicensed install fully functional                     |
| C++           | Boost.Polygon | FetchContent or vendored headers | One Voronoi construction. Never a user-installed Boost                           |
| C++           | GoogleTest    | FetchContent                     | Existing repository convention                                                   |
| Python        | klayout       | PyPI                             | GDS and OASIS writing and rendering. It provides no DRC                          |
| Python        | rich          | PyPI                             | Progress over long runs, and result tables                                       |
| Python (test) | hypothesis    | PyPI                             | Property-based tests                                                             |
| Optional      | gurobipy      | user-installed                   | Bring-your-own-license solver path                                               |
| Optional      | gdsfactory    | `mqt-scpd[gdsfactory]`           | Adapter over the same geometry IR                                                |

The CLI uses the standard library's `argparse`. Dependencies are declared in
`cmake/ExternalDependencies.cmake`, never inline in a target.

## What we deliberately do not build

Recorded so that future contributors do not add them speculatively.

- No generic pass base class and no plugin loading. Typed interfaces and a name
  map, nothing more.
- No layer axis, via model, or three-dimensional routing graph. The design stays
  planar until flip-chip work actually begins.
- No custom geometry library beyond what the stages use, and no expression
  templates in the MILP layer.
- No CLI or argument parsing in C++, and no file paths inside the core.
- No GDS writer of our own. No SVG writer **in the core** — `plot.py` renders
  from the artifacts, which is why it can render a half-finished run.
- No DRC rule deck for an external tool. The **checker** is ours and is not
  optional — KLayout ships no design-rule checking to defer to, so DRCPolice in
  `MQT::ScpdDrc` is the only thing standing between a routed chip and a claim
  that it is manufacturable. See
  [decision 0022](docs/design/decisions/0022-drc-in-the-core.md).
- No hand-written struct for anything a schema already describes, and no second
  schema language.
- No schema constraints on metrics. They stay loose JSON on purpose.
- No golden metric pins in the test suite.
- No second implementation of any stage in the first release. The registry ships
  with one entry each.
- No entity model recovered from label strings, and equally no chip generator or
  format converter in the first release. Port roles come from configured
  patterns; the chip input stays the prototype's own JSON. Deriving the outer
  port ring does not reopen this: the walk reads geometry and tests one leading
  character, so it never recovers a coupler's qubit pair. See
  [decision 0018](docs/design/decisions/0018-port-roles-unassigned-and-assigned.md),
  [decision 0020](docs/design/decisions/0020-legacy-routing-config-as-input.md)
  and
  [decision 0023](docs/design/decisions/0023-geometric-port-ring-detection.md).
- No environment-variable tuning. The prototype's 75 `getenv` sites are deleted
  outright; `config.toml` is the only tuning surface.
