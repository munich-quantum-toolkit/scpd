# Decision records

One record per architectural decision: what was chosen, what was rejected, and
why. They exist so that settled questions are not silently reopened.

A record is not immutable. If a decision is revisited, supersede it with a new
record and mark the old one `Superseded by NNNN` rather than editing history.

| #                                                  | Decision                                                                    |
| -------------------------------------------------- | --------------------------------------------------------------------------- |
| [0001](0001-byok-milp-solver.md)                   | Bring-your-own-key MILP solver                                              |
| [0002](0002-cli-is-the-product.md)                 | The CLI is the product                                                      |
| [0003](0003-port-then-improve.md)                  | Port first, improve second                                                  |
| [0004](0004-planar-now.md)                         | Planar now, layer axis later                                                |
| [0005](0005-stage-interfaces-and-registry.md)      | Typed stage interfaces with a name registry                                 |
| [0006](0006-geometry-ir-and-klayout.md)            | Neutral geometry IR with a KLayout adapter                                  |
| [0007](0007-deletions.md)                          | What is deleted rather than ported                                          |
| [0008](0008-split-chip-and-config.md)              | Chip description separate from routing configuration _(superseded by 0020)_ |
| [0009](0009-standalone-package.md)                 | Standalone, no ecosystem coupling in the first release                      |
| [0010](0010-generated-benchmark-chips.md)          | Benchmark chips are generated, not committed _(superseded by 0020)_         |
| [0011](0011-hold-runtime-then-parallelize.md)      | Hold runtime, then parallelize                                              |
| [0012](0012-first-release-scope.md)                | First release contains the full pipeline, one algorithm each                |
| [0013](0013-testing-without-golden-pins.md)        | Testing without golden metric pins                                          |
| [0014](0014-resumable-run-directory.md)            | Resumable run directory                                                     |
| [0015](0015-grid-and-memory-model.md)              | Keep eight-way headings, fix memory during the port                         |
| [0016](0016-artifact-formats.md)                   | One schema, formats chosen per artifact _(amended by 0020)_                 |
| [0017](0017-boost-acquisition.md)                  | Boost.Polygon through FetchContent or vendored headers                      |
| [0018](0018-port-roles-unassigned-and-assigned.md) | Port roles are declared, in two stages                                      |
| [0019](0019-design-rules-in-layout-units.md)       | Design rules are physical; grids convert                                    |
| [0020](0020-legacy-routing-config-as-input.md)     | The prototype's routing config is the chip input                            |
| [0021](0021-debug-rendering-in-python.md)          | Per-stage rendering reads artifacts, in Python                              |
| [0022](0022-drc-in-the-core.md)                    | The core owns design-rule checking                                          |
