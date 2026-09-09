# Decision records

One record per architectural decision: what was chosen, what was rejected, and
why. They exist so that settled questions are not silently reopened.

A record is not immutable. If a decision is revisited, supersede it with a new
record and mark the old one `Superseded by NNNN` rather than editing history.

| #                                                       | Decision                                                             |
| ------------------------------------------------------- | -------------------------------------------------------------------- |
| [0001](0001-byok-milp-solver.md)                        | Bring-your-own-key MILP solver                                       |
| [0002](0002-cli-is-the-product.md)                      | The CLI is the product                                               |
| [0005](0005-stage-interfaces-and-registry.md)           | Typed stage interfaces with a name registry                          |
| [0014](0014-resumable-run-directory.md)                 | Resumable run directory                                              |
| [0015](0015-grid-and-memory-model.md)                   | Eight-way headings stay; memory is fixed during the port             |
| [0016](0016-artifact-formats.md)                        | One schema, formats chosen per artifact _(amended by 0020)_          |
| [0017](0017-boost-acquisition.md)                       | Boost.Polygon through FetchContent                                   |
| [0018](0018-port-roles-unassigned-and-assigned.md)      | Port roles are declared, in two stages _(amended by 0027)_           |
| [0019](0019-design-rules-in-layout-units.md)            | Design rules are physical; grids convert                             |
| [0020](0020-legacy-routing-config-as-input.md)          | The prototype's routing config is the chip input _(amended by 0023)_ |
| [0021](0021-debug-rendering-in-python.md)               | Per-stage rendering reads artifacts, in Python _(amended by 0031)_   |
| [0022](0022-drc-in-the-core.md)                         | The core owns design-rule checking _(amended by 0024)_               |
| [0023](0023-geometric-port-ring-detection.md)           | Port ring detection is geometric, and opt-in _(superseded by 0025)_  |
| [0024](0024-wire-loop-is-active.md)                     | Wire loop is an active rule                                          |
| [0025](0025-port-ring-is-manual-input.md)               | The port ring is manual input                                        |
| [0026](0026-global-runs-before-the-assignment.md)       | The Global stage runs before the Assignment stage                    |
| [0027](0027-components-are-declared.md)                 | A port's component is declared, not parsed                           |
| [0028](0028-the-inner-circuit-pays-for-free-space.md)   | The inner circuit pays for the free space it crosses                 |
| [0029](0029-a-feedline-end-is-fed-between-launchers.md) | A feedline end is fed between two launchers                          |
| [0030](0030-bridge-pairs-are-declared.md)               | A bridge pair is declared, and an internal one is shut               |
| [0031](0031-coarse-routing-is-its-own-stage.md)         | Coarse routing is a stage of its own                                 |
