# Decision records

One record per architectural decision: what was chosen, what was rejected, and
why. They exist so that settled questions are not silently reopened.

A record is not immutable. If a decision is revisited, supersede it with a new
record and mark the old one `Superseded by NNNN` rather than editing history.

| #                                                  | Decision                                                             |
| -------------------------------------------------- | -------------------------------------------------------------------- |
| [0002](0002-cli-is-the-product.md)                 | The CLI is the product                                               |
| [0016](0016-artifact-formats.md)                   | One schema, formats chosen per artifact _(amended by 0020)_          |
| [0018](0018-port-roles-unassigned-and-assigned.md) | Port roles are declared, in two stages                               |
| [0019](0019-design-rules-in-layout-units.md)       | Design rules are physical; grids convert                             |
| [0020](0020-legacy-routing-config-as-input.md)     | The prototype's routing config is the chip input _(amended by 0023)_ |
| [0022](0022-drc-in-the-core.md)                    | The core owns design-rule checking _(amended by 0024)_               |
| [0023](0023-geometric-port-ring-detection.md)      | Port ring detection is geometric, and opt-in                         |
| [0024](0024-wire-loop-is-active.md)                | Wire loop is an active rule                                          |
