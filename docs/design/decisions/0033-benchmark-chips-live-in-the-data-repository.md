# 0033 — The benchmark chips live in the data repository

- **Status:** Accepted
- **Date:** 2026-09-28
- **Amends:** [0020](0020-legacy-routing-config-as-input.md)

## Context

Decision 0020 made the prototype's `routing_config.json` the chip input, and
phase 1 committed the eight benchmark chips, each with its `config.toml`, to
`benchmarks/`: 7.6 MB, most of it vertex lists. The paper's supplementary data
already has a home,
[planar-superconducting-pd](https://github.com/cda-tum/planar-superconducting-pd),
which carries the unrouted and routed layouts and the quality-of-results table,
and whose README pointed back to `benchmarks/` for the inputs. Data committed to
the tool repository stays in its history for good, even once it is deleted.

## Decision

The chip inputs live in planar-superconducting-pd, as
`inputs/<chip>/config.toml` and `inputs/<chip>/routing_config.json`, next to the
layouts and results they belong to. The tool repository carries no benchmark
data.

- The unit tests read one hand-written chip, `test/fixtures/mini`, small enough
  to read and chosen to cover what the benchmarks stood in for: all three port
  roles, a `fixed_outer` that is an open sub-chain of `all_outer`, overlapping
  obstacles of opposite winding, and a polygon a tolerance can simplify.
- A benchmark workflow checks out planar-superconducting-pd at a commit pinned
  in `.github/workflows/benchmarks.yml` and runs the `benchmarks` nox session:
  every configuration follows the rules for a shipped file and carries the
  *d*<sub>fix</sub> and *r*<sub>util</sub> of Table I (`qor/qor.csv`), `doctor`
  passes, `plot` draws every port, and the obstacles `render` writes are exactly
  the polygons on layer 1/0 of the published unrouted GDS file.
- A custom Renovate manager moves the pin to the latest commit of the data
  repository's `main`. Those updates are not merged automatically, so that a
  change of the data is looked at before the job moves to it.
- Table I of the paper is the source of truth for the per-chip parameters.
- The user guide (`docs/benchmarks.md`) says how to clone the chips and run the
  tool on them, and what the current release does with them.

## Alternatives considered

**Keep two chips in the repository.** 4Q and 33Q would still put a megabyte of
data into the history, and the acceptance criteria name all eight chips, so the
other six would have to be fetched anyway.

**Fetch the data in the unit tests.** It makes every test run depend on the
network and on a repository outside this one, which a test suite that runs with
`-W error` on four interpreters should not.

**Pin the data in `noxfile.py`.** Renovate does not look into the noxfile, so
the pin would go stale without anyone noticing.

## Consequences

- A change of the loader that breaks a benchmark chip fails the benchmark job,
  not the unit tests; the fixture has to grow when a benchmark exposes a case it
  lacks.
- A new benchmark chip is a pull request against planar-superconducting-pd,
  followed by a Renovate update of the pin.
- `render` writes the port labels to layer 10, because the published layer map
  uses layers 1 to 5 for the obstacles and the routing result.
