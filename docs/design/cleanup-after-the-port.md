# Cleanup after the port

A checklist of what leaves the repository once the port of the FridgeCAD
prototype is finished, meaning all six stages route and the acceptance criteria
hold. Everything listed here earns its place only while the port is in progress.
None of it is a defect; each entry says why it exists now and what removing it
means.

**Delete this document together with the last entry it lists.**

## 1. The SVG debug output

`mqt-scpd plot` renders a run as SVG, in Python, from the artifacts. It exists
because phases 2 to 4 build the grid, the routers and the final stage, and a
fail count alone is not enough feedback while doing that. Once the pipeline
routes, KLayout renders the result, and the per-stage picture has served its
purpose.

| What                                              | Note                                                     |
| ------------------------------------------------- | -------------------------------------------------------- |
| `python/mqt/scpd/plot.py`                         | The whole module, including `simplify` and `STAGES`      |
| `python/mqt/scpd/cli.py`                          | The `plot` subcommand, `command_plot` and the import     |
| `test/python/unit/test_plot.py`                   | The whole file                                           |
| `test/python/unit/test_cli.py`                    | The plot cases of the command-line tests                 |
| `docs/design/decisions/0021-debug-rendering-in-python.md` | Mark superseded rather than delete                |
| `ARCHITECTURE.md`                                 | The `plot.py` row, the `svg` node of the pipeline diagram, the sentence under "what we deliberately do not build" |
| `benchmarks/README.md`, `README.md`, `docs/`      | The `plot` line of every command example                 |

**Decide before removing.** The DRC overlay (`plot --stage final --drc`) is
listed in the pipeline document as a way to look at a violation. If that turns
out to be the only way to see one, the overlay stays and only the other stages go.

## 2. Six of the eight benchmark chips

The repository carries eight chips, 4 to 69 qubits, at 7.6 MB. They are all
needed while the port is measured against the prototype, because the acceptance
criteria name all eight. Afterwards two are enough: **4Q** as the smallest chip
that exercises every code path, and **33Q** as a representative large one.

| What                                                          | Note                                              |
| ------------------------------------------------------------- | -------------------------------------------------- |
| `benchmarks/9q/`, `17q/`, `21q/`, `45q/`, `57q/`, `69q/`       | 7.2 MB of the 7.6 MB in this directory            |
| `benchmarks/README.md`                                        | The chip table and the two per-chip value tables  |
| `test/io/test_benchmark_fixtures.cpp`                         | The 9Q fixture and its two port sequences         |
| `test/python/unit/test_chip.py`, `test_doctor.py`, `test_plot.py` | Every case parameterized over `9q`             |

**Decide before removing.** 9Q is the only chip whose `fixed_outer` is an open
sub-chain of its own `all_outer`, and one test covers exactly that. Either 33Q
takes that role, or the case moves to a hand-written fixture.

## 3. Comments that describe a future state

Several comments say what something *will* be rather than what it *is*. They
carried the intent across phases; once nothing is outstanding they are either
wrong or trivially true. Each one either loses its outlook or gains the sentence
that now describes the finished behaviour.

| Where                                   | What it says now                                                   |
| ---------------------------------------- | -------------------------------------------------------------------- |
| `schemas/geometry.fbs:11`               | "The three grid coordinate types arrive with `MQT::ScpdGrid` in phase 2" |
| `schemas/config.fbs:14`                 | "The stage, component and DRC parameters ... arrive with the stages that read them" |
| `schemas/artifacts.fbs:13`              | "Each stage appends the fields of its output when the stage is implemented" |
| `python/mqt/scpd/plot.py:11`            | "`plot` reads the chip and, from phase 2 on, the artifacts"        |
| `python/mqt/scpd/plot.py:31`            | `STAGES`, which maps a stage to the phase it arrives in            |
| `python/mqt/scpd/cli.py:58`             | "stage '...' arrives with phase 4"                                 |
| `test/io/test_artifacts_schema.cpp:99`  | "The outputs of the stages that are not implemented yet are empty tables" |
| `test/python/unit/test_cli.py:41`       | "a stage of a later phase names that phase"                        |
| `benchmarks/README.md`, last section    | "the stage that reads them is not ported yet"                      |
| `ARCHITECTURE.md:217`                   | "already shaped for the threading work in phase 2"                 |
| `docs/design/data-model.md:284,301`     | "validated by benchmark result in phase 4", "arrive with that module in phase 2" |
| `docs/conf.py:65`                       | "Remove this exclusion together with the documents once the port is complete" |

Two of these are more than wording. `STAGES` in `plot.py` and the message in
`cli.py` are a mechanism: a stage that does not exist yet is refused with the
phase that brings it. When every stage exists, the map becomes the plain list of
what `plot` can draw, and the message goes.

## 4. The design documents themselves

`docs/conf.py` excludes `docs/design/` from the Sphinx build, because those
documents address contributors carrying out the port rather than users. The
exclusion and the documents go together, and this checklist is one of them.
