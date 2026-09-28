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

| What                                                      | Note                                                                                                              |
| --------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------- |
| `python/mqt/scpd/plot.py`                                 | The whole module, including `simplify` and `STAGES`                                                               |
| `python/mqt/scpd/cli.py`                                  | The `plot` subcommand, `command_plot` and the import                                                              |
| `test/python/unit/test_plot.py`                           | The whole file                                                                                                    |
| `test/python/unit/test_cli.py`                            | The plot cases of the command-line tests                                                                          |
| `docs/design/decisions/0021-debug-rendering-in-python.md` | Mark superseded rather than delete                                                                                |
| `ARCHITECTURE.md`                                         | The `plot.py` row, the `svg` node of the pipeline diagram, the sentence under "what we deliberately do not build" |
| `README.md`, `docs/benchmarks.md`                         | The `plot` line of every command example                                                                          |

**Decide before removing.** The DRC overlay (`plot --stage final --drc`) is
listed in the pipeline document as a way to look at a violation. If that turns
out to be the only way to see one, the overlay stays and only the other stages
go.

## 2. Comments that describe a future state

Several comments say what something *will* be rather than what it *is*. They
carried the intent across phases; once nothing is outstanding they are either
wrong or trivially true. Each one either loses its outlook or gains the sentence
that now describes the finished behaviour.

| Where                                  | What it says now                                                                    |
| -------------------------------------- | ----------------------------------------------------------------------------------- |
| `schemas/geometry.fbs:11`              | "The three grid coordinate types arrive with `MQT::ScpdGrid` in phase 2"            |
| `schemas/config.fbs:14`                | "The stage, component and DRC parameters ... arrive with the stages that read them" |
| `schemas/artifacts.fbs:13`             | "Each stage appends the fields of its output when the stage is implemented"         |
| `python/mqt/scpd/plot.py:11`           | "`plot` reads the chip and, from phase 2 on, the artifacts"                         |
| `python/mqt/scpd/plot.py:31`           | `STAGES`, which maps a stage to the phase it arrives in                             |
| `python/mqt/scpd/cli.py:58`            | "stage '...' arrives with phase 4"                                                  |
| `test/io/test_artifacts_schema.cpp:99` | "The outputs of the stages that are not implemented yet are empty tables"           |
| `test/python/unit/test_cli.py:41`      | "a stage of a later phase names that phase"                                         |
| `ARCHITECTURE.md:217`                  | "already shaped for the threading work in phase 2"                                  |
| `docs/design/data-model.md:284,301`    | "validated by benchmark result in phase 4", "arrive with that module in phase 2"    |

Two of these are more than wording. `STAGES` in `plot.py` and the message in
`cli.py` are a mechanism: a stage that does not exist yet is refused with the
phase that brings it. When every stage exists, the map becomes the plain list of
what `plot` can draw, and the message goes.

## 3. The design documents themselves

The design documents address contributors carrying out the port rather than
users, and this checklist is one of them. They go together once the port is
complete.
