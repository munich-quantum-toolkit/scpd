# Terminal Output

`mqt-scpd plan` prints one block per stage. This page fixes the format of that
output. Every stage follows it, and so does every stage that a later release
adds. The tests in `test/python/unit/test_console.py` and
`test/python/unit/test_cli.py` check the format.

## A Run

Without `-v`, a run on the 17-qubit benchmark chip prints this:

```text
mqt-scpd plan 17q → runs/17q  ·  HiGHS 1.11.0

🧩 capacity
   356 partitions · 55 bottlenecks · 72 chains                     0.75s
   ✓ 01-capacity.fb  1.3 MB
🔳 global
   14 inner wires · 8 lattices · length 23419.67                   0.03s
   ✓ 02-global.fb  13.7 kB · fails 0
📌 assign
   58 ports · 7 launcher ends · objective 27.84 · optimal          0.28s
   ✓ 03-assign.fb  2.8 kB · fails 0
🧭 corridor
   round 1 forward    routed 53/58 · fails 5                       0.09s
   round 2 backward   routed 58/58 · fails 0                       0.09s
   routed 58/58 · 2 rounds                                         0.09s
   ✓ 04-corridor.fb  58.7 kB · fails 0

✓ 4 stages · 1.14s · stopped after corridor · fails 0
```

A run that goes on to the routing stages adds two blocks before the blank line.
On the fixture chip of the tests they read:

```text
🧵 detail
   drawn 5/5 · 1 round · unrouted 0 · too close 0                  0.01s
   ✓ 05-detail.fb  3.2 kB · fails 0
🧷 final
   pre-feedline-routing             drawn 5/5 · short 2 · bad 0    0.04s
   pre-feedline-refinement          short 1 · long 0               0.05s
   feedline-insertion               edges 1/1 · angle cost 1       0.14s
   feedline-constrained-routing     drawn 6/6 · short 2 · long 0 · crossing 0 · bad 0  0.24s
   feedline-constrained-refinement  short 2 · long 0 · bad 0       0.32s
   drawn 6/6 · short 2 · long 0 · bad 0 · angle cost 1             0.32s
   ✓ 06-final.fb  30.4 kB · fails 0
```

The final block has one entry per phase, named as `[run] stop_after` names them,
and its result counts every wire of the plan. A chip with more than one feedline
chain ends the `feedline-insertion` entry and the result with the verdict of the
capacity check, `SAT` or `UNSAT`, or with `capacity check off` when
`[stages.final] capacity_check` is off.

The run has four parts:

- The header names the command, the chip, the run directory and the solver.
- A block per stage starts with the mark and the name of the stage. It holds the
  result of the stage, one line per round of a stage that works in rounds, and
  the artifact the stage wrote.
- A blank line.
- The closing line counts the stages, the time and the fails, and names the step
  after which the run stopped.

## Levels

Each `-v` adds lines to the ones below it.

| Option | What the run shows in addition                                                              |
| ------ | ------------------------------------------------------------------------------------------- |
| none   | The header, the result and the rounds of each stage, the artifact line and the closing line |
| `-v`   | The parameters of a stage, its steps, the size of each model and how each solve ended       |
| `-vv`  | One line per item, such as a wire, a feedline chain or a connection                         |
| `-vvv` | The log of HiGHS; the Gurobi backend solves without a log                                   |

Warnings and failures show at every level.

## Lines

- A line of a block starts three spaces in. A line that only `-v` or more shows
  starts five spaces in and is dimmed.
- A line that a stage reports at a time carries the time since the stage began.
  The time ends at column 72 in a file. On a terminal it ends at the width of
  the terminal, but not after column 100. A line too long for the column carries
  its time after two spaces. A solver log line carries no time.
- An entry, such as a round, has a label and figures. The label fills a column
  of 18 characters, and the figures start after it. In the final block the
  labels are the names of its phases, and the column is 32 characters wide.
- The artifact line is `✓ <file>  <size>`, followed by `· fails <n>` for a stage
  that reports fails. The assign stage reports them too, and its count is always
  zero.
- A warning starts with `⚠` and is yellow. A failure starts with `✗` and is red.
  A failure ends the run.
- The closing line starts with `✓` when nothing failed and with `✗` when a stage
  failed or a stage reported fails. After a failure, the closing line is
  `✗ stopped at <stage> after <time>`.

## Figures

A result, an entry and most lines of `-v` are figures, joined by ` · `. A figure
has one of these forms:

| Form      | Example                 | Rule                                                                                      |
| --------- | ----------------------- | ----------------------------------------------------------------------------------------- |
| Count     | `363 partitions`        | The number, then the noun; singular only for a count of one                               |
| Quantity  | `objective 27.84`       | The name, then the value; two decimals for a length or an objective, one for a percentage |
| Ratio     | `routed 56/58`          | The name, then the part and the whole                                                     |
| Verdict   | `fails 2`               | A count that should be zero: green at zero, red otherwise                                 |
| Parameter | `max relaxation 4`      | The configuration key with spaces for underscores, then its value                         |
| Limit     | `max relaxation 1 of 4` | A parameter that the stage limits: the value it uses, then `of` and the configured value  |

Figures are lower case and end without a full stop. Times are seconds below a
minute (`0.38s`), minutes and seconds below an hour (`53m 10s`), and hours and
minutes above. Sizes are decimal: `840 B`, `2.8 kB`, `1.3 MB`.

## Fails

A stage that can fail on part of its work reports how many parts failed. The
artifact line shows that count, and the closing line adds the counts of every
stage. A run that ends with fails exits with code 1.

| Stage    | A fail is                                                                                                                                                                                |
| -------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| global   | An inner port that no wire of the circuit reaches, a ring port that the gates of its chain have no room for, or a ring port without a bridge that its chain does not carry to a launcher |
| assign   | None: the stage gives every ring port a launcher, or it stops with an error. Its artifact line shows `fails 0`                                                                           |
| corridor | A connection without a way through the partitions                                                                                                                                        |
| detail   | A wire without a way, or a wire whose way comes within the design rule of another                                                                                                        |
| final    | A wire with no usable way: unrouted, open, crossing a feedline, or meeting itself. A resonator short of or over its target length is a red figure of the result, not a fail              |

## Stage Marks

Each step has a mark, a plain form and a color.

| Step     | Mark | Plain form | Color       |
| -------- | ---- | ---------- | ----------- |
| capacity | 🧩   | `[#]`      | cyan        |
| global   | 🔳   | `[+]`      | blue        |
| assign   | 📌   | `[>]`      | magenta     |
| corridor | 🧭   | `[~]`      | yellow      |
| detail   | 🧵   | `[-]`      | green       |
| final    | 🧷   | `[=]`      | bright_blue |

A mark is one emoji with the Unicode property `Emoji_Presentation=Yes` and
without a variation selector, so that every terminal draws it two cells wide. A
step that a later release adds follows the same rule.

Where the output cannot encode a symbol, the run writes its plain form:

| Symbol | Plain form |
| ------ | ---------- |
| ✓      | `ok`       |
| ✗      | `FAIL`     |
| ⚠      | `!`        |
| ·      | `-`        |
| →      | `->`       |
| ×      | `x`        |

## Progress

On a terminal, a live line at the bottom shows the stage that runs: a spinner,
the name of the stage, its time, what it is doing and how far it has got. A
solve shows its objective, its gap and its node count. The live line goes away
when the stage ends, so a log or a pipe receives the lasting lines only.

The live line is off when the output is not a terminal, with `--no-progress`,
with `TERM=dumb`, and with `TTY_INTERACTIVE=0`. `NO_COLOR` turns color off.
`TTY_COMPATIBLE=0` turns the live line and color off. `FORCE_COLOR` and
`TTY_COMPATIBLE=1` turn color on, also in a file, but a file still gets no live
line and keeps the time at column 72.

## Exit Codes

| Code | Meaning                                                              |
| ---- | -------------------------------------------------------------------- |
| 0    | The command did what it was asked                                    |
| 1    | An input has a problem, a stage failed, or a run ended with fails    |
| 2    | The arguments are wrong                                              |
| 130  | The command was interrupted                                          |

A command whose output goes to a reader that closes early, such as
`mqt-scpd inspect run/01-capacity.fb | head`, stops without a message and
returns 1.

## Adding a Stage

A stage reports through the `Report` it is given and never prints.

1. Report the result exactly once, through `Report::result`. A stage that can
   fail on part of its work passes the number of fails as well.
2. Report each round of a stage that works in rounds through `Report::entry`,
   with the label `round <n> forward` or `round <n> backward`, counted from 1. A
   pass after the rounds that changes the result gets an entry of its own, such
   as `back to round <n>` and `free slots` in the Corridor stage.
3. Build each figure with `counted`, `verdict` or one of the forms above, and
   join figures with `FIGURE_SEPARATOR`.
4. Report the steps, the parameters and the solves at `Detail::Steps`, and one
   line per item at `Detail::Items`. Ask `Report::wants` before building a line
   that takes time to build.
5. Report progress through `Report::progress` in every loop that can run for
   longer than a second, so that the live line moves and Ctrl-C stops the stage.
   Outside a solve, a progress report is the only place where a stage notices
   Ctrl-C.
6. Add the step to `STEPS` in `python/mqt/scpd/steps.py` and its mark to
   `STAGE_STYLES` in `python/mqt/scpd/console.py`. In `python/mqt/scpd/run.py`,
   add its artifact to `STAGE_FILES`, the configuration sections it reads to
   `STEP_INPUTS`, the artifacts it reads to `ARTIFACT_INPUTS`, and its core call
   to `RunDirectory.run_stage`. If `plot` and `render` draw the stage, add it to
   `STAGES` in `python/mqt/scpd/plot.py`, name the artifacts its picture reads
   in `planning_geometry` in `python/mqt/scpd/planning.py` and in `_drawn` in
   `python/mqt/scpd/cli.py`, and add its GDS layers to `PLANNING_LAYERS` in
   `python/mqt/scpd/export/klayout.py`.
7. Extend the expected run in `test/python/unit/test_cli.py`.
