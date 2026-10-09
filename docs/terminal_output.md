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
   361 partitions · 62 bottlenecks · 69 chains                     1.88s
   ✓ 01-capacity.fb  1.3 MB
🔳 global
   14 inner wires · 8 lattices · length 23419.67                   0.04s
   ✓ 02-global.fb  14.1 kB · fails 0
📌 assign
   58 ports on 7 launchers · objective 27.84 · optimal             0.29s
   ✓ 03-assign.fb  2.8 kB · fails 0
🧭 corridor
   round 1 forward    routed 56/58 · fails 2                       0.07s
   round 2 backward   routed 58/58 · fails 0                       0.08s
   routed 58/58 · 2 rounds                                         0.08s
   ✓ 04-corridor.fb  60.5 kB · fails 0

✓ 4 stages · 2.29s · stopped after corridor · fails 0
```

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
| `-vvv` | The log of the solver                                                                       |

Warnings and failures show at every level.

## Lines

- A line of a block starts three spaces in. A line that only `-v` or more shows
  starts five spaces in and is dimmed.
- A line that a stage reports at a time carries the time since the stage began.
  The time ends at column 72 in a file. On a terminal it ends at the width of
  the terminal, but not after column 100. A line too long for the column carries
  its time after two spaces. A solver log line carries no time.
- An entry, such as a round, has a label and figures. The label fills a column
  of 18 characters, and the figures start after it.
- The artifact line is `✓ <file>  <size>`, followed by `· fails <n>` for a stage
  that can fail on part of its work.
- A warning starts with `⚠` and is yellow. A failure starts with `✗` and is red.
  A failure ends the run.
- The closing line starts with `✓` when nothing failed and with `✗` when a stage
  failed or a stage reported fails.

## Figures

A result, an entry and most lines of `-v` are figures, joined by ` · `. A figure
has one of these forms:

| Form      | Example             | Rule                                                                                      |
| --------- | ------------------- | ----------------------------------------------------------------------------------------- |
| Count     | `363 partitions`    | The number, then the noun; singular only for a count of one                               |
| Quantity  | `objective 27.84`   | The name, then the value; two decimals for a length or an objective, one for a percentage |
| Ratio     | `routed 56/58`      | The name, then the part and the whole                                                     |
| Verdict   | `fails 2`           | A count that should be zero: green at zero, red otherwise                                 |
| Parameter | `max relaxation 30` | The configuration key with spaces for underscores, then its value                         |

Figures are lower case and end without a full stop. Times are seconds below a
minute (`0.38s`), minutes and seconds below an hour (`53m 10s`), and hours and
minutes above. Sizes are decimal: `840 B`, `2.8 kB`, `1.3 MB`.

## Fails

A stage that can fail on part of its work reports how many parts failed. The
artifact line shows that count, and the closing line adds the counts of every
stage. A run that ends with fails exits with code 1.

| Stage    | A fail is                                         |
| -------- | ------------------------------------------------- |
| global   | An inner port that no wire of the circuit reaches |
| assign   | A ring port that the assignment gives no launcher |
| corridor | A connection without a way through the partitions |

## Stage Marks

Each step has a mark, a plain form and a colour.

| Step     | Mark | Plain form | Colour  |
| -------- | ---- | ---------- | ------- |
| capacity | 🧩   | `[#]`      | cyan    |
| global   | 🔳   | `[+]`      | blue    |
| assign   | 📌   | `[>]`      | magenta |
| corridor | 🧭   | `[~]`      | yellow  |

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

The live line is off when the output is no terminal, with `--no-progress`, and
with `TERM=dumb`. `NO_COLOR` turns colour off.

## Exit Codes

| Code | Meaning                                                              |
| ---- | -------------------------------------------------------------------- |
| 0    | The command did what it was asked                                    |
| 1    | An input has a problem, a stage failed, or a run ended with fails    |
| 2    | The arguments are wrong                                              |
| 130  | The command was interrupted                                          |

## Adding a Stage

A stage reports through the `Report` it is given and never prints.

1. Report the result exactly once, through `Report::result`. A stage that can
   fail on part of its work passes the number of fails as well.
2. Report each round of a stage that works in rounds through `Report::entry`,
   with the label `round <n> forward` or `round <n> backward`, counted from 1.
3. Build each figure with `counted`, `verdict` or one of the forms above, and
   join figures with `FIGURE_SEPARATOR`.
4. Report the steps, the parameters and the solves at `Detail::Steps`, and one
   line per item at `Detail::Items`. Ask `Report::wants` before building a line
   that takes time to build.
5. Report progress through `Report::progress` in every loop that can run for
   longer than a second.
6. Add the step to `STEPS` in `python/mqt/scpd/steps.py`, its mark to
   `STAGE_STYLES` in `python/mqt/scpd/console.py`, and its artifact to
   `STAGE_FILES` in `python/mqt/scpd/run.py`.
7. Extend the expected run in `test/python/unit/test_cli.py`.
