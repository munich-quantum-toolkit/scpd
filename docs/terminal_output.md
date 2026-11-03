# Terminal Output

This page fixes what each command of `mqt-scpd` prints. The first section holds
the rules that every command follows. A section per command then shows its
output on the 17-qubit benchmark chip. The tests in
`test/python/unit/test_console.py`, `test/python/unit/test_cli.py` and
`test/python/unit/test_doctor.py` check the format.

## Rules for Every Command

### Result and Error Lines

A command that writes a file prints one result line on stdout. The line starts
with `✓`, names the file and gives its size:

```text
✓ runs/17q-capacity.svg  810.0 kB
```

A command that cannot do what it was asked prints one error line on stderr. The
line starts with `✗` and says why:

```text
✗ runs/17q/01-capacity.fb is missing; run `mqt-scpd plan` up to the capacity step first
```

A warning starts with `⚠`. `plan` prints it on stdout, and `inspect` prints it
on stderr. `doctor` is the exception to these rules; its section describes its
report.

### Symbols and Colors

`✓` is green, `✗` is red and `⚠` is yellow. Where the output cannot encode a
symbol, the command writes its plain form:

| Symbol | Plain form |
| ------ | ---------- |
| ✓      | `ok`       |
| ✗      | `FAIL`     |
| ⚠      | `!`        |
| ·      | `-`        |
| →      | `->`       |
| ×      | `x`        |

`NO_COLOR` turns color off. `TTY_COMPATIBLE=0` turns color and the live line of
`plan` off. `FORCE_COLOR` and `TTY_COMPATIBLE=1` turn color on, also in a file.

### Figures

A result line, a line of `plan` and most lines of `plan -v` are figures, joined
by ` · `. A figure has one of these forms:

| Form      | Example                 | Rule                                                                      |
| --------- | ----------------------- | ------------------------------------------------------------------------- |
| Count     | `363 partitions`        | The number, then the noun; singular only for a count of one               |
| Quantity  | `length 23419.67`       | The name, then the value; two decimals for a length, one for a percentage |
| Verdict   | `fails 2`               | A count that should be zero: green at zero, red otherwise                 |
| Parameter | `crossing pitch 165.00` | The configuration key with spaces for underscores, then its value         |

Figures are lower case and end without a full stop. Times are seconds below a
minute (`0.38s`), minutes and seconds below an hour (`53m 10s`), and hours and
minutes above. Sizes are decimal: `840 B`, `2.8 kB`, `1.3 MB`.

### Exit Codes

| Code | Meaning                                                                                     |
| ---- | ------------------------------------------------------------------------------------------- |
| 0    | The command did what it was asked                                                           |
| 1    | An input has a problem, `doctor` found a problem, a stage failed, or a run ended with fails |
| 2    | The arguments are wrong                                                                     |
| 130  | The command was interrupted                                                                 |

On Ctrl-C, a command prints `✗ interrupted` on stderr. A command whose output
goes to a reader that closes early, such as
`mqt-scpd inspect runs/17q/01-capacity.fb | head`, stops without a message and
returns 1.

Wrong arguments print the usage and the error of `argparse` on stderr:

```text
usage: mqt-scpd [-h] {doctor,plan,plot,render,list-algorithms,inspect} ...
mqt-scpd: error: plot --stage capacity is drawn from a run; pass --run-dir
```

## `doctor`

`doctor` checks a configuration and its chip before a run. It prints a report on
stdout:

```text
$ mqt-scpd doctor -c inputs/17q/config.toml
configuration: inputs/17q/config.toml
chip input: inputs/17q/routing_config.json

obstacles: 95
ports: 202

role          ports  pattern / first labels
launcher         48  ^Chip\.port\d+$
                     Chip.port0, Chip.port1, Chip.port2
resonator        17  ^Qb\d+\.port0$
                     Qb17.port0, Qb13.port0, Qb9.port0
conventional     41  ^(Qb\d+\.port1|Coupler\d+_\d+\.port0)$
                     Coupler13_17.port0, Coupler9_13.port0, Coupler5_9.port0
bridge_pair      96  ^Coupler\d+_\d+\.port[1-4]$
                     Coupler13_17.port1, Coupler13_17.port2, Coupler13_17.port3

outer port ring
all_outer: 76 ports, entering at Qb15.port1
  Qb15.port1, Qb15.port0, Coupler14_15.port3, …
fixed_outer: 44 ports
  Qb15.port0, Coupler14_15.port0, Qb14.port1, …
doctor: OK
```

The example shortens the two port lists. The report has these parts:

1. The configuration and the chip input it names.
2. One line `WARNING: <warning>` per key that no stage reads any more.
3. One line `PROBLEM: <problem>` per problem. A problem is an error in the
   configuration or the chip, or anything that would stop a run of the steps up
   to `[run] stop_after`.
4. The counts of obstacles and ports.
5. The roles of the ports: the count, the pattern and the first three labels of
   each role.
6. With `--ports`, one line per port with its label and its role.
7. The outer port ring: the `all_outer` and the `fixed_outer` sequences.
8. The verdict: `doctor: OK`, or `doctor: <n> problem(s)`.

A problem in the configuration or the chip ends the report after that line:

```text
$ mqt-scpd doctor -c inputs/17q/config.toml
configuration: inputs/17q/config.toml
PROBLEM: inputs/17q/config.toml: [stages] has the unknown key 'assignment'
doctor: 1 problem(s)
```

`doctor` returns 0 when it found no problem and 1 otherwise. A warning does not
change the code.

## `plan`

`plan` prints one block per stage. Every stage follows the format of this
section, and so does every stage that a later release adds.

### A Run

Without `-v`, a run on the 17-qubit benchmark chip prints this:

```text
$ mqt-scpd plan -c inputs/17q/config.toml -o runs/17q
mqt-scpd plan 17q → runs/17q

🧩 capacity
   356 partitions · 55 bottlenecks · 72 chains                     0.74s
   ✓ 01-capacity.fb  1.3 MB

✓ 1 stage · 0.75s · stopped after capacity
```

The run has these parts:

- The header names the command, the chip and the run directory. With `--stage`,
  the header names the run directory in place of the chip.
- One line `⚠ <warning>` per key that no stage reads any more.
- A block per stage starts with the mark and the name of the stage. It holds the
  result of the stage and the artifact the stage wrote.
- A blank line.
- The closing line counts the stages, the time and the fails, and names the step
  after which the run stopped.

`plan` checks the configuration and the chip before its first stage. A problem
prints one error line on stderr and no header, and the run directory stays as it
was.

### Levels

Each `-v` adds lines to the ones below it.

| Option | What the run shows in addition                                               |
| ------ | ---------------------------------------------------------------------------- |
| none   | The header, the result of each stage, the artifact line and the closing line |
| `-v`   | The parameters of a stage and its steps                                      |
| `-vv`  | One line per item, such as a wire or a chain                                 |

Warnings and failures show at every level. With `-v`, the capacity stage prints
its steps:

```text
$ mqt-scpd plan --stage capacity -o runs/17q -v
mqt-scpd plan 17q → runs/17q

🧩 capacity
     capacity grid 25×25 · detail grid 750×750                     0.01s
     154 targets · 48 launchers                                    0.01s
     medial axis of 17188 cells                                    0.03s
     243 candidate bottlenecks                                     0.03s
     72 chains cross 55 of 243 candidate bottlenecks               0.34s
     356 partitions · 600 borders                                  0.74s
   356 partitions · 55 bottlenecks · 72 chains                     0.74s
   ✓ 01-capacity.fb  1.3 MB

✓ 1 stage · 0.74s · stopped after capacity
```

### Lines

- A line of a block starts three spaces in. A line that only `-v` or more shows
  starts five spaces in and is dimmed.
- A line that a stage reports at a time carries the time since the stage began.
  The time ends at column 72 in a file. On a terminal it ends at the width of
  the terminal, but not after column 100. A line too long for the column carries
  its time after two spaces.
- An entry, such as a chain at `-vv`, has a label and figures. The label fills a
  column of 18 characters, and the figures start after it.
- The artifact line is `✓ <file>  <size>`, followed by `· fails <n>` for a stage
  that reports fails.
- A failure of a stage starts with `✗` in its block and ends the run.
- The closing line starts with `✓` when nothing failed and with `✗` when a stage
  failed or a stage reported fails. After a failure, the closing line is
  `✗ stopped at <stage> after <time>`.

### Fails

A stage that can fail on part of its work reports how many parts failed. The
artifact line shows that count, and the closing line adds the counts of every
stage. A run that ends with fails exits with code 1. The capacity stage reports
no fails: it plans the whole chip, or it stops with an error.

### Stage Marks

Each step has a mark, a plain form and a color.

| Step     | Mark | Plain form | Color |
| -------- | ---- | ---------- | ----- |
| capacity | 🧩   | `[#]`      | cyan  |

A mark is one emoji with the Unicode property `Emoji_Presentation=Yes` and
without a variation selector, so that every terminal draws it two cells wide. A
step that a later release adds follows the same rule.

### Progress

On a terminal, a live line at the bottom shows the stage that runs: a spinner,
the name of the stage, its time, what it is doing and how far it has got. The
live line goes away when the stage ends, so a log or a pipe receives the lasting
lines only.

The live line is off when the output is not a terminal, with `--no-progress`,
with `TERM=dumb`, with `TTY_INTERACTIVE=0` and with `TTY_COMPATIBLE=0`. A file
gets no live line, also with `FORCE_COLOR`, and keeps the time at column 72.

## `plot`

`plot` writes an SVG picture and prints one result line:

```text
$ mqt-scpd plot -c inputs/17q/config.toml -o runs/17q-layout.svg
✓ runs/17q-layout.svg  226.2 kB
$ mqt-scpd plot --run-dir runs/17q --stage capacity -o runs/17q-capacity.svg
✓ runs/17q-capacity.svg  810.0 kB
```

`plot` refuses a planning stage whose artifact is missing, or whose inputs have
changed since its step:

```text
$ mqt-scpd plot --run-dir runs/17q --stage capacity -o runs/17q-capacity.svg
✗ [grid] in config.toml changed after the capacity step wrote 01-capacity.fb; run the capacity step again
```

## `render`

`render` writes a GDSII or OASIS file and prints one result line. After the
size, the line gives the format, the polygons and the ports of the chip, and the
shapes of a planning stage:

```text
$ mqt-scpd render -c inputs/17q/config.toml -o runs/17q-layout.gds
✓ runs/17q-layout.gds  143.9 kB · GDS2 · 95 polygons · 202 ports
$ mqt-scpd render --run-dir runs/17q --stage capacity -o runs/17q-capacity.oas
✓ runs/17q-capacity.oas  36.5 kB · OASIS · 95 polygons · 202 ports · 2980 planning shapes
```

`render` refuses a planning stage in the same way as `plot`. Without KLayout, it
prints `✗ render needs KLayout; install it with 'mqt-scpd[klayout]'`.

## `inspect`

`inspect` prints an artifact as JSON on stdout. With `-o`, it writes the JSON to
a file and prints one result line:

```text
$ mqt-scpd inspect runs/17q/01-capacity.fb -o runs/17q-capacity.json
✓ runs/17q-capacity.json  8.8 MB
```

`inspect` also prints an artifact whose inputs have changed since its step. It
first prints a warning on stderr:

```text
⚠ [grid] in config.toml changed after the capacity step wrote 01-capacity.fb; run the capacity step again
```

## `list-algorithms`

`list-algorithms` prints one line per stage: the stage in a column of 18
characters, then the names of its implementations, joined by `, `.

```text
$ mqt-scpd list-algorithms
capacity-planner   watershed
```

## Adding a Stage

A stage reports through the `Report` it is given and never prints.

1. Report the result exactly once, through `Report::result`. A stage that can
   fail on part of its work passes the number of fails as well.
2. Build each figure with `counted`, `verdict` or one of the forms above, and
   join figures with `FIGURE_SEPARATOR`.
3. Report the steps and the parameters at `Detail::Steps`, and one line per item
   at `Detail::Items`. Ask `Report::wants` before building a line that takes
   time to build.
4. Report progress through `Report::progress` in every loop that can run for
   longer than a second, so that the live line moves and Ctrl-C stops the stage.
   A progress report is the only place where a stage notices Ctrl-C.
5. Add the step to `STEPS` in `python/mqt/scpd/steps.py` and its mark to
   `STAGE_STYLES` in `python/mqt/scpd/console.py`. In `python/mqt/scpd/run.py`,
   add its artifact to `STAGE_FILES`, the configuration sections it reads to
   `STEP_INPUTS`, the artifacts it reads to `ARTIFACT_INPUTS`, and its core call
   to `RunDirectory.run_stage`. If `plot` and `render` draw the stage, add it to
   `STAGES` in `python/mqt/scpd/plot.py`, name the artifacts its picture reads
   in `planning_geometry` in `python/mqt/scpd/planning.py` and in `_drawn` in
   `python/mqt/scpd/cli.py`, and add its GDS layers to `PLANNING_LAYERS` in
   `python/mqt/scpd/export/klayout.py`.
6. Extend the expected run in `test/python/unit/test_cli.py`, and add the mark
   to the table of this page.
