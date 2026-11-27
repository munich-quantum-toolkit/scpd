# Terminal Output

Use this reference to read command output or check the format that MQT SCPD uses
in its CLI tests. The examples use the 17-qubit benchmark chip.

## Rules for Every Command

### Result and Error Lines

A file result on stdout starts with `✓` and gives the filename and size.

```text
✓ runs/17q-capacity.svg  810.0 kB
```

An error on stderr starts with `✗` and gives the reason.

```text
✗ runs/17q/01-capacity.fb is missing; run `mqt-scpd plan` up to the capacity step first
```

Warnings start with `⚠`. `plan` sends them to stdout and `inspect` sends them to
stderr. `doctor` has its own report format.

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

The example shortens the port lists and omits the solver availability section.
Use `--ports` to list every port and its role. `doctor` prints input errors and
preflight problems through `[run] stop_after` as `PROBLEM: <problem>`. The
report ends with `doctor: OK` or `doctor: <n> problem(s)`.

A problem in the configuration or the chip ends the report after that line:

```text
$ mqt-scpd doctor -c inputs/17q/config.toml
configuration: inputs/17q/config.toml
PROBLEM: inputs/17q/config.toml: [stages] has the unknown key 'example'
doctor: 1 problem(s)
```

`doctor` returns 0 when it found no problem and 1 otherwise. A warning does not
change the code.

## `plan`

`plan` prints one block per stage.

### A Run

Without `-v`, a run on the 17-qubit benchmark chip prints this:

```text
$ mqt-scpd plan -c inputs/17q/config.toml -o runs/17q
mqt-scpd plan 17q → runs/17q  ·  HiGHS 1.16.0

🧩 capacity
   356 partitions · 55 bottlenecks · 72 chains                     2.58s
   ✓ 01-capacity.fb  1.4 MB
🔳 global
   14 inner wires · 8 lattices · length 23419.67                   0.09s
   ✓ 02-global.fb  13.7 kB · fails 0
📌 assign
   58 ports · 7 launcher ends · objective 27.84 · optimal          0.53s
   ✓ 03-assign.fb  2.8 kB · fails 0
🧭 corridor
   round 1 forward    routed 53/58 · fails 5                       0.09s
   round 2 backward   routed 58/58 · fails 0                       0.09s
   routed 58/58 · 2 rounds                                         0.09s
   ✓ 04-corridor.fb  58.7 kB · fails 0
🧵 detail
   drawn 72/72 · 7 rounds · unrouted 0 · too close 0               0.33s
   ✓ 05-detail.fb  76.8 kB · fails 0

✓ 5 stages · 3.80s · stopped after detail · fails 0
```

With `--stage`, the header shows the run directory in place of the chip name.

`plan` accepts a new directory, an empty directory, or an existing run with a
valid `.mqt-scpd-run` marker. A preflight error prints only the error line on
stderr. A full run saves its input snapshot and artifacts after all requested
stages succeed. If a stage fails or the run stops before publication, the
previous run stays intact. The next planning command recovers an interrupted
publication.

### Levels

Each `-v` adds lines to the ones below it.

| Option | What the run shows in addition                                                               |
| ------ | -------------------------------------------------------------------------------------------- |
| none   | The header, stage results, round summaries when present, artifact lines and the closing line |
| `-v`   | The parameters of a stage, its steps, the size of each model and how each solve ended        |
| `-vv`  | One line per item, such as a wire, a feedline chain or a connection                          |
| `-vvv` | The log of HiGHS; the Gurobi backend solves without a log                                    |

Warnings and failures show at every level. For example, `-v` prints Capacity's
individual steps.

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
   ✓ 01-capacity.fb  1.4 MB

✓ 1 stage · 0.74s · stopped after capacity
```

### Lines

- A line of a block starts three spaces in. A line that only `-v` or more shows
  starts five spaces in and is dimmed.
- A line that a stage reports at a time carries the time since the stage began.
  The time ends at column 72 in a file. On a terminal it ends at the width of
  the terminal, but not after column 100. A line too long for the column carries
  its time after two spaces. A solver log line carries no time.
- An entry, such as a round, has a label and figures. The label fills a column
  of 18 characters, and the figures start after it. Corridor reports ratios such
  as `routed 56/58` and limits such as `max relaxation 1 of 4`.
- The artifact line is `✓ <file>  <size>`, followed by `· fails <n>` for a stage
  that reports fails. The assign stage reports them too, and its count is always
  zero.
- A failure of a stage starts with `✗` in its block and ends the run.
- The closing line starts with `✓` when nothing failed and with `✗` when a stage
  failed or a stage reported fails. After a failure, the closing line is
  `✗ stopped at <stage> after <time>`.

### Fails

Each artifact line shows the stage's fail count when available. The closing line
adds those counts. Any fails give the run exit code 1.

Global counts unreachable inner ports, ring ports without gate capacity, and
non-bridge ring ports cut off from a launcher. The {doc}`configuration`
reference explains the gate checks. Assignment either assigns every ring port
and shows `fails 0`, or stops with an error. Corridor counts connections it
could not route through the partitions. Detail counts connections with a
corridor but without a path, and wires that come within the wire clearance of
another wire. The result of Detail shows the two counts as `unrouted` and
`too close`.

### Stage Marks

Each step has a mark, a plain form and a color.

| Step     | Mark | Plain form | Color   |
| -------- | ---- | ---------- | ------- |
| capacity | 🧩   | `[#]`      | cyan    |
| global   | 🔳   | `[+]`      | blue    |
| assign   | 📌   | `[>]`      | magenta |
| corridor | 🧭   | `[~]`      | yellow  |
| detail   | 🧵   | `[-]`      | green   |

A mark is one emoji with the Unicode property `Emoji_Presentation=Yes` and
without a variation selector, so that every terminal draws it two cells wide.

### Progress

The live line shows the active stage, elapsed time, and progress. A solve adds
the objective, gap, and node count. The line disappears at the end of the stage.
Redirected output keeps only the lasting lines, even with `FORCE_COLOR`. Use
`--no-progress`, `TERM=dumb`, `TTY_INTERACTIVE=0`, or `TTY_COMPATIBLE=0` to hide
the live line.

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
2. Use `Report::entry` for each round and each later pass that changes the
   result. Corridor labels its sweeps `round <n> forward` and
   `round <n> backward`, counted from 1. It reports `back to round <n>` when it
   restores an earlier result and `free slots` when the final pass places
   another route.
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
   to `RunDirectory.run_stage`. The drawing commands take their stage names from
   `STAGE_FILES`. Set layer visibility and style in `python/mqt/scpd/plot.py`
   and add new layers to `_LAYERS`. Name the artifacts the view reads in
   `planning_geometry` in `python/mqt/scpd/planning.py` and in `_drawn` in
   `python/mqt/scpd/cli.py`. Add its GDS layers to `PLANNING_LAYERS` in
   `python/mqt/scpd/export/klayout.py`.
7. Extend the expected run in `test/python/unit/test_cli.py`, and add the mark
   to the table of this page.
