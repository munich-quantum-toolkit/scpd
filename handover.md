# Where the Final stage stands

The status of phase 4, step 3, for whoever picks it up next. What was built,
what it measures, and what is left.

- Checkout: `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`
- Branch: `phase-4-routing-stages`. **Nothing is committed — the user commits
  per phase.** Everything is in the working tree.
- Prototype: `/Users/michaelfeldmeier/Documents/GitHub/FridgeCAD` (`0c5d6d9`),
  read only.

Three documents go with this one:

| File | What it is |
| --- | --- |
| [user_final.md](user_final.md) | how to run the Corridor, Detail and Final stages: every command, every knob, and where each piece of the code is |
| [summary-final-routing.md](summary-final-routing.md) | what the Final stage does, what was measured, and what is deliberately different from the prototype |
| [handover-final-couplers.md](handover-final-couplers.md) | the three phases that are still open, with what the first two learned |

## The pipeline now

```text
01-capacity.fb  02-global.fb  03-assign.fb  04-corridor.fb  05-detail.fb  06-final.fb
                                                                          └ new
```

Six of seven artifacts are written. `07-geometry.fb` — the Finalize stage — is
untouched.

The Final stage runs five phases and leaves a snapshot after each:

| # | Phase | State |
| --- | --- | --- |
| 1 | `inner` — the inner circuit, inside its unit cells | **built** |
| 2 | `outer` — the ring, against the inner circuit and itself | **built** |
| 3 | `couplers` — the CPW couplers and the ports they create | open |
| 4 | `feedlines` — the launcher-to-launcher chains | open |
| 5 | `refined` — the refinement of those chains | open |

A phase that changes nothing leaves the state of the phase before it, so the
last three snapshots are the outer routing again until they are built. Each has
its own SVG and its own GDS layer already, so nothing about the rendering has
to change when they arrive.

## What was built

- **The stage.** `IFinalRouter`, `makeDubinsFinalRouter()`, and
  `src/pipeline/FinalRouter.cpp`: the router grid, the mask with the obstacle
  keepout baked in, the clearance field, and one rip-up-and-reroute driver that
  runs every routing phase. The prototype writes that loop out four times.
- **The design-rule check**, in `MQT::ScpdDrc`: wire clearance, wire loop and
  obstacle clearance in the cell view, each written once. The stage's own tests
  call them, and `mqt-scpd drc <run>` writes `drc.json`.
- **The pictures.** `plot --stage final --phase <name>` for each of the five,
  and `render --stage final` for one GDS that carries all five on layers 24 to
  28, beside the clearance band on 23.
- **`mqt-scpd plan -v`**, on the Corridor, Detail and Final stages: one line
  per round with how many wires were tried, how many settled, how many are
  still open and how many have no way at all.
- **Tests.** `test/pipeline/test_final_router.cpp` over all eight chips,
  `test/drc/test_rules.cpp` on wires built by hand, and Python tests for the
  phases, the report and the two new command-line options.

## What is different from the prototype, and why

Five corrections, each the answer to a defect its own pictures show. The full
argument is in [summary-final-routing.md](summary-final-routing.md); the short
form:

1. **The clearance holds against every wire**, through a field of how many
   wires guard each cell, not against the two beside it in the ring.
2. **A wire is judged against the full field**, after everything let go of for
   its search is put back. On the 21-qubit chip that took ten violations to
   one.
3. **A wire with no way is not entitled to one that breaks the rule.** During
   the rounds it stays undrawn and is tried again; only the rescue at the end
   draws it regardless. Taking a conflicting way in the first round is what put
   three consecutive wires one cell apart, and no later round could undo three
   at once.
4. **The relaxation runs both ways along the sweep.** The prototype only ever
   goes one way, so a wire blocked by the wire behind it has no move at all.
5. **Everything outside the ring of sources is blocked.** A wire that enters
   the strip between that rectangle and the chip outline comes back in
   somewhere else, having gone *around* the sources of the wires beside it.

Every knob is a length or a price per step. The prototype's cell counts —
`19`, `9`, `200`, `40`, `6` — are derived from `[design_rules]` or from the
chip, and its `FG_OBSTACLE_INFLATE` environment override is not carried over.

## What is open

1. **Phases 3 to 5.** [handover-final-couplers.md](handover-final-couplers.md)
   says what each has to do, which prototype function to read, and what the
   first two phases learned the hard way.
2. **The meander is not built.** `FinalParams.meander_length` is read and
   converted and nothing uses it, so a resonator's way is as long as its route
   makes it. It was in the plan for this step and is deliberately not in the
   tree: its only consumer is the coupler splice of phase 3, and building it
   before that consumer would lengthen every resonator, tighten the clearance
   and be measurable against nothing. The port is
   `dubin_router_opt.hpp:2182 meander_insertion` and `:8723
   compute_meander_between_points`.
3. **The lenpoint constraint is not ported.** It keeps a disc clear around the
   point where a resonator reaches its target length, which is where phase 3
   splices the coupler. Read `outer_res_lenpoint_clearance` in `FinalGrid.hpp`
   before phase 3.
4. **The stage's own counter and the checker disagree, and the checker is
   right.** `mqt-scpd plan -v` ends the 17-qubit run with "0 too close" while
   `mqt-scpd drc` reports two pairs, at 176.4 and 178.4 layout units against a
   rule of 185. Both are supposed to ask the same question — the same rule in
   layout units, the same junction and component exemptions — so one of the two
   reads something the other does not. `Driver::conflictsIn` in
   `src/pipeline/FinalRouter.cpp` and `checkClearance` in `src/drc/Rules.cpp`
   are the two, and the pairs to reproduce it on are 17q's (29, 30) and
   (41, 42). **This is the first thing to find out**: until it is settled, the
   driver cannot be trusted to repair what it cannot see.
5. **The wires that still come too close.** Every one of them is in `drc.json`
   of its run, with the two wires, the place and the distance. Two shapes: a
   near miss at 176 to 184 layout units, which comes from the straight runs the
   router adds to a way after it has searched; and a short at one cell, which
   comes from a wire taking a way through a neighbour's room.
6. **The last full measurement of all eight chips is one build old.** The
   figures in [summary-final-routing.md](summary-final-routing.md) mark which
   two chips were measured again; re-run the loop at the end of
   [user_final.md](user_final.md) to replace the rest.
7. **Nothing is compared against the prototype's own output**, only against its
   formulation as read and the counts in its checked-in run logs.

## Verification

```bash
cmake --build --preset release && ctest --preset release
uv run --no-sync pytest test/python/unit
uvx nox -s schemas          # after any .fbs change
uvx nox -s lint
```

All eight chips, with the pictures and the check, is the loop at the end of
[user_final.md](user_final.md).

## The traps, all of which bit

- `uv sync` does **not** rebuild the extension when only C++ changed. Use
  `uv pip install --python .venv/bin/python --no-build-isolation --no-deps
  --reinstall-package mqt-scpd -e .` and check behaviour, not the `.so` time.
  Three changes in a row produced byte-identical output here, and only one of
  the three was really a no-op.
- `cmake --build --preset release` does not pick up a new source file until the
  glob is re-run; touch the module's `CMakeLists.txt`.
- `mqt-scpd plan --stage <name>` reads the configuration **from the run
  directory**, not from the one named on the command line. Copy it over before
  an experiment, or the override does nothing.
- `uvx ruff check` rewrites every `# noqa: <code>` into `# ruff: ignore[<name>]`
  across the whole repository; the pinned hook in `nox -s lint` does not.
  Revert every file the change does not own.
- `uvx nox -s stubs` emits invalid Python for the `global` stage, so
  `python/mqt/scpd/pyscpd.pyi` is maintained by hand.
- The 69-qubit router grid is 3016 × 3016 cells and its search scratch is
  583 MB. Stay single-threaded: determinism is a stage contract and concurrency
  is phase 6.
