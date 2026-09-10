# Phase 4, step 1 — handover

Written for whoever picks this up next. It says what the Corridor stage is,
what was measured, what is deliberately different from the prototype, and what
is still open.

Step 2, the detail router, has its own briefing:
[handover-detail-routing.md](handover-detail-routing.md).

- Checkout: `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`
- Branch: `phase-4-routing-stages`, based on `8ef300a`
- Prototype: `/Users/michaelfeldmeier/Documents/GitHub/FridgeCAD` (`0c5d6d9`)

## What this step delivers

The **Corridor** stage: every connection of the assignment is routed through
the partitions before any pixel is drawn. It is the prototype's own first
routing pass — `CapacityGrid::run_capacity_routing_requests`, whose failures
its benchmark table counts as `GlobalFail` — and it had no place in the
six-stage pipeline, so it becomes a stage of its own
([decision 0031](docs/design/decisions/0031-coarse-routing-is-its-own-stage.md)).

The run directory therefore numbers seven artifacts, and everything after `03`
moved up by one:

```text
01-capacity.fb  02-global.fb  03-assign.fb
04-corridor.fb  05-detail.fb  06-final.fb  07-geometry.fb
```

## The model

The search runs on the **partition graph**, not on pixels:

- A **crossing slot** is a place a wire may cross a partition border. The slots
  of one border sit one *crossing pitch* apart along it, and their count is what
  `PartitionBorder.budget` reports — one function, `grid::borderSlots`, gives
  both the budget and the places, so the two cannot disagree.
- A **state** is a slot together with the partition the wire carries on into,
  because a slot joins two of them and which side a wire came from decides where
  it may go next.
- An **edge** runs between two slots of one partition and costs the distance
  between them, plus what the slot has been charged.

Six rules make the plan one the detail router can follow:

1. One wire per slot. A place where three partitions meet is one slot of each
   border, and still one place: one wire crosses there and no more.
2. Two wires may not **cross** inside a partition, and may not **run along** each
   other there.
3. A wire may not run over a point another wire is **pinned** to: the point it is
   fed at, or the cell of its target port. Grazing a crossing slot at a point is
   allowed — the chords are a schematic of which partitions a wire uses, and the
   detail router crosses a border a little to either side of a slot as it sees
   fit — but a pin cannot move, so a second wire drawn over one is two wires on
   one point of the chip.
4. A wire may not cross a border outside the **ring the ports feed from** — the
   rectangle the feed points span. On it is not inside either: such a place sits
   beside some port's own feed point, and a wire crossing there has slipped
   behind that port.
5. A wire may not turn straight back into the partition it just left.
6. A wire may not cross itself or come back to a place it already used.

## What it took to reach zero

Every one of these was measured, and each is written up in `pipeline.md`.

1. **The crossing pitch is not the wire spacing.** The prototype divides a border
   at its wire spacing less twenty units and records no reason for the twenty.
   Using the design rule instead left one 45-qubit wire without a way into a
   chamber that then had three slots rather than four. The pitch is now
   `[stages.capacity] crossing_pitch`, a planning figure and not a clearance
   rule: how far two wires actually keep apart is the detail router's answer.
2. **The slots are placed by walking the border**, each a pitch past the last,
   which is what the prototype does. Spacing them against every slot already
   placed loses the ones on a border that bends back towards itself.
3. **A target that no border reaches leaves along its own port's approach.** The
   capacity grid places a target on the first cell beyond its port's band that
   was free before the bands; it does not open the band, and where the band and
   the artwork close around a port the free space left is a pocket. Five wires
   ended there. `digTargetBeyondBand` in `MQT::ScpdGrid` is the router-grid twin
   of that placement, is documented for exactly this, and is called by nothing —
   the way out is added to the partition graph instead.
4. **The prototype's sweep and the wires actually in the way are both needed.**
   The sweep rips the ring neighbours, which is rarely who is in the way, but it
   opens a large enough hole to matter. Asking the displaced route who holds its
   slots names the right wires, one or two per failure. The sweep alone leaves
   fifteen wires over and the targeted rip alone thirty-two; together they leave
   none.
5. **A slot remembers being wanted**, without which two wires trade one slot for
   ever.
6. **A route that is a short is not the end of the search.** Crossing itself is a
   property of the whole route, so the search cannot rule it out as it goes;
   throwing the search away with it left wires unrouted where a second-best way
   was there.
7. **Two passes after the last sweep.** A wire the sweep ripped on its way past
   has no sweep after it, so every wire still without a way is offered the room
   the others left; then each one left is offered a swap, undone unless it leaves
   fewer wires over.
8. **A wire stays inside the ring the ports feed from.** The prototype clamps
   every border pixel to the box its launcher slots span, and this stage had
   been leaving that out. Without it the eight chips route twenty-seven
   crossings behind a port, on four of them: the wire leaves through the ring,
   runs along the back of it and comes in again somewhere else.
9. **A wire may not run over another wire's pin.** This is the one rule the
   prototype does not have, and it is what placed the last two connections. A
   wire that tunnelled through the feed points of its neighbours was taking a
   way no crossing test could refuse it; once refused, it and the wire it had
   been standing on both found real ways.

## What the figures say

At the shipped defaults (`rounds = 12`, `max_relaxation = 30`,
`crossing_pitch = 165`), all eight chips:

| chip | connections | routed | unrouted | crossings | longest | slot lists/slots | corridor |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 4q  |  12 |  12 | 0 |   10 |  2 |    48/194 |  0.3 s |
| 9q  |  30 |  30 | 0 |   85 |  5 |   122/796 |  0.1 s |
| 17q |  58 |  58 | 0 |  305 | 10 |  479/1892 |  0.2 s |
| 21q |  70 |  70 | 0 |  414 |  9 |  476/2963 |  0.3 s |
| 33q | 110 | 110 | 0 |  966 | 13 |  896/5358 |  0.7 s |
| 45q | 150 | 150 | 0 | 1636 | 16 | 1407/8454 |  2.2 s |
| 57q | 190 | 190 | 0 | 2885 | 22 | 2533/12794 | 4.2 s |
| 69q | 230 | 230 | 0 | 4463 | 25 | 4008/23940 | 11.6 s |

The slot lists are one per border that offers a crossing, plus one for the ways
out of the pockets.

**All 968 connections have a corridor**, against nineteen unrouted over four
chips before the prototype was read properly. That is the prototype's own bar,
which its benchmark table reports it clears on all eight chips.

No two wires cross inside a partition, none lies along another, none runs over a
point another wire is pinned to, and none crosses a border outside the ring the
ports feed from; that is checked in ctest and again against the written
artifacts.

Runtime sits inside the prototype's own band for this pass, 0.35 s to 11.5 s.

## Deliberate deviations from the prototype

- **Per-connection identity survives the stage.** The prototype keeps only the
  union of the chords per partition and works out afterwards which wire is which,
  which costs it a cell at each end of every wire.
- **Two wires may not run along each other.** The prototype tests for a crossing
  only, and a wire that runs along a border passes through the slots of that
  border; a second wire crossing at one of them starts on top of it. A
  verification test found it on the 17- and 45-qubit chips.
- **The routing boundary is the ring the ports feed from.** The prototype has
  the same clamp; what is new is that the rectangle is derived from the feed
  points the assignment writes, rather than kept as four numbers beside the
  grid.
- **No wire runs over a point another wire is pinned to** — the point it is fed
  at, or the cell of its target port. Neither can be moved aside, and the
  crossing test cannot see the case at all, because a chord that stops on
  another one never reaches its far side. Measured: without the rule the eight
  chips carry eleven of these, on four of them, and three wires in a row run
  down one line of feed points on the 69-qubit chip. The
  prototype allows it: `check_edge_crossing` tests proper crossings, and the
  occupancy it keeps beside that is a set of pixels, never the lines between
  them. Ruling it out is also what routed the last two connections — the wire
  that tunnelled had been taking a way it was not entitled to, and once it was
  refused, both it and the wire it displaced found real ways.
- **A wire keeps the way it had** when a re-route finds nothing and the old way
  still fits. The prototype drops it.
- **The border budget is a wire count.** It was the length of the border in
  detail cells, which its own comment already denied.

## Open, in rough priority order

1. **The crossing pitch decides which chip is hard.** At the design rule's 185
   the 45-qubit chip left a connection over where the prototype's 165 does not.
   The pitch is one number for every chip on purpose, so this is not per-chip
   tuning to reach for. What it says is that the chips sit close to the edge of
   what the current placement of slots allows, and a placement that fits more
   slots into the same border is worth more than either value. The last border
   of a fan is where it shows: a fan of wires takes the rungs of one border in
   ring order, and the last wire needs a rung past the last one placed.
2. **A displacement chain was built and measured, and it is not in the code.**
   Where a fan holds every rung of a ladder, no single swap gains anything and
   the whole fan has to shift by one. A chain that displaces a wire, places it
   again, and settles what it has placed does exactly that, and it does shift
   the ladder — but every arrangement it reached left some other wire with
   nowhere to go. With the rule above in place it changes not one byte of any of
   the eight artifacts, so it was taken out again. It is written up here because
   the next stage may want it: `git log` has none of it, the reasoning is above.
3. **`digTargetBeyondBand` is still called by nothing.** The way out of a port's
   pocket is added to the partition graph rather than dug into the mask, because
   the partitions come from the artifact. The detail router builds its own mask
   and should dig; the two ways of saying the same thing then want reconciling.
4. **The stage has never been compared against the prototype's own output**, only
   against its formulation as read and against its published failure counts.

## Verification

```bash
cmake --build --preset release && ctest --preset release   # 287 tests
uv run --no-sync pytest test/python/unit                   # 125 tests
uvx nox -s schemas                                         # after any .fbs change
uvx nox -s stubs                                           # after any bindings change
uvx nox -s lint                                            # see the trap below
```

`uvx nox -s lint` **reformats 66 files this change never touched**, exactly as it
did in phase 3: the hook revisions are pinned and the committed tree does not
match them. Among them it rewrites every `# noqa: <code>` into
`# ruff: ignore[<name>]`, which is a repository-wide migration and not this
change. Revert everything the change does not own, and map those comments back
in the files it does. What is left after that is the four `ruff` findings and
the `typos` complaint about `std::countr_zero` that phase 3 already recorded.

The eight benchmarks, and the pictures:

```bash
mqt-scpd plan   -c benchmarks/9q/config.toml -o artifacts/9q --stage corridor
mqt-scpd plot   -c benchmarks/9q/config.toml --stage corridor --run-dir artifacts/9q -o artifacts/9q/9q-corridor.svg
mqt-scpd render -c benchmarks/9q/config.toml --stage corridor --run-dir artifacts/9q -o artifacts/9q/9q-corridor.gds
```

**The trap from phase 3 is still there:** `uv sync` does not rebuild the
extension when only C++ changed. Use

```bash
uv pip install --python .venv/bin/python --no-build-isolation --no-deps \
    --reinstall-package mqt-scpd -e .
```

and confirm behaviourally; a fresh `.so` timestamp only means an archive was
extracted.

**A place read back from the plan is not exactly where it was.** The border
samples are stored in layout units, so `toCell` on them gives 937.4999999999999
where the sample was 937.5. Every rule of this stage rests on the determinant of
three places being *exactly* zero when they lie on a line, and one ulp is enough
to turn a point that sits on a chord into one that sits beside it — two wires
then meet and the search does not see it. The slots are snapped back onto the
half-cell grid when they are read. It cost an hour, and the overlap test is what
found it.

**`uvx nox -s stubs` does not produce a usable stub.** It writes
`global: bytes` for the `assign` binding, which is a Python keyword and
therefore invalid syntax, and it drops the `Callable` type of `set_solver` for
`object`. The session then fails at its own `ruff-check` before the fix-up pass
that would have repaired the first of those. The committed stub is the repaired
form, so `route_corridor` was added to it by hand rather than by regenerating
over it. Whoever fixes the generator should regenerate the whole file then.
