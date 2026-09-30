# A\* over coupler option prefixes

Written for whoever takes this further. It replaces the per-chain trellis of
the coupler insertion with a best-first search over **prefixes** of coupler
options, so that every feedline edge is routed against the ways the options
before it have already laid. Read
[handover-cpw-coupler-insertion.md](handover-cpw-coupler-insertion.md) first —
this document assumes its vocabulary (chains, options, the analytic bound, the
rounds, `stateFaults`) and describes only what changes.

- Checkout: `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`
- Branch `phase-4-routing-stages`, HEAD **`b002669` A\* exact coupler
  insertion**, which carries the search itself. Four things sit
  **uncommitted** on top of it: the pad-crossing filter in
  `Driver::makeOption`, the shortfall rule, the solo fence and the kept ways.
  The user commits per phase — the work is in the tree and this says what was
  verified.
- **Every switch below defaults to what was measured last.**
  `SCPD_CHAIN_ASTAR=0` is the way back to the trellis and `SCPD_CHAIN_DP=0`
  from there to the greedy; the other three are named where they are
  described.

## Why

The trellis rests on one assumption, which `ChainTrellis.hpp` states:

> *the price of a step depends on the choice at both of its ends and on
> nothing else*

**It is false.** Two feedline edges that meet at a coupler block each other,
so a step's price depends on what was laid before it. The trellis prices a
pair that cannot both be built as though it could, `stateFaults` counts the
difference, and the commit is where an edge is lost.

Carrying the predecessor in the node — the second order — closed that for the
two edges that **share** a coupler, and on 17q that was the whole of it: six
undrawn edges to none. It never reached 57q and 69q. 57q lost the same five
edges whether its widest chains were solved at second order or handed back to
the first, at 2641 s against 810 s for the privilege, so what was lost there
was blocked by edges that are **not** neighbours, and no width of node reaches
that.

This is that approximation dropped. A node is a prefix, each of its edges is
routed against the ways the prefix has laid, and a prefix that blocks itself
costs everything and the search takes another combination.

## Where it stands

Defaults throughout, no environment variable set, one run per chip,
`stop_after = "couplers"`, `repair_trials = 0`, `--stage final`. One sweep,
2026-09-30.

| chip | not drawn | angle | insertion | `stateFaults` | | trellis: not drawn / angle / insertion |
| --- | ---: | ---: | ---: | ---: | --- | --- |
| 4q | **0** | 8 | 0.3 s | 0 | | 0 / 8 / 0.3 s |
| 9q | **0** | 20 | 0.6 s | 0 | | 0 / 20 / 0.6 s |
| 17q | **0** | 47 | **2.0 s** | 0 | | 0 / 51 / 11.2 s |
| 21q | **0** | 48 | 3.1 s | 0 | | 0 / 48 / 2.9 s |
| 33q | **0** | 76 | **7.3 s** | 0 | | 0 / 76 / 10.6 s |
| 45q | **0** | 105 | **23.6 s** | 0 | | 0 / 105 / 36.0 s |
| 57q | **0** | 132 | **32.1 s** | **0** | | **2** / 133 / 356.8 s |
| 69q | **0** | 166 | **70.9 s** | **22** | | **11** / 162 / 1023.3 s |

**No chip loses a feedline edge**, where the trellis lost two on 57q and
eleven on 69q and spent 357 s and 1023 s doing it. Wall and CPU time agree
throughout — 69q 380.6 s against 376.6 s user — so no row is an artefact of
the machine being busy.

**69q's zero is not the other seven's zero.** Twenty-two of its eighty-one
edges lie across others; they count as drawn only because
`SCPD_CHAIN_KEEP_WAYS` skips the commit's test. **For 69q the figure is the
22, not the 0.** On the other seven the zero is real: no fault means every way
survives the chip it lies on. See *Keeping the ways*.

**Three chains are not proved optimal, each for a different reason:**

| chip | chain | what the log says |
| --- | --- | --- |
| 17q | 2 | `best found, an end pair laid the other way round` — the reorder, which costs the proof |
| 57q | 4 | `best in the time given` — the budget; the price is 220000 either way |
| 69q | 8 | `out of time before any run of options joined the chain` |

**69q's chain 8 is what the ten-second budget costs.** In 10 s it finds no
complete run at all (223 edge searches) and is left as it stands; at 180 s it
came home at 240000 through the end-pair reorder. That is why 69q settles 11
of 12 chains rather than 12, why its faults are 22 rather than 17, and why its
angle is 166 rather than 164. `SCPD_CHAIN_ASTAR_SECONDS=30` is the obvious
thing to try and has not been run.

**Read the angle column only where the same edges were drawn.** An edge that
was not drawn turns nowhere and `angleCostOf` scores it zero, so the trellis's
162 on 69q is over 70 drawn edges and our 166 over 81.

## The search

`routing::solveChainAStar` (`include/mqt-scpd/routing/ChainSearch.hpp`,
`src/routing/ChainSearch.cpp`), router-free and unit-tested alone, exactly as
`routing::solveTrellis` beside it.

**A node is a prefix** — the options chosen for layers `0..k`. It is held in
an arena as `{parent, choice, layer, price, real}`, so the run of choices is
read by walking parents and no node copies one.

**The price** of a node is the real routed price of the edges in its prefix,
each routed against a corridor holding everything committed elsewhere **plus
the ways of the earlier edges of this same prefix**.

**The heuristic** is `rest[layer][choice]`: the cheapest run of analytic
bounds (`routing::AnalyticDubins`) from that choice to the end of the chain,
settled by one backward pass over the bound matrices — `O(L n²)` answers of a
function that costs half a microsecond. It is the exact optimum of the
relaxation in which no step blocks any other, so it is a lower bound under
*any* prefix, and A\* with it returns the true optimum.

**A step is routed when the search reaches it, not when it is generated.** A
child is pushed carrying the analytic bound for its own step; when it is
popped, that step is routed for real, its price and `f` are corrected and it
goes back on the queue. Only a node popped with its own step already real is
expanded. The placeholder is a lower bound, so this changes nothing about the
answer, and it means a step is routed exactly when the search commits to
looking at it. `solveTrellis` defers its pricing the same way.

**Ties break on the deeper prefix, then on the older node.** Neither can bear
on the price of the answer; they are there so that two runs of one chain
answer the same, and preferring the deeper one dives at the end of the chain
rather than spreading across a layer.

**Nothing is merged.** The node count is exponential in the worst case, and
the edge cache and the time limit below are the answer to that.

## There are no rounds

`Driver::optimizeChainsPrefix` is one pass over the chains, each solved
against the ways the chains before it settled on. `optimizeChainsExact` and
its four rounds stay for the trellis and are untouched.

The rounds exist because the trellis prices a chain against a fence it cannot
see all of: its answer may not survive its own fence, so a later round has
something to correct, and the round that is kept has to be chosen by
`stateFaults`. None of that applies here, because the prefix search routes
every edge against the ways the options before it laid.

**That is measured, not argued.** Under the rounds, seven of the eight chips
reported **zero faults after round 0** and the loop stopped there of its own
accord; only 69q ever paid for a second round, where it changed nothing and
cost 200 s. `stateFaults` is still reported once, at the end, because it is
the number this whole approach is judged by, but it decides nothing.

**`stateFaults` is pessimistic, and by a factor of two.** It said 17 on 69q
where the commit kept 72 edges, routed 6 again and lost 3 — nine touched. It
judges every edge against every other edge's *searched* way, while the commit
lays them in order, so an edge that is routed again changes the ground for the
ones after it and some that would have faulted then fit. Read it as an upper
bound on what the commit has to repair, not as the commit's own count.

**What the rounds were worth on 69q, said plainly.** They gave a chain that
ran out of time a second attempt, and at a 60 s budget that was worth three
edges. Raising the budget to 180 s buys the same three back in one pass and
more cheaply, which is why the rounds could go — but if 69q ever loses edges
to the clock again, this is the thing that was taken away.

## The edge cache

**The two edges at each end of a chain are routed again for every prefix. The
ones in between are remembered by the pair of options at their two ends**
(`CHAIN_FRESH_EDGES = 2`, user 2026-09-28), through `edgeMemo_`.

The pair of options is not the whole truth on this path — the prefix fence is
part of the question — so a remembered entry outlives the ground it was found
on, exactly as `edgeMemo_` already says of the greedy's entries. The ends of a
chain are where that costs too much to accept: an edge at a launcher leaves
the terminal on one heading and cannot yield, and the edges beside it want the
same ground.

**It is the difference between finishing and not.** Without it, one chain of
nine waypoints on 57q had routed **24 897 edges in 1 400 s** and was not done;
**14 400 of those were one edge**, asked for the same 256 pairs of options
over and over. With it, 45q fell from 67.0 s to 27.7 s and 57q finished at
all. The small chips reuse nothing, because a chain of five or six edges is
all ends and no middle.

## The other order, for the pair at either end

The prefix lays its edges front to back, so the edge before this one took the
room first and this one has to work around it. **At the ends of a chain that
order decides whether both fit at all** (user, 2026-09-28): an edge at a
launcher leaves the terminal on one heading and cannot yield, and the edge
beside it wants the same ground.

So when the step into the second edge of either end pair finds no way, it is
tried the other way round: **this edge first, then its predecessor again
against it.** If both come home, the pair is kept in that order. The
predecessor's new way hangs off *this* prefix's key, because the shorter
prefix is shared by every choice that extends it and only this choice needed
its predecessor moved.

**It costs the proof, and the search says so.** The price of the prefix
changes with the predecessor's way, which `ChainProblem::step` carries back as
`redone`; a prefix can then be *cheaper* than the prefix it extends, and that
is the one thing A\* rests on not happening. `ChainSolution::relaid` records
it and clears `optimal`, and the log then reads `best found, an end pair laid
the other way round` rather than `optimum`.

**What it buys, measured on 69q:** it fires 41 times, and chain 8 comes home
only through it. It does **not** rescue 69q's chain 2, which exhausts every
run of options in 52 s: there really is no sequence that joins that chain.
On 57q it fires on chain 4 and on the smaller chips a handful of times,
changing no chip's outcome.

## The time limit

**`SCPD_CHAIN_ASTAR_SECONDS` bounds one attempt at one chain, 10 s by
default.** When it runs out the search hands back **the cheapest complete run
of options it has reached** — every edge of that run drawn, and drawn against
the run itself — and says so: `ChainSolution::outOfTime` is set and the log
reads `best in the time given`. Zero is no limit.

**Ten seconds, because the answer arrives early and the rest is the proof.**
57q's chain 4 is the chain that always reaches the limit, and it comes home at
**220000 either way** — after 202 edge searches at 10 s, after 3209 at 180 s.
The extra 170 s buy no better run of options, only the certainty that none
exists, and 57q still draws all 66 edges at `stateFaults` 0.

**A wall-clock budget makes the answer machine-dependent, and that is
measured.** 17q run twice on pure defaults is identical chain for chain; 57q
run twice agrees on everything reported — 0 undrawn, angle 132 — but its
chain 4 does `56 prefixes / 209 steps` once and `55 / 204` the next time,
landing on 220000 both times. Only a chain that **reaches** the limit can
vary, and what varies first is the work, not the answer; but nothing
guarantees the answer. A run that has to be reproducible to the digit needs
`SCPD_CHAIN_ASTAR_SECONDS=0` and the time that costs.

**What it costs is 69q's chain 8**, which finds no complete run in 10 s where
180 s found one. That is the whole of the difference between the two settings
on the eight chips, and it is the reason to keep
`SCPD_CHAIN_ASTAR_SECONDS=30` in mind rather than treating 10 s as settled.

**It is spent per attempt, not per chain.** A chain whose plain options do not
reach opens its jogs and searches again, and the second attempt starts the
clock over, so such a chain can spend twice this.

**`-v 1` says when a run of options comes home.** Every time the search
completes a run whose every feedline edge is drawn, it prints the run and what
it costs:

```text
[Coupler Insertion]   chain 0: every feedline edge drawn on options
                      launcher 18 3 0 12 32 launcher -> cost 180000
[Coupler Insertion]   chain 0: every feedline edge drawn on options
                      launcher 18 3 0 12 41 launcher -> cost 160000
```

They do not arrive cheapest first: the end of a chain is reached in order of
the *bound* on what a run costs, and its last step is priced only once it is
reached. The cheapest is kept. **A chain that never prints this line is a
chain the search never joined**, which is the quickest way to read a failure.

**Two failures, and they are not the same failure.** A chain that comes back
unsolved reports either `no run of options joins the chain` — the search saw
every run there is, which is a verdict about the geometry — or `out of time
before any run of options joined the chain`, which is a verdict about the
clock. `ChainSolution::outOfTime` is what separates them, and it has to be its
own field: `optimal` will not do, because a reordered end pair clears that too
and a chain that had merely reordered then read as one that had run out of
time. Calling the two by one name hid a 69q chain that had spent 74 s against
a 60 s budget for a whole sweep.

## Each chain on its own chip

**`fenceCommittedEdges` closes no other chain's edges while a chain is
searched** — `SCPD_CHAIN_SOLO=0` is the way back.

The chains are settled one after another, so fencing them against each other
makes the order decide who gets the room: the first chain has the run of the
chip and the last has to work around everything. That is what strangled the
late chains, and the numbers are not marginal.

| 69q | fenced | on its own |
| --- | ---: | ---: |
| chains solved | 9 of 12 | **12 of 12** |
| chain 2 | no run joins it (52 searches) | **optimum, 15** |
| chain 5 | no run joins it (713 searches) | **optimum, 8** |
| chain 11 | no run joins it (974 searches) | **optimum, 11** |
| edges drawn | 76 of 81 | **78 of 81** |
| angle | 176 | **164** |
| `stateFaults` | 20 | **17** |
| the chain phase | 139 s | **26 s** |

It moves the conflict to the commit, and the commit turns out to settle it
better than the fence avoided it.

**On 17q it changes nothing at all** — the two arms are identical to the
digit, though the fence demonstrably differs (11 ways closed against 0). The
other chains' ways lie outside the 250-cell edge box there, so closing them
closes cells the box had closed already.

**What it costs is the resonators.** Without the fence the couplers pick other
places: 69q spans 5806 to 6130 units against 6008 to 6128, and nine rather
than six sit more than 100 off the figure. All stay inside the shortfall rule,
so the meander has more to add and nothing is beyond it.

## Keeping the ways

**The commit takes the ways the search found, as they are** —
`SCPD_CHAIN_KEEP_WAYS=0` restores the test.

It used to rebuild the corridor per edge and ask whether the way still lay in
it (`edgeWayStillOpen`), routing again what did not. That test is now dropped.
The endpoints are still checked, which is not a rule but an identity: a way
that does not run between the ports the couplers ended on is a different edge,
not a stale one.

**What this is worth, and what it is not.** On 69q the commit went from
*72 kept, 6 routed again, 3 with no way* to **81 kept, 0 routed again, 0 with
no way**. The `==>` line therefore reads 0 undrawn.

**`stateFaults` did not move: 17 either way.** The same seventeen edges lie so
that they do not survive the chip they lie on. The three that used to be lost
were not rescued, they stopped being counted — the difference between 78 and
81 is the difference between *three edges missing* and *three edges lying
across others*. **Under this switch the figure to read is `stateFaults`, not
the undrawn count.** Two of those three had a **buried source port**: another
chain's feedline lies across the coupler's own feedline port, which no routing
fixes and only moving the coupler would.

The reasoning for taking them anyway is that the first and last feedline wires
are drawn again in the routing phase that follows (user, 2026-09-29), which
`stop_after = "couplers"` never reaches, so none of these measurements sees it.

**This is the rule trap 2 of the coupler handover describes, walked into on
purpose.** Keep that paragraph in view before turning the switch back on.

## The shortfall rule

**An option is refused when what is left of the resonator falls more than
three tenths below `target_resonator_length`** — `SCPD_COUPLER_MAX_SHORTFALL`
sets the share, `couplerMaxShortfall()` holds the default of 0.30.

**It was a tenth first, and a tenth was measurably too tight.** It caught what
it was for, but it also took 15 % of every coupler's options away, left no
coupler on 69q with all 48, cost **17q two feedline edges and 2.1 s → 26.8 s**,
and took **4q's angle from 8 to 32**. At three tenths both are back where they
were — 4q at 8, 17q at 0 undrawn and 2.0 s — and the case the rule exists for
is still refused by a wide margin: 1455 units against a floor of 4200.

`couplerPlace` offers every cell of the way ordered by how near it leaves the
design figure, and each orientation walks that list until one place *fits*. A
place that fails a legality rule is skipped and the walk carries on **away
from the figure**. Nothing stopped it: only an overshoot was refused. On 69q
that left a resonator of **1455 units against 6000**, which no later phase
recovers — the meander lengthens a way, but not by four times.

**The figure it holds against is the spliced length**: the lead, what is left
to the qubit, and the run to the port. The lead is absolute, about 290 layout
units, which is a twentieth of a 6000-unit target and an eighth of a
2500-unit one; measured without it the same share would refuse on the small
chips the very place `couplerPlace` aims at.

**It works, and it is not free.**

| 69q | without | with |
| --- | --- | --- |
| resonator span | **1231** – 6131 | **6008** – 6128 |
| below 5400 | 3 | **0** |
| more than 100 off | 14 | **6** |
| options over all 69 couplers | 3196 | **2726** (−15 %) |
| couplers with all 48 options | 40 | **0** |

| 17q | without | with |
| --- | ---: | ---: |
| edges not drawn | **0** | **2** |
| angle | **47** | 66 |
| insertion | **2.1 s** | **26.8 s** |

So it is a clear gain on 69q and a clear loss on 17q, and 4q's angle went from
8 to 32 under it. **No coupler has yet been left with no option at all.** The
share was not swept; 15 % and 20 % are one run each.

## Where the pieces are

| Piece | What it does |
| --- | --- |
| `routing::ChainProblem` | `width`, `bound(layer, from, to)`, `step(prefix, to, redone)`, `budget`, `found` |
| `routing::ChainSolution` | `chosen`, `cost`, `expansions`, `routed`, `reached`, `solved`, `optimal`, `outOfTime`, `relaid` |
| `routing::solveChainAStar` | the search: the arena, the heuristic, the queue, the budget |
| `Driver::optimizeChainsPrefix` | **one pass over the chains, no rounds** |
| `Driver::solveChainAStar` | one chain: the bounds built, `step` supplied, the end-pair reorder, the winner's ways read back |
| `Driver::chainAStar` | the `SCPD_CHAIN_ASTAR` gate, on by default |
| `Driver::chainSolo` | `SCPD_CHAIN_SOLO`: no other chain is fenced while one is searched |
| `Driver::chainKeepWays` | `SCPD_CHAIN_KEEP_WAYS`: the commit takes the search's ways as they are |
| `Driver::soloChain_` | the chain being searched on its own, never set around the commit |
| `couplerMaxShortfall` | `SCPD_COUPLER_MAX_SHORTFALL`, three tenths |
| `Driver::chainAStarBudget` | `SCPD_CHAIN_ASTAR_SECONDS`, 10 s, per attempt |
| `Driver::CHAIN_FRESH_EDGES` | how many edges at each end are never remembered |
| `Driver::prefixFence_` | the ways the caller has already laid, which `corridorOfEdge` closes |
| `Driver::openOptionsOf` | which options a waypoint offers, factored out of `solveChain` |
| `Driver::portOfOption` | the port an edge leaves a waypoint by or arrives at it by |

- **`prefixFence_`** replaces the single `aheadFence_` the second order added:
  a list of ways that `corridorOfEdge` closes with
  `alongDisc(..., stencilFor(tuning_.clearance), ...)` immediately after
  `fenceCommittedEdges`, exactly where `aheadFence_` was closed. The
  second-order trellis puts one way in it and the prefix search every edge of
  the prefix.
- **`edgeCost` takes the fence and a `remember` flag**, and `remember` is
  false for the four edges at the ends of a chain and for both routes of a
  reorder. `edgeMemoKey` and its `ahead` field are otherwise untouched, so the
  trellis reads and writes what it always did.
- **The winner's ways** come out of a `std::map` from prefix to
  `{way, before}`, filled as `step` prices each one. `std::map` because the
  key is a vector and because its values never move — the fence holds
  pointers into it.

## The tests

`test/routing/test_chain_search.cpp`, sixteen of them, on the `Table` fixture
of `test_chain_trellis.cpp` with one thing added: a `conflict` that reads the
whole prefix. With no conflict the two searches solve the same problem and
must agree; with one they do not, and that is the reason this search exists.

Two are that reason written down as something that runs.

- **`APrefixThatBlocksItselfCostsAnother`.** The table discounts exactly the
  three steps of one run of choices and charges ten for every other step, so
  that run costs 3 and no other costs less than 21 — and it is the one run the
  conflict forbids. The prefix search answers 21. The trellis answers 3, names
  that run, and the run it names cannot be built at all.
- **`BeatsTheTrellisOnRandomChainsThatBlockThemselves`.** Three hundred small
  random chains with a random rule that makes pairs of choices refuse to stand
  together, each brute-forced over every assignment. The prefix search returns
  the brute-force optimum on every one. The trellis, priced honestly, is above
  it on more than a tenth of the chains it solves at all.

The rest hold the contract: an exact bound prices exactly `L-1` steps, a zero
bound gives the same price and strictly more steps, an unbuildable step is
routed round, a layer nothing reaches is named by `reached`, a chain of one
layer prices nothing, ties break the same way twice, a `redone` price is
counted and clears `optimal`, a budget of a nanosecond claims nothing, no
budget means no limit, every complete run is reported and the cheapest kept,
and on chains where nothing blocks the two searches agree.

## How to run it

```bash
cmake --build --preset release
./build/release/test/routing/mqt-scpd-routing-test \
    --gtest_filter='ChainSearch.*:ChainTrellis.*:AnalyticDubins.*'
uv pip install --python .venv/bin/python --no-build-isolation --no-deps \
    --reinstall-package mqt-scpd -e .

cp benchmarks/17q/config.toml artifacts/17q-dp/config.toml     # ← not optional
.venv/bin/mqt-scpd plan -c benchmarks/17q/config.toml \
    -o artifacts/17q-dp --stage final -v 1
# no time limit, for a chip small enough to prove every chain
SCPD_CHAIN_ASTAR_SECONDS=0 .venv/bin/mqt-scpd plan \
    -c benchmarks/17q/config.toml -o artifacts/17q-dp --stage final -v 1
# 60 s instead of 180 s: 57q in 86 s rather than 201 s, at the same 66 edges
SCPD_CHAIN_ASTAR_SECONDS=60 .venv/bin/mqt-scpd plan \
    -c benchmarks/57q/config.toml -o artifacts/57q-dp --stage final -v 1
# the way back to the trellis, and from there to the greedy
SCPD_CHAIN_ASTAR=0 .venv/bin/mqt-scpd plan -c benchmarks/17q/config.toml \
    -o artifacts/17q-dp --stage final -v 1
# the three switches this session added, each back to what it replaced
SCPD_CHAIN_SOLO=0 ...            # fence every other chain again
SCPD_CHAIN_KEEP_WAYS=0 ...       # test every kept way, route again what fails
SCPD_COUPLER_MAX_SHORTFALL=1 ... # accept any remainder, however short
```

Report per chip: **feedline edges NOT drawn, angle cost, insertion seconds,
and `steps routed` per chain**, and for every chain whether it read `optimum`,
`best found, an end pair laid the other way round`, or `best in the time
given`.

## What is open

1. **69q draws every edge, and seventeen of them do not survive the chip they
   lie on.** That is the open number now, and *Keeping the ways* says why the
   undrawn count no longer reports it. Two of the worst cases are a buried
   feedline port, which only moving the coupler fixes.
2. **The repair is switched off and is exactly what a buried port needs.**
   `repair_trials = 0` in all eight configs, for measuring. The repair turns a
   coupler near a fail to another of its options, redraws what that unsettles
   and keeps the turn when it leaves strictly fewer fails. It has never been
   run against the prefix search: one run, no code.
3. **The shortfall rule costs 17q two edges and 4q three quarters of its angle
   quality**, and nothing has been swept. See *The shortfall rule*.
4. **Merging, measured against the unmerged answer.** The natural first rule
   is the second-order equivalence: prefixes that agree on `(layer, choice
   here, choice before)` are interchangeable for the future, because an edge
   sees only a 250-cell box (`EDGE_BOX_MARGIN`) and at a shared coupler it is
   the immediately preceding edge that reaches into it. That collapses the
   node count to `n²` a layer while the prices still carry the true prefix
   fence. Doing it *after* the unmerged version exists is the whole point: the
   approximation error can then be measured against an exact reference
   instead of assumed. The exact rule, if the second order loses something, is
   geometric: key a node by the prefix's fenced cells that fall inside the
   union of the *remaining* edges' boxes.
5. **`CHAIN_FRESH_EDGES = 2` was not swept.** On 57q's chain 4 the two dearest
   edges by far are the **last two** — 14 400 and 7 200 routes of 24 897 — and
   those are precisely the ones the rule never remembers. Raising the reuse to
   them would collapse that chain; whether it costs an edge is the measurement
   nobody has made.
6. **A chain is still fenced by its own edges as the pass before left them.**
   `looseChain_` is only set under `SCPD_CHAIN_DP=2`, so
   `fenceCommittedEdges` closes this chain's other edges on top of the prefix
   fence. With one pass those are empty for every chain, so it costs nothing
   today — but it would bite the moment a second pass came back.
7. **The chains are independent and are not run in parallel.** With the rounds
   gone this is the largest runtime lever left. It needs one router context
   per thread, since `router_`, `corridor_`, `box_`, `proximity_`, `stencils_`
   and `frame_` are all shared `Driver` state.
8. **The cost still sees only the feedline.** Length, length difference and
   `100 × guarded` are commented out, so the two resonator ports of one
   coupler are an exact tie.

## Traps that cost time

0. **A budget in seconds is not reproducible.** Two runs of 57q on pure
   defaults agree on every reported figure and differ in chain 4's work —
   56 prefixes against 55. The answer held; it need not. Any table that has
   to be exact wants `SCPD_CHAIN_ASTAR_SECONDS=0`.
1. **Measure sequentially, and watch what else the machine is doing.**
   `artifacts/` grows to 15 GB and every run rewrites the GDS and SVG in it,
   which sets Spotlight indexing it again. One 69q run came back at **5553 s
   against 922 s** for the identical result. Check with
   `ps -A -o %cpu,comm -r | awk '$2 ~ /mds_stores|mediaanalysisd/ {s+=$1} END {print s}'`,
   and measure CPU time beside wall time — the two agreeing is what says a row
   is clean.
2. **Use `-v 1`.** `-v 2` alone made a 69q run take four times as long.
3. **`plan --stage final` reads the earlier stages' artifacts out of the run
   directory**, so its numbers are not a full pipeline run's. Compare only
   like with like, and copy the config over every time.
4. **The pipeline tests are not a quick check.** `test/pipeline/Benchmarks.hpp`
   reads `refinement_rounds` from the config but **not** `repair_trials` or
   `stop_after`, so `EveryChip/Final.*` runs the full stage at the schema
   default `repair_trials = 100` — over two hours, unfinished.
5. **A chain that says it cannot be joined may only have run out of time.**
   That message was ambiguous for a whole sweep and hid a 69q chain that had
   spent 74 s against a 60 s budget. It now names which of the two it was.
6. **An editor with `FinalRouter.cpp` open can write a stale buffer back over
   your change.** If a switch does not take, check the source before the
   logic.
7. **The configs are in a measuring state** and must go back before a commit:
   `repair_trials = 0` and `stop_after = "couplers"` in all eight.
   `test_the_final_routing_carries_a_snapshot_of_every_phase` fails while they
   are in.

## The pad filter (uncommitted, and keep it)

In `Driver::makeOption`, right after the pad cell set is built: an option is
refused when **what is left of the resonator runs through its own pad**.
Nothing refused it before — `free()` prices another wire's copper but exempts
the resonator's own (`owner != resonator.key`), and the self-intersection test
asks whether the path meets *itself*, not whether it meets the body.

It does not usually reject an option outright; `couplerPlace` walks on to the
next place along the resonator, so the coupler moves rather than disappearing.
Measured against the trellis: six chips unchanged, **57q 5 undrawn → 2 and
2641 s → 357 s**, 69q 10 → 11 undrawn and 366 s → 1023 s. An option whose pad
is crossed by its own resonator is not buildable, whatever a metric says
somewhere.
