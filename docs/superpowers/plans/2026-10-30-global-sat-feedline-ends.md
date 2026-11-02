# Global SAT: conflicts between feedline ends — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The global SAT search finds the two competing feedline ends of every bottleneck over its capacity and tries other options for exactly those ends, alone and together, so that 33q_dense with the snap options ends SAT.

**Architecture:** Two pure functions in `CombinationOrder` (`conflictPairs`, `pairRoundPicks`) decide which chain ends compete and in which order their states are tried. `FinalRouter` collects states per chain end by re-searching the end segment with the end coupler pinned to each open option (`Segment::pinned`), and runs pair rounds before the conflict-directed stage 2. `CouplerSession::endStates` exposes the end-state collection for tests and the debug pages.

**Tech Stack:** C++20, GoogleTest, FlatBuffers (config schema), Python config loader, the `_cdt-bench` bench.

**Spec:** `docs/superpowers/specs/2026-10-30-global-sat-feedline-ends-design.md`

## Global Constraints

- New config field `global_sat_end_options: uint32 = 16` at the end of `FinalParams` in `schemas/config.fbs`; 0 switches the pair rounds off (the search exactly as today).
- A chain's state 0 stays the committed state; the order of all trials depends on nothing but the chip.
- `global_sat_tries` bounds pair rounds and stage 2 together.
- Pairs are stored lower chain first, kept once, in the order of their first gate.
- Log lines, verbatim shapes: `global SAT: <gate>: <end|start> of chain <A> (waypoint <w>) ↔ <end|start> of chain <B> (waypoint <w>)`; `global SAT: chain <c> <end|start>: <n> options tried, <k> locally SAT (<s> with the other chains standing)`; `global SAT: pair round <r>: chains <A> and <B>; <n> combinations`.
- Build: tests in `build/release` (`cmake --build build/release --target <test>`); the Python module: `cmake --build build/cp311-abi3-macosx_15_0_arm64/Release`, then `rm` the venv's `mqt/scpd/pyscpd.abi3.so` and `cp` `bindings/pyscpd.abi3.so` there (never overwrite in place). Schema: `.venv/bin/nox -s schemas`.
- Commit only when the user asks (repo rule); the commit steps below are the points where a commit belongs.

## Review Focus

- A gate whose ends map to no chain at all (artwork · plain wire port): no pair, the old naming handles it — test in Task 1.
- A chain with a single coupler (start == end): it has one end; both sides map to the same waypoint, and the pair must not list the chain twice — test in Task 1.
- An end coupler whose only open option is the one it stands on: no state, no crash, the log says `0 options tried` — test in Task 3.
- Two bottlenecks over in one check that name the same pair: one pair, one round, its states collected once — test `APairComesOnceInTheOrderOfItsFirstGate` in Task 1 and the per-end cache in Task 4.
- `global_sat_end_options = 0`: the search runs exactly as before (same combination lines in the existing global SAT tests) — test in Task 4.

---

### Task 1: `conflictPairs`

**Files:**
- Modify: `include/mqt-scpd/pipeline/CombinationOrder.hpp`, `src/pipeline/CombinationOrder.cpp`
- Test: `test/pipeline/test_combination_order.cpp`

**Interfaces:**
- Produces:
  ```cpp
  enum class ChainSide : std::uint8_t { Start, End };
  struct ChainEnd { std::uint32_t chain = 0; ChainSide side = ChainSide::Start;
                    auto operator<=>(const ChainEnd&) const = default; };
  struct ConflictPair { ChainEnd first; ChainEnd second; std::string gate; };  // first.chain < second.chain
  struct OverGate { std::string name; std::vector<std::string> ends; std::array<double, 2> middle{}; };
  struct FeedlineMap {
    struct Feedline { std::uint32_t chain = 0; std::size_t from = 0; std::size_t to = 0; bool terminal = false; };
    std::map<std::string, Feedline> feedlines;                                   // "f33"
    std::map<std::string, std::pair<std::uint32_t, std::size_t>> resonators;     // resonator name → chain, waypoint
    std::vector<std::size_t> size;                                               // per chain, its waypoints
    std::vector<std::array<std::size_t, 2>> ends;                                // per chain, waypoint of its start and end coupler
    std::vector<std::array<std::array<double, 2>, 2>> centres;                   // per chain, pad centre of its start and end coupler
  };
  [[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::vector<ConflictPair>
  conflictPairs(const std::vector<OverGate>& gates, const FeedlineMap& chip);
  ```
- Mapping of one wall name (spec §1): `fN at its coupler|launcher, …` with `terminal` → `ChainSide::Start` when `from == 0`, else `End`; `fN` (inner) → `Start` when `from < size / 2`, else `End`; `lead of R`, `pad of R`, `R at its source|target`, R in `resonators` → the side whose end waypoint is nearer R's waypoint (ties: `Start`); anything else → none. Partner rule: when one wall maps, or both map to one chain, the partner is the end of another chain whose centre is nearest `middle` (ties: lower chain, then `Start`).

- [ ] **Step 1: Write the failing tests** — one chip for all: chain 0 and chain 1 of 4 waypoints each (0 and 3 launchers); `f0` 0→1 terminal, `f1` 1→2, `f2` 2→3 terminal on chain 0; `f3`, `f4`, `f5` the same on chain 1; `ends = {{1, 2}, {1, 2}}`; centres chain 0 start (0, 0), end (10, 0), chain 1 start (12, 0), end (30, 0); `resonators = {{"R5", {1, 2}}}`.
  - `ATerminalStubAgainstAPortTakesTheNearestOtherEnd`: gate `{"g380", {"f2 at its coupler, right", "7 at its target, right"}, {11, 1}}` → one pair `{chain 0 End, chain 1 Start, "g380"}`.
  - `TwoFeedlineWallsMakeThePairDirectly`: ends `{"f2 at its coupler, left", "f3 at its coupler, right"}` → `{0 End, 1 Start}`.
  - `AnInnerEdgeMapsToItsNearerEnd`: ends `{"f1", "f4"}` → `{0 Start, 1 End}`.
  - `ALeadMapsThroughItsResonator`: ends `{"f2", "lead of R5, left"}` → `{0 End, 1 End}`.
  - `APairComesOnceInTheOrderOfItsFirstGate`: two gates `g1` (`f1`,`f4`) then `g2` (`f2`, `f3`), then `g3` again `f1`,`f4` → pairs `[{0 Start,1 End,"g1"}, {0 End,1 Start,"g2"}]`.
  - `AGateOfNoFeedlineMakesNoPair`: ends `{"artwork near Qb1.port0", "7 at its target, left"}` → empty.
  - `AChainOfOneCouplerIsNotItsOwnPartner`: chain 2 with `ends = {1, 1}`, both walls on chain 2 → its partner is another chain, never chain 2.

- [ ] **Step 2: Run them to see them fail**
  Run: `cmake --build build/release --target mqt-scpd-pipeline-test && build/release/test/pipeline/mqt-scpd-pipeline-test --gtest_filter='CombinationOrder.*'`
  Expected: compile error, `conflictPairs` not declared.

- [ ] **Step 3: Implement `conflictPairs` in `src/pipeline/CombinationOrder.cpp`** — parse names with `std::regex` (`^(f\d+)( at its (coupler|launcher))?` and `^(lead|pad) of (\S+?),?` / `^(\S+) at its (source|target)`), the rules above.

- [ ] **Step 4: Run the tests to see them pass** (same command). Expected: all `CombinationOrder.*` PASS.

- [ ] **Step 5: Commit** `✨ Name the competing feedline ends of a bottleneck over (conflictPairs)`

### Task 2: `pairRoundPicks`

**Files:** same as Task 1.

**Interfaces:**
- Consumes: `Combination` (existing), `ConflictPair` (Task 1).
- Produces:
  ```cpp
  /// The combinations of one pair round on top of `base`: chain `a` alone on each
  /// of `statesA` (state numbers), then `b` alone on each of `statesB`, then both,
  /// by the sum of the two ranks, then a's rank, then b's; `tried` left out.
  [[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::vector<Combination>
  pairRoundPicks(const Combination& base, std::uint32_t a, const std::vector<std::size_t>& statesA,
                 std::uint32_t b, const std::vector<std::size_t>& statesB,
                 const std::set<Combination>& tried);
  ```
  The rank of a state is its index in its list (0 first).

- [ ] **Step 1: Write the failing tests**
  - `APairRoundTriesEachSideAloneThenBoth`: base `{0, 0, 0}`, `a = 0` states `{3, 4}`, `b = 2` states `{5}` → `{{3,0,0}, {4,0,0}, {0,0,5}, {3,0,5}, {4,0,5}}`.
  - `APairRoundOrdersTheJointPicksByRankSum`: states `{1, 2}` and `{1, 2}` → joint part `{{1,0,1}, {1,0,2}, {2,0,1}, {2,0,2}}` (rank sums 0, 1, 1, 2; tie by a's rank).
  - `APairRoundLeavesOutWhatWasTried`: `tried = {{3,0,0}}` → `{3,0,0}` absent.
  - `APairRoundKeepsWhatTheBaseMoves`: base `{0, 7, 0}` → every pick has `[1] == 7`.

- [ ] **Step 2: Run them to see them fail** (command of Task 1). Expected: `pairRoundPicks` not declared.
- [ ] **Step 3: Implement `pairRoundPicks`.**
- [ ] **Step 4: Run the tests to see them pass.**
- [ ] **Step 5: Commit** `✨ Order the combinations of a pair round (pairRoundPicks)`

### Task 3: States at a chain end (pin, collection, `CouplerSession::endStates`, config)

**Files:**
- Modify: `schemas/config.fbs` (field at the end of `FinalParams`), regenerated `include/mqt-scpd/flatbuffers/config.hpp` and `python/mqt/scpd/flatbuffers/config/FinalParams.py` (`.venv/bin/nox -s schemas`), `python/mqt/scpd/config.py` (default + `take`), `docs/configuration.md` (row after `coupler_snap_r`), `test/python/unit/test_config.py` (`assert final.globalSatEndOptions == 16`)
- Modify: `src/pipeline/FinalRouter.cpp` — `Segment` (`pinned`), `researchSegment` (the `open[k]` loop), `Tuning` (`globalSatEndOptions`, read in `tuningOf`), new member `collectEndStates`
- Modify: `include/mqt-scpd/pipeline/CouplerSession.hpp`, its source — `endStates`
- Test: `test/pipeline/test_final_router.cpp`

**Interfaces:**
- Consumes: `ChainEnd`, `ChainSide` (Task 1).
- Produces:
  ```cpp
  // FinalRouter.cpp
  struct Segment { …; std::map<std::size_t, std::size_t> pinned; };   // waypoint → option
  struct EndState { std::size_t option = 0; std::vector<std::size_t> options; bool standing = false; };
  struct EndStates { std::size_t waypoint = 0; std::vector<std::size_t> tried; std::vector<EndState> states; };
  EndStates collectEndStates(std::vector<Wire>& wires, ChainEnd end, std::array<double, 2> gateMiddle);
  // CouplerSession.hpp — JSON of collectEndStates for chain `chain`, its start (`end == false`)
  // or end, the gate middle taken as the end coupler's own pad centre:
  // {"chain", "side": "start"|"end", "waypoint", "tried": [...], "states": [{"option", "options", "standing"}]}
  [[nodiscard]] std::string endStates(std::uint32_t chain, bool end);
  ```
- `collectEndStates` (spec §2): candidates = `openOptionsOf(chain, waypoint)` without the option it stands on, farthest pad centre from `gateMiddle` first, ties by option index, at most `tuning_.globalSatEndOptions`; per candidate `researchSegment` on the end segment (`lo..hi` as `collectChainStates` picks the end span, `span = 3`) with `pinned = {{waypoint, option}}`, `trialsLeft = 1`, collecting the first run through `segmentCollect_`; `checkChainLocally` on it; `standing` from `judged.standing()`; a state kept when standing or alone; states ordered standing first, then the candidates' order. Logs the per-end line of Global Constraints. The candidates come from `openOptionsOf(chain, waypoint, &standing)` with the same guard `researchSegment` builds (`standingCellsBut(wires, chain, lo, hi)` of the end segment), so every pin is an open option; `researchSegment` still drops a pin it does not find open, with a `tell` line.

- [ ] **Step 1: Write the failing tests** (9q, `nineQubit()`, `plan`, a `CouplerSession`)
  - `CouplerSession.AnEndStateStandsOnItsPinnedOption`: `endStates(0, false)` and `endStates(0, true)` parse; for every state `options[waypoint] == option`; every `option` is in `tried`; `tried.size() <= 16`; the option the end coupler stood on is not in `tried`.
  - `CouplerSession.EndStatesAreCappedByTheSetting`: with `finalParamsOf(chip).global_sat_end_options = 2`, `tried.size() <= 2`.
  - `CouplerSession.AnEndWithNoOtherOptionHasNoState`: with `finalParamsOf(chip).coupler_offsets = "0"` and `jog_options = ""`, an end whose `tried` is empty has no `states` (the case the Review Focus names; assert `states.empty()` wherever `tried.empty()`).
  - Python: `test_config.py` asserts the default 16.

- [ ] **Step 2: Run them to see them fail**
  Run: `cmake --build build/release --target mqt-scpd-pipeline-test && build/release/test/pipeline/mqt-scpd-pipeline-test --gtest_filter='CouplerSession.*EndState*:CouplerSession.EndStates*'` and `.venv/bin/python -m pytest test/python/unit/test_config.py -q`
  Expected: compile error (`endStates`) and an AttributeError in the Python test.

- [ ] **Step 3: Add the config field** (schema, `nox -s schemas`, `config.py`, docs row: `` | `global_sat_end_options` | `16` | How many options of each competing feedline end the pair rounds of the global SAT search try; 0 for the search without pair rounds | ``).
- [ ] **Step 4: Implement `Segment::pinned` in `researchSegment`**, `collectEndStates`, `CouplerSession::endStates`.
- [ ] **Step 5: Run the tests to see them pass** (both commands). Expected: PASS, `51 passed` or more in Python.
- [ ] **Step 6: Commit** `✨ Collect the states of a feedline end, its end coupler pinned to each option`

### Task 4: Pair rounds in `searchGlobalSat`

**Files:**
- Modify: `src/pipeline/FinalRouter.cpp` — `wholeChipLost` (also returns `std::vector<OverGate>`), a builder `feedlineMap()` for `FeedlineMap`, `searchGlobalSat` (pair rounds before stage 2, replacing stage 1 when `globalSatEndOptions > 0`), `FailedCombination` gets the over gates of its check
- Modify: `include/mqt-scpd/pipeline/CombinationOrder.hpp` — `FailedCombination::gates` (`std::vector<OverGate>`)
- Test: `test/pipeline/test_final_router.cpp`

**Interfaces:**
- Consumes: `conflictPairs`, `pairRoundPicks` (Tasks 1–2), `collectEndStates` (Task 3).
- Produces: `std::vector<std::string> wholeChipLost(const std::vector<Wire>& wires, std::set<std::string>* touching = nullptr, std::vector<std::array<double, 2>>* overGates = nullptr, std::vector<OverGate>* over = nullptr)` — an `OverGate` per bottleneck over: `name`, its `SlotGraph::Edge::ends`, `middle = (from + to) / 2`; `FeedlineMap feedlineMap() const`.
- Pair rounds (spec §3): round 1 on the commit's gates; per pair in order, the end states of both ends (collected once per end and cached for the search), their state numbers appended to `chainStates_` of each chain, `pairRoundPicks(base, …)` tried with `tryAll`; the next round's base is the failed combination with the fewest against (earliest on a tie) whose `conflictPairs` give a pair with an end it does not move yet; rounds end when there is none; then stage 2 as today with the checks left.

- [ ] **Step 1: Write the failing tests**
  - `FinalRouter.ThePairRoundsAreOffAtZero`: 17q with `chain_rule_refusals = 1` (the setting of `RecordsEveryChainStateTheGlobalSatSearchChecks`) and `global_sat_end_options = 0`: no line of `globalSatOf(...).lines` contains `pair round` or ` ↔ `.
  - `FinalRouter.ThePairRoundsNameTheirPair`: the same chip with the default: when its lines contain `the chip is UNSAT` and a bottleneck over, at least one line matches `global SAT: g\d+: (start|end) of chain \d+ \(waypoint \d+\) ↔ (start|end) of chain \d+ \(waypoint \d+\)`, and every `pair round` line comes before the first `stage 2 round` line (GTEST_SKIP when the commit is SAT).
- [ ] **Step 2: Run them to see the second fail** — `build/release/test/pipeline/mqt-scpd-pipeline-test --gtest_filter='FinalRouter.ThePairRounds*'`. Expected: `ThePairRoundsAreOffAtZero` PASS (the config field exists since Task 3), `ThePairRoundsNameTheirPair` FAIL (no such line) or SKIP; if it skips, note it and rely on Task 5's 33q_dense bench for the pair rounds.
- [ ] **Step 3: Implement** `OverGate` collection in `wholeChipLost`, `feedlineMap`, the pair rounds and their log lines.
- [ ] **Step 4: Run the global SAT tests** — `--gtest_filter='*GlobalSat*:CombinationOrder.*:CouplerSession.*EndState*'`. Expected: PASS (pre-existing failures named in Task 5 aside).
- [ ] **Step 5: Commit** `✨ Try the competing feedline ends of a conflict in pair rounds of the global SAT search`

### Task 5: Verify on the chips

**Files:** none in the repo (bench runs under `planar-superconducting-pd/runs/_cdt-bench/`).

- [ ] **Step 1: Rebuild and install the Python module** (Global Constraints).
- [ ] **Step 2: Run the suites** — `build/release/test/grid/mqt-scpd-grid-test`, `build/release/test/routing/mqt-scpd-routing-test`, `build/release/test/pipeline/mqt-scpd-pipeline-test`, `.venv/bin/python -m pytest test/python -q`. Expected: green except the four pipeline tests red before this work (`FinalRouter.ChecksNoStepAgainOnAPairAnEndBlacklisted`, `CouplerSession.CopiesThePlainOptionsInPlaceOfTheJogs`, `CouplerSession.CentresTheExtensionOptionsOnTheResonator`, `CouplerSession.RecordsEveryChainStateTheGlobalSatSearchChecks`); report any other by name.
- [ ] **Step 3: Snap chips** — `python3 bench.py run --label snap-pairs --chips 4q,9q,17q,21q,33q,33q_dense --set coupler_snap_pitch=50.0 --set router_grid_snap=50.0 --set coupler_merge=false --set "coupler_offsets='0,1,2,6,7'" --set coupler_grid_options=0 --set feedline_lattice_edges=0`. Expected: 33q_dense `global SAT: SAT after …` and `CAPACITY GRAPH SAT — by the crossing slots`; the other five as in `snap-contact`.
- [ ] **Step 4: Standard chips** — `python3 bench.py run --label pairs --chips 4q,9q,17q,21q,33q,45q,33q_dense,45q_dense`. Expected: verdicts, edges and angle costs as in `contact`; for 45q_dense the pair lines and the global SAT seconds reported.
- [ ] **Step 5: Report** the table (chip, verdict search/report, edges, angle, global SAT seconds, combinations) against `snap-contact` and `contact`.
