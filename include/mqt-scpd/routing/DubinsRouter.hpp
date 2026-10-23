/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#pragma once

#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/routing/AnalyticDubins.hpp"
#include "mqt-scpd/routing/BucketQueue.hpp"
#include "mqt-scpd/routing/CrossingConstraints.hpp"
#include "mqt-scpd/routing/Heading.hpp"
#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/Primitives.hpp"
#include "mqt-scpd/routing/SearchScratch.hpp"
#include "mqt-scpd/routing/SelfIntersection.hpp"
#include "mqt-scpd/routing/mqt_scpd_routing_export.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

namespace mqt::scpd::routing {

/// The estimate the search steers by.
enum class Heuristic : uint8_t {
  /// The true distance to the target over the free cells of the corridor,
  /// from one backward Dijkstra per search. It follows the corridor, so a
  /// search in a winding corridor does not flood it, and it proves cells
  /// that cannot reach the target unreachable before the search expands
  /// them. The default.
  DistanceField,
  /// The octile distance to the target. What the coupler insertion is
  /// calibrated on.
  Octile,
};

/// The curvature-constrained A* of the final routing: a search over cells
/// and eight-way headings whose moves are the primitives of one bend radius.
///
/// A router reads shared grids and writes only its own scratch, so one
/// router per thread over the same grids is the threading model. The
/// obstacle mask and the corridor are bit grids attached by pointer; the
/// static proximity penalty is owned; the wire proximity penalty is a view.
/// The corridor and the static proximity are packed into one working byte
/// per cell, so the inner loop touches one cache line per swept cell.
///
/// A router and its scratch are one per-thread context: the scratch belongs
/// to the caller and outlives the router, and nothing else a router reads is
/// copied per thread. The primitives are shared by pointer and never bound by
/// reference, so a router outlives whatever built them; the prototype bound
/// them to a caller's stack local and worked around the dangling result.
class MQT_SCPD_ROUTING_EXPORT DubinsRouter {
public:
  /// A router over the grid of the scratch.
  ///
  /// @throws std::invalid_argument when the bend radius of the parameters is
  /// not the one the primitives were built with, or when the primitives have
  /// more moves per heading or longer moves than the search tables hold.
  DubinsRouter(std::shared_ptr<const MovePrimitives> primitives,
               SearchScratch& scratch, SearchParams params = {});

  [[nodiscard]] uint32_t width() const { return width_; }
  [[nodiscard]] uint32_t height() const { return height_; }
  [[nodiscard]] std::size_t cells() const {
    return static_cast<std::size_t>(width_) * height_;
  }
  [[nodiscard]] const MovePrimitives& primitives() const {
    return *primitives_;
  }
  [[nodiscard]] const SearchParams& params() const { return params_; }

  /// Change the search parameters. Only a changed bend penalty rebuilds the
  /// search tables.
  ///
  /// @throws std::invalid_argument when the bend radius is not the one the
  /// primitives were built with.
  void setParams(const SearchParams& params);
  void setHeuristic(const Heuristic heuristic) { heuristic_ = heuristic; }

  /// Bound the eighth-turns a way may make: a move that would take a way
  /// past `turns` is not generated, so a search under the bound explores
  /// only the ways within it and finds the cheapest of those or nothing.
  /// `-1` is no bound. `lastSearchCutOff` says whether the last search
  /// dropped a move for it — a search that found nothing under a bound and
  /// dropped nothing would have found nothing without it either.
  void setMaxTurns(const int16_t turns) { maxTurns_ = turns; }
  [[nodiscard]] bool lastSearchCutOff() const { return cutOff_; }

  /// The turn bounds under which the last route() would have run the same.
  ///
  /// The search compares a number with the turn bound at two places: the
  /// turns of a way after a move (`setMaxTurns`) and, for a move that
  /// improves the state it reaches, those turns plus the turns that state
  /// still owes (`setTurnLowerBound`). Nothing else the search does reads
  /// the bound. Take another bound `b`. If every comparison comes out the
  /// same under `b`, the search pushes the same states in the same order,
  /// by induction over its moves, and returns the same way at the same cost.
  ///
  /// - `kept` is the largest number that passed for an improving move, and
  ///   `below` the least number that failed for any move. Under every bound
  ///   `b` with `kept <= b < below` the search returns the same way at the
  ///   same cost. A move that improves nothing is pushed under no bound, so
  ///   the outcome of its comparison only sets `lastSearchCutOff`; a pass
  ///   of such a move does not count in `kept`.
  /// - `passed` is the largest number that passed for any move. Under every
  ///   bound with `passed <= b < below`, `lastSearchCutOff` is the same too.
  ///
  /// No bound (`-1`) counts as a bound above every number: every comparison
  /// passes, and `below` stays at its maximum. Valid only after a route()
  /// that expanded a state (`lastSearchExpansions` above zero).
  struct BoundWindow {
    uint32_t kept = 0;
    uint32_t passed = 0;
    uint32_t below = UINT32_MAX;
  };
  [[nodiscard]] BoundWindow lastBoundWindow() const { return boundWindow_; }

  /// Add the unavoidable turning toward the target heading to the heuristic.
  ///
  /// A way has to arrive on the target heading, and every eighth turn it
  /// still owes costs one bend penalty, so `bendPenalty` times the cyclic
  /// distance from a state's heading to the target's is a lower bound on
  /// what that state has left to pay. It is what the orthogonal search has
  /// always added; the free search did not.
  ///
  /// **On by default.** It is admissible on its own, but the sum of it and
  /// the distance term is not guaranteed to be consistent, and this search
  /// never reopens a closed state — so in principle it can return a way that
  /// turns more than the cheapest one. Measured on 4q, 9q, 17q and 21q it
  /// does not: the feedline angle cost, the edges drawn and the whole fail
  /// list come out byte for byte the same, and the Final stage runs 1.2 to
  /// 1.4 times faster. The prototype leaves the same term off by default
  /// (`DUBIN_TDH=2`) and records the trade at
  /// `dubin_router_opt.hpp:263-286`; measure the angle cost, not only the
  /// clock, before trusting it on a chip that is not in that list.
  void setBendLowerBound(const bool on) { bendLowerBound_ = on; }
  [[nodiscard]] Heuristic heuristic() const { return heuristic_; }

  /// Bound the turning a state still owes by the analytic Dubins bound, in
  /// the heuristic and against the turn bound of `setMaxTurns`. Off by
  /// default.
  ///
  /// For a state and the goal state of the search (the target moved back
  /// by its forced arrival run, as route() tests it), `minTurns` is
  /// `AnalyticDubins::minTurns` from the state to the goal: the fewest
  /// eighth turns of any way of this router's moves when nothing is in the
  /// way, or `AnalyticDubins::cap() + 1` when no such way turns at most the
  /// cap. Two things use it:
  ///
  /// - **The heuristic.** The bend term becomes the larger of the heading
  ///   term (`setBendLowerBound`) and `bendPenalty * minTurns`.
  /// - **The pruning.** Under a turn bound, a move is dropped when the
  ///   turns of the way so far plus `minTurns` of the state it reaches
  ///   exceed the bound, and `lastSearchCutOff` says so, as for a move that
  ///   exceeds the bound itself.
  ///
  /// **Why the heuristic stays admissible.** The cost of a move is its
  /// length, its proximity penalty and `bendPenalty` times its eighth turns
  /// (`headingDistance` from its heading to its exit heading). Take the
  /// three components one by one. The length and the penalty are bounded
  /// by the distance term, which the turn bound does not touch. For the
  /// bend component, `AnalyticDubins` models a way as the turning moves of
  /// the same primitives, read from the same `MovePrimitives`, joined by
  /// straight runs of any real length that is at least zero. The one move
  /// of a heading that does not turn is the straight step along it, which
  /// is such a run. Obstacles, the corridor, the grid edge and the spacing
  /// of arcs are dropped. So every way the search can take from a state to
  /// the goal is a way of the model with the same eighth turns, and the
  /// bend component of its cost is at least `bendPenalty * minTurns`, which
  /// is the optimum of the relaxed model. At `cap() + 1` the model has no
  /// way within the cap, so every real way turns more than the cap.
  ///
  /// **Why it stays consistent.** Let a move `m` turn `t` eighths and lead
  /// from state `s` to state `s'`. Take the best way of the model from `s'`
  /// and put `m` in front of it: `m` is a turning move of the model or a
  /// straight run along the heading of `s`, so the result is a way of the
  /// model from `s` with `t + minTurns(s')` turns. Hence
  /// `minTurns(s) <= t + minTurns(s')`, and the bend term drops by at most
  /// the bend component of the move. A value capped at `cap() + 1` keeps
  /// this: `min(a, c) <= t + min(b, c)` whenever `a <= t + b`. The heading
  /// term is consistent in the same way, and so is the larger of two
  /// consistent terms. `minTurns` is never below the heading term, because
  /// the analytic search starts at the heading distance. So the turn bound
  /// adds no inconsistency: each cost component is bounded by its own term,
  /// and the sum is consistent wherever the distance term is consistent
  /// against the length and the penalty (see `setBendLowerBound`).
  ///
  /// **Why the pruning loses no way within the bound.** A way through the
  /// reached state makes the turns so far plus the turns of its rest, and
  /// its rest is a way of the search from that state to the goal, so it
  /// turns at least `minTurns`. A move the pruning drops therefore has no
  /// continuation within the bound. The search closes a state at its least
  /// cost, not at its fewest turns, so without the pruning a cheaper way
  /// with too many turns can close a state that a way within the bound
  /// needs. The pruning keeps such ways off the open list, so a search
  /// under a bound may now find a way within it where it found none before.
  ///
  /// `minTurns` depends on the displacement to the goal and on the two
  /// headings alone, so the router keeps its values for its whole life:
  /// one byte per displacement up to `TURN_WINDOW` cells on either axis,
  /// and beyond that one byte per class of displacements that provably
  /// share their answer (see `minTurnsTo`).
  void setTurnLowerBound(bool on);
  [[nodiscard]] bool turnLowerBound() const { return turnLowerBound_; }

  // --- Grids -------------------------------------------------------------

  /// The static obstacles, shared and read-only. Read by the static
  /// proximity and the free box, not by the search itself, which reads the
  /// corridor.
  void attachObstacles(const grid::BitGrid* obstacles);
  [[nodiscard]] const grid::BitGrid* obstacles() const { return obstacles_; }

  /// The corridor of the wire about to be routed: the search enters the
  /// clear cells only. Packs the corridor with the static proximity into the
  /// working grid. Only the words of 64 cells that differ from the corridor
  /// packed last are packed again, so a corridor that changes in one box
  /// costs that box and one comparison per word of the grid.
  ///
  /// The packed grid is a copy of what the corridor says now, so a corridor
  /// is changed before it is attached, never after. A caller that rebuilds
  /// a corridor between two routes attaches it again.
  void attachCorridor(const grid::BitGrid* corridor);

  /// Like attachCorridor without the packing pass: the search reads the
  /// corridor bits and the proximity bytes separately, slower per search
  /// but free per attachment. For a caller that attaches a fresh corridor
  /// for each of many short searches.
  void attachCorridorUnpacked(const grid::BitGrid* corridor);
  [[nodiscard]] const grid::BitGrid* corridor() const { return corridor_; }
  [[nodiscard]] bool corridorBlocked(uint32_t x, uint32_t y) const;

  /// The static proximity penalty of every cell, 0..127.
  ///
  /// @throws std::invalid_argument on a wrong size or a value above 127.
  void setStaticProximity(std::vector<uint8_t> penalty);
  [[nodiscard]] const std::vector<uint8_t>& staticProximity() const {
    return static_;
  }

  /// The static proximity from the attached obstacles: an obstacle cell
  /// carries the full penalty, and it decays linearly to one over distance
  /// cells of four-connected growth.
  void computeStaticProximity(uint32_t distance, uint8_t penalty);

  /// Recompute the static proximity inside a window of cells only, with
  /// obstacles up to distance cells outside the window as seeds. Needs a
  /// full computation first.
  void computeStaticProximityWindow(uint32_t distance, uint8_t penalty,
                                    CellBox window);

  /// The wire proximity penalty of every cell, read through a view that
  /// the caller keeps alive while routing.
  ///
  /// Sixteen bits, not eight. The static layer stays a byte because it is a
  /// decay over a few cells, but a caller that wants to price one move in
  /// the range of a bend — a toll for crossing a wire, say — needs more than
  /// the 127 a byte allows: a bend is 14 950 on a 33-qubit grid, and the
  /// term is `PENALTY_SCALE * value`, so 127 could never buy more than
  /// 12 700 (user, 2026-10-01).
  void attachWireProximity(const std::vector<uint16_t>* penalty);
  [[nodiscard]] uint8_t staticPenalty(const std::size_t index) const {
    return static_[index];
  }
  [[nodiscard]] uint16_t wirePenalty(std::size_t index) const;

  // --- Crossing constraints ---------------------------------------------

  /// Build the crossing constraints of routeOrthogonal from routed wires:
  /// the cells within expandRadius of a straight run remember the run's
  /// heading, so a route may cross them at a right angle only; the cells
  /// around a curve and around the first and last ten cells of a wire may
  /// not be crossed at all. Wires flagged in skip are left out.
  void buildOrthogonalConstraints(const std::vector<Path>& wires,
                                  const std::vector<bool>& skip,
                                  int expandRadius = 10);
  void clearOrthogonalConstraints();

  /// The one feedline the next orthogonal route may cross, once. On the far
  /// side of it, within straightRadius of a straight run, a cell may be
  /// entered only while leaving the feedline at a right angle; within
  /// curveRadius of a curve or of the feedline's first cells, never. A wire
  /// that has crossed can therefore only leave. Nothing is set for null.
  void setSingleCrossingFeedline(const Path* feedline, int straightRadius,
                                 int curveRadius);

  /// Cells the crossing constraints do not bind in the next orthogonal
  /// routes: the room around a resonator's own coupler, where the edges of
  /// its chain pin and run beside it on purpose. Replaces the exemption set
  /// before; empty for none.
  void setCrossingExemption(const std::vector<uint32_t>& cells);

  /// Whether a route may enter a cell under a heading, by the test the
  /// orthogonal search runs on every move, so that a check of a committed
  /// path asks exactly what the search asked.
  [[nodiscard]] bool crossingAllowedOrthogonal(uint32_t x, uint32_t y,
                                               Heading heading) const;

  /// Whether the orthogonal search also tests the cell a move ends on with
  /// the heading the move **leaves** it on. Off, it tests every cell a move
  /// sweeps — the end cell included — with the heading the move entered on,
  /// while `reconstruct` records the end cell with the exit heading, which
  /// is what a check of the path then reads: a wire that crossed a feedline
  /// straight and turned on the last cell of its halo passed the search and
  /// failed the check (12 of the 22 crossing wires of 2026-10-04's baseline,
  /// each "10 cells from edge"). On, the search refuses what the check
  /// refuses.
  void setOrthogonalExitCheck(const bool on) { exitCheck_ = on; }

  /// The constraint mask of a cell: 0 free, 0xFF a curve or pin zone, else
  /// the bits of the headings of the straight runs present there.
  [[nodiscard]] uint8_t constraintMaskAt(uint32_t x, uint32_t y) const;

  /// The constraints themselves, which the design-rule check reads so that
  /// what it reports is what the search refused.
  [[nodiscard]] const CrossingConstraints& crossingConstraints() const {
    return constraints_;
  }

  // --- Searches -----------------------------------------------------------

  /// Route from the source to the target inside the corridor. The result
  /// starts at the source, runs its straight stubs, and ends at the target;
  /// it is empty when no path exists or the found path crossed itself.
  ///
  /// @param usePenalty Add the static and wire proximity penalties.
  /// @param onlyStraight Forbid diagonal headings.
  [[nodiscard]] Path route(const RoutingObjective& objective,
                           bool usePenalty = false, bool onlyStraight = false);

  /// Route as route does, but cross the wires of the crossing constraints
  /// at right angles only, and obey the single crossing feedline.
  [[nodiscard]] Path routeOrthogonal(const RoutingObjective& objective,
                                     bool usePenalty = false,
                                     bool onlyStraight = false);

  /// What the last route() paid for its way, in the cost units of the
  /// search: the bends, the length and, with `usePenalty`, the proximity.
  /// Zero when it found no way.
  [[nodiscard]] uint32_t lastSearchCost() const { return lastCost_; }

  /// How many states the last route() expanded: the states
  /// it took off the open list and closed.
  [[nodiscard]] uint64_t lastSearchExpansions() const { return expansions_; }

  /// Searches whose path crossed itself and were rejected.
  [[nodiscard]] uint64_t loopGuardRejections() const {
    return loopGuardRejections_;
  }

  // --- Helpers for the consumers of a route ------------------------------

  /// The point a search starts or ends at: the source moved forward along
  /// its heading by the start straight length, or the target moved back
  /// along its heading by the end straight length. Not clamped to the grid.
  [[nodiscard]] PathPoint sanitize(PathPoint point, bool isTarget) const;

  /// The cells of a straight stub of a length from a point along its
  /// heading, from the point outward, tagged with the straight primitive.
  [[nodiscard]] Path straightStub(PathPoint point, bool isTarget,
                                  uint32_t length) const;

  /// The widest obstacle-free strip along the segment between two cells:
  /// the box of the rectangle grown to either side of the segment until an
  /// edge hits an obstacle. Needs attached obstacles.
  [[nodiscard]] CellBox freeStripAlong(PathPoint from, PathPoint to) const;

  /// What `freeStripAlong` learnt of the free cells beside the cells of an
  /// axis-parallel segment: per cell and side, how many cells in a row are
  /// free. It holds while the obstacles do not change, so a caller keeps one
  /// for the calls of one meander insertion.
  struct FreeStripMemo {
    std::unordered_map<uint64_t, uint32_t> depths;
  };

  /// `freeStripAlong` with a memo. On an axis-parallel segment the strip
  /// grows on a side until a cell beside the segment is blocked, so its width
  /// is the least free depth of the segment's cells on that side; the depths
  /// are taken from the memo. Every other segment is measured as without it.
  [[nodiscard]] CellBox freeStripAlong(PathPoint from, PathPoint to,
                                       FreeStripMemo& memo) const;

private:
  struct QueueEntry {
    uint16_t x = 0;
    uint16_t y = 0;
    uint8_t heading = 0;
    /// Eighth-turns made so far on the way to this state, for `setMaxTurns`.
    uint8_t turns = 0;
    uint16_t primitive = 0;
    uint32_t f = 0;
    uint32_t g = 0;
  };

  /// A node of the swept-cell trie of one heading: the cells the primitives
  /// of the heading sweep, merged by common prefix, in preorder with a
  /// subtree skip. An expansion walks the shared cells once and drops every
  /// primitive behind a blocked cell in one step.
  struct TrieNode {
    int32_t linear = 0;
    int8_t dx = 0;
    int8_t dy = 0;
    uint16_t primitives = 0;
    uint8_t completes = NO_PRIMITIVE;
    uint8_t depth = 0;
    uint16_t skip = 0;
  };
  struct TriePrimitive {
    uint32_t costBend = 0;
    int16_t endDx = 0;
    int16_t endDy = 0;
    uint16_t id = 0;
    uint16_t sweptCount = 0;
    uint8_t exit = 0;
    uint8_t sweepsOrigin = 0;
    uint8_t endExtra = 0;
  };
  static constexpr uint8_t NO_PRIMITIVE = 0xFF;
  static constexpr uint32_t MAX_PRIMITIVES_PER_HEADING = 16;
  static constexpr uint32_t MAX_TRIE_DEPTH = 60;

  static constexpr uint32_t FIELD_DISTANCE_BITS = 22;
  static constexpr uint32_t FIELD_DISTANCE_MASK =
      (1U << FIELD_DISTANCE_BITS) - 1U;
  static constexpr uint32_t FIELD_BUCKETS = 1024;
  static constexpr uint8_t SIDE_FAR = 0x80;
  static constexpr uint8_t SIDE_STRAIGHT = 0x40;
  static constexpr uint8_t CURVE_ZONE = CrossingConstraints::CURVE_ZONE;

  void buildTables();
  void rebuildPacked();
  /// `freeStripAlong`, with a memo or without one.
  [[nodiscard]] CellBox freeStrip(PathPoint from, PathPoint to,
                                  FreeStripMemo* memo) const;
  void buildDistanceField(uint32_t tx, uint32_t ty);
  bool beginSingleCrossingOverlay(const PathPoint& source,
                                  const PathPoint& target);
  void endSingleCrossingOverlay();

  [[nodiscard]] uint32_t stateIndex(const uint32_t x, const uint32_t y,
                                    const Heading heading) const {
    return scratch_.index(x, y, heading);
  }
  [[nodiscard]] PathPoint unpackState(uint32_t index) const;
  [[nodiscard]] uint32_t parentState(uint32_t index) const;
  [[nodiscard]] Path reconstruct(uint32_t goalIndex, uint32_t startIndex) const;
  [[nodiscard]] Path assemble(Path searched, const PathPoint& source,
                              const PathPoint& target) const;
  [[nodiscard]] bool inGrid(const PathPoint& point) const {
    return point.x < width_ && point.y < height_;
  }
  [[nodiscard]] bool canCrossOrthogonal(uint32_t x, uint32_t y,
                                        Heading heading) const;

  template <typename Search> Path guarded(Search&& search);
  template <bool PACKED, bool USE_PENALTY>
  Path searchFree(const RoutingObjective& objective, const PathPoint& source,
                  const PathPoint& target, bool onlyStraight);
  template <bool PACKED, bool USE_PENALTY>
  void expandFree(const QueueEntry& current, const RoutingObjective& objective,
                  bool onlyStraight, const uint8_t* blockMap,
                  const uint8_t* penaltyMap, uint8_t blockMask,
                  uint8_t penaltyMask, const uint16_t* wireMap, bool useField);
  template <bool USE_PENALTY>
  Path searchOrthogonal(const RoutingObjective& objective,
                        const PathPoint& source, const PathPoint& target,
                        bool onlyStraight);
  template <bool USE_PENALTY>
  void expandOrthogonal(const QueueEntry& current,
                        const RoutingObjective& objective, bool onlyStraight);

  std::shared_ptr<const MovePrimitives> primitives_;
  SearchScratch& scratch_;
  SearchParams params_;
  uint32_t width_;
  uint32_t height_;
  Heuristic heuristic_ = Heuristic::DistanceField;
  int16_t maxTurns_ = -1;
  bool cutOff_ = false;
  BoundWindow boundWindow_;
  uint32_t lastCost_ = 0;
  uint64_t expansions_ = 0;
  bool bendLowerBound_ = true;
  /// Per heading, what a state facing it still owes in turning, in the cost
  /// units of the search. Filled once per search; all zeroes when the bend
  /// lower bound is off, so the expansion adds it unconditionally.
  std::array<uint32_t, 8> bendLb_{};

  /// The analytic turn bound (`setTurnLowerBound`): the switch, the model,
  /// the goal states the running search bounds against (empty when the
  /// running search does not use it) and the remembered values.
  bool turnLowerBound_ = false;
  std::unique_ptr<AnalyticDubins> analytic_;
  std::vector<PathPoint> turnGoals_;
  /// The displacements up to `TURN_WINDOW` cells on either axis, per goal
  /// heading: one byte per displacement and state heading, `TURN_UNKNOWN`
  /// until computed. Allocated per goal heading when first used.
  static constexpr int32_t TURN_WINDOW = 255;
  static constexpr uint8_t TURN_UNKNOWN = 0xFF;
  std::array<std::vector<uint8_t>, NUM_HEADINGS> turnWindow_;
  /// The displacements outside the window, one value per class of
  /// displacements that share their answer (see `minTurnsTo`), per goal
  /// heading and state heading.
  std::array<std::vector<uint8_t>, NUM_HEADINGS> turnFarTable_;
  /// How far a partial sum of the model's turning moves reaches on either
  /// axis: `cap()` times the longest turning move.
  int32_t turnReach_ = 0;
  /// `minTurns` from a state to a goal state, remembered.
  [[nodiscard]] uint32_t minTurnsTo(uint32_t x, uint32_t y, Heading heading,
                                    const PathPoint& goal);
  /// The least `minTurnsTo` over `turnGoals_`.
  [[nodiscard]] uint32_t turnBoundOf(uint32_t x, uint32_t y, Heading heading);

  const grid::BitGrid* obstacles_ = nullptr;
  const grid::BitGrid* corridor_ = nullptr;
  std::vector<uint8_t> static_;
  const std::vector<uint16_t>* wire_ = nullptr;
  std::vector<uint8_t> packed_;
  bool packedValid_ = false;
  /// The corridor words `packed_` was last packed from.
  std::vector<uint64_t> packedWords_;
  /// Whether `static_` changed since the last packing, so that every cell
  /// has to be packed again.
  bool packedStale_ = true;

  CrossingConstraints constraints_;
  std::vector<uint8_t> exempt_;
  std::vector<uint32_t> exemptCells_;
  bool exitCheck_ = false;
  std::vector<uint8_t> crossingSide_;
  std::vector<uint32_t> crossingSideCells_;
  bool crossingSideActive_ = false;
  const Path* singleCrossing_ = nullptr;
  int singleCrossingStraightRadius_ = 0;
  int singleCrossingCurveRadius_ = 1;

  std::array<std::vector<TrieNode>, NUM_HEADINGS> trie_;
  std::array<std::vector<TriePrimitive>, NUM_HEADINGS> triePrimitives_;
  std::array<uint32_t, NUM_HEADINGS> marginLeft_{};
  std::array<uint32_t, NUM_HEADINGS> marginRight_{};
  std::array<uint32_t, NUM_HEADINGS> marginUp_{};
  std::array<uint32_t, NUM_HEADINGS> marginDown_{};
  bool tablesValid_ = false;

  std::vector<uint32_t> field_;
  uint32_t fieldStamp_ = 0;
  bool fieldValid_ = false;
  std::array<std::vector<uint32_t>, FIELD_BUCKETS> fieldBuckets_;

  BucketQueue<QueueEntry> open_;
  PathLoopScratch loopScratch_;
  uint64_t loopGuardRejections_ = 0;
  std::vector<uint8_t> proximityVisited_;
  std::vector<uint32_t> proximityFrontA_;
  std::vector<uint32_t> proximityFrontB_;
};

} // namespace mqt::scpd::routing
