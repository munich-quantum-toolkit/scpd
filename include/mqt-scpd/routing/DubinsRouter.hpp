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
#include "mqt-scpd/routing/BucketQueue.hpp"
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

/// A box of cells, both bounds included.
struct CellBox {
  uint32_t minX = 0;
  uint32_t maxX = 0;
  uint32_t minY = 0;
  uint32_t maxY = 0;
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
  [[nodiscard]] const MovePrimitives& primitives() const { return *primitives_; }
  [[nodiscard]] const SearchParams& params() const { return params_; }

  /// Change the search parameters. Only a changed bend penalty rebuilds the
  /// search tables.
  ///
  /// @throws std::invalid_argument when the bend radius is not the one the
  /// primitives were built with.
  void setParams(const SearchParams& params);
  void setHeuristic(const Heuristic heuristic) { heuristic_ = heuristic; }
  [[nodiscard]] Heuristic heuristic() const { return heuristic_; }

  // --- Grids -------------------------------------------------------------

  /// The static obstacles, shared and read-only. Read by the static
  /// proximity and the free box, not by the search itself, which reads the
  /// corridor.
  void attachObstacles(const grid::BitGrid* obstacles);
  [[nodiscard]] const grid::BitGrid* obstacles() const { return obstacles_; }

  /// The corridor of the wire about to be routed: the search enters the
  /// clear cells only. Packs the corridor with the static proximity into the
  /// working grid, which costs one pass over the grid.
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
  void attachWireProximity(const std::vector<uint8_t>* penalty);
  [[nodiscard]] uint8_t staticPenalty(const std::size_t index) const {
    return static_[index];
  }
  [[nodiscard]] uint8_t wirePenalty(std::size_t index) const;

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

  /// Whether a route may enter a cell under a heading, by the test the
  /// orthogonal search runs on every move, so that a check of a committed
  /// path asks exactly what the search asked.
  [[nodiscard]] bool crossingAllowedOrthogonal(uint32_t x, uint32_t y,
                                               Heading heading) const;

  /// The constraint mask of a cell: 0 free, 0xFF a curve or pin zone, else
  /// the bits of the headings of the straight runs present there.
  [[nodiscard]] uint8_t constraintMaskAt(uint32_t x, uint32_t y) const;

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

private:
  struct QueueEntry {
    uint16_t x = 0;
    uint16_t y = 0;
    uint8_t heading = 0;
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
  static constexpr uint32_t FIELD_DISTANCE_MASK = (1U << FIELD_DISTANCE_BITS) - 1U;
  static constexpr uint32_t FIELD_BUCKETS = 1024;
  static constexpr uint8_t SIDE_FAR = 0x80;
  static constexpr uint8_t SIDE_STRAIGHT = 0x40;
  static constexpr uint8_t CURVE_ZONE = 0xFF;

  void buildTables();
  void rebuildPacked();
  void buildDistanceField(uint32_t tx, uint32_t ty);
  bool beginSingleCrossingOverlay(const PathPoint& source, const PathPoint& target);
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
                  uint8_t penaltyMask, const uint8_t* wireMap, bool useField);
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

  const grid::BitGrid* obstacles_ = nullptr;
  const grid::BitGrid* corridor_ = nullptr;
  std::vector<uint8_t> static_;
  const std::vector<uint8_t>* wire_ = nullptr;
  std::vector<uint8_t> packed_;
  bool packedValid_ = false;

  std::vector<uint8_t> constraints_;
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
