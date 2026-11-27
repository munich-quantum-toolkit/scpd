/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/pipeline/DetailRouter.hpp"

#include "mqt-scpd/flatbuffers/artifacts.hpp"
#include "mqt-scpd/flatbuffers/config.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/flatbuffers/geometry.hpp"
#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/grid/Partitions.hpp"
#include "mqt-scpd/grid/Watershed.hpp"
#include "mqt-scpd/pipeline/CapacityPlanner.hpp"
#include "mqt-scpd/pipeline/Report.hpp"
#include "mqt-scpd/pipeline/Stages.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <functional>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <queue>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace mqt::scpd::pipeline {
namespace {

namespace fba = flatbuffers::artifacts;
namespace fbg = flatbuffers::geometry;

using grid::LABEL_NONE;
using grid::PartitionLabel;

/// No cell of the grid.
constexpr std::uint32_t NOWHERE = std::numeric_limits<std::uint32_t>::max();
/// No wire.
constexpr std::uint32_t NO_OWNER = std::numeric_limits<std::uint32_t>::max();

/// The eight steps of the search and their costs.
///
/// A step costs 10 along an axis and 14 along a diagonal, an integer form of
/// 1 and the square root of 2. The heuristic is the octile distance in the
/// same units, so it never overestimates.
constexpr std::array<std::int32_t, 8> STEP_X = {1, -1, 0, 0, 1, 1, -1, -1};
constexpr std::array<std::int32_t, 8> STEP_Y = {0, 0, 1, -1, 1, -1, 1, -1};
constexpr std::array<std::uint32_t, 8> STEP_COST = {10, 10, 10, 10,
                                                    14, 14, 14, 14};
constexpr std::size_t DIAGONAL_STEPS = 4;

/// The weight of one unit of the obstacle cost and of the steering cost, so
/// that one unit costs as much as one step along an axis.
///
/// Every cost of the stage is a cost per step, never per cell of a clearance
/// disc. A step is the same fraction of a path on every grid, so the costs
/// mean the same on a fine grid and on a coarse one.
constexpr std::uint32_t PENALTY_WEIGHT = 10;

/// The cost of a step onto a cell, per cell of another wire among its eight
/// neighbors.
///
/// The cost is not the clearance, which the stage enforces as a block. It
/// keeps a wire away from the others where there is room, so that the first
/// wire through a passage does not take its middle.
constexpr std::uint32_t CROWDING_WEIGHT = 10;

/// The parameters of the stage, with the defaults of the schema.
struct Tuning {
  /// How far a wire may move to either side of its path, in wire spacings.
  std::uint32_t corridorSpacings = 4;
  std::uint32_t rounds = 30;
  std::uint32_t maxRelaxation = 30;
  /// How far the obstacle penalty reaches, in layout units.
  double penaltyReach = 185.0;
  std::uint32_t penalty = 10;
  std::uint32_t orderings = 16;
};

[[nodiscard]] Tuning tuningOf(const ConfigT& config) {
  if (config.stages != nullptr && config.stages->detail != nullptr) {
    const auto& params = *config.stages->detail;
    return {.corridorSpacings = params.corridor_spacings,
            .rounds = params.rounds,
            .maxRelaxation = params.max_relaxation,
            .penaltyReach = params.obstacle_penalty_reach,
            .penalty = params.obstacle_penalty,
            .orderings = params.orderings};
  }
  return {};
}

/// The clearance between two wires, as a disc on the detail grid.
///
/// `min_wire_spacing` is a length in layout units, and the router works in
/// cells. The rule spans `ceil(spacing / cell)` cells, and the stage keeps one
/// cell less than that clear: two wires are far enough apart when the distance
/// between their cells is more than `pixels`. A cell is coarser than the rule,
/// so a wire's position is known only to a cell, and the check of the stage
/// uses the same number of cells as the search.
struct Rule {
  /// The rule as the configuration gives it, in layout units.
  double spacing = 0.0;
  /// The clearance on this grid, in cells.
  std::uint32_t pixels = 0;
  /// The half-width of each disc row, indexed by distance from the center row.
  std::vector<std::uint32_t> halfWidths;

  [[nodiscard]] bool binds() const { return !halfWidths.empty(); }
};

[[nodiscard]] Rule ruleOf(const ConfigT& config,
                          const grid::GridMetrics& detail) {
  Rule rule;
  if (config.rules == nullptr || config.rules->min_wire_spacing <= 0.0) {
    return rule;
  }
  rule.spacing = config.rules->min_wire_spacing;
  const auto spans = grid::cellsFor(rule.spacing, detail);
  if (spans == 0) {
    return rule;
  }
  rule.pixels = spans - 1;
  const auto radius = static_cast<std::uint64_t>(rule.pixels);
  const auto squaredRadius = radius * radius;
  const auto maxRow = std::min(rule.pixels, detail.height - 1);
  auto halfWidth =
      static_cast<std::uint64_t>(std::min(rule.pixels, detail.width - 1));
  rule.halfWidths.reserve(static_cast<std::size_t>(maxRow) + 1);
  for (std::uint64_t dy = 0; dy <= maxRow; ++dy) {
    const auto squaredWidth = squaredRadius - (dy * dy);
    // The disc narrows as the rows move away from the center.
    while (halfWidth * halfWidth > squaredWidth) {
      --halfWidth;
    }
    rule.halfWidths.push_back(static_cast<std::uint32_t>(halfWidth));
  }
  return rule;
}

/// What one search may enter, and what it pays to enter it.
struct Query {
  std::uint32_t start = NOWHERE;
  std::uint32_t goal = NOWHERE;
  /// The wire the search is for. Its own cells are released before the
  /// search runs, except possibly its goal.
  std::uint32_t wire = NO_OWNER;
  /// The partition the search may not leave, or `LABEL_NONE` for a search
  /// that `mask` bounds instead.
  PartitionLabel partition = LABEL_NONE;
  /// The cells the search may enter, or none for all of them.
  const std::vector<std::uint8_t>* mask = nullptr;
  /// Whether a step close to an obstacle costs extra.
  bool avoidObstacles = false;
  /// A cost per cell that steers the search without blocking anything, or
  /// none. The rerouting builds one when the first search of a wire fails.
  const std::vector<std::uint8_t>* steering = nullptr;
  /// The cost of a step through a cell of another wire, or zero to block it.
  /// Set above the cost of any path, it makes the search take a free path
  /// where one exists and cross as few wires as it can where none does.
  std::uint32_t displacement = 0;
};

/// The detail grid that a run is drawn on.
///
/// The canvas holds the blocked cells of the scene, the partition of each
/// cell, the cost of running beside an obstacle and the wire that owns each
/// cell. It runs the one search that every pass of the stage uses, and it
/// keeps the working arrays of that search, so the stage allocates them once
/// and not once per search.
class Canvas {
public:
  Canvas(const grid::GridMetrics& metrics, grid::BitGrid blocked,
         std::vector<PartitionLabel> labels, const Tuning& tuning)
      : gridMetrics(metrics), blockedCells(std::move(blocked)),
        cellLabels(std::move(labels)),
        proximity(proximityOf(blockedCells,
                              grid::cellsFor(tuning.penaltyReach, metrics),
                              tuning.penalty)),
        cellOwners(blockedCells.size(), NO_OWNER),
        crowding(blockedCells.size(), 0), guardedCells(blockedCells.size(), 0),
        guardCount(blockedCells.size(), 0), searchCost(blockedCells.size(), 0),
        searchState(blockedCells.size(), 0) {}

  [[nodiscard]] const grid::GridMetrics& metrics() const { return gridMetrics; }
  [[nodiscard]] std::uint32_t width() const { return gridMetrics.width; }
  [[nodiscard]] std::uint32_t height() const { return gridMetrics.height; }
  [[nodiscard]] std::size_t cells() const { return blockedCells.size(); }

  [[nodiscard]] std::int64_t columnOf(const std::uint32_t cell) const {
    return static_cast<std::int64_t>(cell % gridMetrics.width);
  }
  [[nodiscard]] std::int64_t rowOf(const std::uint32_t cell) const {
    return static_cast<std::int64_t>(cell / gridMetrics.width);
  }

  [[nodiscard]] bool blocked(const std::uint32_t cell) const {
    return blockedCells.test(cell);
  }
  [[nodiscard]] PartitionLabel label(const std::uint32_t cell) const {
    return cellLabels[cell];
  }
  [[nodiscard]] std::uint32_t owner(const std::uint32_t cell) const {
    return cellOwners[cell];
  }
  /// Whether a wire may stand on a cell: the cell is free and no wire owns it.
  [[nodiscard]] bool standable(const std::uint32_t cell) const {
    return !blockedCells.test(cell) && cellOwners[cell] == NO_OWNER;
  }

  /// The wire whose diagonal a step from one cell to another crosses, or
  /// none. Only a diagonal step crosses a wire, and only when that wire owns
  /// both cells beside the step.
  [[nodiscard]] std::uint32_t crossedBy(const std::uint32_t from,
                                        const std::uint32_t to) const {
    const auto x = columnOf(to);
    const auto y = rowOf(to);
    const auto px = columnOf(from);
    const auto py = rowOf(from);
    if (x == px || y == py) {
      return NO_OWNER;
    }
    const auto width = static_cast<std::int64_t>(gridMetrics.width);
    const auto first = static_cast<std::uint32_t>((py * width) + x);
    const auto second = static_cast<std::uint32_t>((y * width) + px);
    return cellOwners[first] == cellOwners[second] ? cellOwners[first]
                                                   : NO_OWNER;
  }

  /// Gives a cell to a wire.
  void take(const std::uint32_t cell, const std::uint32_t wire) {
    if (cellOwners[cell] == NO_OWNER) {
      crowd(cell, 1);
    }
    cellOwners[cell] = wire;
  }

  /// Takes a cell from its wire, with the clearance it holds.
  void release(const std::uint32_t cell) {
    unguard(cell);
    if (cellOwners[cell] != NO_OWNER) {
      crowd(cell, -1);
    }
    cellOwners[cell] = NO_OWNER;
  }

  /// Adds one wire cell to the crowding of the eight neighbors of a cell, or
  /// removes one.
  void crowd(const std::uint32_t cell, const int by) {
    const auto width = static_cast<std::int64_t>(gridMetrics.width);
    const auto height = static_cast<std::int64_t>(gridMetrics.height);
    for (std::size_t step = 0; step < STEP_X.size(); ++step) {
      const auto nx = columnOf(cell) + STEP_X[step];
      const auto ny = rowOf(cell) + STEP_Y[step];
      if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
        continue;
      }
      auto& value = crowding[static_cast<std::size_t>((ny * width) + nx)];
      value = static_cast<std::uint8_t>(static_cast<int>(value) + by);
    }
  }

  /// The cheapest path from `start` to `goal` that the query allows.
  ///
  /// Only the blocked cells bind the two ends. A wire starts where the
  /// assignment feeds it and ends at the cell of its target port, and a piece
  /// of a wire starts and ends at a crossing that lies on a partition border.
  /// The query bounds the path between the ends.
  ///
  /// @return The cells from `start` to `goal`, or none when the query allows
  /// no path.
  [[nodiscard]] std::optional<std::vector<std::uint32_t>>
  search(const Query& query) {
    if (query.start == NOWHERE || query.goal == NOWHERE ||
        blockedCells.test(query.start) || blockedCells.test(query.goal)) {
      return std::nullopt;
    }
    if (query.start == query.goal) {
      return std::vector<std::uint32_t>{query.start};
    }

    generation += GENERATION_STEP;
    if (generation == 0) {
      std::ranges::fill(searchState, 0);
      generation = GENERATION_STEP;
    }
    const auto width = static_cast<std::int64_t>(gridMetrics.width);
    const auto height = static_cast<std::int64_t>(gridMetrics.height);
    const auto goalX = columnOf(query.goal);
    const auto goalY = rowOf(query.goal);
    const auto heuristic = [goalX, goalY](const std::int64_t x,
                                          const std::int64_t y) {
      const auto dx = std::abs(x - goalX);
      const auto dy = std::abs(y - goalY);
      return static_cast<std::uint32_t>((10 * std::max(dx, dy)) +
                                        (4 * std::min(dx, dy)));
    };

    using Entry = std::pair<std::uint64_t, std::uint32_t>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<>> open;
    searchCost[query.start] = 0;
    searchState[query.start] = generation;
    open.emplace(heuristic(columnOf(query.start), rowOf(query.start)),
                 query.start);

    while (!open.empty()) {
      const auto cell = open.top().second;
      open.pop();
      if ((searchState[cell] & CLOSED_BIT) != 0) {
        continue;
      }
      searchState[cell] |= CLOSED_BIT;
      const auto cost = searchCost[cell];
      if (cell == query.goal) {
        return trace(query.start, query.goal);
      }
      const auto x = columnOf(cell);
      const auto y = rowOf(cell);
      for (std::size_t step = 0; step < STEP_X.size(); ++step) {
        const auto nx = x + STEP_X[step];
        const auto ny = y + STEP_Y[step];
        if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
          continue;
        }
        const auto next = static_cast<std::uint32_t>((ny * width) + nx);
        const auto state = searchState[next];
        // The octile heuristic is consistent and each edge surcharge is
        // nonnegative, so a closed cell already has its lowest cost.
        if (state >= (generation | CLOSED_BIT) || !admits(query, next)) {
          continue;
        }
        // A diagonal step between two cells of one wire crosses that wire in
        // copper. It is the only way two eight-connected paths cross without
        // a shared cell, because the other wire then owns the opposite
        // diagonal of the same two-by-two block. Two different wires on the
        // two cells only pass close, which the clearance rule covers. A
        // diagonal step past a blocked cell cuts the obstacle.
        auto charge =
            query.avoidObstacles
                ? PENALTY_WEIGHT * static_cast<std::uint32_t>(proximity[next])
                : 0U;
        if (cellOwners[next] != NO_OWNER && cellOwners[next] != query.wire) {
          charge += query.displacement;
        }
        if (query.steering != nullptr) {
          charge += PENALTY_WEIGHT *
                    static_cast<std::uint32_t>((*query.steering)[next]);
        }
        charge += CROWDING_WEIGHT * static_cast<std::uint32_t>(crowding[next]);
        if (step >= DIAGONAL_STEPS) {
          const auto first = static_cast<std::uint32_t>((y * width) + nx);
          const auto second = static_cast<std::uint32_t>((ny * width) + x);
          if (blockedCells.test(first) || blockedCells.test(second)) {
            continue;
          }
          const auto crossed = cellOwners[first] == cellOwners[second]
                                   ? cellOwners[first]
                                   : NO_OWNER;
          if (crossed != NO_OWNER) {
            // A wire never crosses its own diagonal, at any cost: that is a
            // short in one wire. Nor does it cross a wire that is pinned.
            if (crossed == query.wire || query.displacement == 0 ||
                guardCount[first] != 0 || guardCount[second] != 0) {
              continue;
            }
            charge += query.displacement;
          }
        }
        const auto reached = cost + STEP_COST[step] + charge;
        if (state >= generation && reached >= searchCost[next]) {
          continue;
        }
        searchCost[next] = reached;
        searchState[next] = generation | static_cast<std::uint32_t>(step);
        open.emplace(reached + heuristic(nx, ny), next);
      }
    }
    return std::nullopt;
  }

  /// Puts the clearance rule into force.
  ///
  /// Before this call, a wire blocks only its own cells. After it, every cell
  /// within the clearance of a wire is guarded, and no search enters a guarded
  /// cell. The seed passes run before this call, so they do not maintain a
  /// field they do not read.
  void armClearance(const Rule& rule) {
    activeRule = rule;
    if (!activeRule.binds()) {
      return;
    }
    for (std::size_t cell = 0; cell < cellOwners.size(); ++cell) {
      if (cellOwners[cell] != NO_OWNER) {
        guard(static_cast<std::uint32_t>(cell));
      }
    }
  }

  /// How many guarded wire cells lie within the clearance of a cell. While
  /// the wire of the cell is off the canvas, this counts the cells of other
  /// wires that are too close.
  [[nodiscard]] std::uint32_t guardOf(const std::uint32_t cell) const {
    return guardCount[cell];
  }

  /// Calls `apply` with the first cell index and length of each clearance
  /// row clipped to the grid.
  template <class Apply>
  void forClearanceRows(const std::uint32_t cell, const Apply& apply) const {
    if (!activeRule.binds()) {
      return;
    }
    const auto width = static_cast<std::int64_t>(gridMetrics.width);
    const auto height = static_cast<std::int64_t>(gridMetrics.height);
    const auto cx = columnOf(cell);
    const auto cy = rowOf(cell);
    const auto reach =
        static_cast<std::int64_t>(activeRule.halfWidths.size() - 1);
    const auto lastRow = std::min(height - 1, cy + reach);
    for (auto y = std::max(std::int64_t{0}, cy - reach); y <= lastRow; ++y) {
      const auto halfWidth =
          activeRule.halfWidths[static_cast<std::size_t>(std::abs(y - cy))];
      const auto left = std::max(std::int64_t{0}, cx - halfWidth);
      const auto right = std::min(width, cx + halfWidth + 1);
      apply(static_cast<std::size_t>((y * width) + left),
            static_cast<std::size_t>(right - left));
    }
  }

  /// Gives a cell to a wire and guards the clearance around it.
  void occupy(const std::uint32_t cell, const std::uint32_t wire) {
    take(cell, wire);
    guard(cell);
  }

  /// Blocks one cell for every search, also for a search that may displace
  /// wires, or opens it again. A pinned cell counts as guarded, with or
  /// without a clearance rule.
  void pin(const std::uint32_t cell, const bool on) {
    if (on) {
      ++guardCount[cell];
    } else {
      --guardCount[cell];
    }
  }

  /// Guards the clearance around one cell, once.
  void guard(const std::uint32_t cell) {
    if (guardedCells[cell] != 0 || !activeRule.binds()) {
      return;
    }
    guardedCells[cell] = 1;
    stamp(cell, 1);
  }

  /// Removes the guard around one cell, once. The wire keeps the cell, so a
  /// wire that is routed again keeps its feed and its target this way.
  void unguard(const std::uint32_t cell) {
    if (guardedCells[cell] == 0) {
      return;
    }
    guardedCells[cell] = 0;
    stamp(cell, -1);
  }

private:
  /// Adds or removes the guard of one cell on every cell within its
  /// clearance.
  void stamp(const std::uint32_t cell, const int by) {
    forClearanceRows(cell, [this, by](const std::size_t first,
                                      const std::size_t length) {
      for (auto& value : std::span{guardCount}.subspan(first, length)) {
        value =
            static_cast<std::uint32_t>(static_cast<std::int64_t>(value) + by);
      }
    });
  }

  /// Whether a search may enter a cell.
  [[nodiscard]] bool admits(const Query& query,
                            const std::uint32_t cell) const {
    if (blockedCells.test(cell)) {
      return false;
    }
    if (cell == query.goal) {
      // The goal is the target cell or the feed of the wire. The wire has to
      // end there, so no clearance keeps it out.
      return cellOwners[cell] == NO_OWNER || cellOwners[cell] == query.wire ||
             query.displacement > 0;
    }
    if (cellOwners[cell] != NO_OWNER && query.displacement == 0) {
      return false;
    }
    if (guardCount[cell] != 0) {
      return false;
    }
    if (query.partition != LABEL_NONE && cellLabels[cell] != query.partition) {
      return false;
    }
    return query.mask == nullptr || (*query.mask)[cell] != 0;
  }

  [[nodiscard]] std::vector<std::uint32_t>
  trace(const std::uint32_t start, const std::uint32_t goal) const {
    std::vector<std::uint32_t> path;
    for (auto cell = goal;;) {
      path.push_back(cell);
      if (cell == start) {
        break;
      }
      const auto step = searchState[cell] & DIRECTION_MASK;
      cell = static_cast<std::uint32_t>(
          static_cast<std::int64_t>(cell) - STEP_X[step] -
          (static_cast<std::int64_t>(gridMetrics.width) * STEP_Y[step]));
    }
    std::ranges::reverse(path);
    return path;
  }

  /// The cost of a step close to an obstacle, cell by cell.
  ///
  /// A four-connected breadth-first sweep out of the blocked cells, whose
  /// value falls off linearly over `radius` layers. The cost blocks nothing;
  /// it keeps a wire off the artwork where there is room.
  [[nodiscard]] static std::vector<std::uint8_t>
  proximityOf(const grid::BitGrid& blocked, const std::uint64_t radius,
              const std::uint32_t penalty) {
    std::vector<std::uint8_t> values(blocked.size(), 0);
    if (radius == 0 || penalty == 0) {
      return values;
    }
    const auto width = static_cast<std::int64_t>(blocked.width());
    const auto height = static_cast<std::int64_t>(blocked.height());
    std::vector<std::uint32_t> front;
    std::vector<std::uint32_t> next;
    std::vector<bool> seen(blocked.size(), false);
    for (std::size_t cell = 0; cell < blocked.size(); ++cell) {
      if (blocked.test(cell)) {
        values[cell] = static_cast<std::uint8_t>(penalty);
        seen[cell] = true;
        front.push_back(static_cast<std::uint32_t>(cell));
      }
    }
    constexpr std::array<std::int64_t, 4> aroundX = {1, -1, 0, 0};
    constexpr std::array<std::int64_t, 4> aroundY = {0, 0, 1, -1};
    for (std::uint64_t layer = 1; layer <= radius && !front.empty(); ++layer) {
      const auto value = static_cast<std::uint8_t>(std::max<std::uint64_t>(
          1, (penalty * (radius - layer + 1)) / (radius + 1)));
      next.clear();
      for (const auto cell : front) {
        const auto x = static_cast<std::int64_t>(cell % blocked.width());
        const auto y = static_cast<std::int64_t>(cell / blocked.width());
        for (std::size_t step = 0; step < aroundX.size(); ++step) {
          const auto nx = x + aroundX[step];
          const auto ny = y + aroundY[step];
          if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
            continue;
          }
          const auto neighbor = static_cast<std::uint32_t>((ny * width) + nx);
          if (seen[neighbor]) {
            continue;
          }
          seen[neighbor] = true;
          values[neighbor] = value;
          next.push_back(neighbor);
        }
      }
      front.swap(next);
    }
    return values;
  }

  grid::GridMetrics gridMetrics;
  grid::BitGrid blockedCells;
  std::vector<PartitionLabel> cellLabels;
  std::vector<std::uint8_t> proximity;
  std::vector<std::uint32_t> cellOwners;
  std::vector<std::uint8_t> crowding;
  /// The clearance rule in force, the cells whose clearance is guarded, and
  /// for each cell how many guarded wire cells lie within the clearance.
  Rule activeRule;
  std::vector<std::uint8_t> guardedCells;
  std::vector<std::uint32_t> guardCount;

  /// The low three bits store the step from the parent. The next bit marks
  /// a closed cell. Higher bits identify the search that visited the cell.
  static constexpr std::uint32_t CLOSED_BIT = 1U << 3U;
  static constexpr std::uint32_t DIRECTION_MASK = CLOSED_BIT - 1;
  static constexpr std::uint32_t GENERATION_STEP = 2 * CLOSED_BIT;
  static_assert(STEP_X.size() == CLOSED_BIT && STEP_Y.size() == CLOSED_BIT &&
                STEP_COST.size() == CLOSED_BIT);
  std::vector<std::uint64_t> searchCost;
  std::vector<std::uint32_t> searchState;
  std::uint32_t generation = 0;
};

/// One wire, as the corridor routing or the global routing gives it and as
/// the stage draws it.
struct Wire {
  /// The cell the wire is fed at, and the cell of its target port.
  std::uint32_t feed = NOWHERE;
  std::uint32_t target = NOWHERE;
  /// The cell of each crossing of its corridor.
  std::vector<std::uint32_t> crossings;
  /// The partitions its corridor names, one more than it has crossings.
  std::vector<PartitionLabel> partitions;
  /// The cells of the wire, from the feed to the target.
  std::vector<std::uint32_t> path;
  /// Whether Detail must draw this wire.
  bool required = false;
  /// Whether the wire has claimed the cells it needs to start routing.
  bool planned = false;
  /// Whether the wire has a path.
  bool drawn = false;
  /// Whether the wire belongs to the inner circuit. Both kinds of wire share
  /// one canvas and one clearance rule, so they share one list, and only the
  /// artifact keeps them apart.
  bool inner = false;
  /// The index of the wire in its list of the artifact.
  std::uint32_t slot = 0;
  /// How many of its cells lay within the clearance of another wire when the
  /// stage last placed it.
  std::uint32_t conflicts = 0;
};

/// One piece of a wire: the part of it that lies in one partition.
struct Piece {
  std::uint32_t wire = NOWHERE;
  std::uint32_t from = NOWHERE;
  std::uint32_t to = NOWHERE;
  PartitionLabel partition = LABEL_NONE;
  /// The index of the piece in its wire, so the pieces join in order.
  std::uint32_t position = 0;
};

/// The pieces of one partition as one order drew them, and the cells they
/// took.
struct Attempt {
  std::size_t drawn = 0;
  std::vector<std::vector<std::uint32_t>> paths;
  std::vector<std::uint32_t> taken;
};

/// What the first pass drew: the pieces of every wire, and the cells of each
/// piece. A piece without cells found no path.
struct Drawing {
  std::vector<std::vector<Piece>> pieces;
  std::vector<std::vector<std::vector<std::uint32_t>>> paths;

  /// Whether every piece of a wire has a path, which the wire needs to have
  /// one.
  [[nodiscard]] bool whole(const std::size_t wire) const {
    return !pieces[wire].empty() &&
           std::ranges::none_of(paths[wire],
                                [](const auto& path) { return path.empty(); });
  }

  [[nodiscard]] std::size_t undrawn() const {
    std::size_t left = 0;
    for (std::size_t wire = 0; wire < pieces.size(); ++wire) {
      left += (!pieces[wire].empty() && !whole(wire)) ? 1U : 0U;
    }
    return left;
  }
};

/// The cells that one search may enter.
///
/// The mask remembers the cells it set, so clearing it costs the cells it
/// marked and not the whole grid. A band holds a small part of the grid, and
/// the stage builds one band per search.
struct Mask {
  std::vector<std::uint8_t> cells;
  std::vector<std::uint32_t> marked;

  explicit Mask(const std::size_t size) : cells(size, 0) {}

  void clear() {
    for (const auto cell : marked) {
      cells[cell] = 0;
    }
    marked.clear();
  }

  void set(const std::uint32_t cell) {
    cells[cell] = 1;
    marked.push_back(cell);
  }

  void unset(const std::uint32_t cell) { cells[cell] = 0; }
};

/// A coordinate snapped to the nearest half cell.
///
/// The corridor routing stores its places in layout units, and reading one
/// back into cells can land a fraction of an ulp off. Every place of the
/// corridor routing is a whole or a half cell, and the half decides which
/// cells a crossing lies between.
[[nodiscard]] double onGrid(const double value) {
  return std::round(value * 2.0) / 2.0;
}

/// Whether a snapped coordinate names a cell edge and not a cell center.
[[nodiscard]] bool onEdge(const double value) {
  return std::abs(value - std::round(value)) < 0.25;
}

/// Checks that a plan was made on the detail grid that the stage rebuilds from
/// the chip and the configuration.
///
/// @throws std::invalid_argument When the two grids differ.
void requireSameGrid(const CapacityPlanT& plan,
                     const grid::GridMetrics& detail) {
  const auto* extent = plan.detail_grid.get();
  if (extent == nullptr) {
    throw std::invalid_argument("the capacity plan has no detail grid");
  }
  if (extent->width != detail.width || extent->height != detail.height ||
      extent->origin != detail.origin ||
      extent->cell_width != detail.cellWidth ||
      extent->cell_height != detail.cellHeight) {
    throw std::invalid_argument(std::format(
        "the capacity plan was made on a detail grid of {}x{} cells of "
        "{:.6g}x{:.6g} and the configuration asks for {}x{} cells of "
        "{:.6g}x{:.6g}",
        extent->width, extent->height, extent->cell_width, extent->cell_height,
        detail.width, detail.height, detail.cellWidth, detail.cellHeight));
  }
}

/// The partition of every cell of the detail grid.
///
/// No artifact stores the labels, but the first pass confines each search to
/// one partition. The outlines of the plan run along cell edges, and a hole is
/// a ring of its own, so the even-odd fill of the outlines gives the cells of
/// each partition back.
[[nodiscard]] std::vector<PartitionLabel>
labelsOf(const CapacityPlanT& plan, const grid::GridMetrics& detail,
         const Report& report) {
  const auto progress = [&report] {
    report.progress("rasterizing the partitions", "", 0, 0);
  };
  std::vector<grid::PartitionOutline> outlines;
  for (const auto& partition : plan.partitions) {
    for (const auto& ring : partition->outlines) {
      grid::PartitionOutline outline{
          .label = static_cast<PartitionLabel>(partition->label), .ring = {}};
      outline.ring.reserve(ring->vertices.size());
      for (const auto& vertex : ring->vertices) {
        outline.ring.push_back(grid::layoutToCorner(detail, vertex));
      }
      outlines.push_back(std::move(outline));
    }
  }
  return grid::rasterizePartitions(outlines, detail, progress);
}

/// The pixel A* detail router.
class PixelAStarRouter final : public IDetailRouter {
public:
  [[nodiscard]] DetailRoutingT
  run(const ChipT& chip, const CapacityPlanT& capacity,
      const GlobalRoutingT& global, const AssignmentT& assignment,
      const CorridorRoutingT& corridor, const ConfigT& config,
      const Report& report) const override {
    report.progress("rasterizing the chip", "", 0, 0);
    const auto scene = buildScene(chip, config);
    requireSameGrid(capacity, scene.detail);
    if (capacity.blocked_words != scene.blocked.words()) {
      throw std::invalid_argument(
          "the blocked cells of the capacity plan do not match the chip and "
          "configuration; run the capacity step again");
    }
    if (corridor.corridors.size() != assignment.connections.size()) {
      throw std::invalid_argument(
          std::format("the corridor routing holds {} and the assignment {}",
                      counted(corridor.corridors.size(), "corridor"),
                      counted(assignment.connections.size(), "connection")));
    }
    const auto tuning = tuningOf(config);

    Canvas canvas(scene.detail, scene.blocked,
                  labelsOf(capacity, scene.detail, report), tuning);

    // The seed: each corridor cut into one piece per partition, each piece
    // searched inside its partition, and the pieces joined. A corridor is a
    // coarse way and not a constraint, so the rerouting after the seed may
    // move a wire off it.
    const auto say = [&report](const std::string& line) {
      report.line(Detail::Steps, line);
    };

    auto wires = wiresOf(canvas, corridor);
    const auto ring = static_cast<std::uint32_t>(wires.size());
    say(std::format("grid {}{}{} cells{}{}{}{}", scene.detail.width, TIMES,
                    scene.detail.height, FIGURE_SEPARATOR,
                    counted(ring, "ring connection"), FIGURE_SEPARATOR,
                    counted(global.connections.size(), "inner connection")));
    report.progress("drawing inside the partitions", "", 0, 0);
    auto drawing = drawInsidePartitions(canvas, wires, tuning, report);
    say(std::format("inside the partitions{}{} without a path",
                    FIGURE_SEPARATOR, counted(drawing.undrawn(), "piece")));
    report.progress("moving the pieces in the way", "", 0, 0);
    placeWhatIsLeft(canvas, drawing, tuning);
    say(std::format("after moving wires{}{} without a path", FIGURE_SEPARATOR,
                    counted(drawing.undrawn(), "piece")));
    joinPieces(canvas, wires, drawing);
    report.progress("drawing whole wires", "", 0, 0);
    drawWhatIsLeftWhole(canvas, wires, tuning);
    report.progress("drawing the inner circuit", "", 0, 0);
    seedInnerCircuit(canvas, scene, global, wires);
    say(std::format("seed{}drawn {}/{}", FIGURE_SEPARATOR,
                    std::ranges::count_if(
                        wires, [](const Wire& wire) { return wire.drawn; }),
                    wires.size()));

    // The clearance: the stage routes wires again until no wire lies within
    // the clearance of another.
    const auto rule = ruleOf(config, scene.detail);
    canvas.armClearance(rule);
    say(std::format(
        "clearance {}{}{:.0f} layout units for a rule of {:.0f}",
        counted(rule.pixels, "cell"), FIGURE_SEPARATOR,
        rule.pixels * std::min(scene.detail.cellWidth, scene.detail.cellHeight),
        rule.spacing));
    // The band is a length in wire spacings, converted to cells on this grid.
    const auto band = static_cast<std::uint64_t>(tuning.corridorSpacings) *
                      grid::cellsFor(rule.spacing, scene.detail);
    const auto reach = static_cast<std::uint32_t>(std::clamp<std::uint64_t>(
        band, 1, std::max(canvas.width(), canvas.height())));
    const auto rounds =
        crossBoundaryRouting(canvas, wires, tuning, ring, reach, report);
    // A wire fails when it has no path or when its path comes within the
    // clearance of another wire, counted with every wire placed.
    const auto [unrouted, tooClose] = countFailures(canvas, wires);
    const auto drawn = std::ranges::count_if(
        wires, [](const Wire& wire) { return wire.drawn; });
    report.result({{.text = std::format("drawn {}/{}", drawn, wires.size())},
                   {.text = counted(rounds, "round")},
                   verdict("unrouted", unrouted),
                   verdict("too close", tooClose)},
                  unrouted + tooClose);

    return artifactOf(scene.detail, wires, ring, global.connections.size());
  }

private:
  /// The steering cost of a cell outside the corridor between the two
  /// neighbors of a wire in the ring.
  static constexpr std::uint8_t STEERING_PRICE = 10;

  /// How many levels deep a displaced wire may displace other wires when the
  /// stage draws a whole wire.
  ///
  /// One level lets a fan of wires through a narrow place shift by one
  /// position. A deeper chain costs one search per wire per level.
  static constexpr std::uint32_t DISPLACEMENT_CHAIN = 1;

  /// How many levels deep a wire that the rescue of the first pass moves may
  /// move other wires.
  ///
  /// Where a fan of wires enters a passage in the wrong order, the wire
  /// nearest the wall moves the wires beside it, the first of those moves the
  /// wires beside it, and so on. Two levels let a fan of four wires settle.
  static constexpr std::uint32_t RESCUE_CHAIN = 2;

  /// How far from a corridor crossing the stage looks for a free
  /// cell, in cells.
  ///
  /// A crossing on a crowded border may need a nearby free cell.
  static constexpr std::uint32_t PLACE_REACH = 4;

  /// The two cells that a place of the corridor routing lies between, or the
  /// one cell it lies on twice, as indices of the canvas.
  [[nodiscard]] static std::array<std::uint32_t, 2>
  cellsAt(const grid::GridMetrics& detail, const fbg::Point& place) {
    const auto cells = cellsOfPlace(detail, place);
    if (!cells.has_value()) {
      return {NOWHERE, NOWHERE};
    }
    const auto indexOf = [&detail](const fbg::DCoord& cell) {
      return (cell.y() * detail.width) + cell.x();
    };
    return {indexOf((*cells)[0]), indexOf((*cells)[1])};
  }

  /// The wires of the ring, one per corridor, in the order of the corridor
  /// routing.
  ///
  /// The stage claims every place of the corridor routing before any search
  /// runs: the feed, the target cell and every crossing. Otherwise one wire
  /// could run over the place where another has to start, end or cross, and
  /// no later search could undo that.
  [[nodiscard]] static std::vector<Wire>
  wiresOf(Canvas& canvas, const CorridorRoutingT& corridor) {
    const auto connections = corridor.corridors.size();
    std::vector<Wire> wires(connections);
    for (std::uint32_t index = 0; index < connections; ++index) {
      wires[index].slot = index;
      if (corridor.corridors[index] == nullptr) {
        throw std::invalid_argument("corridor entry is missing");
      }
      const auto& route = *corridor.corridors[index];
      if ((!route.partitions.empty() || !route.crossings.empty()) &&
          route.crossings.size() + 1 != route.partitions.size()) {
        throw std::invalid_argument(
            "corridor crossing count does not match its partitions");
      }
      auto& wire = wires[index];
      wire.required = !route.partitions.empty();
      if (!wire.required || route.source == nullptr ||
          route.target == nullptr) {
        continue;
      }
      wire.feed = cellsAt(canvas.metrics(), *route.source)[0];
      wire.target = cellsAt(canvas.metrics(), *route.target)[0];
      if (wire.feed == NOWHERE || wire.target == NOWHERE ||
          !canvas.standable(wire.feed) || !canvas.standable(wire.target)) {
        continue;
      }
      canvas.take(wire.feed, index);
      canvas.take(wire.target, index);
      wire.planned = true;
    }
    // Reserve every fixed terminal before choosing cells for the crossings.
    for (std::uint32_t index = 0; index < connections; ++index) {
      auto& wire = wires[index];
      if (!wire.planned) {
        continue;
      }
      const auto& route = *corridor.corridors[index];
      wire.partitions.reserve(route.partitions.size());
      for (const auto partition : route.partitions) {
        wire.partitions.push_back(static_cast<PartitionLabel>(partition));
      }
      wire.crossings.reserve(route.crossings.size());
      for (const auto& crossing : route.crossings) {
        wire.crossings.push_back(
            claim(canvas, cellsAt(canvas.metrics(), crossing), index));
      }
      wire.planned =
          std::ranges::find(wire.crossings, NOWHERE) == wire.crossings.end();
      if (!wire.planned) {
        const auto releaseClaim = [&canvas](const std::uint32_t cell) {
          if (cell != NOWHERE) {
            canvas.release(cell);
          }
        };
        releaseClaim(wire.feed);
        releaseClaim(wire.target);
        std::ranges::for_each(wire.crossings, releaseClaim);
      }
    }
    return wires;
  }

  /// Gives one place of a wire to that wire.
  ///
  /// @return The cell the wire got, or NOWHERE when no free cell lies within
  /// PLACE_REACH.
  [[nodiscard]] static std::uint32_t
  claim(Canvas& canvas, const std::array<std::uint32_t, 2>& candidates,
        const std::uint32_t wire) {
    for (const auto candidate : candidates) {
      if (candidate != NOWHERE && canvas.standable(candidate)) {
        canvas.take(candidate, wire);
        return candidate;
      }
    }
    for (const auto candidate : candidates) {
      if (candidate != NOWHERE && canvas.owner(candidate) == wire) {
        return candidate;
      }
    }
    const auto near = standableNear(canvas, candidates[0], PLACE_REACH);
    if (near != NOWHERE) {
      canvas.take(near, wire);
    }
    return near;
  }

  /// The nearest cell around a cell, in rings of growing Chebyshev radius,
  /// that is free and that no wire owns. The cell itself is not checked.
  [[nodiscard]] static std::uint32_t standableNear(const Canvas& canvas,
                                                   const std::uint32_t cell,
                                                   const std::uint32_t reach) {
    if (cell == NOWHERE) {
      return NOWHERE;
    }
    const auto width = static_cast<std::int64_t>(canvas.width());
    const auto height = static_cast<std::int64_t>(canvas.height());
    const auto centerX = canvas.columnOf(cell);
    const auto centerY = canvas.rowOf(cell);
    for (std::int64_t radius = 1; std::cmp_less_equal(radius, reach);
         ++radius) {
      for (auto dy = -radius; dy <= radius; ++dy) {
        for (auto dx = -radius; dx <= radius; ++dx) {
          if (std::max(std::abs(dx), std::abs(dy)) != radius) {
            continue;
          }
          const auto x = centerX + dx;
          const auto y = centerY + dy;
          if (x < 0 || y < 0 || x >= width || y >= height) {
            continue;
          }
          const auto near = static_cast<std::uint32_t>((y * width) + x);
          if (canvas.standable(near)) {
            return near;
          }
        }
      }
    }
    return NOWHERE;
  }

  /// The pieces one wire is cut into, in order.
  [[nodiscard]] static std::vector<Piece> piecesOf(const Wire& wire,
                                                   const std::uint32_t index) {
    std::vector<Piece> pieces;
    if (!wire.planned) {
      return pieces;
    }
    pieces.reserve(wire.partitions.size());
    auto from = wire.feed;
    for (std::size_t at = 0; at < wire.crossings.size(); ++at) {
      pieces.push_back({.wire = index,
                        .from = from,
                        .to = wire.crossings[at],
                        .partition = wire.partitions[at],
                        .position = static_cast<std::uint32_t>(at)});
      from = wire.crossings[at];
    }
    pieces.push_back(
        {.wire = index,
         .from = from,
         .to = wire.target,
         .partition = wire.partitions.back(),
         .position = static_cast<std::uint32_t>(wire.crossings.size())});
    return pieces;
  }

  /// Draws the pieces of one partition in the order given.
  ///
  /// A piece takes only the cells between its ends, so the pieces before and
  /// after a crossing share the cell of the crossing.
  [[nodiscard]] static Attempt attempt(Canvas& canvas,
                                       const std::vector<Piece>& pieces) {
    Attempt made;
    made.paths.resize(pieces.size());
    for (std::size_t at = 0; at < pieces.size(); ++at) {
      const auto& piece = pieces[at];
      auto found = canvas.search({.start = piece.from,
                                  .goal = piece.to,
                                  .wire = piece.wire,
                                  .partition = piece.partition});
      if (!found.has_value()) {
        continue;
      }
      for (std::size_t cell = 1; cell + 1 < found->size(); ++cell) {
        canvas.take((*found)[cell], piece.wire);
        made.taken.push_back((*found)[cell]);
      }
      made.paths[at] = std::move(*found);
      ++made.drawn;
    }
    return made;
  }

  /// Releases the cells an attempt took.
  static void undo(Canvas& canvas, const Attempt& made) {
    for (const auto cell : made.taken) {
      canvas.release(cell);
    }
  }

  /// Draws every piece of every wire, one partition at a time.
  ///
  /// Each piece is searched inside its partition alone. The first piece drawn
  /// can take the room the next one needs, so the pieces of one partition are
  /// tried in several orders, and the order that draws the most of them is
  /// kept. A partition whose pieces all draw in the first order takes one
  /// attempt.
  [[nodiscard]] static Drawing
  drawInsidePartitions(Canvas& canvas, const std::vector<Wire>& wires,
                       const Tuning& tuning, const Report& report) {
    Drawing drawing;
    drawing.pieces.resize(wires.size());
    drawing.paths.resize(wires.size());
    std::map<PartitionLabel, std::vector<Piece>> byPartition;
    for (std::uint32_t index = 0; index < wires.size(); ++index) {
      drawing.pieces[index] = piecesOf(wires[index], index);
      drawing.paths[index].resize(drawing.pieces[index].size());
      for (const auto& piece : drawing.pieces[index]) {
        byPartition[piece.partition].push_back(piece);
      }
    }

    std::size_t done = 0;
    for (auto& [partition, pieces] : byPartition) {
      report.progress("drawing inside the partitions", "", done++,
                      byPartition.size());
      // A total order, so that the first attempt is the same on every run.
      // The permutations after it order the pieces by their start alone.
      std::ranges::sort(pieces, [](const Piece& a, const Piece& b) {
        return std::tie(a.from, a.wire, a.position) <
               std::tie(b.from, b.wire, b.position);
      });

      auto order = pieces;
      auto best = attempt(canvas, order);
      auto bestOrder = order;
      for (std::uint32_t variant = 1; variant < tuning.orderings; ++variant) {
        if (best.drawn == order.size()) {
          break;
        }
        if (!std::ranges::next_permutation(order, [](const Piece& a,
                                                     const Piece& b) {
               return a.from < b.from;
             }).found) {
          break;
        }
        undo(canvas, best);
        auto made = attempt(canvas, order);
        if (made.drawn > best.drawn) {
          best = std::move(made);
          bestOrder = order;
          continue;
        }
        // The order just tried is no better, so the canvas gets the best
        // order back.
        undo(canvas, made);
        for (std::size_t at = 0; at < bestOrder.size(); ++at) {
          const auto& path = best.paths[at];
          for (std::size_t cell = 1; cell + 1 < path.size(); ++cell) {
            canvas.take(path[cell], bestOrder[at].wire);
          }
        }
      }
      for (std::size_t at = 0; at < bestOrder.size(); ++at) {
        drawing.paths[bestOrder[at].wire][bestOrder[at].position] =
            std::move(best.paths[at]);
      }
    }
    return drawing;
  }

  /// Draws the pieces of one wire that have no path yet.
  ///
  /// @return Whether every piece of the wire has a path afterwards.
  [[nodiscard]] static bool draw(Canvas& canvas, Drawing& drawing,
                                 const std::uint32_t wire) {
    for (std::size_t at = 0; at < drawing.pieces[wire].size(); ++at) {
      if (!drawing.paths[wire][at].empty()) {
        continue;
      }
      const auto& piece = drawing.pieces[wire][at];
      auto found = canvas.search({.start = piece.from,
                                  .goal = piece.to,
                                  .wire = piece.wire,
                                  .partition = piece.partition});
      if (!found.has_value()) {
        continue;
      }
      for (std::size_t cell = 1; cell + 1 < found->size(); ++cell) {
        canvas.take((*found)[cell], wire);
      }
      drawing.paths[wire][at] = std::move(*found);
    }
    return drawing.whole(wire);
  }

  /// Takes the pieces of one wire off the canvas. The wire keeps its places:
  /// the feed, the target cell and the crossings.
  static void giveUp(Canvas& canvas, Drawing& drawing,
                     const std::uint32_t wire) {
    for (auto& path : drawing.paths[wire]) {
      for (std::size_t cell = 1; cell + 1 < path.size(); ++cell) {
        canvas.release(path[cell]);
      }
      path.clear();
    }
  }

  /// Puts every piece back as an earlier drawing had it.
  static void restore(Canvas& canvas, Drawing& drawing, const Drawing& before) {
    for (std::uint32_t wire = 0; wire < drawing.paths.size(); ++wire) {
      giveUp(canvas, drawing, wire);
    }
    drawing.paths = before.paths;
    for (std::uint32_t wire = 0; wire < drawing.paths.size(); ++wire) {
      for (const auto& path : drawing.paths[wire]) {
        for (std::size_t cell = 1; cell + 1 < path.size(); ++cell) {
          canvas.take(path[cell], wire);
        }
      }
    }
  }

  /// The wires in the way of the pieces of a wire that have no path.
  ///
  /// The search that may cross other wires at a high cost names them. That
  /// path finds the few wires actually in the way and not every wire of the
  /// partition.
  [[nodiscard]] static std::vector<std::uint32_t>
  blockersOf(Canvas& canvas, const Drawing& drawing, const std::uint32_t wire) {
    // Scale the crossing penalty with the grid size to favor free detours.
    const auto price = 14U * (canvas.width() + canvas.height());
    std::vector<std::uint32_t> owners;
    for (std::size_t at = 0; at < drawing.pieces[wire].size(); ++at) {
      if (!drawing.paths[wire][at].empty()) {
        continue;
      }
      const auto& piece = drawing.pieces[wire][at];
      const auto found = canvas.search({.start = piece.from,
                                        .goal = piece.to,
                                        .wire = piece.wire,
                                        .partition = piece.partition,
                                        .displacement = price});
      if (!found.has_value()) {
        continue;
      }
      for (std::size_t cell = 0; cell < found->size(); ++cell) {
        const auto owner = canvas.owner((*found)[cell]);
        if (owner != NO_OWNER && owner != wire) {
          owners.push_back(owner);
        }
        // A diagonal step between two cells of another wire crosses that
        // wire without touching its cells.
        if (cell == 0) {
          continue;
        }
        const auto crossed =
            canvas.crossedBy((*found)[cell - 1], (*found)[cell]);
        if (crossed != NO_OWNER && crossed != wire) {
          owners.push_back(crossed);
        }
      }
    }
    std::ranges::sort(owners);
    const auto duplicates = std::ranges::unique(owners);
    owners.erase(duplicates.begin(), duplicates.end());
    return owners;
  }

  /// Draws the pieces that the first pass left without a path, by moving the
  /// wires in their way.
  ///
  /// The stage keeps a swap only when it leaves fewer pieces without a path,
  /// so a swap that trades one piece for another is undone. The wires in the
  /// way are found again after each swap, because they change as wires move.
  static void placeWhatIsLeft(Canvas& canvas, Drawing& drawing,
                              const Tuning& tuning) {
    for (std::uint32_t wire = 0; wire < drawing.pieces.size(); ++wire) {
      if (drawing.pieces[wire].empty() || drawing.whole(wire)) {
        continue;
      }
      for (std::uint32_t attempt = 0; attempt < tuning.rounds; ++attempt) {
        const auto before = drawing;
        const auto left = drawing.undrawn();
        std::vector<std::uint32_t> busy;
        if (!swapIn(canvas, drawing, wire, RESCUE_CHAIN, busy)) {
          break;
        }
        // Any wire may take the room that the swap freed.
        for (std::uint32_t other = 0; other < drawing.pieces.size(); ++other) {
          if (!drawing.pieces[other].empty() && !drawing.whole(other)) {
            static_cast<void>(draw(canvas, drawing, other));
          }
        }
        if (drawing.undrawn() >= left) {
          restore(canvas, drawing, before);
          break;
        }
        if (drawing.whole(wire)) {
          break;
        }
      }
    }
  }

  /// Takes the wires in the way of one wire off the canvas, draws the wire,
  /// and draws them again.
  ///
  /// Each wire that is drawn again takes the shortest path it finds, which
  /// can be the room the next one needs. A wire that finds no path may
  /// therefore move the wires in its own way, `chain` levels deep. The wires
  /// that are being placed are busy, because moving one of them would undo
  /// the swap that is being tried.
  ///
  /// @return Whether any wire was in the way.
  [[nodiscard]] static bool swapIn(Canvas& canvas, Drawing& drawing,
                                   const std::uint32_t wire,
                                   const std::uint32_t chain,
                                   std::vector<std::uint32_t>& busy) {
    // The busy wires stay, so the search that names the wires in the way may
    // not cross them either. With the busy wires pinned, it names only wires
    // that can move.
    for (const auto other : busy) {
      fence(canvas, drawing, other, true);
    }
    auto blockers = blockersOf(canvas, drawing, wire);
    for (const auto other : busy) {
      fence(canvas, drawing, other, false);
    }
    std::erase_if(blockers, [&busy](const std::uint32_t blocker) {
      return std::ranges::find(busy, blocker) != busy.end();
    });
    if (blockers.empty()) {
      return false;
    }
    busy.push_back(wire);
    for (const auto blocker : blockers) {
      giveUp(canvas, drawing, blocker);
    }
    static_cast<void>(draw(canvas, drawing, wire));
    for (const auto blocker : blockers) {
      if (draw(canvas, drawing, blocker) || chain == 0) {
        continue;
      }
      static_cast<void>(swapIn(canvas, drawing, blocker, chain - 1, busy));
    }
    busy.pop_back();
    return true;
  }

  /// Pins the cells of the pieces of one wire, or unpins them.
  static void fence(Canvas& canvas, const Drawing& drawing,
                    const std::uint32_t wire, const bool on) {
    for (const auto& path : drawing.paths[wire]) {
      for (const auto cell : path) {
        canvas.pin(cell, on);
      }
    }
  }

  /// Joins the pieces of every wire into one path, in the order of its
  /// corridor.
  ///
  /// The corridor says which piece belongs to which wire, so joining is a
  /// concatenation and not a search. A wire with a missing piece has no path.
  /// It releases its cells and its crossings, so that the later passes may
  /// use them.
  static void joinPieces(Canvas& canvas, std::vector<Wire>& wires,
                         Drawing& drawing) {
    for (std::uint32_t index = 0; index < wires.size(); ++index) {
      auto& wire = wires[index];
      wire.path.clear();
      wire.drawn = wire.planned && drawing.whole(index);
      if (!wire.drawn) {
        // A wire that is searched again from end to end may cross a border
        // anywhere, so it releases its crossings. It keeps its two ends.
        giveUp(canvas, drawing, index);
        for (const auto crossing : wire.crossings) {
          if (crossing != NOWHERE && crossing != wire.feed &&
              crossing != wire.target && canvas.owner(crossing) == index) {
            canvas.release(crossing);
          }
        }
        continue;
      }
      for (std::size_t at = 0; at < drawing.paths[index].size(); ++at) {
        const auto& piece = drawing.paths[index][at];
        // The piece before ends where this one starts, so the path holds the
        // shared crossing once.
        wire.path.insert(wire.path.end(),
                         std::next(piece.begin(), at == 0 ? 0 : 1),
                         piece.end());
      }
    }
  }

  /// Draws each wire that has no path in one search from end to end.
  ///
  /// A piece has to start and end at the crossings of its corridor. Where two
  /// wires have to pass one narrow place, the first piece drawn can take it
  /// in either order, because neither piece may bend around the ends of the
  /// other. A wire without a path is therefore searched again from its feed to
  /// its target, over the free space and around the wires already drawn,
  /// without its corridor. Where that search fails too, the stage moves the
  /// wires in its way and draws them again after it.
  static void drawWhatIsLeftWhole(Canvas& canvas, std::vector<Wire>& wires,
                                  const Tuning& tuning) {
    for (std::uint32_t index = 0; index < wires.size(); ++index) {
      std::vector<std::uint32_t> busy;
      static_cast<void>(
          place(canvas, wires, index, tuning, DISPLACEMENT_CHAIN, busy));
    }
  }

  /// Draws one wire that has no path, and moves the wires in its way where it
  /// has to.
  ///
  /// The wires in the way are found again after each one is moved, because
  /// they change as wires move. The stage keeps a swap only when it leaves
  /// fewer wires without a path, so a swap that trades one wire for another
  /// is undone, and the next attempt moves one more wire.
  ///
  /// A moved wire may move other wires in turn, `chain` levels deep. Where a
  /// fan of wires takes every path through a narrow place, no single swap
  /// helps, because the moved wire has no other path either. The whole fan
  /// has to shift by one position, which a chain of moves does.
  ///
  /// @return Whether this call drew the wire.
  [[nodiscard]] static bool place(Canvas& canvas, std::vector<Wire>& wires,
                                  const std::uint32_t index,
                                  const Tuning& tuning,
                                  const std::uint32_t chain,
                                  std::vector<std::uint32_t>& busy) {
    if (wires[index].drawn || !wires[index].planned) {
      return false;
    }
    if (drawWhole(canvas, wires, index)) {
      return true;
    }
    const auto left = undrawn(wires);
    std::vector<std::vector<std::uint32_t>> saved;
    std::vector<std::uint32_t> out;
    busy.push_back(index);

    for (std::uint32_t attempt = 0; attempt < tuning.maxRelaxation; ++attempt) {
      const auto blockers = inTheWayOf(canvas, wires, index);
      std::optional<std::uint32_t> next;
      for (const auto blocker : blockers) {
        // A wire that is being placed is busy. Moving it would undo the swap
        // that is being tried, and the two wires would move each other
        // without end.
        if (wires[blocker].drawn &&
            std::ranges::find(out, blocker) == out.end() &&
            std::ranges::find(busy, blocker) == busy.end()) {
          next = blocker;
          break;
        }
      }
      if (!next.has_value()) {
        break;
      }
      if (out.empty()) {
        saved.reserve(wires.size());
        for (const auto& wire : wires) {
          saved.push_back(wire.path);
        }
      }
      out.push_back(*next);
      vacate(canvas, wires[*next], *next);
      if (offer(canvas, wires, index, out, left, tuning, chain, busy)) {
        busy.pop_back();
        return true;
      }
    }

    // Restore the paths before the first displacement, including nested swaps.
    if (!out.empty()) {
      for (std::uint32_t i = 0; i < wires.size(); ++i) {
        if (wires[i].planned && wires[i].path != saved[i]) {
          vacate(canvas, wires[i], i);
        }
      }
      for (std::uint32_t i = 0; i < wires.size(); ++i) {
        if (!saved[i].empty() && !wires[i].drawn) {
          wires[i].path = std::move(saved[i]);
          wires[i].drawn = true;
          for (const auto cell : wires[i].path) {
            canvas.take(cell, i);
          }
        }
      }
    }
    busy.pop_back();
    return false;
  }

  /// Draws one wire in the room that the moved wires left, and then draws the
  /// moved wires again. Each of them may move other wires while the chain
  /// lasts.
  ///
  /// @return Whether fewer wires than `left` are without a path afterwards.
  /// When not, the wire comes off the canvas again, so that the next attempt
  /// finds the wires in its way and does not follow the path it just took.
  [[nodiscard]] static bool offer(Canvas& canvas, std::vector<Wire>& wires,
                                  const std::uint32_t index,
                                  const std::vector<std::uint32_t>& out,
                                  const std::size_t left, const Tuning& tuning,
                                  const std::uint32_t chain,
                                  std::vector<std::uint32_t>& busy) {
    if (!drawWhole(canvas, wires, index)) {
      return false;
    }
    for (const auto other : out) {
      if (drawWhole(canvas, wires, other) || chain == 0) {
        continue;
      }
      static_cast<void>(place(canvas, wires, other, tuning, chain - 1, busy));
    }
    if (undrawn(wires) < left) {
      return true;
    }
    vacate(canvas, wires[index], index);
    return false;
  }

  /// How many planned wires have no path.
  [[nodiscard]] static std::size_t undrawn(const std::vector<Wire>& wires) {
    return static_cast<std::size_t>(std::ranges::count_if(
        wires, [](const Wire& wire) { return wire.planned && !wire.drawn; }));
  }

  /// Searches one wire from its feed to its target, over the free space and
  /// around every other wire.
  [[nodiscard]] static bool drawWhole(Canvas& canvas, std::vector<Wire>& wires,
                                      const std::uint32_t index) {
    auto& wire = wires[index];
    if (wire.drawn || !wire.planned) {
      return wire.drawn;
    }
    auto found = canvas.search({.start = wire.feed,
                                .goal = wire.target,
                                .wire = index,
                                .avoidObstacles = true});
    if (!found.has_value()) {
      return false;
    }
    wire.path = std::move(*found);
    wire.drawn = true;
    for (const auto cell : wire.path) {
      canvas.take(cell, index);
    }
    return true;
  }

  /// Takes a wire off the canvas. The wire keeps its feed and its target
  /// cell.
  static void vacate(Canvas& canvas, Wire& wire, const std::uint32_t index) {
    for (const auto cell : wire.path) {
      canvas.release(cell);
    }
    wire.path.clear();
    wire.drawn = false;
    for (const auto place : {wire.feed, wire.target}) {
      if (place != NOWHERE) {
        canvas.take(place, index);
      }
    }
  }

  /// The wires in the way of a wire that has no path, named by the search that
  /// may cross other wires at a high cost.
  [[nodiscard]] static std::vector<std::uint32_t>
  inTheWayOf(Canvas& canvas, const std::vector<Wire>& wires,
             const std::uint32_t index) {
    const auto price = 14U * (canvas.width() + canvas.height());
    const auto found = canvas.search({.start = wires[index].feed,
                                      .goal = wires[index].target,
                                      .wire = index,
                                      .displacement = price});
    std::vector<std::uint32_t> owners;
    if (!found.has_value()) {
      return owners;
    }
    for (std::size_t at = 0; at < found->size(); ++at) {
      const auto owner = canvas.owner((*found)[at]);
      if (owner != NO_OWNER && owner != index) {
        owners.push_back(owner);
      }
      if (at == 0) {
        continue;
      }
      const auto crossed = canvas.crossedBy((*found)[at - 1], (*found)[at]);
      if (crossed != NO_OWNER && crossed != index) {
        owners.push_back(crossed);
      }
    }
    std::ranges::sort(owners);
    const auto duplicates = std::ranges::unique(owners);
    owners.erase(duplicates.begin(), duplicates.end());
    return owners;
  }

  /// Counts required wires without a path and drawn wires with a clearance
  /// conflict.
  ///
  /// The clearance of a wire covers its own cells, so the stage removes the
  /// guard of the wire before it counts and adds it again afterwards. The
  /// count then equals that of a check on the finished paths.
  ///
  /// @return The counts of required wires without a path and wires too
  /// close to another wire.
  [[nodiscard]] static std::pair<std::size_t, std::size_t>
  countFailures(Canvas& canvas, const std::vector<Wire>& wires) {
    std::size_t unrouted = 0;
    std::size_t tooClose = 0;
    // Undrawn wires reserve their endpoints during routing only.
    for (const auto& wire : wires) {
      if (wire.planned && !wire.drawn) {
        makeWay(canvas, wire);
      }
    }
    for (const auto& wire : wires) {
      if (!wire.required) {
        continue;
      }
      if (!wire.drawn || wire.path.empty()) {
        ++unrouted;
        continue;
      }
      for (const auto cell : wire.path) {
        canvas.unguard(cell);
      }
      const auto conflicts =
          std::ranges::any_of(wire.path, [&canvas](const auto cell) {
            return canvas.guardOf(cell) != 0;
          });
      for (const auto cell : wire.path) {
        canvas.guard(cell);
      }
      tooClose += conflicts ? 1U : 0U;
    }
    return {unrouted, tooClose};
  }

  /// Routes every wire again across the partition borders and keeps the
  /// clearance to every other wire.
  ///
  /// Each round sweeps the wires, forward and backward in turn, and leaves
  /// alone every wire that already keeps the clearance. A wire that does not
  /// is searched in a band around its path. Where that fails, it releases its
  /// neighbors one at a time and searches the same band with a steering cost.
  ///
  /// The clearance holds against every wire, not only against the two
  /// neighbors in the ring. Cutting the clearance of every wire out of the
  /// band of each search would cost a pass over all wires per search, so the
  /// canvas carries a field instead: every placed wire guards the cells
  /// within its clearance, and no search enters a guarded cell. Releasing a
  /// wire removes its guard but leaves its cells in place, so no two wires
  /// share a cell during the sweep.
  ///
  /// @return How many rounds ran.
  static std::uint32_t
  crossBoundaryRouting(Canvas& canvas, std::vector<Wire>& wires,
                       const Tuning& tuning, const std::uint32_t ring,
                       const std::uint32_t reach, const Report& report) {
    Mask mask(canvas.cells());
    std::vector<std::uint8_t> steering(canvas.cells(), 0);
    std::vector<std::uint8_t> settled(wires.size(), 0);

    std::uint32_t rounds = 0;
    for (std::uint32_t round = 0; round < tuning.rounds; ++round) {
      ++rounds;
      const auto forward = (round % 2) == 0;
      const auto name = std::format("round {} {}", round + 1,
                                    forward ? "forward" : "backward");
      std::size_t failed = 0;
      for (std::size_t step = 0; step < wires.size(); ++step) {
        report.progress(name, "", step, wires.size());
        const auto index = static_cast<std::uint32_t>(
            forward ? step : wires.size() - 1 - step);
        if (settled[index] != 0 || !wires[index].planned) {
          continue;
        }
        if (reroute(canvas, wires, index, tuning, mask, steering, settled,
                    forward, ring, reach)) {
          settled[index] = 1;
        } else {
          ++failed;
        }
      }
      report.progress(name, "", wires.size(), wires.size());
      report.entry(
          Detail::Steps, name,
          {{.text = std::format("to settle {}/{}", failed, wires.size())}});
      if (failed == 0) {
        break;
      }
    }
    return rounds;
  }

  /// Routes one wire again: first in the band around its path, then with its
  /// neighbors released.
  ///
  /// @return Whether the wire found a path that keeps the clearance. On
  /// a search without a path the wire keeps its previous path. A path found
  /// after releasing neighbors may still need another clearance sweep.
  [[nodiscard]] static bool
  reroute(Canvas& canvas, std::vector<Wire>& wires, const std::uint32_t index,
          const Tuning& tuning, Mask& mask, std::vector<std::uint8_t>& steering,
          std::vector<std::uint8_t>& settled, const bool forward,
          const std::uint32_t ring, const std::uint32_t reach) {
    const auto original = wires[index].path;
    lift(canvas, wires, index);
    makeWay(canvas, wires[index]);

    const auto& wire = wires[index];
    Query query{.start = wire.feed,
                .goal = wire.target,
                .wire = index,
                .avoidObstacles = true};
    // All attempts use the same band. A wire without a path has no band.
    if (!original.empty() && reach != 0) {
      buildBand(canvas, original, reach, mask);
      query.mask = &mask.cells;
    }
    auto found = canvas.search(query);

    // Relaxation: the wires ahead of this one in the sweep are released first,
    // one more each time. Where that is not enough, the wires behind it follow,
    // one at a time.
    std::vector<std::uint32_t> ripped;
    if (!found.has_value() && tuning.maxRelaxation != 0 && wires.size() > 1) {
      steer(canvas, wires, index, original, ring, steering);
      query.steering = &steering;
      const auto count = static_cast<std::int64_t>(wires.size());
      const auto ahead = forward ? 1 : -1;
      const auto limit = static_cast<std::int64_t>(tuning.maxRelaxation);
      const auto aheadLimit = std::min(limit, count - 1);
      // Both walks wrap around the wire list. Stop before they meet.
      const auto behindLimit = std::min(limit, count - 1 - aheadLimit);
      const auto letOneGo = [&](const std::int64_t away) {
        const auto other = static_cast<std::uint32_t>(
            (((static_cast<std::int64_t>(index) + away) % count) + count) %
            count);
        // Releasing a wire removes its guard. Its cells stay until it is drawn
        // again, so this search cannot run over them.
        settled[other] = 0;
        ripped.push_back(other);
        letGo(canvas, wires[other]);
        found = canvas.search(query);
      };
      for (std::int64_t step = 1; step <= aheadLimit && !found.has_value();
           ++step) {
        letOneGo(ahead * step);
      }
      for (std::int64_t step = 1; step <= behindLimit && !found.has_value();
           ++step) {
        letOneGo(-ahead * step);
      }
    }

    // The released wires guard their cells again, so the count below checks
    // the new path against every wire.
    for (const auto other : ripped) {
      takeUpAgain(canvas, wires[other]);
    }
    // The feed and the target of the wire are still unguarded here. The count
    // below checks the cells of the wire against the guards, and a guard on
    // its own ends would count every wire as too close to itself.
    settle(canvas, wires, index, found.has_value() ? *found : original);
    giveWayBack(canvas, wires[index]);
    return found.has_value() && wires[index].conflicts == 0;
  }

  /// Removes the guard of a wire and leaves its cells in place.
  static void letGo(Canvas& canvas, const Wire& wire) {
    for (const auto cell : wire.path) {
      canvas.unguard(cell);
    }
  }

  /// Guards the cells of a wire again.
  static void takeUpAgain(Canvas& canvas, const Wire& wire) {
    for (const auto cell : wire.path) {
      canvas.guard(cell);
    }
  }

  /// Takes one wire off the canvas.
  ///
  /// The wire keeps its feed and its target cell, and both keep their guard.
  /// Otherwise another wire could settle within the clearance of a place that
  /// neither wire can move, and no later round could repair that.
  static void lift(Canvas& canvas, std::vector<Wire>& wires,
                   const std::uint32_t index) {
    auto& wire = wires[index];
    for (const auto cell : wire.path) {
      canvas.release(cell);
    }
    wire.path.clear();
    wire.drawn = false;
    wire.conflicts = 0;
    for (const auto place : {wire.feed, wire.target}) {
      if (place != NOWHERE) {
        canvas.occupy(place, index);
      }
    }
  }

  /// Removes the guard of the feed and the target of one wire for its own
  /// search, so that the guard does not keep the wire away from its own ends.
  static void makeWay(Canvas& canvas, const Wire& wire) {
    for (const auto place : {wire.feed, wire.target}) {
      if (place != NOWHERE) {
        canvas.unguard(place);
      }
    }
  }

  /// Guards the feed and the target of one wire again.
  static void giveWayBack(Canvas& canvas, const Wire& wire) {
    for (const auto place : {wire.feed, wire.target}) {
      if (place != NOWHERE) {
        canvas.guard(place);
      }
    }
  }

  /// Places one wire on a path and counts its cells that lie within the
  /// clearance of another wire.
  static void settle(Canvas& canvas, std::vector<Wire>& wires,
                     const std::uint32_t index,
                     const std::vector<std::uint32_t>& path) {
    auto& wire = wires[index];
    wire.conflicts = 0;
    for (const auto cell : path) {
      wire.conflicts += canvas.guardOf(cell) != 0 ? 1U : 0U;
    }
    wire.path = path;
    wire.drawn = !path.empty();
    for (const auto cell : wire.path) {
      canvas.occupy(cell, index);
    }
  }

  /// The steering cost that the relaxation searches under.
  ///
  /// Every cell outside the closed ring of the two ends of the wire and the
  /// paths of its two neighbors in the ring costs, so the search stays between
  /// the neighbors. Every cell within the clearance of a neighbor costs too,
  /// so the wire does not settle against one. Both are costs and block
  /// nothing, so a wire that has to leave the ring still can.
  static void steer(const Canvas& canvas, const std::vector<Wire>& wires,
                    const std::uint32_t index,
                    const std::vector<std::uint32_t>& own,
                    const std::uint32_t ring,
                    std::vector<std::uint8_t>& steering) {
    std::ranges::fill(steering, STEERING_PRICE);
    if (own.empty()) {
      return;
    }
    if (index >= ring || ring < 3) {
      // A wire of the inner circuit has no neighbors in the ring, so nothing
      // steers it.
      std::ranges::fill(steering, 0);
      return;
    }
    const auto count = static_cast<std::int64_t>(ring);
    const auto at = [count](const std::int64_t where) {
      return static_cast<std::uint32_t>(((where % count) + count) % count);
    };
    const auto& before = wires[at(static_cast<std::int64_t>(index) - 1)].path;
    const auto& after = wires[at(static_cast<std::int64_t>(index) + 1)].path;
    std::vector<std::pair<double, double>> corridor;
    corridor.reserve(before.size() + after.size() + 2);
    const auto corner = [&canvas, &corridor](const std::uint32_t cell) {
      corridor.emplace_back(static_cast<double>(canvas.columnOf(cell)) + 0.5,
                            static_cast<double>(canvas.rowOf(cell)) + 0.5);
    };
    corner(own.front());
    for (const auto cell : before) {
      corner(cell);
    }
    corner(own.back());
    for (const auto cell : std::ranges::reverse_view(after)) {
      corner(cell);
    }
    if (corridor.size() >= 3) {
      fillRing(canvas, corridor, steering);
    }
    // Inside the ring or not, a wire keeps off its neighbors.
    for (const auto* path : {&before, &after}) {
      for (const auto cell : *path) {
        canvas.forClearanceRows(cell, [&steering](const std::size_t first,
                                                  const std::size_t length) {
          std::ranges::fill(std::span{steering}.subspan(first, length),
                            STEERING_PRICE);
        });
      }
    }
  }

  /// Sets the steering cost of every cell that the closed ring encloses to
  /// zero, by the even-odd rule.
  ///
  /// The fill runs one row at a time: the edges that cross the row give the
  /// crossings, and the spans between consecutive crossings lie inside. This
  /// costs the edges once per row and not once per cell.
  static void fillRing(const Canvas& canvas,
                       const std::vector<std::pair<double, double>>& ring,
                       std::vector<std::uint8_t>& steering) {
    auto minY = ring.front().second;
    auto maxY = ring.front().second;
    for (const auto& [x, y] : ring) {
      minY = std::min(minY, y);
      maxY = std::max(maxY, y);
    }
    const auto width = static_cast<std::int64_t>(canvas.width());
    const auto first =
        std::max<std::int64_t>(0, static_cast<std::int64_t>(std::floor(minY)));
    const auto last =
        std::min<std::int64_t>(static_cast<std::int64_t>(canvas.height()) - 1,
                               static_cast<std::int64_t>(std::ceil(maxY)));
    std::vector<double> crossings;
    for (auto row = first; row <= last; ++row) {
      const auto y = static_cast<double>(row) + 0.5;
      crossings.clear();
      for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
        const auto [xi, yi] = ring[i];
        const auto [xj, yj] = ring[j];
        if ((yi > y) != (yj > y)) {
          crossings.push_back((((xj - xi) * (y - yi)) / (yj - yi)) + xi);
        }
      }
      std::ranges::sort(crossings);
      for (std::size_t at = 0; at + 1 < crossings.size(); at += 2) {
        const auto from = std::max<std::int64_t>(
            0, static_cast<std::int64_t>(std::ceil(crossings[at] - 0.5)));
        const auto to = std::min<std::int64_t>(
            width - 1,
            static_cast<std::int64_t>(std::ceil(crossings[at + 1] - 0.5)) - 1);
        for (auto column = from; column <= to; ++column) {
          steering[static_cast<std::size_t>((row * width) + column)] = 0;
        }
      }
    }
  }

  /// The cells one wire may move onto: a band `reach` cells to either side of
  /// its path.
  ///
  /// The band does not hold the clearance. The canvas guards the clearance of
  /// every wire.
  static void buildBand(const Canvas& canvas,
                        const std::vector<std::uint32_t>& along,
                        const std::uint32_t reach, Mask& mask) {
    mask.clear();
    const auto width = static_cast<std::int64_t>(canvas.width());
    const auto height = static_cast<std::int64_t>(canvas.height());
    const auto span = static_cast<std::int64_t>(reach);

    std::int64_t minX = width;
    std::int64_t minY = height;
    std::int64_t maxX = -1;
    std::int64_t maxY = -1;
    for (const auto cell : along) {
      minX = std::min(minX, canvas.columnOf(cell));
      minY = std::min(minY, canvas.rowOf(cell));
      maxX = std::max(maxX, canvas.columnOf(cell));
      maxY = std::max(maxY, canvas.rowOf(cell));
    }
    if (maxX < 0) {
      return;
    }
    minX = std::max<std::int64_t>(0, minX - span);
    minY = std::max<std::int64_t>(0, minY - span);
    maxX = std::min<std::int64_t>(width - 1, maxX + span);
    maxY = std::min<std::int64_t>(height - 1, maxY + span);

    // One interval per row. The band is the union of a Chebyshev box around
    // every cell of the path, so marking the two ends of each row of a box and
    // scanning the window once costs one window width per row and not one
    // square per cell of the path.
    const auto stride = maxX - minX + 2;
    std::vector<std::int32_t> edges(
        static_cast<std::size_t>(stride * (maxY - minY + 1)), 0);
    for (const auto cell : along) {
      const auto x = canvas.columnOf(cell);
      const auto y = canvas.rowOf(cell);
      const auto left = std::max(minX, x - span) - minX;
      const auto right = std::min(maxX, x + span) - minX;
      for (auto row = std::max(minY, y - span); row <= std::min(maxY, y + span);
           ++row) {
        const auto base = (row - minY) * stride;
        ++edges[static_cast<std::size_t>(base + left)];
        --edges[static_cast<std::size_t>(base + right + 1)];
      }
    }

    for (auto y = minY; y <= maxY; ++y) {
      std::int32_t inside = 0;
      const auto base = (y - minY) * stride;
      for (auto x = minX; x <= maxX; ++x) {
        inside += edges[static_cast<std::size_t>(base + (x - minX))];
        if (inside <= 0) {
          continue;
        }
        mask.set(static_cast<std::uint32_t>((y * width) + x));
      }
    }
  }

  /// Seeds the connections of the inner circuit.
  ///
  /// The Corridor stage does not route the inner circuit, so these wires have
  /// no corridor. Each is searched over the free space that the ring leaves,
  /// and joins the wires of the ring in one list, under one clearance rule.
  static void seedInnerCircuit(Canvas& canvas, const CapacityScene& scene,
                               const GlobalRoutingT& global,
                               std::vector<Wire>& wires) {
    std::map<std::uint32_t, std::uint32_t> cellOfPort;
    for (std::size_t at = 0; at < scene.targetPort.size(); ++at) {
      cellOfPort.emplace(scene.targetPort[at],
                         static_cast<std::uint32_t>(scene.targetCell[at]));
    }

    for (std::uint32_t index = 0; index < global.connections.size(); ++index) {
      if (global.connections[index] == nullptr) {
        throw std::invalid_argument("inner connection is missing");
      }
      const auto& connection = *global.connections[index];
      const auto at = static_cast<std::uint32_t>(wires.size());
      auto& added = wires.emplace_back();
      added.inner = true;
      added.slot = index;
      added.required = connection.source != nullptr;
      if (!added.required) {
        continue;
      }
      const auto source = cellOfPort.find(connection.source->index());
      const auto target = cellOfPort.find(connection.target.index());
      if (source == cellOfPort.end() || target == cellOfPort.end() ||
          !canvas.standable(source->second) ||
          !canvas.standable(target->second)) {
        continue;
      }
      auto& wire = wires[at];
      wire.feed = source->second;
      wire.target = target->second;
      wire.planned = true;
      canvas.take(wire.feed, at);
      canvas.take(wire.target, at);
      auto found = canvas.search({.start = wire.feed,
                                  .goal = wire.target,
                                  .wire = at,
                                  .avoidObstacles = true});
      if (!found.has_value()) {
        continue;
      }
      for (const auto cell : *found) {
        canvas.take(cell, at);
      }
      wire.path = std::move(*found);
      wire.drawn = true;
    }
  }

  [[nodiscard]] static DetailRoutingT
  artifactOf(const grid::GridMetrics& detail, const std::vector<Wire>& wires,
             const std::uint32_t ring, const std::size_t inner) {
    const auto wireOf = [&detail](const std::vector<std::uint32_t>& path) {
      auto drawn = std::make_unique<fba::DetailWireT>();
      drawn->path.reserve(path.size());
      for (const auto cell : path) {
        drawn->path.emplace_back(cell % detail.width, cell / detail.width);
      }
      return drawn;
    };

    DetailRoutingT routing;
    auto extent = std::make_unique<fba::GridExtentT>();
    extent->width = detail.width;
    extent->height = detail.height;
    extent->origin = detail.origin;
    extent->cell_width = detail.cellWidth;
    extent->cell_height = detail.cellHeight;
    routing.grid = std::move(extent);

    // One wire per connection of the assignment, in its order, and one per
    // connection of the global routing, so a reader needs no key. A
    // connection without a path is an empty wire, not a missing one.
    const std::vector<std::uint32_t> nothing;
    routing.wires.reserve(ring);
    for (std::uint32_t at = 0; at < ring; ++at) {
      routing.wires.push_back(wireOf(nothing));
    }
    routing.inner.reserve(inner);
    for (std::size_t at = 0; at < inner; ++at) {
      routing.inner.push_back(wireOf(nothing));
    }
    for (const auto& wire : wires) {
      if (!wire.drawn) {
        continue;
      }
      auto& into = wire.inner ? routing.inner : routing.wires;
      if (wire.slot < into.size()) {
        into[wire.slot] = wireOf(wire.path);
      }
    }
    return routing;
  }
};

} // namespace

std::optional<std::array<fbg::DCoord, 2>>
cellsOfPlace(const grid::GridMetrics& detail, const fbg::Point& place) {
  // The cell is the floor of the snapped corner coordinate. Rounding a center
  // such as 202.5 would pick either cell, depending on the last bit of the
  // conversion.
  const auto corner = grid::layoutToCorner(detail, place);
  const auto x = onGrid(corner.x());
  const auto y = onGrid(corner.y());
  if (!std::isfinite(x) || !std::isfinite(y) || x < 0 || x >= detail.width ||
      y < 0 || y >= detail.height) {
    return std::nullopt;
  }
  const auto column = static_cast<std::int64_t>(std::floor(x));
  const auto row = static_cast<std::int64_t>(std::floor(y));
  const fbg::DCoord here(static_cast<std::uint32_t>(column),
                         static_cast<std::uint32_t>(row));
  // A vertical edge separates the cell on its left from the one it is the
  // left edge of; a horizontal edge separates the cells above and below.
  if (onEdge(x) && !onEdge(y)) {
    const auto left = column == 0 ? here : fbg::DCoord(here.x() - 1, here.y());
    return std::array{left, here};
  }
  if (!onEdge(x) && onEdge(y)) {
    const auto above = row == 0 ? here : fbg::DCoord(here.x(), here.y() - 1);
    return std::array{above, here};
  }
  return std::array{here, here};
}

std::unique_ptr<IDetailRouter> makePixelAStarRouter() {
  return std::make_unique<PixelAStarRouter>();
}

} // namespace mqt::scpd::pipeline
