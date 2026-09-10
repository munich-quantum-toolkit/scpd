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
#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/grid/Partitions.hpp"
#include "mqt-scpd/grid/Watershed.hpp"
#include "mqt-scpd/pipeline/CapacityPlanner.hpp"
#include "mqt-scpd/pipeline/Stages.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <queue>
#include <ranges>
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

/// No cell of the grid, and no wire.
constexpr std::uint32_t NOWHERE = std::numeric_limits<std::uint32_t>::max();
constexpr std::uint32_t NO_OWNER = std::numeric_limits<std::uint32_t>::max();

/// The eight steps of the search, and what each costs.
///
/// Ten along an axis and fourteen along a diagonal is the prototype's own
/// integer stand-in for one and the square root of two. The heuristic is the
/// octile distance in the same units, which is exact on an empty grid and
/// never overestimates on any other.
constexpr std::array<std::int32_t, 8> STEP_X = {1, -1, 0, 0, 1, 1, -1, -1};
constexpr std::array<std::int32_t, 8> STEP_Y = {0, 0, 1, -1, 1, -1, 1, -1};
constexpr std::array<std::uint32_t, 8> STEP_COST = {10, 10, 10, 10,
                                                    14, 14, 14, 14};
constexpr std::size_t DIAGONAL_STEPS = 4;

/// What one step of the obstacle penalty is charged at, so that a penalty of
/// one is worth a tenth of a step along an axis.
constexpr std::uint32_t PENALTY_WEIGHT = 10;

/// What one round of having been contested costs a cell, and how often a cell
/// may be counted.
///
/// One round is worth two steps along an axis, so a place that has failed to
/// hold the rule a few times over is worth a detour of that many cells and no
/// more. The cap is there so that the price stays a price: a cell nothing else
/// can replace is still reachable however often it has been fought over.
///
/// Every price in this stage is quoted per **step**, against the ten a step
/// along an axis costs, and never per cell of the grid or per cell of a
/// clearance disc. That is what makes it the same price on all eight
/// benchmarks: one cell is 19 layout units on the 17-qubit chip and 40 on the
/// 9-qubit one, so a way of a given length is twice as many steps on the one
/// as on the other — and a price per step is the same fraction of it either
/// way. A price quoted per cell of a disc would be worse still, because the
/// disc of one wire spacing holds 298 cells on the finer grid and 83 on the
/// coarser.

/// What running right beside another wire costs, per cell of that wire in the
/// eight around it.
///
/// It is not a clearance — the clearance is a design rule and is enforced as a
/// blockade — but a price, so that a wire keeps its distance wherever there is
/// room to and closes up only where there is not. Without it the first wire
/// through a corridor takes the middle of it and the next one has nowhere to
/// go: measured over the eight benchmarks, five connections are left over
/// rather than three.
constexpr std::uint32_t CROWDING_WEIGHT = 10;

/// What the tuning of the stage says, with the prototype's own figures as the
/// defaults the schema declares.
struct Tuning {
  /// How far to either side of the way it has a wire may be moved, in wire
  /// spacings.
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

/// The design rule the stage holds between two wires, as offsets on the grid
/// it draws on.
///
/// `min_wire_spacing` is a length in layout units and the router works in
/// cells, so the rule has to be converted, and the conversion is the
/// prototype's: the rule spans `ceil(spacing / cell)` cells, and what is kept
/// clear is one less than that. Two wires are far enough apart when the
/// distance between their cells is more than `pixels`.
///
/// The conversion is where the rule stops being 185. It cannot be anything
/// else: a wire's cell is 19 layout units across on the 17-qubit chip and 40
/// on the 9-qubit one, so where a wire runs is only known to half a cell in
/// the first place, and asking the grid for a distance it cannot express asks
/// it to tell apart two answers that are the same drawing. What the stage
/// therefore keeps is `pixels` cells, which is 153 to 182 layout units against
/// a rule of 185, and the design-rule check is made against the same number
/// rather than against the rule — a check the grid cannot pass is a check of
/// the grid and not of the copper.
struct Rule {
  /// The rule as it was written, in layout units.
  double spacing = 0.0;
  /// The rule on this grid, in cells.
  std::uint32_t pixels = 0;
  /// Every offset within `pixels` of a cell, the cell itself included.
  std::vector<std::pair<std::int32_t, std::int32_t>> offsets;

  [[nodiscard]] bool binds() const { return !offsets.empty(); }
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
  const auto reach = static_cast<std::int32_t>(rule.pixels);
  const auto limit = reach * reach;
  for (std::int32_t dy = -reach; dy <= reach; ++dy) {
    for (std::int32_t dx = -reach; dx <= reach; ++dx) {
      if (((dx * dx) + (dy * dy)) <= limit) {
        rule.offsets.emplace_back(dx, dy);
      }
    }
  }
  return rule;
}

/// What one search may enter, and what it is charged for entering it.
struct Query {
  std::uint32_t start = NOWHERE;
  std::uint32_t goal = NOWHERE;
  /// The wire the search is for. Its own cells are never in the way, because
  /// they are given up before it runs; what it may still own is its goal.
  std::uint32_t wire = NO_OWNER;
  /// The partition the search may not leave, or `LABEL_NONE` for a search
  /// that `mask` bounds instead.
  PartitionLabel partition = LABEL_NONE;
  /// The cells the search may enter, or nothing for all of them.
  const std::vector<std::uint8_t>* mask = nullptr;
  /// Whether running close to an obstacle is charged for.
  bool avoidObstacles = false;
  /// A price per cell that steers the search without forbidding anything, or
  /// nothing. The cross-boundary pass builds one when its first attempt fails.
  const std::vector<std::uint8_t>* steering = nullptr;
  /// What passing through a cell another wire holds costs, or zero to forbid
  /// it. Set well above the length of any way, it makes the search prefer a
  /// free one wherever there is one and name as few wires as it can where
  /// there is none.
  std::uint32_t displacement = 0;
};

/// The detail grid a run is drawn on.
///
/// It carries what the scene blocks, which partition holds each cell, what
/// running beside an obstacle costs and which wire owns each cell; and it runs
/// the one search that every pass of the stage uses. The working set of the
/// search lives here too, so a stage that runs five thousand of them allocates
/// once rather than five thousand times — the prototype allocates two arrays
/// the size of the whole grid per search, which on the largest benchmark is
/// nine megabytes each.
class Canvas {
public:
  Canvas(const grid::GridMetrics& metrics, grid::BitGrid blocked,
         std::vector<PartitionLabel> labels, const Tuning& tuning)
      : metrics_(metrics), blocked_(std::move(blocked)),
        labels_(std::move(labels)),
        proximity_(proximityOf(blocked_,
                               grid::cellsFor(tuning.penaltyReach, metrics),
                               tuning.penalty)),
        owner_(blocked_.size(), NO_OWNER), crowding_(blocked_.size(), 0),
        guarded_(blocked_.size(), 0), guard_(blocked_.size(), 0),
        cost_(blocked_.size(), 0), parent_(blocked_.size(), NOWHERE),
        seen_(blocked_.size(), 0), closed_(blocked_.size(), 0) {}

  [[nodiscard]] const grid::GridMetrics& metrics() const { return metrics_; }
  [[nodiscard]] std::uint32_t width() const { return metrics_.width; }
  [[nodiscard]] std::uint32_t height() const { return metrics_.height; }
  [[nodiscard]] std::size_t cells() const { return blocked_.size(); }

  [[nodiscard]] std::int64_t columnOf(const std::uint32_t cell) const {
    return static_cast<std::int64_t>(cell % metrics_.width);
  }
  [[nodiscard]] std::int64_t rowOf(const std::uint32_t cell) const {
    return static_cast<std::int64_t>(cell / metrics_.width);
  }

  [[nodiscard]] bool blocked(const std::uint32_t cell) const {
    return blocked_.test(cell);
  }
  [[nodiscard]] PartitionLabel label(const std::uint32_t cell) const {
    return labels_[cell];
  }
  [[nodiscard]] std::uint32_t owner(const std::uint32_t cell) const {
    return owner_[cell];
  }
  /// Whether a wire could stand on a cell at all: free, and owned by nobody.
  [[nodiscard]] bool standable(const std::uint32_t cell) const {
    return !blocked_.test(cell) && owner_[cell] == NO_OWNER;
  }

  /// The wire whose diagonal a step from one cell to another crosses, or
  /// nobody. Only a diagonal step crosses anything, and only when one wire
  /// holds both of the cells beside it.
  [[nodiscard]] std::uint32_t crossedBy(const std::uint32_t from,
                                        const std::uint32_t to) const {
    const auto x = columnOf(to);
    const auto y = rowOf(to);
    const auto px = columnOf(from);
    const auto py = rowOf(from);
    if (x == px || y == py) {
      return NO_OWNER;
    }
    const auto width = static_cast<std::int64_t>(metrics_.width);
    const auto first = static_cast<std::uint32_t>((py * width) + x);
    const auto second = static_cast<std::uint32_t>((y * width) + px);
    return owner_[first] == owner_[second] ? owner_[first] : NO_OWNER;
  }

  /// Give a cell to a wire, or hand it back.
  void take(const std::uint32_t cell, const std::uint32_t wire) {
    if (owner_[cell] == NO_OWNER) {
      crowd(cell, 1);
    }
    owner_[cell] = wire;
  }
  void release(const std::uint32_t cell) {
    unguard(cell);
    if (owner_[cell] != NO_OWNER) {
      crowd(cell, -1);
    }
    owner_[cell] = NO_OWNER;
  }

  void crowd(const std::uint32_t cell, const int by) {
    const auto width = static_cast<std::int64_t>(metrics_.width);
    const auto height = static_cast<std::int64_t>(metrics_.height);
    for (std::size_t step = 0; step < STEP_X.size(); ++step) {
      const auto nx = columnOf(cell) + STEP_X[step];
      const auto ny = rowOf(cell) + STEP_Y[step];
      if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
        continue;
      }
      auto& value = crowding_[static_cast<std::size_t>((ny * width) + nx)];
      value = static_cast<std::uint8_t>(static_cast<int>(value) + by);
    }
  }

  /// The cheapest way from `start` to `goal` the query allows.
  ///
  /// The two ends are exempt from everything but the obstacle mask: a wire is
  /// fed where the assignment feeds it and ends at the cell of its target
  /// port, and a piece of a wire begins and ends at a crossing that belongs to
  /// no partition in particular. What the query bounds is the way between
  /// them.
  ///
  /// @returns The cells from `start` to `goal`, or nothing when the query
  /// admits no way.
  [[nodiscard]] std::optional<std::vector<std::uint32_t>>
  search(const Query& query) {
    if (query.start == NOWHERE || query.goal == NOWHERE ||
        blocked_.test(query.start) || blocked_.test(query.goal)) {
      return std::nullopt;
    }
    if (query.start == query.goal) {
      return std::vector<std::uint32_t>{query.start};
    }

    ++generation_;
    const auto width = static_cast<std::int64_t>(metrics_.width);
    const auto height = static_cast<std::int64_t>(metrics_.height);
    const auto goalX = columnOf(query.goal);
    const auto goalY = rowOf(query.goal);
    const auto heuristic = [goalX, goalY](const std::int64_t x,
                                          const std::int64_t y) {
      const auto dx = std::abs(x - goalX);
      const auto dy = std::abs(y - goalY);
      return static_cast<std::uint32_t>((10 * std::max(dx, dy)) +
                                        (4 * std::min(dx, dy)));
    };

    using Entry = std::pair<std::uint32_t, std::uint32_t>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<>> open;
    cost_[query.start] = 0;
    parent_[query.start] = NOWHERE;
    seen_[query.start] = generation_;
    open.emplace(heuristic(columnOf(query.start), rowOf(query.start)),
                 query.start);

    while (!open.empty()) {
      const auto cell = open.top().second;
      open.pop();
      if (closed_[cell] == generation_) {
        continue;
      }
      closed_[cell] = generation_;
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
        if (!admits(query, next)) {
          continue;
        }
        // A diagonal step past a corner both of whose cells belong to **one**
        // wire slips between two cells of that wire, which in copper is a
        // crossing. It is the only way two eight-connected paths can cross
        // without sharing a cell: a crossing needs the other wire to hold the
        // opposite diagonal of the same two-by-two block, and holding it means
        // owning both of its cells. Two different wires there are two wires
        // passing close, which is a clearance question and not a short. The
        // same step past an obstacle corner cuts the obstacle.
        auto charge =
            query.avoidObstacles
                ? PENALTY_WEIGHT * static_cast<std::uint32_t>(proximity_[next])
                : 0U;
        if (owner_[next] != NO_OWNER && owner_[next] != query.wire) {
          charge += query.displacement;
        }
        if (query.steering != nullptr) {
          charge += PENALTY_WEIGHT *
                    static_cast<std::uint32_t>((*query.steering)[next]);
        }
        charge += CROWDING_WEIGHT * static_cast<std::uint32_t>(crowding_[next]);
        if (step >= DIAGONAL_STEPS) {
          const auto first = static_cast<std::uint32_t>((y * width) + nx);
          const auto second = static_cast<std::uint32_t>((ny * width) + x);
          if (blocked_.test(first) || blocked_.test(second)) {
            continue;
          }
          const auto crossed =
              owner_[first] == owner_[second] ? owner_[first] : NO_OWNER;
          if (crossed != NO_OWNER) {
            // Its own diagonal is never crossed, whatever the search is
            // allowed to pay: that is a short in one wire.
            if (crossed == query.wire || query.displacement == 0) {
              continue;
            }
            charge += query.displacement;
          }
        }
        const auto reached = cost_[cell] + STEP_COST[step] + charge;
        if (seen_[next] == generation_ && reached >= cost_[next]) {
          continue;
        }
        cost_[next] = reached;
        parent_[next] = cell;
        seen_[next] = generation_;
        open.emplace(reached + heuristic(nx, ny), next);
      }
    }
    return std::nullopt;
  }

  /// Put the design rule into force.
  ///
  /// Until this is called, a cell a wire holds is in nobody's way but its
  /// own; afterwards every cell within the rule of a wire is charged to that
  /// wire, and a search that must hold the rule cannot enter one. The
  /// clearance is armed once the pass that has to hold it begins, so the
  /// passes that only seed the wires are not slowed by carrying a field they
  /// do not read.
  void armClearance(const Rule& rule) {
    rule_ = rule;
    if (!rule_.binds()) {
      return;
    }
    for (std::size_t cell = 0; cell < owner_.size(); ++cell) {
      if (owner_[cell] != NO_OWNER) {
        guard(static_cast<std::uint32_t>(cell));
      }
    }
  }

  [[nodiscard]] const Rule& rule() const { return rule_; }

  /// How many cells of other wires lie within the rule of this one. Read
  /// while the wire itself is off the canvas, this is exactly the number of
  /// foreign wire cells that are too close.
  [[nodiscard]] std::uint32_t guardOf(const std::uint32_t cell) const {
    return guard_[cell];
  }

  /// The wires that keep a cell within the rule.
  ///
  /// The rule is symmetric, so the wires whose clearance covers a cell are
  /// exactly the wires holding a cell the rule's own offsets reach.
  void guardsOf(const std::uint32_t cell,
                std::vector<std::uint32_t>& into) const {
    const auto width = static_cast<std::int64_t>(metrics_.width);
    const auto height = static_cast<std::int64_t>(metrics_.height);
    const auto cx = columnOf(cell);
    const auto cy = rowOf(cell);
    for (const auto& [dx, dy] : rule_.offsets) {
      const auto x = cx + dx;
      const auto y = cy + dy;
      if (x < 0 || y < 0 || x >= width || y >= height) {
        continue;
      }
      const auto near = owner_[static_cast<std::size_t>((y * width) + x)];
      if (near != NO_OWNER) {
        into.push_back(near);
      }
    }
  }

  /// Take a cell for a wire and charge its clearance to that wire.
  void occupy(const std::uint32_t cell, const std::uint32_t wire) {
    take(cell, wire);
    guard(cell);
  }

  /// Charge the clearance of one cell, once.
  void guard(const std::uint32_t cell) {
    if (guarded_[cell] != 0 || !rule_.binds()) {
      return;
    }
    guarded_[cell] = 1;
    stamp(cell, 1);
  }

  /// Hand the clearance of one cell back, once. The cell itself stays where
  /// it is, which is how a wire that is being re-routed gets out of the two
  /// places it may not be moved off.
  void unguard(const std::uint32_t cell) {
    if (guarded_[cell] == 0) {
      return;
    }
    guarded_[cell] = 0;
    stamp(cell, -1);
  }

private:
  /// Charge or discharge the rule around one cell.
  void stamp(const std::uint32_t cell, const int by) {
    const auto width = static_cast<std::int64_t>(metrics_.width);
    const auto height = static_cast<std::int64_t>(metrics_.height);
    const auto cx = columnOf(cell);
    const auto cy = rowOf(cell);
    for (const auto& [dx, dy] : rule_.offsets) {
      const auto x = cx + dx;
      const auto y = cy + dy;
      if (x < 0 || y < 0 || x >= width || y >= height) {
        continue;
      }
      auto& value = guard_[static_cast<std::size_t>((y * width) + x)];
      value = static_cast<std::uint16_t>(static_cast<int>(value) + by);
    }
  }

  /// Whether a search may enter a cell.
  [[nodiscard]] bool admits(const Query& query,
                            const std::uint32_t cell) const {
    if (blocked_.test(cell)) {
      return false;
    }
    if (cell == query.goal) {
      // The goal is the wire's own place: the cell of its target port, or the
      // point the assignment feeds it at. It is where the wire has to be, and
      // no clearance can move it there.
      return owner_[cell] == NO_OWNER || owner_[cell] == query.wire ||
             query.displacement > 0;
    }
    if (owner_[cell] != NO_OWNER && query.displacement == 0) {
      return false;
    }
    if (guard_[cell] != 0) {
      return false;
    }
    if (query.partition != LABEL_NONE && labels_[cell] != query.partition) {
      return false;
    }
    return query.mask == nullptr || (*query.mask)[cell] != 0;
  }

  [[nodiscard]] std::vector<std::uint32_t>
  trace(const std::uint32_t start, const std::uint32_t goal) const {
    std::vector<std::uint32_t> path;
    for (auto cell = goal; cell != NOWHERE; cell = parent_[cell]) {
      path.push_back(cell);
      if (cell == start) {
        break;
      }
    }
    std::ranges::reverse(path);
    return path;
  }

  /// What running close to an obstacle costs, cell by cell.
  ///
  /// A breadth-first sweep out of the obstacles, four-connected, whose value
  /// falls off linearly over `radius` layers. Nothing here forbids anything —
  /// the clearance itself is already in the mask; this only keeps a wire off
  /// the artwork where there is room to be off it.
  [[nodiscard]] static std::vector<std::uint8_t>
  proximityOf(const grid::BitGrid& blocked, const std::uint32_t radius,
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
    constexpr std::array<std::int64_t, 4> AROUND_X = {1, -1, 0, 0};
    constexpr std::array<std::int64_t, 4> AROUND_Y = {0, 0, 1, -1};
    for (std::uint32_t layer = 1; layer <= radius && !front.empty(); ++layer) {
      const auto value = static_cast<std::uint8_t>(
          std::max(1U, (penalty * (radius - layer + 1)) / (radius + 1)));
      next.clear();
      for (const auto cell : front) {
        const auto x = static_cast<std::int64_t>(cell % blocked.width());
        const auto y = static_cast<std::int64_t>(cell / blocked.width());
        for (std::size_t step = 0; step < AROUND_X.size(); ++step) {
          const auto nx = x + AROUND_X[step];
          const auto ny = y + AROUND_Y[step];
          if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
            continue;
          }
          const auto neighbour = static_cast<std::uint32_t>((ny * width) + nx);
          if (seen[neighbour]) {
            continue;
          }
          seen[neighbour] = true;
          values[neighbour] = value;
          next.push_back(neighbour);
        }
      }
      front.swap(next);
    }
    return values;
  }

  grid::GridMetrics metrics_;
  grid::BitGrid blocked_;
  std::vector<PartitionLabel> labels_;
  std::vector<std::uint8_t> proximity_;
  std::vector<std::uint32_t> owner_;
  std::vector<std::uint8_t> crowding_;
  /// The design rule in force, and the field that holds it: which cells carry
  /// a clearance of their own, and how many foreign wire cells are within the
  /// rule of each cell.
  Rule rule_;
  std::vector<std::uint8_t> guarded_;
  std::vector<std::uint16_t> guard_;

  // The working set of the search. A cell counts as touched only while its
  // stamp matches the current generation, so nothing is cleared between two
  // searches.
  std::vector<std::uint32_t> cost_;
  std::vector<std::uint32_t> parent_;
  std::vector<std::uint32_t> seen_;
  std::vector<std::uint32_t> closed_;
  std::uint32_t generation_ = 0;
};

/// One wire of the ring, as the corridor stage handed it over and as this
/// stage draws it.
struct Wire {
  /// The cell the assignment feeds it at, and the cell of its target port.
  std::uint32_t feed = NOWHERE;
  std::uint32_t target = NOWHERE;
  /// The cell each crossing of its corridor is realised at.
  std::vector<std::uint32_t> crossings;
  /// The partitions its corridor names, one more than there are crossings.
  std::vector<PartitionLabel> partitions;
  /// The cells it is drawn on, from the feed to the target.
  std::vector<std::uint32_t> path;
  /// Whether the corridor stage gave it a route at all.
  bool planned = false;
  /// Whether every piece of it was drawn.
  bool drawn = false;
  /// Whether it belongs to the inner circuit rather than to the outer ring.
  /// Both are drawn on one canvas and both hold the same rule against every
  /// wire on it, so they are one list; only the artifact tells them apart.
  bool inner = false;
  /// Where it belongs in the artifact's own list.
  std::uint32_t slot = 0;
  /// How many of its cells lie within the design rule of another wire, as
  /// counted when it was last put down.
  std::uint32_t conflicts = 0;
};

/// One piece of a wire: the stretch of it that lies in one partition.
struct Piece {
  std::uint32_t wire = NOWHERE;
  std::uint32_t from = NOWHERE;
  std::uint32_t to = NOWHERE;
  PartitionLabel partition = LABEL_NONE;
  /// Where the piece sits in its wire, so the pieces can be joined in order.
  std::uint32_t position = 0;
};

/// The pieces of one partition, as they were drawn, and the cells they took.
struct Attempt {
  std::size_t drawn = 0;
  std::vector<std::vector<std::uint32_t>> paths;
  std::vector<std::uint32_t> taken;
};

/// What the first pass drew: the pieces every wire is cut into, and the cells
/// each of them was given. A piece with no cells is one that found no way.
struct Drawing {
  std::vector<std::vector<Piece>> pieces;
  std::vector<std::vector<std::vector<std::uint32_t>>> paths;

  /// Whether every piece of a wire was drawn, which is what it takes for the
  /// wire to exist at all.
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

/// The cells one search is bounded to.
///
/// It remembers which of them it set, so that clearing it costs what it marked
/// rather than the whole grid: a wire's corridor is a few thousand cells out of
/// the two and a quarter million the largest benchmark has, and the mask is
/// built once per search.
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

/// A cell coordinate back on the grid it came from.
///
/// The plan carries its places in layout units, and reading one back costs a
/// bit: a crossing that was exactly half a cell comes back a fraction of an
/// ulp away from it. Every place the corridor stage works with is a whole
/// number or a half, and which cells a crossing sits between is decided by
/// exactly that half.
[[nodiscard]] double onGrid(const double value) {
  return std::round(value * 2.0) / 2.0;
}

/// Whether a snapped coordinate names a cell edge rather than a cell centre.
[[nodiscard]] bool onEdge(const double value) {
  return std::abs(value - std::round(value)) < 0.25;
}

/// The cell a place of the plan lies in.
///
/// The corridor stage works in the frame the partition geometry uses, where a
/// whole number is a cell corner and the centre of a cell is a half; a place is
/// written out as the layout point of that number. Reading it back is therefore
/// the floor of the snapped cell coordinate, and **not** a rounding: a centre
/// that comes back as 202.5 rounds to either cell depending on the last bit of
/// the conversion, and the two answers are two different cells of the chip.
[[nodiscard]] std::uint32_t cellAt(const grid::GridMetrics& detail,
                                   const fbg::Point& place) {
  const auto cell = detail.toCell(place);
  const auto x = static_cast<std::int64_t>(std::floor(onGrid(cell.x())));
  const auto y = static_cast<std::int64_t>(std::floor(onGrid(cell.y())));
  if (!detail.contains(x, y)) {
    return NOWHERE;
  }
  return static_cast<std::uint32_t>(
      (y * static_cast<std::int64_t>(detail.width)) + x);
}

/// The labels of the plan's partitions, back on the detail grid.
///
/// A label grid is derived state and appears in no artifact, but every search
/// of the first pass is confined to one partition. The outlines carry it: they
/// run along cell edges and a hole comes back as a ring of its own, so filling
/// them by the even-odd rule gives exactly the cells the label had. Growing
/// the partitions again would be the whole watershed, which on the largest
/// benchmark is most of the capacity stage's runtime.
[[nodiscard]] std::vector<PartitionLabel>
labelsOf(const CapacityPlanT& plan, const grid::GridMetrics& detail) {
  std::vector<grid::PartitionOutline> outlines;
  for (const auto& partition : plan.partitions) {
    for (const auto& ring : partition->outlines) {
      grid::PartitionOutline outline{
          .label = static_cast<PartitionLabel>(partition->label), .ring = {}};
      outline.ring.reserve(ring->vertices.size());
      for (const auto& vertex : ring->vertices) {
        outline.ring.push_back(detail.toCell(vertex));
      }
      outlines.push_back(std::move(outline));
    }
  }
  return grid::rasterizePartitions(outlines, detail);
}

/// The detail router of the first release.
class PixelAStarRouter final : public IDetailRouter {
public:
  [[nodiscard]] DetailRoutingT
  run(const ChipT& chip, const CapacityPlanT& capacity,
      const GlobalRoutingT& global, const AssignmentT& assignment,
      const CorridorRoutingT& corridor, const ConfigT& config) const override {
    const auto scene = buildScene(chip, config);
    const auto tuning = tuningOf(config);

    Canvas canvas(scene.detail, scene.blocked, labelsOf(capacity, scene.detail),
                  tuning);

    // A coarse way for every wire, on the grid it will be drawn on: the
    // corridor cut into one piece per partition it names, each piece searched
    // for inside that partition alone, and the pieces joined. This is a seed
    // and nothing more — the corridor stage's route is a coarse way and not a
    // constraint, and the pass that follows is free to move a wire off it
    // entirely.
    auto wires = wiresOf(canvas, corridor, assignment.connections.size());
    const auto ring = static_cast<std::uint32_t>(wires.size());
    auto drawing = drawInsidePartitions(canvas, wires, tuning);
    placeWhatIsLeft(canvas, drawing, tuning);
    joinPieces(canvas, wires, drawing);
    drawWhatIsLeftWhole(canvas, wires, tuning);
    seedInnerCircuit(canvas, scene, global, wires);

    // The design rule, held by taking wires out and drawing them again until
    // no wire is left within a wire spacing of another.
    const auto rule = ruleOf(config, scene.detail);
    canvas.armClearance(rule);
    // The band is a length and not a cell count: how many cells span four wire
    // spacings is what the grid decides, and it is 39 cells on the finest of
    // the eight grids and 21 on the coarsest. `corridor_half_width` is 40
    // cells on all of them, which is 760 layout units on the one and 1440 on
    // the other.
    const auto reach =
        std::max(1U, tuning.corridorSpacings *
                         grid::cellsFor(rule.spacing, scene.detail));
    crossBoundaryRouting(canvas, wires, tuning, ring, reach);

    return artifactOf(scene.detail, wires, ring, global.connections.size());
  }

private:
  /// What a cell outside the corridor between a wire's two neighbours costs
  /// the relaxation, per cell. The prototype's own figure.
  static constexpr std::uint8_t STEERING_PRICE = 10;

  /// How many wires a displacement may displace in turn.
  ///
  /// One is enough for the benchmark chips: what fails there is a fan whose
  /// wires each need a cell or four of the one beside them, so the fan has to
  /// shift by one and no further. It is a constant rather than a parameter
  /// because it is a property of the mechanism and not a knob to turn — a
  /// deeper chain costs a search per wire per level and placed nothing more.
  static constexpr std::uint32_t DISPLACEMENT_CHAIN = 1;

  /// How far from the place the plan names a wire may be fed, in cells.
  ///
  /// A place the plan names is a place on the grid, and almost always a cell a
  /// wire can stand on. Almost: a resonator is fed on the segment between two
  /// launchers, and where a launcher's own sweep covers that segment the point
  /// lands on a cell the sweep blocked.
  static constexpr std::uint32_t PLACE_REACH = 4;

  /// The two cells a place of the plan sits between, or the one it sits on.
  ///
  /// A crossing on a partition border is a cell edge, so either of the two
  /// cells it separates will do: the piece before it and the piece after it
  /// share the cell, and it is the one cell of a wire's way that belongs to no
  /// partition in particular. A feed, a target and a way out of a port's own
  /// pocket are cell centres and name their cell outright.
  [[nodiscard]] static std::array<std::uint32_t, 2>
  cellsAt(const grid::GridMetrics& detail, const fbg::Point& place) {
    const auto cell = detail.toCell(place);
    const auto here = cellAt(detail, place);
    if (here == NOWHERE) {
      return {NOWHERE, NOWHERE};
    }
    // A vertical edge separates the cell to its left from the one it is the
    // left edge of; a horizontal edge separates the cells above and below.
    if (onEdge(onGrid(cell.x())) && !onEdge(onGrid(cell.y()))) {
      const auto left = here % detail.width == 0 ? NOWHERE : here - 1;
      return {left, here};
    }
    if (!onEdge(onGrid(cell.x())) && onEdge(onGrid(cell.y()))) {
      const auto above = here < detail.width ? NOWHERE : here - detail.width;
      return {above, here};
    }
    return {here, here};
  }

  /// Read the corridor routing into the wires this stage draws.
  ///
  /// Every place the plan names is claimed here, before any search runs: the
  /// point a wire is fed at, the cell of its target port, and every crossing
  /// it was given. Claiming them first is what keeps one wire from drawing
  /// itself over the place another one has to start, end or cross at, which no
  /// search could take back afterwards.
  [[nodiscard]] static std::vector<Wire>
  wiresOf(Canvas& canvas, const CorridorRoutingT& corridor,
          const std::size_t connections) {
    // One wire per connection of the assignment, in its order, which is the
    // order the corridor stage wrote its own list in. Sizing the list from the
    // assignment rather than from the corridor is what keeps the artifact's
    // promise even when the two disagree: a connection the corridor stage left
    // out is a wire without a route, not a wire that is missing.
    std::vector<Wire> wires(connections);
    for (std::uint32_t index = 0; index < connections; ++index) {
      wires[index].slot = index;
    }
    const auto planned = std::min(connections, corridor.corridors.size());
    for (std::uint32_t index = 0; index < planned; ++index) {
      const auto& route = *corridor.corridors[index];
      auto& wire = wires[index];
      if (route.partitions.empty() || route.source == nullptr ||
          route.target == nullptr) {
        continue;
      }
      wire.planned = true;
      wire.slot = index;
      wire.partitions.reserve(route.partitions.size());
      for (const auto partition : route.partitions) {
        wire.partitions.push_back(static_cast<PartitionLabel>(partition));
      }
      wire.feed =
          claim(canvas, cellsAt(canvas.metrics(), *route.source), index);
      wire.target =
          claim(canvas, cellsAt(canvas.metrics(), *route.target), index);
      wire.crossings.reserve(route.crossings.size());
      for (const auto& crossing : route.crossings) {
        wire.crossings.push_back(
            claim(canvas, cellsAt(canvas.metrics(), crossing), index));
      }
      wire.planned =
          wire.feed != NOWHERE && wire.target != NOWHERE &&
          std::ranges::find(wire.crossings, NOWHERE) == wire.crossings.end();
    }
    return wires;
  }

  /// Give one place of a wire to that wire, and say which cell it got.
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

  /// The nearest cell to one that is free and belongs to nobody, the cell
  /// itself included.
  [[nodiscard]] static std::uint32_t standableNear(const Canvas& canvas,
                                                   const std::uint32_t cell,
                                                   const std::uint32_t reach) {
    if (cell == NOWHERE) {
      return NOWHERE;
    }
    const auto width = static_cast<std::int64_t>(canvas.width());
    const auto height = static_cast<std::int64_t>(canvas.height());
    const auto centreX = canvas.columnOf(cell);
    const auto centreY = canvas.rowOf(cell);
    for (std::int64_t radius = 1; radius <= static_cast<std::int64_t>(reach);
         ++radius) {
      for (auto dy = -radius; dy <= radius; ++dy) {
        for (auto dx = -radius; dx <= radius; ++dx) {
          if (std::max(std::abs(dx), std::abs(dy)) != radius) {
            continue;
          }
          const auto x = centreX + dx;
          const auto y = centreY + dy;
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

  /// Draw the pieces of one partition in the order given.
  ///
  /// Only the cells between the ends of a piece are taken, so the piece before
  /// a crossing and the piece after it share the cell they meet at — they are
  /// one wire.
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

  /// Hand back what an attempt took.
  static void undo(Canvas& canvas, const Attempt& made) {
    for (const auto cell : made.taken) {
      canvas.release(cell);
    }
  }

  /// Draw every piece of every wire, one partition at a time.
  ///
  /// A wire is cut at its crossings into one piece per partition it runs
  /// through, and each piece is searched for inside that partition alone. The
  /// pieces of one partition are tried in several orders, because the first
  /// piece drawn takes the room the next one wanted, and the order that draws
  /// the most of them is the one kept. A partition whose pieces all draw in
  /// the first order is settled with one attempt, which is what happens almost
  /// everywhere.
  [[nodiscard]] static Drawing
  drawInsidePartitions(Canvas& canvas, const std::vector<Wire>& wires,
                       const Tuning& tuning) {
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

    for (auto& [partition, pieces] : byPartition) {
      // A total order, so that the first attempt is the same on every run. The
      // permutations after it order the pieces by their start alone, which is
      // what the prototype permutes on.
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
        if (!std::next_permutation(order.begin(), order.end(),
                                   [](const Piece& a, const Piece& b) {
                                     return a.from < b.from;
                                   })) {
          break;
        }
        undo(canvas, best);
        auto made = attempt(canvas, order);
        if (made.drawn > best.drawn) {
          best = std::move(made);
          bestOrder = order;
          continue;
        }
        // The order just tried is no better, so the best one is drawn again;
        // the canvas has to end up holding what is kept.
        undo(canvas, made);
        best = attempt(canvas, bestOrder);
      }
      for (std::size_t at = 0; at < bestOrder.size(); ++at) {
        drawing.paths[bestOrder[at].wire][bestOrder[at].position] =
            std::move(best.paths[at]);
      }
    }
    return drawing;
  }

  /// Draw the pieces of one wire that have none yet.
  ///
  /// @returns Whether the wire has all of its pieces afterwards.
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

  /// Take one wire's pieces off the canvas. The places the plan gave it — the
  /// point it is fed at, its target and its crossings — are not given up: they
  /// are where the wire has to be, not where it happens to run.
  static void giveUp(Canvas& canvas, Drawing& drawing,
                     const std::uint32_t wire) {
    for (auto& path : drawing.paths[wire]) {
      for (std::size_t cell = 1; cell + 1 < path.size(); ++cell) {
        canvas.release(path[cell]);
      }
      path.clear();
    }
  }

  /// Put every wire back exactly as it was.
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

  /// The wires standing in the way of one that has none.
  ///
  /// The way it would take if it could push the others aside names them, and
  /// asking that way is what tells the one or two wires actually in the way
  /// from the dozens that merely share the partition. The corridor stage
  /// learned the same thing: ripping the neighbours in the ring opens a large
  /// hole and rarely the right one.
  [[nodiscard]] static std::vector<std::uint32_t>
  blockersOf(Canvas& canvas, const Drawing& drawing, const std::uint32_t wire) {
    // More than the longest way across the grid, so length never buys a
    // displacement.
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
        // A way that squeezes diagonally through another wire is in that
        // wire's way as much as one that runs over its cells, and it never
        // touches one of them.
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

  /// Place the wires the first pass left over, by taking out the ones in their
  /// way and offering everyone the room that frees.
  ///
  /// A swap is kept only when fewer wires are left over than before it, so a
  /// pass that merely trades one wire for another undoes itself. The wires in
  /// the way are asked for again after each swap, because what stands in the
  /// way changes as they move.
  static void placeWhatIsLeft(Canvas& canvas, Drawing& drawing,
                              const Tuning& tuning) {
    for (std::uint32_t wire = 0; wire < drawing.pieces.size(); ++wire) {
      if (drawing.pieces[wire].empty() || drawing.whole(wire)) {
        continue;
      }
      for (std::uint32_t attempt = 0; attempt < tuning.rounds; ++attempt) {
        const auto blockers = blockersOf(canvas, drawing, wire);
        if (blockers.empty()) {
          break;
        }
        const auto before = drawing;
        const auto left = drawing.undrawn();
        for (const auto blocker : blockers) {
          giveUp(canvas, drawing, blocker);
        }
        static_cast<void>(draw(canvas, drawing, wire));
        for (const auto blocker : blockers) {
          static_cast<void>(draw(canvas, drawing, blocker));
        }
        // A wire the swap freed room for takes it, whoever it is.
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

  /// Join the pieces of every wire into one path, in corridor order.
  ///
  /// A concatenation and not a search. The prototype has to recover which
  /// fragment belongs to which wire by walking from every launcher and
  /// matching endpoints, which costs it a cell at each end of every wire; the
  /// corridor already says which piece is whose. What is left is that a wire
  /// missing one piece has no path at all, and gives its cells and its places
  /// back so that the passes after this one may use them.
  static void joinPieces(Canvas& canvas, std::vector<Wire>& wires,
                         Drawing& drawing) {
    for (std::uint32_t index = 0; index < wires.size(); ++index) {
      auto& wire = wires[index];
      wire.path.clear();
      wire.drawn = wire.planned && drawing.whole(index);
      if (!wire.drawn) {
        // The crossings go back, because a wire that is searched for again
        // from end to end crosses where it likes; the two ends stay, because
        // they are where it has to begin and finish.
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
        // The piece before ends where this one begins, so the crossing they
        // share is written once.
        wire.path.insert(wire.path.end(),
                         std::next(piece.begin(), at == 0 ? 0 : 1),
                         piece.end());
      }
    }
  }

  /// Draw the wires the pieces could not join, in one search from end to end.
  ///
  /// A piece has to begin and end exactly at the crossings the plan names, and
  /// where two wires both have to pass a narrow place the first one drawn
  /// takes it with nothing left for the second — in either order, because
  /// neither piece may bend around the other's ends. Nothing asks for that: the
  /// corridor is what told the pieces where to go, and once the pieces do not
  /// fit it has said all it has to say. So a wire still without a way is
  /// searched for once more from the point it is fed at to its target, over the
  /// free space and around the wires already drawn — the grid, and nothing
  /// else, which is the only thing the prototype's own second pass looks at.
  /// Where even that finds nothing, the wires actually in the way are taken out
  /// and drawn again after it.
  static void drawWhatIsLeftWhole(Canvas& canvas, std::vector<Wire>& wires,
                                  const Tuning& tuning) {
    for (std::uint32_t index = 0; index < wires.size(); ++index) {
      std::vector<std::uint32_t> busy;
      static_cast<void>(
          place(canvas, wires, index, tuning, DISPLACEMENT_CHAIN, busy));
    }
  }

  /// Draw one wire that has none, taking out the wires in its way if that is
  /// what it takes.
  ///
  /// The wires in the way are asked for again after each one comes out, because
  /// what stands in the way changes as they go. A swap is kept only when fewer
  /// wires are left over than before it, so a pass that merely trades one wire
  /// for another puts everything back and takes one more wire out instead.
  ///
  /// A wire that is displaced may displace in turn, `chain` times over. Where a
  /// fan of wires holds every way through a narrow place, no single swap gains
  /// anything: the wire that comes out has nowhere to go either, and the swap
  /// is undone. What is needed there is for the whole fan to shift one place
  /// along, which is what a displacement that displaces does. The three
  /// connections it places on the benchmark chips each needed one to four cells
  /// of exactly one neighbour, and it was that measurement that put it here.
  ///
  /// @returns Whether this call is what drew it.
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
    std::vector<std::uint32_t> out;
    busy.push_back(index);

    for (std::uint32_t attempt = 0; attempt < tuning.maxRelaxation; ++attempt) {
      const auto blockers = inTheWayOf(canvas, wires, index);
      std::optional<std::uint32_t> next;
      for (const auto blocker : blockers) {
        // A wire that is itself in the middle of being placed is not one to
        // take out: undoing it here would undo the very swap that is being
        // tried, and the two would displace each other for ever.
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
      out.push_back(*next);
      vacate(canvas, wires[*next], *next);
      if (offer(canvas, wires, index, out, left, tuning, chain, busy)) {
        busy.pop_back();
        return true;
      }
    }

    // Nothing worked. This wire comes out again and the wires it displaced are
    // given a way; where that way is a different one from the way they had,
    // the next pass sees an arrangement this one has not tried.
    vacate(canvas, wires[index], index);
    for (const auto other : out) {
      static_cast<void>(drawWhole(canvas, wires, other));
    }
    busy.pop_back();
    return false;
  }

  /// Offer one wire the room the wires that came out left, and give them a way
  /// again after it — letting each of them displace in turn while the chain
  /// lasts.
  ///
  /// @returns Whether that left fewer wires over than `left`. When it did not,
  /// the wire comes out again, so that the next attempt asks what stands in
  /// its way rather than following the way it just took.
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

  /// How many wires the corridor stage planned are still without a way.
  [[nodiscard]] static std::size_t undrawn(const std::vector<Wire>& wires) {
    return static_cast<std::size_t>(std::ranges::count_if(
        wires, [](const Wire& wire) { return wire.planned && !wire.drawn; }));
  }

  /// Search one wire from the point it is fed at to its target, over the free
  /// space and around every other wire.
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

  /// Take a wire off the canvas, keeping the two places it cannot be moved
  /// off: the point it is fed at and the cell of its target port.
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

  /// The wires standing in the way of one that has none, named by the way it
  /// would take if it could push them aside.
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

  /// Count, for every wire on the canvas, how many of its cells lie within
  /// the rule of another wire.
  ///
  /// A wire's own clearance covers its own cells, so the count is taken with
  /// the wire lifted off the field and put back afterwards. That is what makes
  /// the number the one a check on the finished copper reports, rather than
  /// the stage's own bookkeeping.
  ///
  /// @returns How many wires are either undrawn or too close to another.
  [[nodiscard]] static std::size_t recount(Canvas& canvas,
                                           std::vector<Wire>& wires) {
    std::size_t open = 0;
    for (auto& wire : wires) {
      if (!wire.planned) {
        continue;
      }
      if (!wire.drawn || wire.path.empty()) {
        wire.conflicts = 0;
        ++open;
        continue;
      }
      for (const auto cell : wire.path) {
        canvas.unguard(cell);
      }
      wire.conflicts = 0;
      for (const auto cell : wire.path) {
        wire.conflicts += canvas.guardOf(cell) != 0 ? 1U : 0U;
      }
      for (const auto cell : wire.path) {
        canvas.guard(cell);
      }
      open += wire.conflicts != 0 ? 1U : 0U;
    }
    return open;
  }

  /// Draw every wire again across the partition borders, holding the design
  /// rule against every other wire.
  ///
  /// This is the prototype's `detailed_cross_boundary_routing`, and it keeps
  /// its shape: sweeps over the wire list alternating direction, a wire that
  /// has been re-routed left alone, and for a wire that has not two phases —
  /// the corridor around the way it has, and then a relaxation that lets go of
  /// the wires ahead of it one place further along each time and steers with
  /// prices instead of confining.
  ///
  /// Two things are different, and both are the same correction.
  ///
  /// The prototype's `astar_in_corridor` knows nothing about where the other
  /// wires are. What keeps them apart is `build_corridor` alone: the box `K`
  /// cells around the wire's own way, **less a disc of the wire spacing around
  /// the way its two neighbours in the ring take**. Against the other two
  /// hundred wires there is no rule at all. So the correction is to cut the
  /// same disc out for every wire and not for two — which is what the rule
  /// says, and what the pictures show the prototype does not do.
  ///
  /// Cutting it per search would cost the whole chip's copper per attempt, so
  /// the cut is not in the mask: the canvas carries a field that says how many
  /// wire cells are within the rule of each cell, every wire charges its own
  /// when it is put down, and a search that must hold the rule cannot enter a
  /// charged cell. Ripping a wire is then exactly what the prototype means by
  /// it — its clearance stops standing in the way, and the wire is marked as
  /// one still to be drawn — with its copper left where it is, so that no two
  /// wires ever share a cell while the sweep is running.
  static void crossBoundaryRouting(Canvas& canvas, std::vector<Wire>& wires,
                                   const Tuning& tuning,
                                   const std::uint32_t ring,
                                   const std::uint32_t reach) {
    Mask mask(canvas.cells());
    std::vector<std::uint8_t> steering(canvas.cells(), 0);
    std::vector<std::uint8_t> settled(wires.size(), 0);

    for (std::uint32_t round = 0; round < tuning.rounds; ++round) {
      const auto forward = (round % 2) == 0;
      std::size_t failed = 0;
      for (std::size_t step = 0; step < wires.size(); ++step) {
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
      if (failed == 0) {
        break;
      }
    }
    static_cast<void>(recount(canvas, wires));
  }

  /// Draw one wire again: the prototype's two phases.
  ///
  /// @returns Whether it found a way that holds the rule. On failure it keeps
  /// the way it had, exactly as the prototype does — `wire.path` is replaced
  /// on success alone.
  [[nodiscard]] static bool
  reroute(Canvas& canvas, std::vector<Wire>& wires, const std::uint32_t index,
          const Tuning& tuning, Mask& mask, std::vector<std::uint8_t>& steering,
          std::vector<std::uint8_t>& settled, const bool forward,
          const std::uint32_t ring, const std::uint32_t reach) {
    const auto original = wires[index].path;
    // A wire with no way at all has no corridor either, and is searched for
    // over whatever the other wires have left.
    const auto band = original.empty() ? 0U : reach;

    lift(canvas, wires, index);
    makeWay(canvas, wires[index]);

    // Phase 1: the corridor around the way it has.
    auto found = search(canvas, wires, index, mask, original, band, nullptr);

    // Phase 2: the relaxation.
    //
    // The wires ahead of this one in the sweep are let go of first, one place
    // further along each time. Where the whole sweep ahead is not enough, the
    // wires behind it follow, one at a time — which is what the prototype's
    // own comment says its `back_count` does, and what its `back_count <= 0`
    // stops it from ever doing.
    std::vector<std::uint32_t> ripped;
    if (!found.has_value()) {
      steer(canvas, wires, index, original, ring, steering);
      const auto count = static_cast<std::int64_t>(wires.size());
      const auto ahead = forward ? 1 : -1;
      const auto letOneGo = [&](const std::int64_t away) {
        const auto other = static_cast<std::uint32_t>(
            (((static_cast<std::int64_t>(index) + away) % count) + count) %
            count);
        if (other == index) {
          return;
        }
        // Letting go of a wire is letting go of its clearance. Its copper
        // stays where it is until it is drawn again, so nothing this search
        // finds can run over it.
        settled[other] = 0;
        ripped.push_back(other);
        letGo(canvas, wires[other]);
        found = search(canvas, wires, index, mask, original, band, &steering);
      };
      for (std::uint32_t relax = 1;
           relax <= tuning.maxRelaxation && !found.has_value(); ++relax) {
        letOneGo(ahead * static_cast<std::int64_t>(relax));
      }
      for (std::uint32_t back = 1;
           back <= tuning.maxRelaxation && !found.has_value(); ++back) {
        letOneGo(-ahead * static_cast<std::int64_t>(back));
      }
    }

    // Whatever was let go of stands in the way again, so that what this wire
    // is put down on is judged against every wire on the chip.
    for (const auto other : ripped) {
      takeUpAgain(canvas, wires[other]);
    }
    // The wire is still off its own two places here, and it has to be: what
    // is counted next is how many of its cells lie within the rule of another
    // wire, and its own feed and its own target are within the rule of the
    // cells beside them. Charging them before the count makes every wire too
    // close to itself, and then no wire is ever finished.
    settle(canvas, wires, index, found.has_value() ? *found : original);
    giveWayBack(canvas, wires[index]);
    return found.has_value() && wires[index].conflicts == 0;
  }

  /// Let go of a wire's clearance, leaving its copper where it is.
  static void letGo(Canvas& canvas, const Wire& wire) {
    for (const auto cell : wire.path) {
      canvas.unguard(cell);
    }
  }

  /// Charge a wire's clearance again.
  static void takeUpAgain(Canvas& canvas, const Wire& wire) {
    for (const auto cell : wire.path) {
      canvas.guard(cell);
    }
  }

  /// One search for one wire, from the point it is fed at to its target.
  ///
  /// @param reach How far to either side of `along` the wire may be moved, or
  /// zero for the whole free space.
  [[nodiscard]] static std::optional<std::vector<std::uint32_t>>
  search(Canvas& canvas, const std::vector<Wire>& wires,
         const std::uint32_t index, Mask& mask,
         const std::vector<std::uint32_t>& along, const std::uint32_t reach,
         const std::vector<std::uint8_t>* steering) {
    const auto& wire = wires[index];
    if (wire.feed == NOWHERE || wire.target == NOWHERE) {
      return std::nullopt;
    }
    Query query{.start = wire.feed,
                .goal = wire.target,
                .wire = index,
                .avoidObstacles = true,
                .steering = steering};
    if (reach != 0 && !along.empty()) {
      buildBand(canvas, along, reach, mask);
      query.mask = &mask.cells;
    }
    return canvas.search(query);
  }

  /// Take one wire off the canvas.
  ///
  /// The two places it cannot be moved off — the point the assignment feeds it
  /// at and the cell of its target port — stay its own **and keep their
  /// clearance**. A wire that is off the canvas for a moment still has to be
  /// able to come back to them, and a wire that is drawn while it is off must
  /// not be allowed to settle within a wire spacing of where it has to return
  /// to: that is a violation no later round can undo, because neither wire can
  /// move the place.
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

  /// Let one wire out of its own two places, for the length of its own
  /// search. A wire may not be kept out of its own way, and it is the only
  /// wire whose clearance is lifted while it is being drawn.
  static void makeWay(Canvas& canvas, const Wire& wire) {
    for (const auto place : {wire.feed, wire.target}) {
      if (place != NOWHERE) {
        canvas.unguard(place);
      }
    }
  }

  static void giveWayBack(Canvas& canvas, const Wire& wire) {
    for (const auto place : {wire.feed, wire.target}) {
      if (place != NOWHERE) {
        canvas.guard(place);
      }
    }
  }

  /// Put one wire down on a way, and count what it ends up too close to.
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

  /// The price field the relaxation searches under.
  ///
  /// Two prices, both of them the prototype's
  /// `compute_corridor_polygon_proximity` and `compute_proximity_grid`.
  /// Everything outside the closed ring made of the wire's own two ends and the
  /// ways its two neighbours in the ring take costs, so the search is drawn
  /// back between them; and everything within a wire spacing of either
  /// neighbour costs, so it does not settle against one. Both are prices and
  /// neither forbids anything, so a wire that has to leave the corridor still
  /// can.
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
      // A wire of the inner circuit stands in no ring, so there is no corridor
      // to draw it back into and nothing here to price.
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
    // Inside the corridor or not, a wire keeps off its neighbours.
    for (const auto& path : {before, after}) {
      for (const auto cell : path) {
        stampDisc(canvas, cell, canvas.rule(), steering);
      }
    }
  }

  /// Set every cell the closed ring encloses to nothing, by the even-odd rule.
  ///
  /// One row at a time: the edges that straddle the row give the crossings,
  /// and the spans between consecutive crossings are what the ring holds. That
  /// is the same rule as testing every cell against every edge, and it costs
  /// the edges once per row rather than once per cell.
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

  /// Charge every cell the rule reaches from one cell.
  static void stampDisc(const Canvas& canvas, const std::uint32_t centre,
                        const Rule& rule, std::vector<std::uint8_t>& steering) {
    const auto width = static_cast<std::int64_t>(canvas.width());
    const auto height = static_cast<std::int64_t>(canvas.height());
    const auto cx = canvas.columnOf(centre);
    const auto cy = canvas.rowOf(centre);
    for (const auto& [dx, dy] : rule.offsets) {
      const auto x = cx + dx;
      const auto y = cy + dy;
      if (x < 0 || y < 0 || x >= width || y >= height) {
        continue;
      }
      steering[static_cast<std::size_t>((y * width) + x)] = STEERING_PRICE;
    }
  }

  /// The cells one wire may be moved onto: a band `reach` cells to either
  /// side of the way it has.
  ///
  /// The band is the only bound left on the search. The clearance is no longer
  /// cut out of it, because the clearance is not a property of one wire's band
  /// any more — it is a field the canvas carries, and it is held against every
  /// wire rather than against the two beside this one in the ring.
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

    // One interval per row. A Chebyshev box around every cell of the way is
    // the union of its rows, so marking the two ends of each row and scanning
    // the window once costs the width of the window per row rather than a
    // square per cell of the way.
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

  /// Seed the connections of the inner circuit.
  ///
  /// These never entered the corridor stage — the inner circuit paid for the
  /// free space it crosses while it was solved — so there is no coarse way for
  /// them to follow, and each is searched for over whatever free space the
  /// ring has left. They then join the ring's own wires in one list: one
  /// canvas, one rule, and the pass that holds the rule does not care which of
  /// the two a wire belongs to. The prototype draws them against nothing at
  /// all, so an inner wire may be laid straight over a feedline.
  static void seedInnerCircuit(Canvas& canvas, const CapacityScene& scene,
                               const GlobalRoutingT& global,
                               std::vector<Wire>& wires) {
    std::map<std::uint32_t, std::uint32_t> cellOfPort;
    for (std::size_t at = 0; at < scene.targetPort.size(); ++at) {
      cellOfPort.emplace(scene.targetPort[at],
                         static_cast<std::uint32_t>(scene.targetCell[at]));
    }

    for (std::uint32_t index = 0; index < global.connections.size(); ++index) {
      const auto& connection = *global.connections[index];
      const auto at = static_cast<std::uint32_t>(wires.size());
      wires.push_back({.inner = true, .slot = index});
      if (connection.source == nullptr) {
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
    // connection of the inner circuit, so a reader needs no key to line them
    // up. A connection with no way is an empty wire and not a missing one.
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

std::unique_ptr<IDetailRouter> makePixelAStarRouter() {
  return std::make_unique<PixelAStarRouter>();
}

} // namespace mqt::scpd::pipeline
