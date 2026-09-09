/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/grid/Bottlenecks.hpp"

#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/grid/Rasterize.hpp"
#include "mqt-scpd/grid/Voronoi.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <numeric>
#include <optional>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace mqt::scpd::grid {
namespace {

/// The eight steps around a cell: the four straight ones first, so that the
/// diagonal ones can be told apart by their index.
constexpr std::array<int, 8> STEP_X = {0, 0, 1, -1, 1, -1, 1, -1};
constexpr std::array<int, 8> STEP_Y = {1, -1, 0, 0, 1, 1, -1, -1};
constexpr std::size_t FIRST_DIAGONAL_STEP = 4;

/// Which shore of a saddle a cell belongs to. Zero is neither.
using Shore = std::uint8_t;

/// The grid geometry the walks share, so that no walk re-derives an index.
class CellSpace {
public:
  CellSpace(const BitGrid& blocked, const MedialAxis& axis,
            std::span<const std::uint32_t> squaredDistance)
      : blocked_(blocked), axis_(axis), distance_(squaredDistance) {}

  [[nodiscard]] std::int64_t x(const std::size_t cell) const {
    return static_cast<std::int64_t>(cell % blocked_.width());
  }
  [[nodiscard]] std::int64_t y(const std::size_t cell) const {
    return static_cast<std::int64_t>(cell / blocked_.width());
  }
  [[nodiscard]] bool inside(const std::int64_t px, const std::int64_t py) const {
    return px >= 0 && py >= 0 && px < static_cast<std::int64_t>(blocked_.width()) &&
           py < static_cast<std::int64_t>(blocked_.height());
  }
  [[nodiscard]] std::size_t index(const std::int64_t px, const std::int64_t py) const {
    return (static_cast<std::size_t>(py) * blocked_.width()) + static_cast<std::size_t>(px);
  }
  [[nodiscard]] bool isBlocked(const std::size_t cell) const { return blocked_.test(cell); }
  [[nodiscard]] bool onAxis(const std::size_t cell) const { return axis_.onAxis(cell); }
  [[nodiscard]] std::uint32_t clearance(const std::size_t cell) const { return distance_[cell]; }

  /// Whether a diagonal step would cut the corner between two walls, or slip
  /// between two cells of the axis. Both are artifacts of the raster rather
  /// than passages a wire could take.
  [[nodiscard]] bool cutsACorner(const std::size_t from, const std::int64_t nx,
                                 const std::int64_t ny) const {
    const auto side1 = index(nx, y(from));
    const auto side2 = index(x(from), ny);
    if (onAxis(side1) && onAxis(side2)) {
      return true;
    }
    return isBlocked(side1) && isBlocked(side2);
  }

private:
  const BitGrid& blocked_;
  const MedialAxis& axis_;
  std::span<const std::uint32_t> distance_;
};

/// The cells of the axis whose clearance is a local minimum along it.
std::vector<std::size_t> saddlePoints(const CellSpace& space, const MedialAxis& axis,
                                      const std::uint32_t limit) {
  std::vector<std::size_t> saddles;
  for (const auto& [cell, neighbors] : axis.neighbors) {
    // A dead end is where the axis stops, not where a corridor narrows.
    if (axis.cells[cell] == AxisCell::Endpoint || neighbors.size() != 2) {
      continue;
    }
    const auto here = space.clearance(cell);
    if (here >= limit) {
      continue;
    }
    const auto before = space.clearance(neighbors[0]);
    const auto after = space.clearance(neighbors[1]);
    // A true minimum, or either edge of a plateau of them. The plateau cases
    // matter because a corridor of constant width has no strict minimum at
    // all and would otherwise contribute no bottleneck.
    if ((here <= before && here <= after) || (here == before && here > after) ||
        (here == after && here > before)) {
      saddles.push_back(cell);
    }
  }
  // The axis is held in a hash map, so the order it yields is not the order
  // of the grid. Sorting makes the bottleneck list, and every label derived
  // from it, the same on every run.
  std::ranges::sort(saddles);
  return saddles;
}

/// The connected groups of free, off-axis cells around a saddle. Two of them
/// are the two sides of the corridor.
std::vector<std::vector<std::size_t>> shoresAround(const CellSpace& space,
                                                   const std::size_t saddle) {
  std::vector<std::size_t> starts;
  for (std::size_t step = 0; step < STEP_X.size(); ++step) {
    const auto nx = space.x(saddle) + STEP_X[step];
    const auto ny = space.y(saddle) + STEP_Y[step];
    if (!space.inside(nx, ny)) {
      continue;
    }
    const auto cell = space.index(nx, ny);
    if (!space.onAxis(cell)) {
      starts.push_back(cell);
    }
  }

  std::vector<std::vector<std::size_t>> shores;
  std::vector<bool> taken(starts.size(), false);
  for (std::size_t seed = 0; seed < starts.size(); ++seed) {
    if (taken[seed]) {
      continue;
    }
    std::vector<std::size_t> shore;
    std::queue<std::size_t> pending;
    pending.push(starts[seed]);
    taken[seed] = true;

    while (!pending.empty()) {
      const auto current = pending.front();
      pending.pop();
      shore.push_back(current);

      for (std::size_t step = 0; step < STEP_X.size(); ++step) {
        const auto nx = space.x(current) + STEP_X[step];
        const auto ny = space.y(current) + STEP_Y[step];
        if (!space.inside(nx, ny)) {
          continue;
        }
        const auto cell = space.index(nx, ny);
        // Only the eight cells around the saddle take part; the grouping asks
        // which of them are joined to each other, not how far the free space
        // reaches.
        const auto found = std::ranges::find(starts, cell);
        if (found == starts.end()) {
          continue;
        }
        const auto position = static_cast<std::size_t>(found - starts.begin());
        if (taken[position]) {
          continue;
        }
        if (step >= FIRST_DIAGONAL_STEP && space.cutsACorner(current, nx, ny)) {
          continue;
        }
        taken[position] = true;
        pending.push(cell);
      }
    }
    shores.push_back(std::move(shore));
  }
  return shores;
}

/// Walk downhill from a shore until a wall is reached, staying on that shore.
///
/// @returns The obstacle cell the walk ended on, or nothing.
std::optional<std::size_t> wallBelow(const CellSpace& space,
                                     const std::unordered_map<std::size_t, Shore>& ownership,
                                     const std::vector<std::size_t>& shore, const Shore mine) {
  if (shore.empty()) {
    return std::nullopt;
  }
  auto current = shore.front();
  for (const auto cell : shore) {
    if (space.clearance(cell) < space.clearance(current)) {
      current = cell;
    }
  }

  std::unordered_set<std::size_t> walked;
  while (!space.isBlocked(current)) {
    walked.insert(current);
    auto best = current;
    auto lowest = space.clearance(current);

    for (std::size_t step = 0; step < STEP_X.size(); ++step) {
      const auto nx = space.x(current) + STEP_X[step];
      const auto ny = space.y(current) + STEP_Y[step];
      if (!space.inside(nx, ny)) {
        continue;
      }
      const auto cell = space.index(nx, ny);
      if (space.onAxis(cell)) {
        continue;
      }
      // Crossing to the other shore is what would put both ends of the
      // bottleneck on the same wall.
      const auto owner = ownership.find(cell);
      if (owner != ownership.end() && owner->second != mine) {
        continue;
      }
      if (step >= FIRST_DIAGONAL_STEP && space.cutsACorner(current, nx, ny)) {
        continue;
      }
      if (space.clearance(cell) < lowest && !walked.contains(cell)) {
        lowest = space.clearance(cell);
        best = cell;
      }
    }

    if (best == current) {
      // The descent has flattened out. A wall directly beside it is still the
      // wall this shore faces.
      for (std::size_t step = 0; step < STEP_X.size(); ++step) {
        const auto nx = space.x(current) + STEP_X[step];
        const auto ny = space.y(current) + STEP_Y[step];
        if (!space.inside(nx, ny)) {
          continue;
        }
        const auto cell = space.index(nx, ny);
        if (space.onAxis(cell)) {
          continue;
        }
        if (step >= FIRST_DIAGONAL_STEP && space.cutsACorner(current, nx, ny)) {
          continue;
        }
        if (space.isBlocked(cell)) {
          return cell;
        }
      }
      return std::nullopt;
    }
    current = best;
  }
  return current;
}

/// Whether the line between two cells runs over a target, which would make it
/// a division of the space around a port rather than a bottleneck.
bool crossesATarget(const CellSpace& space, const std::unordered_set<std::size_t>& targets,
                    const GridMetrics& grid, const std::size_t from, const std::size_t to) {
  if (targets.empty()) {
    return false;
  }
  for (const auto cell : lineCells(space.x(from), space.y(from), space.x(to), space.y(to),
                                   grid.width, grid.height)) {
    if (targets.contains(cell)) {
      return true;
    }
  }
  return false;
}

} // namespace

/// One line per narrowing, where the search reported several.
///
/// Two candidates are the same place when they end on a common wall cell and
/// their other ends are closer together than `reach`. Both halves are needed:
/// two openings on either side of a pillar end on the pillar together, and
/// what tells them apart is that their far ends are nowhere near each other.
/// The relation is closed transitively, so a plateau of any length becomes
/// one group, and the shortest line of the group is kept because that is the
/// one that binds. The order of the result is the order the search found them
/// in.
[[nodiscard]] std::vector<Bottleneck> narrowestOfEveryPlateau(
    const std::vector<Bottleneck>& candidates, const GridMetrics& grid, const double reach) {
  if (!(reach > 0.0)) {
    return candidates;
  }
  const auto count = candidates.size();
  std::vector<std::size_t> group(count);
  std::iota(group.begin(), group.end(), std::size_t{0});
  const auto find = [&group](std::size_t at) {
    while (group[at] != at) {
      group[at] = group[group[at]];
      at = group[at];
    }
    return at;
  };

  const auto layout = [&](const std::size_t cell) {
    return grid.toLayout(static_cast<double>(cell % grid.width),
                         static_cast<double>(cell / grid.width));
  };
  const auto apart = [&](const std::size_t left, const std::size_t right) {
    const auto one = layout(left);
    const auto other = layout(right);
    return std::hypot(other.x() - one.x(), other.y() - one.y());
  };

  /// Whether two cuts share a wall and end within `reach` of each other.
  const auto sameNarrowing = [&](const Bottleneck& left, const Bottleneck& right) {
    if (left.first == right.first) {
      return apart(left.second, right.second) < reach;
    }
    if (left.first == right.second) {
      return apart(left.second, right.first) < reach;
    }
    if (left.second == right.first) {
      return apart(left.first, right.second) < reach;
    }
    if (left.second == right.second) {
      return apart(left.first, right.first) < reach;
    }
    return false;
  };

  for (std::size_t left = 0; left < count; ++left) {
    for (std::size_t right = left + 1; right < count; ++right) {
      if (find(left) == find(right) || !sameNarrowing(candidates[left], candidates[right])) {
        continue;
      }
      group[find(right)] = find(left);
    }
  }

  const auto span = [&](const Bottleneck& gate) { return apart(gate.first, gate.second); };

  // The narrowest of each group, by the first candidate of it where two are
  // the same width, so the choice does not depend on the order of a hash.
  std::unordered_map<std::size_t, std::size_t> narrowest;
  for (std::size_t at = 0; at < count; ++at) {
    const auto root = find(at);
    const auto found = narrowest.find(root);
    if (found == narrowest.end() || span(candidates[at]) < span(candidates[found->second])) {
      narrowest[root] = at;
    }
  }

  std::vector<Bottleneck> kept;
  kept.reserve(narrowest.size());
  for (std::size_t at = 0; at < count; ++at) {
    if (narrowest.at(find(at)) == at) {
      kept.push_back(candidates[at]);
    }
  }
  return kept;
}

std::vector<Bottleneck> findBottlenecks(const BitGrid& blocked, const MedialAxis& axis,
                                        const std::span<const std::uint32_t> squaredDistance,
                                        const GridMetrics& grid,
                                        const BottleneckOptions& options) {
  if (blocked.width() != grid.width || blocked.height() != grid.height) {
    throw std::invalid_argument(std::format("the mask is {}x{} cells and the grid {}x{}",
                                            blocked.width(), blocked.height(), grid.width,
                                            grid.height));
  }
  if (axis.cells.size() != grid.cells() || squaredDistance.size() != grid.cells()) {
    throw std::invalid_argument(
        std::format("the axis has {} cells and the distance transform {}, on a grid of {}",
                    axis.cells.size(), squaredDistance.size(), grid.cells()));
  }

  const CellSpace space(blocked, axis, squaredDistance);
  const std::unordered_set<std::size_t> targets(options.targets.begin(), options.targets.end());

  std::vector<Bottleneck> bottlenecks;
  // A pair of walls is one bottleneck however many saddles look at it.
  std::unordered_set<std::uint64_t> seen;
  const auto pairKey = [](const std::size_t a, const std::size_t b) {
    const auto low = static_cast<std::uint64_t>(std::min(a, b));
    const auto high = static_cast<std::uint64_t>(std::max(a, b));
    return (low << 32U) ^ high;
  };

  for (const auto saddle : saddlePoints(space, axis, options.maximumSquaredClearance)) {
    const auto shores = shoresAround(space, saddle);
    // One shore means the axis runs along a wall rather than between two.
    if (shores.size() < 2) {
      continue;
    }

    std::unordered_map<std::size_t, Shore> ownership;
    for (const auto cell : shores[0]) {
      ownership[cell] = 1;
    }
    for (const auto cell : shores[1]) {
      ownership[cell] = 2;
    }

    const auto first = wallBelow(space, ownership, shores[0], 1);
    const auto second = wallBelow(space, ownership, shores[1], 2);
    if (!first.has_value() || !second.has_value() || *first == *second) {
      continue;
    }
    if (!seen.insert(pairKey(*first, *second)).second) {
      continue;
    }
    if (crossesATarget(space, targets, grid, *first, *second)) {
      continue;
    }
    bottlenecks.push_back({.first = *first, .second = *second, .saddle = saddle});
  }
  return narrowestOfEveryPlateau(bottlenecks, grid, options.sameNarrowing);
}

std::uint32_t bottleneckCapacity(const Bottleneck& bottleneck, const GridMetrics& grid,
                                 const double wireSpacing, const double obstacleSpacing,
                                 const bool roundDown) {
  const auto x0 = static_cast<double>(bottleneck.first % grid.width);
  const auto y0 = static_cast<double>(bottleneck.first / grid.width);
  const auto x1 = static_cast<double>(bottleneck.second % grid.width);
  const auto y1 = static_cast<double>(bottleneck.second / grid.width);
  // The gap is measured along the axis with the smaller cell step, which is
  // the same conversion `cellsFor` inverts.
  const auto step = std::min(grid.cellWidth, grid.cellHeight);
  const auto gap = std::hypot(x1 - x0, y1 - y0) * step;

  const auto pitch = wireSpacing + obstacleSpacing;
  if (gap <= obstacleSpacing || pitch <= 0.0) {
    return 0;
  }
  if (gap < wireSpacing) {
    // Below one wire spacing the corridor carries whatever whole wires fit,
    // which is none unless the gap already exceeds a pitch.
    return static_cast<std::uint32_t>(std::max(0.0, gap / pitch));
  }
  const auto exact = gap / pitch;
  const auto count = roundDown ? std::floor(exact) : std::ceil(exact);
  return static_cast<std::uint32_t>(std::max(0.0, count));
}

} // namespace mqt::scpd::grid
