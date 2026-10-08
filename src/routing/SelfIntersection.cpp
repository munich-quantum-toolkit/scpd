/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/routing/SelfIntersection.hpp"

#include "mqt-scpd/routing/Path.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <utility>
#include <vector>

namespace mqt::scpd::routing {

namespace {

/// The first column or row that the signed 32-bit output cannot hold.
constexpr int64_t COORDINATE_LIMIT = int64_t{INT32_MAX} + 1;

uint64_t cellKey(const int32_t x, const int32_t y) {
  return (static_cast<uint64_t>(static_cast<uint32_t>(x)) << 32U) |
         static_cast<uint32_t>(y);
}

/**
 * @brief Narrows the steps of a walk to those that can round into the grid
 * along one axis.
 *
 * The position at step @c v is @p from plus @c v times @p increment. Only a
 * position in (-0.5, @p size - 0.5) rounds to a cell of the grid. The bounds
 * keep one more step on each side, so that no rounding of the division drops
 * a step that lies inside.
 *
 * @param from The position at step zero.
 * @param increment The change of the position per step.
 * @param size The number of cells along the axis.
 * @param first The first step to walk. The function raises it.
 * @param last The last step to walk. The function lowers it. A value below
 * @p first leaves no step.
 */
void narrowToGrid(const int64_t from, const double increment,
                  const int64_t size, double& first, double& last) {
  if (increment == 0.0) {
    if (from < 0 || from >= size) {
      last = first - 1.0;
    }
    return;
  }
  double low = (-0.5 - static_cast<double>(from)) / increment;
  double high =
      (static_cast<double>(size) - 0.5 - static_cast<double>(from)) / increment;
  if (increment < 0.0) {
    std::swap(low, high);
  }
  first = std::max(first, std::floor(low) - 1.0);
  last = std::min(last, std::ceil(high) + 1.0);
}

/**
 * @brief Runs the scan of scanCellsForSelfIntersection().
 * @param scratch The scratch that holds the cells.
 * @param hits Receives every event when it is not @c nullptr.
 * @param stopAtFirst Whether the scan returns at the first event.
 * @param firstHit Receives the first event when it is not @c nullptr and the
 * scan finds one.
 * @return The number of events.
 */
uint32_t scanCells(PathLoopScratch& scratch, std::vector<PathLoopHit>* hits,
                   const bool stopAtFirst, PathLoopHit* firstHit) {
  constexpr std::size_t none = PathLoopScratch::NO_STEP;
  const auto& cx = scratch.xs;
  const auto& cy = scratch.ys;
  scratch.spurRevisitsIgnored = 0;
  scratch.maxSpurDistance = 0;
  const std::size_t count = std::min(cx.size(), cy.size());
  if (count < 3) {
    return 0;
  }

  // The latest earlier visit of the cell of each step: sorted by cell and
  // then by step, the visits of one cell stand next to each other in order.
  auto& byCell = scratch.cellSteps;
  byCell.clear();
  for (std::size_t j = 0; j < count; ++j) {
    byCell.push_back({.key = cellKey(cx[j], cy[j]), .step = j});
  }
  std::ranges::sort(byCell);
  auto& previousVisit = scratch.previousVisit;
  previousVisit.assign(count, none);
  for (std::size_t i = 1; i < count; ++i) {
    if (byCell[i].key == byCell[i - 1].key) {
      previousVisit[byCell[i].step] = byCell[i - 1].step;
    }
  }

  // The latest earlier diagonal step on the other diagonal of the same 2 by 2
  // block. Slope 0 is a step whose x and y offsets have the same sign, slope
  // 1 a step whose offsets have opposite signs. With room for one entry per
  // step, a scratch that scanned a path scans any shorter path without new
  // memory.
  const auto sameSign = [&](const std::size_t j) {
    return (static_cast<int64_t>(cx[j]) - cx[j - 1]) *
               (static_cast<int64_t>(cy[j]) - cy[j - 1]) >
           0;
  };
  auto& byBlock = scratch.diagonalSteps;
  byBlock.clear();
  byBlock.reserve(count);
  for (std::size_t j = 1; j < count; ++j) {
    const int64_t dx = static_cast<int64_t>(cx[j]) - cx[j - 1];
    const int64_t dy = static_cast<int64_t>(cy[j]) - cy[j - 1];
    if (std::abs(dx) == 1 && std::abs(dy) == 1) {
      byBlock.push_back({.key = cellKey(std::min(cx[j], cx[j - 1]),
                                        std::min(cy[j], cy[j - 1])),
                         .step = j});
    }
  }
  std::ranges::sort(byBlock);
  auto& previousCrossing = scratch.previousCrossing;
  previousCrossing.assign(count, none);
  std::array<std::size_t, 2> latest = {none, none};
  for (std::size_t i = 0; i < byBlock.size(); ++i) {
    if (i == 0 || byBlock[i].key != byBlock[i - 1].key) {
      latest = {none, none};
    }
    const std::size_t step = byBlock[i].step;
    const std::size_t slope = sameSign(step) ? 0U : 1U;
    previousCrossing[step] = latest.at(1U - slope);
    latest.at(slope) = step;
  }

  // After a hit at step r, the scan forgets every visit before r and every
  // diagonal step up to r.
  std::size_t restart = 0;
  uint32_t found = 0;
  for (std::size_t j = 0; j < count; ++j) {
    PathLoopKind kind = PathLoopKind::Revisit;
    std::size_t first = none;
    if (const std::size_t crossing = previousCrossing[j];
        crossing != none && crossing > restart) {
      kind = PathLoopKind::DiagonalCross;
      first = crossing;
    } else if (const std::size_t visit = previousVisit[j];
               visit != none && visit >= restart) {
      // A revisit close enough to be a spur leaves the newer visit as the
      // one the next revisit compares with, so that a chain of spurs cannot
      // add up to a long-range hit.
      if (j - visit <= PATH_LOOP_SPUR_WINDOW) {
        ++scratch.spurRevisitsIgnored;
        scratch.maxSpurDistance = std::max(scratch.maxSpurDistance, j - visit);
      } else {
        first = visit;
      }
    }
    if (first == none) {
      continue;
    }
    const PathLoopHit hit{.kind = kind,
                          .x = static_cast<uint32_t>(cx[j]),
                          .y = static_cast<uint32_t>(cy[j]),
                          .firstIndex = first,
                          .secondIndex = j};
    if (hits != nullptr) {
      hits->push_back(hit);
    }
    if (firstHit != nullptr && found == 0) {
      *firstHit = hit;
    }
    ++found;
    if (stopAtFirst) {
      return found;
    }
    restart = j;
  }
  return found;
}

} // namespace

void rasterizePathCells(const Path& path, const uint32_t width,
                        const uint32_t height, std::vector<int32_t>& xs,
                        std::vector<int32_t>& ys) {
  const int64_t w = std::min<int64_t>(width, COORDINATE_LIMIT);
  const int64_t h = std::min<int64_t>(height, COORDINATE_LIMIT);
  xs.clear();
  ys.clear();
  const auto inside = [&](const int64_t x, const int64_t y) {
    return x >= 0 && y >= 0 && x < w && y < h;
  };
  const auto push = [&](const int64_t x, const int64_t y) {
    if (!inside(x, y)) {
      return;
    }
    const auto cellX = static_cast<int32_t>(x);
    const auto cellY = static_cast<int32_t>(y);
    if (!xs.empty() && xs.back() == cellX && ys.back() == cellY) {
      return;
    }
    xs.push_back(cellX);
    ys.push_back(cellY);
  };
  if (path.empty()) {
    return;
  }
  if (path.size() == 1) {
    push(path[0].x, path[0].y);
    return;
  }
  for (std::size_t k = 0; k + 1 < path.size(); ++k) {
    const int64_t x1 = path[k].x;
    const int64_t y1 = path[k].y;
    const int64_t x2 = path[k + 1].x;
    const int64_t y2 = path[k + 1].y;
    const int64_t dx = x2 - x1;
    const int64_t dy = y2 - y1;
    const int64_t steps = std::max(std::abs(dx), std::abs(dy));
    const double divisor = steps != 0 ? static_cast<double>(steps) : 1.0;
    const double xInc = static_cast<double>(dx) / divisor;
    const double yInc = static_cast<double>(dy) / divisor;
    double first = 0.0;
    auto last = static_cast<double>(steps);
    // The grid is a box, so a move between two points inside it stays
    // inside.
    if (!inside(x1, y1) || !inside(x2, y2)) {
      narrowToGrid(x1, xInc, w, first, last);
      narrowToGrid(y1, yInc, h, first, last);
    }
    if (first > last) {
      continue;
    }
    const auto begin = static_cast<int64_t>(first);
    const auto end = static_cast<int64_t>(last);
    double x = static_cast<double>(x1) + (static_cast<double>(begin) * xInc);
    double y = static_cast<double>(y1) + (static_cast<double>(begin) * yInc);
    for (int64_t v = begin; v <= end; ++v) {
      push(std::llround(x), std::llround(y));
      x += xInc;
      y += yInc;
    }
  }
}

uint32_t scanCellsForSelfIntersection(PathLoopScratch& scratch,
                                      std::vector<PathLoopHit>* hits,
                                      const bool stopAtFirst) {
  return scanCells(scratch, hits, stopAtFirst, nullptr);
}

uint32_t findPathSelfIntersections(const Path& path, const uint32_t width,
                                   const uint32_t height,
                                   PathLoopScratch& scratch,
                                   std::vector<PathLoopHit>* hits) {
  rasterizePathCells(path, width, height, scratch.xs, scratch.ys);
  return scanCells(scratch, hits, false, nullptr);
}

bool pathSelfIntersects(const Path& path, const uint32_t width,
                        const uint32_t height, PathLoopScratch& scratch,
                        PathLoopHit* first) {
  rasterizePathCells(path, width, height, scratch.xs, scratch.ys);
  return scanCells(scratch, nullptr, true, first) != 0;
}

} // namespace mqt::scpd::routing
