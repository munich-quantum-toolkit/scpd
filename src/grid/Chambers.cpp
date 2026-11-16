/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/grid/Chambers.hpp"

#include "mqt-scpd/grid/Bottlenecks.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace mqt::scpd::grid {
namespace {

/// The direction that undoes each of the eight steps.
constexpr std::array<std::size_t, 8> REVERSE_STEP = {1, 0, 3, 2, 7, 6, 5, 4};

/// Whether two segments cross, strictly.
bool segmentsCrossStrictly(const double ax, const double ay, const double bx,
                           const double by, const double cx, const double cy,
                           const double dx, const double dy) {
  const auto side = [](const double px, const double py, const double qx,
                       const double qy, const double rx, const double ry) {
    return ((qx - px) * (ry - py)) - ((qy - py) * (rx - px));
  };
  const auto first = side(ax, ay, bx, by, cx, cy);
  const auto second = side(ax, ay, bx, by, dx, dy);
  const auto third = side(cx, cy, dx, dy, ax, ay);
  const auto fourth = side(cx, cy, dx, dy, bx, by);
  return ((first > 0 && second < 0) || (first < 0 && second > 0)) &&
         ((third > 0 && fourth < 0) || (third < 0 && fourth > 0));
}

/// Whether a point lies on a segment.
bool pointOnSegment(const double px, const double py, const double x0,
                    const double y0, const double x1, const double y1) {
  constexpr double epsilon = 1.0e-6;
  if (std::abs(((py - y0) * (x1 - x0)) - ((px - x0) * (y1 - y0))) > epsilon) {
    return false;
  }
  return px >= std::min(x0, x1) - epsilon && px <= std::max(x0, x1) + epsilon &&
         py >= std::min(y0, y1) - epsilon && py <= std::max(y0, y1) + epsilon;
}

} // namespace

std::vector<std::vector<BlockedMove>>
bottleneckMoves(const std::vector<Bottleneck>& bottlenecks,
                const GridMetrics& grid) {
  std::vector<std::vector<BlockedMove>> moves(bottlenecks.size());
  const auto width = static_cast<std::int64_t>(grid.width);
  const auto height = static_cast<std::int64_t>(grid.height);

  for (std::size_t index = 0; index < bottlenecks.size(); ++index) {
    const auto& gate = bottlenecks[index];
    const std::size_t firstRow = gate.first / grid.width;
    const std::size_t secondRow = gate.second / grid.width;
    const auto x0 = static_cast<double>(gate.first % grid.width) + 0.5;
    const auto y0 = static_cast<double>(firstRow) + 0.5;
    const auto x1 = static_cast<double>(gate.second % grid.width) + 0.5;
    const auto y1 = static_cast<double>(secondRow) + 0.5;

    const auto minX = std::max<std::int64_t>(
        0, static_cast<std::int64_t>(std::floor(std::min(x0, x1))) - 1);
    const auto maxX = std::min<std::int64_t>(
        width - 1, static_cast<std::int64_t>(std::ceil(std::max(x0, x1))) + 1);
    const auto minY = std::max<std::int64_t>(
        0, static_cast<std::int64_t>(std::floor(std::min(y0, y1))) - 1);
    const auto maxY = std::min<std::int64_t>(
        height - 1, static_cast<std::int64_t>(std::ceil(std::max(y0, y1))) + 1);

    auto& blocked = moves[index];
    for (std::int64_t y = minY; y <= maxY; ++y) {
      for (std::int64_t x = minX; x <= maxX; ++x) {
        const auto cell = grid.index(static_cast<std::uint32_t>(x),
                                     static_cast<std::uint32_t>(y));
        const auto ax = static_cast<double>(x) + 0.5;
        const auto ay = static_cast<double>(y) + 0.5;
        const auto onLine = pointOnSegment(ax, ay, x0, y0, x1, y1) &&
                            cell != gate.first && cell != gate.second;

        for (std::size_t step = 0; step < STEP_DX.size(); ++step) {
          const auto nx = x + STEP_DX[step];
          const auto ny = y + STEP_DY[step];
          if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
            continue;
          }
          const auto neighbor = grid.index(static_cast<std::uint32_t>(nx),
                                           static_cast<std::uint32_t>(ny));
          if (!onLine) {
            // Each edge is decided once, from its lower cell.
            if (cell > neighbor) {
              continue;
            }
            if (!segmentsCrossStrictly(ax, ay, static_cast<double>(nx) + 0.5,
                                       static_cast<double>(ny) + 0.5, x0, y0,
                                       x1, y1)) {
              continue;
            }
          }
          blocked.push_back({.cell = cell, .direction = step});
          blocked.push_back(
              {.cell = neighbor, .direction = REVERSE_STEP[step]});
        }
      }
    }
  }
  return moves;
}

} // namespace mqt::scpd::grid
