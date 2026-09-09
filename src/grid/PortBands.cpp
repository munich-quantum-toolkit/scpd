/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/grid/PortBands.hpp"

#include "mqt-scpd/flatbuffers/geometry.hpp"
#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <optional>
#include <queue>
#include <unordered_set>
#include <vector>

namespace mqt::scpd::grid {

namespace {

constexpr int64_t DX8[8] = {1, -1, 0, 0, 1, -1, 1, -1};
constexpr int64_t DY8[8] = {0, 0, 1, -1, 1, 1, -1, -1};

/// The nearest cell to start, by breadth-first search over the eight
/// neighborhood, that satisfies the predicate.
template <typename Predicate>
std::optional<std::size_t> nearestCell(const uint32_t width,
                                       const uint32_t height,
                                       const std::size_t start,
                                       Predicate&& accept) {
  std::queue<std::size_t> queue;
  std::unordered_set<std::size_t> seen;
  queue.push(start);
  seen.insert(start);
  while (!queue.empty()) {
    const std::size_t cell = queue.front();
    queue.pop();
    if (accept(cell)) {
      return cell;
    }
    const auto x = static_cast<int64_t>(cell % width);
    const auto y = static_cast<int64_t>(cell / width);
    for (int k = 0; k < 8; ++k) {
      const int64_t nx = x + DX8[k];
      const int64_t ny = y + DY8[k];
      if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
        continue;
      }
      const std::size_t n =
          (static_cast<std::size_t>(ny) * width) + static_cast<std::size_t>(nx);
      if (seen.insert(n).second) {
        queue.push(n);
      }
    }
  }
  return std::nullopt;
}

std::size_t clampedIndex(const uint32_t width, const uint32_t height,
                         const int64_t x, const int64_t y) {
  const auto cx = std::clamp<int64_t>(x, 0, width - 1);
  const auto cy = std::clamp<int64_t>(y, 0, height - 1);
  return (static_cast<std::size_t>(cy) * width) + static_cast<std::size_t>(cx);
}

} // namespace

Step orientationStep(const double orientationDegrees) {
  const double radians = orientationDegrees * (std::numbers::pi / 180.0);
  const double dx = std::cos(radians);
  const double dy = std::sin(radians);
  constexpr double threshold = 0.05;
  return {.x = static_cast<int8_t>(dx > threshold ? 1 : (dx < -threshold ? -1 : 0)),
          .y = static_cast<int8_t>(dy > threshold ? 1 : (dy < -threshold ? -1 : 0))};
}

uint32_t bandHalfWidth(const double spacing, const GridMetrics& grid,
                       const bool diagonal) {
  auto square = static_cast<int64_t>(std::ceil(spacing / grid.cellWidth));
  if (square % 2 == 0) {
    ++square;
  }
  if (diagonal) {
    return static_cast<uint32_t>(
        (static_cast<double>(square) / std::numbers::sqrt2) / 2.0);
  }
  return static_cast<uint32_t>(square / 2);
}

uint32_t bandLength(const double length, const GridMetrics& grid,
                    const bool diagonal) {
  double cells = length / grid.cellWidth;
  if (diagonal) {
    cells /= std::numbers::sqrt2;
  }
  return static_cast<uint32_t>(std::max(0.0, std::floor(cells)));
}

StampedBand stampBand(BitGrid& mask, const PortBand& band) {
  StampedBand stamped;
  const uint32_t width = mask.width();
  const uint32_t height = mask.height();
  const int64_t perpX = -band.step.y;
  const int64_t perpY = band.step.x;
  std::unordered_set<std::size_t> covered;

  const auto markStrip = [&](const int64_t cx, const int64_t cy) {
    for (int64_t t = -static_cast<int64_t>(band.halfWidth);
         t <= static_cast<int64_t>(band.halfWidth); ++t) {
      const int64_t px = cx + (t * perpX);
      const int64_t py = cy + (t * perpY);
      if (px < 0 || py < 0 || px >= width || py >= height) {
        continue;
      }
      const std::size_t index =
          (static_cast<std::size_t>(py) * width) + static_cast<std::size_t>(px);
      if (covered.insert(index).second) {
        stamped.cells.push_back(index);
        stamped.wasBlocked.push_back(mask.test(index));
        mask.set(index);
      }
    }
  };

  const int64_t startX = band.center.x();
  const int64_t startY = band.center.y();
  int64_t lastX = startX;
  int64_t lastY = startY;

  // Forward along the step, with the joining cell of a diagonal move.
  int64_t prevX = startX;
  int64_t prevY = startY;
  for (int64_t d = 0; d <= static_cast<int64_t>(band.forward); ++d) {
    const int64_t cx = startX + (band.step.x * d);
    const int64_t cy = startY + (band.step.y * d);
    lastX = cx;
    lastY = cy;
    if (band.step.diagonal() && (cx != prevX || cy != prevY)) {
      markStrip(cx, prevY);
    }
    markStrip(cx, cy);
    prevX = cx;
    prevY = cy;
  }
  // Backward against the step, from the first cell past the center.
  prevX = startX;
  prevY = startY;
  for (int64_t d = 1; d <= static_cast<int64_t>(band.backward); ++d) {
    const int64_t cx = startX - (band.step.x * d);
    const int64_t cy = startY - (band.step.y * d);
    if (band.step.diagonal() && (cx != prevX || cy != prevY)) {
      markStrip(cx, prevY);
    }
    markStrip(cx, cy);
    prevX = cx;
    prevY = cy;
  }
  stamped.lastForwardCenter = DCoord(
      static_cast<uint32_t>(std::clamp<int64_t>(lastX, 0, width - 1)),
      static_cast<uint32_t>(std::clamp<int64_t>(lastY, 0, height - 1)));
  return stamped;
}

void unstampBand(BitGrid& mask, const StampedBand& band) {
  for (std::size_t i = 0; i < band.cells.size(); ++i) {
    mask.set(band.cells[i], band.wasBlocked[i]);
  }
}

std::optional<std::size_t> digTargetBeyondBand(BitGrid& mask,
                                               const StampedBand& band,
                                               const Step step) {
  const uint32_t width = mask.width();
  const uint32_t height = mask.height();
  const std::unordered_set<std::size_t> own(band.cells.begin(), band.cells.end());
  const std::size_t ideal = clampedIndex(
      width, height, static_cast<int64_t>(band.lastForwardCenter.x()) + step.x,
      static_cast<int64_t>(band.lastForwardCenter.y()) + step.y);

  int64_t cx = static_cast<int64_t>(ideal % width);
  int64_t cy = static_cast<int64_t>(ideal / width);
  while (true) {
    cx += step.x;
    cy += step.y;
    if (cx < 0 || cy < 0 || cx >= width || cy >= height) {
      break;
    }
    const std::size_t next =
        (static_cast<std::size_t>(cy) * width) + static_cast<std::size_t>(cx);
    if (!own.contains(next)) {
      // Out of the band: this is the exit. Free it and its neighbors.
      for (int64_t dy = -1; dy <= 1; ++dy) {
        for (int64_t dx = -1; dx <= 1; ++dx) {
          const int64_t nx = cx + dx;
          const int64_t ny = cy + dy;
          if (nx >= 0 && ny >= 0 && nx < width && ny < height) {
            mask.setCell(static_cast<uint32_t>(nx), static_cast<uint32_t>(ny), false);
          }
        }
      }
      return next;
    }
    mask.set(next, false);
  }
  return nearestCell(width, height, ideal,
                     [&](const std::size_t cell) { return !mask.test(cell); });
}

std::optional<std::size_t> placeTargetBeyondBand(BitGrid& mask,
                                                 const BitGrid& before,
                                                 const StampedBand& band,
                                                 const Step step) {
  const uint32_t width = mask.width();
  const uint32_t height = mask.height();
  const std::size_t ideal = clampedIndex(
      width, height, static_cast<int64_t>(band.lastForwardCenter.x()) + step.x,
      static_cast<int64_t>(band.lastForwardCenter.y()) + step.y);
  if (!before.test(ideal)) {
    mask.set(ideal, false);
    return ideal;
  }

  // The ideal target hits an obstacle that was there before the bands: undo
  // this band and look past the obstacle along the step.
  unstampBand(mask, band);
  std::optional<std::size_t> found;
  if (step.x != 0 || step.y != 0) {
    int64_t cx = static_cast<int64_t>(ideal % width);
    int64_t cy = static_cast<int64_t>(ideal / width);
    while (!found) {
      const std::size_t cell =
          (static_cast<std::size_t>(cy) * width) + static_cast<std::size_t>(cx);
      if (!before.test(cell)) {
        found = cell;
        break;
      }
      for (int k = 0; k < 8 && !found; ++k) {
        const int64_t nx = cx + DX8[k];
        const int64_t ny = cy + DY8[k];
        if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
          continue;
        }
        const std::size_t n = (static_cast<std::size_t>(ny) * width) +
                              static_cast<std::size_t>(nx);
        if (!before.test(n)) {
          found = n;
        }
      }
      cx += step.x;
      cy += step.y;
      if (cx < 0 || cy < 0 || cx >= width || cy >= height) {
        break;
      }
    }
  }
  if (!found) {
    found = nearestCell(width, height, ideal,
                        [&](const std::size_t cell) { return !before.test(cell); });
  }
  if (found) {
    mask.set(*found, false);
  }
  return found;
}

std::size_t stampLauncherSweep(BitGrid& mask, const DCoord center,
                               const Step step, const uint32_t length,
                               const uint32_t halfWidth) {
  const uint32_t width = mask.width();
  const uint32_t height = mask.height();
  int64_t lastX = center.x();
  int64_t lastY = center.y();
  for (int64_t d = 0; d <= static_cast<int64_t>(length); ++d) {
    const int64_t cx = static_cast<int64_t>(center.x()) + (step.x * d);
    const int64_t cy = static_cast<int64_t>(center.y()) + (step.y * d);
    lastX = cx;
    lastY = cy;
    for (int64_t dy = -static_cast<int64_t>(halfWidth);
         dy <= static_cast<int64_t>(halfWidth); ++dy) {
      for (int64_t dx = -static_cast<int64_t>(halfWidth);
           dx <= static_cast<int64_t>(halfWidth); ++dx) {
        const int64_t px = cx + dx;
        const int64_t py = cy + dy;
        if (px >= 0 && py >= 0 && px < width && py < height) {
          mask.setCell(static_cast<uint32_t>(px), static_cast<uint32_t>(py));
        }
      }
    }
  }
  const std::size_t slot =
      clampedIndex(width, height, lastX + (step.x * (static_cast<int64_t>(halfWidth) + 1)),
                   lastY + (step.y * (static_cast<int64_t>(halfWidth) + 1)));
  mask.set(slot, false);
  return slot;
}

} // namespace mqt::scpd::grid
