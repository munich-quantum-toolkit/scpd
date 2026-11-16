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

#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <queue>
#include <unordered_set>
#include <utility>
#include <vector>

namespace mqt::scpd::grid {

namespace {

constexpr std::array<int64_t, 8> DX8 = {1, -1, 0, 0, 1, -1, 1, -1};
constexpr std::array<int64_t, 8> DY8 = {0, 0, 1, -1, 1, 1, -1, -1};

/// The nearest cell to start, by breadth-first search over the eight
/// neighborhood, that satisfies the predicate.
template <typename Predicate>
std::optional<std::size_t>
nearestCell(const uint32_t width, const uint32_t height,
            const std::size_t start, const Predicate& accept) {
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
    for (std::size_t k = 0; k < DX8.size(); ++k) {
      const int64_t nx = x + DX8[k];
      const int64_t ny = y + DY8[k];
      if (nx < 0 || ny < 0 || std::cmp_greater_equal(nx, width) ||
          std::cmp_greater_equal(ny, height)) {
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

/// Whether a cell lies on a grid of the given size.
bool onGrid(const uint32_t width, const uint32_t height, const int64_t x,
            const int64_t y) {
  return x >= 0 && y >= 0 && std::cmp_less(x, width) &&
         std::cmp_less(y, height);
}

/// Undoes a stamped band: every cell returns to what it was.
void unstampBand(BitGrid& mask, const StampedBand& band) {
  for (std::size_t i = 0; i < band.cells.size(); ++i) {
    mask.set(band.cells[i], band.wasBlocked[i]);
  }
}

} // namespace

Step orientationStep(const double orientationDegrees) {
  const double degrees = std::remainder(orientationDegrees, 360.0);
  const double radians = degrees * (std::numbers::pi / 180.0);
  const double dx = std::cos(radians);
  const double dy = std::sin(radians);
  constexpr double threshold = 0.05;
  const auto sign = [](const double value) {
    if (value > threshold) {
      return 1;
    }
    return value < -threshold ? -1 : 0;
  };
  return {.x = sign(dx), .y = sign(dy)};
}

namespace {

constexpr auto MAX_BAND_CELLS =
    static_cast<double>(std::numeric_limits<uint32_t>::max());

/// Bounds a cell count before narrowing it to the band representation.
uint32_t boundedBandCells(const double cells) {
  return static_cast<uint32_t>(std::min(std::max(0.0, cells), MAX_BAND_CELLS));
}

/// Restricts the range of t to positions start + step * t on one grid axis.
void clipStripAxis(const int64_t start, const int64_t step,
                   const int64_t lastCell, int64_t& first, int64_t& last) {
  if (step > 0) {
    first = std::max(first, -start);
    last = std::min(last, lastCell - start);
  } else if (step < 0) {
    first = std::max(first, start - lastCell);
    last = std::min(last, start);
  } else if (start < 0 || start > lastCell) {
    last = first - 1;
  }
}

/// The length of one step in layout units.
double stepLength(const Step step, const GridMetrics& grid) {
  return std::hypot(static_cast<double>(step.x) * grid.cellWidth,
                    static_cast<double>(step.y) * grid.cellHeight);
}

} // namespace

uint32_t bandHalfWidth(const double spacing, const GridMetrics& grid,
                       const Step step) {
  // The width runs along the step at a right angle to the band.
  const Step across{.x = -step.y, .y = step.x};
  auto cell = stepLength(across, grid);
  if (cell <= 0.0) {
    return 0;
  }
  if (step.diagonal()) {
    // The spacing is first counted in cells along an axis, as for a straight
    // band, and the diagonal then shortens it.
    cell /= std::numbers::sqrt2;
  }
  // Even on a diagonal, four times the largest half width saturates
  // after converting the odd square width into a half width.
  const auto cells = std::min(std::ceil(spacing / cell), 4.0 * MAX_BAND_CELLS);
  if (!(cells > 0.0)) {
    return 0;
  }
  const auto square = static_cast<uint64_t>(cells) | uint64_t{1};
  const auto diagonalScale = step.diagonal() ? std::numbers::sqrt2 : 1.0;
  return boundedBandCells((static_cast<double>(square) / diagonalScale) / 2.0);
}

uint32_t bandLength(const double length, const GridMetrics& grid,
                    const Step step) {
  const auto cell = stepLength(step, grid);
  if (cell <= 0.0) {
    return 0;
  }
  return boundedBandCells(std::floor(length / cell));
}

StampedBand stampBand(BitGrid& mask, const PortBand& band) {
  StampedBand stamped;
  if (mask.empty()) {
    stamped.lastForwardOnGrid = false;
    return stamped;
  }
  const uint32_t width = mask.width();
  const uint32_t height = mask.height();
  const int64_t perpX = -band.step.y;
  const int64_t perpY = band.step.x;
  std::unordered_set<std::size_t> covered;

  const auto markStrip = [&](const int64_t cx, const int64_t cy) {
    auto last = static_cast<int64_t>(band.halfWidth);
    auto first = -last;
    clipStripAxis(cx, perpX, width - 1, first, last);
    clipStripAxis(cy, perpY, height - 1, first, last);
    for (auto t = first; t <= last; ++t) {
      const int64_t px = cx + (t * perpX);
      const int64_t py = cy + (t * perpY);
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
  const auto markAlong = [&](const int64_t dx, const int64_t dy,
                             const int64_t first, const uint32_t reach) {
    const auto stride = (dx * dx) + (dy * dy);
    if (stride == 0) {
      markStrip(startX, startY);
      return;
    }

    // The perpendicular offsets disappear when projected onto the band
    // direction. Only strips whose projection can reach the grid can
    // contribute cells. A diagonal joining strip projects one unit earlier.
    const auto xEnd = dx * (width - 1);
    const auto yEnd = dy * (height - 1);
    const auto origin = (startX * dx) + (startY * dy);
    const auto low = std::min<int64_t>(0, xEnd) + std::min<int64_t>(0, yEnd);
    const auto high = std::max<int64_t>(0, xEnd) + std::max<int64_t>(0, yEnd);
    const int64_t join = band.step.diagonal() ? 1 : 0;
    const auto last = std::min<int64_t>(reach, (high - origin + join) / stride);
    for (auto d = std::max(first, (low - origin) / stride); d <= last; ++d) {
      const auto cx = startX + (dx * d);
      const auto cy = startY + (dy * d);
      if (join != 0 && d != 0) {
        markStrip(cx, cy - dy);
      }
      markStrip(cx, cy);
    }
  };
  markAlong(band.step.x, band.step.y, 0, band.forward);
  markAlong(-band.step.x, -band.step.y, 1, band.backward);

  // The unclipped end tells target placement whether to undo the band.
  const auto lastX =
      startX + (band.step.x * static_cast<int64_t>(band.forward));
  const auto lastY =
      startY + (band.step.y * static_cast<int64_t>(band.forward));
  stamped.lastForwardCenter =
      DCoord(static_cast<uint32_t>(std::clamp<int64_t>(lastX, 0, width - 1)),
             static_cast<uint32_t>(std::clamp<int64_t>(lastY, 0, height - 1)));
  stamped.lastForwardOnGrid = onGrid(width, height, lastX, lastY);
  return stamped;
}

std::optional<std::size_t> placeTargetBeyondBand(BitGrid& mask,
                                                 const BitGrid& before,
                                                 const StampedBand& band,
                                                 const Step step) {
  const uint32_t width = mask.width();
  const uint32_t height = mask.height();
  const auto idealX = static_cast<int64_t>(band.lastForwardCenter.x()) + step.x;
  const auto idealY = static_cast<int64_t>(band.lastForwardCenter.y()) + step.y;
  const std::size_t ideal = clampedIndex(width, height, idealX, idealY);
  // A band that runs off the grid has no cell beyond it: the clamped cell is
  // one of the band's own, so the band is undone before the walk takes it.
  if (band.lastForwardOnGrid && onGrid(width, height, idealX, idealY) &&
      !before.test(ideal)) {
    mask.set(ideal, false);
    return ideal;
  }

  // The ideal target hits an obstacle that was there before the bands: undo
  // this band and look past the obstacle along the step.
  unstampBand(mask, band);
  std::optional<std::size_t> found;
  if (step.x != 0 || step.y != 0) {
    auto cx = static_cast<int64_t>(ideal % width);
    auto cy = static_cast<int64_t>(ideal / width);
    while (!found) {
      const std::size_t cell =
          (static_cast<std::size_t>(cy) * width) + static_cast<std::size_t>(cx);
      if (!before.test(cell)) {
        found = cell;
        break;
      }
      for (std::size_t k = 0; k < DX8.size() && !found; ++k) {
        const int64_t nx = cx + DX8[k];
        const int64_t ny = cy + DY8[k];
        if (nx < 0 || ny < 0 || std::cmp_greater_equal(nx, width) ||
            std::cmp_greater_equal(ny, height)) {
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
      if (cx < 0 || cy < 0 || std::cmp_greater_equal(cx, width) ||
          std::cmp_greater_equal(cy, height)) {
        break;
      }
    }
  }
  if (!found) {
    found = nearestCell(width, height, ideal, [&](const std::size_t cell) {
      return !before.test(cell);
    });
  }
  if (found) {
    mask.set(*found, false);
  }
  return found;
}

} // namespace mqt::scpd::grid
