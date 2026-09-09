/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/grid/DistanceTransform.hpp"

#include "mqt-scpd/grid/BitGrid.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace mqt::scpd::grid {

std::vector<uint32_t> squaredDistanceTransform(const BitGrid& blocked) {
  const uint32_t width = blocked.width();
  const uint32_t height = blocked.height();
  std::vector<uint32_t> distance(blocked.size(), DISTANCE_UNBOUNDED);
  if (width == 0 || height == 0) {
    return distance;
  }
  for (std::size_t i = 0; i < distance.size(); ++i) {
    if (blocked.test(i)) {
      distance[i] = 0;
    }
  }

  // Along each row: the distance to the nearest blocked cell of the row,
  // then squared.
  for (uint32_t y = 0; y < height; ++y) {
    const std::size_t row = static_cast<std::size_t>(y) * width;
    for (uint32_t x = 1; x < width; ++x) {
      if (distance[row + x - 1] < DISTANCE_UNBOUNDED) {
        distance[row + x] =
            std::min(distance[row + x], distance[row + x - 1] + 1);
      }
    }
    for (int64_t x = static_cast<int64_t>(width) - 2; x >= 0; --x) {
      const std::size_t i = row + static_cast<std::size_t>(x);
      if (distance[i + 1] < DISTANCE_UNBOUNDED) {
        distance[i] = std::min(distance[i], distance[i + 1] + 1);
      }
    }
    for (uint32_t x = 0; x < width; ++x) {
      const uint32_t value = distance[row + x];
      distance[row + x] =
          (value >= DISTANCE_UNBOUNDED) ? DISTANCE_UNBOUNDED : value * value;
    }
  }

  // Down each column: the minimum over the rows of the row distance plus the
  // squared row offset. A better row cannot lie further away than the
  // square root of the best distance so far.
  std::vector<uint32_t> column(height);
  for (uint32_t x = 0; x < width; ++x) {
    for (uint32_t y = 0; y < height; ++y) {
      column[y] = distance[(static_cast<std::size_t>(y) * width) + x];
    }
    for (uint32_t y = 0; y < height; ++y) {
      uint32_t best = column[y];
      if (best == 0) {
        continue;
      }
      const auto range = static_cast<int64_t>(std::sqrt(best)) + 1;
      const int64_t from = std::max<int64_t>(0, static_cast<int64_t>(y) - range);
      const int64_t to =
          std::min<int64_t>(static_cast<int64_t>(height) - 1,
                            static_cast<int64_t>(y) + range);
      for (int64_t sy = from; sy <= to; ++sy) {
        const int64_t dy = static_cast<int64_t>(y) - sy;
        const uint32_t candidate =
            column[static_cast<std::size_t>(sy)] + static_cast<uint32_t>(dy * dy);
        best = std::min(best, candidate);
      }
      distance[(static_cast<std::size_t>(y) * width) + x] = best;
    }
  }
  return distance;
}

} // namespace mqt::scpd::grid
