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
#include <limits>
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
  // squared row offset, as the lower envelope of one parabola per row
  // (Felzenszwalb and Huttenlocher), in time linear in the column. A row
  // without a blocked cell has no parabola. A cell whose own row has none
  // keeps at most `DISTANCE_UNBOUNDED`, as the scan this replaces did.
  std::vector<int64_t> rowCost(height);
  std::vector<uint32_t> root(height);
  std::vector<double> from(static_cast<std::size_t>(height) + 1);
  for (uint32_t x = 0; x < width; ++x) {
    const auto at = [&](const uint32_t y) -> uint32_t& {
      return distance[(static_cast<std::size_t>(y) * width) + x];
    };
    std::size_t parabolas = 0;
    for (uint32_t y = 0; y < height; ++y) {
      const auto value = at(y);
      rowCost[y] = value;
      if (value >= DISTANCE_UNBOUNDED) {
        continue;
      }
      const auto cost = [&](const uint32_t r) {
        return static_cast<double>(rowCost[r]) +
               (static_cast<double>(r) * static_cast<double>(r));
      };
      // Where the new parabola falls below the last one kept, dropping
      // every parabola it hides.
      double meets = 0.0;
      while (parabolas > 0) {
        const auto last = root[parabolas - 1];
        meets = (cost(y) - cost(last)) /
                (2.0 * (static_cast<double>(y) - static_cast<double>(last)));
        if (parabolas == 1 || meets > from[parabolas - 1]) {
          break;
        }
        --parabolas;
      }
      root[parabolas] = y;
      from[parabolas] =
          parabolas == 0 ? -std::numeric_limits<double>::infinity() : meets;
      ++parabolas;
    }
    if (parabolas == 0) {
      continue;
    }
    std::size_t k = 0;
    for (uint32_t y = 0; y < height; ++y) {
      while (k + 1 < parabolas && from[k + 1] <= static_cast<double>(y)) {
        ++k;
      }
      const auto dy = static_cast<int64_t>(y) - static_cast<int64_t>(root[k]);
      const auto best = static_cast<uint64_t>(rowCost[root[k]] + (dy * dy));
      const auto own = static_cast<uint64_t>(rowCost[y]);
      at(y) = static_cast<uint32_t>(std::min(best, own));
    }
  }
  return distance;
}

} // namespace mqt::scpd::grid
