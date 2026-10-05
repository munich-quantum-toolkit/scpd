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
  // then squared. The square saturates at DISTANCE_UNBOUNDED, because a row
  // distance of 65536 or more overflows its square in 32 bits.
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
      const uint64_t value = distance[row + x];
      distance[row + x] = static_cast<uint32_t>(
          std::min<uint64_t>(value * value, DISTANCE_UNBOUNDED));
    }
  }

  // Down each column: the minimum over the rows s of the squared row distance
  // of row s plus (y - s)^2. Every row s gives a parabola over y, and the
  // lower envelope of the parabolas gives the minimum for every row at once
  // (Felzenszwalb and Huttenlocher). The envelope lists the rows whose
  // parabolas take part in it, bottom to top, and the first row at which each
  // one is the lowest. These first rows are whole numbers, so the arithmetic
  // is exact in int64 for every height below 2^31. The minimum of a row is at
  // most its own squared row distance, which fits in 32 bits.
  const auto rows = static_cast<int64_t>(height);
  std::vector<int64_t> column(height);
  std::vector<int64_t> apex(height);
  std::vector<int64_t> first(height);
  const auto offset = [&](const int64_t s) {
    return column[static_cast<std::size_t>(s)];
  };
  const auto parabola = [&](const int64_t s, const int64_t y) {
    return offset(s) + ((y - s) * (y - s));
  };
  for (uint32_t x = 0; x < width; ++x) {
    for (uint32_t y = 0; y < height; ++y) {
      column[y] = distance[(static_cast<std::size_t>(y) * width) + x];
    }
    std::size_t size = 1;
    apex[0] = 0;
    first[0] = 0;
    for (int64_t s = 1; s < rows; ++s) {
      // Drop every parabola that the new one undercuts at its first row.
      while (size > 0 && parabola(apex[size - 1], first[size - 1]) >
                             parabola(s, first[size - 1])) {
        --size;
      }
      if (size == 0) {
        apex[0] = s;
        first[0] = 0;
        size = 1;
        continue;
      }
      // The new parabola is the lowest from the first row after its
      // intersection with the top parabola. The numerator is not negative,
      // because the new parabola does not undercut the top one at its first
      // row.
      const int64_t top = apex[size - 1];
      const int64_t numerator =
          offset(s) - offset(top) + ((s - top) * (s + top));
      const int64_t from = 1 + (numerator / (2 * (s - top)));
      if (from < rows) {
        apex[size] = s;
        first[size] = from;
        ++size;
      }
    }
    std::size_t k = size - 1;
    for (int64_t y = rows - 1; y >= 0; --y) {
      distance[(static_cast<std::size_t>(y) * width) + x] =
          static_cast<uint32_t>(parabola(apex[k], y));
      if (y == first[k] && k > 0) {
        --k;
      }
    }
  }
  return distance;
}

} // namespace mqt::scpd::grid
