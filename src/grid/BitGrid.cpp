/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/grid/BitGrid.hpp"

#include <algorithm>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace mqt::scpd::grid {

BitGrid::BitGrid(const uint32_t width, const uint32_t height, const bool value)
    : gridWidth(width), gridHeight(height),
      packedWords((size() + 63U) / 64U, 0) {
  if (value) {
    fill(true);
  }
}

void BitGrid::fill(const bool value) {
  std::ranges::fill(packedWords, value ? ~uint64_t{0} : uint64_t{0});
  if (value && (size() % 64U) != 0U) {
    // Keep the bits past the last cell clear, so that count() stays exact.
    packedWords.back() &= (uint64_t{1} << (size() % 64U)) - 1U;
  }
}

std::size_t BitGrid::count() const {
  std::size_t total = 0;
  for (const uint64_t word : packedWords) {
    total += static_cast<std::size_t>(std::popcount(word));
  }
  return total;
}

void BitGrid::setRange(const std::size_t first, const std::size_t last) {
  if (first >= last) {
    return;
  }
  const auto from = first >> 6U;
  const auto to = (last - 1) >> 6U;
  const auto low = ~uint64_t{0} << (first & 63U);
  const auto high = ~uint64_t{0} >> (63U - ((last - 1) & 63U));
  if (from == to) {
    packedWords[from] |= low & high;
    return;
  }
  packedWords[from] |= low;
  std::fill(packedWords.begin() + static_cast<std::ptrdiff_t>(from + 1),
            packedWords.begin() + static_cast<std::ptrdiff_t>(to),
            ~uint64_t{0});
  packedWords[to] |= high;
}

} // namespace mqt::scpd::grid
