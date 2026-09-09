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
    : width_(width), height_(height), words_((size() + 63U) / 64U, 0) {
  if (value) {
    fill(true);
  }
}

void BitGrid::fill(const bool value) {
  std::fill(words_.begin(), words_.end(), value ? ~uint64_t{0} : uint64_t{0});
  if (value && (size() % 64U) != 0U) {
    // Keep the bits past the last cell clear, so that count() stays exact.
    words_.back() &= (uint64_t{1} << (size() % 64U)) - 1U;
  }
}

std::size_t BitGrid::count() const {
  std::size_t total = 0;
  for (const uint64_t word : words_) {
    total += static_cast<std::size_t>(std::popcount(word));
  }
  return total;
}


} // namespace mqt::scpd::grid
