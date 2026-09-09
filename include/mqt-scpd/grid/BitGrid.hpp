/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#pragma once

#include "mqt-scpd/grid/mqt_scpd_grid_export.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace mqt::scpd::grid {

/// A grid of one bit per cell, row-major, packed into 64-bit words.
///
/// The obstacle masks of a run are read by every search and every thread but
/// written once, so they are held as bits: the router grid of the largest
/// benchmark is nine million cells, which is a megabyte here and nine as one
/// byte per cell. A search that wants a byte per cell folds the mask into its
/// own working grid once, as the router does with its corridor.
class MQT_SCPD_GRID_EXPORT BitGrid {
public:
  BitGrid() = default;
  BitGrid(uint32_t width, uint32_t height, bool value = false);

  [[nodiscard]] uint32_t width() const { return width_; }
  [[nodiscard]] uint32_t height() const { return height_; }
  [[nodiscard]] std::size_t size() const {
    return static_cast<std::size_t>(width_) * height_;
  }
  [[nodiscard]] bool empty() const { return size() == 0; }

  [[nodiscard]] bool test(std::size_t index) const {
    return ((words_[index >> 6U] >> (index & 63U)) & 1U) != 0U;
  }
  [[nodiscard]] bool testCell(uint32_t x, uint32_t y) const {
    return test((static_cast<std::size_t>(y) * width_) + x);
  }

  void set(std::size_t index, bool value = true) {
    const uint64_t bit = uint64_t{1} << (index & 63U);
    if (value) {
      words_[index >> 6U] |= bit;
    } else {
      words_[index >> 6U] &= ~bit;
    }
  }
  void setCell(uint32_t x, uint32_t y, bool value = true) {
    set((static_cast<std::size_t>(y) * width_) + x, value);
  }

  /// Set every cell to a value.
  void fill(bool value);

  /// The number of set cells.
  [[nodiscard]] std::size_t count() const;

  /// The packed words, row-major over all cells.
  [[nodiscard]] const std::vector<uint64_t>& words() const { return words_; }

  [[nodiscard]] bool operator==(const BitGrid&) const = default;

private:
  uint32_t width_ = 0;
  uint32_t height_ = 0;
  std::vector<uint64_t> words_;
};

} // namespace mqt::scpd::grid
