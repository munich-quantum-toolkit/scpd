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

/**
 * @brief A grid of one bit per cell, row-major, packed into 64-bit words.
 *
 * An obstacle mask is written once and then read many times. One bit per cell
 * keeps such a mask small: a router grid of nine million cells takes about one
 * megabyte, where one byte per cell would take nine. Code that needs one byte
 * per cell can copy the mask into its own working grid once. The const member
 * functions only read, so several threads can read one grid at the same time.
 *
 * Cell (x, y) has the row-major index y times width() plus x.
 */
class MQT_SCPD_GRID_EXPORT BitGrid {
public:
  /**
   * @brief Creates an empty grid of zero by zero cells.
   */
  BitGrid() = default;

  /**
   * @brief Creates a grid of a given size with every cell set to one value.
   * @param width The number of cells along x.
   * @param height The number of cells along y.
   * @param value The value of every cell. The default @c false leaves every
   * cell clear.
   */
  BitGrid(uint32_t width, uint32_t height, bool value = false);

  /**
   * @brief Returns the number of cells along x.
   * @return The width, in cells.
   */
  [[nodiscard]] uint32_t width() const { return gridWidth; }

  /**
   * @brief Returns the number of cells along y.
   * @return The height, in cells.
   */
  [[nodiscard]] uint32_t height() const { return gridHeight; }

  /**
   * @brief Counts the cells of the grid.
   * @return The width times the height.
   */
  [[nodiscard]] std::size_t size() const {
    return static_cast<std::size_t>(gridWidth) * gridHeight;
  }

  /**
   * @brief Checks whether the grid has no cell.
   * @return @c true when the width or the height is zero.
   */
  [[nodiscard]] bool empty() const { return size() == 0; }

  /**
   * @brief Reads one cell by its row-major index.
   * @param index The row-major index of the cell.
   * @pre @p index is less than size().
   * @return @c true when the cell is set.
   */
  [[nodiscard]] bool test(std::size_t index) const {
    return ((packedWords[index >> 6U] >> (index & 63U)) & 1U) != 0U;
  }

  /**
   * @brief Reads one cell by its column and row.
   * @param x The column of the cell.
   * @param y The row of the cell.
   * @pre @p x is less than width(), and @p y is less than height().
   * @return @c true when the cell is set.
   */
  [[nodiscard]] bool testCell(uint32_t x, uint32_t y) const {
    return test((static_cast<std::size_t>(y) * gridWidth) + x);
  }

  /**
   * @brief Writes one cell by its row-major index.
   * @param index The row-major index of the cell.
   * @param value The new value of the cell.
   * @pre @p index is less than size().
   */
  void set(std::size_t index, bool value = true) {
    const uint64_t bit = uint64_t{1} << (index & 63U);
    if (value) {
      packedWords[index >> 6U] |= bit;
    } else {
      packedWords[index >> 6U] &= ~bit;
    }
  }

  /**
   * @brief Writes one cell by its column and row.
   * @param x The column of the cell.
   * @param y The row of the cell.
   * @param value The new value of the cell.
   * @pre @p x is less than width(), and @p y is less than height().
   */
  void setCell(uint32_t x, uint32_t y, bool value = true) {
    set((static_cast<std::size_t>(y) * gridWidth) + x, value);
  }

  /**
   * @brief Sets every cell to one value.
   * @param value The new value of every cell.
   * @post Every cell holds @p value. The bits past the last cell stay clear,
   * so count() stays exact.
   */
  void fill(bool value);

  /**
   * @brief Counts the set cells.
   * @return The number of cells that hold @c true.
   */
  [[nodiscard]] std::size_t count() const;

  /**
   * @brief Gives read access to the packed words.
   *
   * The cell of index @c i is bit @c i mod 64 of word @c i / 64, counted from
   * the least significant bit. The bits past the last cell are clear.
   *
   * @return The words, row-major over all cells.
   */
  [[nodiscard]] const std::vector<uint64_t>& words() const {
    return packedWords;
  }

  /**
   * @brief Compares two grids cell by cell.
   * @return @c true when both grids have the same width, the same height and
   * the same cells.
   */
  [[nodiscard]] bool operator==(const BitGrid&) const = default;

private:
  /// The number of cells along x.
  uint32_t gridWidth = 0;
  /// The number of cells along y.
  uint32_t gridHeight = 0;
  /// The cells, 64 to a word.
  std::vector<uint64_t> packedWords;
};

} // namespace mqt::scpd::grid
