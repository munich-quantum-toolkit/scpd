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

#include "mqt-scpd/routing/Heading.hpp"
#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/mqt_scpd_routing_export.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace mqt::scpd::routing {

class MovePrimitives;

/**
 * @brief Records where a route may cross the feedlines, and on which heading.
 *
 * The constraints hold one byte per cell. A cell within @c expandRadius of a
 * straight run of a feedline remembers the heading of the run, so a straight
 * step may enter the cell at a right angle to that heading only. A closed
 * cell may not be entered at all: a cell within one cell of a turn, or of the
 * first and last ten cells of a feedline, which form its end zones. A turn
 * may not touch any constrained cell, because the heading of an arc changes
 * along it: only straight steps cross a feedline, so the crossing is at a
 * right angle in the rendered geometry too.
 *
 * The class is separate from the router, so a caller can check the same
 * crossing rules. With primitive tables, tagged runs identify turns even
 * where their swept cells start along a straight tangent. Without tables,
 * a heading change after a tagged run identifies that run as a turn. Cells
 * that do not step along their heading are closed in either case.
 *
 * Use the tables that produced the feedlines to include terminal turns.
 * The router supplies its tables. Untagged straight feedlines are supported.
 */
class MQT_SCPD_ROUTING_EXPORT CrossingConstraints {
public:
  /**
   * @brief The mask of a closed cell, which no route may enter on any
   * heading.
   *
   * A closed cell lies within one cell of a cell of a feedline that is on a
   * turn, off the straight runs, or in an end zone.
   */
  static constexpr uint8_t CURVE_ZONE = 0xFF;

  /** @brief Creates empty constraints, which permit every cell and heading. */
  CrossingConstraints() = default;

  /**
   * @brief Builds the constraints of a grid from routed feedlines.
   * @param width The number of cells of the grid along the x axis.
   * @param height The number of cells of the grid along the y axis.
   * @param feedlines The routed feedlines, in the format of Path.
   * @param skip One flag per feedline. The function leaves out a feedline
   * whose flag is set. A feedline beyond the end of @p skip counts.
   * @param expandRadius The distance in cells, along each axis, up to which a
   * straight run constrains the cells around it.
   * @param primitives Optional tables that produced the tagged feedlines.
   * Untagged runs use their cell directions and heading changes.
   * @post The new constraints replace the previous ones. Allocation failure
   * preserves the previous dimensions and masks.
   */
  void build(uint32_t width, uint32_t height,
             const std::vector<Path>& feedlines, const std::vector<bool>& skip,
             int expandRadius = 10, const MovePrimitives* primitives = nullptr);

  /**
   * @brief Removes every constraint and releases the memory of the masks.
   * @post allowed() permits every cell and heading, and heldBytes() is zero.
   */
  void clear();

  /**
   * @brief Counts the bytes the masks hold.
   * @return The capacity of the masks: one byte per cell of the grid of the
   * last build(), or zero before the first build() and after clear().
   */
  [[nodiscard]] std::size_t heldBytes() const { return masks.capacity(); }

  /**
   * @brief Tests whether the constraints hold no cell.
   * @return @c true when the constraints are empty, so that allowed()
   * permits everything.
   */
  [[nodiscard]] bool empty() const { return masks.empty(); }

  /**
   * @brief Tests whether a straight step may enter a cell under a heading.
   * @param x The column of the cell.
   * @param y The row of the cell.
   * @param heading The heading of the step. Only its three low bits are read,
   * as build() reads the headings of the feedlines.
   * @return @c true when the constraints are empty, when the cell is free, or
   * when @p heading is at a right angle to the heading of every straight run
   * present in the cell. @c false for a cell outside the grid of non-empty
   * constraints.
   */
  [[nodiscard]] bool allowed(uint32_t x, uint32_t y, Heading heading) const;

  /**
   * @brief Tests whether a turn may touch a cell.
   * @param x The column of the cell.
   * @param y The row of the cell.
   * @return @c true when the constraints are empty or when the cell is free.
   * @c false for a constrained cell and for a cell outside the grid of
   * non-empty constraints.
   */
  [[nodiscard]] bool turnAllowed(uint32_t x, uint32_t y) const;

  /**
   * @brief Returns the masks of all cells.
   * @return The mask of every cell in row-major order, as maskAt() gives it,
   * or an empty span when the constraints are empty.
   */
  [[nodiscard]] std::span<const uint8_t> cellMasks() const { return masks; }

  /**
   * @brief Returns the mask of a cell.
   * @param x The column of the cell.
   * @param y The row of the cell.
   * @return @c 0 for a free cell, CURVE_ZONE for a closed cell, or else the
   * set of the headings of the straight runs present there, bit @c h for
   * heading @c h. The mask is also @c 0 for a cell outside the grid and when
   * the constraints are empty.
   */
  [[nodiscard]] uint8_t maskAt(const uint32_t x, const uint32_t y) const {
    if (masks.empty() || x >= gridWidth || y >= gridHeight) {
      return 0;
    }
    return masks[(static_cast<std::size_t>(y) * gridWidth) + x];
  }

private:
  /// The number of cells of the grid along the x axis.
  uint32_t gridWidth = 0;
  /// The number of cells of the grid along the y axis.
  uint32_t gridHeight = 0;
  /// The mask of every cell, in row-major order.
  std::vector<uint8_t> masks;
};

} // namespace mqt::scpd::routing
