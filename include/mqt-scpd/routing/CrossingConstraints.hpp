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
 * The class is separate from the router, so that a check of a routed path can
 * ask the questions the search asks. The path does not record which question
 * the search asked at a cell. A check therefore asks turnAllowed() for every
 * cell of a turn, from the start of its arc to its end, and allowed() with the
 * heading of the step for every cell of a straight step. The router tests the
 * first cell of a path as if a straight step entered it on the heading of
 * that cell, so a check asks allowed() there too. Where the search of a path
 * begins with a turn, the start of the arc is the last cell of the source
 * stub, which keeps the straight tag of the stub (see Path). A check asks
 * turnAllowed() for that cell too: for the last cell of the source stub when
 * the point after it carries a turn tag and the arc ends at that cell plus
 * the end offset of the primitive of the tag. The tags alone do not tell: a
 * search that begins with a straight step of one cell and then turns gives
 * the same tags. The router applies its exemptions and its single-crossing
 * rule on top of these constraints.
 *
 * A straight run is read from the cells. A cell that steps to the next cell
 * along its own heading is on a straight run. The turns are read from the
 * tags (see Path). A run of points under one tag whose next point has another
 * heading is a turn. The cells of a turn are the points of the run and the
 * point after them, which is the end of the arc. The turn after the first run
 * of a path also takes the point before its run, because that point is the
 * start of the arc where the search began with the turn. Otherwise that point
 * lies one step before the start of the arc. The cells within one cell of a
 * cell of a turn are closed, and so are the cells within one cell of every
 * other cell that is not on a straight run. A turn first sweeps cells
 * straight ahead, and its rendered curve bends from its start. These cells
 * are on a straight run and on a turn at the same time: they constrain the
 * cells around them to their heading, and the cells next to them are closed.
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
   * @post The new constraints replace the previous ones.
   */
  void build(uint32_t width, uint32_t height,
             const std::vector<Path>& feedlines, const std::vector<bool>& skip,
             int expandRadius = 10);

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
   * @param heading The heading of the step.
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
