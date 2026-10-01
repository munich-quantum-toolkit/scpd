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
#include <vector>

namespace mqt::scpd::routing {

/**
 * @brief Records where a route may cross the feedlines, and on which heading.
 *
 * The constraints hold one byte per cell. A cell within @c expandRadius of a
 * straight run of a feedline remembers the heading of the run, so a route may
 * enter the cell at a right angle to that heading only. A cell within one
 * cell of a bend, or of the first and last ten cells of a feedline, may not
 * be entered at all. The class is separate from the router, so that a check
 * of a routed path asks exactly the question the search asked, and the two
 * answers cannot differ.
 *
 * A straight run is read from the cells, not from the moves. A cell that
 * steps to the next cell along its own heading is on a straight run. Every
 * other cell is on a bend. The constraints therefore depend only on the cells
 * and the headings of a path, not on its primitives. The straight lead of a
 * move that runs straight before it bends counts as a straight run.
 */
class MQT_SCPD_ROUTING_EXPORT CrossingConstraints {
public:
  /** @brief The mask of a cell that no route may enter on any heading. */
  static constexpr uint8_t CURVE_ZONE = 0xFF;

  /** @brief Creates empty constraints, which permit every cell and heading. */
  CrossingConstraints() = default;

  /**
   * @brief Builds the constraints of a grid from routed feedlines.
   * @param width The number of cells of the grid along the x axis.
   * @param height The number of cells of the grid along the y axis.
   * @param feedlines The routed feedlines.
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
   * @brief Removes every constraint.
   * @post allowed() permits every cell and heading.
   */
  void clear();

  /**
   * @brief Tests whether the constraints hold no cell.
   * @return @c true when the constraints are empty, so that allowed()
   * permits everything.
   */
  [[nodiscard]] bool empty() const { return masks.empty(); }

  /**
   * @brief Tests whether a route may enter a cell under a heading.
   * @param x The column of the cell.
   * @param y The row of the cell.
   * @param heading The heading of the route.
   * @return @c true when the constraints are empty, when the cell is free, or
   * when @p heading is at a right angle to the heading of every straight run
   * present in the cell. @c false for a cell outside the grid of non-empty
   * constraints.
   */
  [[nodiscard]] bool allowed(uint32_t x, uint32_t y, Heading heading) const;

  /**
   * @brief Returns the mask of a cell.
   * @param x The column of the cell.
   * @param y The row of the cell.
   * @return @c 0 for a free cell, CURVE_ZONE for a cell no route may enter,
   * or else the set of the headings of the straight runs present there, bit
   * @c h for heading @c h. The mask is also @c 0 for a cell outside the grid
   * and when the constraints are empty.
   */
  [[nodiscard]] uint8_t maskAt(uint32_t x, uint32_t y) const;

private:
  /// The number of cells of the grid along the x axis.
  uint32_t gridWidth = 0;
  /// The number of cells of the grid along the y axis.
  uint32_t gridHeight = 0;
  /// The mask of every cell, in row-major order.
  std::vector<uint8_t> masks;
};

} // namespace mqt::scpd::routing
