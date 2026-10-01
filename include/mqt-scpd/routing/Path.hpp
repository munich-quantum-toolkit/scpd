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

#include "mqt-scpd/flatbuffers/geometry.hpp"
#include "mqt-scpd/routing/Heading.hpp"

#include <cstdint>
#include <vector>

namespace mqt::scpd::routing {

/**
 * @brief A point of a routed path on the router grid.
 *
 * A point holds the search state that produced it and the primitive that led
 * to the next point. The primitive makes it possible to rebuild the bends of
 * the path exactly.
 */
struct PathPoint {
  /// The x coordinate of the cell.
  uint32_t x = 0;
  /// The y coordinate of the cell.
  uint32_t y = 0;
  /// The heading of the wire in the cell.
  Heading heading = 0;
  /// The identifier of the primitive that led to the next point.
  uint16_t primitive = 0;

  /**
   * @brief Returns the search state of the point.
   * @return The cell and the heading of the point.
   */
  [[nodiscard]] flatbuffers::geometry::RCoord state() const {
    return {x, y, heading};
  }
  /**
   * @brief Reports whether two points occupy the same cell.
   * @param other The point to compare with.
   * @return @c true when the two points have the same cell, whatever their
   * headings and primitives.
   */
  [[nodiscard]] bool samePlace(const PathPoint& other) const {
    return x == other.x && y == other.y;
  }
  /**
   * @brief Compares two points field by field.
   * @return @c true when the cell, the heading and the primitive are equal.
   */
  [[nodiscard]] bool operator==(const PathPoint&) const = default;
};

/**
 * @brief A routed path, from its source to its target.
 */
using Path = std::vector<PathPoint>;

/**
 * @brief A box of cells on the router grid.
 *
 * Both bounds of each axis belong to the box.
 */
struct CellBox {
  /// The smallest x coordinate in the box.
  uint32_t minX = 0;
  /// The largest x coordinate in the box.
  uint32_t maxX = 0;
  /// The smallest y coordinate in the box.
  uint32_t minY = 0;
  /// The largest y coordinate in the box.
  uint32_t maxY = 0;
};

/**
 * @brief The two ends of one routing request.
 *
 * The heading of the source is the heading the wire leaves with. The heading
 * of the target is the heading the wire arrives with. The primitive fields of
 * both ends are ignored.
 */
struct RoutingObjective {
  /// The end the wire starts at.
  PathPoint source;
  /// The end the wire arrives at.
  PathPoint target;
};

/**
 * @brief One run of a path that carries one primitive tag.
 *
 * A segment is a straight run of several steps, or a single turn primitive.
 * reconstructSegments() builds the segments of a path with nominal lengths,
 * and samplePath() replaces them with the rendered lengths.
 */
struct PathSegment {
  /// The heading the run starts with.
  Heading heading = 0;
  /// The identifier of the primitive of the run.
  uint16_t primitive = 0;
  /// @c true when the run is straight and has more than one step. A straight
  /// run of one step reads @c false, as a turn does.
  bool straight = false;
  /// The grid cells of the run, one per step.
  std::vector<PathPoint> cells;
  /// The path length up to and including each step, in cells.
  std::vector<double> lengthAt;
  /**
   * @brief Counts the steps of the run.
   * @return The number of cells in @c cells.
   */
  [[nodiscard]] uint32_t steps() const {
    return static_cast<uint32_t>(cells.size());
  }
};

/**
 * @brief The tuning of one search.
 */
struct SearchParams {
  /// The number of cells a wire runs straight out of its source before the
  /// search starts.
  uint32_t startStraightLength = 30;
  /// The number of cells a wire runs straight into its target after the
  /// search ends.
  uint32_t endStraightLength = 30;
  /// The bend radius, in cells. It must equal the radius the primitives were
  /// built with, and a router refuses parameters with another radius.
  uint8_t minRadius = 5;
  /// The cost of one eighth turn, in hundredths of a cell.
  uint16_t bendPenalty = 100;
};

} // namespace mqt::scpd::routing
