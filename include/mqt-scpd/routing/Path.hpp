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
 * A point holds a cell and a tag: a heading and the identifier of a primitive
 * of that heading. The tag names the move the point belongs to, so that the
 * bends of the path can be rebuilt exactly. Path describes the format.
 */
struct PathPoint {
  /// The x coordinate of the cell.
  uint32_t x = 0;
  /// The y coordinate of the cell.
  uint32_t y = 0;
  /// The heading the move of the point starts on. In a straight run, it is
  /// the heading of the wire in the cell.
  Heading heading = 0;
  /// The identifier of the primitive of the move the point belongs to, among
  /// the primitives of @c heading.
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
 *
 * The router and the coupler dogleg write paths in one format:
 * - A straight run is a sequence of points one step apart. Each point carries
 *   the straight primitive of its heading.
 * - A turn lists the cells it sweeps under its own tag: the heading it starts
 *   on and its primitive. The first of these points is the start of the arc.
 *   A point where a move starts carries that move, so the straight run before
 *   a turn ends one step before the start of the arc.
 * - One exception: where the search of a routed path begins with a turn, the
 *   start of the arc is the last cell of the source stub, which keeps the tag
 *   of the stub. The first point of that turn is the next cell the arc sweeps.
 *   At a radius of one cell, the exact quarter turn of a diagonal heading
 *   sweeps only its start and its end. Where the search begins with that
 *   turn, no point carries its tag: reconstructSegments() finds no turn
 *   there and counts a straight step in place of it, and samplePath() draws
 *   no arc there.
 * - The arc ends at its start plus the end offset of the primitive,
 *   (Primitive::dx, Primitive::dy). In a routed path, the point after a turn
 *   is that end. A coupler dogleg instead lists the end as a point of the
 *   turn, also where the turn does not sweep it, and goes on with a straight
 *   step from the end. A turn that follows directly takes that point over as
 *   the start of its arc. The last point of a dogleg never lies on the cell
 *   of the point of the routed path that follows it.
 * - The swept cells of a turn are the cells the search tests, not a chain of
 *   neighboring cells (see Primitive::swept). Some eighth turns list a cell
 *   twice or a cell past their end, so a path can step off a cell and back
 *   onto it two points later. Some turns that leave a diagonal heading do not
 *   list their end; in a routed path, the end then follows as the next point.
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
  /**
   * @brief Reports whether the run is straight with more than one step.
   *
   * Only a straight run gathers more than one cell, so a straight run of one
   * step reads @c false, as a turn does.
   *
   * @return @c true when the run has more than one cell.
   */
  [[nodiscard]] bool straight() const { return cells.size() > 1; }
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
