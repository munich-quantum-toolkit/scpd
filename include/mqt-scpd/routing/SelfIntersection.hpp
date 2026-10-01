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

#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/mqt_scpd_routing_export.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace mqt::scpd::routing {

// Detection of a routed path that crosses or touches itself.
//
// The search runs over (cell, heading) states. The same cell at two headings
// is two states, so a route can return to a cell it already occupies. On the
// chip, that is a short circuit. No clearance rule sees it, because a
// clearance rule compares two different wires. Each path therefore needs a
// check of its own.
//
// The detector finds two shapes. A revisit is the same cell twice. A diagonal
// crossing is two diagonal steps that cross inside one 2 by 2 block while all
// four cells stay distinct. Each diagonal step is keyed on its block and its
// orientation, and the two orientations in one block always meet at the
// center of the block.
//
// The router emits a sequence of states, so every heading change emits the
// cell it turns on a second time: an A-B-A spur. A window suppresses these
// spurs. A revisit counts only when the two visits are more than
// PATH_LOOP_SPUR_WINDOW steps apart. No real loop is that short at the
// default bend radius of five cells. The detector uses a window and not a
// stack that collapses spurs, because such a stack pops every A-B-A. It would
// unwind a long exact retrace one cell at a time and so remove the defect the
// detector must find.

/**
 * @brief The largest distance, in steps, between two visits of one cell that
 * the detector ignores as a spur.
 */
inline constexpr std::size_t PATH_LOOP_SPUR_WINDOW = 4;

/**
 * @brief The shape of a self-intersection.
 */
enum class PathLoopKind : uint8_t {
  /// The same grid cell is occupied twice.
  Revisit,
  /// Two diagonal steps cross inside one 2 by 2 block.
  DiagonalCross,
};

/**
 * @brief One self-intersection event of a path.
 *
 * A step is a position in the rasterized cells of the path, as
 * rasterizePathCells() produces them.
 */
struct PathLoopHit {
  /// The shape of the event.
  PathLoopKind kind = PathLoopKind::Revisit;
  /// The x coordinate of the cell at step @c secondIndex, in router grid
  /// cells.
  uint32_t x = 0;
  /// The y coordinate of the cell at step @c secondIndex, in router grid
  /// cells.
  uint32_t y = 0;
  /// The step of the earlier visit.
  std::size_t firstIndex = 0;
  /// The step of the revisit or crossing.
  std::size_t secondIndex = 0;
};

/**
 * @brief The working set of the detector.
 *
 * A caller that keeps one scratch across calls avoids new allocations once the
 * containers have grown.
 */
struct MQT_SCPD_ROUTING_EXPORT PathLoopScratch {
  /// The x coordinates of the rasterized cells of the last path checked.
  std::vector<int32_t> xs;
  /// The y coordinates of the rasterized cells of the last path checked.
  std::vector<int32_t> ys;
  /// The number of revisits the spur window ignored on the last scan.
  uint32_t spurRevisitsIgnored = 0;
  /// The largest distance, in steps, of a revisit the spur window ignored on
  /// the last scan. A heading change produces a spur of two steps. While this
  /// value stays at two, the window ignored nothing close to its limit.
  std::size_t maxSpurDistance = 0;

  /**
   * @brief The diagonal steps seen in one 2 by 2 block.
   */
  struct DiagonalVisit {
    /// One bit per orientation: bit 0 for a step whose x and y offsets have
    /// the same sign, bit 1 for a step whose offsets have opposite signs.
    uint8_t mask = 0;
    /// The step at which the latest diagonal step of each orientation
    /// arrived, in the order of the bits.
    std::array<std::size_t, 2> at = {0, 0};
  };
  /// The step of the last recorded visit of each cell, keyed on the cell.
  std::unordered_map<uint64_t, std::size_t> seenCell;
  /// The diagonal steps of each 2 by 2 block, keyed on the corner of the block
  /// with the smallest coordinates.
  std::unordered_map<uint64_t, DiagonalVisit> seenDiagonal;
};

/**
 * @brief Converts a path into a sequence of router grid cells.
 *
 * The walk moves from each point of the path to the next in steps of one cell
 * along the axis of the larger difference, and rounds each position to the
 * nearest cell. On a stretch inside the grid, consecutive cells differ by at
 * most one along each axis. The walk drops a cell outside the grid and a cell
 * that repeats the one before it.
 *
 * @param path The path to rasterize.
 * @param width The number of cells of the router grid along x.
 * @param height The number of cells of the router grid along y.
 * @param xs Receives the x coordinates of the cells. Its previous content is
 * discarded.
 * @param ys Receives the y coordinates of the cells. Its previous content is
 * discarded.
 * @post @p xs and @p ys have the same size.
 */
MQT_SCPD_ROUTING_EXPORT void rasterizePathCells(const Path& path,
                                                uint32_t width, uint32_t height,
                                                std::vector<int32_t>& xs,
                                                std::vector<int32_t>& ys);

/**
 * @brief Scans the rasterized cells of a scratch for self-intersections.
 *
 * After a hit, the scan forgets every earlier visit and restarts from the
 * current cell. A path that runs back along itself for two hundred cells
 * therefore counts as one event, not as two hundred.
 *
 * @param scratch The scratch whose @c xs and @c ys hold the cells to scan. The
 * scan updates its counters and its maps.
 * @param hits Receives one entry per event, after its existing entries. The
 * scan never clears it. May be @c nullptr when only the count is needed.
 * @param stopAtFirst Whether the scan returns at the first event.
 * @return The number of self-intersection events, zero for fewer than three
 * cells.
 */
MQT_SCPD_ROUTING_EXPORT uint32_t scanCellsForSelfIntersection(
    PathLoopScratch& scratch, std::vector<PathLoopHit>* hits, bool stopAtFirst);

/**
 * @brief Finds every self-intersection event of a path.
 *
 * The function rasterizes @p path into @p scratch and scans the cells. The
 * scratch keeps the rasterized cells, so a caller that reports the cells
 * around a hit need not rasterize the path again.
 *
 * @param path The path to check.
 * @param width The number of cells of the router grid along x.
 * @param height The number of cells of the router grid along y.
 * @param scratch The working set. It holds the rasterized cells afterward.
 * @param hits Receives one entry per event, after its existing entries. May
 * be @c nullptr.
 * @return The number of self-intersection events.
 */
MQT_SCPD_ROUTING_EXPORT uint32_t findPathSelfIntersections(
    const Path& path, uint32_t width, uint32_t height, PathLoopScratch& scratch,
    std::vector<PathLoopHit>* hits);

/**
 * @brief Reports whether a path crosses or touches itself.
 *
 * The scan stops at the first event.
 *
 * @param path The path to check.
 * @param width The number of cells of the router grid along x.
 * @param height The number of cells of the router grid along y.
 * @param scratch The working set. It holds the rasterized cells afterward.
 * @param first Receives the first event when it is not @c nullptr and the
 * path meets itself. Otherwise the pointee is left unchanged.
 * @return @c true when the path has at least one self-intersection event.
 */
MQT_SCPD_ROUTING_EXPORT bool pathSelfIntersects(const Path& path,
                                                uint32_t width, uint32_t height,
                                                PathLoopScratch& scratch,
                                                PathLoopHit* first = nullptr);

} // namespace mqt::scpd::routing
