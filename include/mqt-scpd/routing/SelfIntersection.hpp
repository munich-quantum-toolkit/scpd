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

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace mqt::scpd::routing {

// Whether a routed path crosses or touches itself.
//
// The search runs over (cell, heading) states, so the same cell at two
// headings is two states and a route may return to a cell it already
// occupied. In copper that is a short, and no clearance rule sees it, because
// a clearance rule compares two different wires. The router refuses such a
// path, and the design-rule check reports one; both call the functions here,
// so the two cannot drift apart.
//
// Two shapes are found. A revisit is the same cell twice. A diagonal crossing
// is two diagonal steps that cross inside one 2 by 2 block while all four
// cells stay distinct: each diagonal step is keyed on its block and its
// orientation, and both orientations in one block always meet at the center.
//
// The router emits a state sequence, so every heading change re-emits the
// cell it turns on: an A-B-A micro backtrack, hundreds per layout. They are
// suppressed by a window, never by collapsing spurs off a stack. A stack
// collapse pops any A-B-A and so unwinds a long exact retrace one cell at a
// time, removing the defect it is meant to find. A revisit is ignored only
// when the two visits are at most PATH_LOOP_SPUR_WINDOW steps apart; no real
// loop is that short, because the bend radius is five cells.

/// The window of the spur suppression, in steps.
inline constexpr std::size_t PATH_LOOP_SPUR_WINDOW = 4;

enum class PathLoopKind : uint8_t {
  /// The same grid cell is occupied twice.
  Revisit,
  /// Two diagonal steps cross inside one 2 by 2 block.
  DiagonalCross,
};

struct PathLoopHit {
  PathLoopKind kind = PathLoopKind::Revisit;
  /// Where the path meets itself, in router grid cells.
  uint32_t x = 0;
  uint32_t y = 0;
  /// The step of the earlier visit.
  std::size_t firstIndex = 0;
  /// The step of the revisit or crossing.
  std::size_t secondIndex = 0;
};

/// The working set of the detector, reused across calls so that a caller in a
/// hot loop allocates nothing.
struct MQT_SCPD_ROUTING_EXPORT PathLoopScratch {
  /// The rasterized cells of the last path checked.
  std::vector<int32_t> xs;
  std::vector<int32_t> ys;
  /// What the spur window swallowed on the last call. As long as the largest
  /// spur stays at the artifact's two steps, nothing near the window was
  /// suppressed.
  uint32_t spurRevisitsIgnored = 0;
  std::size_t maxSpurDistance = 0;

  struct DiagonalVisit {
    uint8_t mask = 0;
    std::size_t at[2] = {0, 0};
  };
  std::unordered_map<uint64_t, std::size_t> seenCell;
  std::unordered_map<uint64_t, DiagonalVisit> seenDiagonal;
};

/// Rasterize a path onto the router grid with the same walk the obstacle
/// marking uses, so that the cells examined are the ones the corridors were
/// stamped from. Consecutive cells differ by at most one along each axis;
/// cells outside the grid are dropped.
MQT_SCPD_ROUTING_EXPORT void rasterizePathCells(const Path& path, uint32_t width,
                                                uint32_t height,
                                                std::vector<int32_t>& xs,
                                                std::vector<int32_t>& ys);

/// Scan the rasterized cells of the scratch for self-intersections.
///
/// After a hit the detector restarts from the current cell, so a stretch
/// where a path runs back along itself for two hundred cells counts as the
/// one crossing it is. Hits are appended, never cleared. With stopAtFirst the
/// scan returns at the first hit.
///
/// @returns The number of self-intersection events.
MQT_SCPD_ROUTING_EXPORT uint32_t
scanCellsForSelfIntersection(PathLoopScratch& scratch,
                             std::vector<PathLoopHit>* hits, bool stopAtFirst);

/// Rasterize a path and return every self-intersection event in it. The
/// scratch keeps the rasterized cells, so a caller that reports context
/// around a hit does not rasterize twice.
MQT_SCPD_ROUTING_EXPORT uint32_t
findPathSelfIntersections(const Path& path, uint32_t width, uint32_t height,
                          PathLoopScratch& scratch, std::vector<PathLoopHit>* hits);

/// Whether a path crosses or touches itself. Stops at the first hit, which
/// is written to first when given.
MQT_SCPD_ROUTING_EXPORT bool pathSelfIntersects(const Path& path, uint32_t width,
                                                uint32_t height,
                                                PathLoopScratch& scratch,
                                                PathLoopHit* first = nullptr);

} // namespace mqt::scpd::routing
