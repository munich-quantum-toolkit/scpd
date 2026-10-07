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

#include <compare>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace mqt::scpd::routing {

/**
 * @brief The largest distance, in steps, between two visits of one cell that
 * the detector ignores as a spur.
 *
 * A routed path lists the cells each move sweeps (see Path). Some eighth
 * turns list one cell twice, two steps apart: an A-B-A spur. At the default
 * bend radius of five cells, every eighth turn from a cardinal heading sweeps
 * one cell past the end of its arc, so the path reads the end, the cell past
 * it, and the end again. At most other radii, some eighth turns list a cell
 * twice in the same way. Straight steps and quarter turns leave no spur.
 *
 * A spur is not a loop, so a revisit counts only when the two visits are more
 * than this many steps apart. The window therefore passes exactly two shapes:
 * a step to a neighboring cell and back, and a ring through three cells that
 * all touch one another. A ring around a 2 by 2 block closes after four steps
 * and counts, and so does a run of two cells out and back. The window is one
 * step wider than routed paths need: at every bend radius that a DubinsRouter
 * accepts, the moves revisit a cell only two steps apart, and no sequence of
 * up to five moves gives a revisit three or four steps apart. The window is
 * therefore conservative.
 *
 * The detector uses a window and not a stack that collapses spurs, because
 * such a stack pops every A-B-A. It would unwind a long exact retrace one cell
 * at a time and so remove the defect the detector must find.
 */
inline constexpr std::size_t PATH_LOOP_SPUR_WINDOW = 3;

/**
 * @brief The shape of a self-intersection.
 *
 * The search runs over (cell, heading) states. The same cell at two headings
 * is two states, so a route can return to a cell it already occupies. On the
 * chip, that is a short circuit. A clearance rule between two wires does not
 * see it, so each path needs a check of its own. The check reads the
 * rasterized cells of the path and finds two shapes: a revisited cell and a
 * crossing of two diagonal steps.
 *
 * The check does not measure the distance between two parts of one path. Two
 * parts in neighboring cells are no event, and their rendered curves can pass
 * less than a tenth of a cell apart. A caller that needs a clearance between
 * the parts of one wire must measure it on the rendered geometry (see
 * samplePath()).
 */
enum class PathLoopKind : uint8_t {
  /// The same grid cell is occupied twice.
  Revisit,
  /// Two diagonal steps cross inside one 2 by 2 block while all four cells
  /// stay distinct. The two diagonals of a block always meet at its center.
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
 * Every container keeps its capacity from one scan to the next and grows with
 * the number of rasterized cells only. A caller that keeps one scratch across
 * calls therefore allocates nothing for a path that rasterizes to no more
 * cells than one it checked before.
 */
struct MQT_SCPD_ROUTING_EXPORT PathLoopScratch {
  /// The value the detector stores for a step that has no earlier visit or
  /// crossing to name.
  static constexpr std::size_t NO_STEP = SIZE_MAX;

  /// The x coordinates of the rasterized cells of the last path checked.
  std::vector<int32_t> xs;
  /// The y coordinates of the rasterized cells of the last path checked.
  std::vector<int32_t> ys;
  /// The number of revisits the spur window ignored on the last scan.
  uint32_t spurRevisitsIgnored = 0;
  /// The largest distance, in steps, of a revisit the spur window ignored on
  /// the last scan. It never exceeds PATH_LOOP_SPUR_WINDOW.
  std::size_t maxSpurDistance = 0;

  /**
   * @brief A step of the path, keyed on a cell or on a 2 by 2 block.
   *
   * The key packs the two coordinates into one number: x in the high 32 bits
   * and y in the low 32 bits.
   */
  struct KeyedStep {
    /// The cell, or the corner of the block with the smallest coordinates.
    uint64_t key = 0;
    /// The step.
    std::size_t step = 0;
    /**
     * @brief Orders two entries by key and then by step.
     * @return The order of the two entries.
     */
    [[nodiscard]] auto operator<=>(const KeyedStep&) const = default;
  };
  /// Every step of the last scan, keyed on its cell and sorted.
  std::vector<KeyedStep> cellSteps;
  /// Every diagonal step of the last scan, keyed on the 2 by 2 block it
  /// crosses and sorted.
  std::vector<KeyedStep> diagonalSteps;
  /// For each step of the last scan, the latest earlier step on the same
  /// cell, or NO_STEP.
  std::vector<std::size_t> previousVisit;
  /// For each step of the last scan, the latest earlier diagonal step that
  /// crosses the same 2 by 2 block on the other diagonal, or NO_STEP. A step
  /// that is not diagonal holds NO_STEP.
  std::vector<std::size_t> previousCrossing;

  /**
   * @brief Counts the bytes the scratch holds.
   * @return The capacity of every container, in bytes.
   */
  [[nodiscard]] std::size_t heldBytes() const {
    return ((xs.capacity() + ys.capacity()) * sizeof(int32_t)) +
           ((cellSteps.capacity() + diagonalSteps.capacity()) *
            sizeof(KeyedStep)) +
           ((previousVisit.capacity() + previousCrossing.capacity()) *
            sizeof(std::size_t));
  }
};

/**
 * @brief Converts a path into a sequence of router grid cells.
 *
 * The walk moves from each point of the path to the next in steps of one cell
 * along the axis of the larger difference, and rounds each position to the
 * nearest cell. The walk drops a cell outside the grid and a cell that repeats
 * the one before it. Consecutive cells therefore differ by at most one along
 * each axis, except where the path leaves the grid and comes back: the first
 * cell after such a stretch can lie anywhere. The walk skips the steps that
 * round to a cell outside the grid, so a jump between two distant points costs
 * no more than the cells it covers inside the grid. The output holds signed
 * 32-bit coordinates, so a column or row from 2^31 on counts as outside the
 * grid.
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
 * The scan keys each diagonal step on its 2 by 2 block and its slope,
 * and ignores a revisit within PATH_LOOP_SPUR_WINDOW steps. A diagonal step
 * moves by one cell along each axis. A step between two cells that are not
 * neighbors, which only a stretch outside the grid leaves, crosses no block.
 * After a hit, the scan forgets every earlier visit and restarts from the
 * current cell. A path that runs back along itself for two hundred cells
 * therefore counts as one event, not as two hundred.
 *
 * @param scratch The scratch whose @c xs and @c ys hold the cells to scan.
 * Only the first min(xs.size(), ys.size()) cells count. The scan updates the
 * rest of the scratch.
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
 * @brief Reports whether a path revisits a cell or crosses itself between
 * two diagonal steps.
 *
 * The scan stops at the first event. The function finds the shapes of
 * PathLoopKind only. It does not test the clearance between two parts of the
 * path (see PathLoopKind).
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
