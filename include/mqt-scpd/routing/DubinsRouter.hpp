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

#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/routing/BucketQueue.hpp"
#include "mqt-scpd/routing/CrossingConstraints.hpp"
#include "mqt-scpd/routing/Heading.hpp"
#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/Primitives.hpp"
#include "mqt-scpd/routing/SearchScratch.hpp"
#include "mqt-scpd/routing/SelfIntersection.hpp"
#include "mqt-scpd/routing/mqt_scpd_routing_export.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace mqt::scpd::routing {

/**
 * @brief The estimate that route() steers by.
 *
 * routeOrthogonal() always steers by the octile distance, whatever the
 * setting.
 */
enum class Heuristic : uint8_t {
  /// The shortest eight-connected distance to the target over the free cells
  /// of the corridor, from one backward Dijkstra search per route. It follows
  /// the corridor, so a search in a winding corridor does not flood the
  /// corridor. The search also drops every move that ends on a cell from
  /// which the target cannot be reached. The default.
  DistanceField,
  /// The octile distance to the target. It ignores the corridor and needs no
  /// computation before the search.
  Octile,
};

/**
 * @brief A curvature-constrained A* search over cells and eight-way headings
 * whose moves are the primitives of one bend radius.
 *
 * A router reads shared grids and writes only its own scratch. The threading
 * model is therefore one router per thread over the same grids. The obstacle
 * mask and the corridor are bit grids attached by pointer. The router owns
 * the static proximity penalty. The wire proximity penalty is a view. The
 * router packs the corridor and the static proximity into one working byte
 * per cell, so the inner loop touches one cache line per swept cell.
 *
 * A router and its scratch form one per-thread context. The scratch belongs
 * to the caller and outlives the router. Nothing else that a router reads is
 * copied per thread. The primitives are shared by pointer and never bound by
 * reference, so a router outlives whatever built them. A router can be neither
 * copied nor moved. A copy would share the scratch of the original, and a
 * moved-from router would keep the state of tables it no longer holds.
 *
 * The search tests the cells that a move sweeps and its end cell against the
 * corridor, and it tests the cell it starts from. It does not test the cells
 * of the straight stubs against the corridor; the caller keeps them clear of
 * it. The test is coarser than the wire in two ways:
 * - A wall of blocked cells stops the search only where its cells share
 *   edges. A diagonal step sweeps only its end cell, so it passes between two
 *   blocked cells that touch at a corner.
 * - The centreline of an arc can leave the cells its move sweeps. It stays
 *   within half a cell of them for every bend radius up to 29 cells: 0.16
 *   cell at a radius of 5, 0.21 at 12 and 0.49 at 20. Larger radii stray
 *   further, 0.78 cell at 30 and 0.93 at 40.
 *
 * The corridor and the keepout must therefore leave at least one cell beyond
 * the half-width of the wire, for bend radii up to 29 cells.
 */
class MQT_SCPD_ROUTING_EXPORT DubinsRouter {
public:
  /**
   * @brief Creates a router over the grid of a scratch.
   * @param primitives The move primitives, shared with other routers.
   * @param scratch The search records of this thread. The router takes the
   * size of its grid from @p scratch and keeps a reference to it.
   * @param params The search parameters.
   * @pre @p scratch outlives the router, and nothing moves from it while the
   * router holds it.
   * @throws std::invalid_argument If @p primitives is null, if the grid of
   * @p scratch has no cells or more than 65535 cells along an axis, if the
   * bend radius of @p params is not the one the primitives were built with,
   * or if the primitives do not fit the search tables. The primitives do not
   * fit when a heading has more than 16 moves, when a move covers more than
   * 60 cells with its start and end cell, when a move reaches more than 127
   * cells from its start along an axis, or when two moves of one heading
   * sweep the same cells in the same order.
   */
  DubinsRouter(std::shared_ptr<const MovePrimitives> primitives,
               SearchScratch& scratch, SearchParams params = {});

  DubinsRouter(const DubinsRouter&) = delete;
  DubinsRouter& operator=(const DubinsRouter&) = delete;
  DubinsRouter(DubinsRouter&&) = delete;
  DubinsRouter& operator=(DubinsRouter&&) = delete;
  ~DubinsRouter() = default;

  /**
   * @brief Returns the width of the router grid.
   * @return The number of cells along the x axis.
   */
  [[nodiscard]] uint32_t width() const { return gridWidth; }

  /**
   * @brief Returns the height of the router grid.
   * @return The number of cells along the y axis.
   */
  [[nodiscard]] uint32_t height() const { return gridHeight; }

  /**
   * @brief Returns the number of cells of the router grid.
   * @return The width times the height.
   */
  [[nodiscard]] std::size_t cells() const {
    return static_cast<std::size_t>(gridWidth) * gridHeight;
  }

  /**
   * @brief Counts the bytes the router holds.
   *
   * The count is the size of the router object, plus every container the
   * router owns by its capacity, plus the bytes the crossing constraints
   * hold. It leaves out the scratch, which belongs to the caller, and the
   * hash maps of the self-intersection test, which follow the length of the
   * last path rather than the grid.
   *
   * @return The number of bytes.
   */
  [[nodiscard]] std::size_t heldBytes() const;

  /**
   * @brief Returns the move primitives of the router.
   * @return The primitives the router was created with.
   */
  [[nodiscard]] const MovePrimitives& primitives() const {
    return *movePrimitives;
  }

  /**
   * @brief Returns the search parameters.
   * @return The parameters set last.
   */
  [[nodiscard]] const SearchParams& params() const { return searchParams; }

  /**
   * @brief Changes the search parameters.
   *
   * Only a changed bend penalty rebuilds the search tables.
   *
   * @param params The new parameters.
   * @throws std::invalid_argument If the bend radius of @p params is not the
   * one the primitives were built with.
   */
  void setParams(const SearchParams& params);

  /**
   * @brief Selects the estimate that route() steers by.
   *
   * routeOrthogonal() always steers by the octile distance.
   *
   * @param heuristic The estimate.
   */
  void setHeuristic(const Heuristic heuristic) { activeHeuristic = heuristic; }

  /**
   * @brief Selects whether route() adds the unavoidable turning toward the
   * target heading to its heuristic.
   *
   * A way has to arrive on the target heading, and every eighth turn it still
   * owes costs one bend penalty. The term is therefore @c bendPenalty times
   * the cyclic distance from the heading of a state to the target heading. It
   * is a lower bound on the turning that the state still has to pay, and it
   * cuts the number of states the search expands. routeOrthogonal() always
   * adds the term, whatever this setting.
   *
   * The term is on by default. It is consistent on its own: a move pays at
   * least the bend penalty times the heading distance it turns, and the
   * cyclic heading distance obeys the triangle inequality. The distance term
   * is the part that can overestimate. A cardinal eighth turn costs about
   * 95 % of the octile distance of its end, 421 against 441 at a bend radius
   * of 5 and 973 against 1023 at a radius of 12. Without the bend term, the
   * bend penalty the turn pays covers that excess when the penalty is at
   * least as large. With the bend term, the estimate already holds the
   * penalty, so it can exceed what a state still has to pay, and the search
   * never reopens a closed state. route() is therefore close to optimal but
   * not exactly optimal. A comparison with a brute-force search on 1500
   * random grids at a radius of 5 and a bend penalty of 500 pins this: without
   * the bend term every way is the cheapest; with it, fewer than one way in a
   * hundred costs more, and none by more than 20, the excess of one cardinal
   * eighth turn.
   *
   * @param on Whether route() adds the term.
   */
  void setBendLowerBound(const bool on) { bendLowerBound = on; }

  /**
   * @brief Returns the estimate that route() steers by.
   * @return The heuristic set last, Heuristic::DistanceField by default.
   */
  [[nodiscard]] Heuristic heuristic() const { return activeHeuristic; }

  // --- Grids -------------------------------------------------------------

  /**
   * @brief Attaches the static obstacles, shared and read-only.
   *
   * The static proximity and the free strip read the obstacles. The search
   * itself does not read them; it reads the corridor.
   *
   * @param obstacles The obstacle mask, in which a set bit is an obstacle
   * cell, or @c nullptr to detach the obstacles.
   * @pre @p obstacles stays alive while it is attached.
   * @throws std::invalid_argument If @p obstacles does not have the width and
   * the height of the router grid.
   */
  void attachObstacles(const grid::BitGrid* obstacles);

  /**
   * @brief Returns the attached obstacles.
   * @return The obstacle mask, or @c nullptr when none is attached.
   */
  [[nodiscard]] const grid::BitGrid* obstacles() const { return obstacleMask; }

  /**
   * @brief Attaches the corridor of the wire about to be routed.
   *
   * The search enters the clear cells of the corridor only. The function
   * packs the corridor with the static proximity into the working grid, which
   * costs one pass over the grid.
   *
   * The packed grid is a copy of the corridor at the time of the call. After
   * a change to the corridor, attach the corridor again.
   *
   * @param corridor The corridor, in which a set bit blocks its cell, or
   * @c nullptr to detach the corridor.
   * @pre @p corridor stays alive while it is attached, and it does not change
   * after the call.
   * @throws std::invalid_argument If @p corridor does not have the width and
   * the height of the router grid.
   */
  void attachCorridor(const grid::BitGrid* corridor);

  /**
   * @brief Attaches the corridor without the packing pass.
   *
   * The search then reads the corridor bits and the proximity bytes
   * separately. Each search is slower, but the attachment costs nothing. This
   * suits a fresh corridor for each of many short searches. A change of the
   * static proximity does not pack the corridor either, so every search reads
   * the corridor as it is at the time of the search.
   *
   * @param corridor The corridor, in which a set bit blocks its cell, or
   * @c nullptr to detach the corridor.
   * @pre @p corridor stays alive while it is attached.
   * @throws std::invalid_argument If @p corridor does not have the width and
   * the height of the router grid.
   */
  void attachCorridorUnpacked(const grid::BitGrid* corridor);

  /**
   * @brief Returns the attached corridor.
   * @return The corridor, or @c nullptr when none is attached.
   */
  [[nodiscard]] const grid::BitGrid* corridor() const { return corridorMask; }

  /**
   * @brief Tests whether the attached corridor blocks a cell.
   * @param x The column of the cell.
   * @param y The row of the cell.
   * @pre The cell lies in the router grid.
   * @return @c true when a corridor is attached and blocks the cell, and
   * @c false otherwise.
   */
  [[nodiscard]] bool corridorBlocked(uint32_t x, uint32_t y) const;

  /**
   * @brief Sets the static proximity penalty of every cell.
   *
   * The limit of @c 127 leaves the high bit of the working byte for the
   * corridor.
   *
   * @param penalty One penalty per cell in row-major order, each from @c 0 to
   * @c 127.
   * @post The working grid is packed again when attachCorridor() attached the
   * corridor.
   * @throws std::invalid_argument If @p penalty does not have one entry per
   * cell, or if an entry is above @c 127.
   */
  void setStaticProximity(std::vector<uint8_t> penalty);

  /**
   * @brief Returns the static proximity penalty of every cell.
   * @return One penalty per cell in row-major order, each from @c 0 to
   * @c 127.
   */
  [[nodiscard]] const std::vector<uint8_t>& staticProximity() const {
    return staticPenalties;
  }

  /**
   * @brief Computes the static proximity from the attached obstacles.
   *
   * An obstacle cell carries the full penalty. Around the obstacles, the
   * penalty decays linearly to one over @p distance cells of four-connected
   * growth. Every other cell carries no penalty. Without attached obstacles,
   * or with a zero @p distance or @p penalty, no cell carries a penalty. The
   * growth needs one byte per cell while it runs and frees it afterwards.
   *
   * @param distance The number of cells of growth around the obstacles.
   * @param penalty The penalty of an obstacle cell, at most @c 127.
   * @post The working grid is packed again when attachCorridor() attached the
   * corridor.
   * @throws std::invalid_argument If @p penalty is above @c 127 while
   * obstacles are attached and @p distance is not zero. The penalties then
   * stay as they were.
   */
  void computeStaticProximity(uint32_t distance, uint8_t penalty);

  /**
   * @brief Recomputes the static proximity inside a window of cells only.
   *
   * Obstacles up to @p distance cells outside the window seed the growth, so
   * the window gets the values that computeStaticProximity() with the same
   * @p distance and @p penalty gives it. The cells outside the window keep
   * their penalties. Without attached obstacles, the function changes no
   * penalty.
   *
   * @param distance The number of cells of growth around the obstacles.
   * @param penalty The penalty of an obstacle cell, at most @c 127.
   * @param window The cells to recompute, both bounds included. The function
   * clips the window to the router grid. A window that lies wholly outside
   * the grid holds no cell, so the function changes no penalty.
   * @pre computeStaticProximity() has run, so that the cells outside the
   * window hold current penalties.
   * @post The working grid is out of date. Until a call packs it again, such
   * as attachCorridor(), route() reads the corridor and the proximity
   * separately.
   * @throws std::invalid_argument If @p penalty is above @c 127 while
   * obstacles are attached, @p window holds a cell of the router grid, and
   * @p distance is not zero. The penalties then stay as they were.
   */
  void computeStaticProximityWindow(uint32_t distance, uint8_t penalty,
                                    CellBox window);

  /**
   * @brief Attaches the wire proximity penalty of every cell.
   *
   * The router reads the penalties through a view and does not copy them.
   *
   * @param penalty One penalty per cell in row-major order, or @c nullptr to
   * detach the wire proximity.
   * @pre @p penalty stays alive while it is attached.
   * @throws std::invalid_argument If @p penalty does not have one entry per
   * cell.
   */
  void attachWireProximity(const std::vector<uint8_t>* penalty);

  /**
   * @brief Returns the static proximity penalty of one cell.
   * @param index The row-major index of the cell.
   * @pre @p index is less than cells().
   * @return The penalty, from @c 0 to @c 127.
   */
  [[nodiscard]] uint8_t staticPenalty(const std::size_t index) const {
    return staticPenalties[index];
  }

  /**
   * @brief Returns the wire proximity penalty of one cell.
   * @param index The row-major index of the cell.
   * @pre @p index is less than cells().
   * @return The penalty, or @c 0 when no wire proximity is attached.
   */
  [[nodiscard]] uint8_t wirePenalty(std::size_t index) const;

  // --- Crossing constraints ---------------------------------------------

  /**
   * @brief Builds the crossing constraints of routeOrthogonal() from routed
   * wires.
   *
   * The cells within @p expandRadius of a straight run remember the heading
   * of the run, so a straight step may cross them at a right angle only. The
   * cells within one cell of a curve, or of the first and last ten cells of a
   * wire, may not be crossed at all. A turn may not touch any of these cells.
   * CrossingConstraints describes the rules.
   *
   * @param wires The routed wires.
   * @param skip One flag per wire. The function leaves out a wire whose flag
   * is set. A wire beyond the end of @p skip counts.
   * @param expandRadius The distance in cells, along each axis, up to which a
   * straight run constrains the cells around it.
   * @post The new constraints replace the previous ones.
   */
  void buildOrthogonalConstraints(const std::vector<Path>& wires,
                                  const std::vector<bool>& skip,
                                  int expandRadius = 10);

  /**
   * @brief Removes every crossing constraint.
   * @post The crossing constraints permit every cell and heading.
   */
  void clearOrthogonalConstraints();

  /**
   * @brief Sets the one feedline that the orthogonal routes may cross, once.
   *
   * Each orthogonal route builds zones around points of the feedline. A zone
   * covers the cells up to a radius along each axis from its points, on the far
   * side of a line through each point. The far side of a line is the side that
   * holds the target of the route. A line gets no zone when the source lies on
   * the same side, or when the target lies on the line. The rule builds three
   * kinds of zone:
   * - A straight zone surrounds each straight segment of the feedline, with
   *   @p straightRadius and the line of the segment. A straight step may
   *   enter a cell of the zone only on the heading that leaves the feedline
   *   at a right angle, and a turn may not touch the cell at all. A wire that
   *   crosses a straight segment therefore leaves it at a right angle.
   * - A curve zone surrounds one point of each run of points under one tag
   *   that is not a straight run over two or more cells: a turn, or a single
   *   straight step. The point is the last of the run, which for a turn is
   *   the last cell its arc sweeps. The line runs through that point along
   *   its heading, which for a turn is the heading the turn starts on. The
   *   zone has @p curveRadius. It does not cover the cells beside the rest
   *   of the arc.
   * - A head zone surrounds each of the first ten points of the feedline,
   *   with @p curveRadius. The line of a point runs along the step to the
   *   next of these points, or from the previous one for the last of them.
   *
   * No route may enter a cell of a curve zone or a head zone. A cell in the
   * straight zones of two segments with different exit headings may not be
   * entered either. A cell in a straight zone and in a curve zone or a head
   * zone keeps the rule of the straight zone.
   *
   * The setting holds for every orthogonal route until the next call. The
   * rule applies only when @p straightRadius is positive and @p feedline has
   * at least two cells.
   *
   * @param feedline The feedline, or @c nullptr for no single-crossing rule.
   * @param straightRadius The distance in cells, along each axis, up to which
   * a straight segment of the feedline constrains the far side.
   * @param curveRadius The distance in cells, along each axis, up to which a
   * curve zone or a head zone blocks the far side.
   * @pre @p feedline stays alive while it is set.
   */
  void setSingleCrossingFeedline(const Path* feedline, int straightRadius,
                                 int curveRadius);

  /**
   * @brief Exempts cells from the crossing rules of the orthogonal routes.
   *
   * An exempt cell passes the crossing tests of straight steps and turns:
   * neither the crossing constraints nor the single-crossing rule bind it.
   * The exemption suits the room around a coupler, where wires pin and run
   * beside the coupler on purpose.
   *
   * @param cells The row-major indices of the cells to exempt. The function
   * ignores an index outside the grid. An empty list removes the exemption.
   * @post The new exemption replaces the previous one.
   */
  void setCrossingExemption(const std::vector<uint32_t>& cells);

  /**
   * @brief Tests whether a straight step may enter a cell under a heading.
   *
   * This is the test that routeOrthogonal() runs, with the heading of the
   * step, on each cell that a straight step of the search or of a straight
   * stub enters. turnAllowedOrthogonal() is the test of a turn. A check of a
   * routed path asks the same questions: this test for each cell that a
   * straight step enters, and turnAllowedOrthogonal() for each cell of a
   * turn, from the start of its arc to its end. The tags of the path show
   * the turns, with one exception (see Path). Where the search began with a
   * turn, the arc starts on the last cell of the source stub, which keeps
   * the straight tag of the stub. The tags alone do not show this: a search
   * that begins with a straight step of one cell and then turns gives the
   * same tags. The arc starts on the last cell of the stub when the point
   * after it carries a turn tag and the arc ends at that cell plus the
   * offset of the move of the tag.
   *
   * The single-crossing rule exists only while routeOrthogonal() runs, so
   * outside a search the test reads the exemption and the crossing
   * constraints only. The test does not read the corridor.
   *
   * @param x The column of the cell.
   * @param y The row of the cell.
   * @param heading The heading of the step.
   * @return @c true when the step may enter the cell, and @c false for a
   * cell outside the router grid.
   */
  [[nodiscard]] bool crossingAllowedOrthogonal(uint32_t x, uint32_t y,
                                               Heading heading) const;

  /**
   * @brief Tests whether a turn may touch a cell.
   *
   * This is the test that the orthogonal search runs on every cell of a
   * turn: the cell the turn starts on, every cell it sweeps, and its end. The
   * heading of an arc changes along it, so a turn passes only a cell that no
   * crossing rule constrains, or an exempt cell. Outside a search, the test
   * reads the exemption and the crossing constraints only, as
   * crossingAllowedOrthogonal() does. The test does not read the corridor.
   *
   * @param x The column of the cell.
   * @param y The row of the cell.
   * @return @c true when a turn may touch the cell, and @c false for a cell
   * outside the router grid.
   */
  [[nodiscard]] bool turnAllowedOrthogonal(uint32_t x, uint32_t y) const;

  /**
   * @brief Returns the constraint mask of a cell.
   * @param x The column of the cell.
   * @param y The row of the cell.
   * @return @c 0 for a free cell, @c 0xFF for a curve or pin zone, or else
   * the set of the headings of the straight runs present there, bit @c h for
   * heading @c h. The mask is also @c 0 for a cell outside the router grid
   * and when no constraints are built.
   */
  [[nodiscard]] uint8_t constraintMaskAt(uint32_t x, uint32_t y) const;

  /**
   * @brief Returns the crossing constraints themselves.
   *
   * A check that reads them reports what the search refused.
   *
   * @return The constraints that buildOrthogonalConstraints() built last, or
   * empty constraints after clearOrthogonalConstraints(). They include
   * neither the exemption nor the single-crossing rule.
   */
  [[nodiscard]] const CrossingConstraints& crossingConstraints() const {
    return constraints;
  }

  // --- Searches -----------------------------------------------------------

  /**
   * @brief Routes a wire from the source to the target inside the corridor.
   *
   * The search runs between the two points that sanitize() computes. The
   * result starts at the source, runs its straight stubs, and ends at the
   * target. When the two points are the same cell on the same heading, the
   * result is the two stubs joined, with their shared cell once.
   *
   * The way is close to the cheapest, but with the bend lower bound it can
   * cost slightly more; setBendLowerBound() gives the reason and the figures.
   *
   * @param objective The source and the target of the wire.
   * @param usePenalty Whether the search adds the static and wire proximity
   * penalties.
   * @param onlyStraight Whether the search forbids moves that end on a
   * diagonal heading.
   * @pre A corridor is attached. With @p usePenalty, a wire proximity is
   * attached too. The source and the target lie in the router grid.
   * @return The path, or an empty path when no path exists, when a search
   * end lies outside the router grid, when the corridor blocks the cell the
   * search starts from, or when the found path crossed itself. A rejected
   * path counts in loopGuardRejections().
   * @throws std::logic_error If no corridor is attached, or if @p usePenalty
   * is set and no wire proximity is attached.
   * @throws std::invalid_argument If the heading of the source or of the
   * target is @c NUM_HEADINGS or more.
   */
  [[nodiscard]] Path route(const RoutingObjective& objective,
                           bool usePenalty = false, bool onlyStraight = false);

  /**
   * @brief Routes a wire as route() does, but crosses constrained wires at
   * right angles only.
   *
   * The search obeys the crossing constraints, the exemption and the single
   * crossing feedline. A straight step may enter a constrained cell only on
   * a heading that crossingAllowedOrthogonal() admits. A turn may not touch
   * a constrained cell at all, from the cell it starts on to its end (see
   * turnAllowedOrthogonal()). The straight stubs are fixed, so the function
   * tests them before the search: each cell that a step of a stub enters
   * must pass the test of a straight step on the heading of the stub. A wire
   * therefore crosses a feedline on straight steps only, at a right angle in
   * the rendered geometry. The search steers by the octile distance plus the
   * bend lower bound, whatever heuristic() and setBendLowerBound() select.
   * The search does not enter the cell it starts from, so no step of the
   * search tests that cell as a straight step; the last step of the source
   * stub does. A turn from that cell tests it as a turn, and the corridor
   * test reads it too.
   *
   * @param objective The source and the target of the wire.
   * @param usePenalty Whether the search adds the static and wire proximity
   * penalties.
   * @param onlyStraight Whether the search forbids moves that end on a
   * diagonal heading.
   * @pre A corridor is attached. With @p usePenalty, a wire proximity is
   * attached too. The source and the target lie in the router grid.
   * @return The path, or an empty path when no path exists, when a search
   * end lies outside the router grid, when the corridor blocks the cell the
   * search starts from, when a cell of a stub fails its crossing test, or
   * when the found path crossed itself. A rejected path counts in
   * loopGuardRejections(). When the two search ends are the same cell on the
   * same heading, the path is the two stubs joined, with their shared cell
   * once.
   * @throws std::logic_error If no corridor is attached, or if @p usePenalty
   * is set and no wire proximity is attached.
   * @throws std::invalid_argument If the heading of the source or of the
   * target is @c NUM_HEADINGS or more.
   */
  [[nodiscard]] Path routeOrthogonal(const RoutingObjective& objective,
                                     bool usePenalty = false,
                                     bool onlyStraight = false);

  /**
   * @brief Counts the searches that rejected their path because it crossed
   * itself.
   * @return The number of rejected searches since the router was created.
   */
  [[nodiscard]] uint64_t loopGuardRejections() const {
    return loopGuardRejectionCount;
  }

  // --- Helpers for the consumers of a route ------------------------------

  /**
   * @brief Computes the point a search starts or ends at.
   *
   * A source moves forward along its heading by the start straight length. A
   * target moves back along its heading by the end straight length. The
   * result is not clamped to the grid; a point off the grid has a coordinate
   * at or above the width or the height.
   *
   * @param point The source or the target.
   * @param isTarget Whether @p point is the target.
   * @return The moved point, with the three low bits of the heading of
   * @p point and primitive @c 0.
   */
  [[nodiscard]] PathPoint sanitize(PathPoint point, bool isTarget) const;

  /**
   * @brief Lists the cells of a straight stub.
   *
   * The stub of a source runs forward along its heading. The stub of a target
   * runs back against its heading. Every cell carries the straight primitive
   * of the heading. The cells are not clamped to the grid.
   *
   * @param point The source or the target.
   * @param isTarget Whether @p point is the target.
   * @param length The length of the stub, in cells.
   * @return The @p length + 1 cells of the stub, from @p point outward.
   */
  [[nodiscard]] Path straightStub(PathPoint point, bool isTarget,
                                  uint32_t length) const;

  /**
   * @brief Computes the widest obstacle-free strip along the segment between
   * two cells.
   *
   * The function grows the segment to either side, one cell of offset at a
   * time. A step of growth covers every cell whose centre lies between the
   * previous edge of the strip and the new one and whose projection onto the
   * line of the segment falls on the segment. A side stops growing when a
   * covered cell is an obstacle or when an end of the new edge leaves the
   * grid. Each side grows on its own. The cells on the line of the segment
   * itself are not tested.
   *
   * @param from The first end of the segment.
   * @param to The second end of the segment.
   * @return The bounding box of the strip, clamped to the grid. The box is
   * the whole grid when no obstacles are attached, and the single cell
   * @p from when the two ends coincide.
   */
  [[nodiscard]] CellBox freeStripAlong(PathPoint from, PathPoint to) const;

private:
  /** @brief An entry of the open list: a state and its costs. */
  struct QueueEntry {
    /// The column of the state.
    uint16_t x = 0;
    /// The row of the state.
    uint16_t y = 0;
    /// The heading of the state.
    uint8_t heading = 0;
    /// The cost so far plus the heuristic, the priority in the open list.
    uint32_t f = 0;
    /// The cost so far.
    uint32_t g = 0;
  };

  /**
   * @brief A node of the swept-cell trie of one heading.
   *
   * The trie holds the cells that the primitives of the heading sweep,
   * merged by common prefix, in preorder with a subtree skip. An expansion
   * walks the shared cells once and drops every primitive behind a blocked
   * cell in one step.
   */
  struct TrieNode {
    /// The offset of the cell from the cell of the state, as a row-major
    /// index difference.
    int32_t linear = 0;
    /// The offset of the cell from the cell of the state along the x axis.
    int8_t dx = 0;
    /// The offset of the cell from the cell of the state along the y axis.
    int8_t dy = 0;
    /// The moves that sweep the cell, bit @c i for move @c i of the heading.
    uint16_t primitives = 0;
    /// The move whose cells end at this node, or NO_PRIMITIVE.
    uint8_t completes = NO_PRIMITIVE;
    /// The depth of the node, @c 0 for a root.
    uint8_t depth = 0;
    /// The size of the subtree of the node, which a blocked cell skips.
    uint16_t skip = 0;
  };

  /** @brief A move of one heading as the search tables hold it. */
  struct TriePrimitive {
    /// The cost of the move plus the bend penalty of its turn, in hundredths
    /// of a cell.
    uint32_t costBend = 0;
    /// The end of the move along the x axis, relative to its start.
    int16_t endDx = 0;
    /// The end of the move along the y axis, relative to its start.
    int16_t endDy = 0;
    /// The identifier of the move.
    uint16_t id = 0;
    /// The number of cells the move sweeps.
    uint16_t sweptCount = 0;
    /// The heading at the end of the move.
    uint8_t exit = 0;
    /// @c 1 when the swept cells include the start cell. The trie holds no
    /// start cell, so the expansion adds its penalty separately.
    uint8_t sweepsOrigin = 0;
    /// @c 1 when the swept cells end with the end cell. The penalty of a move
    /// is the sum over its swept cells plus the end cell, so the end cell
    /// then counts twice.
    uint8_t endExtra = 0;
  };
  /// The value of TrieNode::completes for a node that ends no move.
  static constexpr uint8_t NO_PRIMITIVE = 0xFF;
  /// The most moves per heading that the tables hold, one bit each.
  static constexpr uint32_t MAX_PRIMITIVES_PER_HEADING = 16;
  /// The most cells of one move, with its start and end cell, that the
  /// tables hold.
  static constexpr uint32_t MAX_TRIE_DEPTH = 60;

  /// The low bits of a distance-field entry that hold the distance. The high
  /// bits hold the stamp of the search that wrote the entry.
  static constexpr uint32_t FIELD_DISTANCE_BITS = 22;
  /// The mask of the distance in a distance-field entry, which is also the
  /// largest distance.
  static constexpr uint32_t FIELD_DISTANCE_MASK =
      (1U << FIELD_DISTANCE_BITS) - 1U;
  /// The number of buckets of the distance-field search.
  static constexpr uint32_t FIELD_BUCKETS = 1024;
  /// The rule bit of a cell on the far side of the single crossing feedline.
  static constexpr uint8_t SIDE_FAR = 0x80;
  /// The rule bit of a far-side cell that a route may enter on one heading.
  /// The bits SIDE_HEADING hold that heading.
  static constexpr uint8_t SIDE_STRAIGHT = 0x40;
  /// The rule bits of the heading of a SIDE_STRAIGHT cell.
  static constexpr uint8_t SIDE_HEADING = 0x07;
  /// The rule bits of the single-crossing overlay.
  static constexpr uint8_t SIDE_BITS = SIDE_FAR | SIDE_STRAIGHT | SIDE_HEADING;
  /// The rule bit of a cell that the crossing constraints constrain.
  static constexpr uint8_t CONSTRAINED = 0x08;
  /// The rule bit of a cell exempt from the crossing rules.
  static constexpr uint8_t EXEMPT = 0x10;
  /// The rule bits of a cell where a crossing rule can refuse a step. A cell
  /// without them passes the crossing tests of a straight step and of a turn.
  static constexpr uint8_t RULED = CONSTRAINED | SIDE_FAR;

  /**
   * @brief Builds the swept-cell tries and the move costs of every heading.
   * @throws std::invalid_argument If the primitives do not fit the tables.
   */
  void buildTables();

  /**
   * @brief Packs the corridor and the static proximity into the working
   * grid. Does nothing without a corridor.
   */
  void rebuildPacked();

  /**
   * @brief Computes the distance field toward a target cell over the free
   * cells of the corridor.
   * @param tx The column of the target cell.
   * @param ty The row of the target cell.
   * @pre A corridor is attached, and the target cell lies in the router
   * grid.
   */
  void buildDistanceField(uint32_t tx, uint32_t ty);

  /** @brief Allocates the crossing rules of every cell, unless they exist. */
  void holdCrossingRules();

  /**
   * @brief Releases the crossing rules of every cell when no crossing
   * constraints, exemption or single crossing feedline are set.
   */
  void releaseUnusedCrossingRules();

  /**
   * @brief Marks the far side of the single crossing feedline for one route.
   * @param source The source of the route.
   * @param target The target of the route.
   */
  void beginSingleCrossingOverlay(const PathPoint& source,
                                  const PathPoint& target);

  /** @brief Clears the cells that the single-crossing overlay marked. */
  void endSingleCrossingOverlay();

  /**
   * @brief Tests the cells of a straight stub with the crossing test of a
   * straight step.
   * @param from The cell the stub starts from, with the heading of the stub.
   * @param length The length of the stub, in cells.
   * @return @c true when each of the @p length cells after @p from, along
   * its heading, passes canCrossOrthogonal() on that heading.
   */
  [[nodiscard]] bool stubCrossingAllowed(const PathPoint& from,
                                         uint32_t length) const;

  /**
   * @brief Computes the index of a state in the scratch.
   * @param x The column of the state.
   * @param y The row of the state.
   * @param heading The heading of the state.
   * @return The index.
   */
  [[nodiscard]] uint32_t stateIndex(const uint32_t x, const uint32_t y,
                                    const Heading heading) const {
    return searchScratch->index(x, y, heading);
  }

  /**
   * @brief Computes the cell and the heading of a state.
   * @param index The index of the state.
   * @return The point of the state, with primitive @c 0.
   */
  [[nodiscard]] PathPoint unpackState(uint32_t index) const;

  /**
   * @brief Computes the state that a state was reached from.
   * @param index The index of the state.
   * @return The index of the parent state.
   */
  [[nodiscard]] uint32_t parentState(uint32_t index) const;

  /**
   * @brief Lists the cells of the way from the start state to the goal
   * state.
   * @param goalIndex The index of the goal state.
   * @param startIndex The index of the start state.
   * @return The cells from the start state on. The goal state is left out,
   * because it belongs to the target stub.
   */
  [[nodiscard]] Path reconstruct(uint32_t goalIndex, uint32_t startIndex) const;

  /**
   * @brief Joins the source stub, the searched way and the target stub.
   * @param searched The way the search found, as reconstruct() lists it. It
   * is empty when the search start is the search goal.
   * @param source The source of the wire.
   * @param target The target of the wire.
   * @return The whole path. The source stub keeps its last point, so a turn
   * at the search start starts on a point with the tag of the stub. Without a
   * searched way, the path is the two stubs joined, with their shared cell
   * once.
   */
  [[nodiscard]] Path assemble(const Path& searched, const PathPoint& source,
                              const PathPoint& target) const;

  /**
   * @brief Tests whether a point lies in the router grid.
   * @param point The point to test.
   * @return @c true when both coordinates are inside the grid.
   */
  [[nodiscard]] bool inGrid(const PathPoint& point) const {
    return point.x < gridWidth && point.y < gridHeight;
  }

  /**
   * @brief Runs the crossing test of the orthogonal search for a straight
   * step.
   * @param x The column of the cell.
   * @param y The row of the cell.
   * @param heading The heading of the step.
   * @return @c true when the step may enter the cell.
   */
  [[nodiscard]] bool canCrossOrthogonal(uint32_t x, uint32_t y,
                                        Heading heading) const;

  /**
   * @brief Runs the crossing test of the orthogonal search for a turn.
   *
   * A cell that passes this test also passes canCrossOrthogonal() on every
   * heading.
   *
   * @param x The column of the cell.
   * @param y The row of the cell.
   * @return @c true when a turn may touch the cell.
   */
  [[nodiscard]] bool canTurnOrthogonal(uint32_t x, uint32_t y) const;

  /**
   * @brief Runs a search and rejects a path that crosses itself.
   * @tparam Search A callable that returns a Path.
   * @param search The search to run.
   * @return The path, or an empty path when it crosses itself.
   */
  template <typename Search> Path guarded(const Search& search);

  /**
   * @brief Runs the search of route().
   * @tparam PACKED Whether the search reads the packed working grid.
   * @tparam USE_PENALTY Whether the search adds the proximity penalties.
   * @param objective The moved source and target the search runs between.
   * @param source The source of the wire, for its stub.
   * @param target The target of the wire, for its stub.
   * @param onlyStraight Whether the search forbids diagonal exit headings.
   * @return The path, or an empty path when no path exists.
   */
  template <bool PACKED, bool USE_PENALTY>
  Path searchFree(const RoutingObjective& objective, const PathPoint& source,
                  const PathPoint& target, bool onlyStraight);

  /**
   * @brief Pushes the successors of a state of the search of route().
   * @tparam PACKED Whether the search reads the packed working grid.
   * @tparam USE_PENALTY Whether the search adds the proximity penalties.
   * @param current The state to expand.
   * @param objective The moved source and target the search runs between.
   * @param onlyStraight Whether the search forbids diagonal exit headings.
   * @param blockMap The packed working grid, or empty when the search reads
   * the corridor instead.
   * @param penaltyMap The packed working grid or the static proximity.
   * @param blockMask The mask of the corridor bit in @p blockMap.
   * @param penaltyMask The mask of the penalty bits in @p penaltyMap.
   * @param wireMap The wire proximity, or empty.
   * @param useField Whether the heuristic reads the distance field.
   */
  template <bool PACKED, bool USE_PENALTY>
  void expandFree(const QueueEntry& current, const RoutingObjective& objective,
                  bool onlyStraight, std::span<const uint8_t> blockMap,
                  std::span<const uint8_t> penaltyMap, uint8_t blockMask,
                  uint8_t penaltyMask, std::span<const uint8_t> wireMap,
                  bool useField);

  /**
   * @brief Runs the search of routeOrthogonal().
   * @tparam USE_PENALTY Whether the search adds the proximity penalties.
   * @param objective The moved source and target the search runs between.
   * @param source The source of the wire, for its stub.
   * @param target The target of the wire, for its stub.
   * @param onlyStraight Whether the search forbids diagonal exit headings.
   * @return The path, or an empty path when no path exists.
   */
  template <bool USE_PENALTY>
  Path searchOrthogonal(const RoutingObjective& objective,
                        const PathPoint& source, const PathPoint& target,
                        bool onlyStraight);

  /**
   * @brief Pushes the successors of a state of the search of
   * routeOrthogonal().
   * @tparam USE_PENALTY Whether the search adds the proximity penalties.
   * @param current The state to expand.
   * @param objective The moved source and target the search runs between.
   * @param onlyStraight Whether the search forbids diagonal exit headings.
   */
  template <bool USE_PENALTY>
  void expandOrthogonal(const QueueEntry& current,
                        const RoutingObjective& objective, bool onlyStraight);

  /// The move primitives, shared with other routers.
  std::shared_ptr<const MovePrimitives> movePrimitives;
  /// The search records of this thread, owned by the caller.
  SearchScratch* searchScratch;
  /// The search parameters.
  SearchParams searchParams;
  /// The number of cells along the x axis.
  uint32_t gridWidth;
  /// The number of cells along the y axis.
  uint32_t gridHeight;
  /// The estimate that route() steers by.
  Heuristic activeHeuristic = Heuristic::DistanceField;
  /// Whether route() adds the bend lower bound.
  bool bendLowerBound = true;
  /// Per heading, what a state facing it still owes in turning, in the cost
  /// units of the search. Filled once per search of route(); all zeroes when
  /// the bend lower bound is off, so the expansion adds it unconditionally.
  std::array<uint32_t, 8> bendBoundByHeading{};

  /// The attached obstacles, or null.
  const grid::BitGrid* obstacleMask = nullptr;
  /// The attached corridor, or null.
  const grid::BitGrid* corridorMask = nullptr;
  /// The static proximity penalty of every cell, from 0 to 127.
  std::vector<uint8_t> staticPenalties;
  /// The attached wire proximity, or null.
  const std::vector<uint8_t>* wireProximity = nullptr;
  /// The working grid: per cell, the corridor in the high bit and the static
  /// proximity in the low seven bits.
  std::vector<uint8_t> packedGrid;
  /// Whether the working grid matches the corridor and the static proximity.
  bool packedValid = false;
  /// Whether attachCorridor() attached the corridor, so that a change of the
  /// static proximity packs the working grid again.
  bool keepPacked = false;

  /// The crossing constraints of the orthogonal search.
  CrossingConstraints constraints;
  /// Per cell, the crossing rules of the orthogonal search in one byte:
  /// CONSTRAINED when the crossing constraints constrain the cell, EXEMPT
  /// when the cell is exempt, and during a search the single-crossing
  /// overlay in SIDE_BITS. The overlay bits are 0 for no rule, SIDE_FAR alone
  /// for a blocked cell, or SIDE_FAR and SIDE_STRAIGHT with the one heading
  /// a route may enter on. The search reads one byte to learn whether any
  /// rule applies to a cell. Empty while no crossing constraints, exemption
  /// or single crossing feedline are set.
  std::vector<uint8_t> crossingRules;
  /// The exempt cells, so that the exemption clears without a pass over the
  /// grid.
  std::vector<uint32_t> exemptCells;
  /// The cells the overlay marked, so that it clears without a pass over the
  /// grid.
  std::vector<uint32_t> crossingSideCells;
  /// The feedline that an orthogonal route may cross once, or null.
  const Path* singleCrossing = nullptr;
  /// The reach of a straight run of the single crossing feedline, in cells.
  int singleCrossingStraightRadius = 0;
  /// The reach of a curve of the single crossing feedline, in cells.
  int singleCrossingCurveRadius = 1;

  /// Per heading, the swept-cell trie in preorder.
  std::array<std::vector<TrieNode>, NUM_HEADINGS> trie;
  /// Per heading, the moves of the trie, in the order of the primitives.
  std::array<std::vector<TriePrimitive>, NUM_HEADINGS> triePrimitives;
  /// Per heading, how far a move reaches toward negative x, in cells.
  std::array<uint32_t, NUM_HEADINGS> marginLeft{};
  /// Per heading, how far a move reaches toward positive x, in cells.
  std::array<uint32_t, NUM_HEADINGS> marginRight{};
  /// Per heading, how far a move reaches toward negative y, in cells.
  std::array<uint32_t, NUM_HEADINGS> marginUp{};
  /// Per heading, how far a move reaches toward positive y, in cells.
  std::array<uint32_t, NUM_HEADINGS> marginDown{};

  /// The distance field: per cell, a stamp and a distance.
  std::vector<uint32_t> distanceField;
  /// The stamp of the current distance field.
  uint32_t fieldStamp = 0;
  /// The buckets of the distance-field search, by distance modulo
  /// FIELD_BUCKETS.
  std::array<std::vector<uint32_t>, FIELD_BUCKETS> fieldBuckets;

  /// The open list of the search.
  BucketQueue<QueueEntry> open;
  /// The scratch of the self-intersection test.
  PathLoopScratch loopScratch;
  /// The number of searches that rejected their path because it crossed
  /// itself.
  uint64_t loopGuardRejectionCount = 0;
};

} // namespace mqt::scpd::routing
