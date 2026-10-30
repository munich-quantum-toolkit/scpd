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
#include "mqt-scpd/grid/Bottlenecks.hpp"
#include "mqt-scpd/grid/CdtGates.hpp"
#include "mqt-scpd/grid/Chambers.hpp"
#include "mqt-scpd/grid/mqt_scpd_grid_export.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace mqt::scpd::grid {

/**
 * @brief One object of a `CdtWorld`: walls that come and go together, such
 * as a feedline edge, or a coupler pad with its lead and its runs.
 *
 * An object is the stamp its walls take of the grid on their own, not what
 * is left of it where other walls took cells first: two objects, or an
 * object and the static walls, may overlap. Their outlines then overlap in
 * the triangulation instead of being joined, and the free space is what
 * none of them covers. They may as well touch, cell beside cell, as the
 * pads, runs, leads and feedline edges of a wall raster do: their outlines
 * then run along one line of the grid, and a corner of one that lies on a
 * piece of the other splits that piece.
 */
struct CdtObject {
  /// Who the object is: the same key always names the same cells and walls.
  /// It is the object's identity in the world's caches, and it orders the
  /// obstacles of different objects where an order is needed, so the result
  /// does not depend on when an object was first seen.
  std::uint64_t key = 0;
  /// Its walls. `group` is an index into this list: walls of one group (the
  /// two flanks of a port run) are one obstacle. A group past the end makes
  /// the wall an obstacle of its own.
  std::vector<CdtWall> walls;
  /// Per wall, whether a gate that ends on it cuts the chambers (the walls
  /// `couplerCuts` of the router counts). Empty: every wall does.
  std::vector<std::uint8_t> cuts;
  /// Its cells, row-major indices of the grid.
  std::vector<std::size_t> cells;
  /// Per cell, the index of its wall in `walls`.
  std::vector<std::uint32_t> wallOf;
};

/// An object as a wall raster names it: its key and the labels its cells
/// carry in that raster (`CdtWorld::state` with a raster).
struct CdtObjectRef {
  std::uint64_t key = 0;
  std::span<const std::uint32_t> labels;
};

/**
 * @brief The object whose walls carry `labels` in a wall raster: the cells
 * of `mask` whose label is one of them, its walls as `walls` has them, and
 * its groups where a group is one of the labels.
 *
 * @param cuts Per label of `walls`, whether a gate that ends on it cuts the
 * chambers; empty: every wall does.
 */
[[nodiscard]] MQT_SCPD_GRID_EXPORT CdtObject
cdtObjectOf(std::uint64_t key, const BitGrid& mask,
            std::span<const std::uint32_t> owner,
            std::span<const CdtWall> walls,
            std::span<const std::uint32_t> labels,
            std::span<const std::uint8_t> cuts = {});

/// What one state of a `CdtWorld` took to build, in milliseconds, and how
/// much it touched.
struct CdtStepStats {
  /// Copying the parent: the triangulation and the arrays.
  double copy = 0.0;
  /// Inserting the points and constraints of the new objects: the foot
  /// points between the new objects and those present (`pairs`), the points
  /// (`vertices`) and the constraints (`edges`), and the rest of the
  /// bookkeeping.
  double insert = 0.0;
  double pairs = 0.0;
  double vertices = 0.0;
  double edges = 0.0;
  /// Finding the triangles the insertion changed and those around the new
  /// objects.
  double patch = 0.0;
  /// Classifying those triangles as free or not.
  double classify = 0.0;
  /// The candidates on their edges.
  double candidates = 0.0;
  /// The basins of the channels the candidates belong to.
  double basins = 0.0;
  /// The wall cells of the gates that came or moved.
  double measure = 0.0;
  /// Labelling the chambers the change touched.
  double chambers = 0.0;
  /// The output: the gates in canonical order, the chambers numbered
  /// canonically, and the chambers beside every gate.
  double output = 0.0;
  double total = 0.0;
  std::size_t objects = 0;
  std::size_t newVertices = 0;
  std::size_t steinerVertices = 0;
  std::size_t changedTriangles = 0;
  std::size_t reclassified = 0;
  std::size_t channels = 0;
  std::size_t channelCandidates = 0;
  std::size_t relabelled = 0;
  /// Whether the state was built from scratch (all objects at once).
  bool scratch = false;
};

/// What the cache of a `CdtWorld` came to over its calls of `state`.
struct CdtWorldCacheStats {
  /// The calls answered by a state in the cache as it was.
  std::size_t hits = 0;
  /// The calls answered by inserting objects into a state of the cache.
  std::size_t extended = 0;
  /// The calls answered by inserting every object into the base.
  std::size_t fromBase = 0;
  /// The objects inserted over all calls.
  std::size_t inserted = 0;
  /// The states built on the way to an answer: the common part of the
  /// objects asked for and a state of the cache that shares more of them
  /// than any state they contain, kept for the passes that share it too.
  std::size_t synthesized = 0;
};

/**
 * @brief The gates and chambers of `cdtGates`, kept on a persistent,
 * canonical constrained Delaunay triangulation that objects are inserted
 * into rather than built anew.
 *
 * The world is built once over the static walls (the base): their outline,
 * simplified and cut into pieces no longer than a clearance, with the foot
 * points among them, triangulated once. Every object (`add`) is outlined
 * once, on its own stamp, with the foot points of the base's corners on it;
 * the foot points of its corners on the base and on other objects are points
 * of their own, a hair off the piece on its free side (the sliver between
 * such a foot and its piece is never free: the foot touches the wall). A
 * state is a set of
 * objects: the base with their outlines inserted. It is built from a state
 * whose objects are a subset of its own by copying that state and inserting
 * the others, and only the triangles that changed, and those around the new
 * objects, are classified, measured and labelled again.
 *
 * Every point is moved by a deterministic amount, so the triangulation of a
 * state is the one Delaunay triangulation of its points and constraints,
 * and every tie is broken by the geometry. The move keeps every row and
 * every column of the grid a line, exactly: x and y are moved apart, each by
 * a hash of its own, on a dyadic step, and then sheared by a power of two
 * (about 1e-3 cells at most), every coordinate exact in a double. Outlines
 * that touch along a row or a column therefore lie on one line, and a
 * corner on another's piece lies on it, instead of a hair off it at an
 * angle the triangulation cannot always resolve. A state therefore is the
 * same whichever states it was built from, in whichever order its objects
 * were inserted: the gates, their order, the chambers and their numbers are
 * a function of the set of objects alone (inserted one by one ≡ all at
 * once, bit for bit). The move is not that of `canonical` in
 * `CdtGateOptions`; widths and ends are measured on the unmoved points.
 *
 * Within a state the rules are those of `cdtGates`: a candidate is an edge
 * between two free triangles whose ends lie on two different obstacles, not
 * both static, not a target run and a static wall, holding fewer than
 * `widest` wires; a gate is the narrowest candidate of its basin. A triangle
 * is free when its centroid lies in a free cell of the raster (the static
 * walls with the stamps of the objects) and outside the outlines of the
 * static walls and of every object. A point takes, besides the obstacles of
 * the outline pieces it lies on, those whose cells touch it.
 *
 * The chambers are the components of the free triangles, cut by the
 * constraints and by the gates that cut (a gate whose wall cells could not
 * be found does not). They are numbered by the lowest centroid (y, then x)
 * of a triangle each holds.
 *
 * A state holds its own triangulation, about 2 to 3 MB on a grid of 4
 * million cells. A state no one holds any more goes back to the world (a
 * few are kept) and the next copy is written over it, which reuses its
 * memory: a step costs half the copy then.
 *
 * Not thread-safe: a world, its cache and its states belong to one thread.
 */
class MQT_SCPD_GRID_EXPORT CdtWorld {
public:
  /// A state: the base with a set of objects. Opaque; ask the world.
  struct State;
  using StatePtr = std::shared_ptr<const State>;

  /// What a state comes to.
  struct Result {
    /// The gates, as `cdtGates` gives them with `canonical`: the two wall
    /// cells nearest its ends, the cell of its middle, sorted by the two
    /// cells, the lower first, and the middle.
    std::vector<Bottleneck> gates;
    /// Per gate, whether it cuts the chambers: one of its wall cells belongs
    /// to a wall that `cuts`.
    std::vector<std::uint8_t> cuts;
    /// Per gate, the wires it holds (`wiresIn` of its width across the
    /// nearest heading).
    std::vector<std::uint32_t> capacity;
    /// Per gate, its length between the two points of the outlines it
    /// joins, in cells.
    std::vector<double> width;
    /// Per gate, its two ends on the outlines, in cells (x0, y0, x1, y1).
    std::vector<std::array<double, 4>> ends;
    /// Per gate, the chambers on either side of it, the lower first; the
    /// second is `NO_CHAMBER` where both sides lie in one chamber (the free
    /// space runs round the gate). `chambers` gives them as `chambersOf`
    /// does, a list per gate.
    std::vector<std::array<std::uint32_t, 2>> beside;
    /// How many chambers there are.
    std::uint32_t chambers = 0;
  };

  /**
   * @brief Builds the world over the static walls: the base state.
   *
   * @param staticMask The cells of the static walls.
   * @param staticOwner The label of every cell, row-major.
   * @param walls The walls by label, as `cdtGates` takes them. A label past
   * its end is the border.
   * @param border The label the cells outside the grid take.
   * @param options As `cdtGates` takes them; `canonical` and `shuffle` are
   * ignored: the world is always canonical.
   * @param cuts Per label, whether a gate that ends on that wall cuts the
   * chambers; empty: every wall does.
   * @param cacheSize How many states `state` keeps.
   */
  CdtWorld(std::uint32_t width, std::uint32_t height,
           const BitGrid& staticMask,
           std::span<const std::uint32_t> staticOwner,
           std::span<const CdtWall> walls, std::uint32_t border,
           const CdtGateOptions& options,
           std::span<const std::uint8_t> cuts = {}, std::size_t cacheSize = 8);
  ~CdtWorld();
  CdtWorld(const CdtWorld&) = delete;
  CdtWorld& operator=(const CdtWorld&) = delete;
  CdtWorld(CdtWorld&&) noexcept;
  CdtWorld& operator=(CdtWorld&&) noexcept;

  /// Adds an object, or finds the one of its key: its index. The outline
  /// and the foot points of an object are found here, once.
  std::uint32_t add(const CdtObject& object);
  /// The index of the object of a key, if it was added.
  [[nodiscard]] std::optional<std::uint32_t> find(std::uint64_t key) const;
  /// How many objects there are.
  [[nodiscard]] std::size_t objectCount() const;

  /// The state of the static walls alone.
  [[nodiscard]] StatePtr base() const;
  /// `parent` with the objects inserted (those it has already are skipped).
  [[nodiscard]] StatePtr with(const StatePtr& parent,
                              std::span<const std::uint32_t> objects) const;
  /// The state of a set of objects built from scratch: every point and
  /// constraint at once, every triangle classified. For tests: a state
  /// built by `with` is the same.
  [[nodiscard]] StatePtr
  fromScratch(std::span<const std::uint32_t> objects) const;
  /// The state of a set of objects, through the cache: a state of the same
  /// set is the answer; else the state of the cache whose objects are the
  /// largest subset (the base if none) with the others inserted, which is
  /// then kept, the least recently used state making room.
  [[nodiscard]] StatePtr state(std::span<const std::uint32_t> objects);
  /// `state` for a wall raster: every object the raster names, added from
  /// the raster the first time its key is seen (`cdtObjectOf`, one pass over
  /// the raster for all new keys). The static walls are those of the world,
  /// whatever the raster holds besides the objects.
  [[nodiscard]] StatePtr state(const BitGrid& mask,
                               std::span<const std::uint32_t> owner,
                               std::span<const CdtWall> walls,
                               std::span<const CdtObjectRef> objects,
                               std::span<const std::uint8_t> cuts = {});

  /// The gates and chambers of a state.
  [[nodiscard]] const Result& result(const State& state) const;
  /// The objects of a state, sorted.
  [[nodiscard]] const std::vector<std::uint32_t>&
  objectsOf(const State& state) const;
  /// What building a state took.
  [[nodiscard]] const CdtStepStats& statsOf(const State& state) const;
  /// What the cache of `state` came to.
  [[nodiscard]] const CdtWorldCacheStats& cacheStats() const;

  /**
   * @brief The chambers of a state as `chambersOf` gives them: the chamber
   * of every cell (the one whose free triangle holds the cell's centre; a
   * wall cell, and a free cell under a triangle that is not free, in none),
   * their count, and per gate of `result` the chambers beside it.
   *
   * Every cell of the grid is written: this costs a pass over the raster.
   * `chamberAt` answers for one cell without it.
   */
  void chambers(const State& state, Chambers& into) const;
  /// The chamber of one cell, as `chambers` has it, by locating the cell's
  /// centre in the triangulation.
  [[nodiscard]] std::uint32_t chamberAt(const State& state, std::uint32_t x,
                                        std::uint32_t y) const;

  /// A canonical description of a state for tests: its free triangles, by
  /// their corners, each with its chamber, sorted; two states with the same
  /// objects give the same.
  [[nodiscard]] std::vector<std::array<double, 7>>
  canonicalTriangles(const State& state) const;

  /// What a state holds, as a line of text: how many vertices, triangles,
  /// constraints, candidates and chambers, and how many entries its arrays
  /// have (for profiling the copy of a state).
  [[nodiscard]] std::string describeState(const State& state) const;

  /// The number of points, constraints and objects of the world, and the
  /// time it took to build, in milliseconds: the outline, the foot points,
  /// and the base state.
  struct BuildStats {
    std::size_t points = 0;
    std::size_t baseVertices = 0;
    std::size_t baseConstraints = 0;
    double outline = 0.0;
    double feet = 0.0;
    double base = 0.0;
  };
  [[nodiscard]] const BuildStats& buildStats() const;
  /// Milliseconds `add` spent on new objects so far, by section: walls,
  /// stamp, squares, the points near it, outline, the outline's index, its
  /// foot points, its corners' foot points, polygon, the pairs with the
  /// objects near it.
  [[nodiscard]] std::array<double, 10> addSeconds() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace mqt::scpd::grid
