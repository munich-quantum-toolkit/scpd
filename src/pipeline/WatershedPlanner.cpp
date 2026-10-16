/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/flatbuffers/artifacts.hpp"
#include "mqt-scpd/flatbuffers/config.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/flatbuffers/geometry.hpp"
#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/Bottlenecks.hpp"
#include "mqt-scpd/grid/Chambers.hpp"
#include "mqt-scpd/grid/DistanceTransform.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/grid/Partitions.hpp"
#include "mqt-scpd/grid/Rasterize.hpp"
#include "mqt-scpd/grid/Voronoi.hpp"
#include "mqt-scpd/grid/Watershed.hpp"
#include "mqt-scpd/pipeline/CapacityPlanner.hpp"
#include "mqt-scpd/pipeline/Report.hpp"
#include "mqt-scpd/pipeline/Stages.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace mqt::scpd::pipeline {
namespace {

namespace fbg = flatbuffers::geometry;
namespace fba = flatbuffers::artifacts;

/// How far the majority filter looks when it smooths a partition border, and
/// how often it repeats. Both are the prototype's.
constexpr int SMOOTHING_RADIUS = 2;
constexpr int SMOOTHING_ROUNDS = 5;

/// The eight steps around a cell, straight ones first. The order is the one
/// `grid::BlockedMove` numbers its directions in, which the walks rely on.
constexpr std::array<int, 8> STEP_X = {0, 0, 1, -1, 1, -1, 1, -1};
constexpr std::array<int, 8> STEP_Y = {1, -1, 0, 0, 1, 1, -1, -1};
static_assert(STEP_X == grid::STEP_DX && STEP_Y == grid::STEP_DY);
constexpr std::size_t FIRST_DIAGONAL_STEP = 4;

/// The extent of a grid, as the artifact carries it.
std::unique_ptr<fba::GridExtentT> extentOf(const grid::GridMetrics& metrics) {
  auto extent = std::make_unique<fba::GridExtentT>();
  extent->width = metrics.width;
  extent->height = metrics.height;
  extent->origin = metrics.origin;
  extent->cell_width = metrics.cellWidth;
  extent->cell_height = metrics.cellHeight;
  return extent;
}

/// The layout point of a cell.
fbg::Point layoutOf(const grid::GridMetrics& grid, const std::size_t cell) {
  const std::size_t row = cell / grid.width;
  return grid.toLayout(static_cast<double>(cell % grid.width),
                       static_cast<double>(row));
}

/// The cells a bottleneck's line runs through, which is where a chain is
/// seeded once the gate is open.
std::vector<std::vector<std::size_t>>
bottleneckLines(const std::vector<grid::Bottleneck>& bottlenecks,
                const grid::GridMetrics& grid) {
  std::vector<std::vector<std::size_t>> lines;
  lines.reserve(bottlenecks.size());
  for (const auto& bottleneck : bottlenecks) {
    lines.push_back(grid::lineCells(
        static_cast<std::int64_t>(bottleneck.first % grid.width),
        static_cast<std::int64_t>(bottleneck.first / grid.width),
        static_cast<std::int64_t>(bottleneck.second % grid.width),
        static_cast<std::int64_t>(bottleneck.second / grid.width), grid.width,
        grid.height));
  }
  return lines;
}

/// Whether two segments cross, for the visibility test below.
bool segmentsCross(const fbg::Point& a0, const fbg::Point& a1,
                   const fbg::Point& b0, const fbg::Point& b1) {
  const auto side = [](const fbg::Point& p, const fbg::Point& q,
                       const fbg::Point& r) {
    const auto value = ((q.x() - p.x()) * (r.y() - p.y())) -
                       ((q.y() - p.y()) * (r.x() - p.x()));
    constexpr double epsilon = 1.0e-9;
    if (value > epsilon) {
      return 1;
    }
    return value < -epsilon ? -1 : 0;
  };
  const auto d1 = side(a0, a1, b0);
  const auto d2 = side(a0, a1, b1);
  const auto d3 = side(b0, b1, a0);
  const auto d4 = side(b0, b1, a1);
  return d1 * d2 < 0 && d3 * d4 < 0;
}

/// Sets of grid cells that a walk starts afresh many times over.
///
/// Every set has a generation, and a cell is in the set whose generation it
/// carries. A new set therefore costs one increment rather than a clear of
/// the grid or of a hash set.
class CellMarks {
public:
  explicit CellMarks(const std::size_t cells) : marks(cells, 0) {}

  /// Starts a new, empty set and returns its generation.
  [[nodiscard]] std::uint32_t next() {
    if (++generation == 0) {
      // The counter wrapped, so old marks could match again.
      std::ranges::fill(marks, 0U);
      generation = 1;
    }
    return generation;
  }

  [[nodiscard]] bool contains(const std::uint32_t set,
                              const std::size_t cell) const {
    return marks[cell] == set;
  }

  /// Puts a cell into a set.
  ///
  /// @return Whether the cell was not in it before.
  bool insert(const std::uint32_t set, const std::size_t cell) {
    if (marks[cell] == set) {
      return false;
    }
    marks[cell] = set;
    return true;
  }

private:
  std::vector<std::uint32_t> marks;
  std::uint32_t generation = 0;
};

/// One set of grid cells that is cleared often.
class CellSet {
public:
  explicit CellSet(const std::size_t cells)
      : marks(cells), current(marks.next()) {}

  void clear() { current = marks.next(); }

  [[nodiscard]] bool contains(const std::size_t cell) const {
    return marks.contains(current, cell);
  }

  void insert(const std::vector<std::size_t>& cells) {
    for (const auto cell : cells) {
      static_cast<void>(marks.insert(current, cell));
    }
  }

private:
  CellMarks marks;
  std::uint32_t current;
};

/// Builds the capacity chains: the trees of gates a launcher's wires pass on
/// the way to the targets it feeds.
class ChainBuilder {
public:
  using NodeList = std::vector<std::unique_ptr<fba::CapacityNodeT>>;

  ChainBuilder(const CapacityScene& scene,
               const std::vector<grid::Bottleneck>& bottlenecks,
               NodeList& nodes)
      : planScene(&scene), gateList(&bottlenecks),
        gateLines(bottleneckLines(bottlenecks, scene.detail)),
        gateMoves(grid::bottleneckMoves(bottlenecks, scene.detail)),
        nodeList(&nodes), gateClosed(bottlenecks.size(), false),
        visitedCells(scene.detail.cells()), floodMarks(scene.detail.cells()) {
    for (std::size_t index = 0; index < scene.targetCell.size(); ++index) {
      portOfTarget.emplace(scene.targetCell[index], scene.targetPort[index]);
    }
    launcherCells.insert(scene.launcherCell.begin(), scene.launcherCell.end());

    // Which gates refuse a move out of each cell, so that a walk can tell in
    // one lookup whether a step crosses one.
    for (std::size_t index = 0; index < gateMoves.size(); ++index) {
      for (const auto& move : gateMoves[index]) {
        gatesOnMove[key(move.cell, move.direction)].push_back(index);
      }
    }
  }

  /// The chain rooted at one target, as the index of its root node.
  [[nodiscard]] std::uint32_t build(const std::size_t targetCell,
                                    const std::uint32_t targetPort) {
    visitedCells.clear();
    servedInWalk.clear();
    openedInWalk.clear();
    servedInWalk.insert(targetCell);
    heldCells.insert(targetCell);
    closeEveryGate();

    const auto root = addNode(fba::CapacityElement::Target, targetPort, 0);
    expand(root, {targetCell}, {});
    openEveryGate();
    return root;
  }

  /// Close every gate that takes part, which is where a walk starts.
  void closeEveryGate() {
    for (std::size_t gate = 0; gate < gateClosed.size(); ++gate) {
      setClosed(gate, true);
    }
  }

  /// Leave nothing closed behind, so the next walk starts from a clean state.
  void openEveryGate() {
    for (std::size_t gate = 0; gate < gateClosed.size(); ++gate) {
      setClosed(gate, false);
    }
  }

  /// Walk one chain again with only its own gates open, writing a partition
  /// label for every chamber the walk enters.
  ///
  /// This is where the partitioning actually comes from. A chamber is the
  /// free space a wire can reach without passing a gate, so it is exactly the
  /// region the border budgets apply within, and every gate the chain crosses
  /// starts a new one. The watershed over the clear capacity cells runs
  /// afterwards and only fills what no chamber claimed.
  void stampChambers(const std::size_t targetCell,
                     const std::vector<std::size_t>& gates,
                     std::vector<grid::PartitionLabel>& labels,
                     grid::PartitionLabel& nextLabel) {
    visitedCells.clear();
    servedInWalk.clear();
    openedInWalk.clear();
    servedInWalk.insert(targetCell);
    // Only the gates this chain kept take part; every other one stays open,
    // so a chamber is bounded by the chain's own constraints and not by a
    // candidate that pruning dropped.
    ownGates = std::unordered_set<std::size_t>(gates.begin(), gates.end());
    labelSink = &labels;
    nextStampLabel = nextLabel;
    closeEveryGate();

    // The target seeds the first chamber and takes its label. A walk whose
    // first chamber reaches a launcher labels nothing.
    static_cast<void>(stamp({targetCell}, {}));

    openEveryGate();
    nextLabel = nextStampLabel;
    labelSink = nullptr;
    ownGates.reset();
  }

  /// The targets some chain has already served.
  [[nodiscard]] const std::unordered_set<std::size_t>& served() const {
    return servedOverall;
  }

private:
  std::uint32_t addNode(const fba::CapacityElement kind, const std::uint32_t id,
                        const std::uint32_t capacity) {
    auto node = std::make_unique<fba::CapacityNodeT>();
    node->kind = kind;
    node->id = id;
    node->capacity = capacity;
    nodeList->push_back(std::move(node));
    return static_cast<std::uint32_t>(nodeList->size() - 1);
  }

  /// Whether a gate is one of the walk's own. Every gate takes part unless a
  /// restricted set says otherwise.
  [[nodiscard]] bool takesPart(const std::size_t gate) const {
    return !ownGates.has_value() || ownGates->contains(gate);
  }

  /// A move, as one number.
  [[nodiscard]] static std::uint64_t key(const std::size_t cell,
                                         const std::size_t direction) {
    return (static_cast<std::uint64_t>(cell) << 3U) | direction;
  }

  /// Close or open one gate: every move it refuses gains or loses a reason to
  /// stay blocked. Counting rather than setting is what lets two gates share
  /// a move without either of them opening it for the other.
  void setClosed(const std::size_t gate, const bool closed) {
    if (gateClosed[gate] == closed || !takesPart(gate)) {
      return;
    }
    gateClosed[gate] = closed;
    for (const auto& move : gateMoves[gate]) {
      auto& count = closedCount[key(move.cell, move.direction)];
      if (closed) {
        ++count;
      } else if (count > 0) {
        --count;
      }
    }
  }

  /// Whether a step out of a cell crosses a closed gate.
  [[nodiscard]] bool crossesAGate(const std::size_t cell,
                                  const std::size_t direction) const {
    const auto found = closedCount.find(key(cell, direction));
    return found != closedCount.end() && found->second > 0;
  }

  /// The gates that refuse a step, whether or not they are closed.
  [[nodiscard]] const std::vector<std::size_t>*
  gatesOn(const std::size_t cell, const std::size_t direction) const {
    const auto found = gatesOnMove.find(key(cell, direction));
    return found == gatesOnMove.end() ? nullptr : &found->second;
  }

  /// One chamber: everything reachable without passing a closed gate.
  struct Chamber {
    std::vector<std::size_t> cells;
    std::vector<std::size_t> gates;
    std::vector<std::pair<std::size_t, std::uint32_t>> targets;
    bool reachesALauncher = false;
  };

  [[nodiscard]] Chamber flood(const std::vector<std::size_t>& seeds) const {
    const auto& grid = planScene->detail;
    Chamber chamber;
    const auto local = floodMarks.next();
    std::queue<std::size_t> pending;

    // A seed is taken even where an earlier chamber already claimed it. The
    // cells beside an opened gate are exactly such cells — they are what the
    // chamber on this side ended at — and refusing them would leave every
    // chamber beyond a gate empty.
    for (const auto seed : seeds) {
      if (planScene->blocked.test(seed) || !floodMarks.insert(local, seed)) {
        continue;
      }
      pending.push(seed);
    }

    std::unordered_set<std::size_t> touchedGates;
    while (!pending.empty()) {
      const auto current = pending.front();
      pending.pop();
      chamber.cells.push_back(current);

      // A launcher ends the chamber: the wires of everything found so far
      // reach the chip through it, and nothing beyond it belongs to this
      // chain.
      if (launcherCells.contains(current)) {
        chamber.reachesALauncher = true;
        break;
      }
      if (const auto target = portOfTarget.find(current);
          target != portOfTarget.end() && !servedInWalk.contains(current)) {
        chamber.targets.emplace_back(current, target->second);
      }

      const auto cx = static_cast<std::int64_t>(current % grid.width);
      const auto cy = static_cast<std::int64_t>(current / grid.width);
      for (std::size_t step = 0; step < STEP_X.size(); ++step) {
        const auto nx = cx + STEP_X[step];
        const auto ny = cy + STEP_Y[step];
        if (nx < 0 || ny < 0 || std::cmp_greater_equal(nx, grid.width) ||
            std::cmp_greater_equal(ny, grid.height)) {
          continue;
        }
        const auto cell = grid.index(static_cast<std::uint32_t>(nx),
                                     static_cast<std::uint32_t>(ny));

        // A gate this step would cross is a branch of the chain, whether or
        // not it is passable from here.
        if (const auto* gates = gatesOn(current, step); gates != nullptr) {
          for (const auto gate : *gates) {
            if (gateClosed[gate] && takesPart(gate) &&
                !openedInWalk.contains(gate)) {
              touchedGates.insert(gate);
            }
          }
        }
        if (floodMarks.contains(local, cell) || visitedCells.contains(cell) ||
            planScene->blocked.test(cell)) {
          continue;
        }
        // A diagonal step between two obstacle corners is a passage the
        // raster invented, not one a wire could take.
        if (step >= FIRST_DIAGONAL_STEP &&
            planScene->blocked.test(
                grid.index(static_cast<std::uint32_t>(nx),
                           static_cast<std::uint32_t>(cy))) &&
            planScene->blocked.test(
                grid.index(static_cast<std::uint32_t>(cx),
                           static_cast<std::uint32_t>(ny)))) {
          continue;
        }
        // The step itself is what a closed gate refuses.
        if (crossesAGate(current, step)) {
          continue;
        }
        static_cast<void>(floodMarks.insert(local, cell));
        pending.push(cell);
      }
    }

    chamber.gates.assign(touchedGates.begin(), touchedGates.end());
    // The gates are collected in a hash set, so their order is not the grid's.
    // Sorting is what makes the chain the same on every run.
    std::ranges::sort(chamber.gates);
    return chamber;
  }

  /// The gates of a chamber that are not hidden behind another one.
  ///
  /// Two gates in a line are one gate as far as the chain is concerned: the
  /// wires that pass the nearer one are exactly the wires that reach the
  /// farther one, so recording both would count the same constraint twice.
  ///
  /// A gate is hidden when the chamber cannot see it at all, so **one** clear
  /// line of sight is what keeps it. A rule that every cell beside a gate has
  /// to see it is much stricter and wrong: a gate is a line of cells, the
  /// sight line from one of its own ends to its middle runs almost along it,
  /// and the gate next to it lies across that line. Where several gates ring
  /// one chamber, that rule would drop every one of them, and the free space
  /// beyond a branching corridor would fall out of the plan.
  [[nodiscard]] std::vector<std::size_t>
  visibleGates(const Chamber& chamber) const {
    if (chamber.gates.size() < 2) {
      return chamber.gates;
    }
    const auto& grid = planScene->detail;
    std::vector<std::size_t> visible;

    for (const auto candidate : chamber.gates) {
      const auto from = layoutOf(grid, (*gateList)[candidate].first);
      const auto to = layoutOf(grid, (*gateList)[candidate].second);
      const fbg::Point middle{(from.x() + to.x()) / 2.0,
                              (from.y() + to.y()) / 2.0};

      // A closed gate refuses the moves across its line, so the chamber
      // holds at most the cells on its own side of the line. It sees the gate
      // from the cells on or beside the line that it holds. A gate with no
      // such cell is one the chamber does not border at all, which is exactly
      // the gate this test exists to drop.
      bool seenFromSomewhere = false;
      for (const auto eye : observersOf(candidate, chamber)) {
        const auto seenFrom = layoutOf(grid, eye);
        bool clear = true;
        for (const auto other : chamber.gates) {
          if (other == candidate) {
            continue;
          }
          if (segmentsCross(seenFrom, middle,
                            layoutOf(grid, (*gateList)[other].first),
                            layoutOf(grid, (*gateList)[other].second))) {
            clear = false;
            break;
          }
        }
        if (clear) {
          seenFromSomewhere = true;
          break;
        }
      }
      if (seenFromSomewhere) {
        visible.push_back(candidate);
      }
    }
    return visible;
  }

  /// The cells of a chamber that look onto a gate: the ones beside its line.
  [[nodiscard]] std::vector<std::size_t>
  observersOf(const std::size_t gate, const Chamber& chamber) const {
    const auto& grid = planScene->detail;
    const std::unordered_set<std::size_t> seen(chamber.cells.begin(),
                                               chamber.cells.end());
    std::vector<std::size_t> observers;
    std::unordered_set<std::size_t> taken;
    for (const auto cell : gateLines[gate]) {
      if (seen.contains(cell) && taken.insert(cell).second) {
        observers.push_back(cell);
      }
      const auto cx = static_cast<std::int64_t>(cell % grid.width);
      const auto cy = static_cast<std::int64_t>(cell / grid.width);
      for (std::size_t step = 0; step < STEP_X.size(); ++step) {
        const auto nx = cx + STEP_X[step];
        const auto ny = cy + STEP_Y[step];
        if (nx < 0 || ny < 0 || std::cmp_greater_equal(nx, grid.width) ||
            std::cmp_greater_equal(ny, grid.height)) {
          continue;
        }
        const auto neighbor = grid.index(static_cast<std::uint32_t>(nx),
                                         static_cast<std::uint32_t>(ny));
        if (seen.contains(neighbor) && taken.insert(neighbor).second) {
          observers.push_back(neighbor);
        }
      }
    }
    return observers;
  }

  void expand(const std::uint32_t parent, const std::vector<std::size_t>& seeds,
              const std::unordered_set<std::size_t>& skip) {
    const auto chamber = flood(seeds);

    if (chamber.reachesALauncher) {
      // The chamber that reaches a launcher is not consumed: another chain
      // may reach the same launcher through it.
      const auto launcher = addNode(fba::CapacityElement::Launcher, 0, 0);
      (*nodeList)[parent]->next.push_back(launcher);
      return;
    }
    visitedCells.insert(chamber.cells);

    // A target lies on one chain only: the Global stage asks one wire of
    // each target over its chain, so a target on two chains would ask for
    // its wire twice.
    for (const auto& [cell, port] : chamber.targets) {
      if (heldCells.contains(cell) || !servedInWalk.insert(cell).second) {
        continue;
      }
      heldCells.insert(cell);
      servedOverall.insert(cell);
      const auto target = addNode(fba::CapacityElement::Target, port, 0);
      (*nodeList)[parent]->next.push_back(target);
    }

    const auto gates = visibleGates(chamber);
    const std::unordered_set<std::size_t> siblings(gates.begin(), gates.end());
    for (const auto gate : gates) {
      if (skip.contains(gate)) {
        continue;
      }
      openedInWalk.insert(gate);
      const auto node =
          addNode(fba::CapacityElement::Bottleneck,
                  static_cast<std::uint32_t>(gate), capacityOf(gate));

      setClosed(gate, false);
      expand(node, seedsBeyond(gate, chamber), siblings);
      setClosed(gate, true);

      (*nodeList)[parent]->next.push_back(node);
    }
  }

  /// One chamber of a chain, labelled, then the chambers beyond its gates.
  ///
  /// A cell that an earlier chamber of the walk holds keeps its label; only
  /// the seeds beside the gate can be such cells. A reserved cell keeps its
  /// label too: it belongs to the approach of a port and to no partition,
  /// even where the stage took its band back and a wire may pass it. Every
  /// other cell takes the label of this chamber, the free cells of the gate
  /// line too: a closed gate refuses the moves across its line, so the
  /// chamber before the gate may not reach every cell of the line, and
  /// without a label such a cell would part the two chambers.
  ///
  /// @return Whether the chamber was labelled, which it is unless it reaches
  /// a launcher.
  [[nodiscard]] bool stamp(const std::vector<std::size_t>& seeds,
                           const std::unordered_set<std::size_t>& skip) {
    const auto chamber = flood(seeds);
    if (chamber.reachesALauncher) {
      // The chamber that reaches a launcher belongs to no chain in
      // particular, so it is left for the watershed rather than claimed.
      return false;
    }

    for (const auto cell : chamber.cells) {
      if (!visitedCells.contains(cell) && !planScene->reserved.test(cell)) {
        (*labelSink)[cell] = nextStampLabel;
      }
    }
    visitedCells.insert(chamber.cells);
    for (const auto& [cell, port] : chamber.targets) {
      servedInWalk.insert(cell);
    }

    const auto gates = visibleGates(chamber);
    const std::unordered_set<std::size_t> siblings(gates.begin(), gates.end());
    for (const auto gate : gates) {
      if (skip.contains(gate)) {
        continue;
      }
      openedInWalk.insert(gate);
      setClosed(gate, false);
      // Every chamber beyond a gate is a partition of its own, which is what
      // makes a gate a border rather than a line drawn through one region.
      nextStampLabel = grid::nextPartitionLabel(nextStampLabel);
      static_cast<void>(stamp(seedsBeyond(gate, chamber), siblings));
      setClosed(gate, true);
    }
    return true;
  }

  /// The cells a chain continues from once a gate is open: the cells of the
  /// gate itself and the ones beside it that the chamber has already seen.
  [[nodiscard]] std::vector<std::size_t>
  seedsBeyond(const std::size_t gate, const Chamber& chamber) const {
    const auto& grid = planScene->detail;
    const std::unordered_set<std::size_t> seen(chamber.cells.begin(),
                                               chamber.cells.end());
    std::vector<std::size_t> seeds;
    std::unordered_set<std::size_t> taken;

    for (const auto cell : gateLines[gate]) {
      if (taken.insert(cell).second) {
        seeds.push_back(cell);
      }
      const auto cx = static_cast<std::int64_t>(cell % grid.width);
      const auto cy = static_cast<std::int64_t>(cell / grid.width);
      for (std::size_t step = 0; step < STEP_X.size(); ++step) {
        const auto nx = cx + STEP_X[step];
        const auto ny = cy + STEP_Y[step];
        if (nx < 0 || ny < 0 || std::cmp_greater_equal(nx, grid.width) ||
            std::cmp_greater_equal(ny, grid.height)) {
          continue;
        }
        const auto neighbor = grid.index(static_cast<std::uint32_t>(nx),
                                         static_cast<std::uint32_t>(ny));
        if (seen.contains(neighbor) && taken.insert(neighbor).second) {
          seeds.push_back(neighbor);
        }
      }
    }
    return seeds;
  }

  [[nodiscard]] std::uint32_t capacityOf(const std::size_t gate) const {
    return budgets[gate];
  }

public:
  /// The wire budget of every bottleneck, filled by the stage before the
  /// chains are built.
  std::vector<std::uint32_t> budgets;

private:
  const CapacityScene* planScene;
  const std::vector<grid::Bottleneck>* gateList;
  std::vector<std::vector<std::size_t>> gateLines;
  std::vector<std::vector<grid::BlockedMove>> gateMoves;
  NodeList* nodeList;
  std::vector<bool> gateClosed;
  /// The gates that refuse each move, and how many closed ones do.
  std::unordered_map<std::uint64_t, std::vector<std::size_t>> gatesOnMove;
  std::unordered_map<std::uint64_t, std::uint16_t> closedCount;
  std::unordered_map<std::size_t, std::uint32_t> portOfTarget;
  std::unordered_set<std::size_t> launcherCells;
  /// The cells the walk in progress has put into a chamber.
  CellSet visitedCells;
  /// The cells of the chamber a flood is collecting, one generation per
  /// flood.
  mutable CellMarks floodMarks;
  std::unordered_set<std::size_t> servedInWalk;
  std::unordered_set<std::size_t> openedInWalk;
  /// The targets that a chain holds below its root.
  std::unordered_set<std::size_t> servedOverall;
  /// The targets of every chain built so far, the roots included.
  std::unordered_set<std::size_t> heldCells;
  /// The gates of the walk in progress, when it is a chain's own rather than
  /// the search over every candidate.
  std::optional<std::unordered_set<std::size_t>> ownGates;
  /// Where a stamping walk writes, and the label it writes next.
  std::vector<grid::PartitionLabel>* labelSink = nullptr;
  grid::PartitionLabel nextStampLabel = grid::FIRST_PARTITION_LABEL;
};

/// Reduce a chain to the gates that actually constrain it.
///
/// Two shapes are redundant, and both are removed to a fixpoint because
/// removing one can expose another:
///
/// A gate with nothing beyond it constrains nothing. It is a place the walk
/// found and then could not get past, so no wire ever passes it.
///
/// A gate whose only child is another gate is two gates in a line, and the
/// wires that pass the first are exactly the wires that reach the second. Only
/// the one with the smaller wire budget binds, the shorter one where the
/// budgets are equal; keeping both would count one constraint twice.
class ChainPruner {
public:
  using NodeList = std::vector<std::unique_ptr<fba::CapacityNodeT>>;

  ChainPruner(NodeList& nodes, const std::vector<grid::Bottleneck>& bottlenecks,
              const std::vector<std::uint32_t>& budgets,
              const grid::GridMetrics& grid)
      : nodeList(&nodes), gateList(&bottlenecks), gateBudgets(&budgets),
        detailGrid(&grid) {}

  void prune(const std::uint32_t root) {
    // One rule feeds the other: collapsing a run can leave a gate with nothing
    // beyond it, and removing that can leave its parent childless. So both run
    // to one fixpoint rather than one after the other.
    bool changed = true;
    while (changed) {
      changed = dropEmptyGates(root);
      changed = collapseGateRuns(root) || changed;
    }
  }

private:
  [[nodiscard]] bool isGate(const std::uint32_t node) const {
    return (*nodeList)[node]->kind == fba::CapacityElement::Bottleneck;
  }

  /// Whether the first of two gates binds tighter than the second: it carries
  /// fewer wires, or as many and is shorter.
  [[nodiscard]] bool tighter(const std::uint32_t first,
                             const std::uint32_t second) const {
    const auto firstBudget = (*gateBudgets)[(*nodeList)[first]->id];
    const auto secondBudget = (*gateBudgets)[(*nodeList)[second]->id];
    if (firstBudget != secondBudget) {
      return firstBudget < secondBudget;
    }
    return width(first) < width(second);
  }

  /// The length of a gate, in cells.
  [[nodiscard]] double width(const std::uint32_t node) const {
    const auto& gate = (*gateList)[(*nodeList)[node]->id];
    const std::size_t firstRow = gate.first / detailGrid->width;
    const std::size_t secondRow = gate.second / detailGrid->width;
    const auto x0 = static_cast<double>(gate.first % detailGrid->width);
    const auto y0 = static_cast<double>(firstRow);
    const auto x1 = static_cast<double>(gate.second % detailGrid->width);
    const auto y1 = static_cast<double>(secondRow);
    return std::hypot(x1 - x0, y1 - y0);
  }

  bool dropEmptyGates(const std::uint32_t node) {
    bool changed = false;
    auto& children = (*nodeList)[node]->next;
    for (auto it = children.begin(); it != children.end();) {
      if (isGate(*it) && (*nodeList)[*it]->next.empty()) {
        it = children.erase(it);
        changed = true;
      } else {
        changed = dropEmptyGates(*it) || changed;
        ++it;
      }
    }
    return changed;
  }

  bool collapseGateRuns(const std::uint32_t node) {
    bool changed = false;
    for (const auto child : (*nodeList)[node]->next) {
      changed = collapseGateRuns(child) || changed;
    }
    for (auto& child : (*nodeList)[node]->next) {
      if (!isGate(child) || (*nodeList)[child]->next.size() != 1) {
        continue;
      }
      const auto grandchild = (*nodeList)[child]->next.front();
      if (!isGate(grandchild)) {
        continue;
      }
      if (tighter(grandchild, child)) {
        // The far gate binds tighter, so it replaces the near one and
        // keeps what lay beyond it.
        child = grandchild;
      } else {
        // The near gate binds; it takes over what lay beyond the far one.
        (*nodeList)[child]->next = (*nodeList)[grandchild]->next;
      }
      changed = true;
    }
    return changed;
  }

  NodeList* nodeList;
  const std::vector<grid::Bottleneck>* gateList;
  const std::vector<std::uint32_t>* gateBudgets;
  const grid::GridMetrics* detailGrid;
};

/// The capacity planner of the first release.
class WatershedPlanner final : public ICapacityPlanner {
public:
  [[nodiscard]] CapacityPlanT run(const ChipT& chip, const ConfigT& config,
                                  const Report& report) const override {
    report.progress("rasterizing the chip", "", 0, 0);
    const auto scene = buildScene(chip, config);
    const auto& rules = *config.rules;
    if (report.wants(Detail::Steps)) {
      report.line(Detail::Steps,
                  std::format("capacity grid {}{}{}{}detail grid {}{}{}",
                              scene.capacity.width, TIMES,
                              scene.capacity.height, FIGURE_SEPARATOR,
                              scene.detail.width, TIMES, scene.detail.height));
      report.line(Detail::Steps,
                  std::format("{}{}{}",
                              counted(scene.targetCell.size(), "target"),
                              FIGURE_SEPARATOR,
                              counted(scene.launcherCell.size(), "launcher")));
    }

    CapacityPlanT plan;
    plan.capacity_grid = extentOf(scene.capacity);
    plan.detail_grid = extentOf(scene.detail);

    report.progress("distance transform", "", 0, 0);
    const auto distance = grid::squaredDistanceTransform(scene.blocked);
    report.progress("medial axis", "", 0, 0);
    const auto axis = grid::rasterizeMedialAxis(
        scene.blocked, scene.detail, grid::medialAxis(scene.blocked));
    report.line(Detail::Steps,
                "medial axis of " + counted(axis.neighbors.size(), "cell"));

    // A place is a bottleneck only where the free radius around it is below
    // the configured fraction of the chip. The threshold is a squared cell
    // count because that is what the distance transform holds.
    const auto pitch = rules.min_wire_spacing + rules.min_obstacle_spacing;
    const auto clearance = clearanceLimit(config, scene, pitch);
    report.progress("bottlenecks", "", 0, 0);
    const auto candidates =
        grid::findBottlenecks(scene.blocked, axis, distance, scene.detail,
                              {.maximumSquaredClearance = clearance,
                               .sameNarrowing = pitch,
                               .targets = scene.targetCell});
    report.line(Detail::Steps,
                counted(candidates.size(), "candidate bottleneck"));

    // The chains come first: which gates matter is decided by walking them,
    // and a gate that no chain crosses constrains nothing and is dropped. The
    // same builder walks the chains again to stamp their chambers; every walk
    // starts from a clean state.
    const auto capacities = capacitiesOf(candidates, scene, rules);
    ChainBuilder builder(scene, candidates, plan.nodes);
    builder.budgets = capacities;
    const auto chains = buildChains(plan, builder, candidates, scene, report);

    // The partitioning is what the gates carve out. Every chamber a chain
    // walks through is one partition, and the watershed over the clear
    // capacity cells only fills what no chamber claimed.
    std::vector<grid::PartitionLabel> labels(scene.detail.cells(),
                                             grid::LABEL_NONE);
    for (std::size_t cell = 0; cell < labels.size(); ++cell) {
      if (scene.reserved.test(cell)) {
        labels[cell] = grid::LABEL_RESERVED;
      }
    }
    const auto seeds =
        grid::freeCellSeeds(scene.blocked, scene.detail, scene.capacity);
    // The chambers, the watershed and the pieces of a cut partition each take
    // labels, and any of them can take the last one.
    try {
      auto nextLabel = grid::FIRST_PARTITION_LABEL;
      for (std::size_t index = 0; index < chains.size(); ++index) {
        report.progress("chambers", "", index, chains.size());
        builder.stampChambers(chains[index].cell, chains[index].gates, labels,
                              nextLabel);
        nextLabel = grid::nextPartitionLabel(nextLabel);
      }

      report.progress("watershed", "", 0, 0);
      nextLabel = grid::runWatershed(scene.blocked, seeds, labels, nextLabel);
      grid::smoothPartitionBorders(scene.blocked, seeds, labels,
                                   grid::FIRST_PARTITION_LABEL,
                                   SMOOTHING_RADIUS, SMOOTHING_ROUNDS);
      // A partition is one region that a wire can cross from any border to
      // any other. The vote can cut a partition in two, and a later chamber
      // can cut through an earlier one, so every piece that is cut off becomes
      // a partition of its own.
      static_cast<void>(grid::separatePartitionPieces(scene.blocked, labels,
                                                      scene.detail, nextLabel));
    } catch (const std::length_error&) {
      // Every free capacity cell seeds a partition of its own.
      throw std::length_error(std::format(
          "the capacity grid has {} free cells, and with the chambers and the "
          "pieces of cut partitions they need more than the {} partition "
          "labels a plan holds; lower [grid] capacity_cells_x",
          seeds.size(),
          std::numeric_limits<grid::PartitionLabel>::max() -
              grid::FIRST_PARTITION_LABEL));
    }

    report.progress("partitions", "", 0, 0);
    const auto partitions =
        grid::extractPartitions(scene.blocked, labels, scene.detail);
    fillPartitions(plan, partitions, scene, crossingPitch(config));
    fillBottlenecks(plan, candidates, chains, scene, capacities);
    fillLaunchers(plan, scene);
    fillPortKeepout(plan, scene);
    report.line(
        Detail::Steps,
        std::format("{}{}{}", counted(plan.partitions.size(), "partition"),
                    FIGURE_SEPARATOR, counted(plan.borders.size(), "border")));
    report.result({{.text = counted(plan.partitions.size(), "partition")},
                   {.text = counted(plan.bottlenecks.size(), "bottleneck")},
                   {.text = counted(plan.chains.size(), "chain")}});
    return plan;
  }

  /// One chain, as the artifact holds it and as the stamping pass needs it.
  struct Chain {
    std::uint32_t root = 0;
    std::size_t cell = 0;
    /// The candidate gates the chain kept, in candidate numbering.
    std::vector<std::size_t> gates;
  };

private:
  /// The squared cell clearance a place has to be below to count.
  ///
  /// An absent section is the schema's defaults rather than nothing, which is
  /// what the loader promises and what a configuration built by hand has to
  /// get as well. The default-constructed parameters carry them, so the
  /// figure is never repeated here.
  [[nodiscard]] static std::uint32_t clearanceLimit(const ConfigT& config,
                                                    const CapacityScene& scene,
                                                    const double pitch) {
    const flatbuffers::config::CapacityParamsT defaults;
    const auto* params =
        config.stages != nullptr && config.stages->capacity != nullptr
            ? config.stages->capacity.get()
            : &defaults;
    const auto fraction = params->bottleneck_clearance;
    if (fraction <= 0.0) {
      return 0;
    }
    const auto step = std::min(scene.detail.cellWidth, scene.detail.cellHeight);
    if (step <= 0.0) {
      return 0;
    }
    // The threshold is a length: how much free radius a place may have and
    // still count as narrow. Dividing by the cell step turns it into the
    // cells the distance transform counts, and squaring it into what the
    // transform actually holds.
    // A clearance wider than the counter holds counts every place as narrow,
    // which the largest count says as well.
    const auto radius = fraction * pitch / step;
    const auto squared = radius * radius;
    constexpr auto largest = std::numeric_limits<std::uint32_t>::max();
    if (!(squared < static_cast<double>(largest))) {
      return largest;
    }
    return static_cast<std::uint32_t>(squared);
  }

  /// The outline of what the ports' own approaches keep clear.
  ///
  /// The region is traced the same way a partition is: every cell of it takes
  /// one label and everything else is an obstacle, so the rings follow the
  /// cells and a band along a diagonal comes out as the staircase the grid
  /// actually blocks. One ring per connected piece, and one more per hole.
  static void fillPortKeepout(CapacityPlanT& plan, const CapacityScene& scene) {
    std::vector<grid::PartitionLabel> labels(scene.detail.cells(),
                                             grid::LABEL_NONE);
    grid::BitGrid outside(scene.detail.width, scene.detail.height);
    for (std::size_t cell = 0; cell < labels.size(); ++cell) {
      if (scene.keepout.test(cell)) {
        labels[cell] = grid::FIRST_PARTITION_LABEL;
      } else {
        outside.set(cell, true);
      }
    }
    for (const auto& outline :
         grid::extractPartitions(outside, labels, scene.detail).outlines) {
      auto ring = std::make_unique<fbg::PolygonT>();
      ring->vertices.reserve(outline.ring.size());
      for (const auto& corner : outline.ring) {
        ring->vertices.push_back(
            grid::cornerToLayout(scene.detail, corner.x(), corner.y()));
      }
      plan.port_keepout.push_back(std::move(ring));
    }
  }

  static void fillPartitions(CapacityPlanT& plan,
                             const grid::Partitions& partitions,
                             const CapacityScene& scene, const double pitch) {
    std::unordered_map<grid::PartitionLabel, std::size_t> position;
    for (const auto& outline : partitions.outlines) {
      auto found = position.find(outline.label);
      if (found == position.end()) {
        auto partition = std::make_unique<fba::PartitionT>();
        partition->label = outline.label;
        plan.partitions.push_back(std::move(partition));
        found =
            position.emplace(outline.label, plan.partitions.size() - 1).first;
      }
      auto ring = std::make_unique<fbg::PolygonT>();
      ring->vertices.reserve(outline.ring.size());
      for (const auto& corner : outline.ring) {
        ring->vertices.push_back(
            grid::cornerToLayout(scene.detail, corner.x(), corner.y()));
      }
      plan.partitions[found->second]->outlines.push_back(std::move(ring));
    }

    for (const auto& border : partitions.borders) {
      auto entry = std::make_unique<fba::PartitionBorderT>();
      entry->first = border.first;
      entry->second = border.second;
      entry->samples.reserve(border.samples.size());
      for (const auto& sample : border.samples) {
        entry->samples.push_back(
            grid::cornerToLayout(scene.detail, sample.x(), sample.y()));
      }
      const auto center = border.center();
      entry->center =
          grid::cornerToLayout(scene.detail, center.x(), center.y());
      // The border carries one wire per crossing slot. The Corridor stage
      // routes across the slots of the same function, so the budget counts
      // the slots it offers.
      entry->budget = static_cast<std::uint32_t>(
          crossingSlots(border, scene, pitch).size());
      plan.borders.push_back(std::move(entry));
    }
  }

  /// The gates that survived, and the renumbering of the chain nodes onto
  /// them.
  ///
  /// The artifact carries the pruned set only. A candidate the search found
  /// and the chains then dropped is not a constraint on anything, and keeping
  /// it would put a line in every picture that nothing routes around.
  static void fillBottlenecks(CapacityPlanT& plan,
                              const std::vector<grid::Bottleneck>& candidates,
                              const std::vector<Chain>& chains,
                              const CapacityScene& scene,
                              const std::vector<std::uint32_t>& capacities) {
    std::vector<std::size_t> kept;
    for (const auto& chain : chains) {
      kept.insert(kept.end(), chain.gates.begin(), chain.gates.end());
    }
    std::ranges::sort(kept);
    const auto duplicates = std::ranges::unique(kept);
    kept.erase(duplicates.begin(), duplicates.end());

    std::unordered_map<std::size_t, std::uint32_t> renumbered;
    plan.bottlenecks.reserve(kept.size());
    for (const auto candidate : kept) {
      renumbered.emplace(candidate,
                         static_cast<std::uint32_t>(plan.bottlenecks.size()));
      auto entry = std::make_unique<fba::BottleneckT>();
      entry->from = layoutOf(scene.detail, candidates[candidate].first);
      entry->to = layoutOf(scene.detail, candidates[candidate].second);
      entry->capacity = capacities[candidate];
      plan.bottlenecks.push_back(std::move(entry));
    }

    for (auto& node : plan.nodes) {
      if (node->kind == fba::CapacityElement::Bottleneck) {
        node->id = renumbered.at(node->id);
      }
    }
  }

  static void fillLaunchers(CapacityPlanT& plan, const CapacityScene& scene) {
    plan.launchers.reserve(scene.launcherCell.size());
    for (std::size_t index = 0; index < scene.launcherCell.size(); ++index) {
      auto slot = std::make_unique<fba::LauncherSlotT>();
      slot->port = flatbuffers::design::PortRef(scene.launcherPort[index]);
      slot->position = layoutOf(scene.detail, scene.launcherCell[index]);
      plan.launchers.push_back(std::move(slot));
    }
  }

  /// The wire budget of every candidate gate.
  [[nodiscard]] static std::vector<std::uint32_t>
  capacitiesOf(const std::vector<grid::Bottleneck>& candidates,
               const CapacityScene& scene,
               const flatbuffers::design::DesignRulesT& rules) {
    std::vector<std::uint32_t> capacities;
    capacities.reserve(candidates.size());
    for (const auto& gate : candidates) {
      const auto roundDown =
          scene.reserved.test(gate.first) || scene.reserved.test(gate.second);
      capacities.push_back(
          grid::bottleneckCapacity(gate, scene.detail, rules.min_wire_spacing,
                                   rules.min_obstacle_spacing, roundDown));
    }
    return capacities;
  }

  /// The gates a chain names, in candidate numbering.
  static void collectGates(const CapacityPlanT& plan, const std::uint32_t node,
                           std::vector<std::size_t>& gates) {
    const auto& entry = *plan.nodes[node];
    if (entry.kind == fba::CapacityElement::Bottleneck) {
      gates.push_back(entry.id);
    }
    for (const auto child : entry.next) {
      collectGates(plan, child, gates);
    }
  }

  /// Build one chain per target that no other chain already serves, then
  /// reduce each to the gates that bind.
  [[nodiscard]] static std::vector<Chain>
  buildChains(CapacityPlanT& plan, ChainBuilder& builder,
              const std::vector<grid::Bottleneck>& candidates,
              const CapacityScene& scene, const Report& report) {
    std::vector<Chain> chains;
    for (std::size_t index = 0; index < scene.targetCell.size(); ++index) {
      report.progress("chains", "", index, scene.targetCell.size());
      if (builder.served().contains(scene.targetCell[index])) {
        continue;
      }
      chains.push_back({.root = builder.build(scene.targetCell[index],
                                              scene.targetPort[index]),
                        .cell = scene.targetCell[index],
                        .gates = {}});
    }

    ChainPruner pruner(plan.nodes, candidates, builder.budgets, scene.detail);
    for (auto& chain : chains) {
      pruner.prune(chain.root);
      collectGates(plan, chain.root, chain.gates);
      std::ranges::sort(chain.gates);
      const auto duplicates = std::ranges::unique(chain.gates);
      chain.gates.erase(duplicates.begin(), duplicates.end());
    }
    compact(plan, chains);
    if (report.wants(Detail::Steps)) {
      std::unordered_set<std::uint32_t> kept;
      for (const auto& node : plan.nodes) {
        if (node->kind == fba::CapacityElement::Bottleneck) {
          kept.insert(node->id);
        }
      }
      report.line(
          Detail::Steps,
          std::format("{} cross {} of {}", counted(chains.size(), "chain"),
                      kept.size(),
                      counted(candidates.size(), "candidate bottleneck")));
    }
    return chains;
  }

  /// Drop the nodes the pruning left unreachable and renumber the rest.
  ///
  /// Pruning detaches a node rather than erasing it, because a node's index is
  /// how its parent names it. What the artifact carries is the chains, so
  /// everything no chain reaches goes.
  static void compact(CapacityPlanT& plan, std::vector<Chain>& chains) {
    std::vector<std::uint32_t> order;
    std::unordered_map<std::uint32_t, std::uint32_t> moved;

    const auto walk = [&](auto&& self, const std::uint32_t node) -> void {
      if (moved.contains(node)) {
        return;
      }
      moved.emplace(node, static_cast<std::uint32_t>(order.size()));
      order.push_back(node);
      for (const auto child : plan.nodes[node]->next) {
        self(self, child);
      }
    };
    for (const auto& chain : chains) {
      walk(walk, chain.root);
    }

    std::vector<std::unique_ptr<fba::CapacityNodeT>> kept;
    kept.reserve(order.size());
    for (const auto node : order) {
      kept.push_back(std::move(plan.nodes[node]));
      for (auto& child : kept.back()->next) {
        child = moved.at(child);
      }
    }
    plan.nodes = std::move(kept);

    plan.chains.reserve(chains.size());
    for (auto& chain : chains) {
      chain.root = moved.at(chain.root);
      plan.chains.push_back(chain.root);
    }
  }
};

} // namespace

std::unique_ptr<ICapacityPlanner> makeWatershedPlanner() {
  return std::make_unique<WatershedPlanner>();
}

} // namespace mqt::scpd::pipeline
