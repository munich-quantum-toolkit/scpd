/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The Corridor stage on the benchmark inputs. What is checked is the plan it
// promises the detail router: every wire has a way through, a border slot
// carries one wire, and no two wires cross inside a partition.

#include "Benchmarks.hpp"
#include "mqt-scpd/flatbuffers/artifacts.hpp"
#include "mqt-scpd/flatbuffers/geometry.hpp"
#include "mqt-scpd/pipeline/Registry.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace mqt::scpd::pipeline {
namespace {

using flatbuffers::artifacts::CapacityPlanT;
using flatbuffers::artifacts::CorridorRoutingT;
using flatbuffers::geometry::Point;

/// What the three stages before the corridor router produce.
struct Planned {
  CapacityPlanT capacity;
  flatbuffers::artifacts::GlobalRoutingT global;
  flatbuffers::artifacts::AssignmentT assignment;
};

Planned plan(const Benchmark& benchmark) {
  Planned planned;
  planned.capacity = capacityPlanners()
                         .make("watershed")
                         ->run(benchmark.chip, benchmark.config);
  planned.global =
      globalRouters()
          .make("hanan-milp")
          ->run(benchmark.chip, planned.capacity, benchmark.config);
  planned.assignment = assigners()
                           .make("ordered-milp")
                           ->run(benchmark.chip, planned.capacity,
                                 planned.global, benchmark.config);
  return planned;
}

CorridorRoutingT routeCorridors(const Benchmark& benchmark,
                                const Planned& planned) {
  return corridorRouters()
      .make("partition-astar")
      ->run(benchmark.chip, planned.capacity, planned.assignment,
            benchmark.config);
}

/// The two partitions each border joins, keyed the way a corridor names them.
std::map<std::pair<std::uint32_t, std::uint32_t>, std::size_t>
bordersOf(const CapacityPlanT& capacity) {
  std::map<std::pair<std::uint32_t, std::uint32_t>, std::size_t> borders;
  for (std::size_t i = 0; i < capacity.borders.size(); ++i) {
    const auto& border = *capacity.borders[i];
    borders.emplace(std::minmax(border.first, border.second), i);
  }
  return borders;
}

/// Whether two chords cross each other properly.
///
/// The stage decides this on the grid, where every coordinate is a whole
/// number or a half and the determinant is exact. Here the same points are
/// layout units in the tens of thousands, so three points on one line come out
/// at about 1e-10 rather than at zero. The sign is therefore taken against the
/// lengths of the two segments: below that, the points are on a line and the
/// chords touch rather than cross.
int sideOf(const Point& p, const Point& q, const Point& r) {
  const auto cross =
      ((q.x() - p.x()) * (r.y() - p.y())) - ((q.y() - p.y()) * (r.x() - p.x()));
  const auto scale = std::hypot(q.x() - p.x(), q.y() - p.y()) *
                     std::hypot(r.x() - p.x(), r.y() - p.y());
  if (std::abs(cross) <= 1.0e-9 * scale) {
    return 0;
  }
  return cross > 0.0 ? 1 : -1;
}

bool crosses(const Point& a, const Point& b, const Point& c, const Point& d) {
  return (sideOf(a, b, c) * sideOf(a, b, d)) < 0 &&
         (sideOf(c, d, a) * sideOf(c, d, b)) < 0;
}

/// One chord of a corridor: which partition it runs across, where it goes, and
/// whether its ends are places the wire is pinned to — the point it is fed at
/// and the cell of its target port — rather than crossing slots.
struct Chord {
  std::uint32_t partition = 0;
  Point from;
  Point to;
  bool fromPin = false;
  bool toPin = false;
};

/// The chords one corridor draws, in order.
std::vector<Chord> chordsOf(const flatbuffers::artifacts::CorridorT& corridor) {
  std::vector<Chord> chords;
  if (corridor.partitions.empty() || corridor.source == nullptr ||
      corridor.target == nullptr) {
    return chords;
  }
  auto place = *corridor.source;
  auto pin = true;
  for (std::size_t i = 0; i < corridor.crossings.size(); ++i) {
    chords.push_back({.partition = corridor.partitions[i],
                      .from = place,
                      .to = corridor.crossings[i],
                      .fromPin = pin,
                      .toPin = false});
    place = corridor.crossings[i];
    pin = false;
  }
  chords.push_back({.partition = corridor.partitions.back(),
                    .from = place,
                    .to = *corridor.target,
                    .fromPin = pin,
                    .toPin = true});
  return chords;
}

/// Every crossing slot carries one wire, which is what makes a border's budget
/// a bound rather than a wish.
void expectEachSlotCarriesOneWire(const CorridorRoutingT& routing) {
  std::set<std::pair<double, double>> taken;
  for (const auto& corridor : routing.corridors) {
    for (const auto& crossing : corridor->crossings) {
      EXPECT_TRUE(taken.emplace(crossing.x(), crossing.y()).second)
          << "two wires cross at (" << crossing.x() << ", " << crossing.y()
          << ")";
    }
  }
}

/// A wire leaves a partition only into one that adjoins it, and it names one
/// more partition than it has crossings.
void expectEveryStepCrossesABorder(const CorridorRoutingT& routing,
                                   const CapacityPlanT& capacity) {
  const auto borders = bordersOf(capacity);
  // A step over a way out of a port's own pocket is on no border of the plan,
  // and the artifact says which crossings those are.
  std::set<std::pair<double, double>> pockets;
  for (const auto& slots : routing.slots) {
    if (slots->pocket) {
      for (const auto& position : slots->positions) {
        pockets.emplace(position.x(), position.y());
      }
    }
  }

  for (const auto& corridor : routing.corridors) {
    if (corridor->partitions.empty()) {
      continue;
    }
    ASSERT_EQ(corridor->crossings.size() + 1, corridor->partitions.size());
    for (std::size_t i = 0; i + 1 < corridor->partitions.size(); ++i) {
      const auto& crossing = corridor->crossings[i];
      if (pockets.contains({crossing.x(), crossing.y()})) {
        continue;
      }
      const auto pair =
          std::minmax(corridor->partitions[i], corridor->partitions[i + 1]);
      EXPECT_TRUE(borders.contains(pair))
          << "no border joins partition " << pair.first << " to "
          << pair.second;
    }
  }
}

/// Whether two chords lie along each other over a length.
///
/// Two wires that share a stretch of one corridor are one piece of copper. The
/// chords are a schematic of which partitions a wire uses, so two that graze at
/// a single point still leave the detail router room; two that run along each
/// other do not.
bool overlaps(const Point& a, const Point& b, const Point& c, const Point& d) {
  if (sideOf(a, b, c) != 0 || sideOf(a, b, d) != 0) {
    return false;
  }
  const auto vertical = std::abs(b.x() - a.x()) < std::abs(b.y() - a.y());
  const auto span = [vertical](const Point& p, const Point& q) {
    return vertical ? std::pair{std::min(p.y(), q.y()), std::max(p.y(), q.y())}
                    : std::pair{std::min(p.x(), q.x()), std::max(p.x(), q.x())};
  };
  const auto first = span(a, b);
  const auto second = span(c, d);
  const auto shared = std::min(first.second, second.second) -
                      std::max(first.first, second.first);
  // A shared stretch shorter than a thousandth of a cell is the float error of
  // the layout round trip, not two wires in one place.
  return shared > 1.0e-3;
}

/// No wire lies on another. Two wires that cross inside a partition, or run
/// along each other there, are one piece of copper, and the detail router is
/// handed a short it cannot undo.
void expectNoTwoWiresOverlap(const CorridorRoutingT& routing) {
  std::map<std::uint32_t, std::vector<std::pair<std::size_t, Chord>>> drawn;
  for (std::size_t wire = 0; wire < routing.corridors.size(); ++wire) {
    for (const auto& chord : chordsOf(*routing.corridors[wire])) {
      for (const auto& [other, theirs] : drawn[chord.partition]) {
        if (other == wire) {
          continue;
        }
        EXPECT_FALSE(crosses(chord.from, chord.to, theirs.from, theirs.to) ||
                     overlaps(chord.from, chord.to, theirs.from, theirs.to))
            << "wires " << wire << " and " << other
            << " lie on each other in partition " << chord.partition << ": ("
            << chord.from.x() << ", " << chord.from.y() << ")->("
            << chord.to.x() << ", " << chord.to.y() << ") and ("
            << theirs.from.x() << ", " << theirs.from.y() << ")->("
            << theirs.to.x() << ", " << theirs.to.y() << ")";
      }
      drawn[chord.partition].emplace_back(wire, chord);
    }
  }
}

/// Whether the point `c` lies between the ends of the chord from `a` to `b`,
/// the three being on one line already.
bool between(const Point& a, const Point& b, const Point& c) {
  const auto slack = 1.0e-9 * std::hypot(b.x() - a.x(), b.y() - a.y());
  return std::min(a.x(), b.x()) - slack <= c.x() &&
         c.x() <= std::max(a.x(), b.x()) + slack &&
         std::min(a.y(), b.y()) - slack <= c.y() &&
         c.y() <= std::max(a.y(), b.y()) + slack;
}

bool samePlace(const Point& a, const Point& b) {
  return std::abs(a.x() - b.x()) < 1.0e-6 && std::abs(a.y() - b.y()) < 1.0e-6;
}

/// No wire crosses a border outside the ring the ports feed from.
///
/// Every wire starts at a point on the launcher ring, so the rectangle those
/// points span is the outside of the chip. A crossing on it or beyond it is a
/// way behind some port: the wire leaves through the ring, runs along the back
/// of it and comes in again somewhere else.
void expectEveryCrossingIsInsideTheRing(const CorridorRoutingT& routing) {
  auto found = false;
  auto minX = 0.0;
  auto minY = 0.0;
  auto maxX = 0.0;
  auto maxY = 0.0;
  for (const auto& corridor : routing.corridors) {
    if (corridor->source == nullptr) {
      continue;
    }
    const auto& source = *corridor->source;
    if (!found) {
      minX = source.x();
      maxX = source.x();
      minY = source.y();
      maxY = source.y();
      found = true;
      continue;
    }
    minX = std::min(minX, source.x());
    maxX = std::max(maxX, source.x());
    minY = std::min(minY, source.y());
    maxY = std::max(maxY, source.y());
  }
  ASSERT_TRUE(found);

  for (std::size_t wire = 0; wire < routing.corridors.size(); ++wire) {
    for (const auto& crossing : routing.corridors[wire]->crossings) {
      EXPECT_TRUE(crossing.x() > minX && crossing.x() < maxX &&
                  crossing.y() > minY && crossing.y() < maxY)
          << "wire " << wire << " crosses at (" << crossing.x() << ", "
          << crossing.y() << "), outside the ring " << minX << ".." << maxX
          << " by " << minY << ".." << maxY;
    }
  }
}

/// Whether one wire is pinned to a point the other runs over.
bool touches(const Chord& a, const Chord& b) {
  const auto over = [](const Chord& chord, const Point& pin) {
    return sideOf(chord.from, chord.to, pin) == 0 &&
           between(chord.from, chord.to, pin) && !samePlace(chord.from, pin) &&
           !samePlace(chord.to, pin);
  };
  const auto pinned = [&over](const Chord& chord, const Chord& other) {
    return (other.fromPin && over(chord, other.from)) ||
           (other.toPin && over(chord, other.to));
  };
  return pinned(a, b) || pinned(b, a);
}

/// No wire runs over a point another wire is pinned to.
///
/// A wire is pinned where it is fed and at the cell of its target port, and
/// neither can be moved aside, so a second wire drawn over one is two wires on
/// one point of the chip. The crossing test does not see it: a chord that
/// stops on another one never reaches its far side, so the two never cross
/// properly, and a wire can run down a whole line of feed points without a
/// crossing being reported.
void expectNoWireRunsOverAnothersPin(const CorridorRoutingT& routing) {
  std::map<std::uint32_t, std::vector<std::pair<std::size_t, Chord>>> drawn;
  for (std::size_t wire = 0; wire < routing.corridors.size(); ++wire) {
    for (const auto& chord : chordsOf(*routing.corridors[wire])) {
      for (const auto& [other, theirs] : drawn[chord.partition]) {
        EXPECT_FALSE(touches(chord, theirs))
            << "wire " << wire << " and wire " << other
            << " share a pin in partition " << chord.partition << ": ("
            << chord.from.x() << ", " << chord.from.y() << ")->("
            << chord.to.x() << ", " << chord.to.y() << ") and ("
            << theirs.from.x() << ", " << theirs.from.y() << ")->("
            << theirs.to.x() << ", " << theirs.to.y() << ")";
      }
      drawn[chord.partition].emplace_back(wire, chord);
    }
  }
}

/// Two wires that share a partition do not cross inside it. A plan that broke
/// this would ask the detail router for a short.
void expectNoTwoWiresCrossInAPartition(const CorridorRoutingT& routing) {
  std::map<std::uint32_t, std::vector<Chord>> drawn;
  for (const auto& corridor : routing.corridors) {
    for (const auto& chord : chordsOf(*corridor)) {
      for (const auto& theirs : drawn[chord.partition]) {
        EXPECT_FALSE(crosses(chord.from, chord.to, theirs.from, theirs.to))
            << "two wires cross inside partition " << chord.partition;
      }
      drawn[chord.partition].push_back(chord);
    }
  }
}

class BenchmarkCorridors : public testing::TestWithParam<std::string> {};

TEST_P(BenchmarkCorridors, FollowsTheRulesOfThePartitionGraph) {
  const auto benchmark = benchmarkOf(GetParam());
  const auto planned = plan(benchmark);
  const auto routing = routeCorridors(benchmark, planned);

  // One corridor per connection, in the assignment's own order, so a reader
  // needs no key to line the two up.
  ASSERT_EQ(routing.corridors.size(), planned.assignment.connections.size());
  for (const auto& corridor : routing.corridors) {
    EXPECT_NE(corridor->source, nullptr);
    EXPECT_NE(corridor->target, nullptr);
  }

  expectEachSlotCarriesOneWire(routing);
  expectEveryStepCrossesABorder(routing, planned.capacity);
  expectNoTwoWiresCrossInAPartition(routing);
  expectNoTwoWiresOverlap(routing);
  expectNoWireRunsOverAnothersPin(routing);
  expectEveryCrossingIsInsideTheRing(routing);
}

INSTANTIATE_TEST_SUITE_P(EveryChip, BenchmarkCorridors,
                         testing::Values("4q", "9q", "17q", "21q", "33q", "45q",
                                         "57q", "69q"),
                         [](const testing::TestParamInfo<std::string>& chip) {
                           return chip.param;
                         });

/// Every connection of every benchmark chip finds a corridor.
///
/// This is the bar the stage is judged against, and it is the prototype's own:
/// its benchmark table reports no coarse-routing failure on any of the eight.
class CompleteCorridors : public testing::TestWithParam<std::string> {};

TEST_P(CompleteCorridors, RouteEveryConnection) {
  const auto benchmark = benchmarkOf(GetParam());
  const auto planned = plan(benchmark);
  const auto routing = routeCorridors(benchmark, planned);

  std::size_t unrouted = 0;
  for (const auto& corridor : routing.corridors) {
    unrouted += corridor->partitions.empty() ? 1U : 0U;
  }
  EXPECT_EQ(unrouted, 0U) << unrouted << " of " << routing.corridors.size()
                          << " connections found no corridor";
}

INSTANTIATE_TEST_SUITE_P(EveryChip, CompleteCorridors,
                         testing::Values("4q", "9q", "17q", "21q", "33q", "45q",
                                         "57q", "69q"),
                         [](const testing::TestParamInfo<std::string>& chip) {
                           return chip.param;
                         });

TEST(Corridors, GiveTheSameAnswerTwice) {
  // The stage contract is deterministic, which is what makes a resumed run
  // equal to an uninterrupted one.
  const auto benchmark = benchmarkOf("9q");
  const auto planned = plan(benchmark);

  const auto first = routeCorridors(benchmark, planned);
  const auto second = routeCorridors(benchmark, planned);

  ASSERT_EQ(first.corridors.size(), second.corridors.size());
  for (std::size_t i = 0; i < first.corridors.size(); ++i) {
    EXPECT_EQ(first.corridors[i]->partitions, second.corridors[i]->partitions);
    EXPECT_EQ(first.corridors[i]->crossings.size(),
              second.corridors[i]->crossings.size());
  }
}

} // namespace
} // namespace mqt::scpd::pipeline
