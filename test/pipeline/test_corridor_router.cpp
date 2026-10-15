/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The Corridor stage on the two-qubit fixture: the plan it promises the later
// stages, which is that every wire has a way through, a crossing slot carries
// one wire and no two wires cross inside a partition, and what it reports.

#include "../SplitMix.hpp"
#include "MiniFixture.hpp"
#include "mqt-scpd/flatbuffers/artifacts.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/flatbuffers/geometry.hpp"
#include "mqt-scpd/io/Artifacts.hpp"
#include "mqt-scpd/pipeline/Assigner.hpp"
#include "mqt-scpd/pipeline/CapacityPlanner.hpp"
#include "mqt-scpd/pipeline/CorridorRouter.hpp"
#include "mqt-scpd/pipeline/GlobalRouter.hpp"
#include "mqt-scpd/pipeline/Report.hpp"
#include "mqt-scpd/pipeline/Stages.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace mqt::scpd::pipeline {
namespace {

using flatbuffers::artifacts::ArtifactT;
using flatbuffers::artifacts::CorridorT;
using flatbuffers::geometry::Point;
using mqt::scpd::test::SplitMix;

/// What the stages before the corridor router make of the fixture.
struct Planned {
  ChipT chip;
  CapacityPlanT capacity;
  AssignmentT assignment;
};

Planned planMini() {
  const auto config = test::miniConfig();
  Planned planned{
      .chip = test::miniChip(config), .capacity = {}, .assignment = {}};
  planned.capacity = makeWatershedPlanner()->run(planned.chip, config, {});
  const auto global =
      makeHananMilpRouter()->run(planned.chip, planned.capacity, config, {});
  planned.assignment = makeOrderedMilpAssigner()->run(
      planned.chip, planned.capacity, global, config, {});
  return planned;
}

CorridorRoutingT routeMini(const Planned& planned, const Report& report = {},
                           const ConfigT& config = test::miniConfig()) {
  return makePartitionAStarRouter()->run(planned.chip, planned.capacity,
                                         planned.assignment, config, report);
}

/// The fixture with every connection and its feed point repeated, so that
/// the copies compete for the same slots.
Planned crowded(const std::size_t copies) {
  auto planned = planMini();
  const auto connections = planned.assignment.connections.size();
  for (std::size_t copy = 1; copy < copies; ++copy) {
    for (std::size_t index = 0; index < connections; ++index) {
      planned.assignment.connections.push_back(
          std::make_unique<flatbuffers::design::ConnectionT>(
              *planned.assignment.connections[index]));
      planned.assignment.feeds.push_back(planned.assignment.feeds[index]);
    }
  }
  return planned;
}

/// The fixture configuration with a number of sweeps and of wires that a
/// failed one may rip up.
ConfigT sweepConfig(const std::uint32_t rounds,
                    const std::uint32_t maxRelaxation) {
  auto config = test::miniConfig();
  config.stages->corridor->rounds = rounds;
  config.stages->corridor->max_relaxation = maxRelaxation;
  return config;
}

/// What a report receives: the lines and entries of each level, and the
/// fails of the result.
struct Received {
  std::vector<std::string> steps;
  std::vector<std::pair<std::string, std::vector<Figure>>> summary;
  std::vector<std::pair<std::string, std::vector<Figure>>> items;
  std::optional<std::uint64_t> fails;
};

Report receiving(Received& received, const Detail level) {
  return {level,
          {.line =
               [&received](const Detail detail, const std::string_view text) {
                 if (detail == Detail::Steps) {
                   received.steps.emplace_back(text);
                 }
               },
           .entry =
               [&received](const Detail detail, const std::string_view label,
                           const std::vector<Figure>& figures) {
                 auto& entries = detail == Detail::Summary ? received.summary
                                                           : received.items;
                 entries.emplace_back(label, figures);
               },
           .result =
               [&received](const std::vector<Figure>& /*figures*/,
                           const std::optional<std::uint64_t> count) {
                 received.fails = count;
               }}};
}

/// How many wires the routing leaves without a way.
std::size_t unroutedOf(const CorridorRoutingT& routing) {
  return static_cast<std::size_t>(
      std::ranges::count_if(routing.corridors, [](const auto& corridor) {
        return corridor->partitions.empty();
      }));
}

/// Which side of the line through @p p and @p q the point @p r lies on. The
/// points are in layout units, so three points on one line come out near
/// zero rather than at it, and the sign is taken against the lengths.
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

/// One chord of a corridor: the partition it runs across and its two ends.
struct Chord {
  std::uint32_t partition = 0;
  Point from;
  Point to;
};

/// The chords a corridor draws, from its source over its crossings to its
/// target.
std::vector<Chord> chordsOf(const CorridorT& corridor) {
  std::vector<Chord> chords;
  if (corridor.partitions.empty() || corridor.source == nullptr ||
      corridor.target == nullptr) {
    return chords;
  }
  auto place = *corridor.source;
  for (std::size_t index = 0; index < corridor.crossings.size(); ++index) {
    chords.push_back({.partition = corridor.partitions[index],
                      .from = place,
                      .to = corridor.crossings[index]});
    place = corridor.crossings[index];
  }
  chords.push_back({.partition = corridor.partitions.back(),
                    .from = place,
                    .to = *corridor.target});
  return chords;
}

/// Checks that no two wires take one slot.
void expectOneWirePerSlot(const CorridorRoutingT& routing) {
  std::set<std::pair<double, double>> taken;
  for (const auto& corridor : routing.corridors) {
    for (const auto& crossing : corridor->crossings) {
      EXPECT_TRUE(taken.emplace(crossing.x(), crossing.y()).second)
          << "two wires cross at (" << crossing.x() << ", " << crossing.y()
          << ")";
    }
  }
}

/// Checks that the crossings of two wires lie at least one crossing pitch
/// apart, on one border or on two. One wire may cross closer to itself, as it
/// does where it passes a corner from one partition into the next.
void expectThePitchBetweenWires(const CorridorRoutingT& routing,
                                const double pitch) {
  for (std::size_t first = 0; first < routing.corridors.size(); ++first) {
    for (std::size_t second = first + 1; second < routing.corridors.size();
         ++second) {
      for (const auto& a : routing.corridors[first]->crossings) {
        for (const auto& b : routing.corridors[second]->crossings) {
          EXPECT_GE(std::hypot(a.x() - b.x(), a.y() - b.y()), pitch)
              << "wires " << first << " and " << second << " cross at ("
              << a.x() << ", " << a.y() << ") and (" << b.x() << ", " << b.y()
              << ")";
        }
      }
    }
  }
}

/// Whether @p chord runs over @p end between its own two ends.
bool runsOver(const Chord& chord, const Point& end) {
  const auto at = [&end](const Point& point) {
    return std::abs(point.x() - end.x()) < 1.0e-6 &&
           std::abs(point.y() - end.y()) < 1.0e-6;
  };
  const auto slack = 1.0e-9 * std::hypot(chord.to.x() - chord.from.x(),
                                         chord.to.y() - chord.from.y());
  return sideOf(chord.from, chord.to, end) == 0 &&
         std::min(chord.from.x(), chord.to.x()) - slack <= end.x() &&
         end.x() <= std::max(chord.from.x(), chord.to.x()) + slack &&
         std::min(chord.from.y(), chord.to.y()) - slack <= end.y() &&
         end.y() <= std::max(chord.from.y(), chord.to.y()) + slack &&
         !at(chord.from) && !at(chord.to);
}

/// Whether two points are one place, up to the round trip through layout
/// units.
bool samePlace(const Point& a, const Point& b) {
  return std::abs(a.x() - b.x()) < 1.0e-6 && std::abs(a.y() - b.y()) < 1.0e-6;
}

/// Checks that no two wires cross inside a partition, that no chord runs over
/// a point where a chord of another wire ends, and that no two wires share an
/// end. Such chords meet the other wire although the two never cross
/// properly.
void expectNoCrossingInsideAPartition(const CorridorRoutingT& routing) {
  std::map<std::uint32_t, std::vector<std::pair<std::size_t, Chord>>> drawn;
  for (std::size_t wire = 0; wire < routing.corridors.size(); ++wire) {
    for (const auto& chord : chordsOf(*routing.corridors[wire])) {
      for (const auto& [other, theirs] : drawn[chord.partition]) {
        if (other == wire) {
          continue;
        }
        EXPECT_FALSE(crosses(chord.from, chord.to, theirs.from, theirs.to))
            << "wires " << wire << " and " << other
            << " cross inside partition " << chord.partition;
        EXPECT_FALSE(runsOver(chord, theirs.from) ||
                     runsOver(chord, theirs.to) ||
                     runsOver(theirs, chord.from) || runsOver(theirs, chord.to))
            << "wires " << wire << " and " << other
            << " meet at an end inside partition " << chord.partition;
        EXPECT_FALSE(samePlace(chord.from, theirs.from) ||
                     samePlace(chord.from, theirs.to) ||
                     samePlace(chord.to, theirs.from) ||
                     samePlace(chord.to, theirs.to))
            << "wires " << wire << " and " << other
            << " share an end inside partition " << chord.partition;
      }
      drawn[chord.partition].emplace_back(wire, chord);
    }
  }
}

/// Checks that no wire is a short: it passes each place once, and two of its
/// chords meet only where one follows the other.
void expectNoShortInsideAWire(const CorridorRoutingT& routing) {
  for (std::size_t wire = 0; wire < routing.corridors.size(); ++wire) {
    const auto& crossings = routing.corridors[wire]->crossings;
    for (std::size_t first = 0; first < crossings.size(); ++first) {
      for (std::size_t second = first + 1; second < crossings.size();
           ++second) {
        EXPECT_FALSE(samePlace(crossings[first], crossings[second]))
            << "wire " << wire << " passes one place twice";
      }
    }
    const auto chords = chordsOf(*routing.corridors[wire]);
    for (std::size_t first = 0; first < chords.size(); ++first) {
      for (std::size_t second = first + 2; second < chords.size(); ++second) {
        const auto& mine = chords[first];
        const auto& theirs = chords[second];
        if (mine.partition != theirs.partition) {
          continue;
        }
        EXPECT_FALSE(crosses(mine.from, mine.to, theirs.from, theirs.to) ||
                     runsOver(mine, theirs.from) || runsOver(mine, theirs.to) ||
                     runsOver(theirs, mine.from) || runsOver(theirs, mine.to))
            << "wire " << wire << " meets itself inside partition "
            << mine.partition;
      }
    }
  }
}

TEST(CorridorRouter, RoutesEveryConnectionOfTheFixture) {
  const auto planned = planMini();
  const auto routing = routeMini(planned);

  ASSERT_EQ(routing.corridors.size(), planned.assignment.connections.size());
  for (const auto& corridor : routing.corridors) {
    EXPECT_NE(corridor->source, nullptr);
    EXPECT_NE(corridor->target, nullptr);
    ASSERT_FALSE(corridor->partitions.empty());
    // A wire names one more partition than it has crossings.
    EXPECT_EQ(corridor->crossings.size() + 1, corridor->partitions.size());
  }
}

TEST(CorridorRouter, GivesEachSlotToOneWire) {
  expectOneWirePerSlot(routeMini(planMini()));
}

TEST(CorridorRouter, KeepsTheCrossingPitchBetweenWiresWhereBordersMeet) {
  // Where partitions meet, the borders end next to one another, and the
  // slots of two borders lie closer than the pitch. Two wires are fed at two
  // such slots, one at each, so that both want to cross next to the corner.
  const auto config = test::miniConfig();
  const auto pitch = crossingPitch(config);
  const auto fixture = planMini();
  std::vector<std::pair<std::uint32_t, Point>> offered;
  for (const auto& slots : routeMini(fixture).slots) {
    for (const auto& position : slots->positions) {
      offered.emplace_back(slots->border, position);
    }
  }
  std::vector<std::pair<Point, Point>> close;
  for (std::size_t i = 0; i < offered.size(); ++i) {
    for (std::size_t j = i + 1; j < offered.size(); ++j) {
      const auto& [firstBorder, a] = offered[i];
      const auto& [secondBorder, b] = offered[j];
      if (firstBorder != secondBorder &&
          std::hypot(a.x() - b.x(), a.y() - b.y()) < pitch) {
        close.emplace_back(a, b);
      }
    }
  }
  ASSERT_FALSE(close.empty())
      << "no two borders of the fixture have slots closer than the pitch";

  const auto& connections = fixture.assignment.connections;
  auto planned = planMini();
  std::size_t routed = 0;
  for (const auto& [a, b] : close) {
    for (std::size_t target = 0; target < connections.size(); ++target) {
      SCOPED_TRACE(std::format("fed at ({}, {}) and ({}, {}), target {}", a.x(),
                               a.y(), b.x(), b.y(), target));
      planned.assignment.connections.clear();
      planned.assignment.feeds.clear();
      for (const auto& [feed, index] :
           {std::pair{a, target}, {b, (target + 1) % connections.size()}}) {
        planned.assignment.connections.push_back(
            std::make_unique<flatbuffers::design::ConnectionT>(
                *connections[index]));
        planned.assignment.feeds.push_back(feed);
      }

      const auto routing = routeMini(planned, {}, config);

      expectThePitchBetweenWires(routing, pitch);
      routed += routing.corridors.size() - unroutedOf(routing);
    }
  }
  EXPECT_GT(routed, 0U);
}

TEST(CorridorRouter, StartsAndEndsEveryWireAtTheCentreOfACell) {
  // The source of a corridor is the cell its feed was placed at, and its
  // target the cell its port is reached at, so both are layout points of
  // cells of the detail grid.
  const auto planned = planMini();
  const auto routing = routeMini(planned);
  const auto detail = buildScene(planned.chip, test::miniConfig()).detail;
  constexpr auto tolerance = 1.0e-9;
  std::size_t ends = 0;
  for (const auto& corridor : routing.corridors) {
    for (const auto* end : {corridor->source.get(), corridor->target.get()}) {
      if (end == nullptr) {
        continue;
      }
      const auto cell = detail.toCell(*end);
      EXPECT_NEAR(cell.x(), std::round(cell.x()), tolerance);
      EXPECT_NEAR(cell.y(), std::round(cell.y()), tolerance);
      ++ends;
    }
  }
  EXPECT_GT(ends, 0U);
}

TEST(CorridorRouter, LeavesAPartitionOnlyIntoANeighbour) {
  const auto planned = planMini();
  const auto routing = routeMini(planned);

  std::set<std::pair<std::uint32_t, std::uint32_t>> borders;
  for (const auto& border : planned.capacity.borders) {
    borders.insert(std::minmax(border->first, border->second));
  }
  for (const auto& corridor : routing.corridors) {
    for (std::size_t index = 0; index + 1 < corridor->partitions.size();
         ++index) {
      EXPECT_TRUE(borders.contains(std::minmax(
          corridor->partitions[index], corridor->partitions[index + 1])));
    }
  }
}

TEST(CorridorRouter, KeepsWiresFromCrossingInsideAPartition) {
  expectNoCrossingInsideAPartition(routeMini(planMini()));
}

TEST(CorridorRouter, CountsAConnectionWithoutAWayAsAFail) {
  // A launcher has no target cell, so a connection to one has nowhere to go.
  auto planned = planMini();
  auto connection = std::make_unique<flatbuffers::design::ConnectionT>();
  connection->target = flatbuffers::design::PortRef(0);
  planned.assignment.connections.push_back(std::move(connection));
  std::optional<std::uint64_t> fails;
  const Report report(
      Detail::Summary,
      {.result = [&fails](const std::vector<Figure>& /*figures*/,
                          const std::optional<std::uint64_t> count) {
        fails = count;
      }});

  const auto routing = routeMini(planned, report);

  ASSERT_EQ(routing.corridors.size(), planned.assignment.connections.size());
  EXPECT_TRUE(routing.corridors.back()->partitions.empty());
  EXPECT_EQ(fails, 1U);
}

TEST(CorridorRouter, GivesTheSameAnswerTwice) {
  const auto planned = planMini();
  const auto bytes = [&planned] {
    ArtifactT artifact;
    artifact.producer = "test";
    artifact.output.Set(routeMini(planned));
    return io::writeArtifact(artifact);
  };
  EXPECT_EQ(bytes(), bytes());
}

TEST(CorridorRouter, ReportsEveryRoundAndTheResult) {
  std::vector<std::pair<std::string, std::vector<Figure>>> entries;
  std::vector<std::vector<Figure>> results;
  std::vector<std::optional<std::uint64_t>> fails;
  std::vector<std::string> steps;
  const Report report(
      Detail::Steps,
      {.line =
           [&steps](const Detail detail, const std::string_view text) {
             if (detail == Detail::Steps) {
               steps.emplace_back(text);
             }
           },
       .entry =
           [&entries](const Detail detail, const std::string_view label,
                      const std::vector<Figure>& figures) {
             if (detail == Detail::Summary) {
               entries.emplace_back(label, figures);
             }
           },
       .result =
           [&](const std::vector<Figure>& figures,
               const std::optional<std::uint64_t> count) {
             results.push_back(figures);
             fails.push_back(count);
           }});

  const auto planned = planMini();
  static_cast<void>(routeMini(planned, report));

  const auto connections = planned.assignment.connections.size();
  ASSERT_FALSE(entries.empty());
  EXPECT_EQ(entries[0].first, "round 1 forward");
  ASSERT_EQ(entries.back().second.size(), 2U);
  EXPECT_EQ(entries.back().second[0].text,
            std::format("routed {}/{}", connections, connections));
  EXPECT_EQ(entries.back().second[1].text, "fails 0");
  EXPECT_EQ(entries.back().second[1].tone, Tone::Good);
  ASSERT_EQ(results.size(), 1U);
  EXPECT_EQ(results[0][0].text,
            std::format("routed {}/{}", connections, connections));
  EXPECT_EQ(fails[0], 0U);
  EXPECT_TRUE(std::ranges::any_of(steps, [&](const std::string& line) {
    return line.starts_with(counted(connections, "connection"));
  }));
}

TEST(CorridorRouter, KeepsItsRulesWhenWiresCompete) {
  // Four copies of every wire are more than the slots of the fixture carry,
  // so wires are ripped up, displaced and left without a way.
  const auto planned = crowded(4);
  for (const std::uint32_t maxRelaxation : {1U, 30U}) {
    SCOPED_TRACE(std::format("max relaxation {}", maxRelaxation));
    Received received;
    const auto routing =
        routeMini(planned, receiving(received, Detail::Summary),
                  sweepConfig(2, maxRelaxation));

    ASSERT_EQ(routing.corridors.size(), planned.assignment.connections.size());
    expectOneWirePerSlot(routing);
    expectThePitchBetweenWires(routing, crossingPitch(test::miniConfig()));
    expectNoCrossingInsideAPartition(routing);
    EXPECT_GT(unroutedOf(routing), 0U);
    EXPECT_LT(unroutedOf(routing), routing.corridors.size());
    EXPECT_EQ(received.fails, unroutedOf(routing));
  }
}

TEST(CorridorRouter, PlacesTheWiresThatNoRoundPlaced) {
  // Without a sweep, every wire is placed in the room the others leave.
  const auto planned = crowded(2);
  Received received;
  const auto routing = routeMini(planned, receiving(received, Detail::Summary),
                                 sweepConfig(0, 30));

  const auto routed = routing.corridors.size() - unroutedOf(routing);
  ASSERT_EQ(received.summary.size(), 1U);
  EXPECT_EQ(received.summary[0].first, "rescue");
  EXPECT_EQ(received.summary[0].second[0].text,
            std::format("placed {}", routed));
  EXPECT_GT(routed, 0U);
  expectOneWirePerSlot(routing);
  expectNoCrossingInsideAPartition(routing);
}

TEST(CorridorRouter, MovesAWireThatHoldsASlotNearTheOneAnotherWants) {
  // Four wires fed at slots of the fixture, found by a search over random
  // feeds. With no sweep, the exchange pass is the only one that moves a
  // wire. The way of the wire to Q1.port0 needs a slot next to one that
  // another wire holds, so the exchange pass has to see the holder of that
  // slot as a wire in the way, or the wire stays without a way.
  const auto fixture = planMini();
  std::vector<Point> offered;
  for (const auto& slots : routeMini(fixture).slots) {
    for (const auto& position : slots->positions) {
      offered.push_back(position);
    }
  }
  ASSERT_GT(offered.size(), 278U);
  const auto portOf = [&fixture](const std::string_view label) {
    const auto found =
        std::ranges::find_if(fixture.chip.ports, [&](const auto& port) {
          return port->label == label;
        });
    EXPECT_NE(found, fixture.chip.ports.end()) << label;
    return static_cast<std::uint32_t>(found - fixture.chip.ports.begin());
  };
  auto planned = planMini();
  planned.assignment.connections.clear();
  planned.assignment.feeds.clear();
  for (const auto& [slot, label] :
       {std::pair<std::size_t, std::string_view>{278, "C12.port0"},
        {150, "Q1.port1"},
        {116, "Q2.port0"},
        {3, "Q1.port0"}}) {
    auto connection = std::make_unique<flatbuffers::design::ConnectionT>();
    connection->target = flatbuffers::design::PortRef(portOf(label));
    planned.assignment.connections.push_back(std::move(connection));
    planned.assignment.feeds.push_back(offered[slot]);
  }

  const auto routing = routeMini(planned, {}, sweepConfig(0, 30));

  EXPECT_EQ(unroutedOf(routing), 0U);
  expectOneWirePerSlot(routing);
  expectThePitchBetweenWires(routing, crossingPitch(test::miniConfig()));
  expectNoCrossingInsideAPartition(routing);
}

TEST(CorridorRouter, RoutesAWireThatCrossesBordersOnItsOwn) {
  // Which slots exist does not depend on which wires are routed, so a wire
  // that crosses borders among all the others still has its way alone.
  const auto planned = planMini();
  const auto full = routeMini(planned);
  const auto crossing =
      std::ranges::find_if(full.corridors, [](const auto& corridor) {
        return !corridor->crossings.empty();
      });
  ASSERT_NE(crossing, full.corridors.end());
  const auto index =
      static_cast<std::size_t>(crossing - full.corridors.begin());

  auto alone = planMini();
  auto connection = std::make_unique<flatbuffers::design::ConnectionT>(
      *alone.assignment.connections[index]);
  const auto feed = alone.assignment.feeds[index];
  alone.assignment.connections.clear();
  alone.assignment.connections.push_back(std::move(connection));
  alone.assignment.feeds = {feed};

  const auto routing = routeMini(alone);

  ASSERT_EQ(routing.corridors.size(), 1U);
  EXPECT_EQ(unroutedOf(routing), 0U);
  EXPECT_FALSE(routing.corridors[0]->crossings.empty());
}

TEST(CorridorRouter, OffersEveryBorderAsManySlotsAsItsBudget) {
  const auto planned = planMini();
  const auto routing = routeMini(planned);

  std::vector<std::size_t> offered(planned.capacity.borders.size(), 0);
  for (const auto& slots : routing.slots) {
    ASSERT_LT(slots->border, offered.size());
    offered[slots->border] += slots->positions.size();
  }
  for (std::size_t border = 0; border < offered.size(); ++border) {
    EXPECT_EQ(offered[border], planned.capacity.borders[border]->budget)
        << "border " << border;
  }
}

TEST(CorridorRouter, RoutesNoWireWithoutAFeedPoint) {
  auto planned = planMini();
  planned.assignment.feeds.clear();
  Received received;

  const auto routing = routeMini(planned, receiving(received, Detail::Summary));

  for (const auto& corridor : routing.corridors) {
    EXPECT_EQ(corridor->source, nullptr);
    EXPECT_NE(corridor->target, nullptr);
    EXPECT_TRUE(corridor->partitions.empty());
  }
  EXPECT_EQ(received.fails, planned.assignment.connections.size());
}

/// The fixture with the feed point of every connection moved to the centre of
/// its target port, shifted by @p offset along x.
Planned fedAtTheTarget(const double offset) {
  auto planned = planMini();
  for (std::size_t index = 0; index < planned.assignment.connections.size();
       ++index) {
    const auto& centre =
        planned.chip
            .ports[planned.assignment.connections[index]->target.index()]
            ->center;
    planned.assignment.feeds[index] = Point(centre.x() + offset, centre.y());
  }
  return planned;
}

TEST(CorridorRouter, NeedsNoCrossingForAWireFedInItsTargetPartition) {
  const auto routing = routeMini(fedAtTheTarget(100.0));

  std::size_t inOne = 0;
  for (const auto& corridor : routing.corridors) {
    if (!corridor->partitions.empty() &&
        corridor->partitions.front() == corridor->partitions.back()) {
      ++inOne;
      EXPECT_EQ(corridor->partitions.size(), 1U);
      EXPECT_TRUE(corridor->crossings.empty());
    }
  }
  EXPECT_GT(inOne, 0U);
}

TEST(CorridorRouter, LeavesAWireFedDeepInsideTheArtworkUnrouted) {
  // The centre of a port lies on its pad, further from free space than a
  // feed point looks for a partition.
  const auto routing = routeMini(fedAtTheTarget(0.0));

  for (const auto& corridor : routing.corridors) {
    EXPECT_EQ(corridor->source, nullptr);
    EXPECT_TRUE(corridor->partitions.empty());
  }
}

TEST(CorridorRouter, CrossesOnlyAtTheSlotsOfTheBordersOfThePlan) {
  // Without borders no wire leaves the partition it is fed in. A target port
  // with a partition of its own stays out of reach, even where its band runs
  // on through the artwork to a partition beyond.
  auto planned = planMini();
  planned.capacity.borders.clear();
  Received received;

  const auto routing = routeMini(planned, receiving(received, Detail::Summary));

  EXPECT_TRUE(routing.slots.empty());
  for (const auto& corridor : routing.corridors) {
    EXPECT_TRUE(corridor->crossings.empty());
    EXPECT_LE(corridor->partitions.size(), 1U);
  }
  EXPECT_EQ(received.fails, unroutedOf(routing));
  EXPECT_GT(unroutedOf(routing), 0U);
}

/// The message the stage refuses a plan with, or an empty one when it routes
/// the plan.
std::string refusalOf(const Planned& planned,
                      const ConfigT& config = test::miniConfig()) {
  try {
    static_cast<void>(routeMini(planned, {}, config));
  } catch (const std::invalid_argument& error) {
    return error.what();
  }
  return {};
}

TEST(CorridorRouter, RefusesAPlanMadeOnAnotherDetailGrid) {
  // The configuration is read again for the stage, so a plan made with
  // other grid settings would lay its outlines across cell edges.
  auto planned = planMini();
  auto config = test::miniConfig();
  config.grid->capacity_cells_x += 1;

  EXPECT_NE(refusalOf(planned, config).find("detail grid"), std::string::npos);
}

TEST(CorridorRouter, RefusesAPlanMadeWithAnotherCrossingPitch) {
  // A border then offers another number of slots than its budget, and the
  // corridors would no longer keep to what the plan promised.
  auto planned = planMini();
  auto config = test::miniConfig();
  config.stages->capacity->crossing_pitch = crossingPitch(config) / 2.0;

  const auto refusal = refusalOf(planned, config);
  EXPECT_NE(refusal.find("another crossing pitch or by another version"),
            std::string::npos)
      << refusal;
  EXPECT_NE(refusal.find("run the capacity step again"), std::string::npos)
      << refusal;
}

TEST(CorridorRouter, RefusesAPartitionThatFallsApartIntoPieces) {
  // The search treats a partition as one region. Two pieces under one label
  // would let a wire enter one piece and leave from the other, over a gap
  // that no wire can cross.
  auto planned = planMini();
  auto& partitions = planned.capacity.partitions;
  std::set<std::pair<std::uint32_t, std::uint32_t>> borders;
  for (const auto& border : planned.capacity.borders) {
    borders.insert(std::minmax(border->first, border->second));
  }
  // Two partitions without a border between them do not touch, so the rings
  // of both under the first label are two pieces.
  std::optional<std::pair<std::size_t, std::size_t>> apart;
  for (std::size_t first = 0; first < partitions.size() && !apart; ++first) {
    for (std::size_t second = first + 1; second < partitions.size(); ++second) {
      if (!borders.contains(std::minmax(partitions[first]->label,
                                        partitions[second]->label))) {
        apart.emplace(first, second);
        break;
      }
    }
  }
  ASSERT_TRUE(apart.has_value());
  for (auto& ring : partitions[apart->second]->outlines) {
    partitions[apart->first]->outlines.push_back(std::move(ring));
  }
  partitions.erase(partitions.begin() +
                   static_cast<std::ptrdiff_t>(apart->second));

  EXPECT_EQ(refusalOf(planned),
            std::format("partition {} of the capacity plan falls apart into "
                        "pieces that do not touch",
                        partitions[apart->first]->label));
}

TEST(CorridorRouter, CompletesTheProgressOfEveryRound) {
  std::map<std::string, std::pair<std::uint64_t, std::uint64_t>> last;
  const Report report(Detail::Summary,
                      {.progress = [&last](const std::string_view task,
                                           const std::string_view /*detail*/,
                                           const std::uint64_t done,
                                           const std::uint64_t total) {
                        last[std::string(task)] = {done, total};
                      }});

  static_cast<void>(routeMini(planMini(), report));

  const auto first = last.find("round 1 forward");
  ASSERT_NE(first, last.end());
  EXPECT_GT(first->second.second, 0U);
  EXPECT_EQ(first->second.first, first->second.second);
}

TEST(CorridorRouter, SweepsTwelveRoundsWithoutACorridorSection) {
  auto config = test::miniConfig();
  config.stages->corridor.reset();
  Received received;

  const auto routing =
      routeMini(planMini(), receiving(received, Detail::Steps), config);

  EXPECT_NE(std::ranges::find(
                received.steps,
                std::format("rounds 12{}max relaxation 30", FIGURE_SEPARATOR)),
            received.steps.end());
  EXPECT_EQ(unroutedOf(routing), 0U);
}

TEST(CorridorRouter, ReportsEveryConnectionAtTheItemsLevel) {
  const auto planned = crowded(2);
  Received received;

  const auto routing = routeMini(planned, receiving(received, Detail::Items),
                                 sweepConfig(1, 30));

  ASSERT_EQ(received.items.size(), routing.corridors.size());
  for (std::size_t index = 0; index < routing.corridors.size(); ++index) {
    const auto& [label, figures] = received.items[index];
    const auto& corridor = *routing.corridors[index];
    EXPECT_EQ(label,
              planned.chip
                  .ports[planned.assignment.connections[index]->target.index()]
                  ->label);
    if (corridor.partitions.empty()) {
      ASSERT_EQ(figures.size(), 1U);
      EXPECT_EQ(figures[0].text, "unrouted");
      EXPECT_EQ(figures[0].tone, Tone::Bad);
      continue;
    }
    ASSERT_EQ(figures.size(), 2U);
    EXPECT_EQ(figures[0].text,
              counted(corridor.partitions.size(), "partition"));
    EXPECT_EQ(figures[1].text, counted(corridor.crossings.size(), "crossing"));
  }
}

// The stage on many assignments: whatever the assignment asks for, the
// corridors keep the rules that the later stages rely on.

/// Checks every rule of the routing against the plan it was made from: one
/// corridor per connection, a way through neighbouring partitions over
/// offered slots, one wire per slot, no wires that meet inside a partition,
/// no wire that meets itself, and as many fails as wires without a way.
void expectTheRulesHold(const Planned& planned, const CorridorRoutingT& routing,
                        const Received& received) {
  ASSERT_EQ(routing.corridors.size(), planned.assignment.connections.size());
  std::set<std::pair<std::uint32_t, std::uint32_t>> borders;
  for (const auto& border : planned.capacity.borders) {
    borders.insert(std::minmax(border->first, border->second));
  }
  std::vector<Point> offered;
  for (const auto& slots : routing.slots) {
    for (const auto& position : slots->positions) {
      offered.push_back(position);
    }
  }
  const auto among = [](const std::vector<Point>& points, const Point& point) {
    return std::ranges::any_of(points, [&point](const Point& candidate) {
      return samePlace(candidate, point);
    });
  };
  for (std::size_t wire = 0; wire < routing.corridors.size(); ++wire) {
    const auto& corridor = *routing.corridors[wire];
    if (corridor.partitions.empty()) {
      EXPECT_TRUE(corridor.crossings.empty()) << "wire " << wire;
      continue;
    }
    EXPECT_NE(corridor.source, nullptr) << "wire " << wire;
    EXPECT_NE(corridor.target, nullptr) << "wire " << wire;
    ASSERT_EQ(corridor.crossings.size() + 1, corridor.partitions.size())
        << "wire " << wire;
    for (std::size_t index = 0; index < corridor.crossings.size(); ++index) {
      const auto& crossing = corridor.crossings[index];
      EXPECT_TRUE(among(offered, crossing))
          << "wire " << wire << " crosses where no slot is offered";
      EXPECT_TRUE(borders.contains(std::minmax(corridor.partitions[index],
                                               corridor.partitions[index + 1])))
          << "wire " << wire << " leaves partition "
          << corridor.partitions[index] << " into one that is no neighbour";
    }
  }
  expectOneWirePerSlot(routing);
  expectThePitchBetweenWires(routing, crossingPitch(test::miniConfig()));
  expectNoCrossingInsideAPartition(routing);
  expectNoShortInsideAWire(routing);
  EXPECT_EQ(received.fails, unroutedOf(routing));
}

/// The smallest box around every polygon of the chip.
std::pair<Point, Point> extentOf(const ChipT& chip) {
  Point low(0.0, 0.0);
  Point high(0.0, 0.0);
  bool first = true;
  for (const auto& polygon : chip.obstacles) {
    for (const auto& vertex : polygon->vertices) {
      if (first) {
        low = vertex;
        high = vertex;
        first = false;
      }
      low = Point(std::min(low.x(), vertex.x()), std::min(low.y(), vertex.y()));
      high =
          Point(std::max(high.x(), vertex.x()), std::max(high.y(), vertex.y()));
    }
  }
  return {low, high};
}

/// An assignment of @p planned drawn at random: between one and twice as many
/// connections as the fixture has, each to a port of the ring or, now and
/// then, to a launcher, which has no target cell. A feed point lies on a
/// launcher slot, on the way between two slots, or anywhere on the chip,
/// pads and artwork included.
AssignmentT randomAssignment(const Planned& planned, SplitMix& random) {
  std::vector<std::uint32_t> targets;
  targets.reserve(planned.assignment.connections.size());
  for (const auto& connection : planned.assignment.connections) {
    targets.push_back(connection->target.index());
  }
  const auto& slots = planned.capacity.launchers;
  const auto [low, high] = extentOf(planned.chip);
  const auto between = [&random](const Point& from, const Point& to) {
    const auto ratio = random.unit();
    return Point(from.x() + (ratio * (to.x() - from.x())),
                 from.y() + (ratio * (to.y() - from.y())));
  };

  AssignmentT assignment;
  const auto count = random.between(
      1, 2 * static_cast<std::int64_t>(planned.assignment.connections.size()));
  for (std::int64_t index = 0; index < count; ++index) {
    auto connection = std::make_unique<flatbuffers::design::ConnectionT>();
    const auto pick = [&random](const std::size_t size) {
      return static_cast<std::size_t>(
          random.between(0, static_cast<std::int64_t>(size) - 1));
    };
    connection->target = flatbuffers::design::PortRef(
        random.unit() < 0.1 ? slots[pick(slots.size())]->port.index()
                            : targets[pick(targets.size())]);
    assignment.connections.push_back(std::move(connection));

    const auto& slot = slots[pick(slots.size())]->position;
    const auto& next = slots[pick(slots.size())]->position;
    const auto draw = random.unit();
    if (draw < 0.3) {
      assignment.feeds.push_back(slot);
    } else if (draw < 0.8) {
      assignment.feeds.push_back(between(slot, next));
    } else {
      // Two draws in one argument list come in an order that the compiler
      // chooses, so each has a statement of its own.
      const auto x = random.unit();
      const auto y = random.unit();
      assignment.feeds.emplace_back(low.x() + (x * (high.x() - low.x())),
                                    low.y() + (y * (high.y() - low.y())));
    }
  }
  return assignment;
}

TEST(CorridorRouter, KeepsItsRulesOnRandomAssignments) {
  // Random assignments with random sweep settings. Many of them ask for more
  // wires than the borders have slots, feed a wire from artwork, or send it
  // to a launcher, so wires compete, are ripped up and are left without a
  // way.
  SplitMix random(71);
  const auto fixture = planMini();
  auto planned = planMini();
  std::size_t routed = 0;
  std::size_t unrouted = 0;
  for (int round = 0; round < 200; ++round) {
    SCOPED_TRACE(std::format("round {}", round));
    planned.assignment = randomAssignment(fixture, random);
    const auto rounds = static_cast<std::uint32_t>(random.between(0, 12));
    const auto maxRelaxation =
        static_cast<std::uint32_t>(random.between(0, 30));
    const auto config = sweepConfig(rounds, maxRelaxation);
    Received received;

    const auto routing =
        routeMini(planned, receiving(received, Detail::Summary), config);

    expectTheRulesHold(planned, routing, received);
    if (HasFailure()) {
      return;
    }
    unrouted += unroutedOf(routing);
    routed += routing.corridors.size() - unroutedOf(routing);
  }
  // The draws cover both outcomes.
  EXPECT_GT(routed, 0U);
  EXPECT_GT(unrouted, 0U);
}

TEST(CorridorRouter, RoutesEveryWireOfEveryAssignmentOfTheFixture) {
  // The assignments that the Assignment stage makes of the fixture, for each
  // port the ring may open at and each feedline utilization that carries its
  // chain of two resonators: every wire finds a way.
  const auto ring = test::miniConfig().ports->sequences->all_outer;
  std::set<std::vector<std::uint32_t>> distinct;
  for (std::size_t first = 0; first < ring.size(); ++first) {
    for (const std::uint32_t utilization : {2U, 3U, 4U}) {
      SCOPED_TRACE(std::format("the ring opens at {}, utilization {}",
                               ring[first], utilization));
      auto config = test::miniConfig();
      std::ranges::rotate(config.ports->sequences->all_outer,
                          config.ports->sequences->all_outer.begin() +
                              static_cast<std::ptrdiff_t>(first));
      config.rules->max_feedline_utilization = utilization;
      Planned planned{
          .chip = test::miniChip(config), .capacity = {}, .assignment = {}};
      planned.capacity = makeWatershedPlanner()->run(planned.chip, config, {});
      const auto global = makeHananMilpRouter()->run(
          planned.chip, planned.capacity, config, {});
      planned.assignment = makeOrderedMilpAssigner()->run(
          planned.chip, planned.capacity, global, config, {});
      std::vector<std::uint32_t> launchers;
      launchers.reserve(planned.assignment.launchers.size());
      for (const auto& launcher : planned.assignment.launchers) {
        launchers.push_back(launcher.index());
      }
      distinct.insert(launchers);
      Received received;

      const auto routing = makePartitionAStarRouter()->run(
          planned.chip, planned.capacity, planned.assignment, config,
          receiving(received, Detail::Summary));

      expectTheRulesHold(planned, routing, received);
      EXPECT_EQ(unroutedOf(routing), 0U);
    }
  }
  // The rotations give the router more than one assignment to route.
  EXPECT_GT(distinct.size(), 1U);
}

} // namespace
} // namespace mqt::scpd::pipeline
