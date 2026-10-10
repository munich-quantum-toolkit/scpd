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
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace mqt::scpd::pipeline {
namespace {

using flatbuffers::artifacts::ArtifactT;
using flatbuffers::artifacts::CorridorT;
using flatbuffers::geometry::Point;

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

/// Checks that no two wires cross inside a partition.
void expectNoCrossingInsideAPartition(const CorridorRoutingT& routing) {
  std::map<std::uint32_t, std::vector<std::pair<std::size_t, Chord>>> drawn;
  for (std::size_t wire = 0; wire < routing.corridors.size(); ++wire) {
    for (const auto& chord : chordsOf(*routing.corridors[wire])) {
      for (const auto& [other, theirs] : drawn[chord.partition]) {
        EXPECT_FALSE(crosses(chord.from, chord.to, theirs.from, theirs.to))
            << "wires " << wire << " and " << other
            << " cross inside partition " << chord.partition;
      }
      drawn[chord.partition].emplace_back(wire, chord);
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

TEST(CorridorRouter, LeavesAPartitionOnlyIntoANeighbour) {
  const auto planned = planMini();
  const auto routing = routeMini(planned);

  std::set<std::pair<std::uint32_t, std::uint32_t>> borders;
  for (const auto& border : planned.capacity.borders) {
    borders.insert(std::minmax(border->first, border->second));
  }
  // A way out of a port's own pocket is on no border, and the artifact marks
  // those slots.
  std::set<std::pair<double, double>> pockets;
  for (const auto& slots : routing.slots) {
    if (slots->pocket) {
      for (const auto& position : slots->positions) {
        pockets.emplace(position.x(), position.y());
      }
    }
  }
  for (const auto& corridor : routing.corridors) {
    for (std::size_t index = 0; index + 1 < corridor->partitions.size();
         ++index) {
      const auto& crossing = corridor->crossings[index];
      if (pockets.contains({crossing.x(), crossing.y()})) {
        continue;
      }
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

  EXPECT_LT(unroutedOf(routing), routing.corridors.size());
  for (const auto& corridor : routing.corridors) {
    if (!corridor->partitions.empty()) {
      EXPECT_EQ(corridor->partitions.size(), 1U);
      EXPECT_TRUE(corridor->crossings.empty());
    }
  }
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

TEST(CorridorRouter, OpensAWayOutOfAPocketThatNoBorderReaches) {
  // Without borders, the only ways between partitions are the approaches of
  // the target ports.
  auto planned = planMini();
  planned.capacity.borders.clear();

  const auto routing = routeMini(planned);

  ASSERT_EQ(routing.slots.size(), 1U);
  EXPECT_TRUE(routing.slots[0]->pocket);
  std::set<std::pair<double, double>> pockets;
  for (const auto& position : routing.slots[0]->positions) {
    pockets.emplace(position.x(), position.y());
  }
  EXPECT_LT(unroutedOf(routing), routing.corridors.size());
  for (const auto& corridor : routing.corridors) {
    for (const auto& crossing : corridor->crossings) {
      EXPECT_TRUE(pockets.contains({crossing.x(), crossing.y()}));
    }
  }
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

} // namespace
} // namespace mqt::scpd::pipeline
