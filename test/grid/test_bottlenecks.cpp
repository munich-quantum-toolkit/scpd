/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// Bottleneck detection, on masks with one narrow place put where the test
// knows it is.

#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/Bottlenecks.hpp"
#include "mqt-scpd/grid/Chambers.hpp"
#include "mqt-scpd/grid/DistanceTransform.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/grid/Voronoi.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace mqt::scpd::grid {
namespace {

constexpr std::uint32_t WIDTH = 61;
constexpr std::uint32_t HEIGHT = 41;
/// The side of the square grid the slotted stub is turned on.
constexpr std::uint32_t SIDE = 61;

GridMetrics unitGrid(const std::uint32_t width = WIDTH, const std::uint32_t height = HEIGHT) {
  return GridMetrics::fit({.minX = 0.0,
                           .minY = 0.0,
                           .maxX = static_cast<double>(width - 1),
                           .maxY = static_cast<double>(height - 1)},
                          width, height);
}

/// A corridor with a pinch: two walls that step towards each other over a few
/// columns in the middle and back out again.
BitGrid pinchedCorridor(const std::uint32_t pinchHalfHeight) {
  BitGrid mask(WIDTH, HEIGHT);
  constexpr std::uint32_t OPEN_HALF = 15;
  constexpr std::uint32_t MIDDLE = HEIGHT / 2;
  constexpr std::uint32_t PINCH_FROM = 28;
  constexpr std::uint32_t PINCH_TO = 32;

  for (std::uint32_t x = 0; x < WIDTH; ++x) {
    const auto half = (x >= PINCH_FROM && x <= PINCH_TO) ? pinchHalfHeight : OPEN_HALF;
    for (std::uint32_t y = 0; y < HEIGHT; ++y) {
      if (y + half < MIDDLE || y > MIDDLE + half) {
        mask.setCell(x, y);
      }
    }
  }
  return mask;
}

/// Everything a bottleneck search needs, derived from one mask.
struct Scene {
  BitGrid mask;
  GridMetrics grid;
  MedialAxis axis;
  std::vector<std::uint32_t> distance;
};

Scene sceneOf(BitGrid mask) {
  auto grid = unitGrid(mask.width(), mask.height());
  auto distance = squaredDistanceTransform(mask);
  auto axis = rasterizeMedialAxis(mask, grid, medialAxis(mask));
  return {std::move(mask), grid, std::move(axis), std::move(distance)};
}

TEST(Bottlenecks, FindsThePinchOfACorridorAndSpansIt) {
  const auto scene = sceneOf(pinchedCorridor(3));
  const auto found = findBottlenecks(scene.mask, scene.axis, scene.distance, scene.grid);
  ASSERT_FALSE(found.empty());

  // Every bottleneck runs between two obstacle cells, and across the
  // corridor rather than along it: the two ends differ in y far more than
  // in x.
  bool spansThePinch = false;
  for (const auto& bottleneck : found) {
    EXPECT_TRUE(scene.mask.test(bottleneck.first));
    EXPECT_TRUE(scene.mask.test(bottleneck.second));

    const auto x0 = static_cast<int>(bottleneck.first % WIDTH);
    const auto y0 = static_cast<int>(bottleneck.first / WIDTH);
    const auto x1 = static_cast<int>(bottleneck.second % WIDTH);
    const auto y1 = static_cast<int>(bottleneck.second / WIDTH);
    if (std::abs(y1 - y0) > std::abs(x1 - x0) && x0 >= 26 && x0 <= 34) {
      spansThePinch = true;
    }
  }
  EXPECT_TRUE(spansThePinch);
}

TEST(Bottlenecks, FindsNothingWhereEveryPlaceIsWide) {
  // A corridor of constant width has no place narrower than the admission
  // threshold, so nothing is a bottleneck.
  const auto scene = sceneOf(pinchedCorridor(15));
  const auto found = findBottlenecks(scene.mask, scene.axis, scene.distance, scene.grid);
  EXPECT_TRUE(found.empty());
}

TEST(Bottlenecks, IsTheSameOnASecondRun) {
  const auto scene = sceneOf(pinchedCorridor(3));
  const auto first = findBottlenecks(scene.mask, scene.axis, scene.distance, scene.grid);
  const auto second = findBottlenecks(scene.mask, scene.axis, scene.distance, scene.grid);

  ASSERT_EQ(first.size(), second.size());
  for (std::size_t index = 0; index < first.size(); ++index) {
    EXPECT_EQ(first[index].first, second[index].first);
    EXPECT_EQ(first[index].second, second[index].second);
    EXPECT_EQ(first[index].saddle, second[index].saddle);
  }
}

/// A corridor of half height 8 whose upper wall steps one cell in at every
/// sixth column, the unevenness a raster gives a wall, with a pinch of half
/// height 3 in the middle.
BitGrid unevenPinchedCorridor() {
  BitGrid mask(WIDTH, HEIGHT);
  constexpr std::uint32_t OPEN_HALF = 8;
  constexpr std::uint32_t PINCH_HALF = 3;
  constexpr std::uint32_t MIDDLE = HEIGHT / 2;
  constexpr std::uint32_t PINCH_FROM = 28;
  constexpr std::uint32_t PINCH_TO = 32;

  for (std::uint32_t x = 0; x < WIDTH; ++x) {
    const bool pinch = x >= PINCH_FROM && x <= PINCH_TO;
    const auto half = pinch ? PINCH_HALF : OPEN_HALF;
    const std::uint32_t bump = (!pinch && x % 6 == 0) ? 1 : 0;
    for (std::uint32_t y = 0; y < HEIGHT; ++y) {
      if (y + half < MIDDLE || y + bump > MIDDLE + half) {
        mask.setCell(x, y);
      }
    }
  }
  return mask;
}

/// Whether both ends of a bottleneck lie in the columns around the pinch.
bool atThePinch(const Bottleneck& bottleneck) {
  const auto x0 = bottleneck.first % WIDTH;
  const auto x1 = bottleneck.second % WIDTH;
  return x0 >= 26 && x0 <= 34 && x1 >= 26 && x1 <= 34;
}

TEST(Bottlenecks, DropsTheUnevennessOfAWallButKeepsThePinch) {
  const auto scene = sceneOf(unevenPinchedCorridor());
  const auto every =
      findBottlenecks(scene.mask, scene.axis, scene.distance, scene.grid);
  // Without the rule every bump is a narrowing of its own.
  ASSERT_TRUE(std::ranges::any_of(
      every, [](const auto& one) { return !atThePinch(one); }));

  const auto kept = findBottlenecks(scene.mask, scene.axis, scene.distance,
                                    scene.grid, {.minimumRise = 2.0});
  ASSERT_FALSE(kept.empty());
  EXPECT_TRUE(std::ranges::all_of(kept, atThePinch));
}

TEST(Bottlenecks, KeepsAPinchOnlyWhenItIsDeeperThanTheMinimumRise) {
  // The pinch leaves 4 cells of clearance and the corridor 16 on both sides
  // of it, a rise of 12.
  const auto scene = sceneOf(pinchedCorridor(3));
  const auto shallower = findBottlenecks(scene.mask, scene.axis, scene.distance,
                                         scene.grid, {.minimumRise = 11.0});
  EXPECT_TRUE(std::ranges::any_of(shallower, atThePinch));

  const auto deeper = findBottlenecks(scene.mask, scene.axis, scene.distance,
                                      scene.grid, {.minimumRise = 13.0});
  EXPECT_TRUE(deeper.empty());
}

TEST(Bottlenecks, KeepsOneCutOfAPinchOfEvenWidth) {
  // The pinch is five columns of one clearance: every column is a minimum
  // of the axis, and they are one narrowing.
  const auto scene = sceneOf(pinchedCorridor(3));
  const auto found = findBottlenecks(scene.mask, scene.axis, scene.distance,
                                     scene.grid, {.minimumRise = 2.0});
  ASSERT_EQ(std::ranges::count_if(found, atThePinch), 1);
  // The one cut lies in the middle of the five columns.
  const auto cut = std::ranges::find_if(found, atThePinch);
  EXPECT_EQ(cut->saddle % WIDTH, 30U);
}

TEST(Bottlenecks, DropsOneThatWouldRunOverATarget) {
  const auto scene = sceneOf(pinchedCorridor(3));
  const auto without = findBottlenecks(scene.mask, scene.axis, scene.distance, scene.grid);
  ASSERT_FALSE(without.empty());

  // Putting a target on every cell of the first bottleneck's line removes
  // exactly that bottleneck: a port is not a wall that capacity divides
  // around.
  std::vector<std::size_t> targets;
  const auto& gone = without.front();
  const auto x0 = static_cast<int>(gone.first % WIDTH);
  const auto y0 = static_cast<int>(gone.first / WIDTH);
  const auto x1 = static_cast<int>(gone.second % WIDTH);
  const auto y1 = static_cast<int>(gone.second / WIDTH);
  targets.push_back(static_cast<std::size_t>(((y0 + y1) / 2) * WIDTH) +
                    static_cast<std::size_t>((x0 + x1) / 2));

  const auto with = findBottlenecks(scene.mask, scene.axis, scene.distance, scene.grid,
                                    {.targets = targets});
  EXPECT_LT(with.size(), without.size());
}

/// A stub standing up from the bottom edge under a wall along the top, with
/// a slot in the middle of its top face where a terminal lies: the walls,
/// labelled 1 left of the slot, 2 right of it and 3 for the wall above, and
/// the free cells of the slot. Turned by `turn` quarter turns about the
/// middle of a square grid, so that the stub faces each of the four ways;
/// with `fallsAway` the wall above falls away right of the slot, with
/// `leansIn` it comes closer left of the slot, and with `onABorder` the stub
/// stands on a wall along the bottom, labelled 4.
struct SlottedStub {
  BitGrid mask{SIDE, SIDE};
  std::vector<std::uint32_t> wallOf =
      std::vector<std::uint32_t>(SIDE * SIDE, 0);
  std::vector<std::size_t> slot;
};

SlottedStub slottedStub(const int turn, const bool fallsAway = false,
                        const bool onABorder = false,
                        const bool leansIn = false) {
  constexpr std::uint32_t STUB_FROM = 20;
  constexpr std::uint32_t STUB_TO = 40;
  constexpr std::uint32_t TIP = 21;
  constexpr std::uint32_t WALL = 30;
  constexpr std::uint32_t SLOT_FROM = 29;
  constexpr std::uint32_t SLOT_TO = 31;
  constexpr std::uint32_t SLOT_DEPTH = 4;
  // A cell of the upright stub, turned.
  const auto turned = [turn](std::uint32_t x, std::uint32_t y) {
    for (int k = 0; k < turn; ++k) {
      const auto nx = SIDE - 1 - y;
      y = x;
      x = nx;
    }
    return (static_cast<std::size_t>(y) * SIDE) + x;
  };
  SlottedStub stub;
  for (std::uint32_t y = 0; y < SIDE; ++y) {
    for (std::uint32_t x = 0; x < SIDE; ++x) {
      const auto cell = turned(x, y);
      // Right of the slot's middle the wall may fall away, two cells for a
      // cell.
      constexpr std::uint32_t SLOT_MIDDLE = (SLOT_FROM + SLOT_TO) / 2;
      // Left of the slot it may lean in towards the stub's corner, a cell for
      // three.
      auto wall =
          fallsAway && x > SLOT_MIDDLE ? WALL + (2 * (x - SLOT_MIDDLE)) : WALL;
      if (leansIn && x < SLOT_FROM) {
        wall -= (SLOT_FROM - x) / 3;
      }
      if (onABorder && y == 0 && (x < STUB_FROM || x > STUB_TO)) {
        stub.mask.set(cell);
        stub.wallOf[cell] = 4;
      } else if (y >= wall) {
        stub.mask.set(cell);
        stub.wallOf[cell] = 3;
      } else if (x >= STUB_FROM && x <= STUB_TO && y <= TIP) {
        if (x >= SLOT_FROM && x <= SLOT_TO && y + SLOT_DEPTH > TIP) {
          stub.slot.push_back(cell);
          continue;
        }
        stub.mask.set(cell);
        stub.wallOf[cell] = x < SLOT_FROM ? 1 : 2;
      }
    }
  }
  return stub;
}

std::vector<Bottleneck> slottedCuts(const SlottedStub& stub,
                                    const Scene& scene) {
  return findBottlenecks(scene.mask, scene.axis, scene.distance, scene.grid,
                         {.sameNarrowing = 40.0,
                          .minimumRise = 2.0,
                          .slots = stub.slot,
                          .wallOf = stub.wallOf});
}

TEST(Bottlenecks, ATerminalSlotPartsTheNarrowingInFrontOfIt) {
  // One narrowing runs across the whole top face of the stub; with the slot
  // it is two, one on either side of the terminal, however close they are
  // and whichever way the stub faces.
  for (int turn = 0; turn < 4; ++turn) {
    const auto stub = slottedStub(turn);
    const auto scene = sceneOf(stub.mask);
    const auto found = slottedCuts(stub, scene);
    ASSERT_EQ(found.size(), 2U) << "turned " << turn;
    std::vector<std::uint32_t> flanks;
    for (const auto& gate : found) {
      const auto low =
          std::min(stub.wallOf[gate.first], stub.wallOf[gate.second]);
      const auto high =
          std::max(stub.wallOf[gate.first], stub.wallOf[gate.second]);
      EXPECT_EQ(high, 3U) << "turned " << turn;
      flanks.push_back(low);
    }
    std::ranges::sort(flanks);
    EXPECT_EQ(flanks, (std::vector<std::uint32_t>{1, 2})) << "turned " << turn;
  }
}

TEST(Bottlenecks, ASideNarrowestAtTheSlotIsCutBesideIt) {
  // The wall falls away over the right half of the face, so on that side
  // the clearance rises from the fork in front of the slot and no cell of
  // the axis there is a minimum. The side is cut all the same.
  for (int turn = 0; turn < 4; ++turn) {
    const auto stub = slottedStub(turn, true);
    const auto scene = sceneOf(stub.mask);
    const auto found = slottedCuts(stub, scene);
    std::vector<std::uint32_t> flanks;
    for (const auto& gate : found) {
      flanks.push_back(
          std::min(stub.wallOf[gate.first], stub.wallOf[gate.second]));
    }
    std::ranges::sort(flanks);
    EXPECT_EQ(flanks, (std::vector<std::uint32_t>{1, 2})) << "turned " << turn;
    const auto chambers = chambersOf(scene.mask, found, scene.grid);
    const auto own = chambers.of[stub.slot.front()];
    for (std::size_t gate = 0; gate < found.size(); ++gate) {
      EXPECT_TRUE(std::ranges::find(chambers.beside[gate], own) !=
                  chambers.beside[gate].end())
          << "turned " << turn;
    }
  }
}

TEST(Bottlenecks, AStubOnABorderIsCutOnBothSidesOfItsSlot) {
  // A stub on a border, as a launcher stands, under a wall that comes
  // closer towards its left corner: one cut on each side of the terminal,
  // and only one, however the narrowing on the left is shaped.
  for (int turn = 0; turn < 4; ++turn) {
    const auto stub = slottedStub(turn, false, true, true);
    const auto scene = sceneOf(stub.mask);
    const auto found = slottedCuts(stub, scene);
    std::vector<std::uint32_t> flanks;
    for (const auto& gate : found) {
      const auto low =
          std::min(stub.wallOf[gate.first], stub.wallOf[gate.second]);
      const auto high =
          std::max(stub.wallOf[gate.first], stub.wallOf[gate.second]);
      if (high == 3) {
        flanks.push_back(low);
      }
    }
    std::ranges::sort(flanks);
    EXPECT_EQ(flanks, (std::vector<std::uint32_t>{1, 2})) << "turned " << turn;
  }
}

TEST(Bottlenecks, TheTwoCutsBesideASlotCloseItsTerminalsChamber) {
  for (int turn = 0; turn < 4; ++turn) {
    const auto stub = slottedStub(turn);
    const auto scene = sceneOf(stub.mask);
    const auto found = slottedCuts(stub, scene);
    const auto chambers = chambersOf(scene.mask, found, scene.grid);
    // The slot and the room in front of it are one chamber, which both cuts
    // close off from the room to either side.
    const auto own = chambers.of[stub.slot.front()];
    ASSERT_NE(own, NO_CHAMBER) << "turned " << turn;
    for (const auto cell : stub.slot) {
      EXPECT_EQ(chambers.of[cell], own) << "turned " << turn;
    }
    for (std::size_t gate = 0; gate < found.size(); ++gate) {
      EXPECT_EQ(chambers.beside[gate].size(), 2U) << "turned " << turn;
      EXPECT_TRUE(std::ranges::find(chambers.beside[gate], own) !=
                  chambers.beside[gate].end())
          << "turned " << turn;
    }
  }
}

/// A port run as two thin walls standing up from a border under a wall along
/// the top, the space between them its slot: the walls, labelled 1 for the
/// left one, 2 for the right one, 3 for the wall above and 4 for the border,
/// and the free cells between the two walls. Turned by `turn` quarter turns
/// about the middle of a square grid, as `slottedStub` is.
SlottedStub walledSlot(const int turn) {
  constexpr std::uint32_t LEFT = 24;
  constexpr std::uint32_t RIGHT = 36;
  constexpr std::uint32_t TIP = 21;
  constexpr std::uint32_t WALL = 35;
  const auto turned = [turn](std::uint32_t x, std::uint32_t y) {
    for (int k = 0; k < turn; ++k) {
      const auto nx = SIDE - 1 - y;
      y = x;
      x = nx;
    }
    return (static_cast<std::size_t>(y) * SIDE) + x;
  };
  SlottedStub stub;
  for (std::uint32_t y = 0; y < SIDE; ++y) {
    for (std::uint32_t x = 0; x < SIDE; ++x) {
      const auto cell = turned(x, y);
      std::uint32_t wall = 0;
      if (y == 0) {
        wall = 4;
      } else if (y >= WALL) {
        wall = 3;
      } else if (y <= TIP && (x == LEFT || x == RIGHT)) {
        wall = x == LEFT ? 1 : 2;
      } else if (y <= TIP && x > LEFT && x < RIGHT) {
        stub.slot.push_back(cell);
      }
      if (wall != 0) {
        stub.mask.set(cell);
        stub.wallOf[cell] = wall;
      }
    }
  }
  return stub;
}

TEST(Bottlenecks, ASlotBetweenTwoThinWallsIsCutAtBothWallEnds) {
  // The room in front of the open end narrows at the end of each wall, and
  // the clearance rises too little from there to the fork in front of the
  // open end: the fork counts as the rise because an arm of it runs into the
  // slot. One cut runs from each wall end to the wall above, none across the
  // open end between the two walls, and the two close off the terminal's
  // chamber.
  for (int turn = 0; turn < 4; ++turn) {
    const auto stub = walledSlot(turn);
    const auto scene = sceneOf(stub.mask);
    const auto found = slottedCuts(stub, scene);
    std::vector<std::uint32_t> flanks;
    for (const auto& gate : found) {
      const auto low =
          std::min(stub.wallOf[gate.first], stub.wallOf[gate.second]);
      const auto high =
          std::max(stub.wallOf[gate.first], stub.wallOf[gate.second]);
      EXPECT_FALSE(low == 1 && high == 2) << "turned " << turn;
      if (high == 3) {
        flanks.push_back(low);
      }
    }
    std::ranges::sort(flanks);
    ASSERT_EQ(flanks, (std::vector<std::uint32_t>{1, 2})) << "turned " << turn;
    const auto chambers = chambersOf(scene.mask, found, scene.grid);
    const auto own = chambers.of[stub.slot.front()];
    ASSERT_NE(own, NO_CHAMBER) << "turned " << turn;
    for (const auto cell : stub.slot) {
      EXPECT_EQ(chambers.of[cell], own) << "turned " << turn;
    }
    for (std::size_t gate = 0; gate < found.size(); ++gate) {
      const auto low = std::min(stub.wallOf[found[gate].first],
                                stub.wallOf[found[gate].second]);
      const auto high = std::max(stub.wallOf[found[gate].first],
                                 stub.wallOf[found[gate].second]);
      if (high == 3 && low <= 2) {
        EXPECT_EQ(chambers.beside[gate].size(), 2U) << "turned " << turn;
        EXPECT_TRUE(std::ranges::find(chambers.beside[gate], own) !=
                    chambers.beside[gate].end())
            << "turned " << turn;
      }
    }
  }
}

TEST(Bottlenecks, RefusesInputsThatDoNotDescribeOneGrid) {
  const auto scene = sceneOf(pinchedCorridor(3));
  EXPECT_THROW(static_cast<void>(
                   findBottlenecks(scene.mask, scene.axis, scene.distance, unitGrid(10, 10))),
               std::invalid_argument);

  const std::vector<std::uint32_t> shortDistance(4, 0);
  EXPECT_THROW(
      static_cast<void>(findBottlenecks(scene.mask, scene.axis, shortDistance, scene.grid)),
      std::invalid_argument);
}

TEST(BottleneckCapacity, CountsTheWiresThatFitTheGap) {
  // Ten cells apart on a grid of ten layout units per cell is a gap of 100.
  const auto grid = GridMetrics::fit({.minX = 0.0, .minY = 0.0, .maxX = 990.0, .maxY = 990.0},
                                     100, 100);
  ASSERT_DOUBLE_EQ(grid.cellWidth, 10.0);

  const Bottleneck across{.first = 0, .second = 10, .saddle = 5};
  // A gap of 100 against a pitch of 30 holds three wires, rounded up to four
  // where neither end sits on a reserved cell.
  EXPECT_EQ(bottleneckCapacity(across, grid, 25.0, 5.0, false), 4U);
  EXPECT_EQ(bottleneckCapacity(across, grid, 25.0, 5.0, true), 3U);
}

TEST(BottleneckCapacity, CarriesNothingThroughAGapBelowTheObstacleClearance) {
  const auto grid = GridMetrics::fit({.minX = 0.0, .minY = 0.0, .maxX = 990.0, .maxY = 990.0},
                                     100, 100);
  const Bottleneck touching{.first = 0, .second = 1, .saddle = 0};
  EXPECT_EQ(bottleneckCapacity(touching, grid, 185.0, 25.0, false), 0U);
}

} // namespace
} // namespace mqt::scpd::grid
