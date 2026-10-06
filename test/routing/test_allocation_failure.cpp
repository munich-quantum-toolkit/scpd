/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The tests of this file replace the global operator new, so they build into
// a test binary of their own.

#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/routing/DubinsRouter.hpp"
#include "mqt-scpd/routing/Heading.hpp"
#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/Primitives.hpp"
#include "mqt-scpd/routing/SearchScratch.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <memory>
#include <new>
#include <utility>
#include <vector>

namespace {

/// The number of allocations that succeed before one fails, or a negative
/// number while no allocation fails. The count is negative unless a test sets
/// it, so every other allocation of the program behaves as usual.
int& allocationsBeforeFailure() {
  static int count = -1;
  return count;
}

} // namespace

// A replacement of the global allocation functions has to take raw memory
// from malloc and give it back to free.
// NOLINTBEGIN(cppcoreguidelines-no-malloc,cppcoreguidelines-owning-memory)

/// Allocates as the standard operator new does, but throws std::bad_alloc
/// once when allocationsBeforeFailure() reaches zero.
void* operator new(const std::size_t size) {
  int& count = allocationsBeforeFailure();
  if (count == 0) {
    count = -1;
    throw std::bad_alloc();
  }
  if (count > 0) {
    --count;
  }
  void* memory = std::malloc(size == 0 ? 1 : size);
  if (memory == nullptr) {
    throw std::bad_alloc();
  }
  return memory;
}

/// Releases the memory of the replaced operator new.
void operator delete(void* memory) noexcept { std::free(memory); }

/// Releases the memory of the replaced operator new.
void operator delete(void* memory, std::size_t /*size*/) noexcept {
  std::free(memory);
}

// NOLINTEND(cppcoreguidelines-no-malloc,cppcoreguidelines-owning-memory)

namespace {

using namespace mqt::scpd;
using namespace mqt::scpd::routing;

constexpr uint32_t WIDTH = 120;
constexpr uint32_t HEIGHT = 80;
constexpr SearchParams PARAMS{.startStraightLength = 5,
                              .endStraightLength = 5,
                              .minRadius = 5,
                              .bendPenalty = 300};

/// A router over an empty grid, with its scratch and its corridor.
struct Fixture {
  std::shared_ptr<const MovePrimitives> primitives;
  SearchScratch scratch{WIDTH, HEIGHT};
  grid::BitGrid outsideCorridor{WIDTH, HEIGHT};
  DubinsRouter router;

  explicit Fixture(std::shared_ptr<const MovePrimitives> shared)
      : primitives(std::move(shared)), router(primitives, scratch, PARAMS) {
    router.attachCorridor(&outsideCorridor);
  }
};

/// A straight run toward positive y along one column of the whole grid.
Path columnRun(const MovePrimitives& primitives, const uint32_t x) {
  Path run;
  for (uint32_t y = 0; y < HEIGHT; ++y) {
    run.push_back(
        {.x = x, .y = y, .heading = 4, .primitive = primitives.straight(4)});
  }
  return run;
}

/// What a router answers about every cell outside a search: per cell, bit
/// @c h when a straight step on heading @c h may enter, bit 8 when a turn
/// may touch it, and the constraint mask from bit 9 on.
std::vector<uint32_t> answers(const DubinsRouter& router) {
  std::vector<uint32_t> result;
  result.reserve(static_cast<std::size_t>(WIDTH) * HEIGHT);
  for (uint32_t y = 0; y < HEIGHT; ++y) {
    for (uint32_t x = 0; x < WIDTH; ++x) {
      uint32_t bits = 0;
      for (Heading heading = 0; heading < NUM_HEADINGS; ++heading) {
        if (router.crossingAllowedOrthogonal(x, y, heading)) {
          bits |= 1U << heading;
        }
      }
      if (router.turnAllowedOrthogonal(x, y)) {
        bits |= 1U << 8U;
      }
      bits |= static_cast<uint32_t>(router.constraintMaskAt(x, y)) << 9U;
      result.push_back(bits);
    }
  }
  return result;
}

/// Counts the cells where a router answers otherwise than expected.
std::size_t differingCells(const DubinsRouter& router,
                           const std::vector<uint32_t>& expected) {
  const std::vector<uint32_t> actual = answers(router);
  std::size_t differing = 0;
  for (std::size_t cell = 0; cell < actual.size(); ++cell) {
    differing += actual[cell] != expected[cell] ? 1U : 0U;
  }
  return differing;
}

/// Runs a call again and again, and fails one more allocation of the step
/// under test each time: the first, the second, and so on, until the step
/// completes.
/// @param call The call. It receives a function that arms the failure, and
/// calls it after its preparations, right before the step under test.
/// @param check The check that runs after each failed call, with the number
/// of allocations that succeeded before the failure.
/// @return The number of calls that failed.
int failEachAllocation(
    const std::function<void(const std::function<void()>& arm)>& call,
    const std::function<void(int)>& check) {
  int failures = 0;
  for (int allocation = 0; allocation < 100'000; ++allocation) {
    bool failed = false;
    try {
      call([allocation] { allocationsBeforeFailure() = allocation; });
    } catch (const std::bad_alloc&) {
      failed = true;
    }
    allocationsBeforeFailure() = -1;
    if (!failed) {
      return failures;
    }
    ++failures;
    check(allocation);
  }
  ADD_FAILURE() << "the call never completed";
  return failures;
}

TEST(AllocationFailure, AFailedOrthogonalRouteLeavesNoCrossingRule) {
  const auto primitives = std::make_shared<const MovePrimitives>(5);
  const Path feedline = columnRun(*primitives, 60);
  const RoutingObjective across{
      .source = {.x = 10, .y = 40, .heading = 6, .primitive = 0},
      .target = {.x = 110, .y = 40, .heading = 6, .primitive = 0}};
  Fixture fresh(primitives);
  fresh.router.setSingleCrossingFeedline(&feedline, 6, 3);
  const std::vector<uint32_t> expected = answers(fresh.router);
  const Path expectedPath = fresh.router.routeOrthogonal(across);
  ASSERT_FALSE(expectedPath.empty());

  // The single-crossing rule exists only while the search runs. After any
  // failed allocation, the router answers as a router that never routed,
  // and it routes alike.
  std::unique_ptr<Fixture> f;
  const int failures = failEachAllocation(
      [&](const std::function<void()>& arm) {
        f = std::make_unique<Fixture>(primitives);
        f->router.setSingleCrossingFeedline(&feedline, 6, 3);
        arm();
        static_cast<void>(f->router.routeOrthogonal(across));
      },
      [&](const int allocation) {
        EXPECT_EQ(differingCells(f->router, expected), 0U) << allocation;
        EXPECT_EQ(f->router.routeOrthogonal(across), expectedPath)
            << allocation;
      });
  EXPECT_GT(failures, 0);
}

TEST(AllocationFailure, AFailedExemptionKeepsThePreviousOne) {
  const auto primitives = std::make_shared<const MovePrimitives>(5);
  const Path wire = columnRun(*primitives, 30);
  std::vector<uint32_t> previous;
  previous.reserve(10);
  for (uint32_t y = 30; y < 40; ++y) {
    previous.push_back((y * WIDTH) + 30);
  }
  std::vector<uint32_t> next;
  next.reserve(HEIGHT);
  for (uint32_t y = 0; y < HEIGHT; ++y) {
    next.push_back((y * WIDTH) + 31);
  }
  // With constraints and an exemption, and with no crossing rule at all.
  for (const bool constrained : {true, false}) {
    const auto prepare = [&](DubinsRouter& router) {
      if (constrained) {
        router.buildOrthogonalConstraints({wire}, {}, 4);
        router.setCrossingExemption(previous);
      }
    };
    Fixture before(primitives);
    prepare(before.router);
    const std::vector<uint32_t> expected = answers(before.router);
    before.router.setCrossingExemption({});
    const std::vector<uint32_t> expectedWithout = answers(before.router);

    std::unique_ptr<Fixture> f;
    const int failures = failEachAllocation(
        [&](const std::function<void()>& arm) {
          f = std::make_unique<Fixture>(primitives);
          prepare(f->router);
          arm();
          f->router.setCrossingExemption(next);
        },
        [&](const int allocation) {
          EXPECT_EQ(differingCells(f->router, expected), 0U)
              << constrained << " " << allocation;
          f->router.setCrossingExemption({});
          EXPECT_EQ(differingCells(f->router, expectedWithout), 0U)
              << constrained << " " << allocation;
        });
    EXPECT_GT(failures, 0) << constrained;
  }
}

TEST(AllocationFailure, AFailedConstraintBuildKeepsThePreviousConstraints) {
  const auto primitives = std::make_shared<const MovePrimitives>(5);
  const Path previousWire = columnRun(*primitives, 90);
  const Path nextWire = columnRun(*primitives, 30);
  // The free path of the probe crosses both columns on a diagonal.
  const RoutingObjective probe{
      .source = {.x = 5, .y = 10, .heading = 5, .primitive = 0},
      .target = {.x = 115, .y = 70, .heading = 5, .primitive = 0}};
  for (const bool constrained : {true, false}) {
    const auto prepare = [&](DubinsRouter& router) {
      if (constrained) {
        router.buildOrthogonalConstraints({previousWire}, {}, 4);
      }
    };
    Fixture before(primitives);
    prepare(before.router);
    const std::vector<uint32_t> expected = answers(before.router);
    const Path expectedPath = before.router.routeOrthogonal(probe);
    ASSERT_FALSE(expectedPath.empty()) << constrained;

    // The search reads the constraints through the rule bytes, so the probe
    // shows constraints that the rule bytes do not carry.
    std::unique_ptr<Fixture> f;
    const int failures = failEachAllocation(
        [&](const std::function<void()>& arm) {
          f = std::make_unique<Fixture>(primitives);
          prepare(f->router);
          arm();
          f->router.buildOrthogonalConstraints({nextWire}, {}, 4);
        },
        [&](const int allocation) {
          EXPECT_EQ(differingCells(f->router, expected), 0U)
              << constrained << " " << allocation;
          EXPECT_EQ(f->router.routeOrthogonal(probe), expectedPath)
              << constrained << " " << allocation;
        });
    EXPECT_GT(failures, 0) << constrained;
  }
}

} // namespace
