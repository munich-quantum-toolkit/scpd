/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The registry: a name, a factory and a list. It exists so that a second
// implementation of a stage is a new file rather than a refactor.

#include "MiniFixture.hpp"
#include "mqt-scpd/flatbuffers/config.hpp"
#include "mqt-scpd/pipeline/CapacityPlanner.hpp"
#include "mqt-scpd/pipeline/Registry.hpp"
#include "mqt-scpd/pipeline/Report.hpp"
#include "mqt-scpd/pipeline/Stages.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace mqt::scpd::pipeline {
namespace {

/// A planner that answers without doing anything, for the registry's own tests.
class StubPlanner final : public ICapacityPlanner {
public:
  [[nodiscard]] CapacityPlanT run(const ChipT& /*chip*/,
                                  const ConfigT& /*config*/,
                                  const Report& /*report*/) const override {
    return {};
  }
};

TEST(Registry, BuildsWhatANameStandsFor) {
  Registry<ICapacityPlanner> registry;
  registry.add("stub", [] { return std::make_unique<StubPlanner>(); });

  EXPECT_TRUE(registry.contains("stub"));
  EXPECT_NE(registry.make("stub"), nullptr);
}

TEST(Registry, ListsItsNamesInOrder) {
  Registry<ICapacityPlanner> registry;
  registry.add("zeta", [] { return std::make_unique<StubPlanner>(); });
  registry.add("alpha", [] { return std::make_unique<StubPlanner>(); });

  // The order is the map's, not the insertion order, so a listing is the same
  // on every run and every platform.
  EXPECT_EQ(registry.names(), (std::vector<std::string>{"alpha", "zeta"}));
}

TEST(Registry, RefusesAnEmptyOrRepeatedName) {
  Registry<ICapacityPlanner> registry;
  const auto factory = [] { return std::make_unique<StubPlanner>(); };
  registry.add("taken", factory);

  EXPECT_THROW(registry.add("", factory), std::invalid_argument);
  EXPECT_THROW(registry.add("taken", factory), std::invalid_argument);
}

TEST(Registry, NamesWhatIsRegisteredWhenAskedForSomethingElse) {
  Registry<ICapacityPlanner> registry;
  registry.add("watershed", [] { return std::make_unique<StubPlanner>(); });

  try {
    static_cast<void>(registry.make("greedy"));
    FAIL() << "an unknown name has to be refused";
  } catch (const std::invalid_argument& error) {
    EXPECT_NE(std::string(error.what()).find("watershed"), std::string::npos);
  }
}

TEST(ShippedRegistries, ShipOneImplementationPerStage) {
  // One entry each is expected rather than a smell: the second one becomes a
  // new file rather than a refactor.
  EXPECT_EQ(capacityPlanners().names(),
            (std::vector<std::string>{"watershed"}));
}

TEST(ShippedRegistries, FallBackToTheDefaultWhenAConfigurationSelectsNothing) {
  const ConfigT bare;
  EXPECT_EQ(selectedCapacityPlanner(bare), "watershed");

  ConfigT empty;
  empty.stages = std::make_unique<flatbuffers::config::StageParamsT>();
  empty.stages->capacity =
      std::make_unique<flatbuffers::config::CapacityParamsT>();
  EXPECT_EQ(selectedCapacityPlanner(empty), "watershed");
}

TEST(ShippedRegistries, HonorAConfiguredName) {
  // The selection returns the configured name as it is; whether the name is
  // registered is what building the stage checks.
  ConfigT config;
  config.stages = std::make_unique<flatbuffers::config::StageParamsT>();
  config.stages->capacity =
      std::make_unique<flatbuffers::config::CapacityParamsT>();
  config.stages->capacity->planner = "other-planner";
  EXPECT_EQ(selectedCapacityPlanner(config), "other-planner");
}

/// The planning steps of a run that takes them all. A function rather than a
/// constant, so that building the list cannot throw before main().
std::vector<std::string> allSteps() { return {"capacity"}; }

TEST(PlanProblems, FindNothingWrongWithTheFixture) {
  EXPECT_TRUE(
      planProblems(test::miniChip(), test::miniConfig(), allSteps()).empty());
}

TEST(PlanProblems, NameAnImplementationThatIsNotRegistered) {
  auto config = test::miniConfig();
  config.stages->capacity->planner = "other-planner";

  EXPECT_EQ(planProblems(test::miniChip(), config, allSteps()),
            std::vector<std::string>{
                "[stages.capacity] planner 'other-planner' is not a known "
                "implementation; the registered ones are watershed"});
  // A run that does not take the capacity step does not build its planner.
  EXPECT_TRUE(planProblems(test::miniChip(), config, {}).empty());
}

TEST(PlanProblems, RefuseADetailGridAboveTheLimit) {
  // 100 capacity cells times 60 detail cells per axis make 6000 × 6000
  // detail cells.
  auto config = test::miniConfig();
  config.grid->capacity_cells_x = 100;
  config.grid->capacity_cells_y = 100;
  config.grid->detail_factor = 60;

  EXPECT_EQ(planProblems(test::miniChip(), config, allSteps()),
            std::vector<std::string>{
                "[grid] asks for a detail grid of 6000 × 6000 = 36000000 "
                "cells, a capacity grid of 100 × 100 cells times "
                "detail_factor 60; the limit is 25000000 cells"});
  // A run that does not take the capacity step builds no grid.
  EXPECT_TRUE(planProblems(test::miniChip(), config, {}).empty());
}

TEST(PlanProblems, AcceptADetailGridAtTheLimit) {
  auto config = test::miniConfig();
  config.grid->capacity_cells_x = 100;
  config.grid->capacity_cells_y = 100;
  config.grid->detail_factor = 50;
  ASSERT_EQ(100U * 50U * 100U * 50U, MAX_DETAIL_CELLS);

  EXPECT_TRUE(planProblems(test::miniChip(), config, allSteps()).empty());
}

TEST(PlanProblems, CountTheDetailCellsAlongYFromTheAspectRatio) {
  // The check counts the capacity cells along y as the scene does.
  auto config = test::miniConfig();
  config.grid->capacity_cells_x = 600;
  config.grid->detail_factor = 1;
  const auto chip = test::miniChip();
  ASSERT_EQ(buildScene(chip, config).capacity.height, 508U);

  config.grid->detail_factor = 10;
  EXPECT_EQ(planProblems(chip, config, {"capacity"}),
            std::vector<std::string>{
                "[grid] asks for a detail grid of 6000 × 5080 = 30480000 "
                "cells, a capacity grid of 600 × 508 cells times "
                "detail_factor 10; the limit is 25000000 cells"});
}

TEST(PlanProblems, NameEveryRoutablePortThatFacesNoStepOfTheGrid) {
  auto chip = test::miniChip();
  for (auto& port : chip.ports) {
    if (port->label == "Q1.port0") {
      port->orientation = 10.0;
    } else if (port->label == "Q2.port1") {
      port->orientation = -100.0;
    } else if (port->label == "Chip.port0") {
      // A launcher has no band, so its orientation does not count.
      port->orientation = 30.0;
    }
  }

  EXPECT_EQ(planProblems(chip, test::miniConfig(), allSteps()),
            std::vector<std::string>{
                "the orientation of a routable port has to be a multiple of "
                "45°, so that its approach runs along an axis or a diagonal "
                "of the grid; Q1.port0 faces 10°, Q2.port1 faces -100°"});
  // A run that does not take the capacity step stamps no band.
  EXPECT_TRUE(planProblems(chip, test::miniConfig(), {}).empty());
}

TEST(PlanProblems, RefuseADetailGridBeyondTheRangeOfAnAxis) {
  // 2^32 detail cells along x do not fit the 32-bit cell coordinate.
  auto config = test::miniConfig();
  config.grid->capacity_cells_x = 65536;
  config.grid->capacity_cells_y = 2;
  config.grid->detail_factor = 65536;

  const auto problems = planProblems(test::miniChip(), config, {"capacity"});
  ASSERT_EQ(problems.size(), 1U);
  EXPECT_TRUE(problems[0].starts_with(
      "[grid] asks for a detail grid of 4294967296 × 131072 = "))
      << problems[0];
}

} // namespace
} // namespace mqt::scpd::pipeline
