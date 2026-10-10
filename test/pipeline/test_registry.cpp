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

#include "mqt-scpd/flatbuffers/config.hpp"
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
  EXPECT_EQ(globalRouters().names(), (std::vector<std::string>{"hanan-milp"}));
  EXPECT_EQ(assigners().names(), (std::vector<std::string>{"ordered-milp"}));
  EXPECT_EQ(corridorRouters().names(),
            (std::vector<std::string>{"partition-astar"}));
}

TEST(ShippedRegistries, FallBackToTheDefaultWhenAConfigurationSelectsNothing) {
  const ConfigT bare;
  EXPECT_EQ(selectedCapacityPlanner(bare), "watershed");
  EXPECT_EQ(selectedGlobalRouter(bare), "hanan-milp");
  EXPECT_EQ(selectedAssigner(bare), "ordered-milp");
  EXPECT_EQ(selectedCorridorRouter(bare), "partition-astar");

  ConfigT empty;
  empty.stages = std::make_unique<flatbuffers::config::StageParamsT>();
  empty.stages->capacity =
      std::make_unique<flatbuffers::config::CapacityParamsT>();
  empty.stages->global = std::make_unique<flatbuffers::config::GlobalParamsT>();
  empty.stages->assignment =
      std::make_unique<flatbuffers::config::AssignmentParamsT>();
  empty.stages->corridor =
      std::make_unique<flatbuffers::config::CorridorParamsT>();
  EXPECT_EQ(selectedCapacityPlanner(empty), "watershed");
  EXPECT_EQ(selectedGlobalRouter(empty), "hanan-milp");
  EXPECT_EQ(selectedAssigner(empty), "ordered-milp");
  EXPECT_EQ(selectedCorridorRouter(empty), "partition-astar");
}

TEST(ShippedRegistries, HonorAConfiguredName) {
  // The selection returns the configured name as it is; whether the name is
  // registered is what building the stage checks.
  ConfigT config;
  config.stages = std::make_unique<flatbuffers::config::StageParamsT>();
  config.stages->capacity =
      std::make_unique<flatbuffers::config::CapacityParamsT>();
  config.stages->capacity->planner = "other-planner";
  config.stages->global =
      std::make_unique<flatbuffers::config::GlobalParamsT>();
  config.stages->global->router = "other-router";
  config.stages->assignment =
      std::make_unique<flatbuffers::config::AssignmentParamsT>();
  config.stages->assignment->assigner = "other-assigner";
  config.stages->corridor =
      std::make_unique<flatbuffers::config::CorridorParamsT>();
  config.stages->corridor->router = "other-corridors";
  EXPECT_EQ(selectedCapacityPlanner(config), "other-planner");
  EXPECT_EQ(selectedGlobalRouter(config), "other-router");
  EXPECT_EQ(selectedAssigner(config), "other-assigner");
  EXPECT_EQ(selectedCorridorRouter(config), "other-corridors");
}

} // namespace
} // namespace mqt::scpd::pipeline
