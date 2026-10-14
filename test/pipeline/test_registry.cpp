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
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/pipeline/Registry.hpp"
#include "mqt-scpd/pipeline/Report.hpp"
#include "mqt-scpd/pipeline/Stages.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
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

/// The planning steps of a run that takes them all. A function rather than a
/// constant, so that building the list cannot throw before main().
std::vector<std::string> allSteps() {
  return {"capacity", "global", "assign", "corridor"};
}

/// Whether one of the problems starts with a given text.
bool mentions(const std::vector<std::string>& problems,
              const std::string_view start) {
  return std::ranges::any_of(problems, [&](const std::string& problem) {
    return problem.starts_with(start);
  });
}

TEST(PlanProblems, FindNothingWrongWithTheFixture) {
  EXPECT_TRUE(
      planProblems(test::miniChip(), test::miniConfig(), allSteps()).empty());
}

TEST(PlanProblems, NameAnImplementationThatIsNotRegistered) {
  auto config = test::miniConfig();
  config.stages->global->router = "other-router";
  config.stages->corridor->router = "other-corridors";

  const auto problems = planProblems(test::miniChip(), config, allSteps());
  EXPECT_EQ(problems.size(), 2U);
  EXPECT_TRUE(mentions(problems, "[stages.global] router 'other-router' is no "
                                 "known implementation; the registered ones "
                                 "are hanan-milp"));
  EXPECT_TRUE(mentions(problems, "[stages.corridor] router 'other-corridors'"));
  // A run that ends before the corridor does not build its router.
  EXPECT_EQ(
      planProblems(test::miniChip(), config, {"capacity", "global"}).size(),
      1U);
}

TEST(PlanProblems, AskForTheLauncherTargetOnlyWhenTheRunAssigns) {
  auto config = test::miniConfig();
  config.stages->assignment->launcher_target = 0;

  EXPECT_TRUE(mentions(planProblems(test::miniChip(), config, allSteps()),
                       "[stages.assignment] launcher_target is missing"));
  EXPECT_TRUE(
      planProblems(test::miniChip(), config, {"capacity", "global"}).empty());
}

TEST(PlanProblems, RefuseAnOddNumberOfChainEnds) {
  // One launcher end and no termination leave one chain with one end.
  auto config = test::miniConfig();
  config.rules->feedline_terminations = 0;

  const auto problems = planProblems(test::miniChip(), config, allSteps());
  ASSERT_EQ(problems.size(), 1U);
  EXPECT_EQ(problems[0],
            "[stages.assignment] launcher_target 1 plus [design_rules] "
            "feedline_terminations 0 is odd; every feedline chain has two "
            "ends, so the sum has to be even");
}

TEST(PlanProblems, RefuseLaunchersListedWithTheRing) {
  // The fixture lists its launchers clockwise against a counterclockwise
  // ring. The reversed ring runs with them.
  auto config = test::miniConfig();
  auto& ring = config.ports->sequences->all_outer;
  std::ranges::reverse(ring);

  EXPECT_TRUE(mentions(planProblems(test::miniChip(config), config, allSteps()),
                       "the chip input lists the launcher ports in the "
                       "direction of the port ring"));
}

TEST(PlanProblems, RefuseTooFewLaunchersForTheResonatorsWhenTheRunAssigns) {
  // Two of the four launchers of the fixture become ports of no role.
  auto chip = test::miniChip();
  for (auto& port : chip.ports) {
    if (port->label == "Chip.port2" || port->label == "Chip.port3") {
      port->role = flatbuffers::design::UnassignedRole::Unset;
    }
  }

  EXPECT_TRUE(mentions(planProblems(chip, test::miniConfig(), allSteps()),
                       "the chip has 2 launcher ports; feeding resonators"));
  EXPECT_TRUE(
      planProblems(chip, test::miniConfig(), {"capacity", "global"}).empty());
}

TEST(SolvingSteps, NameTheAssignmentAndNotAGlobalStepWithoutAnInnerCircuit) {
  // The ring of the fixture carries every routable port.
  const auto config = test::miniConfig();

  EXPECT_EQ(solvingSteps(test::miniChip(config), config, allSteps()),
            std::vector<std::string>{"assign"});
  EXPECT_TRUE(
      solvingSteps(test::miniChip(config), config, {"capacity", "global"})
          .empty());
}

TEST(SolvingSteps, NameAGlobalStepWithAnInnerCircuit) {
  // Q2.port1 leaves the ring, and the global step has to reach it.
  auto config = test::miniConfig();
  std::erase(config.ports->sequences->all_outer, "Q2.port1");

  EXPECT_EQ(solvingSteps(test::miniChip(config), config, allSteps()),
            (std::vector<std::string>{"global", "assign"}));
}

TEST(SolvingSteps, NameAGlobalStepThatRefusesItsRing) {
  // A ring label that names no port stops the global step, which keeps it in
  // the list.
  auto config = test::miniConfig();
  config.ports->sequences->all_outer.emplace_back("Nowhere.port0");

  EXPECT_EQ(solvingSteps(test::miniChip(), config, {"capacity", "global"}),
            std::vector<std::string>{"global"});
}

} // namespace
} // namespace mqt::scpd::pipeline
