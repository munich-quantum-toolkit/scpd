/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The in-process backend, on models small enough that the answer is known by
// hand. HiGHS is linked in, so these run everywhere.

#include "mqt-scpd/milp/Backend.hpp"
#include "mqt-scpd/milp/Model.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>

namespace mqt::scpd::milp {
namespace {

TEST(HighsBackend, SolvesAKnapsackToItsKnownOptimum) {
  // Maximize x + 2y with x + y <= 4 and both integral in [0, 3]: y takes 3,
  // x takes the remaining 1, for 7.
  Model model("knapsack");
  const auto x = model.addInteger("x", 0.0, 3.0, 1.0);
  const auto y = model.addInteger("y", 0.0, 3.0, 2.0);
  model.setSense(Sense::Maximize);
  model.addLessOrEqual("capacity", (1.0 * x) + (1.0 * y), 4.0);

  const auto solution = makeHighsBackend()->solve(model, {});
  ASSERT_EQ(solution.status, SolveStatus::Optimal);
  EXPECT_NEAR(solution.objective, 7.0, 1.0e-9);
  EXPECT_NEAR(solution.valueOf(x), 1.0, 1.0e-6);
  EXPECT_NEAR(solution.valueOf(y), 3.0, 1.0e-6);
  EXPECT_EQ(solution.backend, "highs");
}

TEST(HighsBackend, CarriesTheObjectiveOffset) {
  Model model("offset");
  const auto x = model.addContinuous("x", 1.0, 1.0, 2.0);
  model.addObjective(10.0);
  static_cast<void>(x);

  const auto solution = makeHighsBackend()->solve(model, {});
  ASSERT_EQ(solution.status, SolveStatus::Optimal);
  EXPECT_NEAR(solution.objective, 12.0, 1.0e-9);
}

TEST(HighsBackend, ReportsAnInfeasibleModelRatherThanThrowing) {
  Model model("infeasible");
  const auto x = model.addBinary("x");
  model.addGreaterOrEqual("high", 1.0 * x, 1.0);
  model.addLessOrEqual("low", 1.0 * x, 0.0);

  const auto solution = makeHighsBackend()->solve(model, {});
  EXPECT_EQ(solution.status, SolveStatus::Infeasible);
  EXPECT_FALSE(solution.hasValues());
}

TEST(HighsBackend, ReportsAnUnboundedModel) {
  Model model("unbounded");
  static_cast<void>(model.addContinuous("x", 0.0, INFINITE_BOUND, -1.0));

  const auto solution = makeHighsBackend()->solve(model, {});
  EXPECT_EQ(solution.status, SolveStatus::Unbounded);
}

TEST(HighsBackend, HonorsARangeConstraint) {
  // 2 <= x + y <= 3 with the objective pulling both up: the sum lands on 3.
  Model model("range");
  const auto x = model.addContinuous("x", 0.0, 5.0, 1.0);
  const auto y = model.addContinuous("y", 0.0, 5.0, 1.0);
  model.setSense(Sense::Maximize);
  model.addRange("band", (1.0 * x) + (1.0 * y), 2.0, 3.0);

  const auto solution = makeHighsBackend()->solve(model, {});
  ASSERT_EQ(solution.status, SolveStatus::Optimal);
  EXPECT_NEAR(solution.objective, 3.0, 1.0e-9);
}

TEST(SolverChoice, ReadsTheThreeNamesAndRefusesTheRest) {
  EXPECT_EQ(backendChoiceFromName("auto"), BackendChoice::Auto);
  EXPECT_EQ(backendChoiceFromName("highs"), BackendChoice::Highs);
  EXPECT_EQ(backendChoiceFromName("gurobi"), BackendChoice::External);
  EXPECT_THROW(static_cast<void>(backendChoiceFromName("cplex")), std::invalid_argument);
}

TEST(SolverChoice, FallsBackToHighsWhenNoExternalBackendIsRegistered) {
  setExternalBackend(nullptr);
  EXPECT_EQ(resolveBackend(BackendChoice::Auto)->name(), "highs");
  EXPECT_EQ(resolveBackend(BackendChoice::Highs)->name(), "highs");
  EXPECT_THROW(static_cast<void>(resolveBackend(BackendChoice::External)), std::runtime_error);
}

TEST(SolverChoice, PrefersARegisteredExternalBackendUnderAuto) {
  // A backend that answers without solving anything is enough to show which
  // one the choice picked.
  auto stub = makeMpsBackend("stub", [](auto, const auto& names, auto) {
    Solution solution;
    solution.status = SolveStatus::Optimal;
    solution.objective = 42.0;
    solution.values.assign(names.size(), 0.0);
    return solution;
  });
  setExternalBackend(std::move(stub));

  EXPECT_EQ(resolveBackend(BackendChoice::Auto)->name(), "stub");
  EXPECT_EQ(resolveBackend(BackendChoice::External)->name(), "stub");
  // An explicit choice of HiGHS is never overridden by what is registered.
  EXPECT_EQ(resolveBackend(BackendChoice::Highs)->name(), "highs");

  setExternalBackend(nullptr);
}

TEST(MpsBackend, RefusesAWrongNumberOfValues) {
  Model model("mismatch");
  static_cast<void>(model.addBinary("x"));
  static_cast<void>(model.addBinary("y"));

  const auto backend = makeMpsBackend("short", [](auto, const auto&, auto) {
    Solution solution;
    solution.status = SolveStatus::Optimal;
    solution.values = {1.0};
    return solution;
  });
  const auto solution = backend->solve(model, {});
  EXPECT_EQ(solution.status, SolveStatus::Error);
  EXPECT_NE(solution.message.find("2 variables"), std::string::npos);
}

TEST(MpsBackend, NeedsASolveFunction) {
  EXPECT_THROW(static_cast<void>(makeMpsBackend("empty", {})), std::invalid_argument);
}

} // namespace
} // namespace mqt::scpd::milp
