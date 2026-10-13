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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace mqt::scpd::milp {
namespace {

TEST(HighsBackend, NamesTheVersionItLinks) {
  const auto version = highsVersion();
  // Three whole numbers joined by dots, such as 1.11.0.
  ASSERT_EQ(std::ranges::count(version, '.'), 2) << version;
  EXPECT_TRUE(std::ranges::all_of(version, [](const char letter) {
    return letter == '.' || (letter >= '0' && letter <= '9');
  })) << version;
}

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

TEST(HighsBackend, DoesNotGuessBetweenInfeasibleAndUnbounded) {
  // Two integers without an upper bound, which the objective pulls up. The
  // model is unbounded, but HiGHS reports only that it has no optimum, so
  // the backend reports that too rather than a guess.
  Model model("either");
  const auto x = model.addInteger("x", 0.0, INFINITE_BOUND, -1.0);
  const auto y = model.addInteger("y", 0.0, INFINITE_BOUND, -1.0);
  model.addGreaterOrEqual("tie", (1.0 * x) - (1.0 * y), 0.0);

  const auto solution = makeHighsBackend()->solve(model, {});
  EXPECT_EQ(solution.status, SolveStatus::Error);
  EXPECT_NE(solution.message.find("infeasible or unbounded"),
            std::string::npos);
  EXPECT_FALSE(solution.hasValues());
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

/// A 0-1 knapsack that presolve cannot settle, so the solver has to search.
/// The weights and values come from a fixed linear congruential sequence.
Model hardKnapsack() {
  Model model("hard-knapsack");
  LinearExpr weight;
  std::uint64_t state = 12345;
  const auto next = [&state] {
    state = (state * 6364136223846793005ULL) + 1442695040888963407ULL;
    return static_cast<double>((state >> 33U) % 1000U) + 1.0;
  };
  double total = 0.0;
  for (std::size_t item = 0; item < 40; ++item) {
    const auto w = next();
    const auto v = w + (next() / 10.0);
    weight.add(model.addBinary(std::format("x{}", item), v), w);
    total += w;
  }
  model.setSense(Sense::Maximize);
  model.addLessOrEqual("capacity", weight, std::floor(total / 2.0) + 0.5);
  return model;
}

TEST(HighsBackend, ReportsTheBoundTheGapAndTheNodesOfAMixedIntegerSolve) {
  Model model("knapsack");
  const auto x = model.addInteger("x", 0.0, 3.0, 1.0);
  const auto y = model.addInteger("y", 0.0, 3.0, 2.0);
  model.setSense(Sense::Maximize);
  model.addLessOrEqual("capacity", (1.0 * x) + (1.0 * y), 4.0);

  const auto solution = makeHighsBackend()->solve(model, {});

  ASSERT_EQ(solution.status, SolveStatus::Optimal);
  EXPECT_NEAR(solution.bound, 7.0, 1.0e-6);
  EXPECT_NEAR(solution.gap, 0.0, 1.0e-9);
}

TEST(HighsBackend, LeavesTheBoundOfALinearModelOpen) {
  Model model("linear");
  static_cast<void>(model.addContinuous("x", 1.0, 2.0, 1.0));

  const auto solution = makeHighsBackend()->solve(model, {});

  ASSERT_EQ(solution.status, SolveStatus::Optimal);
  EXPECT_TRUE(std::isnan(solution.bound));
  EXPECT_TRUE(std::isnan(solution.gap));
  EXPECT_EQ(solution.nodes, 0U);
}

TEST(HighsBackend, WritesItsLogOnlyToAnObserverThatAsksForIt) {
  const auto model = hardKnapsack();
  testing::internal::CaptureStdout();
  const auto quiet = makeHighsBackend()->solve(model, {});
  std::vector<std::string> lines;
  SolveOptions options;
  options.observer.log = [&lines](const std::string_view line) {
    lines.emplace_back(line);
  };
  const auto talkative = makeHighsBackend()->solve(model, options);
  const auto printed = testing::internal::GetCapturedStdout();

  ASSERT_EQ(quiet.status, SolveStatus::Optimal);
  ASSERT_EQ(talkative.status, SolveStatus::Optimal);
  EXPECT_DOUBLE_EQ(quiet.objective, talkative.objective);
  EXPECT_FALSE(lines.empty());
  for (const auto& line : lines) {
    EXPECT_EQ(line.find('\n'), std::string::npos) << line;
  }
  // The core never writes to the process output, whether or not it logs.
  EXPECT_EQ(printed, "");
}

TEST(HighsBackend, ReportsTheProgressOfASearch) {
  const auto model = hardKnapsack();
  std::vector<SolveProgress> reports;
  SolveOptions options;
  options.observer.progress = [&reports](const SolveProgress& progress) {
    reports.push_back(progress);
  };

  const auto solution = makeHighsBackend()->solve(model, options);

  ASSERT_EQ(solution.status, SolveStatus::Optimal);
  ASSERT_FALSE(reports.empty());
  for (const auto& report : reports) {
    EXPECT_GE(report.seconds, 0.0);
    // Of a maximization, every solution found lies at or below the optimum
    // and every bound proved at or above it.
    if (!std::isnan(report.objective)) {
      EXPECT_LE(report.objective, solution.objective + 1.0e-6);
    }
    if (!std::isnan(report.bound)) {
      EXPECT_GE(report.bound, solution.objective - 1.0e-6);
    }
  }
}

TEST(HighsBackend, SolvesToTheSameOptimumUnderALimitAndAGap) {
  const auto model = hardKnapsack();
  const auto free = makeHighsBackend()->solve(model, {});
  SolveOptions options;
  options.timeLimit = 600.0;
  options.relativeGap = 1.0e-9;

  const auto limited = makeHighsBackend()->solve(model, options);

  ASSERT_EQ(free.status, SolveStatus::Optimal);
  ASSERT_EQ(limited.status, SolveStatus::Optimal);
  EXPECT_DOUBLE_EQ(limited.objective, free.objective);
}

TEST(HighsBackend, GivesValuesToEverySolveThatALimitStops) {
  // A limit that is over before the search starts may leave no solution.
  // Such a solve is an error, and a solve that a limit stops is feasible only
  // with values.
  const auto model = hardKnapsack();
  SolveOptions options;
  options.timeLimit = 1.0e-9;

  const auto solution = makeHighsBackend()->solve(model, options);

  if (solution.status == SolveStatus::Feasible) {
    EXPECT_TRUE(solution.hasValues());
  } else if (solution.status == SolveStatus::Error) {
    EXPECT_FALSE(solution.message.empty());
  }
}

/// A covering program too large to solve within a millisecond: each of 3000
/// rows asks that 30 of 3000 variables in [0, 1] add up to at least 10.
Model largeCover() {
  constexpr std::size_t size = 3000;
  Model model("large-cover");
  std::vector<Var> variables;
  variables.reserve(size);
  for (std::size_t column = 0; column < size; ++column) {
    variables.push_back(
        model.addContinuous(std::format("x{}", column), 0.0, 1.0, 1.0));
  }
  std::uint64_t state = 1;
  const auto next = [&state] {
    state = (state * 6364136223846793005ULL) + 1442695040888963407ULL;
    return state >> 33U;
  };
  for (std::size_t row = 0; row < size; ++row) {
    LinearExpr cover;
    for (std::size_t term = 0; term < 30; ++term) {
      const auto column = static_cast<std::size_t>(next() % size);
      const auto coefficient =
          0.1 + (0.9 * static_cast<double>(next() % 1000U) / 1000.0);
      cover.add(variables[column], coefficient);
    }
    model.addGreaterOrEqual(std::format("r{}", row), cover, 10.0);
  }
  return model;
}

TEST(HighsBackend, ReportsAFeasibleSolveOnlyWithValuesThatHoldEveryRow) {
  // A time limit can stop a linear solve before it reaches a primal feasible
  // point, while HiGHS still hands back an array of values. Those values are
  // no solution, and the backend must not report them as one.
  const auto model = largeCover();
  SolveOptions options;
  options.timeLimit = 1.0e-3;

  const auto solution = makeHighsBackend()->solve(model, options);

  if (solution.status == SolveStatus::Feasible ||
      solution.status == SolveStatus::Optimal) {
    ASSERT_EQ(solution.values.size(), model.variableCount());
    std::size_t violated = 0;
    for (const auto& constraint : model.constraints()) {
      double activity = 0.0;
      for (const auto& [column, coefficient] : constraint.coefficients) {
        activity += coefficient * solution.values[column];
      }
      violated += activity < constraint.lower - 1.0e-6 ? 1U : 0U;
    }
    EXPECT_EQ(violated, 0U) << "of " << model.constraintCount() << " rows";
  } else {
    EXPECT_EQ(solution.status, SolveStatus::Error);
    EXPECT_FALSE(solution.message.empty());
  }
}

TEST(HighsBackend, SolvesWithAnotherThreadCountEachTime) {
  // HiGHS keeps one task scheduler per calling thread. A solve that asks for
  // a thread count other than the one of the solve before still solves.
  const auto model = hardKnapsack();
  for (const std::uint32_t threads : {1U, 2U, 0U, 1U}) {
    SolveOptions options;
    options.threads = threads;

    const auto solution = makeHighsBackend()->solve(model, options);

    EXPECT_EQ(solution.status, SolveStatus::Optimal)
        << "threads " << threads << ": " << solution.message;
  }
}

TEST(HighsBackend, EndsTheSolveWhenTheObserverThrowsAndRethrows) {
  // A Python KeyboardInterrupt reaches the core as an exception from a
  // callback. The solve stops at the next chance and the exception leaves the
  // backend after HiGHS has returned, rather than through it.
  const auto model = hardKnapsack();
  SolveOptions options;
  options.observer.progress = [](const SolveProgress&) {
    throw std::runtime_error("stopped by the observer");
  };

  EXPECT_THROW(static_cast<void>(makeHighsBackend()->solve(model, options)),
               std::runtime_error);
}

TEST(SolverChoice, ReadsTheTwoNamesAndRefusesTheRest) {
  EXPECT_EQ(backendChoiceFromName("highs"), BackendChoice::Highs);
  EXPECT_EQ(backendChoiceFromName("gurobi"), BackendChoice::External);
  EXPECT_THROW(static_cast<void>(backendChoiceFromName("auto")),
               std::invalid_argument);
  EXPECT_THROW(static_cast<void>(backendChoiceFromName("cplex")),
               std::invalid_argument);
}

TEST(SolverChoice, RefusesTheExternalBackendWhenNoneIsRegistered) {
  setExternalBackend(nullptr);
  EXPECT_EQ(resolveBackend(BackendChoice::Highs)->name(), "highs");
  EXPECT_THROW(static_cast<void>(resolveBackend(BackendChoice::External)),
               std::runtime_error);
}

TEST(SolverChoice, UsesARegisteredExternalBackendOnlyWhenAskedFor) {
  // A backend that answers without solving anything is enough to show which
  // one the choice picked.
  auto stub = makeMpsBackend(
      "stub", [](auto, const auto& names, const auto& /*options*/) {
        Solution solution;
        solution.status = SolveStatus::Optimal;
        solution.objective = 42.0;
        solution.values.assign(names.size(), 0.0);
        return solution;
      });
  setExternalBackend(std::move(stub));

  EXPECT_EQ(resolveBackend(BackendChoice::External)->name(), "stub");
  // HiGHS is never replaced by what is registered.
  EXPECT_EQ(resolveBackend(BackendChoice::Highs)->name(), "highs");

  setExternalBackend(nullptr);
}

TEST(MpsBackend, RefusesAWrongNumberOfValues) {
  Model model("mismatch");
  static_cast<void>(model.addBinary("x"));
  static_cast<void>(model.addBinary("y"));

  const auto backend =
      makeMpsBackend("short", [](auto, const auto&, const auto& /*options*/) {
        Solution solution;
        solution.status = SolveStatus::Optimal;
        solution.values = {1.0};
        return solution;
      });
  const auto solution = backend->solve(model, {});
  EXPECT_EQ(solution.status, SolveStatus::Error);
  EXPECT_NE(solution.message.find("2 variables"), std::string::npos);
}

TEST(MpsBackend, NegatesTheObjectiveAndTheBoundOfAMaximization) {
  // MPS states a minimization, so the external solver answers a maximization
  // with the objective and the bound of the negated program.
  Model model("maximum");
  const auto a = model.addBinary("a", 3.0);
  const auto b = model.addBinary("b", 4.0);
  model.setSense(Sense::Maximize);
  model.addLessOrEqual("one", (1.0 * a) + (1.0 * b), 1.0);

  const auto backend = makeMpsBackend(
      "negated", [](auto, const auto& /*names*/, const auto& /*options*/) {
        Solution solution;
        solution.status = SolveStatus::Feasible;
        solution.objective = -4.0;
        solution.bound = -4.5;
        solution.values = {0.0, 1.0};
        return solution;
      });
  const auto solution = backend->solve(model, {});

  ASSERT_EQ(solution.status, SolveStatus::Feasible);
  EXPECT_DOUBLE_EQ(solution.objective, 4.0);
  EXPECT_DOUBLE_EQ(solution.bound, 4.5);
}

TEST(MpsBackend, NeedsASolveFunction) {
  EXPECT_THROW(static_cast<void>(makeMpsBackend("empty", {})),
               std::invalid_argument);
}

} // namespace
} // namespace mqt::scpd::milp
