/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The solve of a stage: which backend the run asks for, and what the solve
// reports.

#include "mqt-scpd/flatbuffers/config.hpp"
#include "mqt-scpd/milp/Backend.hpp"
#include "mqt-scpd/milp/Model.hpp"
#include "mqt-scpd/pipeline/Report.hpp"
#include "mqt-scpd/pipeline/Solver.hpp"
#include "mqt-scpd/pipeline/Stages.hpp"

#include <gtest/gtest.h>

#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace mqt::scpd::pipeline {
namespace {

/// Two binaries that may not both be one, maximized: the optimum is one.
milp::Model pairModel() {
  milp::Model model("pair");
  const auto first = model.addBinary("first", 1.0);
  const auto second = model.addBinary("second", 1.0);
  model.addLessOrEqual("at_most_one", milp::LinearExpr(first) + second, 1.0);
  model.setSense(milp::Sense::Maximize);
  return model;
}

/// A configuration whose solver section names a backend.
ConfigT configWithBackend(const std::string& backend) {
  ConfigT config;
  config.stages = std::make_unique<flatbuffers::config::StageParamsT>();
  config.stages->solver =
      std::make_unique<flatbuffers::config::SolverParamsT>();
  config.stages->solver->backend = backend;
  return config;
}

/// Sets an environment variable while it lives, and clears it after.
class ScopedVariable {
public:
  ScopedVariable(const char* name, const char* value) : variable(name) {
#ifdef _WIN32
    _putenv_s(variable, value);
#else
    // POSIX declares setenv in <stdlib.h>, which <cstdlib> includes.
    setenv(variable, value, 1); // NOLINT(misc-include-cleaner)
#endif
  }
  ScopedVariable(const ScopedVariable&) = delete;
  ScopedVariable& operator=(const ScopedVariable&) = delete;
  ScopedVariable(ScopedVariable&&) = delete;
  ScopedVariable& operator=(ScopedVariable&&) = delete;
  ~ScopedVariable() {
#ifdef _WIN32
    _putenv_s(variable, "");
#else
    unsetenv(variable); // NOLINT(misc-include-cleaner)
#endif
  }

private:
  const char* variable;
};

TEST(SolveWith, SolvesWithTheConfiguredBackend) {
  const auto solution =
      solveWith(pairModel(), configWithBackend("highs"), {}, "solving");

  EXPECT_EQ(solution.status, milp::SolveStatus::Optimal);
  EXPECT_EQ(solution.backend, "highs");
  EXPECT_DOUBLE_EQ(solution.objective, 1.0);
}

TEST(SolveWith, RefusesABackendItDoesNotKnow) {
  EXPECT_THROW(static_cast<void>(solveWith(
                   pairModel(), configWithBackend("cplex"), {}, "solving")),
               std::invalid_argument);
}

TEST(SolveWith, LetsTheEnvironmentOverrideTheConfiguration) {
  // No external backend is registered in this process, so a run that takes
  // the name from the environment has to fail where the configured one solves.
  const ScopedVariable solver("SCPD_SOLVER", "gurobi");
  EXPECT_THROW(static_cast<void>(solveWith(
                   pairModel(), configWithBackend("highs"), {}, "solving")),
               std::runtime_error);
}

TEST(SolveWith, ReportsTheModelAndHowTheSolveEnded) {
  std::vector<std::string> steps;
  const Report report(
      Detail::Steps,
      {.line = [&steps](const Detail detail, const std::string_view text) {
        if (detail == Detail::Steps) {
          steps.emplace_back(text);
        }
      }});

  static_cast<void>(solveWith(pairModel(), ConfigT{}, report, "solving"));

  ASSERT_EQ(steps.size(), 2U);
  EXPECT_EQ(steps[0], "model pair · 2 variables · 1 constraint");
  EXPECT_TRUE(steps[1].starts_with("highs optimal · objective 1.00 · "))
      << steps[1];
}

TEST(SolveWith, SaysNothingAtTheSummaryLevel) {
  std::vector<std::string> lines;
  const Report report(
      Detail::Summary,
      {.line = [&lines](const Detail /*detail*/, const std::string_view text) {
        lines.emplace_back(text);
      }});

  static_cast<void>(solveWith(pairModel(), ConfigT{}, report, "solving"));

  EXPECT_TRUE(lines.empty());
}

TEST(SolverName, NamesHighsWithItsVersion) {
  const auto highs = "HiGHS " + milp::highsVersion();
  EXPECT_EQ(solverName(configWithBackend("highs")), highs);
  // Without an external backend in the process, auto is HiGHS.
  EXPECT_EQ(solverName(ConfigT{}), highs);
}

TEST(SolverName, FollowsTheEnvironmentLikeTheSolve) {
  const ScopedVariable solver("SCPD_SOLVER", "gurobi");
  EXPECT_EQ(solverName(configWithBackend("highs")), "gurobi");
}

} // namespace
} // namespace mqt::scpd::pipeline
