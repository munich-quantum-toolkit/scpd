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

// <cstdlib> declares setenv, unsetenv and _putenv_s, but include-cleaner
// knows no header for these functions outside the C++ standard.
#include <cstdlib> // NOLINT(misc-include-cleaner)
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
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

/// Sets an environment variable while it lives, and restores the value it had
/// before.
class ScopedVariable {
public:
  ScopedVariable(const char* name, const char* value) : variable(name) {
    if (const auto* before = std::getenv(variable); before != nullptr) {
      previous = before;
    }
    set(value);
  }
  ScopedVariable(const ScopedVariable&) = delete;
  ScopedVariable& operator=(const ScopedVariable&) = delete;
  ScopedVariable(ScopedVariable&&) = delete;
  ScopedVariable& operator=(ScopedVariable&&) = delete;
  ~ScopedVariable() {
    if (previous.has_value()) {
      set(previous->c_str());
      return;
    }
#ifdef _WIN32
    _putenv_s(variable, "");
#else
    unsetenv(variable); // NOLINT(misc-include-cleaner)
#endif
  }

private:
  void set(const char* value) const {
#ifdef _WIN32
    _putenv_s(variable, value);
#else
    setenv(variable, value, 1); // NOLINT(misc-include-cleaner)
#endif
  }

  const char* variable;
  std::optional<std::string> previous;
};

/// Registers an external backend while it lives, and clears the registration
/// again, also when the test fails on the way.
class ScopedBackend {
public:
  explicit ScopedBackend(std::shared_ptr<milp::ISolverBackend> backend) {
    milp::setExternalBackend(std::move(backend));
  }
  ScopedBackend(const ScopedBackend&) = delete;
  ScopedBackend& operator=(const ScopedBackend&) = delete;
  ScopedBackend(ScopedBackend&&) = delete;
  ScopedBackend& operator=(ScopedBackend&&) = delete;
  ~ScopedBackend() { milp::setExternalBackend(nullptr); }
};

TEST(SolveWith, SolvesWithTheConfiguredBackend) {
  const auto solution =
      solveWith(pairModel(), configWithBackend("highs"), {}, "solving");

  EXPECT_EQ(solution.status, milp::SolveStatus::Optimal);
  EXPECT_EQ(solution.backend, "highs");
  EXPECT_DOUBLE_EQ(solution.objective, 1.0);
}

/// The message with which solveWith refuses @p config, or an empty string.
std::string refusalOf(const ConfigT& config) {
  try {
    static_cast<void>(solveWith(pairModel(), config, {}, "solving"));
  } catch (const std::invalid_argument& error) {
    return error.what();
  }
  return {};
}

TEST(SolveWith, RefusesABackendItDoesNotKnowAndSaysWhoNamedIt) {
  EXPECT_EQ(refusalOf(configWithBackend("cplex")),
            "[stages.solver] backend: 'cplex' is no solver backend; use "
            "'highs' or 'gurobi'");
  const ScopedVariable solver("SCPD_SOLVER", "cplex");
  EXPECT_EQ(refusalOf(configWithBackend("highs")),
            "SCPD_SOLVER: 'cplex' is no solver backend; use 'highs' or "
            "'gurobi'");
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

TEST(SolveWith, CountsNodesOnlyWhenTheBackendReportsThem) {
  std::vector<std::string> steps;
  const Report report(
      Detail::Steps,
      {.line = [&steps](const Detail detail, const std::string_view text) {
        if (detail == Detail::Steps) {
          steps.emplace_back(text);
        }
      }});
  // An external backend that reports a solution and no search figures.
  const ScopedBackend backend(milp::makeMpsBackend(
      "stub", [](auto, const auto& names, const auto& /*options*/) {
        milp::Solution solution;
        solution.status = milp::SolveStatus::Optimal;
        solution.objective = 1.0;
        solution.values.assign(names.size(), 0.0);
        return solution;
      }));

  static_cast<void>(
      solveWith(pairModel(), configWithBackend("highs"), report, "solving"));
  static_cast<void>(
      solveWith(pairModel(), configWithBackend("gurobi"), report, "solving"));

  ASSERT_EQ(steps.size(), 4U);
  EXPECT_NE(steps[1].find(" node"), std::string::npos) << steps[1];
  // The pair model is a maximization, so the MPS backend negates the
  // objective that the stub states for the minimization of the MPS text.
  EXPECT_EQ(steps[3], "stub optimal · objective -1.00");
}

TEST(SolveWith, HandsTheConfiguredLimitsToTheBackend) {
  milp::SolveOptions seen;
  const ScopedBackend backend(milp::makeMpsBackend(
      "stub", [&seen](auto, const auto& names, const auto& options) {
        seen = options;
        milp::Solution solution;
        solution.status = milp::SolveStatus::Optimal;
        solution.values.assign(names.size(), 0.0);
        return solution;
      }));
  auto config = configWithBackend("gurobi");
  config.stages->solver->time_limit = 7.0;
  config.stages->solver->relative_gap = 0.01;
  config.stages->solver->threads = 3;

  static_cast<void>(solveWith(pairModel(), config, {}, "solving"));

  EXPECT_DOUBLE_EQ(seen.timeLimit, 7.0);
  EXPECT_DOUBLE_EQ(seen.relativeGap, 0.01);
  EXPECT_EQ(seen.threads, 3U);
}

TEST(SolveWith, ReportsTheSearchFiguresOfAnExternalBackend) {
  std::vector<std::string> steps;
  const Report report(
      Detail::Steps,
      {.line = [&steps](const Detail detail, const std::string_view text) {
        if (detail == Detail::Steps) {
          steps.emplace_back(text);
        }
      }});
  const ScopedBackend backend(milp::makeMpsBackend(
      "stub", [](auto, const auto& names, const auto& /*options*/) {
        milp::Solution solution;
        solution.status = milp::SolveStatus::Feasible;
        solution.objective = -1.0;
        solution.bound = -2.0;
        solution.gap = 0.5;
        solution.nodes = 12;
        solution.values.assign(names.size(), 0.0);
        return solution;
      }));

  static_cast<void>(
      solveWith(pairModel(), configWithBackend("gurobi"), report, "solving"));

  ASSERT_EQ(steps.size(), 2U);
  // The pair model is a maximization, so the MPS backend negates the
  // objective and the bound that the stub states for the minimization.
  EXPECT_EQ(steps[1], "stub feasible · objective 1.00 · bound 2.00 · gap "
                      "50.0 % · 12 nodes");
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
  // A configuration that names no backend uses HiGHS.
  EXPECT_EQ(solverName(ConfigT{}), highs);
}

TEST(SolverName, KeepsHighsWhenAnExternalBackendIsRegistered) {
  // A registered backend is used only when the configuration asks for it, so
  // a licence on the machine does not change the result of a run.
  const ScopedBackend backend(milp::makeMpsBackend(
      "stub", [](auto, const auto& names, const auto& /*options*/) {
        milp::Solution solution;
        solution.status = milp::SolveStatus::Optimal;
        solution.values.assign(names.size(), 0.0);
        return solution;
      }));
  EXPECT_EQ(solverName(ConfigT{}), "HiGHS " + milp::highsVersion());
  EXPECT_EQ(solverName(configWithBackend("gurobi")), "stub");
}

TEST(SolverName, FollowsTheEnvironmentLikeTheSolve) {
  const ScopedVariable solver("SCPD_SOLVER", "gurobi");
  EXPECT_EQ(solverName(configWithBackend("highs")), "gurobi");
}

} // namespace
} // namespace mqt::scpd::pipeline
