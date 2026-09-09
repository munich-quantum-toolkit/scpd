/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The MPS text is the whole contract of the bring-your-own-licence path: the
// external solver never sees the model, only this. Every test here reads the
// emitted text back with a solver and asks whether it describes the same
// problem the in-process backend was handed.

#include "Highs.h"
#include "mqt-scpd/milp/Backend.hpp"
#include "mqt-scpd/milp/Model.hpp"
#include "mqt-scpd/milp/Mps.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <cstddef>
#include <format>
#include <filesystem>
#include <fstream>
#include <string>

namespace mqt::scpd::milp {
namespace {

/// The MPS text of a model, solved by reading it back as a file.
///
/// Going through a file rather than a string is the point: it is the same
/// path `gurobipy` takes, so a syntax the reader would reject shows up here.
Solution solveThroughMps(const Model& model) {
  // A counter rather than the process id, so the name is the same on every
  // platform the tests run on.
  static std::atomic<unsigned> counter{0};
  const auto path =
      std::filesystem::temp_directory_path() /
      std::format("mqt-scpd-milp-{}-{}.mps", model.name(), counter.fetch_add(1));
  {
    std::ofstream file(path);
    file << toMps(model);
  }

  Highs highs;
  static_cast<void>(highs.setOptionValue("output_flag", false));
  Solution solution;
  solution.backend = "mps";
  if (highs.readModel(path.string()) != HighsStatus::kOk) {
    solution.message = "the emitted MPS text was rejected";
    std::filesystem::remove(path);
    return solution;
  }
  std::filesystem::remove(path);

  if (highs.run() == HighsStatus::kError) {
    solution.message = "the model read back from MPS failed to solve";
    return solution;
  }
  const auto status = highs.getModelStatus();
  if (status == HighsModelStatus::kInfeasible) {
    solution.status = SolveStatus::Infeasible;
    return solution;
  }
  if (status != HighsModelStatus::kOptimal) {
    solution.message = "the model read back from MPS did not solve to optimality";
    return solution;
  }

  solution.status = SolveStatus::Optimal;
  // MPS carries a minimization, so a maximization comes back negated.
  const auto sign = model.sense() == Sense::Maximize ? -1.0 : 1.0;
  solution.objective = sign * highs.getInfo().objective_function_value;

  // The round trip only preserves names, so the values are matched back by
  // name rather than by position.
  const auto& lp = highs.getLp();
  solution.values.assign(model.variableCount(), 0.0);
  for (std::size_t index = 0; index < model.variableCount(); ++index) {
    const auto& wanted = model.variables()[index].name;
    for (std::size_t column = 0; column < lp.col_names_.size(); ++column) {
      if (lp.col_names_[column] == wanted) {
        solution.values[index] = highs.getSolution().col_value[column];
        break;
      }
    }
  }
  return solution;
}

/// Assert that both paths agree on a model's optimum.
void expectAgreement(const Model& model) {
  const auto direct = makeHighsBackend()->solve(model, {});
  const auto roundTrip = solveThroughMps(model);
  ASSERT_EQ(direct.status, roundTrip.status) << roundTrip.message;
  if (direct.status != SolveStatus::Optimal) {
    return;
  }
  EXPECT_NEAR(direct.objective, roundTrip.objective, 1.0e-6);
  for (std::size_t index = 0; index < model.variableCount(); ++index) {
    EXPECT_NEAR(direct.values[index], roundTrip.values[index], 1.0e-6)
        << "variable " << model.variables()[index].name;
  }
}

TEST(Mps, CarriesABinaryProgram) {
  Model model("binary");
  const auto a = model.addBinary("a", 3.0);
  const auto b = model.addBinary("b", 5.0);
  const auto c = model.addBinary("c", 4.0);
  model.setSense(Sense::Maximize);
  model.addLessOrEqual("pick_two", (1.0 * a) + (1.0 * b) + (1.0 * c), 2.0);
  expectAgreement(model);
}

TEST(Mps, CarriesIntegerBoundsThatAreNotZeroToOne) {
  // The MPS integer marker implies a bound of one on a reader that is given
  // no bound, so an integer variable with a wider range has to state it.
  Model model("integer");
  const auto x = model.addInteger("x", 2.0, 9.0, 1.0);
  const auto y = model.addInteger("y", -4.0, 4.0, 1.0);
  model.setSense(Sense::Maximize);
  model.addLessOrEqual("cap", (1.0 * x) + (1.0 * y), 11.0);
  expectAgreement(model);
}

TEST(Mps, CarriesAnUnboundedIntegerVariable) {
  Model model("plus");
  const auto x = model.addInteger("x", 0.0, INFINITE_BOUND, 1.0);
  model.addLessOrEqual("cap", 1.0 * x, 7.0);
  model.setSense(Sense::Maximize);
  expectAgreement(model);
}

TEST(Mps, CarriesAFreeContinuousVariable) {
  Model model("free");
  const auto x = model.addContinuous("x", -INFINITE_BOUND, INFINITE_BOUND, 1.0);
  model.addGreaterOrEqual("floor", 1.0 * x, -3.0);
  expectAgreement(model);
}

TEST(Mps, CarriesARangeRow) {
  Model model("range");
  const auto x = model.addContinuous("x", 0.0, 5.0, 1.0);
  const auto y = model.addContinuous("y", 0.0, 5.0, 1.0);
  model.setSense(Sense::Maximize);
  model.addRange("band", (1.0 * x) + (1.0 * y), 2.0, 3.0);
  expectAgreement(model);
}

TEST(Mps, CarriesTheObjectiveOffsetInBothSenses) {
  for (const auto sense : {Sense::Minimize, Sense::Maximize}) {
    Model model("offset");
    static_cast<void>(model.addContinuous("x", 1.0, 1.0, 2.0));
    model.addObjective(10.0);
    model.setSense(sense);
    expectAgreement(model);
  }
}

TEST(Mps, CarriesAVariableThatAppearsInNoRowAndNoObjective) {
  Model model("orphan");
  const auto used = model.addBinary("used", 1.0);
  static_cast<void>(model.addInteger("spare", 0.0, 3.0));
  model.addLessOrEqual("cap", 1.0 * used, 1.0);
  model.setSense(Sense::Maximize);

  // The spare variable has to survive the trip, or a solution read back by
  // name would be one value short.
  const auto text = toMps(model);
  EXPECT_NE(text.find("spare"), std::string::npos);
  expectAgreement(model);
}

TEST(Mps, CarriesAnInfeasibleModelAsInfeasible) {
  Model model("infeasible");
  const auto x = model.addBinary("x");
  model.addGreaterOrEqual("high", 1.0 * x, 1.0);
  model.addLessOrEqual("low", 1.0 * x, 0.0);
  expectAgreement(model);
}

TEST(Mps, NamesEveryRowAndColumnOfTheModel) {
  Model model("names");
  const auto x = model.addBinary("alpha");
  model.addEqual("beta", 1.0 * x, 1.0);

  const auto text = toMps(model);
  EXPECT_NE(text.find("NAME          names"), std::string::npos);
  EXPECT_NE(text.find(" E  beta"), std::string::npos);
  EXPECT_NE(text.find("alpha"), std::string::npos);
  EXPECT_NE(text.find("ENDATA"), std::string::npos);
}

} // namespace
} // namespace mqt::scpd::milp
