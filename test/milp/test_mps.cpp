/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The MPS text is the whole contract of the bring-your-own-license path: the
// external solver never sees the model, only this. Every test here reads the
// emitted text back with a solver and asks whether it describes the same
// problem the in-process backend was handed.

#include "../MpsSolver.hpp"
#include "mqt-scpd/milp/Backend.hpp"
#include "mqt-scpd/milp/Model.hpp"
#include "mqt-scpd/milp/Mps.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace mqt::scpd::milp {
namespace {

/// A model solved through the backend that hands MPS text to an external
/// solver.
Solution solveThroughMps(const Model& model) {
  return makeMpsBackend("mps", test::solveMpsText)->solve(model, {});
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

TEST(Mps, CarriesAnIntegerVariableWithOnlyALowerBound) {
  // A reader that gives an integer column the bound of one on each side it
  // finds no bound for would read [2, 1] without the open upper side.
  Model model("lower");
  const auto x = model.addInteger("x", 2.0, INFINITE_BOUND, 1.0);
  const auto y = model.addInteger("y", -INFINITE_BOUND, INFINITE_BOUND, -1.0);
  model.addLessOrEqual("cap", 1.0 * x, 7.0);
  model.addGreaterOrEqual("floor", 1.0 * y, -3.0);
  model.setSense(Sense::Maximize);
  expectAgreement(model);

  const auto text = toMps(model);
  EXPECT_NE(text.find(" LO BND  x  2\n PL BND  x\n"), std::string::npos)
      << text;
}

TEST(Mps, CarriesAnIntegerVariableWithOnlyAnUpperBound) {
  // MI opens the lower side, and UP closes the upper one.
  Model model("upper");
  const auto x = model.addInteger("x", -INFINITE_BOUND, 4.0, 1.0);
  model.addGreaterOrEqual("floor", 1.0 * x, -6.0);
  model.setSense(Sense::Maximize);
  expectAgreement(model);

  const auto text = toMps(model);
  EXPECT_NE(text.find(" MI BND  x\n UP BND  x  4\n"), std::string::npos)
      << text;
}

TEST(Mps, CarriesAContinuousVariableBelowZero) {
  // Some readers take a negative UP on a column without a lower bound as a
  // bound of minus infinity below. The text states the lower bound first,
  // so every reader reads [-5, -2].
  Model model("negative");
  static_cast<void>(model.addContinuous("x", -5.0, -2.0, 1.0));
  static_cast<void>(model.addContinuous("y", -5.0, -2.0, -1.0));
  expectAgreement(model);

  const auto text = toMps(model);
  EXPECT_NE(text.find(" LO BND  x  -5\n UP BND  x  -2\n"), std::string::npos)
      << text;
}

TEST(Mps, CarriesAFreeContinuousVariable) {
  Model model("free");
  const auto x = model.addContinuous("x", -INFINITE_BOUND, INFINITE_BOUND, 1.0);
  model.addGreaterOrEqual("floor", 1.0 * x, -3.0);
  expectAgreement(model);
}

TEST(Mps, CountsTheSameBoundsAsInfiniteAsTheHighsBackend) {
  // An MPS reader counts a bound as infinite from 1e20 on. The direct
  // backend has to draw the line at the same value.
  for (const double bound : {0.5 * INFINITE_BOUND, 1.0e25}) {
    Model model("large");
    static_cast<void>(model.addContinuous("x", 0.0, bound, -1.0));
    expectAgreement(model);
  }
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

TEST(Mps, RefusesNamesTheTextCannotCarry) {
  // A space would split the field, and COST is the objective row.
  Model spaced("names");
  static_cast<void>(spaced.addBinary("two words"));
  EXPECT_THROW(static_cast<void>(toMps(spaced)), std::invalid_argument);

  Model objective("names");
  const auto x = objective.addBinary("x");
  objective.addEqual("COST", 1.0 * x, 1.0);
  EXPECT_THROW(static_cast<void>(toMps(objective)), std::invalid_argument);
}

} // namespace
} // namespace mqt::scpd::milp
