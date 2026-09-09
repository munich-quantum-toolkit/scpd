/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The solver-neutral model: what it accepts, what it refuses, and how an
// expression is reduced before a backend sees it.

#include "mqt-scpd/milp/Model.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

namespace mqt::scpd::milp {
namespace {

TEST(MilpModel, CombinesRepeatedTermsAndDropsZeroes) {
  Model model("terms");
  const auto x = model.addBinary("x");
  const auto y = model.addBinary("y");

  LinearExpr expression;
  expression.add(y, 2.0).add(x, 1.0).add(y, -2.0).add(x, 0.5);

  const auto combined = expression.combined();
  // y cancels itself out and disappears; x is one term, not two, and the
  // terms come back in variable order rather than insertion order.
  ASSERT_EQ(combined.size(), 1U);
  EXPECT_EQ(combined.front().first, x.index);
  EXPECT_DOUBLE_EQ(combined.front().second, 1.5);
}

TEST(MilpModel, MovesTheConstantOfAnExpressionToTheBound) {
  Model model("shift");
  const auto x = model.addContinuous("x", 0.0, 10.0);
  model.addLessOrEqual("shifted", (1.0 * x) + 3.0, 5.0);

  ASSERT_EQ(model.constraintCount(), 1U);
  const auto& row = model.constraints().front();
  EXPECT_DOUBLE_EQ(row.upper, 2.0);
  EXPECT_LE(row.lower, -INFINITE_BOUND);
}

TEST(MilpModel, WritesAnEqualityAsTwoEqualBounds) {
  Model model("equality");
  const auto x = model.addInteger("x", 0.0, 4.0);
  model.addEqual("pinned", 1.0 * x, 3.0);

  const auto& row = model.constraints().front();
  EXPECT_DOUBLE_EQ(row.lower, 3.0);
  EXPECT_DOUBLE_EQ(row.upper, 3.0);
}

TEST(MilpModel, RefusesADuplicateNameAcrossVariablesAndRows) {
  Model model("names");
  const auto x = model.addBinary("shared");
  EXPECT_THROW(static_cast<void>(model.addBinary("shared")), std::invalid_argument);
  EXPECT_THROW(model.addLessOrEqual("shared", 1.0 * x, 1.0), std::invalid_argument);

  model.addLessOrEqual("row", 1.0 * x, 1.0);
  EXPECT_THROW(static_cast<void>(model.addBinary("row")), std::invalid_argument);
}

TEST(MilpModel, RefusesCrossedBoundsAndAnEmptyName) {
  Model model("bounds");
  EXPECT_THROW(static_cast<void>(model.addContinuous("x", 5.0, 1.0)), std::invalid_argument);
  EXPECT_THROW(static_cast<void>(model.addContinuous("", 0.0, 1.0)), std::invalid_argument);

  const auto x = model.addContinuous("x", 0.0, 1.0);
  EXPECT_THROW(model.addRange("crossed", 1.0 * x, 4.0, 2.0), std::invalid_argument);
}

TEST(MilpModel, RefusesAVariableOfAnotherModel) {
  Model first("first");
  Model second("second");
  static_cast<void>(first.addBinary("a"));
  static_cast<void>(first.addBinary("b"));
  const auto foreign = first.addBinary("c");

  // The second model has no variables at all, so index 2 names nothing.
  EXPECT_THROW(second.addLessOrEqual("row", 1.0 * foreign, 1.0), std::invalid_argument);
  EXPECT_THROW(second.addObjective(1.0 * foreign), std::invalid_argument);
}

TEST(MilpModel, ClampsABinaryToZeroAndOne) {
  Model model("binary");
  const auto x = model.addVariable("x", -5.0, 7.0, VarType::Binary);
  EXPECT_DOUBLE_EQ(model.variables()[x.index].lower, 0.0);
  EXPECT_DOUBLE_EQ(model.variables()[x.index].upper, 1.0);
}

TEST(MilpModel, AccumulatesTheObjectiveOnTheVariables) {
  Model model("objective");
  const auto x = model.addBinary("x");
  model.addObjective(2.0 * x);
  model.addObjective((3.0 * x) + 4.0);

  EXPECT_DOUBLE_EQ(model.variables()[x.index].objective, 5.0);
  EXPECT_DOUBLE_EQ(model.objectiveOffset(), 4.0);
}

TEST(MilpModel, KnowsWhetherItIsMixedInteger) {
  Model continuous("continuous");
  static_cast<void>(continuous.addContinuous("x", 0.0, 1.0));
  EXPECT_FALSE(continuous.isMixedInteger());

  Model mixed("mixed");
  static_cast<void>(mixed.addContinuous("x", 0.0, 1.0));
  static_cast<void>(mixed.addBinary("y"));
  EXPECT_TRUE(mixed.isMixedInteger());
}

TEST(MilpSolution, ReadsABinaryThroughATolerance) {
  const Solution solution{.status = SolveStatus::Optimal,
                          .objective = 1.0,
                          .values = {0.9999999, 1.0e-9}};
  EXPECT_TRUE(solution.isSet(Var{0}));
  EXPECT_FALSE(solution.isSet(Var{1}));
  EXPECT_THROW(static_cast<void>(solution.valueOf(Var{2})), std::out_of_range);
}

} // namespace
} // namespace mqt::scpd::milp
