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

#include <limits>
#include <stdexcept>

namespace mqt::scpd::milp {
namespace {

constexpr auto NOT_A_NUMBER = std::numeric_limits<double>::quiet_NaN();
constexpr auto INFINITE = std::numeric_limits<double>::infinity();

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

TEST(MilpModel, AddsTheTermsOfAVariableInTheOrderTheyCame) {
  Model model("order");
  const auto x = model.addContinuous("x", 0.0, 1.0);
  const auto y = model.addContinuous("y", 0.0, 1.0);

  // 1e16 + 1 rounds back to 1e16, so the terms of x sum to 0 in the order
  // they came and to 1 in any order that takes -1e16 before 1. The terms of
  // y between them make the sort move the terms of x.
  LinearExpr expression;
  expression.add(x, 1.0e16).add(y, 1.0).add(x, 1.0).add(y, 1.0).add(x, -1.0e16);

  const auto combined = expression.combined();
  ASSERT_EQ(combined.size(), 1U);
  EXPECT_EQ(combined.front().first, y.index);
  EXPECT_DOUBLE_EQ(combined.front().second, 2.0);
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
  EXPECT_THROW(static_cast<void>(model.addBinary("shared")),
               std::invalid_argument);
  EXPECT_THROW(model.addLessOrEqual("shared", 1.0 * x, 1.0),
               std::invalid_argument);

  model.addLessOrEqual("row", 1.0 * x, 1.0);
  EXPECT_THROW(static_cast<void>(model.addBinary("row")),
               std::invalid_argument);
}

TEST(MilpModel, RefusesCrossedBoundsAndAnEmptyName) {
  Model model("bounds");
  EXPECT_THROW(static_cast<void>(model.addContinuous("x", 5.0, 1.0)),
               std::invalid_argument);
  EXPECT_THROW(static_cast<void>(model.addContinuous("", 0.0, 1.0)),
               std::invalid_argument);
  EXPECT_THROW(static_cast<void>(model.addContinuous(
                   "x", std::numeric_limits<double>::quiet_NaN(), 1.0)),
               std::invalid_argument);
  // Clipped to [0, 1], the bounds [2, 3] of a binary variable cross.
  EXPECT_THROW(
      static_cast<void>(model.addVariable("b", 2.0, 3.0, VarType::Binary)),
      std::invalid_argument);

  const auto x = model.addContinuous("x", 0.0, 1.0);
  EXPECT_THROW(model.addRange("crossed", 1.0 * x, 4.0, 2.0),
               std::invalid_argument);
}

TEST(MilpModel, RefusesBoundsThatLeaveNoValue) {
  // A lower bound of infinity or an upper bound of minus infinity leaves no
  // value. HiGHS refuses such a variable, and the MPS text would write it as
  // free.
  Model model("empty");
  EXPECT_THROW(static_cast<void>(
                   model.addContinuous("x", INFINITE_BOUND, INFINITE_BOUND)),
               std::invalid_argument);
  EXPECT_THROW(static_cast<void>(
                   model.addContinuous("x", -INFINITE_BOUND, -INFINITE_BOUND)),
               std::invalid_argument);
  EXPECT_EQ(model.variableCount(), 0U);

  const auto x = model.addContinuous("x", 0.0, 1.0);
  EXPECT_THROW(model.addLessOrEqual("below", 1.0 * x, -INFINITE_BOUND),
               std::invalid_argument);
  EXPECT_THROW(model.addGreaterOrEqual("above", 1.0 * x, INFINITE_BOUND),
               std::invalid_argument);
  EXPECT_EQ(model.constraintCount(), 0U);
}

TEST(MilpModel, StoresABoundBeyondTheInfiniteBoundAsNone) {
  // A backend reads a bound of INFINITE_BOUND as none, so a larger bound is
  // stored as INFINITE_BOUND.
  Model model("free");
  const auto x = model.addContinuous("x", -INFINITE, 10.0 * INFINITE_BOUND);
  model.addRange("row", (1.0 * x) + 1.0, -10.0 * INFINITE_BOUND, INFINITE);
  EXPECT_EQ(model.variables()[x.index].lower, -INFINITE_BOUND);
  EXPECT_EQ(model.variables()[x.index].upper, INFINITE_BOUND);
  EXPECT_EQ(model.constraints()[0].lower, -INFINITE_BOUND);
  EXPECT_EQ(model.constraints()[0].upper, INFINITE_BOUND);
}

TEST(MilpModel, RefusesARowBoundThatIsNotANumber) {
  // HiGHS refuses a bound that is not a number, and the MPS text cannot carry
  // one, so the model refuses it before either backend sees it.
  Model model("rows");
  const auto x = model.addContinuous("x", 0.0, 10.0);

  EXPECT_THROW(model.addLessOrEqual("upper", 1.0 * x, NOT_A_NUMBER),
               std::invalid_argument);
  EXPECT_THROW(model.addGreaterOrEqual("lower", 1.0 * x, NOT_A_NUMBER),
               std::invalid_argument);
  EXPECT_THROW(model.addEqual("equal", 1.0 * x, NOT_A_NUMBER),
               std::invalid_argument);
  EXPECT_THROW(model.addRange("range", 1.0 * x, 0.0, NOT_A_NUMBER),
               std::invalid_argument);
  EXPECT_EQ(model.constraintCount(), 0U);

  // An infinite bound is a free side.
  model.addLessOrEqual("free", 1.0 * x, INFINITE);
  EXPECT_EQ(model.constraintCount(), 1U);
}

TEST(MilpModel, RefusesACoefficientOrConstantThatIsNotFinite) {
  Model model("terms");
  const auto x = model.addContinuous("x", 0.0, 10.0);

  EXPECT_THROW(model.addLessOrEqual("nan", NOT_A_NUMBER * x, 1.0),
               std::invalid_argument);
  EXPECT_THROW(model.addLessOrEqual("inf", INFINITE * x, 1.0),
               std::invalid_argument);
  EXPECT_THROW(model.addLessOrEqual("constant", (1.0 * x) + INFINITE, 1.0),
               std::invalid_argument);
  EXPECT_EQ(model.constraintCount(), 0U);
}

TEST(MilpModel, RefusesAnObjectiveThatIsNotFinite) {
  Model model("objective");
  EXPECT_THROW(
      static_cast<void>(model.addContinuous("nan", 0.0, 1.0, NOT_A_NUMBER)),
      std::invalid_argument);
  EXPECT_THROW(static_cast<void>(model.addBinary("inf", INFINITE)),
               std::invalid_argument);

  const auto y = model.addBinary("y");
  const auto x = model.addBinary("x", 1.0);
  // The term on y comes first in the model, and must not stay behind when the
  // term on x is refused.
  EXPECT_THROW(model.addObjective((2.0 * y) + (NOT_A_NUMBER * x)),
               std::invalid_argument);
  EXPECT_THROW(model.addObjective((2.0 * y) + NOT_A_NUMBER),
               std::invalid_argument);
  EXPECT_DOUBLE_EQ(model.variables()[x.index].objective, 1.0);
  EXPECT_DOUBLE_EQ(model.variables()[y.index].objective, 0.0);
  EXPECT_DOUBLE_EQ(model.objectiveOffset(), 0.0);
}

TEST(MilpModel, RefusesACoefficientFromTheInfiniteBoundOn) {
  // HiGHS reads a cost from INFINITE_BOUND on as infinite, and another MPS
  // reader as a number, so the backends would solve two models.
  Model model("large");
  EXPECT_THROW(static_cast<void>(model.addBinary("cost", INFINITE_BOUND)),
               std::invalid_argument);
  EXPECT_THROW(static_cast<void>(model.addContinuous("gain", 0.0, 1.0,
                                                     -10.0 * INFINITE_BOUND)),
               std::invalid_argument);

  const auto x = model.addContinuous("x", 0.0, 1.0, 0.6 * INFINITE_BOUND);
  EXPECT_THROW(model.addLessOrEqual("row", INFINITE_BOUND * x, 1.0),
               std::invalid_argument);
  EXPECT_THROW(model.addLessOrEqual("shift", (1.0 * x) - INFINITE_BOUND, 1.0),
               std::invalid_argument);
  // The limit holds for the stored row, in which the terms of one variable
  // are summed.
  EXPECT_THROW(
      model.addLessOrEqual(
          "sum", (0.6 * INFINITE_BOUND * x) + (0.6 * INFINITE_BOUND * x), 1.0),
      std::invalid_argument);
  EXPECT_EQ(model.constraintCount(), 0U);

  // A sum is held to the same limit as a single coefficient, and a refused
  // sum leaves the objective as it was.
  EXPECT_THROW(model.addObjective(0.6 * INFINITE_BOUND * x),
               std::invalid_argument);
  model.addObjective(LinearExpr(0.6 * INFINITE_BOUND));
  EXPECT_THROW(model.addObjective(LinearExpr(0.6 * INFINITE_BOUND)),
               std::invalid_argument);
  EXPECT_DOUBLE_EQ(model.variables()[x.index].objective, 0.6 * INFINITE_BOUND);
  EXPECT_DOUBLE_EQ(model.objectiveOffset(), 0.6 * INFINITE_BOUND);
}

TEST(MilpModel, RefusesAVariableOfAnotherModel) {
  Model first("first");
  Model second("second");
  static_cast<void>(first.addBinary("a"));
  static_cast<void>(first.addBinary("b"));
  const auto foreign = first.addBinary("c");

  // The second model has no variables at all, so index 2 names nothing.
  EXPECT_THROW(second.addLessOrEqual("row", 1.0 * foreign, 1.0),
               std::invalid_argument);
  EXPECT_THROW(second.addObjective(1.0 * foreign), std::invalid_argument);
  // A zero coefficient drops out of the combined terms, but the variable is
  // still foreign.
  EXPECT_THROW(second.addLessOrEqual("zero", 0.0 * foreign, 1.0),
               std::invalid_argument);
  EXPECT_THROW(second.addObjective(0.0 * foreign), std::invalid_argument);
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

TEST(MilpModel, DropsAnObjectiveCoefficientThatSumsToNoTerm) {
  // Each part is a term, but their sum is not. Both backends read the model
  // as it stores the coefficient, so the model drops it once, here.
  Model model("noise");
  const auto x = model.addBinary("x", 1.0);
  const auto y = model.addBinary("y", 0.5 * COEFFICIENT_EPSILON);
  model.addObjective((-1.0 + (0.5 * COEFFICIENT_EPSILON)) * x);

  EXPECT_EQ(model.variables()[x.index].objective, 0.0);
  EXPECT_EQ(model.variables()[y.index].objective, 0.0);
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

TEST(MilpModel, ScalesAndNegatesAnExpressionFromEitherSide) {
  Model model("algebra");
  const auto x = model.addBinary("x");
  const auto y = model.addBinary("y");

  LinearExpr shifted;
  shifted.add(x, 1.0).add(2.0);
  const auto left = 3.0 * shifted;
  const auto right = shifted * 3.0;
  const auto negated = -((x * 2.0) - (1.0 * y));

  for (const auto& scaled : {left, right}) {
    ASSERT_EQ(scaled.combined().size(), 1U);
    EXPECT_DOUBLE_EQ(scaled.combined().front().second, 3.0);
    EXPECT_DOUBLE_EQ(scaled.constant(), 6.0);
  }
  const auto terms = negated.combined();
  ASSERT_EQ(terms.size(), 2U);
  EXPECT_EQ(terms[0].first, x.index);
  EXPECT_DOUBLE_EQ(terms[0].second, -2.0);
  EXPECT_EQ(terms[1].first, y.index);
  EXPECT_DOUBLE_EQ(terms[1].second, 1.0);
}

TEST(MilpModel, NamesItsVariablesAndRefusesAnUnknownOne) {
  Model model("named");
  const auto x = model.addBinary("x");

  EXPECT_EQ(model.nameOf(x), "x");
  EXPECT_THROW(static_cast<void>(model.nameOf(Var{1})), std::out_of_range);
}

TEST(MilpSolution, NamesEveryStatus) {
  EXPECT_EQ(statusName(SolveStatus::Optimal), "optimal");
  EXPECT_EQ(statusName(SolveStatus::Feasible), "feasible");
  EXPECT_EQ(statusName(SolveStatus::Infeasible), "infeasible");
  EXPECT_EQ(statusName(SolveStatus::Unbounded), "unbounded");
  EXPECT_EQ(statusName(SolveStatus::Error), "error");
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
