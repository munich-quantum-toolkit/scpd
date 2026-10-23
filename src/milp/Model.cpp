/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/milp/Model.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <format>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace mqt::scpd::milp {
namespace {

/// Whether a coefficient is a finite number below the infinite bound. HiGHS
/// reads a cost from INFINITE_BOUND on as infinite, while another MPS reader
/// takes it as a number, so only a value below the bound means the same model
/// to every backend.
bool belowInfinity(const double value) {
  return std::abs(value) < INFINITE_BOUND;
}

/// Refuses an expression with a term or a constant that is not a finite
/// number below the infinite bound. @p row names the constraint of the
/// expression, and is empty for the objective.
void requireFinite(
    const std::vector<std::pair<std::size_t, double>>& coefficients,
    const double constant, const std::string_view row) {
  const auto where = [row] {
    return row.empty() ? std::string("the objective")
                       : std::format("constraint '{}'", row);
  };
  for (const auto& [index, coefficient] : coefficients) {
    if (!belowInfinity(coefficient)) {
      throw std::invalid_argument(
          std::format("the coefficient of variable {} in {} is {}, not a "
                      "finite number below {}",
                      index, where(), coefficient, INFINITE_BOUND));
    }
  }
  if (!belowInfinity(constant)) {
    throw std::invalid_argument(
        std::format("the constant of {} is {}, not a finite number below {}",
                    where(), constant, INFINITE_BOUND));
  }
}

/// A coefficient, or zero where it is too small to be a term.
double withoutNoise(const double coefficient) {
  return std::abs(coefficient) <= COEFFICIENT_EPSILON ? 0.0 : coefficient;
}

} // namespace

LinearExpr& LinearExpr::add(const Var variable, const double coefficient) {
  termList.emplace_back(variable.index, coefficient);
  return *this;
}

LinearExpr& LinearExpr::add(const double constant) {
  constantTerm += constant;
  return *this;
}

LinearExpr& LinearExpr::add(const LinearExpr& other) {
  if (this == &other) {
    return add(LinearExpr(other));
  }
  termList.insert(termList.end(), other.termList.begin(), other.termList.end());
  constantTerm += other.constantTerm;
  return *this;
}

LinearExpr& LinearExpr::operator-=(const LinearExpr& other) {
  if (this == &other) {
    return *this -= LinearExpr(other);
  }
  for (const auto& [index, coefficient] : other.termList) {
    termList.emplace_back(index, -coefficient);
  }
  constantTerm -= other.constantTerm;
  return *this;
}

LinearExpr& LinearExpr::operator*=(const double factor) {
  for (auto& [_, coefficient] : termList) {
    coefficient *= factor;
  }
  constantTerm *= factor;
  return *this;
}

std::vector<std::pair<std::size_t, double>> LinearExpr::combined() const {
  auto sorted = termList;
  // A stable sort adds the terms of one variable in the order they were added,
  // so the sum rounds the same with every standard library.
  std::ranges::stable_sort(sorted, {}, &std::pair<std::size_t, double>::first);

  std::vector<std::pair<std::size_t, double>> result;
  result.reserve(sorted.size());
  for (const auto& [index, coefficient] : sorted) {
    if (!result.empty() && result.back().first == index) {
      result.back().second += coefficient;
    } else {
      result.emplace_back(index, coefficient);
    }
  }
  std::erase_if(result, [](const auto& term) {
    return std::abs(term.second) <= COEFFICIENT_EPSILON;
  });
  return result;
}

LinearExpr operator*(const double coefficient, const Var variable) {
  LinearExpr expression;
  expression.add(variable, coefficient);
  return expression;
}

LinearExpr operator*(const Var variable, const double coefficient) {
  return coefficient * variable;
}

LinearExpr operator*(const double factor, LinearExpr expression) {
  expression *= factor;
  return expression;
}

LinearExpr operator*(LinearExpr expression, const double factor) {
  expression *= factor;
  return expression;
}

LinearExpr operator+(LinearExpr left, const LinearExpr& right) {
  left.add(right);
  return left;
}

LinearExpr operator-(LinearExpr left, const LinearExpr& right) {
  left -= right;
  return left;
}

LinearExpr operator-(const LinearExpr& expression) {
  LinearExpr negated;
  negated -= expression;
  return negated;
}

Var Model::addVariable(const std::string_view name, const double lower,
                       const double upper, const VarType type,
                       const double objective) {
  if (name.empty()) {
    throw std::invalid_argument("a model variable needs a name");
  }
  if (std::isnan(lower) || std::isnan(upper)) {
    throw std::invalid_argument(
        std::format("variable '{}' has a bound that is not a number", name));
  }
  if (!belowInfinity(objective)) {
    throw std::invalid_argument(std::format(
        "variable '{}' has the objective coefficient {}, not a finite number "
        "below {}",
        name, objective, INFINITE_BOUND));
  }
  auto low = lower;
  auto high = upper;
  if (type == VarType::Binary) {
    low = std::max(low, 0.0);
    high = std::min(high, 1.0);
  }
  if (low > high) {
    throw std::invalid_argument(std::format(
        "variable '{}' has a lower bound of {} above its upper bound of {}",
        name, low, high));
  }
  // A lower bound of infinity, or an upper bound of minus infinity, leaves no
  // value. HiGHS refuses such a variable, and MPS has no way to write it.
  if (low >= INFINITE_BOUND || high <= -INFINITE_BOUND) {
    throw std::invalid_argument(std::format(
        "variable '{}' has the bounds {} and {}, which no value lies between",
        name, low, high));
  }
  if (takenNames.contains(std::string(name))) {
    throw std::invalid_argument(std::format(
        "the name '{}' is already taken in model '{}'", name, modelName));
  }

  variableList.push_back({.name = std::string(name),
                          .lower = std::max(low, -INFINITE_BOUND),
                          .upper = std::min(high, INFINITE_BOUND),
                          .type = type,
                          .objective = withoutNoise(objective)});
  takenNames.emplace(name);
  return Var{variableList.size() - 1};
}

Var Model::addBinary(const std::string_view name, const double objective) {
  return addVariable(name, 0.0, 1.0, VarType::Binary, objective);
}

Var Model::addInteger(const std::string_view name, const double lower,
                      const double upper, const double objective) {
  return addVariable(name, lower, upper, VarType::Integer, objective);
}

Var Model::addContinuous(const std::string_view name, const double lower,
                         const double upper, const double objective) {
  return addVariable(name, lower, upper, VarType::Continuous, objective);
}

void Model::addRow(const std::string_view name, const LinearExpr& expression,
                   const double lower, const double upper) {
  if (name.empty()) {
    throw std::invalid_argument("a model constraint needs a name");
  }
  if (takenNames.contains(std::string(name))) {
    throw std::invalid_argument(std::format(
        "the name '{}' is already taken in model '{}'", name, modelName));
  }

  // The terms are checked before they are combined, so that a variable of
  // another model is refused also when its coefficient is zero.
  for (const auto& [index, _] : expression.terms()) {
    if (index >= variableList.size()) {
      throw std::invalid_argument(std::format(
          "constraint '{}' names variable {}, which model '{}' does not have",
          name, index, modelName));
    }
  }
  auto coefficients = expression.combined();
  requireFinite(coefficients, expression.constant(), name);
  // An infinite bound is a free side. A bound that is not a number is not a
  // side at all, and the backends would read it in different ways.
  if (std::isnan(lower) || std::isnan(upper)) {
    throw std::invalid_argument(
        std::format("constraint '{}' has a bound that is not a number", name));
  }

  // The constant of the expression moves to the other side, which is what
  // makes `x + 3 <= 5` mean `x <= 2` rather than a row with a constant in it.
  const auto shift = expression.constant();
  auto low = lower;
  auto high = upper;
  if (low > -INFINITE_BOUND) {
    low -= shift;
  }
  if (high < INFINITE_BOUND) {
    high -= shift;
  }
  if (low > high) {
    throw std::invalid_argument(std::format(
        "constraint '{}' has a lower bound of {} above its upper bound of {}",
        name, low, high));
  }
  if (low >= INFINITE_BOUND || high <= -INFINITE_BOUND) {
    throw std::invalid_argument(std::format(
        "constraint '{}' has the bounds {} and {}, which no value lies between",
        name, low, high));
  }

  constraintList.push_back({.name = std::string(name),
                            .coefficients = std::move(coefficients),
                            .lower = std::max(low, -INFINITE_BOUND),
                            .upper = std::min(high, INFINITE_BOUND)});
  takenNames.emplace(name);
}

void Model::addLessOrEqual(const std::string_view name,
                           const LinearExpr& expression, const double bound) {
  addRow(name, expression, -INFINITE_BOUND, bound);
}

void Model::addGreaterOrEqual(const std::string_view name,
                              const LinearExpr& expression,
                              const double bound) {
  addRow(name, expression, bound, INFINITE_BOUND);
}

void Model::addEqual(const std::string_view name, const LinearExpr& expression,
                     const double bound) {
  addRow(name, expression, bound, bound);
}

void Model::addRange(const std::string_view name, const LinearExpr& expression,
                     const double lower, const double upper) {
  addRow(name, expression, lower, upper);
}

void Model::addObjective(const LinearExpr& expression) {
  for (const auto& [index, _] : expression.terms()) {
    if (index >= variableList.size()) {
      throw std::invalid_argument(std::format(
          "the objective names variable {}, which model '{}' does not have",
          index, modelName));
    }
  }
  const auto coefficients = expression.combined();
  requireFinite(coefficients, expression.constant(), {});
  // Each sum is checked as a single coefficient is, and all checks come
  // first, so that a refused expression leaves the objective as it was.
  std::vector<double> sums;
  sums.reserve(coefficients.size());
  for (const auto& [index, coefficient] : coefficients) {
    const auto sum = variableList[index].objective + coefficient;
    if (!belowInfinity(sum)) {
      throw std::invalid_argument(std::format(
          "the objective coefficient of variable '{}' comes to {}, not a "
          "finite number below {}",
          variableList[index].name, sum, INFINITE_BOUND));
    }
    sums.push_back(sum);
  }
  const auto summedOffset = offset + expression.constant();
  if (!belowInfinity(summedOffset)) {
    throw std::invalid_argument(
        std::format("the objective offset comes to {}, not a finite number "
                    "below {}",
                    summedOffset, INFINITE_BOUND));
  }
  // The sum of several expressions can cancel to a coefficient that is no
  // term, which goes as a single small coefficient would.
  for (std::size_t term = 0; term < coefficients.size(); ++term) {
    variableList[coefficients[term].first].objective = withoutNoise(sums[term]);
  }
  offset = summedOffset;
}

bool Model::isMixedInteger() const {
  return std::ranges::any_of(variableList, [](const Variable& variable) {
    return variable.type != VarType::Continuous;
  });
}

const std::string& Model::nameOf(const Var variable) const {
  if (variable.index >= variableList.size()) {
    throw std::out_of_range(std::format("variable {} is not in model '{}'",
                                        variable.index, modelName));
  }
  return variableList[variable.index].name;
}

double Solution::valueOf(const Var variable) const {
  if (variable.index >= values.size()) {
    throw std::out_of_range(
        std::format("variable {} is not in the solution", variable.index));
  }
  return values[variable.index];
}

bool Solution::isSet(const Var variable) const {
  return valueOf(variable) > 0.5;
}

std::string_view statusName(const SolveStatus status) {
  switch (status) {
  case SolveStatus::Optimal:
    return "optimal";
  case SolveStatus::Feasible:
    return "feasible";
  case SolveStatus::Infeasible:
    return "infeasible";
  case SolveStatus::Unbounded:
    return "unbounded";
  case SolveStatus::Error:
    break;
  }
  return "error";
}

} // namespace mqt::scpd::milp
