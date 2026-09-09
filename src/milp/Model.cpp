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

/// A coefficient below this is not a term. It is the scale at which a
/// solver's own tolerances start, so a smaller one carries no information.
constexpr double COEFFICIENT_EPSILON = 1.0e-12;

} // namespace

LinearExpr& LinearExpr::add(const Var variable, const double coefficient) {
  terms_.emplace_back(variable.index, coefficient);
  return *this;
}

LinearExpr& LinearExpr::add(const double constant) {
  constant_ += constant;
  return *this;
}

LinearExpr& LinearExpr::add(const LinearExpr& other) {
  terms_.insert(terms_.end(), other.terms_.begin(), other.terms_.end());
  constant_ += other.constant_;
  return *this;
}

LinearExpr& LinearExpr::operator-=(const LinearExpr& other) {
  for (const auto& [index, coefficient] : other.terms_) {
    terms_.emplace_back(index, -coefficient);
  }
  constant_ -= other.constant_;
  return *this;
}

LinearExpr& LinearExpr::operator*=(const double factor) {
  for (auto& [_, coefficient] : terms_) {
    coefficient *= factor;
  }
  constant_ *= factor;
  return *this;
}

std::vector<std::pair<std::size_t, double>> LinearExpr::combined() const {
  auto sorted = terms_;
  std::ranges::sort(sorted, {}, &std::pair<std::size_t, double>::first);

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
    return std::abs(term.second) < COEFFICIENT_EPSILON;
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

LinearExpr operator-(LinearExpr expression) {
  LinearExpr negated;
  negated -= expression;
  return negated;
}

Var Model::addVariable(const std::string_view name, const double lower, const double upper,
                       const VarType type, const double objective) {
  if (name.empty()) {
    throw std::invalid_argument("a model variable needs a name");
  }
  if (lower > upper) {
    throw std::invalid_argument(std::format(
        "variable '{}' has a lower bound of {} above its upper bound of {}", name, lower, upper));
  }
  if (takenNames_.contains(std::string(name))) {
    throw std::invalid_argument(
        std::format("the name '{}' is already taken in model '{}'", name, name_));
  }

  auto low = lower;
  auto high = upper;
  if (type == VarType::Binary) {
    low = std::max(low, 0.0);
    high = std::min(high, 1.0);
  }
  variables_.push_back({std::string(name), low, high, type, objective});
  takenNames_.emplace(name);
  return Var{variables_.size() - 1};
}

Var Model::addBinary(const std::string_view name, const double objective) {
  return addVariable(name, 0.0, 1.0, VarType::Binary, objective);
}

Var Model::addInteger(const std::string_view name, const double lower, const double upper,
                      const double objective) {
  return addVariable(name, lower, upper, VarType::Integer, objective);
}

Var Model::addContinuous(const std::string_view name, const double lower, const double upper,
                         const double objective) {
  return addVariable(name, lower, upper, VarType::Continuous, objective);
}

void Model::addRow(const std::string_view name, const LinearExpr& expression, const double lower,
                   const double upper) {
  if (name.empty()) {
    throw std::invalid_argument("a model constraint needs a name");
  }
  if (takenNames_.contains(std::string(name))) {
    throw std::invalid_argument(
        std::format("the name '{}' is already taken in model '{}'", name, name_));
  }

  auto coefficients = expression.combined();
  for (const auto& [index, _] : coefficients) {
    if (index >= variables_.size()) {
      throw std::invalid_argument(std::format(
          "constraint '{}' names variable {}, which model '{}' does not have", name, index, name_));
    }
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
        "constraint '{}' has a lower bound of {} above its upper bound of {}", name, low, high));
  }

  constraints_.push_back({std::string(name), std::move(coefficients), low, high});
  takenNames_.emplace(name);
}

void Model::addLessOrEqual(const std::string_view name, const LinearExpr& expression,
                           const double bound) {
  addRow(name, expression, -INFINITE_BOUND, bound);
}

void Model::addGreaterOrEqual(const std::string_view name, const LinearExpr& expression,
                              const double bound) {
  addRow(name, expression, bound, INFINITE_BOUND);
}

void Model::addEqual(const std::string_view name, const LinearExpr& expression,
                     const double bound) {
  addRow(name, expression, bound, bound);
}

void Model::addRange(const std::string_view name, const LinearExpr& expression, const double lower,
                     const double upper) {
  addRow(name, expression, lower, upper);
}

void Model::addObjective(const LinearExpr& expression) {
  for (const auto& [index, coefficient] : expression.combined()) {
    if (index >= variables_.size()) {
      throw std::invalid_argument(std::format(
          "the objective names variable {}, which model '{}' does not have", index, name_));
    }
    variables_[index].objective += coefficient;
  }
  offset_ += expression.constant();
}

bool Model::isMixedInteger() const {
  return std::ranges::any_of(variables_, [](const Variable& variable) {
    return variable.type != VarType::Continuous;
  });
}

const std::string& Model::nameOf(const Var variable) const {
  if (variable.index >= variables_.size()) {
    throw std::out_of_range(
        std::format("variable {} is not in model '{}'", variable.index, name_));
  }
  return variables_[variable.index].name;
}

double Solution::valueOf(const Var variable) const {
  if (variable.index >= values.size()) {
    throw std::out_of_range(std::format("variable {} is not in the solution", variable.index));
  }
  return values[variable.index];
}

bool Solution::isSet(const Var variable) const { return valueOf(variable) > 0.5; }

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
