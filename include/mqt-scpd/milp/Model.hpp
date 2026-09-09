/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#pragma once

#include "mqt-scpd/milp/mqt_scpd_milp_export.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>

namespace mqt::scpd::milp {

/// A bound that does not constrain. The value is what MPS calls infinite and
/// what every backend recognizes as free.
inline constexpr double INFINITE_BOUND = 1.0e30;

/// Whether a variable ranges over the reals or the integers.
enum class VarType : std::uint8_t { Continuous, Integer, Binary };

/// Which way the objective is optimized.
enum class Sense : std::uint8_t { Minimize, Maximize };

/// A variable of a model, as an index into its variable list.
struct Var {
  std::size_t index = 0;

  [[nodiscard]] bool operator==(const Var&) const = default;
};

/// A weighted sum of variables plus a constant.
///
/// The expression keeps its terms in the order they were added and does not
/// combine two terms on the same variable. Combining happens once, when the
/// model is handed to a backend, so that building an expression stays cheap
/// in the loops that build one.
class MQT_SCPD_MILP_EXPORT LinearExpr {
public:
  LinearExpr() = default;
  /// The expression that is one variable.
  LinearExpr(Var variable) : terms_{{variable.index, 1.0}} {} // NOLINT(google-explicit-constructor)
  /// The expression that is one constant.
  LinearExpr(double constant) : constant_(constant) {} // NOLINT(google-explicit-constructor)

  /// Add `coefficient * variable`.
  LinearExpr& add(Var variable, double coefficient);
  /// Add a constant.
  LinearExpr& add(double constant);
  /// Add another expression.
  LinearExpr& add(const LinearExpr& other);

  LinearExpr& operator+=(const LinearExpr& other) { return add(other); }
  LinearExpr& operator-=(const LinearExpr& other);
  /// Multiply every term and the constant by a number. There is no product
  /// of two expressions, because that is not a linear model any more.
  LinearExpr& operator*=(double factor);

  /// The terms, as (variable index, coefficient) pairs, in insertion order.
  [[nodiscard]] const std::vector<std::pair<std::size_t, double>>& terms() const { return terms_; }
  [[nodiscard]] double constant() const { return constant_; }

  /// The terms with every repeated variable combined, sorted by index, and
  /// zero coefficients dropped.
  [[nodiscard]] std::vector<std::pair<std::size_t, double>> combined() const;

private:
  std::vector<std::pair<std::size_t, double>> terms_;
  double constant_ = 0.0;
};

[[nodiscard]] MQT_SCPD_MILP_EXPORT LinearExpr operator*(double coefficient, Var variable);
[[nodiscard]] MQT_SCPD_MILP_EXPORT LinearExpr operator*(Var variable, double coefficient);
[[nodiscard]] MQT_SCPD_MILP_EXPORT LinearExpr operator*(double factor, LinearExpr expression);
[[nodiscard]] MQT_SCPD_MILP_EXPORT LinearExpr operator*(LinearExpr expression, double factor);
[[nodiscard]] MQT_SCPD_MILP_EXPORT LinearExpr operator+(LinearExpr left, const LinearExpr& right);
[[nodiscard]] MQT_SCPD_MILP_EXPORT LinearExpr operator-(LinearExpr left, const LinearExpr& right);
[[nodiscard]] MQT_SCPD_MILP_EXPORT LinearExpr operator-(LinearExpr expression);

/// One variable of a model.
struct Variable {
  std::string name;
  double lower = 0.0;
  double upper = INFINITE_BOUND;
  VarType type = VarType::Continuous;
  /// The objective coefficient. Held on the variable because that is how
  /// both MPS and the backends want it.
  double objective = 0.0;
};

/// One linear constraint, as `lower <= sum(coefficient * variable) <= upper`.
///
/// An equality has both bounds equal, and a one-sided constraint leaves the
/// other bound infinite. The row's coefficients are stored combined and
/// sorted, so a backend can hand them straight to its own matrix.
struct Constraint {
  std::string name;
  std::vector<std::pair<std::size_t, double>> coefficients;
  double lower = -INFINITE_BOUND;
  double upper = INFINITE_BOUND;
};

/// A mixed-integer linear program, independent of any solver.
///
/// The model is the whole surface the planning stages have on a solver:
/// variables with bounds and a type, linear constraints, and one linear
/// objective. That is what MPS carries, which is what makes the in-process
/// backend and the bring-your-own-licence backend interchangeable.
class MQT_SCPD_MILP_EXPORT Model {
public:
  explicit Model(std::string name = "model") : name_(std::move(name)) {}

  /// Add a variable and return it.
  ///
  /// @throws std::invalid_argument when the bounds are crossed, or when the
  /// name is empty or already taken.
  Var addVariable(std::string_view name, double lower, double upper, VarType type,
                  double objective = 0.0);
  /// Add a binary variable.
  Var addBinary(std::string_view name, double objective = 0.0);
  /// Add an integer variable.
  Var addInteger(std::string_view name, double lower, double upper, double objective = 0.0);
  /// Add a continuous variable.
  Var addContinuous(std::string_view name, double lower, double upper, double objective = 0.0);

  /// Add `expression <= bound`.
  /// @throws std::invalid_argument on an unknown variable or a taken name.
  void addLessOrEqual(std::string_view name, const LinearExpr& expression, double bound);
  /// Add `expression >= bound`.
  /// @throws std::invalid_argument on an unknown variable or a taken name.
  void addGreaterOrEqual(std::string_view name, const LinearExpr& expression, double bound);
  /// Add `expression == bound`.
  /// @throws std::invalid_argument on an unknown variable or a taken name.
  void addEqual(std::string_view name, const LinearExpr& expression, double bound);
  /// Add `lower <= expression <= upper`.
  /// @throws std::invalid_argument on an unknown variable, a taken name, or
  /// crossed bounds.
  void addRange(std::string_view name, const LinearExpr& expression, double lower, double upper);

  /// Add `coefficient` to the objective coefficient of every term of the
  /// expression. The constant of the expression joins the objective offset.
  void addObjective(const LinearExpr& expression);
  void setSense(Sense sense) { sense_ = sense; }

  [[nodiscard]] const std::string& name() const { return name_; }
  [[nodiscard]] Sense sense() const { return sense_; }
  [[nodiscard]] double objectiveOffset() const { return offset_; }
  [[nodiscard]] const std::vector<Variable>& variables() const { return variables_; }
  [[nodiscard]] const std::vector<Constraint>& constraints() const { return constraints_; }
  [[nodiscard]] std::size_t variableCount() const { return variables_.size(); }
  [[nodiscard]] std::size_t constraintCount() const { return constraints_.size(); }
  /// Whether any variable is integral, which is what makes the model a MIP.
  [[nodiscard]] bool isMixedInteger() const;

  /// The name of a variable.
  /// @throws std::out_of_range when the variable is not this model's.
  [[nodiscard]] const std::string& nameOf(Var variable) const;

private:
  /// The row the four constraint forms share.
  void addRow(std::string_view name, const LinearExpr& expression, double lower, double upper);

  std::string name_;
  Sense sense_ = Sense::Minimize;
  double offset_ = 0.0;
  std::vector<Variable> variables_;
  std::vector<Constraint> constraints_;
  /// Names are unique so that a solution can be read back by name, which is
  /// the only thing the MPS round trip carries across.
  std::unordered_set<std::string> takenNames_;
};

/// What a solver made of a model.
enum class SolveStatus : std::uint8_t {
  /// A proven optimal solution.
  Optimal,
  /// A feasible solution that was not proven optimal, usually because a
  /// limit was reached.
  Feasible,
  Infeasible,
  Unbounded,
  /// The solve did not produce a usable answer.
  Error,
};

/// The answer a backend gives.
struct Solution {
  SolveStatus status = SolveStatus::Error;
  double objective = std::numeric_limits<double>::quiet_NaN();
  /// One value per variable of the model, in the model's order. Empty unless
  /// the status is Optimal or Feasible.
  std::vector<double> values;
  /// What the backend called itself, for the metrics.
  std::string backend;
  /// Free text when the status is Error.
  std::string message;

  /// Whether values are present.
  [[nodiscard]] bool hasValues() const {
    return status == SolveStatus::Optimal || status == SolveStatus::Feasible;
  }
  /// The value of a variable, rounded to the nearest integer for an integral
  /// variable of the model that produced it.
  /// @throws std::out_of_range when the variable is not in the solution.
  [[nodiscard]] MQT_SCPD_MILP_EXPORT double valueOf(Var variable) const;
  /// Whether a binary variable is set. A solver returns 0.9999999 for a one.
  /// @throws std::out_of_range when the variable is not in the solution.
  [[nodiscard]] MQT_SCPD_MILP_EXPORT bool isSet(Var variable) const;
};

/// The name of a status, for a message or a metric.
[[nodiscard]] MQT_SCPD_MILP_EXPORT std::string_view statusName(SolveStatus status);

} // namespace mqt::scpd::milp
