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

/// A bound that does not constrain. MPS calls this value infinite, and every
/// backend reads it as free.
inline constexpr double INFINITE_BOUND = 1.0e30;

/// Whether a variable ranges over the reals or the integers.
enum class VarType : std::uint8_t {
  /// A real number between its bounds.
  Continuous,
  /// A whole number between its bounds.
  Integer,
  /// Zero or one.
  Binary,
};

/// Which way the objective is optimized.
enum class Sense : std::uint8_t {
  /// The solver looks for the smallest objective.
  Minimize,
  /// The solver looks for the largest objective.
  Maximize,
};

/// A variable of a model, as an index into its variable list.
struct Var {
  /// The position of the variable in the variable list of its model.
  std::size_t index = 0;

  /**
   * @brief Compares two variables by their index.
   * @return @c true when both name the same position.
   */
  [[nodiscard]] bool operator==(const Var&) const = default;
};

/**
 * @brief A weighted sum of variables plus a constant.
 *
 * The expression keeps its terms in the order they were added and does not
 * combine two terms on the same variable. The terms are combined once, when a
 * row or the objective takes the expression, so that building an expression
 * stays cheap in the loops that build one.
 */
class MQT_SCPD_MILP_EXPORT LinearExpr {
public:
  /**
   * @brief Creates the expression zero.
   */
  LinearExpr() = default;

  /**
   * @brief Creates the expression that is one variable.
   * @param variable The variable, with coefficient one.
   */
  LinearExpr(Var variable) // NOLINT(google-explicit-constructor)
      : termList{{variable.index, 1.0}} {}

  /**
   * @brief Creates the expression that is one constant.
   * @param constant The constant.
   */
  LinearExpr(double constant) // NOLINT(google-explicit-constructor)
      : constantTerm(constant) {}

  /**
   * @brief Adds a weighted variable.
   * @param variable The variable.
   * @param coefficient Its weight.
   * @return This expression.
   */
  LinearExpr& add(Var variable, double coefficient);

  /**
   * @brief Adds a constant.
   * @param constant The constant.
   * @return This expression.
   */
  LinearExpr& add(double constant);

  /**
   * @brief Adds every term and the constant of another expression.
   * @param other The expression to add.
   * @return This expression.
   */
  LinearExpr& add(const LinearExpr& other);

  /**
   * @brief Adds another expression.
   * @param other The expression to add.
   * @return This expression.
   */
  LinearExpr& operator+=(const LinearExpr& other) { return add(other); }

  /**
   * @brief Subtracts another expression.
   * @param other The expression to subtract.
   * @return This expression.
   */
  LinearExpr& operator-=(const LinearExpr& other);

  /**
   * @brief Multiplies every term and the constant by a number.
   *
   * There is no product of two expressions, because that would not be a
   * linear model.
   *
   * @param factor The number.
   * @return This expression.
   */
  LinearExpr& operator*=(double factor);

  /**
   * @brief Returns the terms in the order they were added.
   * @return The terms, as (variable index, coefficient) pairs.
   */
  [[nodiscard]] const std::vector<std::pair<std::size_t, double>>&
  terms() const {
    return termList;
  }

  /**
   * @brief Returns the constant.
   * @return The sum of every constant added.
   */
  [[nodiscard]] double constant() const { return constantTerm; }

  /**
   * @brief Combines the terms of each variable into one.
   * @return The terms with every repeated variable combined, sorted by
   * variable index, without the terms whose coefficient is zero.
   */
  [[nodiscard]] std::vector<std::pair<std::size_t, double>> combined() const;

private:
  std::vector<std::pair<std::size_t, double>> termList;
  double constantTerm = 0.0;
};

/**
 * @brief Weights a variable.
 * @param coefficient The weight.
 * @param variable The variable.
 * @return The expression @p coefficient times @p variable.
 */
[[nodiscard]] MQT_SCPD_MILP_EXPORT LinearExpr operator*(double coefficient,
                                                        Var variable);

/**
 * @brief Weights a variable.
 * @param variable The variable.
 * @param coefficient The weight.
 * @return The expression @p coefficient times @p variable.
 */
[[nodiscard]] MQT_SCPD_MILP_EXPORT LinearExpr operator*(Var variable,
                                                        double coefficient);

/**
 * @brief Scales an expression.
 * @param factor The number.
 * @param expression The expression.
 * @return The expression with every term and the constant times @p factor.
 */
[[nodiscard]] MQT_SCPD_MILP_EXPORT LinearExpr operator*(double factor,
                                                        LinearExpr expression);

/**
 * @brief Scales an expression.
 * @param expression The expression.
 * @param factor The number.
 * @return The expression with every term and the constant times @p factor.
 */
[[nodiscard]] MQT_SCPD_MILP_EXPORT LinearExpr operator*(LinearExpr expression,
                                                        double factor);

/**
 * @brief Adds two expressions.
 * @param left The first expression.
 * @param right The second expression.
 * @return The sum.
 */
[[nodiscard]] MQT_SCPD_MILP_EXPORT LinearExpr
operator+(LinearExpr left, const LinearExpr& right);

/**
 * @brief Subtracts one expression from another.
 * @param left The expression to subtract from.
 * @param right The expression to subtract.
 * @return The difference.
 */
[[nodiscard]] MQT_SCPD_MILP_EXPORT LinearExpr
operator-(LinearExpr left, const LinearExpr& right);

/**
 * @brief Negates an expression.
 * @param expression The expression.
 * @return The expression with every term and the constant negated.
 */
[[nodiscard]] MQT_SCPD_MILP_EXPORT LinearExpr
operator-(const LinearExpr& expression);

/// One variable of a model.
struct Variable {
  /// The name, unique in its model, by which a solution is read back.
  std::string name;
  /// The lower bound, or -INFINITE_BOUND for none.
  double lower = 0.0;
  /// The upper bound, or INFINITE_BOUND for none.
  double upper = INFINITE_BOUND;
  /// Whether the variable is continuous, integral or binary.
  VarType type = VarType::Continuous;
  /// The objective coefficient. It is held on the variable because MPS and the
  /// backends take it that way.
  double objective = 0.0;
};

/**
 * @brief One linear constraint, as lower <= sum(coefficient * variable) <=
 * upper.
 *
 * An equality has both bounds equal, and a one-sided constraint leaves the
 * other bound infinite. The coefficients are stored combined and sorted by
 * variable, so a backend can take them as they are.
 */
struct Constraint {
  /// The name, unique in its model.
  std::string name;
  /// The terms, as (variable index, coefficient) pairs, sorted by index.
  std::vector<std::pair<std::size_t, double>> coefficients;
  /// The lower bound, or -INFINITE_BOUND for none.
  double lower = -INFINITE_BOUND;
  /// The upper bound, or INFINITE_BOUND for none.
  double upper = INFINITE_BOUND;
};

/**
 * @brief A mixed-integer linear program, independent of any solver.
 *
 * The model is all the planning stages see of a solver: variables with
 * bounds and a type, linear constraints and one linear objective. That is
 * what MPS carries, which is what makes the backend linked into the process
 * and a backend reached through MPS interchangeable.
 */
class MQT_SCPD_MILP_EXPORT Model {
public:
  /**
   * @brief Creates an empty model that minimizes.
   * @param name The name of the model, for messages and the MPS file.
   */
  explicit Model(std::string name = "model") : modelName(std::move(name)) {}

  /**
   * @brief Adds a variable.
   *
   * The bounds of a binary variable are clipped to zero and one.
   *
   * @param name The name, unique in the model.
   * @param lower The lower bound.
   * @param upper The upper bound.
   * @param type The type.
   * @param objective The objective coefficient.
   * @return The variable.
   * @throws std::invalid_argument When the name is empty or taken, or when
   * @p lower is above @p upper.
   */
  Var addVariable(std::string_view name, double lower, double upper,
                  VarType type, double objective = 0.0);

  /**
   * @brief Adds a binary variable.
   * @param name The name, unique in the model.
   * @param objective The objective coefficient.
   * @return The variable.
   * @throws std::invalid_argument When the name is empty or taken.
   */
  Var addBinary(std::string_view name, double objective = 0.0);

  /**
   * @brief Adds an integer variable.
   * @param name The name, unique in the model.
   * @param lower The lower bound.
   * @param upper The upper bound.
   * @param objective The objective coefficient.
   * @return The variable.
   * @throws std::invalid_argument When the name is empty or taken, or when
   * @p lower is above @p upper.
   */
  Var addInteger(std::string_view name, double lower, double upper,
                 double objective = 0.0);

  /**
   * @brief Adds a continuous variable.
   * @param name The name, unique in the model.
   * @param lower The lower bound.
   * @param upper The upper bound.
   * @param objective The objective coefficient.
   * @return The variable.
   * @throws std::invalid_argument When the name is empty or taken, or when
   * @p lower is above @p upper.
   */
  Var addContinuous(std::string_view name, double lower, double upper,
                    double objective = 0.0);

  /**
   * @brief Adds the constraint expression <= bound.
   *
   * The constant of the expression moves to the other side, so
   * x + 3 <= 5 is stored as x <= 2.
   *
   * @param name The name, unique in the model.
   * @param expression The left side.
   * @param bound The right side.
   * @throws std::invalid_argument When the name is empty or taken, or when the
   * expression names a variable the model does not have.
   */
  void addLessOrEqual(std::string_view name, const LinearExpr& expression,
                      double bound);

  /**
   * @brief Adds the constraint expression >= bound.
   * @param name The name, unique in the model.
   * @param expression The left side.
   * @param bound The right side.
   * @throws std::invalid_argument When the name is empty or taken, or when the
   * expression names a variable the model does not have.
   */
  void addGreaterOrEqual(std::string_view name, const LinearExpr& expression,
                         double bound);

  /**
   * @brief Adds the constraint expression == bound.
   * @param name The name, unique in the model.
   * @param expression The left side.
   * @param bound The right side.
   * @throws std::invalid_argument When the name is empty or taken, or when the
   * expression names a variable the model does not have.
   */
  void addEqual(std::string_view name, const LinearExpr& expression,
                double bound);

  /**
   * @brief Adds the constraint lower <= expression <= upper.
   * @param name The name, unique in the model.
   * @param expression The middle.
   * @param lower The lower bound.
   * @param upper The upper bound.
   * @throws std::invalid_argument When the name is empty or taken, when the
   * expression names a variable the model does not have, or when the bounds
   * cross.
   */
  void addRange(std::string_view name, const LinearExpr& expression,
                double lower, double upper);

  /**
   * @brief Adds an expression to the objective.
   *
   * Each term adds its coefficient to the objective coefficient of its
   * variable, and the constant joins the objective offset.
   *
   * @param expression The expression to add.
   * @throws std::invalid_argument When the expression names a variable the
   * model does not have.
   */
  void addObjective(const LinearExpr& expression);

  /**
   * @brief Sets which way the objective is optimized.
   * @param sense Minimize or Maximize.
   */
  void setSense(Sense sense) { objectiveSense = sense; }

  /**
   * @brief Returns the name of the model.
   * @return The name.
   */
  [[nodiscard]] const std::string& name() const { return modelName; }

  /**
   * @brief Returns which way the objective is optimized.
   * @return Minimize or Maximize.
   */
  [[nodiscard]] Sense sense() const { return objectiveSense; }

  /**
   * @brief Returns the constant of the objective.
   * @return The sum of the constants added to the objective.
   */
  [[nodiscard]] double objectiveOffset() const { return offset; }

  /**
   * @brief Returns the variables.
   * @return The variables, in the order they were added.
   */
  [[nodiscard]] const std::vector<Variable>& variables() const {
    return variableList;
  }

  /**
   * @brief Returns the constraints.
   * @return The constraints, in the order they were added.
   */
  [[nodiscard]] const std::vector<Constraint>& constraints() const {
    return constraintList;
  }

  /**
   * @brief Counts the variables.
   * @return The number of variables.
   */
  [[nodiscard]] std::size_t variableCount() const {
    return variableList.size();
  }

  /**
   * @brief Counts the constraints.
   * @return The number of constraints.
   */
  [[nodiscard]] std::size_t constraintCount() const {
    return constraintList.size();
  }

  /**
   * @brief Checks whether the model is a mixed-integer program.
   * @return @c true when any variable is integral or binary.
   */
  [[nodiscard]] bool isMixedInteger() const;

  /**
   * @brief Returns the name of a variable.
   * @param variable The variable.
   * @return Its name.
   * @throws std::out_of_range When the variable is not one of this model.
   */
  [[nodiscard]] const std::string& nameOf(Var variable) const;

private:
  /// The row that the four constraint forms share.
  void addRow(std::string_view name, const LinearExpr& expression, double lower,
              double upper);

  std::string modelName;
  Sense objectiveSense = Sense::Minimize;
  double offset = 0.0;
  std::vector<Variable> variableList;
  std::vector<Constraint> constraintList;
  /// Names are unique so that a solution can be read back by name, which is
  /// the one thing the MPS round trip carries across.
  std::unordered_set<std::string> takenNames;
};

/// What a solver made of a model.
enum class SolveStatus : std::uint8_t {
  /// A solution proven optimal.
  Optimal,
  /// A feasible solution that was not proven optimal, usually because a limit
  /// was reached.
  Feasible,
  /// The model has no feasible solution.
  Infeasible,
  /// The objective has no bound.
  Unbounded,
  /// The solve gave no usable answer.
  Error,
};

/// The answer a backend gives.
struct Solution {
  /// What the solver made of the model.
  SolveStatus status = SolveStatus::Error;
  /// The objective value of the solution, offset included, or NaN when there
  /// is no solution.
  double objective = std::numeric_limits<double>::quiet_NaN();
  /// The best bound the solver proved on the objective, or NaN when the
  /// backend does not report one.
  double bound = std::numeric_limits<double>::quiet_NaN();
  /// The relative gap between the objective and the bound, or NaN when the
  /// backend does not report one.
  double gap = std::numeric_limits<double>::quiet_NaN();
  /// The branch-and-bound nodes the solver explored, zero for a linear model.
  std::uint64_t nodes = 0;
  /// One value per variable of the model, in the order of the model. Empty
  /// unless the status is Optimal or Feasible.
  std::vector<double> values;
  /// The name of the backend that solved the model.
  std::string backend;
  /// What went wrong, when the status is Error.
  std::string message;

  /**
   * @brief Checks whether the solution carries values.
   * @return @c true when the status is Optimal or Feasible.
   */
  [[nodiscard]] bool hasValues() const {
    return status == SolveStatus::Optimal || status == SolveStatus::Feasible;
  }

  /**
   * @brief Returns the value of a variable, as the backend returned it.
   * @param variable The variable.
   * @return Its value.
   * @throws std::out_of_range When the solution has no value for the variable.
   */
  [[nodiscard]] MQT_SCPD_MILP_EXPORT double valueOf(Var variable) const;

  /**
   * @brief Checks whether a binary variable is set.
   *
   * A solver may return 0.9999999 for a one, so any value above one half
   * counts as set.
   *
   * @param variable The variable.
   * @return @c true when its value is above one half.
   * @throws std::out_of_range When the solution has no value for the variable.
   */
  [[nodiscard]] MQT_SCPD_MILP_EXPORT bool isSet(Var variable) const;
};

/**
 * @brief Names a status.
 * @param status The status.
 * @return One of "optimal", "feasible", "infeasible", "unbounded" or "error".
 */
[[nodiscard]] MQT_SCPD_MILP_EXPORT std::string_view
statusName(SolveStatus status);

} // namespace mqt::scpd::milp
