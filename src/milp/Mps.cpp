/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/milp/Mps.hpp"

#include "mqt-scpd/milp/Model.hpp"

#include <cmath>
#include <cstddef>
#include <format>
#include <string>
#include <vector>

namespace mqt::scpd::milp {
namespace {

/// The name of the objective row. It is a row like any other in MPS, marked
/// free, and no model may take the name.
constexpr const char* OBJECTIVE_ROW = "COST";

/// A number with enough digits that a round trip through the text does not
/// change it.
std::string number(const double value) { return std::format("{:.17g}", value); }

/// Whether a bound is one MPS has to state. An infinite bound is left out,
/// and the reader applies its own default.
bool isFinite(const double bound) { return std::abs(bound) < INFINITE_BOUND; }

/// The column-major coefficients: for each variable, the rows it appears in.
std::vector<std::vector<std::pair<std::size_t, double>>> byColumn(const Model& model) {
  std::vector<std::vector<std::pair<std::size_t, double>>> columns(model.variableCount());
  for (std::size_t row = 0; row < model.constraints().size(); ++row) {
    for (const auto& [column, coefficient] : model.constraints()[row].coefficients) {
      columns[column].emplace_back(row, coefficient);
    }
  }
  return columns;
}

} // namespace

std::string toMps(const Model& model) {
  // MPS states a minimization, so a maximization is written with every
  // objective coefficient negated and the sign undone when the value is read
  // back.
  const double objectiveSign = model.sense() == Sense::Maximize ? -1.0 : 1.0;

  std::string text;
  text += std::format("NAME          {}\n", model.name());

  text += "ROWS\n";
  text += std::format(" N  {}\n", OBJECTIVE_ROW);
  for (const auto& constraint : model.constraints()) {
    // An equality has both bounds equal; a range has two finite bounds and
    // its second bound goes to the RANGES section.
    if (constraint.lower == constraint.upper) {
      text += std::format(" E  {}\n", constraint.name);
    } else if (isFinite(constraint.upper)) {
      text += std::format(" L  {}\n", constraint.name);
    } else if (isFinite(constraint.lower)) {
      text += std::format(" G  {}\n", constraint.name);
    } else {
      text += std::format(" N  {}\n", constraint.name);
    }
  }

  const auto columns = byColumn(model);
  text += "COLUMNS\n";
  bool integerBlockOpen = false;
  std::size_t markerCount = 0;
  for (std::size_t column = 0; column < model.variableCount(); ++column) {
    const auto& variable = model.variables()[column];
    const bool integral = variable.type != VarType::Continuous;
    if (integral != integerBlockOpen) {
      text += std::format("    MARKER{:<4}  'MARKER'                 '{}'\n", markerCount++,
                          integral ? "INTORG" : "INTEND");
      integerBlockOpen = integral;
    }
    if (variable.objective != 0.0) {
      text += std::format("    {}  {}  {}\n", variable.name, OBJECTIVE_ROW,
                          number(objectiveSign * variable.objective));
    }
    for (const auto& [row, coefficient] : columns[column]) {
      text += std::format("    {}  {}  {}\n", variable.name, model.constraints()[row].name,
                          number(coefficient));
    }
    // A variable that appears nowhere still has to exist, or the reader will
    // not know its bounds and its type.
    if (variable.objective == 0.0 && columns[column].empty()) {
      text += std::format("    {}  {}  0\n", variable.name, OBJECTIVE_ROW);
    }
  }
  if (integerBlockOpen) {
    text += std::format("    MARKER{:<4}  'MARKER'                 'INTEND'\n", markerCount);
  }

  text += "RHS\n";
  if (model.objectiveOffset() != 0.0) {
    // A solver reads the objective row's right-hand side as the negated
    // constant of the objective.
    text += std::format("    RHS  {}  {}\n", OBJECTIVE_ROW,
                        number(-objectiveSign * model.objectiveOffset()));
  }
  for (const auto& constraint : model.constraints()) {
    if (constraint.lower == constraint.upper) {
      text += std::format("    RHS  {}  {}\n", constraint.name, number(constraint.lower));
    } else if (isFinite(constraint.upper)) {
      text += std::format("    RHS  {}  {}\n", constraint.name, number(constraint.upper));
    } else if (isFinite(constraint.lower)) {
      text += std::format("    RHS  {}  {}\n", constraint.name, number(constraint.lower));
    }
  }

  // A row with two finite bounds is written as its tighter side plus a
  // range, which is how MPS expresses `lower <= row <= upper`.
  std::string ranges;
  for (const auto& constraint : model.constraints()) {
    if (constraint.lower != constraint.upper && isFinite(constraint.lower) &&
        isFinite(constraint.upper)) {
      ranges += std::format("    RNG  {}  {}\n", constraint.name,
                            number(constraint.upper - constraint.lower));
    }
  }
  if (!ranges.empty()) {
    text += "RANGES\n";
    text += ranges;
  }

  text += "BOUNDS\n";
  for (const auto& variable : model.variables()) {
    const bool defaultLower = variable.lower == 0.0;
    const bool defaultUpper = !isFinite(variable.upper);
    if (variable.type == VarType::Binary && variable.lower == 0.0 && variable.upper == 1.0) {
      text += std::format(" BV BND  {}\n", variable.name);
      continue;
    }
    if (defaultLower && defaultUpper) {
      // An integer variable with no upper bound would otherwise be read as
      // bounded by one, which is what the MPS integer marker implies.
      if (variable.type != VarType::Continuous) {
        text += std::format(" PL BND  {}\n", variable.name);
      }
      continue;
    }
    if (!isFinite(variable.lower) && defaultUpper) {
      text += std::format(" FR BND  {}\n", variable.name);
      continue;
    }
    if (!defaultLower) {
      if (isFinite(variable.lower)) {
        text += std::format(" LO BND  {}  {}\n", variable.name, number(variable.lower));
      } else {
        text += std::format(" MI BND  {}\n", variable.name);
      }
    }
    if (!defaultUpper) {
      text += std::format(" UP BND  {}  {}\n", variable.name, number(variable.upper));
    }
  }

  text += "ENDATA\n";
  return text;
}

} // namespace mqt::scpd::milp
