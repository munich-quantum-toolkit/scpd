/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "Highs.h"
#include "lp_data/HConst.h"
#include "lp_data/HighsCallback.h"
#include "lp_data/HighsStatus.h"
#include "model/HighsModel.h"
#include "mqt-scpd/milp/Backend.hpp"
#include "mqt-scpd/milp/Model.hpp"
#include "util/HighsInt.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <format>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace mqt::scpd::milp {
namespace {

/// The status a HiGHS model status maps to.
SolveStatus statusOf(const HighsModelStatus status) {
  switch (status) {
  case HighsModelStatus::kOptimal:
    return SolveStatus::Optimal;
  case HighsModelStatus::kInfeasible:
    return SolveStatus::Infeasible;
  case HighsModelStatus::kUnbounded:
  case HighsModelStatus::kUnboundedOrInfeasible:
    return SolveStatus::Unbounded;
  case HighsModelStatus::kTimeLimit:
  case HighsModelStatus::kIterationLimit:
  case HighsModelStatus::kSolutionLimit:
  case HighsModelStatus::kObjectiveBound:
  case HighsModelStatus::kObjectiveTarget:
  case HighsModelStatus::kInterrupt:
    // A limit stopped the search. Whether a solution exists is decided by
    // the caller, from whether values came back.
    return SolveStatus::Feasible;
  default:
    return SolveStatus::Error;
  }
}

/// A number HiGHS reports, or NaN where it reports infinity for "none yet".
double finiteOrNan(const double value) {
  return std::isfinite(value) ? value
                              : std::numeric_limits<double>::quiet_NaN();
}

/// Hands each line of a log message to the observer. HiGHS passes a message
/// that ends in a line break and may hold several lines.
void logLines(const SolveObserver& observer, const std::string& message) {
  std::string_view rest(message);
  while (!rest.empty()) {
    const auto end = rest.find('\n');
    auto line = rest.substr(0, end);
    if (!line.empty() && line.back() == '\r') {
      line.remove_suffix(1);
    }
    if (!line.empty()) {
      observer.log(line);
    }
    if (end == std::string_view::npos) {
      break;
    }
    rest.remove_prefix(end + 1);
  }
}

/// The variable type a HiGHS integrality flag stands for.
HighsVarType integralityOf(const VarType type) {
  return type == VarType::Continuous ? HighsVarType::kContinuous
                                     : HighsVarType::kInteger;
}

/// HiGHS, linked into the process.
class HighsBackend final : public ISolverBackend {
public:
  [[nodiscard]] std::string_view name() const override { return "highs"; }

  [[nodiscard]] Solution solve(const Model& model,
                               const SolveOptions& options) const override {
    Solution solution;
    solution.backend = "highs";

    HighsModel highsModel;
    auto& lp = highsModel.lp_;
    lp.num_col_ = static_cast<HighsInt>(model.variableCount());
    lp.num_row_ = static_cast<HighsInt>(model.constraintCount());
    lp.sense_ = model.sense() == Sense::Maximize ? ObjSense::kMaximize
                                                 : ObjSense::kMinimize;
    lp.offset_ = model.objectiveOffset();

    lp.col_cost_.reserve(model.variableCount());
    lp.col_lower_.reserve(model.variableCount());
    lp.col_upper_.reserve(model.variableCount());
    bool integral = false;
    for (const auto& variable : model.variables()) {
      lp.col_cost_.push_back(variable.objective);
      lp.col_lower_.push_back(
          variable.lower <= -INFINITE_BOUND ? -kHighsInf : variable.lower);
      lp.col_upper_.push_back(
          variable.upper >= INFINITE_BOUND ? kHighsInf : variable.upper);
      integral = integral || variable.type != VarType::Continuous;
    }
    if (integral) {
      lp.integrality_.reserve(model.variableCount());
      for (const auto& variable : model.variables()) {
        lp.integrality_.push_back(integralityOf(variable.type));
      }
    }

    lp.row_lower_.reserve(model.constraintCount());
    lp.row_upper_.reserve(model.constraintCount());
    for (const auto& constraint : model.constraints()) {
      lp.row_lower_.push_back(
          constraint.lower <= -INFINITE_BOUND ? -kHighsInf : constraint.lower);
      lp.row_upper_.push_back(
          constraint.upper >= INFINITE_BOUND ? kHighsInf : constraint.upper);
    }

    // The model holds its coefficients by row; HiGHS is handed them by
    // column. Its own row-wise path assumes a column-wise start vector in
    // places and reads such a matrix as empty, which is a constraint that
    // silently does not bind rather than an error. Transposing once here is
    // one pass over the nonzeros and leaves nothing to assume.
    std::vector<HighsInt> counts(model.variableCount() + 1, 0);
    for (const auto& constraint : model.constraints()) {
      for (const auto& [column, _] : constraint.coefficients) {
        ++counts[column + 1];
      }
    }
    lp.a_matrix_.format_ = MatrixFormat::kColwise;
    lp.a_matrix_.num_col_ = lp.num_col_;
    lp.a_matrix_.num_row_ = lp.num_row_;
    lp.a_matrix_.start_.assign(counts.begin(), counts.end());
    for (std::size_t column = 1; column < lp.a_matrix_.start_.size();
         ++column) {
      lp.a_matrix_.start_[column] += lp.a_matrix_.start_[column - 1];
    }
    const auto nonZeros = static_cast<std::size_t>(lp.a_matrix_.start_.back());
    lp.a_matrix_.index_.resize(nonZeros);
    lp.a_matrix_.value_.resize(nonZeros);
    auto next = lp.a_matrix_.start_;
    for (std::size_t row = 0; row < model.constraints().size(); ++row) {
      for (const auto& [column, coefficient] :
           model.constraints()[row].coefficients) {
        const auto slot = static_cast<std::size_t>(next[column]++);
        lp.a_matrix_.index_[slot] = static_cast<HighsInt>(row);
        lp.a_matrix_.value_[slot] = coefficient;
      }
    }

    Highs highs;
    // The core never writes to the process output. HiGHS logs only when an
    // observer asks for the log, and then only into the callback: with a
    // logging callback active it writes nothing to the console.
    const auto& observer = options.observer;
    const bool logs = static_cast<bool>(observer.log);
    if (highs.setOptionValue("output_flag", logs) != HighsStatus::kOk ||
        highs.setOptionValue("log_to_console", logs) != HighsStatus::kOk) {
      solution.message = "HiGHS refused its own output option";
      return solution;
    }

    // An exception from an observer must not unwind through HiGHS. It is kept,
    // the solve is asked to stop, and the exception leaves after run().
    std::exception_ptr failure;
    if (logs || observer.progress) {
      static_cast<void>(highs.setCallback(
          [&observer, &failure](const int type, const std::string& message,
                                const HighsCallbackOutput* out,
                                HighsCallbackInput* in, void* /*data*/) {
            if (failure != nullptr) {
              if (in != nullptr) {
                in->user_interrupt = true;
              }
              return;
            }
            try {
              if (type == kCallbackLogging && observer.log) {
                logLines(observer, message);
              } else if (type == kCallbackMipInterrupt && observer.progress &&
                         out != nullptr) {
                observer.progress(
                    {.objective = finiteOrNan(out->mip_primal_bound),
                     .bound = finiteOrNan(out->mip_dual_bound),
                     .gap = finiteOrNan(out->mip_gap),
                     .nodes = static_cast<std::uint64_t>(
                         std::max<std::int64_t>(out->mip_node_count, 0)),
                     .seconds = out->running_time});
              }
            } catch (...) {
              failure = std::current_exception();
              if (in != nullptr) {
                in->user_interrupt = true;
              }
            }
          }));
      if (logs) {
        static_cast<void>(highs.startCallback(kCallbackLogging));
      }
      if (observer.progress) {
        static_cast<void>(highs.startCallback(kCallbackMipInterrupt));
      }
    }
    if (options.timeLimit > 0.0) {
      static_cast<void>(highs.setOptionValue("time_limit", options.timeLimit));
    }
    if (options.relativeGap > 0.0) {
      static_cast<void>(
          highs.setOptionValue("mip_rel_gap", options.relativeGap));
    }
    if (options.threads > 0) {
      static_cast<void>(highs.setOptionValue(
          "threads", static_cast<HighsInt>(options.threads)));
    }

    if (highs.passModel(highsModel) != HighsStatus::kOk) {
      solution.message = "HiGHS rejected the model";
      return solution;
    }
    const auto status = highs.run();
    if (failure != nullptr) {
      std::rethrow_exception(failure);
    }
    if (status == HighsStatus::kError) {
      solution.message = "HiGHS failed while solving";
      return solution;
    }

    solution.status = statusOf(highs.getModelStatus());
    if (solution.status == SolveStatus::Error) {
      solution.message =
          std::format("HiGHS ended as {}",
                      highs.modelStatusToString(highs.getModelStatus()));
      return solution;
    }
    if (solution.status == SolveStatus::Feasible &&
        !highs.getSolution().value_valid) {
      // A limit was reached before any solution was found, which is not a
      // feasible answer however the status reads.
      solution.status = SolveStatus::Error;
      solution.message = "HiGHS stopped at a limit without a solution";
      return solution;
    }
    if (!solution.hasValues()) {
      return solution;
    }

    solution.objective = highs.getInfo().objective_function_value;
    if (integral) {
      const auto& info = highs.getInfo();
      solution.bound = finiteOrNan(info.mip_dual_bound);
      solution.gap = finiteOrNan(info.mip_gap);
      solution.nodes = static_cast<std::uint64_t>(
          std::max<std::int64_t>(info.mip_node_count, 0));
    }
    const auto& values = highs.getSolution().col_value;
    solution.values.assign(values.begin(), values.end());
    return solution;
  }
};

} // namespace

std::unique_ptr<ISolverBackend> makeHighsBackend() {
  return std::make_unique<HighsBackend>();
}

std::string highsVersion() {
  return std::format("{}.{}.{}", highsVersionMajor(), highsVersionMinor(),
                     highsVersionPatch());
}

} // namespace mqt::scpd::milp
