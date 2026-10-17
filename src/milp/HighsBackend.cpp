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
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <format>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
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

/// The thread count that the last solve on this thread asked HiGHS for, or
/// none before the first solve. HiGHS keeps one task scheduler per calling
/// thread and refuses a solve whose thread count differs from the count the
/// scheduler started with.
std::optional<std::uint32_t>& schedulerThreads() {
  thread_local std::optional<std::uint32_t> threads;
  return threads;
}

/// Restarts the HiGHS scheduler of the calling thread when a solve asks for
/// another thread count than the solve before. Zero asks for the default of
/// HiGHS, which the restarted scheduler then takes.
void prepareScheduler(const std::uint32_t threads) {
  auto& last = schedulerThreads();
  if (last != threads) {
    Highs::resetGlobalScheduler(true);
    last = threads;
  }
}

/// How far a row may miss its bounds and still hold. It is the default primal
/// feasibility tolerance of HiGHS.
constexpr double FEASIBILITY_TOLERANCE = 1.0e-7;

/// The least wall-clock time between two calls of SolveObserver::check. A
/// call into Python costs next to nothing at this rate. A pending interrupt
/// waits this long and then until HiGHS next calls back, which it does every
/// few iterations.
constexpr std::chrono::milliseconds CHECK_INTERVAL{100};

/// The answer to a model without variables. HiGHS calls such a model empty
/// and solves nothing. A row of it has no terms, because the constant of its
/// expression moved into its bounds, so the value of the row is zero. The
/// model is feasible exactly when every row admits zero, and its objective is
/// the offset.
Solution solveWithoutVariables(const Model& model) {
  Solution solution;
  solution.backend = "highs";
  const auto admitsZero = [](const Constraint& constraint) {
    return constraint.lower <= FEASIBILITY_TOLERANCE &&
           constraint.upper >= -FEASIBILITY_TOLERANCE;
  };
  if (!std::ranges::all_of(model.constraints(), admitsZero)) {
    solution.status = SolveStatus::Infeasible;
    return solution;
  }
  solution.status = SolveStatus::Optimal;
  solution.objective = model.objectiveOffset();
  return solution;
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
    if (model.variableCount() == 0) {
      return solveWithoutVariables(model);
    }

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
    // HiGHS counts a bound as infinite from its own option on, drops a
    // coefficient up to its smallest matrix value, and refuses one from its
    // largest. The options follow the model, so HiGHS reads the model that
    // the MPS text carries to every other backend: by default, it would
    // refuse coefficients from 1e15. The smallest matrix value stays at the
    // default of HiGHS: a smaller one changes the assignments of the benchmark
    // chips.
    for (const auto& [option, value] :
         {std::pair{"infinite_bound", INFINITE_BOUND},
          std::pair{"small_matrix_value", COEFFICIENT_EPSILON},
          std::pair{"large_matrix_value", INFINITE_BOUND}}) {
      if (highs.setOptionValue(option, value) != HighsStatus::kOk) {
        solution.message = std::format("HiGHS refused its option {}", option);
        return solution;
      }
    }

    // An exception from an observer must not unwind through HiGHS. It is kept,
    // the solve is asked to stop, and the exception leaves after run().
    std::exception_ptr failure;
    // The simplex and interior point solvers call back on every iteration,
    // so the check runs at most once per CHECK_INTERVAL. HiGHS reports no
    // running time to these callbacks, so the backend reads its own clock.
    std::optional<std::chrono::steady_clock::time_point> checked;
    if (logs || observer.progress || observer.check) {
      static_cast<void>(
          highs.setCallback([&observer, &failure, &checked](
                                const int type, const std::string& message,
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
              } else if ((type == kCallbackSimplexInterrupt ||
                          type == kCallbackIpmInterrupt) &&
                         observer.check) {
                const auto now = std::chrono::steady_clock::now();
                if (!checked.has_value() || now - *checked >= CHECK_INTERVAL) {
                  checked = now;
                  observer.check();
                }
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
      // The logging callback has no input to stop the solve through. The
      // interrupt callbacks do, so they run for every observer: an exception
      // from the log then also stops a simplex, interior point or MIP solve.
      static_cast<void>(highs.startCallback(kCallbackSimplexInterrupt));
      static_cast<void>(highs.startCallback(kCallbackIpmInterrupt));
      static_cast<void>(highs.startCallback(kCallbackMipInterrupt));
    }
    // HiGHS refuses a value outside the range of an option, and a solve
    // without the limit, the gap or the threads it was asked for is not the
    // solve that was asked for.
    if (options.timeLimit > 0.0 &&
        highs.setOptionValue("time_limit", options.timeLimit) !=
            HighsStatus::kOk) {
      solution.message =
          std::format("HiGHS refused a time limit of {}s", options.timeLimit);
      return solution;
    }
    if (options.relativeGap > 0.0 &&
        highs.setOptionValue("mip_rel_gap", options.relativeGap) !=
            HighsStatus::kOk) {
      solution.message = std::format("HiGHS refused a relative gap of {}",
                                     options.relativeGap);
      return solution;
    }
    if (options.threads > 0 &&
        (options.threads >
             static_cast<std::uint32_t>(std::numeric_limits<HighsInt>::max()) ||
         highs.setOptionValue("threads",
                              static_cast<HighsInt>(options.threads)) !=
             HighsStatus::kOk)) {
      solution.message =
          std::format("HiGHS refused {} threads", options.threads);
      return solution;
    }
    prepareScheduler(options.threads);

    // HiGHS warns about a model it accepts, for example about the range of
    // its coefficients, and then solves it. Only an error is a refusal.
    if (highs.passModel(highsModel) == HighsStatus::kError) {
      // HiGHS logs why it rejects the model, and the observer of that log
      // may have thrown.
      if (failure != nullptr) {
        std::rethrow_exception(failure);
      }
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
    if (highs.getModelStatus() == HighsModelStatus::kUnboundedOrInfeasible) {
      // HiGHS reports this for a MIP whose presolve or relaxation shows no
      // optimum. Either answer would be a guess.
      solution.message = "HiGHS found that the model is infeasible or "
                         "unbounded, and not which of the two";
      return solution;
    }
    if (solution.status == SolveStatus::Error) {
      solution.message =
          std::format("HiGHS ended as {}",
                      highs.modelStatusToString(highs.getModelStatus()));
      return solution;
    }
    if (solution.status == SolveStatus::Feasible &&
        (!highs.getSolution().value_valid ||
         highs.getInfo().primal_solution_status != kSolutionStatusFeasible)) {
      // A limit was reached before a feasible point was found. HiGHS may still
      // hand back values, such as the start of a simplex solve, but they break
      // rows, so they are not a feasible answer however the status reads.
      solution.status = SolveStatus::Error;
      solution.message = "HiGHS stopped at a limit without a feasible solution";
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
