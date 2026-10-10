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

#include "mqt-scpd/milp/Model.hpp"
#include "mqt-scpd/milp/mqt_scpd_milp_export.hpp"

#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace mqt::scpd::milp {

/// One state of a branch-and-bound search, as a backend reports it while it
/// solves.
struct SolveProgress {
  /// The objective of the best solution found so far, or NaN before the first.
  double objective = std::numeric_limits<double>::quiet_NaN();
  /// The best bound proved so far, or NaN when the backend has none yet.
  double bound = std::numeric_limits<double>::quiet_NaN();
  /// The relative gap between the two, or NaN when either is missing.
  double gap = std::numeric_limits<double>::quiet_NaN();
  /// The nodes explored so far.
  std::uint64_t nodes = 0;
  /// The seconds since the solve began.
  double seconds = 0.0;
};

/**
 * @brief Where a backend reports while it solves.
 *
 * Both members may be empty. A backend never writes to the process output: it
 * writes its own log to @c log when that is set and nowhere when it is not.
 * An exception thrown from either member ends the solve at the next chance;
 * the backend throws it again once the solver has returned.
 */
struct SolveObserver {
  /// Receives one line of the log of the backend, without the line break.
  std::function<void(std::string_view)> log;
  /// Receives the state of a mixed-integer search, now and then.
  std::function<void(const SolveProgress&)> progress;
};

/// What a solve may spend, and where it reports.
struct SolveOptions {
  /// Seconds of wall clock, or zero for no limit.
  double timeLimit = 0.0;
  /// The relative gap at which a solve stops early, or zero for the default of
  /// the backend.
  double relativeGap = 0.0;
  /// The threads the backend may use, or zero for its own default.
  std::uint32_t threads = 0;
  /// Where the backend reports while it solves.
  SolveObserver observer;
};

/**
 * @brief A solver behind the model abstraction.
 *
 * The interface is what MPS carries. A backend that solves in the process and
 * one that hands the model to another program see the same model and answer
 * the same way.
 */
class MQT_SCPD_MILP_EXPORT ISolverBackend {
public:
  /**
   * @brief Creates a backend.
   */
  ISolverBackend() = default;
  ISolverBackend(const ISolverBackend&) = delete;
  ISolverBackend& operator=(const ISolverBackend&) = delete;
  ISolverBackend(ISolverBackend&&) = delete;
  ISolverBackend& operator=(ISolverBackend&&) = delete;
  virtual ~ISolverBackend() = default;

  /**
   * @brief Returns the name of the backend.
   * @return The name, as a run reports it.
   */
  [[nodiscard]] virtual std::string_view name() const = 0;

  /**
   * @brief Solves a model.
   * @param model The model.
   * @param options What the solve may spend, and where it reports.
   * @return The solution. A model that cannot be solved is a status, never an
   * exception.
   * @throws Whatever an observer of @p options throws.
   */
  [[nodiscard]] virtual Solution solve(const Model& model,
                                       const SolveOptions& options) const = 0;
};

/**
 * @brief Creates the backend that links HiGHS into the process.
 * @return The backend. It is always available.
 */
[[nodiscard]] MQT_SCPD_MILP_EXPORT std::unique_ptr<ISolverBackend>
makeHighsBackend();

/**
 * @brief Returns the version of the HiGHS library that the process links.
 * @return The version, as three numbers joined by dots, such as "1.11.0".
 */
[[nodiscard]] MQT_SCPD_MILP_EXPORT std::string highsVersion();

/**
 * @brief Solves a model handed over as MPS text and returns the values by
 * variable name.
 *
 * This is the seam of the bring-your-own-licence path: Python receives the MPS
 * text, solves it with gurobipy and returns the values it read back. The core
 * neither knows nor links that solver.
 */
using MpsSolveFunction = std::function<Solution(
    std::string_view mpsText, const std::vector<std::string>& variableNames,
    const SolveOptions& options)>;

/**
 * @brief Creates a backend that round-trips a model through MPS text.
 * @param name The name of the backend.
 * @param solve The function that solves the MPS text.
 * @return The backend.
 * @throws std::invalid_argument When @p solve is empty.
 */
[[nodiscard]] MQT_SCPD_MILP_EXPORT std::unique_ptr<ISolverBackend>
makeMpsBackend(std::string name, MpsSolveFunction solve);

/// Which backend a run uses.
enum class BackendChoice : std::uint8_t {
  /// The registered external backend when there is one, else HiGHS.
  Auto,
  /// HiGHS, whatever is registered.
  Highs,
  /// The registered external backend, and an error when there is none.
  External,
};

/**
 * @brief Reads the backend a name stands for.
 * @param name One of "auto", "highs" and "gurobi".
 * @return The choice.
 * @throws std::invalid_argument On any other name.
 */
[[nodiscard]] MQT_SCPD_MILP_EXPORT BackendChoice
backendChoiceFromName(std::string_view name);

/**
 * @brief Registers the external backend of the process, or clears it.
 *
 * The Python layer installs the gurobipy backend here when it can be used.
 *
 * @param backend The backend, or a null pointer to clear it.
 */
MQT_SCPD_MILP_EXPORT void
setExternalBackend(std::shared_ptr<ISolverBackend> backend);

/**
 * @brief Returns the external backend of the process.
 * @return The backend, or a null pointer when none is registered.
 */
[[nodiscard]] MQT_SCPD_MILP_EXPORT std::shared_ptr<ISolverBackend>
externalBackend();

/**
 * @brief Resolves a choice to a backend.
 * @param choice The choice.
 * @return The backend.
 * @throws std::runtime_error When @c External is chosen and no external
 * backend is registered.
 */
[[nodiscard]] MQT_SCPD_MILP_EXPORT std::shared_ptr<ISolverBackend>
resolveBackend(BackendChoice choice);

/**
 * @brief Solves a model with the backend a choice resolves to.
 * @param model The model.
 * @param choice The choice.
 * @param options What the solve may spend, and where it reports.
 * @return The solution.
 * @throws std::runtime_error When @c External is chosen and no external
 * backend is registered.
 */
[[nodiscard]] MQT_SCPD_MILP_EXPORT Solution solve(
    const Model& model, BackendChoice choice, const SolveOptions& options = {});

} // namespace mqt::scpd::milp
