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
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace mqt::scpd::milp {

/// What a solve may spend.
struct SolveOptions {
  /// Seconds of wall clock, or zero for no limit.
  double timeLimit = 0.0;
  /// The relative gap at which a solve stops early, or zero for the
  /// backend's own default.
  double relativeGap = 0.0;
  /// Whether the backend writes its own progress to the process output. Off,
  /// because the core does not write to stdout.
  bool verbose = false;
  /// Threads the backend may use, or zero for its own default.
  std::uint32_t threads = 0;
};

/// A solver behind the model abstraction.
///
/// The interface is what MPS carries, which is the whole point: a backend
/// that solves in process and one that hands the model to another program
/// see the same model and answer the same way.
class MQT_SCPD_MILP_EXPORT ISolverBackend {
public:
  ISolverBackend() = default;
  ISolverBackend(const ISolverBackend&) = delete;
  ISolverBackend& operator=(const ISolverBackend&) = delete;
  ISolverBackend(ISolverBackend&&) = delete;
  ISolverBackend& operator=(ISolverBackend&&) = delete;
  virtual ~ISolverBackend() = default;

  /// What the backend calls itself in the metrics.
  [[nodiscard]] virtual std::string_view name() const = 0;
  /// Solve a model. Never throws on an unsolvable model; the status says so.
  [[nodiscard]] virtual Solution solve(const Model& model, const SolveOptions& options) const = 0;
};

/// The backend that links HiGHS into the process. Always available.
[[nodiscard]] MQT_SCPD_MILP_EXPORT std::unique_ptr<ISolverBackend> makeHighsBackend();

/// A backend that hands the model to a callable as MPS text and reads the
/// solution back by variable name.
///
/// This is the seam the bring-your-own-licence path goes through: Python
/// receives the MPS text, solves it with `gurobipy`, and returns the values
/// it read back. The core neither knows nor links that solver.
using MpsSolveFunction = std::function<Solution(std::string_view mpsText,
                                                const std::vector<std::string>& variableNames,
                                                const SolveOptions& options)>;

/// The backend that round-trips a model through MPS text.
///
/// @throws std::invalid_argument when the function is empty.
[[nodiscard]] MQT_SCPD_MILP_EXPORT std::unique_ptr<ISolverBackend>
makeMpsBackend(std::string name, MpsSolveFunction solve);

/// Which backend a run uses.
enum class BackendChoice : std::uint8_t {
  /// Prefer an external solver when one was registered, else HiGHS.
  Auto,
  Highs,
  /// The registered external backend, and an error when there is none.
  External,
};

/// The choice a name stands for.
///
/// @throws std::invalid_argument on a name that is none of "auto", "highs"
/// and "gurobi".
[[nodiscard]] MQT_SCPD_MILP_EXPORT BackendChoice backendChoiceFromName(std::string_view name);

/// Register the external backend of this process, or clear it with a null
/// pointer. Python installs the `gurobipy` backend here at import time.
MQT_SCPD_MILP_EXPORT void setExternalBackend(std::shared_ptr<ISolverBackend> backend);

/// The external backend of this process, or nothing when none is registered.
[[nodiscard]] MQT_SCPD_MILP_EXPORT std::shared_ptr<ISolverBackend> externalBackend();

/// The backend a choice resolves to.
///
/// @throws std::runtime_error when External is asked for and no external
/// backend is registered.
[[nodiscard]] MQT_SCPD_MILP_EXPORT std::shared_ptr<ISolverBackend> resolveBackend(BackendChoice choice);

/// Solve a model with the backend a choice resolves to.
[[nodiscard]] MQT_SCPD_MILP_EXPORT Solution solve(const Model& model, BackendChoice choice,
                                                  const SolveOptions& options = {});

} // namespace mqt::scpd::milp
