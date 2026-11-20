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

#include "mqt-scpd/flatbuffers/artifacts.hpp"
#include "mqt-scpd/flatbuffers/config.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/pipeline/Report.hpp"
#include "mqt-scpd/pipeline/mqt_scpd_pipeline_export.hpp"

namespace mqt::scpd::pipeline {

using flatbuffers::artifacts::AssignmentT;
using flatbuffers::artifacts::CapacityPlanT;
using flatbuffers::artifacts::GlobalRoutingT;
using flatbuffers::config::ConfigT;
using flatbuffers::design::ChipT;

// Every stage has the same shape, and the shape is part of the contract.
// `run` is const and takes its inputs by const reference, so a stage can
// neither change its own state between runs nor reach into the state of
// another. It returns a new value rather than writing into a shared design
// database. Given the same inputs it produces the same output, which is what
// makes a resumed run equal to an uninterrupted one. There are two
// exceptions. A solve that stops at `[stages.solver] time_limit` gives what it
// has found by then, which depends on the speed of the machine. And with
// `[stages.solver] threads` at 0, the thread count depends on the machine, so
// between optimal solutions of equal objective the answer can differ from one
// machine to the next. A stage reports what it does through the Report it is
// given and never prints.

/**
 * @brief Partitions the free space of the chip and budgets how many wires may
 * cross between the partitions.
 */
class MQT_SCPD_PIPELINE_EXPORT ICapacityPlanner {
public:
  /**
   * @brief Creates a planner.
   */
  ICapacityPlanner() = default;
  ICapacityPlanner(const ICapacityPlanner&) = delete;
  ICapacityPlanner& operator=(const ICapacityPlanner&) = delete;
  ICapacityPlanner(ICapacityPlanner&&) = delete;
  ICapacityPlanner& operator=(ICapacityPlanner&&) = delete;
  virtual ~ICapacityPlanner() = default;

  /**
   * @brief Plans the capacity of a chip.
   * @param chip The classified chip.
   * @param config The run configuration.
   * @param report Where the stage reports what it does.
   * @return The capacity plan.
   * @throws std::invalid_argument When the configuration carries no design
   * rules, asks for a grid of fewer than two cells along an axis, a
   * `detail_factor` of zero or a detail grid above MAX_DETAIL_CELLS, or when
   * the chip carries no launcher or puts the targets of two ports on one cell
   * of the detail grid.
   * @throws std::length_error When the partitions need more labels than a
   * plan holds.
   */
  [[nodiscard]] virtual CapacityPlanT
  run(const ChipT& chip, const ConfigT& config, const Report& report) const = 0;
};

/**
 * @brief Solves the inner circuit on the lattice each capacity chain induces,
 * and reports the outer port ring the assignment consumes.
 *
 * The stage runs before the assignment. The inner circuit decides which
 * outer ports the inner wires surface at, and the ring the assignment works
 * on is the configured one extended by exactly those.
 */
class MQT_SCPD_PIPELINE_EXPORT IGlobalRouter {
public:
  /**
   * @brief Creates a router.
   */
  IGlobalRouter() = default;
  IGlobalRouter(const IGlobalRouter&) = delete;
  IGlobalRouter& operator=(const IGlobalRouter&) = delete;
  IGlobalRouter(IGlobalRouter&&) = delete;
  IGlobalRouter& operator=(IGlobalRouter&&) = delete;
  virtual ~IGlobalRouter() = default;

  /**
   * @brief Solves the inner circuit of a chip.
   * @param chip The classified chip.
   * @param capacity The capacity plan of the run.
   * @param config The run configuration.
   * @param report Where the stage reports what it does.
   * @return The global routing.
   * @throws std::invalid_argument When the ring of the configuration does not
   * fit the chip, or when the capacity plan does not fit the chip: an index
   * names no node or no port, a chain leads back into itself or reaches a
   * node twice, or a target lies on two chains or twice on one; when two
   * ports of one lattice that can carry an inner wire meet it at one node; or
   * when the backend name is neither "highs" nor "gurobi".
   * @throws std::runtime_error When the solver gives no solution, or when
   * "gurobi" is asked for and no external backend is registered.
   * @throws Whatever the solve throws, as solveWith() says.
   */
  [[nodiscard]] virtual GlobalRoutingT run(const ChipT& chip,
                                           const CapacityPlanT& capacity,
                                           const ConfigT& config,
                                           const Report& report) const = 0;
};

/**
 * @brief Assigns resonators and conventional ports to launchers, on the ring
 * of outer ports.
 */
class MQT_SCPD_PIPELINE_EXPORT IAssigner {
public:
  /**
   * @brief Creates an assigner.
   */
  IAssigner() = default;
  IAssigner(const IAssigner&) = delete;
  IAssigner& operator=(const IAssigner&) = delete;
  IAssigner(IAssigner&&) = delete;
  IAssigner& operator=(IAssigner&&) = delete;
  virtual ~IAssigner() = default;

  /**
   * @brief Assigns the ports of the ring to launchers.
   * @param chip The classified chip.
   * @param capacity The capacity plan of the run.
   * @param global The global routing of the run, which carries the ring.
   * @param config The run configuration.
   * @param report Where the stage reports what it does.
   * @return The assignment.
   * @throws std::invalid_argument When the configuration or the chip cannot
   * give an assignment: no design rules or no `launcher_target`, launchers
   * listed in the direction of the ring, too few launchers off one line,
   * counts of the ring that no assignment meets, a feed closer than
   * `[design_rules] min_wire_spacing` to a neighboring feed or to a launcher
   * at an end of its segment, or two feeds on one cell of the detail grid; when
   * the capacity plan or the global routing does not fit the chip; or when the
   * backend name is neither "highs" nor "gurobi".
   * @throws std::runtime_error When the solver gives no solution, or when
   * "gurobi" is asked for and no external backend is registered.
   * @throws Whatever the solve throws, as solveWith() says.
   */
  [[nodiscard]] virtual AssignmentT run(const ChipT& chip,
                                        const CapacityPlanT& capacity,
                                        const GlobalRoutingT& global,
                                        const ConfigT& config,
                                        const Report& report) const = 0;
};

} // namespace mqt::scpd::pipeline
