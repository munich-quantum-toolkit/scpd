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

using flatbuffers::artifacts::CapacityPlanT;
using flatbuffers::config::ConfigT;
using flatbuffers::design::ChipT;

// Every stage has the same shape, and the shape is part of the contract.
// `run` is const and takes its inputs by const reference, so a stage can
// neither change its own state between runs nor reach into the state of
// another. It returns a new value rather than writing into a shared design
// database. Given the same inputs it produces the same output, which is what
// makes a resumed run equal to an uninterrupted one. A stage reports what it
// does through the Report it is given and never prints.

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

} // namespace mqt::scpd::pipeline
