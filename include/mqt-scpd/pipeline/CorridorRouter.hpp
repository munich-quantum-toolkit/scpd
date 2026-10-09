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

#include "mqt-scpd/pipeline/Stages.hpp"
#include "mqt-scpd/pipeline/mqt_scpd_pipeline_export.hpp"

#include <memory>

namespace mqt::scpd::pipeline {

/**
 * @brief Creates the corridor router of the first release.
 *
 * The router searches the partition graph and not the cells. The nodes are
 * the crossing slots of the partition borders, an edge joins two slots of one
 * partition, and its cost is the distance between them. A wire is therefore
 * told which partitions it runs through and where it crosses from one into
 * the next, and a later stage draws it inside each partition.
 *
 * Two rules make the plan one that the later stage can follow. No two wires
 * take the same slot, so a border carries no more wires than it has room for.
 * No two wires cross inside one partition, so the wires leave a border in the
 * order they arrive at it.
 *
 * @return The router.
 */
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::unique_ptr<ICorridorRouter>
makePartitionAStarRouter();

} // namespace mqt::scpd::pipeline
