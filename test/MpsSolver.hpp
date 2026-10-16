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

#include "Highs.h"
#include "lp_data/HConst.h"
#include "lp_data/HighsStatus.h"
#include "mqt-scpd/milp/Backend.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <cstddef>
#include <filesystem>
#include <format>
#include <fstream>
#include <random>
#include <string>
#include <string_view>
#include <vector>

namespace mqt::scpd::milp::test {

/// An external solver as the core sees one: it reads the MPS text from a file
/// and answers for the problem the text states, which is a minimization.
///
/// Going through a file rather than a string is the point: it is the same
/// path `gurobipy` takes, so a syntax the reader would reject shows up here.
inline Solution solveMpsText(const std::string_view mps,
                             const std::vector<std::string>& names,
                             const SolveOptions& /*options*/) {
  // CTest runs every test in a process of its own, and parallel runs share
  // the temporary directory, so the name carries the test, a counter for
  // the solves within it, and a random part against other processes.
  static std::atomic<unsigned> counter{0};
  std::random_device random;
  const auto path =
      std::filesystem::temp_directory_path() /
      std::format("mqt-scpd-milp-{}-{}-{:08x}{:08x}.mps",
                  testing::UnitTest::GetInstance()->current_test_info()->name(),
                  counter.fetch_add(1), random(), random());
  {
    std::ofstream file(path);
    file << mps;
  }

  Highs highs;
  static_cast<void>(highs.setOptionValue("output_flag", false));
  Solution solution;
  if (highs.readModel(path.string()) != HighsStatus::kOk) {
    solution.message = "the emitted MPS text was rejected";
    std::filesystem::remove(path);
    return solution;
  }
  std::filesystem::remove(path);

  if (highs.run() == HighsStatus::kError) {
    solution.message = "the model read back from MPS failed to solve";
    return solution;
  }
  const auto status = highs.getModelStatus();
  if (status == HighsModelStatus::kInfeasible) {
    solution.status = SolveStatus::Infeasible;
    return solution;
  }
  if (status == HighsModelStatus::kUnbounded) {
    solution.status = SolveStatus::Unbounded;
    return solution;
  }
  if (status != HighsModelStatus::kOptimal) {
    solution.message =
        "the model read back from MPS did not solve to optimality";
    return solution;
  }

  solution.status = SolveStatus::Optimal;
  solution.objective = highs.getInfo().objective_function_value;

  // The round trip only preserves names, so the values are matched back by
  // name rather than by position.
  const auto& lp = highs.getLp();
  solution.values.assign(names.size(), 0.0);
  for (std::size_t index = 0; index < names.size(); ++index) {
    for (std::size_t column = 0; column < lp.col_names_.size(); ++column) {
      if (lp.col_names_[column] == names[index]) {
        solution.values[index] = highs.getSolution().col_value[column];
        break;
      }
    }
  }
  return solution;
}

} // namespace mqt::scpd::milp::test
