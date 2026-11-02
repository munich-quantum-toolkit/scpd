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
#include "mqt-scpd/milp/Model.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cstddef>
#include <filesystem>
#include <format>
#include <fstream>
#include <memory>
#include <random>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
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
  // the solves within it, and a random part against other processes. A
  // parameterized test has a slash in its name, so every character that a
  // file name may not carry becomes an underscore.
  static std::atomic<unsigned> counter{0};
  std::string test =
      testing::UnitTest::GetInstance()->current_test_info()->name();
  std::ranges::replace_if(
      test,
      [](const char character) {
        return std::isalnum(static_cast<unsigned char>(character)) == 0 &&
               character != '_' && character != '-';
      },
      '_');
  std::random_device random;
  const auto path = std::filesystem::temp_directory_path() /
                    std::format("mqt-scpd-milp-{}-{}-{:08x}{:08x}.mps", test,
                                counter.fetch_add(1), random(), random());
  Solution solution;
  {
    std::ofstream file(path);
    file << mps;
    // Closing flushes the text, so a write that fails only then shows too.
    file.close();
    if (file.fail()) {
      solution.message = std::format("cannot write {}", path.string());
      std::error_code ignored;
      std::filesystem::remove(path, ignored);
      return solution;
    }
  }

  Highs highs;
  static_cast<void>(highs.setOptionValue("output_flag", false));
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
  // name rather than by position. A name that the text lost is an error, as
  // it is for the gurobipy reader.
  const auto& lp = highs.getLp();
  std::vector<double> values(names.size(), 0.0);
  for (std::size_t index = 0; index < names.size(); ++index) {
    const auto column = std::ranges::find(lp.col_names_, names[index]);
    if (column == lp.col_names_.end()) {
      solution.status = SolveStatus::Error;
      solution.message =
          std::format("the MPS text carries no variable {}", names[index]);
      return solution;
    }
    values[index] = highs.getSolution().col_value[static_cast<std::size_t>(
        column - lp.col_names_.begin())];
  }
  solution.values = std::move(values);
  return solution;
}

/// A backend that solves through solveMpsText() and counts its solves, so
/// that a test can tell that the MPS text was solved at all.
///
/// @param solves The count, which every solve raises by one.
inline std::shared_ptr<ISolverBackend> countingMpsBackend(std::size_t& solves) {
  return makeMpsBackend("mps", [&solves](const std::string_view mps,
                                         const std::vector<std::string>& names,
                                         const SolveOptions& options) {
    ++solves;
    return solveMpsText(mps, names, options);
  });
}

} // namespace mqt::scpd::milp::test
