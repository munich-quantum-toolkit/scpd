/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// What a stage says while it runs: which lines reach the caller at which
// level, how progress is thinned out, and how a solver reports through it.

#include "mqt-scpd/milp/Backend.hpp"
#include "mqt-scpd/pipeline/Report.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace mqt::scpd::pipeline {
namespace {

/// Everything a report hands out, as a renderer would receive it.
struct Recorder {
  std::vector<std::pair<Detail, std::string>> lines;
  std::vector<std::pair<std::string, std::vector<Figure>>> entries;
  std::vector<std::vector<Figure>> results;
  std::vector<std::optional<std::uint64_t>> fails;
  std::vector<std::string> progress;

  [[nodiscard]] Report report(const Detail level) {
    return Report(
        level,
        {.line =
             [this](const Detail detail, const std::string_view text) {
               lines.emplace_back(detail, std::string(text));
             },
         .entry =
             [this](const Detail /*detail*/, const std::string_view label,
                    const std::vector<Figure>& figures) {
               entries.emplace_back(std::string(label), figures);
             },
         .result =
             [this](const std::vector<Figure>& figures,
                    const std::optional<std::uint64_t> count) {
               results.push_back(figures);
               fails.push_back(count);
             },
         .progress =
             [this](const std::string_view task, const std::string_view detail,
                    const std::uint64_t done, const std::uint64_t total) {
               progress.push_back(
                   std::string(task) + "|" + std::string(detail) + "|" +
                   std::to_string(done) + "/" + std::to_string(total));
             }});
  }
};

TEST(Report, AnEmptyReportWantsNothingAndDropsEverything) {
  const Report report;

  EXPECT_FALSE(report.wants(Detail::Summary));
  report.line(Detail::Summary, "nobody hears this");
  report.entry(Detail::Summary, "round 1", {{.text = "fails 0"}});
  report.result({{.text = "363 partitions"}});
  report.progress("task", "", 1, 2);
}

TEST(Report, PassesALineOnlyUpToItsLevel) {
  Recorder recorder;
  const auto report = recorder.report(Detail::Steps);

  EXPECT_TRUE(report.wants(Detail::Summary));
  EXPECT_TRUE(report.wants(Detail::Steps));
  EXPECT_FALSE(report.wants(Detail::Items));
  report.line(Detail::Summary, "a summary");
  report.line(Detail::Steps, "a step");
  report.line(Detail::Items, "an item");
  report.entry(Detail::Items, "an item entry", {});

  ASSERT_EQ(recorder.lines.size(), 2U);
  EXPECT_EQ(recorder.lines[0],
            std::pair(Detail::Summary, std::string("a summary")));
  EXPECT_EQ(recorder.lines[1], std::pair(Detail::Steps, std::string("a step")));
  EXPECT_TRUE(recorder.entries.empty());
}

TEST(Report, HandsOverEntriesAndTheResultWithItsFails) {
  Recorder recorder;
  const auto report = recorder.report(Detail::Summary);

  report.entry(Detail::Summary, "round 1  forward",
               {{.text = "routed 56/58"}, verdict("fails", 2)});
  report.result({{.text = "363 partitions"}}, 0U);

  ASSERT_EQ(recorder.entries.size(), 1U);
  EXPECT_EQ(recorder.entries[0].first, "round 1  forward");
  ASSERT_EQ(recorder.entries[0].second.size(), 2U);
  EXPECT_EQ(recorder.entries[0].second[1].text, "fails 2");
  EXPECT_EQ(recorder.entries[0].second[1].tone, Tone::Bad);
  ASSERT_EQ(recorder.results.size(), 1U);
  EXPECT_EQ(recorder.results[0][0].text, "363 partitions");
  EXPECT_EQ(recorder.fails[0], std::optional<std::uint64_t>(0U));
}

TEST(Report, AVerdictIsGoodAtZeroAndBadOtherwise) {
  EXPECT_EQ(verdict("fails", 0).text, "fails 0");
  EXPECT_EQ(verdict("fails", 0).tone, Tone::Good);
  EXPECT_EQ(verdict("unrouted", 3).text, "unrouted 3");
  EXPECT_EQ(verdict("unrouted", 3).tone, Tone::Bad);
}

TEST(Report, ThinsOutProgressButKeepsEveryTaskAndItsEnd) {
  Recorder recorder;
  const auto report = recorder.report(Detail::Summary);

  for (std::uint64_t done = 1; done <= 10000; ++done) {
    report.progress("bottlenecks", "", done, 10000);
  }
  report.progress("chains", "", 1, 5);

  // A tight loop of ten thousand calls reaches the caller a few times: the
  // first, the last of its task and the first of the next task always.
  ASSERT_GE(recorder.progress.size(), 3U);
  EXPECT_LT(recorder.progress.size(), 100U);
  EXPECT_EQ(recorder.progress.front(), "bottlenecks||1/10000");
  EXPECT_EQ(recorder.progress[recorder.progress.size() - 2],
            "bottlenecks||10000/10000");
  EXPECT_EQ(recorder.progress.back(), "chains||1/5");
}

TEST(Report, ACopySharesTheThinning) {
  Recorder recorder;
  const auto report = recorder.report(Detail::Summary);
  // The copy is what the test is about.
  const Report copy =
      report; // NOLINT(performance-unnecessary-copy-initialization)

  report.progress("task", "", 1, 3);
  copy.progress("task", "", 2, 3);

  // The second call comes within the interval of the first, through a copy.
  EXPECT_EQ(recorder.progress.size(), 1U);
}

TEST(Report, CountsInTheSingularOnlyForOne) {
  EXPECT_EQ(counted(0, "node"), "0 nodes");
  EXPECT_EQ(counted(1, "node"), "1 node");
  EXPECT_EQ(counted(363, "partition"), "363 partitions");
}

TEST(Report, ASolverLogsOnlyAtTheSolverLevel) {
  Recorder quiet;
  EXPECT_FALSE(static_cast<bool>(
      quiet.report(Detail::Items).solveObserver("solving").log));

  Recorder loud;
  const auto observer = loud.report(Detail::Solver).solveObserver("solving");
  ASSERT_TRUE(static_cast<bool>(observer.log));
  observer.log("Presolving model");

  ASSERT_EQ(loud.lines.size(), 1U);
  EXPECT_EQ(loud.lines[0].first, Detail::Solver);
  EXPECT_EQ(loud.lines[0].second, "Presolving model");
}

TEST(Report, ASolverReportsItsSearchAsProgress) {
  Recorder recorder;
  const auto observer =
      recorder.report(Detail::Summary).solveObserver("solving the assignment");
  ASSERT_TRUE(static_cast<bool>(observer.progress));

  observer.progress({.objective = 27.84,
                     .bound = 27.0,
                     .gap = 0.0310,
                     .nodes = 120,
                     .seconds = 0.5});
  observer.progress({});

  ASSERT_EQ(recorder.progress.size(), 1U);
  EXPECT_EQ(
      recorder.progress[0],
      "solving the assignment|objective 27.84 · gap 3.1 % · 120 nodes|0/0");
}

TEST(Report, ASolverCountsOneNodeInTheSingular) {
  Recorder recorder;
  const auto observer =
      recorder.report(Detail::Summary).solveObserver("solving");
  ASSERT_TRUE(static_cast<bool>(observer.progress));

  observer.progress({.nodes = 1});

  ASSERT_EQ(recorder.progress.size(), 1U);
  EXPECT_EQ(recorder.progress[0], "solving|1 node|0/0");
}

} // namespace
} // namespace mqt::scpd::pipeline
