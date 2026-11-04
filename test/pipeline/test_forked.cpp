/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The parallel tasks of the Final stage: forked copies, their answers in
// order, the budget shared by nested tasks, and the bytes they hand back.

#include "mqt-scpd/pipeline/Forked.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace mqt::scpd::pipeline::forked {
namespace {

#ifndef _WIN32

TEST(Forked, AnswersComeBackInTheOrderOfTheTasks) {
  const Budget budget(4);
  const auto answers = map(12, [](const std::size_t index) {
    return std::string(index + 1, static_cast<char>('a' + index));
  });
  ASSERT_EQ(answers.size(), 12U);
  for (std::size_t index = 0; index < answers.size(); ++index) {
    EXPECT_EQ(answers[index],
              std::string(index + 1, static_cast<char>('a' + index)));
  }
}

TEST(Forked, ATaskSeesTheStateItWasForkedOffAndChangesNothing) {
  const Budget budget(3);
  std::vector<int> state(8, 7);
  const auto answers = map(8, [&](const std::size_t index) {
    // Every task finds the state as it stood, whatever its siblings wrote.
    const auto seen = state[index];
    for (auto& value : state) {
      value = -1;
    }
    return std::to_string(seen);
  });
  for (const auto& answer : answers) {
    EXPECT_EQ(answer, "7");
  }
  EXPECT_EQ(state, std::vector<int>(8, 7));
}

TEST(Forked, AStopLeavesTheLaterAnswersEmpty) {
  const Budget budget(2);
  const auto answers = map(
      10, [](const std::size_t index) { return std::to_string(index); },
      [](const std::size_t index, const std::string&) { return index == 3; });
  for (std::size_t index = 0; index <= 3; ++index) {
    EXPECT_EQ(answers[index], std::to_string(index));
  }
  for (std::size_t index = 4; index < answers.size(); ++index) {
    EXPECT_TRUE(answers[index].empty());
  }
}

TEST(Forked, NestedTasksShareTheBudgetAndAnswerAlike) {
  const Budget budget(4);
  EXPECT_EQ(Budget::threads(), 4U);
  EXPECT_FALSE(inTask());
  const auto answers = map(3, [](const std::size_t outer) {
    const auto inner = map(5, [outer](const std::size_t index) {
      return std::to_string((outer * 10) + index) + (inTask() ? "" : "!");
    });
    std::string joined;
    for (const auto& answer : inner) {
      joined += answer + ",";
    }
    return joined;
  });
  EXPECT_EQ(answers[0], "0,1,2,3,4,");
  EXPECT_EQ(answers[1], "10,11,12,13,14,");
  EXPECT_EQ(answers[2], "20,21,22,23,24,");
}

TEST(Forked, OneThreadRunsTheSameTasks) {
  const auto run = [](const std::uint32_t threads) {
    const Budget budget(threads);
    return map(6, [](const std::size_t index) {
      return std::to_string(index * index);
    });
  };
  EXPECT_EQ(run(1), run(6));
}

TEST(Forked, ABigAnswerComesBackWhole) {
  const Budget budget(2);
  const auto answers = map(2, [](const std::size_t index) {
    return std::string(1U << 20U, static_cast<char>('x' + index));
  });
  EXPECT_EQ(answers[0], std::string(1U << 20U, 'x'));
  EXPECT_EQ(answers[1], std::string(1U << 20U, 'y'));
}

#endif

TEST(Forked, AReaderReadsWhatAWriterWrote) {
  Writer out;
  out.put(std::uint32_t{42});
  out.put(std::string_view("way"));
  out.put(std::vector<std::uint16_t>{1, 2, 3});
  out.put(2.5);
  const auto bytes = out.take();
  Reader in(bytes);
  EXPECT_EQ(in.get<std::uint32_t>(), 42U);
  EXPECT_EQ(in.text(), "way");
  EXPECT_EQ(in.vector<std::uint16_t>(), (std::vector<std::uint16_t>{1, 2, 3}));
  EXPECT_EQ(in.get<double>(), 2.5);
  EXPECT_TRUE(in.done());
  EXPECT_THROW(static_cast<void>(in.get<std::uint8_t>()), std::runtime_error);
}

} // namespace
} // namespace mqt::scpd::pipeline::forked
