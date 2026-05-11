/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "Add.hpp"

#include <gtest/gtest.h>

TEST(Add, OnePlusTwo) { EXPECT_EQ(add(1, 2), 3); }
