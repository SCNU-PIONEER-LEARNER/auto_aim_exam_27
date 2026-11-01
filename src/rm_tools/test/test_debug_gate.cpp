#include "rm_tools/debug_gate.hpp"

#include <gtest/gtest.h>

TEST(DebugGate, KeepsDebugDisabledWhenParameterIsFalse)
{
  EXPECT_FALSE(rm_tools::debug_enabled(false, false));
  EXPECT_FALSE(rm_tools::debug_enabled(false, true));
}

TEST(DebugGate, AllowsDebugOnlyOutsideCompetition)
{
  EXPECT_TRUE(rm_tools::debug_enabled(true, false));
  EXPECT_FALSE(rm_tools::debug_enabled(true, true));
}

TEST(DebugGate, AllowsVisualizationOnlyOutsideCompetition)
{
  EXPECT_TRUE(rm_tools::visualization_enabled(false));
  EXPECT_FALSE(rm_tools::visualization_enabled(true));
}
