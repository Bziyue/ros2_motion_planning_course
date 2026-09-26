#include <gtest/gtest.h>
#include <stdexcept>
#include "motion2d/sim/sim_clock.hpp"
using namespace std::chrono_literals;

TEST(SimClock, IntegerTimeDoesNotAccumulateFloatError)
{
  motion2d::SimClock clock(5ms);
  for (int i = 0; i < 1000; ++i) {EXPECT_TRUE(clock.advance());}
  EXPECT_EQ(clock.nanoseconds(), 5000000000LL);
}

TEST(SimClock, PauseStepReset)
{
  motion2d::SimClock clock(5ms);
  EXPECT_FALSE(clock.singleStep());
  clock.advance();
  clock.setPaused(true);
  EXPECT_FALSE(clock.advance());
  EXPECT_EQ(clock.nanoseconds(), 5000000);
  EXPECT_TRUE(clock.singleStep());
  EXPECT_EQ(clock.nanoseconds(), 10000000);
  clock.reset();
  EXPECT_EQ(clock.nanoseconds(), 0);
  EXPECT_TRUE(clock.paused());
}

TEST(SimClock, RejectNonpositiveStep)
{
  EXPECT_THROW(motion2d::SimClock{0ns}, std::invalid_argument);
  EXPECT_THROW(motion2d::SimClock{-1ns}, std::invalid_argument);
}
