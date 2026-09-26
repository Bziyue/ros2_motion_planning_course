#include <gtest/gtest.h>
#include "motion2d/ros/odometry_messages.hpp"

using namespace motion2d;

TEST(TruthOdometry, PoseAndTwistUseDifferentDeclaredFrames)
{
  State2D state;
  state.pose = {{3, -2}, kPi / 2};
  state.velocity = {2, 1};
  state.yaw_rate = -.4;
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 7;
  stamp.nanosec = 123;
  const auto message = truthOdometry(state, stamp);
  EXPECT_EQ(message.header.frame_id, "world");
  EXPECT_EQ(message.child_frame_id, "ground_truth_base");
  EXPECT_EQ(message.header.stamp, stamp);
  EXPECT_DOUBLE_EQ(message.pose.pose.position.x, 3);
  EXPECT_DOUBLE_EQ(message.pose.pose.position.y, -2);
  EXPECT_NEAR(message.twist.twist.linear.x, 1, 1e-12);
  EXPECT_NEAR(message.twist.twist.linear.y, -2, 1e-12);
  EXPECT_DOUBLE_EQ(message.twist.twist.angular.z, -.4);
  EXPECT_NEAR(message.pose.pose.orientation.z, std::sqrt(.5), 1e-12);
  for (int i = 0; i < 36; ++i) {
    EXPECT_EQ(message.pose.covariance[i], 0);
    EXPECT_EQ(message.twist.covariance[i], 0);
  }
}

TEST(TruthOdometry, CircularReferenceHasBodyForwardVelocity)
{
  const builtin_interfaces::msg::Time stamp;
  for (const double t : {0., .142857143, 2., 8.}) {
    const auto message = truthOdometry(sampleCircle({{-8, -8}, .7}, 1, .4, t), stamp);
    EXPECT_NEAR(message.twist.twist.linear.x, .4, 1e-12);
    EXPECT_NEAR(message.twist.twist.linear.y, 0, 1e-12);
    EXPECT_EQ(message.twist.twist.linear.z, 0);
  }
}
