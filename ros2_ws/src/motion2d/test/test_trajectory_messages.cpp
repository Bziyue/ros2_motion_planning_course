#include <gtest/gtest.h>
#include "motion2d/ros/trajectory_messages.hpp"
#include "motion2d/trajectory/quintic.hpp"
#include <limits>

using namespace motion2d;
TEST(TrajectoryMessages, LosslessRoundTripAndRejectedInputs)
{
  TimedTrajectory t{stopAtWaypoints({{1,2},{2,3},{4,2}},.6),1234567890,.3};
  builtin_interfaces::msg::Time published; published.sec=1;
  const auto msg=toTrajectoryMessage(t,published); const auto back=fromTrajectoryMessage(msg);
  EXPECT_EQ(back.start_ns,t.start_ns); EXPECT_DOUBLE_EQ(back.yaw,t.yaw);
  EXPECT_EQ(msg.header.frame_id,"odom"); EXPECT_EQ(msg.header.stamp.sec,1);
  EXPECT_EQ(back.curve.pieces().size(),t.curve.pieces().size());
  for(std::size_t i=0;i<t.curve.pieces().size();++i) {
    EXPECT_TRUE((back.curve.pieces()[i].coefficients.array()==t.curve.pieces()[i].coefficients.array()).all());
    EXPECT_DOUBLE_EQ(back.curve.pieces()[i].duration,t.curve.pieces()[i].duration);
  }
  auto bad=msg; bad.header.frame_id="map"; EXPECT_THROW(fromTrajectoryMessage(bad),std::invalid_argument);
  bad=msg; bad.pieces.clear(); EXPECT_THROW(fromTrajectoryMessage(bad),std::invalid_argument);
  bad=msg; bad.pieces[0].duration=0; EXPECT_THROW(fromTrajectoryMessage(bad),std::invalid_argument);
  bad=msg; bad.pieces[1].x[0]+=.1; EXPECT_THROW(fromTrajectoryMessage(bad),std::invalid_argument);
  bad=msg; bad.yaw=std::numeric_limits<double>::quiet_NaN(); EXPECT_THROW(fromTrajectoryMessage(bad),std::invalid_argument);
  bad=msg; bad.start_time.nanosec=1000000000; EXPECT_THROW(fromTrajectoryMessage(bad),std::invalid_argument);
}
