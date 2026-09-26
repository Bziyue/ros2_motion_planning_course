#include <gtest/gtest.h>
#include "motion2d/ros/navigation_messages.hpp"
#include "motion2d/trajectory/quintic.hpp"
using namespace motion2d;
TEST(NavigationMessages,AtomicDoublePrecisionRoundTripAndValidation) {
  TranslationState start,end;end.position={.5,0};NavigationReference plan{{PolynomialTrajectory({interpolateQuintic(start,end,3)}),500000000,0},
    {makeRegion({{{-.123456789,-1},{1,-1},{1,1},{-.123456789,1}}})},250000000};
  const auto message=toNavigationMessage(plan,builtin_interfaces::msg::Time{});const auto decoded=fromNavigationMessage(message);
  EXPECT_EQ(decoded.motion.start_ns,500000000);EXPECT_EQ(decoded.snapshot_ns,250000000);
  EXPECT_EQ(decoded.regions[0].polygon.vertices[0].x(),-.123456789);
  EXPECT_LT((decoded.motion.curve.sample(1).position-plan.motion.curve.sample(1).position).norm(),1e-14);
  auto bad=message;bad.regions.clear();EXPECT_THROW(fromNavigationMessage(bad),std::invalid_argument);
  bad=message;bad.regions[0].vertices[0].z=1;EXPECT_THROW(fromNavigationMessage(bad),std::invalid_argument);
  bad=message;bad.motion.header.frame_id="map";EXPECT_THROW(fromNavigationMessage(bad),std::invalid_argument);
  bad=message;bad.snapshot_time.nanosec=1000000000;EXPECT_THROW(fromNavigationMessage(bad),std::invalid_argument);
}
