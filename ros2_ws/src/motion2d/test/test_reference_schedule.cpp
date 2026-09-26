#include <gtest/gtest.h>
#include "motion2d/trajectory/reference_schedule.hpp"
#include "motion2d/trajectory/quintic.hpp"
using namespace motion2d;
namespace {
NavigationReference makePlan(const TranslationState & start,const Eigen::Vector2d & finish,double T,std::int64_t ns) {
  TranslationState end;end.position=finish;return {{PolynomialTrajectory({interpolateQuintic(start,end,T)}),ns,0},{},0};
}
TranslationState translation(const State2D & s) {return {s.pose.position,s.velocity,s.acceleration};}
}
TEST(ReferenceSchedule,FutureSamplingDoesNotSwitchEarlyAndJoinIsC2) {
  ReferenceSchedule schedule;auto first=makePlan({}, {2,0},4,200000000);
  ASSERT_TRUE(schedule.accept(first,0).accepted);
  EXPECT_NEAR(schedule.sample(100000000).pose.position.norm(),0,1e-12);
  const auto future=schedule.sample(1000000000);EXPECT_GT(future.pose.position.x(),0);
  EXPECT_TRUE(schedule.pending());schedule.advance(300000000);EXPECT_FALSE(schedule.pending());
  const auto boundary=translation(schedule.sample(1400000000));auto second=makePlan(boundary,{3,1},4,1400000000);
  ASSERT_TRUE(schedule.accept(second,1000000000).accepted);
  EXPECT_EQ(schedule.accept(second,1000000000).reason,"handover_pending");
  const auto before=schedule.sample(1399999999),after=schedule.sample(1400000001);
  EXPECT_LT((before.pose.position-after.pose.position).norm(),1e-8);
  EXPECT_LT((before.velocity-after.velocity).norm(),1e-8);
  EXPECT_LT((before.acceleration-after.acceleration).norm(),1e-8);
  EXPECT_GT((schedule.sample(1500000000).pose.position-sampleHeldTrajectory(first.motion,1500000000).pose.position).norm(),1e-6);
  EXPECT_LT((schedule.sample(1300000000).pose.position-sampleHeldTrajectory(first.motion,1300000000).pose.position).norm(),1e-12);
  schedule.advance(1400000000);EXPECT_FALSE(schedule.pending());
  EXPECT_LT(schedule.sample(9000000000).velocity.norm(),1e-12);
}
TEST(ReferenceSchedule,MapCorrectionTransformsAllDerivativesOnce) {
  ReferenceSchedule schedule;auto first=makePlan({}, {2,0},4,0);ASSERT_TRUE(schedule.accept(first,0).accepted);schedule.advance(1);
  const auto boundary=translation(schedule.sample(1200000000));auto next=makePlan(boundary,{3,1},4,1200000000);
  next.regions={makeRegion({{{-2,-2},{5,-2},{5,3},{-2,3}}})};
  const Pose2D map_from_odom{{2,-1},.7};
  const auto in_map=transformReference(next,map_from_odom);
  const auto frozen=transformReference(in_map,inverse(map_from_odom));
  ASSERT_TRUE(schedule.accept(frozen,1000000000).accepted);
  EXPECT_LT((schedule.sample(1200000000).acceleration-boundary.acceleration).norm(),1e-10);
  ASSERT_NE(schedule.region(1500000000),nullptr);EXPECT_TRUE(schedule.region(1500000000)->contains(schedule.sample(1500000000).pose.position));
  // A later map transform has no input into this odom schedule.
  const auto before=schedule.sample(2000000000);const auto other_map=transformReference(next,Pose2D{{-4,3},-1.2});
  EXPECT_GT((other_map.motion.curve.sample(.8).position-before.pose.position).norm(),1.);
  EXPECT_LT((schedule.sample(2000000000).pose.position-before.pose.position).norm(),1e-12);
}
TEST(ReferenceSchedule,InitialHoldUsesFirstPendingRegionWithoutActivating) {
  ReferenceSchedule schedule;auto plan=makePlan({}, {1,0},3,200000000);
  plan.regions={makeRegion({{{-1,-1},{2,-1},{2,1},{-1,1}}})};
  ASSERT_TRUE(schedule.accept(plan,0).accepted);
  ASSERT_NE(schedule.region(100000000),nullptr);
  EXPECT_TRUE(schedule.region(100000000)->contains(schedule.sample(100000000).pose.position));
  EXPECT_TRUE(schedule.pending());EXPECT_EQ(schedule.reference(100000000),nullptr);
}
TEST(ReferenceSchedule,RejectInvalidReplacementKeepValidAndResetExplicitly) {
  ReferenceSchedule schedule;auto first=makePlan({}, {1,0},3,0);ASSERT_TRUE(schedule.accept(first,0).accepted);schedule.advance(1);
  auto bad=makePlan({}, {2,0},3,1000000000);EXPECT_EQ(schedule.accept(bad,100).reason,"discontinuous_handover");
  EXPECT_LT((schedule.sample(2000000000).pose.position-sampleHeldTrajectory(first.motion,2000000000).pose.position).norm(),1e-12);
  EXPECT_EQ(schedule.accept(first,100).reason,"invalid_or_past_time");
  schedule.reset(Pose2D{{4,5},.3});EXPECT_TRUE(schedule.empty());EXPECT_LT((schedule.sample(0).pose.position-Eigen::Vector2d(4,5)).norm(),1e-12);
}
