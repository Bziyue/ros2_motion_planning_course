#include <gtest/gtest.h>
#include "motion2d/planning/replanner.hpp"
using namespace motion2d;
namespace {
GridConfig geometry() {GridConfig g;g.width=60;g.height=40;g.resolution=.1;g.origin={0,0};return g;}
InflationConfig inflation() {InflationConfig c;c.radius=.1;c.margin=.05;return c;}
}
TEST(LocalRoute,UnknownGoalUsesObservedFrontierAndArcLengthLimit) {
  const auto g=geometry();std::vector<std::int8_t> raw(g.width*g.height,-1);
  for(int y=0;y<g.height;++y) for(int x=0;x<30;++x) raw[y*g.width+x]=0;
  const auto grid=inflateGrid(g,raw,inflation());
  const auto result=observedLocalRoute(grid,raw,{.75,2.},{5.,2.},1.2);
  ASSERT_TRUE(result.success)<<result.status;EXPECT_FALSE(result.reaches_global);EXPECT_EQ(result.status,"frontier_prefix");
  double length=0;for(std::size_t k=1;k<result.path.size();++k) {
    EXPECT_TRUE(segmentIsFree(grid,result.path[k-1],result.path[k]));length+=(result.path[k]-result.path[k-1]).norm();
  }
  EXPECT_NEAR(length,1.2,1e-10);EXPECT_LT(result.path.back().x(),3.);
}
TEST(LocalRoute,KnownDisconnectedMapHasNoFrontierAndInvalidGoalsAreExplicit) {
  const auto g=geometry();std::vector<std::int8_t> raw(g.width*g.height,0);
  for(int y=0;y<g.height;++y) raw[y*g.width+30]=100;
  const auto grid=inflateGrid(g,raw,inflation());
  EXPECT_EQ(observedLocalRoute(grid,raw,{1,2},{5,2}).status,"no_reachable_frontier");
  EXPECT_EQ(observedLocalRoute(grid,raw,{1,2},{3.05,2}).status,"goal_occupied");
  EXPECT_EQ(observedLocalRoute(grid,raw,{1,2},{7,2}).status,"goal_outside_map");
  EXPECT_EQ(observedLocalRoute(grid,raw,{.01,.01},{1,2}).status,"invalid_start");
}
TEST(Replanner,MovingBoundaryAndFallbackHaveIndependentCertificates) {
  const auto g=geometry();const std::vector<std::int8_t> raw(g.width*g.height,0);const auto grid=inflateGrid(g,raw,inflation());const Esdf2D field(g,raw);
  TranslationState start;start.position={1.,2.};start.velocity={.15,0};start.acceleration={.03,0};
  ReplanConfig config;config.optimization.solver.max_wall_seconds=0;
  for(bool optimize:{false,true}) {
    config.optimize=optimize;const auto result=replanObserved(grid,raw,field,start,{3.,2.},config);
    ASSERT_TRUE(result.success)<<result.status<<' '<<result.optimization_status;ASSERT_TRUE(result.curve);
    const auto first=result.curve->sample(0);EXPECT_LT((first.position-start.position).norm(),1e-10);EXPECT_LT((first.velocity-start.velocity).norm(),1e-10);EXPECT_LT((first.acceleration-start.acceleration).norm(),1e-10);
    EXPECT_LT(result.curve->sample(result.curve->duration()).velocity.norm(),1e-8);
    EXPECT_TRUE(certifyBezier(*result.curve,result.regions,config.optimization.limits).certified);
    for(const auto & region:result.regions) EXPECT_TRUE(regionIsFree(grid,region));
    if(!optimize) {EXPECT_FALSE(result.optimized);EXPECT_EQ(result.optimization_status,"disabled");}
  }
}
TEST(Replanner,ImpossibleInitialDerivativeNeverGetsCertified) {
  const auto g=geometry();const std::vector<std::int8_t> raw(g.width*g.height,0);const auto grid=inflateGrid(g,raw,inflation());const Esdf2D field(g,raw);
  TranslationState start;start.position={1,2};start.velocity={2,0};ReplanConfig config;config.optimize=false;
  const auto result=replanObserved(grid,raw,field,start,{3,2},config);EXPECT_FALSE(result.success);EXPECT_FALSE(result.curve);EXPECT_EQ(result.status,"no_certified_local_trajectory");
}

TEST(Replanner,SimulationCadenceDoesNotRepeatPausedOrPending) {
  EXPECT_TRUE(replanDue(0,{},200,false));
  EXPECT_FALSE(replanDue(100,100,200,false));
  EXPECT_FALSE(replanDue(299,100,200,false));
  EXPECT_TRUE(replanDue(300,100,200,false));
  EXPECT_FALSE(replanDue(300,100,200,true));
  EXPECT_FALSE(replanDue(50,100,200,false));
}
