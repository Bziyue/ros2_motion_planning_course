#include <gtest/gtest.h>
#include <algorithm>
#include "motion2d/sim/lidar_cuda.hpp"
using namespace motion2d;
namespace {
void compare(const std::vector<float> & cpu,const std::vector<float> & gpu) {
  ASSERT_EQ(cpu.size(),gpu.size());
  for(std::size_t i=0;i<cpu.size();++i) {
    ASSERT_EQ(std::isnan(cpu[i]),std::isnan(gpu[i]))<<i;
    ASSERT_EQ(std::isinf(cpu[i]),std::isinf(gpu[i]))<<i;
    if(std::isfinite(cpu[i])) EXPECT_NEAR(cpu[i],gpu[i],2e-5)<<i;
  }
}
}
TEST(CudaLidar,RandomPosesSameEncodingAndSharedNoise) {
  WorldConfig wc;wc.seed=42;auto world=generateWorld(wc);LidarConfig c;c.beams=1440;c.range_stddev=.01;validateLidarConfig(c);
  CudaLidar gpu(world,c.beams);std::mt19937 points(8);std::uniform_real_distribution<double> u(-9,9);
  int checked=0;
  while(checked<40) {
    Pose2D pose{{u(points),u(points)},u(points)};if(!isFree(world,pose.position,.2)) continue;++checked;
    auto a=scanCpu(world,pose,c),b=gpu.scan(pose,c);compare(a,b);
    // Same per-beam samples, never independent CPU/GPU generators with one seed.
    std::normal_distribution<double> noise(0,c.range_stddev);
    for(std::size_t i=0;i<a.size();++i) {const double e=noise(points);a[i]=perturbRange(a[i],e,c);b[i]=perturbRange(b[i],e,c);}
    compare(a,b);
  }
}
TEST(CudaLidar,WallsTangentCollinearNearHitNoReturnAndPartialFov) {
  World2D world;world.width=world.height=20;world.circles={{{3,0},1}};
  world.polygons={{{{-3,0},{-2,0},{-2,1},{-3,1}}}};
  LidarConfig c;c.beams=5;c.angle_min=0;c.fov=kPi;c.range_max=20;
  CudaLidar gpu(world,720);Pose2D pose{{0,1},0};compare(scanCpu(world,pose,c),gpu.scan(pose,c));
  auto ranges=gpu.scan(pose,c);EXPECT_NEAR(ranges[0],3,1e-6); // exact circle tangent
  c.beams=720;c.fov=2*kPi;c.range_min=.1;c.range_max=2;
  pose.position={-9.95,0};auto a=scanCpu(world,pose,c),b=gpu.scan(pose,c);compare(a,b);
  EXPECT_GT(std::count_if(b.begin(),b.end(),[](float v){return std::isnan(v);}),0);
  EXPECT_GT(std::count_if(b.begin(),b.end(),[](float v){return std::isinf(v);}),0);
  c.beams=721;EXPECT_THROW(gpu.scan(pose,c),std::invalid_argument);
}
