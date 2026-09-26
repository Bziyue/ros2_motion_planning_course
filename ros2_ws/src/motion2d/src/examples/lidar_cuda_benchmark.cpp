#include "motion2d/sim/lidar_cuda.hpp"
#include <cuda_runtime_api.h>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
using namespace motion2d;
namespace {
/** @brief Synthetic timing fixture; no generator connectivity bias or truth map publication. */
World2D scene(int count) {
  World2D w;w.width=w.height=40;
  for(int i=0;i<count;++i) {
    const double a=i*2.399963229728653,r=3.+(i%17)*.7;Eigen::Vector2d p{r*std::cos(a),r*std::sin(a)};
    if(i%2==0) w.circles.push_back({p,.12});
    else w.polygons.push_back({{p+Eigen::Vector2d(-.1,-.1),p+Eigen::Vector2d(.1,-.1),p+Eigen::Vector2d(.1,.1),p+Eigen::Vector2d(-.1,.1)}});
  }
  return w;
}
double percentile(std::vector<double> v,double fraction) {std::sort(v.begin(),v.end());return v[std::min(v.size()-1,std::size_t(fraction*(v.size()-1)))];}
}
/** @brief Ch20 paired CPU/GPU correctness and warm/resident timing (no ROS yet). */
int main() {
  try {
    cudaDeviceProp property{};auto status=cudaGetDeviceProperties(&property,0);if(status!=cudaSuccess) throw std::runtime_error(cudaGetErrorString(status));
    std::cerr<<property.name<<"; compute capability "<<property.major<<'.'<<property.minor<<"; double arithmetic, 128 threads/block\n";
    std::cout<<"obstacles,beams,repeats,setup_ms,upload_ms,cpu_p50_ms,cpu_p95_ms,kernel_p50_ms,download_p50_ms,gpu_scan_p50_ms,gpu_scan_p95_ms,max_error_m\n"<<std::setprecision(9);
    for(int obstacles:{0,16,128,512}) for(int beams:{90,720,4096,16384}) {
      auto world=scene(obstacles);LidarConfig c;c.beams=beams;c.range_max=30;validateLidarConfig(c);CudaLidar gpu(world,beams);
      Pose2D pose{{0,0},.123};for(int j=0;j<20;++j) {scanCpu(world,pose,c);gpu.scan(pose,c);}
      std::vector<double> cpu,device,copy,total;double max_error=0;const int repeats=60;
      for(int j=0;j<repeats;++j) {
        pose.yaw=.123+j*.001;auto begin=std::chrono::steady_clock::now();auto a=scanCpu(world,pose,c);
        cpu.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count());
        CudaScanTiming timing;auto b=gpu.scan(pose,c,&timing);device.push_back(timing.kernel_seconds*1000);copy.push_back(timing.download_seconds*1000);total.push_back(timing.scan_seconds*1000);
        for(int i=0;i<beams;++i) {
          if(std::isnan(a[i])!=std::isnan(b[i]) || std::isinf(a[i])!=std::isinf(b[i])) throw std::runtime_error("Special-value mismatch");
          if(std::isfinite(a[i])) max_error=std::max(max_error,double(std::abs(a[i]-b[i])));
        }
      }
      if(max_error>2e-5) throw std::runtime_error("Distance tolerance exceeded");
      std::cout<<obstacles<<','<<beams<<','<<repeats<<','<<gpu.setupSeconds()*1000<<','<<gpu.uploadSeconds()*1000<<','<<percentile(cpu,.5)<<','<<percentile(cpu,.95)<<','<<percentile(device,.5)<<','<<percentile(copy,.5)<<','<<percentile(total,.5)<<','<<percentile(total,.95)<<','<<max_error<<'\n'<<std::flush;
    }
  } catch(const std::exception & e) {std::cerr<<e.what()<<'\n';return 1;}
}
