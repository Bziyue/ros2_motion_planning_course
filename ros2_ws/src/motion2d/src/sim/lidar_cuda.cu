#include "motion2d/sim/lidar_cuda.hpp"
#include <cuda_runtime.h>
#include <math_constants.h>
#include <chrono>
#include <stdexcept>
#include <array>
namespace motion2d {
namespace {
using Clock=std::chrono::steady_clock;
double seconds(Clock::time_point a,Clock::time_point b) {return std::chrono::duration<double>(b-a).count();}
void check(cudaError_t error) {if(error!=cudaSuccess) throw std::runtime_error(cudaGetErrorString(error));}
__device__ double cross(double x,double y,double u,double v) {return x*v-y*u;}
// cuda_circle_begin
__device__ double circleHit(double ox,double oy,double dx,double dy,double cx,double cy,double r) {
  const double x=cx-ox,y=cy-oy,h=x*dx+y*dy;
  const double perpendicular=cross(dx,dy,x,y),discriminant=r*r-perpendicular*perpendicular;
  if(discriminant<0.) return CUDART_INF;
  const double root=sqrt(discriminant);
  if(h-root>=0.) return h-root;
  return h+root>=0. ? h+root : CUDART_INF;
}
// cuda_circle_end
__device__ double segmentHit(double ox,double oy,double dx,double dy,double ax,double ay,double bx,double by) {
  const double ex=bx-ax,ey=by-ay,x=ax-ox,y=ay-oy,den=cross(dx,dy,ex,ey);
  const double tolerance=1e-12*fmax(1.,sqrt(ex*ex+ey*ey));
  if(fabs(den)<=tolerance) {
    if(fabs(cross(x,y,dx,dy))>tolerance) return CUDART_INF;
    const double t0=x*dx+y*dy,t1=(bx-ox)*dx+(by-oy)*dy;
    return fmax(t0,t1)<0. ? CUDART_INF : fmax(0.,fmin(t0,t1));
  }
  const double t=cross(x,y,ex,ey)/den,u=cross(x,y,dx,dy)/den;
  return t>=0. && u>=0. && u<=1. ? t : CUDART_INF;
}
// cuda_rays_begin
__global__ void scanKernel(const double * geometry,int circles,int edges,
  double ox,double oy,double yaw,double angle_min,double increment,int beams,
  double range_min,double range_max,float * ranges) {
  const int i=blockIdx.x*blockDim.x+threadIdx.x;
  if(i>=beams) return;
  const double angle=yaw+angle_min+i*increment,dx=cos(angle),dy=sin(angle);
  const double * cx=geometry,*cy=cx+circles,*r=cy+circles;
  const double * ax=r+circles,*ay=ax+edges,*bx=ay+edges,*by=bx+edges;
  double nearest=CUDART_INF;
  for(int j=0;j<circles;++j) nearest=fmin(nearest,circleHit(ox,oy,dx,dy,cx[j],cy[j],r[j]));
  for(int j=0;j<edges;++j) nearest=fmin(nearest,segmentHit(ox,oy,dx,dy,ax[j],ay[j],bx[j],by[j]));
  ranges[i]=nearest<range_min ? CUDART_NAN_F : nearest>range_max ? CUDART_INF_F : float(nearest);
}
// cuda_rays_end
}
struct CudaLidar::Impl {
  double * geometry=nullptr;float * ranges=nullptr;cudaEvent_t begin=nullptr,end=nullptr;
  int circles=0,edges=0,capacity=0;double setup_seconds=0,upload_seconds=0;
  ~Impl() {if(begin) cudaEventDestroy(begin);if(end) cudaEventDestroy(end);if(geometry) cudaFree(geometry);if(ranges) cudaFree(ranges);}
};
CudaLidar::CudaLidar(const World2D & world,int capacity):impl_(std::make_unique<Impl>()) {
  if(capacity<2 || capacity>100000) throw std::invalid_argument("Invalid CUDA beam capacity");
  const auto begin=Clock::now();auto & out=*impl_;out.capacity=capacity;out.circles=int(world.circles.size());
  std::vector<std::array<double,4>> edges;
  for(const auto & p:world.polygons) for(std::size_t j=0;j<p.vertices.size();++j) {
    const auto & a=p.vertices[j];const auto & b=p.vertices[(j+1)%p.vertices.size()];edges.push_back({a.x(),a.y(),b.x(),b.y()});
  }
  const double x=world.width/2,y=world.height/2;
  edges.insert(edges.end(),{{-x,-y,x,-y},{x,-y,x,y},{x,y,-x,y},{-x,y,-x,-y}});out.edges=int(edges.size());
  std::vector<double> packed(3*out.circles+4*out.edges);
  for(int j=0;j<out.circles;++j) {packed[j]=world.circles[j].center.x();packed[out.circles+j]=world.circles[j].center.y();packed[2*out.circles+j]=world.circles[j].radius;}
  for(int k=0;k<4;++k) for(int j=0;j<out.edges;++j) packed[3*out.circles+k*out.edges+j]=edges[j][k];
  check(cudaMalloc(&out.geometry,packed.size()*sizeof(double)));check(cudaMalloc(&out.ranges,capacity*sizeof(float)));
  const auto upload_begin=Clock::now();
  check(cudaMemcpy(out.geometry,packed.data(),packed.size()*sizeof(double),cudaMemcpyHostToDevice));
  out.upload_seconds=seconds(upload_begin,Clock::now());
  check(cudaEventCreate(&out.begin));check(cudaEventCreate(&out.end));out.setup_seconds=seconds(begin,Clock::now());
}
CudaLidar::~CudaLidar()=default;
double CudaLidar::setupSeconds() const {return impl_->setup_seconds;}
double CudaLidar::uploadSeconds() const {return impl_->upload_seconds;}
std::vector<float> CudaLidar::scan(const Pose2D & pose,const LidarConfig & config,CudaScanTiming * timing) {
  if(config.beams>impl_->capacity) throw std::invalid_argument("CUDA scan exceeds reserved beams");
  const auto begin=Clock::now();auto & b=*impl_;std::vector<float> ranges(config.beams);
  if(timing) check(cudaEventRecord(b.begin));
  scanKernel<<<(config.beams+127)/128,128>>>(b.geometry,b.circles,b.edges,pose.position.x(),pose.position.y(),pose.yaw,
    config.angle_min,beamIncrement(config),config.beams,config.range_min,config.range_max,b.ranges);
  check(cudaGetLastError());
  if(timing) {check(cudaEventRecord(b.end));check(cudaEventSynchronize(b.end));}
  const auto copy_begin=Clock::now();
  check(cudaMemcpy(ranges.data(),b.ranges,config.beams*sizeof(float),cudaMemcpyDeviceToHost));
  const auto end=Clock::now();
  if(timing) {float ms=0;check(cudaEventElapsedTime(&ms,b.begin,b.end));*timing={ms*1e-3,seconds(copy_begin,end),seconds(begin,end)};}
  return ranges;
}
}
