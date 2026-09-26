#include "motion2d/trajectory/execution.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motion2d
{
State2D sampleHeldTrajectory(const TimedTrajectory & trajectory, std::int64_t stamp)
{
  const double t=(stamp-trajectory.start_ns)*1e-9;
  const auto value=trajectory.curve.sample(std::clamp(t,0.,trajectory.curve.duration()));
  State2D state; state.pose.position=value.position; state.pose.yaw=trajectory.yaw;
  if(t>=0 && t<=trajectory.curve.duration()) {
    state.velocity=value.velocity; state.acceleration=value.acceleration;
  }
  return state;
}

double trajectoryAccelerationBound(const PolynomialTrajectory & curve,double begin,double end)
{
  if(!std::isfinite(begin) || !std::isfinite(end) || end<begin)
  {throw std::invalid_argument("Invalid acceleration-bound interval");}
  // polynomial_sweep_begin
  double offset=0, bound=0;
  for(const auto & piece : curve.pieces()) {
    if(end>=offset && begin<=offset+piece.duration) {
      const double hi=std::clamp(end-offset,0.,piece.duration);
      double sum=0, power=1;
      for(int k=2;k<6;++k) {
        sum+=k*(k-1)*piece.coefficients.col(k).norm()*power;
        power*=hi;
      }
      bound=std::max(bound,sum);
    }
    offset+=piece.duration;
  }
  return bound;
  // polynomial_sweep_end
}
}  // namespace motion2d
