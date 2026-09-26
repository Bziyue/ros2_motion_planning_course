#include "motion2d/trajectory/quintic.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motion2d
{
QuinticPiece interpolateQuintic(const TranslationState & start,
  const TranslationState & finish, double duration)
{
  if (!std::isfinite(duration) || duration <= 0 || !start.position.allFinite() ||
    !start.velocity.allFinite() || !start.acceleration.allFinite() || !finish.position.allFinite() ||
    !finish.velocity.allFinite() || !finish.acceleration.allFinite())
  {throw std::invalid_argument("Finite boundaries and positive duration required");}
  // quintic_boundary_begin
  const double T = duration;
  Eigen::Matrix<double,2,6> b;
  b.col(0) = start.position;
  b.col(1) = T*start.velocity;
  b.col(2) = .5*T*T*start.acceleration;
  const Eigen::Vector2d r0 = finish.position-b.col(0)-b.col(1)-b.col(2);
  const Eigen::Vector2d r1 = T*finish.velocity-b.col(1)-2*b.col(2);
  const Eigen::Vector2d r2 = T*T*finish.acceleration-2*b.col(2);
  b.col(3) = 10*r0-4*r1+.5*r2;
  b.col(4) = -15*r0+7*r1-r2;
  b.col(5) = 6*r0-3*r1+.5*r2;
  double power = 1;
  for (int k = 0; k < 6; ++k) {b.col(k) /= power; power *= T;}
  // quintic_boundary_end
  if (!b.allFinite()) {throw std::invalid_argument("Quintic coefficient overflow");}
  return {T,b};
}

PolynomialTrajectory stopAtWaypoints(const std::vector<Eigen::Vector2d> & points,
  double nominal_speed, double min_duration)
{
  if (points.size()<2 || !std::isfinite(nominal_speed) || nominal_speed<=0 ||
    !std::isfinite(min_duration) || min_duration<=0)
  {throw std::invalid_argument("Waypoints need positive speed and duration");}
  std::vector<QuinticPiece> pieces;
  for (std::size_t i=1; i<points.size(); ++i) {
    TranslationState a,b; a.position=points[i-1]; b.position=points[i];
    const double T=std::max(min_duration,(b.position-a.position).norm()/nominal_speed);
    pieces.push_back(interpolateQuintic(a,b,T));
  }
  return PolynomialTrajectory(std::move(pieces));
}
}  // namespace motion2d
