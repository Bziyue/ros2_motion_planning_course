#include "motion2d/trajectory/trajectory_cost.hpp"
#include "motion2d/trajectory/minco2d.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motion2d
{
void validateDynamicLimits(const OmniDynamicLimits & d)
{
  validateInertialParameters(d.parameters);
  if(!std::isfinite(d.force) || d.force<=0 || !std::isfinite(d.force_rate) || d.force_rate<=0)
    throw std::invalid_argument("Positive finite force and force-rate limits required");
}
void validateCostConfig(const TrajectoryCostConfig & c)
{
  for(double v : {c.energy_weight,c.time_weight,c.speed_weight,c.acceleration_weight,
    c.corridor_weight,c.corridor_margin,c.esdf_weight,c.esdf_distance,c.force_weight,c.force_rate_weight})
    if(!std::isfinite(v) || v<0) {throw std::invalid_argument("Finite nonnegative trajectory cost settings required");}
  if(c.dynamics) validateDynamicLimits(*c.dynamics);
  if(!std::isfinite(c.speed_max) || c.speed_max<=0 || !std::isfinite(c.acceleration_max) ||
    c.acceleration_max<=0 || c.quadrature_steps<1 || c.quadrature_steps>10000)
    {throw std::invalid_argument("Positive cost limits and quadrature count required");}
}

CoefficientGradient trajectoryCost(const PolynomialTrajectory & trajectory,
  const TrajectoryCostConfig & c,const std::vector<ConvexRegion> & regions,const Esdf2D * field)
{
  const int n=trajectory.pieces().size();
  if(!regions.empty() && regions.size()!=std::size_t(n)) {throw std::invalid_argument("One corridor per piece required");}
  CoefficientGradient result;result.coefficients=Eigen::MatrixX2d::Zero(6*n,2);result.times=Eigen::VectorXd::Zero(n);
  for(int i=0;i<n;++i) {
    const auto & piece=trajectory.pieces()[i]; const double T=piece.duration,dt=T/c.quadrature_steps;
    const Eigen::Matrix<double,6,2> C=piece.coefficients.transpose(); const auto Q=jerkGram(T);
    result.cost+=c.energy_weight*(C.array()*(Q*C).array()).sum()+c.time_weight*T;
    result.coefficients.middleRows<6>(6*i)=2*c.energy_weight*Q*C;
    result.times(i)=c.energy_weight*derivative(piece,T,3).squaredNorm()+c.time_weight;
    for(int j=0;j<=c.quadrature_steps;++j) {
      const double s=double(j)/c.quadrature_steps,t=s*T;
      const double weight=(j==0 || j==c.quadrature_steps) ? .5 : 1;
      const auto b0=polynomialBasis(t,0),b1=polynomialBasis(t,1),b2=polynomialBasis(t,2);
      const Eigen::Vector2d p=(b0*C).transpose(),v=(b1*C).transpose(),a=(b2*C).transpose();
      const Eigen::Vector2d jerk=(polynomialBasis(t,3)*C).transpose();
      Eigen::Vector2d gp=Eigen::Vector2d::Zero(),gv=gp,ga=gp,gj=gp; double penalty=0;
      const auto hinge=[&penalty](double residual,double scale) {
        const double positive=std::max(0.,residual);
        penalty+=scale*positive*positive*positive;
        return 3*scale*positive*positive;
      };
      gv+=hinge(v.squaredNorm()-c.speed_max*c.speed_max,c.speed_weight)*2*v;
      ga+=hinge(a.squaredNorm()-c.acceleration_max*c.acceleration_max,c.acceleration_weight)*2*a;
      // flat_penalty_begin
      if(c.dynamics) {
        const auto & d=*c.dynamics;
        const auto physical=omniForward({v,a,jerk,0,0},d.parameters);
        OmniFlatOutput g;
        g.force=hinge(physical.force.squaredNorm()/(d.force*d.force)-1,c.force_weight)
          *2*physical.force/(d.force*d.force);
        g.force_rate=hinge(physical.force_rate.squaredNorm()/(d.force_rate*d.force_rate)-1,c.force_rate_weight)
          *2*physical.force_rate/(d.force_rate*d.force_rate);
        const auto flat=omniBackward(g,d.parameters);
        gv+=flat.velocity;ga+=flat.acceleration;gj+=flat.jerk;
      }
      // flat_penalty_end
      if(!regions.empty()) {
        const auto & region=regions[i];
        for(int k=0;k<region.A.rows();++k)
          {gp+=hinge(region.A.row(k).dot(p)-region.b(k)+c.corridor_margin,c.corridor_weight)*region.A.row(k).transpose();}
      }
      if(field && c.esdf_weight>0) {
        const auto query=field->sample(p);
        if(!query) {throw std::runtime_error("ESDF query outside finite interpolation domain");}
        gp-=hinge(c.esdf_distance-query->distance,c.esdf_weight)*query->gradient;
      }
      // trajectory_quadrature_begin
      result.cost+=weight*dt*penalty;
      result.coefficients.middleRows<6>(6*i)+=weight*dt*(
        b0.transpose()*gp.transpose()+b1.transpose()*gv.transpose()+b2.transpose()*ga.transpose()+
        polynomialBasis(t,3).transpose()*gj.transpose());
      // Both integration weight T/K and sample location t=s*T depend on T.
      result.times(i)+=weight*(penalty/c.quadrature_steps+
        dt*s*(gp.dot(v)+gv.dot(a)+ga.dot(jerk)+gj.dot(derivative(piece,t,4))));
      // trajectory_quadrature_end
    }
  }
  if(!std::isfinite(result.cost) || !result.coefficients.allFinite() || !result.times.allFinite())
    {throw std::runtime_error("Nonfinite trajectory cost or gradient");}
  return result;
}

SampledFeasibility checkTrajectorySamples(const PolynomialTrajectory & trajectory,
  const TrajectoryLimits & limits,const std::vector<ConvexRegion> & regions,const Esdf2D * field)
{
  if(!std::isfinite(limits.speed) || limits.speed<=0 || !std::isfinite(limits.acceleration) || limits.acceleration<=0 ||
    !std::isfinite(limits.clearance) || limits.clearance<0 || limits.samples_per_piece<1 ||
    (!regions.empty() && regions.size()!=trajectory.pieces().size()))
    {throw std::invalid_argument("Invalid sampled validation limits or regions");}
  if(limits.dynamics) validateDynamicLimits(*limits.dynamics);
  SampledFeasibility result;bool field_ok=true,finite=true;
  if(field) {result.min_clearance_lower_bound=std::numeric_limits<double>::infinity();}
  for(std::size_t i=0;i<trajectory.pieces().size();++i) {
    const auto & piece=trajectory.pieces()[i];
    for(int j=0;j<=limits.samples_per_piece;++j) {
      const double t=(double(j)/limits.samples_per_piece)*piece.duration;const auto p=derivative(piece,t,0);
      finite=finite && p.allFinite() && derivative(piece,t,1).allFinite() && derivative(piece,t,2).allFinite();
      result.peak_speed=std::max(result.peak_speed,derivative(piece,t,1).norm());
      result.peak_acceleration=std::max(result.peak_acceleration,derivative(piece,t,2).norm());
      if(limits.dynamics) {
        const auto f=omniForward({derivative(piece,t,1),derivative(piece,t,2),derivative(piece,t,3),0,0},limits.dynamics->parameters);
        finite=finite && f.force.allFinite() && f.force_rate.allFinite();
        result.peak_force=std::max(result.peak_force,f.force.norm());
        result.peak_force_rate=std::max(result.peak_force_rate,f.force_rate.norm());
      }
      if(!regions.empty()) result.max_corridor_residual=std::max(result.max_corridor_residual,
        (regions[i].A*p-regions[i].b).maxCoeff());
      if(field) {
        const auto query=field->sample(p);
        if(!query) {field_ok=false;result.min_clearance_lower_bound=0;}
        else result.min_clearance_lower_bound=std::min(result.min_clearance_lower_bound,query->clearance_lower_bound);
      }
    }
  }
  const bool dynamics_ok=!limits.dynamics || (result.peak_force<=limits.dynamics->force+1e-8 &&
    result.peak_force_rate<=limits.dynamics->force_rate+1e-8);
  result.samples_feasible=finite && field_ok && dynamics_ok && result.peak_speed<=limits.speed+1e-8 &&
    result.peak_acceleration<=limits.acceleration+1e-8 && result.max_corridor_residual<=1e-8 &&
    (!field || result.min_clearance_lower_bound>limits.clearance);
  result.status=!finite ? "nonfinite_state" : !field_ok ? "invalid_field" : result.samples_feasible ? "sampled_feasible" : "sampled_violation";
  return result;
}
}  // namespace motion2d
