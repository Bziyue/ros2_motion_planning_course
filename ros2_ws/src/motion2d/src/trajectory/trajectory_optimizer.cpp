#include "motion2d/trajectory/trajectory_optimizer.hpp"
#include "motion2d/trajectory/spline2d.hpp"
#include "motion2d/trajectory/bezier_bounds.hpp"
#include <cmath>
#include <stdexcept>
namespace motion2d {
namespace {
template<class Curve>
TrajectoryOptimizationResult optimizeWith(const TranslationState & start,const TranslationState & finish,
  const std::vector<Eigen::Vector2d> & interior,const std::vector<double> & durations,
  const TrajectoryOptimizationConfig & config,const std::vector<ConvexRegion> & regions,const Esdf2D * field)
{
  validateCostConfig(config.cost);
  if(!std::isfinite(config.min_duration) || config.min_duration<=0) throw std::invalid_argument("Positive minimum duration required");
  Curve initial(start,finish,interior,durations); // Check the fixed problem once.
  const int nq=config.optimize_waypoints ? 2*interior.size() : 0;
  const int nt=config.optimize_times ? durations.size() : 0;
  Eigen::VectorXd x(nq+nt);
  for(int k=0;k<nq/2;++k) x.segment<2>(2*k)=interior[k];
  for(int k=0;k<nt;++k) {
    if(durations[k]<=config.min_duration) throw std::invalid_argument("Initial time must exceed Tmin");
    x(nq+k)=std::log(durations[k]-config.min_duration);
  }
  auto decode=[&](const Eigen::VectorXd & z,auto & q,auto & T) {
    q=interior;T=durations;
    for(int k=0;k<nq/2;++k) q[k]=z.segment<2>(2*k);
    for(int k=0;k<nt;++k) T[k]=config.min_duration+std::exp(z(nq+k));
  };
  auto objective=[&](const Eigen::VectorXd & z) {
    ObjectiveValue value;
    try {
      std::vector<Eigen::Vector2d> q;std::vector<double> T;decode(z,q,T);
      Curve curve(start,finish,q,T);
      auto partial=trajectoryCost(curve.trajectory(),config.cost,regions,field);
      if(config.bezier_penalties) {
        const auto extra=bezierCost(curve.trajectory(),regions,config.cost);
        partial.cost+=extra.cost;partial.coefficients+=extra.coefficients;partial.times+=extra.times;
      }
      const auto gradient=curve.propagate(partial.coefficients,partial.times);
      value.value=partial.cost;value.gradient.resize(z.size());
      for(int k=0;k<nq/2;++k) value.gradient.segment<2>(2*k)=gradient.waypoints.row(k).transpose();
      // positive_time_begin
      for(int k=0;k<nt;++k)
        value.gradient(nq+k)=gradient.times(k)*(T[k]-config.min_duration);
      // positive_time_end
    } catch(const std::exception & error) {value.error=error.what();}
    return value;
  };
  TrajectoryOptimizationResult result;result.solver=minimizeBfgs(objective,x,config.solver);
  if(!std::isfinite(result.solver.objective.value)) return result;
  decode(result.solver.x,result.waypoints,result.durations);
  result.curve=Curve(start,finish,result.waypoints,result.durations).trajectory();
  result.samples=checkTrajectorySamples(*result.curve,config.limits,regions,field);
  return result;
}
} // namespace
TrajectoryOptimizationResult optimizeMinco(const TranslationState & a,const TranslationState & b,
 const std::vector<Eigen::Vector2d> & q,const std::vector<double> & T,const TrajectoryOptimizationConfig & c,
 const std::vector<ConvexRegion> & regions,const Esdf2D * field) {return optimizeWith<Minco2D>(a,b,q,T,c,regions,field);}
TrajectoryOptimizationResult optimizeSpline(const TranslationState & a,const TranslationState & b,
 const std::vector<Eigen::Vector2d> & q,const std::vector<double> & T,const TrajectoryOptimizationConfig & c,
 const std::vector<ConvexRegion> & regions,const Esdf2D * field) {return optimizeWith<Spline2D>(a,b,q,T,c,regions,field);}
}
