#include "motion2d/optimization/bfgs.hpp"
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace motion2d
{
BfgsResult minimizeBfgs(const std::function<ObjectiveValue(const Eigen::VectorXd &)> & f,
  const Eigen::VectorXd & initial,const BfgsConfig & config)
{
  if(!initial.allFinite() || config.max_iterations<1 || !std::isfinite(config.gradient_tolerance) ||
    config.gradient_tolerance<=0 || !std::isfinite(config.max_wall_seconds) || config.max_wall_seconds<0)
    {throw std::invalid_argument("Invalid BFGS settings");}
  const auto begin=std::chrono::steady_clock::now();
  const auto elapsed=[&](){return std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();};
  const auto finite=[&](const ObjectiveValue & value){return std::isfinite(value.value) &&
    value.gradient.size()==initial.size() && value.gradient.allFinite();};
  BfgsResult result;result.x=initial;result.objective=f(initial);result.evaluations=1;
  if(!finite(result.objective)) {result.status="invalid_initial: "+result.objective.error;result.seconds=elapsed();return result;}
  const int n=initial.size();Eigen::MatrixXd H=Eigen::MatrixXd::Identity(n,n);
  result.status="max_iterations";
  for(int iteration=0;iteration<config.max_iterations;++iteration) {
    if(n==0 || result.objective.gradient.lpNorm<Eigen::Infinity>()<=config.gradient_tolerance)
      {result.status="converged";break;}
    if(config.max_wall_seconds>0 && elapsed()>=config.max_wall_seconds) {result.status="time_limit";break;}
    Eigen::VectorXd direction=-H*result.objective.gradient;
    if(!direction.allFinite() || direction.dot(result.objective.gradient)>=0) {
      H.setIdentity();direction=-result.objective.gradient;
    }
    const double slope=direction.dot(result.objective.gradient);
    double step=1;ObjectiveValue next;Eigen::VectorXd candidate;bool accepted=false;
    for(int trial=0;trial<35;++trial) {
      if(config.max_wall_seconds>0 && elapsed()>=config.max_wall_seconds) {result.status="time_limit";break;}
      candidate=result.x+step*direction;next=f(candidate);++result.evaluations;
      if(finite(next) && next.value<=result.objective.value+1e-4*step*slope) {accepted=true;break;}
      step*=.5;
    }
    if(!accepted) {if(result.status!="time_limit") result.status="line_search_failed";break;}
    // bfgs_update_begin
    const Eigen::VectorXd s=candidate-result.x,y=next.gradient-result.objective.gradient;
    const double curvature=y.dot(s);
    if(curvature>1e-10*s.norm()*y.norm()) {
      const double rho=1/curvature;
      const Eigen::MatrixXd V=Eigen::MatrixXd::Identity(n,n)-rho*s*y.transpose();
      H=V*H*V.transpose()+rho*s*s.transpose();
      H=.5*(H+H.transpose()).eval();
    }
    result.x=candidate;result.objective=std::move(next);result.iterations=iteration+1;
    // bfgs_update_end
  }
  result.seconds=elapsed();return result;
}
}  // namespace motion2d
