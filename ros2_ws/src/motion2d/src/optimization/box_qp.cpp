#include "motion2d/optimization/box_qp.hpp"
#include <Eigen/Cholesky>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
namespace motion2d {
namespace {
double norm(const Eigen::VectorXd & v) {return v.size() ? v.cwiseAbs().maxCoeff() : 0.;}
bool infeasibleCertificate(const BoxQp & p,const Eigen::VectorXd & delta) {
  const double scale=norm(delta);if(scale<1e-9) return false;
  const Eigen::VectorXd v=delta/scale;
  if(norm(p.A.transpose()*v)>1e-7) return false;
  double support=0;
  for(int i=0;i<v.size();++i) {
    if(v[i]==0) continue;
    const double bound=v[i]>0 ? p.upper[i] : p.lower[i];
    if(!std::isfinite(bound)) return false;
    support+=bound*v[i];
  }
  return support < -1e-7;
}
}
double qpViolation(const BoxQp & p,const Eigen::VectorXd & x) {
  const Eigen::VectorXd a=p.A*x;
  return a.size() ? std::max({0.,(p.lower-a).maxCoeff(),(a-p.upper).maxCoeff()}) : 0.;
}
QpResult solveBoxQp(const BoxQp & p,const QpSettings & s,const Eigen::VectorXd & initial) {
  const auto begin=std::chrono::steady_clock::now();
  const auto elapsed=[&]{return std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();};
  const int n=p.q.size(),m=p.lower.size();
  if(n==0 || p.P.rows()!=n || p.P.cols()!=n || p.A.rows()!=m || p.A.cols()!=n || p.upper.size()!=m ||
    !p.P.allFinite() || !p.A.allFinite() || !p.q.allFinite() || !p.P.isApprox(p.P.transpose(),1e-12) ||
    s.max_iterations<=0 || !std::isfinite(s.absolute_tolerance) || s.absolute_tolerance<=0 ||
    !std::isfinite(s.relative_tolerance) || s.relative_tolerance<0 || !std::isfinite(s.rho) || s.rho<=0 ||
    !std::isfinite(s.sigma) || s.sigma<=0 || !std::isfinite(s.max_wall_seconds) || s.max_wall_seconds<0 ||
    (initial.size() && (initial.size()!=n || !initial.allFinite()))) throw std::invalid_argument("Invalid dense QP or settings");
  Eigen::LDLT<Eigen::MatrixXd> positive(p.P);
  if(positive.info()!=Eigen::Success || (positive.vectorD().array()<=0).any()) throw std::invalid_argument("QP needs positive definite P");
  QpResult result;result.x=initial.size() ? initial : Eigen::VectorXd::Zero(n);
  for(int i=0;i<m;++i) {
    if(std::isnan(p.lower[i]) || std::isnan(p.upper[i]) || p.lower[i]==std::numeric_limits<double>::infinity() ||
      p.upper[i]==-std::numeric_limits<double>::infinity()) throw std::invalid_argument("Invalid QP bounds");
    if(p.lower[i]>p.upper[i]) {result.status="primal_infeasible";result.seconds=elapsed();return result;}
  }
  double rho=s.rho;
  const Eigen::MatrixXd gram=p.A.transpose()*p.A;
  Eigen::LDLT<Eigen::MatrixXd> factor;
  auto refactor=[&]{Eigen::MatrixXd K=p.P+rho*gram;K.diagonal().array()+=s.sigma;factor.compute(K);};
  refactor();
  Eigen::VectorXd z=(p.A*result.x).cwiseMax(p.lower).cwiseMin(p.upper),y=Eigen::VectorXd::Zero(m);
  result.status="max_iterations";
  for(int k=0;k<s.max_iterations;++k) {
    if(s.max_wall_seconds>0 && elapsed()>=s.max_wall_seconds) {result.status="time_limit";break;}
    // qp_admm_begin
    const Eigen::VectorXd old_y=y;
    result.x=factor.solve(s.sigma*result.x-p.q+p.A.transpose()*(rho*z-y));
    const Eigen::VectorXd ax=p.A*result.x;
    z=(ax+y/rho).cwiseMax(p.lower).cwiseMin(p.upper);
    y+=rho*(ax-z);
    result.primal_residual=norm(ax-z);
    result.dual_residual=norm(p.P*result.x+p.q+p.A.transpose()*y);
    // qp_admm_end
    result.iterations=k+1;
    if(!result.x.allFinite() || !y.allFinite()) {result.status="numerical_failure";break;}
    const double eps_p=s.absolute_tolerance+s.relative_tolerance*std::max(norm(ax),norm(z));
    const double eps_d=s.absolute_tolerance+s.relative_tolerance*std::max({norm(p.P*result.x),norm(p.q),norm(p.A.transpose()*y)});
    if(result.primal_residual<=eps_p && result.dual_residual<=eps_d) {result.status="solved";break;}
    if((k+1)%25==0) {
      if(infeasibleCertificate(p,y-old_y)) {result.status="primal_infeasible";break;}
      const double normalized_p=result.primal_residual/std::max({norm(ax),norm(z),1e-12});
      const double normalized_d=result.dual_residual/std::max({norm(p.P*result.x),norm(p.q),norm(p.A.transpose()*y),1e-12});
      const double candidate=std::clamp(rho*std::sqrt(std::max(normalized_p,1e-12)/std::max(normalized_d,1e-12)),1e-5,1e5);
      if(candidate>rho*5 || candidate<rho/5) {rho=candidate;refactor();}
    }
  }
  result.seconds=elapsed();return result;
}
}
