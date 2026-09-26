#include "motion2d/trajectory/spline2d.hpp"
#include "motion2d/trajectory/quintic.hpp"
#include "motion2d/trajectory/minco2d.hpp"
#include <Eigen/Cholesky>
#include <cmath>
#include <stdexcept>
namespace motion2d {
namespace {
using Matrix6=Eigen::Matrix<double,6,6>;
Matrix6 endpointMap(double T) {
  Matrix6 E;
  for(int k=0;k<6;++k) {
    TranslationState a,b;TranslationState & state=k<3 ? a : b;
    if(k%3==0) state.position.x()=1;
    if(k%3==1) state.velocity.x()=1;
    if(k%3==2) state.acceleration.x()=1;
    E.col(k)=interpolateQuintic(a,b,T).coefficients.row(0).transpose();
  }
  return E;
}
}
Spline2D::Spline2D(const TranslationState & a,const TranslationState & b,
  const std::vector<Eigen::Vector2d> & q,const std::vector<double> & T):times_(T) {
  const int n=T.size(),m=n-1;
  if(n<1 || q.size()!=std::size_t(m)) throw std::invalid_argument("N times and N-1 positions required");
  states_=Eigen::MatrixX2d::Zero(3*(n+1),2);
  states_.topRows<3>() << a.position.transpose(),a.velocity.transpose(),a.acceleration.transpose();
  states_.bottomRows<3>() << b.position.transpose(),b.velocity.transpose(),b.acceleration.transpose();
  for(int k=0;k<m;++k) states_.row(3*(k+1))=q[k];
  if(!states_.allFinite()) throw std::invalid_argument("Finite spline states required");
  std::vector<Eigen::Matrix2d> diag(m,Eigen::Matrix2d::Zero());
  lower_.resize(m,Eigen::Matrix2d::Zero());inverse_diagonal_.resize(m);
  Eigen::MatrixX2d rhs=Eigen::MatrixX2d::Zero(2*m,2);
  for(int i=0;i<n;++i) {
    if(!std::isfinite(T[i]) || T[i]<=0) throw std::invalid_argument("Positive finite spline durations required");
    const Matrix6 E=endpointMap(T[i]),R=E.transpose()*jerkGram(T[i])*E;
    endpoint_maps_.push_back(E);energy_maps_.push_back(R);
    const Eigen::Matrix<double,6,2> W=states_.middleRows<6>(3*i),r=-R*W;
    for(int side=0;side<2;++side) {
      const int knot=i+side,index=knot-1,offset=3*side+1;
      if(knot>0 && knot<n) {
        diag[index]+=R.block<2,2>(offset,offset);
        rhs.middleRows<2>(2*index)+=r.middleRows<2>(offset);
      }
    }
    if(i>0 && i<n-1) lower_[i]=R.block<2,2>(4,1);
  }
  // spline_blocks_begin
  for(int i=0;i<m;++i) {
    if(i) {
      const Eigen::Matrix2d off=lower_[i];
      lower_[i]=off*inverse_diagonal_[i-1];
      diag[i]-=lower_[i]*off.transpose();
    }
    Eigen::LLT<Eigen::Matrix2d> factor(diag[i]);
    if(factor.info()!=Eigen::Success) throw std::runtime_error("Spline block is not positive definite");
    inverse_diagonal_[i]=factor.solve(Eigen::Matrix2d::Identity());
  }
  const Eigen::MatrixX2d free=solveBlocks(rhs);
  for(int i=0;i<m;++i) states_.middleRows<2>(3*i+4)=free.middleRows<2>(2*i);
  // spline_blocks_end
  coefficients_.resize(6*n,2);
  for(int i=0;i<n;++i) coefficients_.middleRows<6>(6*i)=endpoint_maps_[i]*states_.middleRows<6>(3*i);
  if(!coefficients_.allFinite()) throw std::runtime_error("Nonfinite spline coefficients");
}
Eigen::MatrixX2d Spline2D::solveBlocks(Eigen::MatrixX2d rhs) const {
  const int m=inverse_diagonal_.size();
  for(int i=1;i<m;++i) rhs.middleRows<2>(2*i)-=lower_[i]*rhs.middleRows<2>(2*i-2);
  for(int i=0;i<m;++i) rhs.middleRows<2>(2*i)=inverse_diagonal_[i]*rhs.middleRows<2>(2*i).eval();
  for(int i=m-2;i>=0;--i) rhs.middleRows<2>(2*i)-=lower_[i+1].transpose()*rhs.middleRows<2>(2*i+2);
  return rhs;
}
PolynomialTrajectory Spline2D::trajectory() const {
  std::vector<QuinticPiece> pieces;
  for(std::size_t i=0;i<times_.size();++i) pieces.push_back({times_[i],coefficients_.middleRows<6>(6*i).transpose()});
  return PolynomialTrajectory(std::move(pieces));
}
double Spline2D::energy() const {
  double result=0;
  for(std::size_t i=0;i<times_.size();++i) {
    const auto C=coefficients_.middleRows<6>(6*i);
    result+=(C.array()*(jerkGram(times_[i])*C).array()).sum();
  }
  return result;
}
TrajectoryGradient Spline2D::energyGradient() const {
  const int n=times_.size();TrajectoryGradient g{Eigen::MatrixX2d(n-1,2),Eigen::VectorXd(n)};
  // spline_energy_gradient_begin
  for(int i=0;i<n-1;++i)
    g.waypoints.row(i)=240*(coefficients_.row(6*i+5)-coefficients_.row(6*i+11));
  for(int i=0;i<n;++i) {
    const auto C=coefficients_.middleRows<6>(6*i);
    g.times(i)=-36*C.row(3).squaredNorm()+96*C.row(4).dot(C.row(2))-240*C.row(5).dot(C.row(1));
  }
  // spline_energy_gradient_end
  return g;
}
TrajectoryGradient Spline2D::propagate(const Eigen::MatrixX2d & G,const Eigen::VectorXd & direct) const {
  const int n=times_.size(),m=n-1;
  if(G.rows()!=6*n || direct.size()!=n || !G.allFinite() || !direct.allFinite()) throw std::invalid_argument("Invalid spline gradient dimensions or values");
  Eigen::MatrixX2d local=Eigen::MatrixX2d::Zero(3*(n+1),2),rhs(2*m,2);
  for(int i=0;i<n;++i) local.middleRows<6>(3*i)+=endpoint_maps_[i].transpose()*G.middleRows<6>(6*i);
  for(int k=0;k<m;++k) rhs.middleRows<2>(2*k)=local.middleRows<2>(3*k+4);
  const Eigen::MatrixX2d lambda=solveBlocks(rhs);
  TrajectoryGradient result{Eigen::MatrixX2d(m,2),direct};
  for(int i=0;i<n;++i) {
    Eigen::Matrix<double,6,2> L=Eigen::Matrix<double,6,2>::Zero();
    if(i>0) L.middleRows<2>(1)=lambda.middleRows<2>(2*i-2);
    if(i<n-1) L.middleRows<2>(4)=lambda.middleRows<2>(2*i);
    const Matrix6 & E=endpoint_maps_[i];const double T=times_[i];const Matrix6 Q=jerkGram(T);
    Matrix6 Mt=Matrix6::Zero();for(int d=0;d<3;++d) Mt.row(3+d)=polynomialBasis(T,d+1);
    const Matrix6 Et=-E*Mt*E,Qt=polynomialBasis(T,3).transpose()*polynomialBasis(T,3);
    const Matrix6 Rt=Et.transpose()*Q*E+E.transpose()*Q*Et+E.transpose()*Qt*E;
    const Eigen::Matrix<double,6,2> W=states_.middleRows<6>(3*i);
    result.times(i)+=(G.middleRows<6>(6*i).array()*(Et*W).array()).sum()-(L.array()*(Rt*W).array()).sum();
    local.middleRows<6>(3*i)-=energy_maps_[i]*L;
  }
  for(int k=0;k<m;++k) result.waypoints.row(k)=local.row(3*k+3);
  if(!result.times.allFinite() || !result.waypoints.allFinite()) throw std::runtime_error("Nonfinite spline gradient");
  return result;
}
}
