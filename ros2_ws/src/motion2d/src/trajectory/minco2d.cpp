#include "motion2d/trajectory/minco2d.hpp"
#include <cmath>
#include <stdexcept>

namespace motion2d
{

Eigen::Matrix<double,6,6> jerkGram(double T)
{
  if(!std::isfinite(T) || T<=0) {throw std::invalid_argument("Positive jerk duration required");}
  Eigen::Matrix<double,6,6> Q=Eigen::Matrix<double,6,6>::Zero();
  for(int k=3;k<6;++k) for(int l=3;l<6;++l) {
    Q(k,l)=k*(k-1)*(k-2)*l*(l-1)*(l-2)*std::pow(T,k+l-5)/(k+l-5);
  }
  return Q;
}

Minco2D::Minco2D(const TranslationState & start,const TranslationState & finish,
  const std::vector<Eigen::Vector2d> & interior,const std::vector<double> & durations)
: durations_(durations),factor_(6*static_cast<int>(durations.size()),6)
{
  const int n=durations_.size();
  if(interior.size()+1!=durations_.size() || !start.position.allFinite() || !start.velocity.allFinite() ||
    !start.acceleration.allFinite() || !finish.position.allFinite() || !finish.velocity.allFinite() ||
    !finish.acceleration.allFinite()) {throw std::invalid_argument("Invalid MINCO boundaries or sizes");}
  for(double T:durations_) if(!std::isfinite(T) || T<=0) {throw std::invalid_argument("Positive MINCO durations required");}
  for(const auto & p:interior) if(!p.allFinite()) {throw std::invalid_argument("Nonfinite MINCO waypoint");}
  Eigen::MatrixX2d rhs=Eigen::MatrixX2d::Zero(6*n,2);
  const auto insert=[this](int row,int segment,const Eigen::Matrix<double,1,6> & b,double sign=1.) {
    for(int k=0;k<6;++k) if(b(k)!=0) {factor_.at(row,6*segment+k)+=sign*b(k);}
  };
  rhs.row(0)=start.position.transpose();rhs.row(1)=start.velocity.transpose();rhs.row(2)=start.acceleration.transpose();
  for(int d=0;d<3;++d) {insert(d,0,polynomialBasis(0,d));}
  // minco_conditions_begin
  for(int i=0;i<n-1;++i) {
    const double T=durations_[i]; const int r=6*i+3;
    insert(r,i,polynomialBasis(T,3)); insert(r,i+1,polynomialBasis(0,3),-1);     // jerk continuity
    insert(r+1,i,polynomialBasis(T,4)); insert(r+1,i+1,polynomialBasis(0,4),-1); // snap continuity
    insert(r+2,i,polynomialBasis(T,0)); rhs.row(r+2)=interior[i].transpose();
    for(int d=0;d<3;++d) {
      insert(r+3+d,i,polynomialBasis(T,d)); insert(r+3+d,i+1,polynomialBasis(0,d),-1);
    }
  }
  // minco_conditions_end
  const int last=6*n-3;
  rhs.row(last)=finish.position.transpose();rhs.row(last+1)=finish.velocity.transpose();
  rhs.row(last+2)=finish.acceleration.transpose();
  for(int d=0;d<3;++d) {insert(last+d,n-1,polynomialBasis(durations_.back(),d));}
  factor_.factor(); coefficients_=factor_.solve(rhs);
}

PolynomialTrajectory Minco2D::trajectory() const
{
  std::vector<QuinticPiece> pieces;
  for(std::size_t i=0;i<durations_.size();++i)
    {pieces.push_back({durations_[i],coefficients_.middleRows<6>(6*i).transpose()});}
  return PolynomialTrajectory(std::move(pieces));
}

CoefficientGradient Minco2D::energyPartials() const
{
  const int n=durations_.size(); CoefficientGradient result;
  result.coefficients=Eigen::MatrixX2d::Zero(6*n,2);result.times=Eigen::VectorXd::Zero(n);
  for(int i=0;i<n;++i) {
    const auto C=coefficients_.middleRows<6>(6*i); const auto Q=jerkGram(durations_[i]);
    result.cost+=(C.array()*(Q*C).array()).sum();
    result.coefficients.middleRows<6>(6*i)=2*Q*C;
    result.times(i)=(polynomialBasis(durations_[i],3)*C).squaredNorm();
  }
  return result;
}

TrajectoryGradient Minco2D::propagate(const Eigen::MatrixX2d & gradient,const Eigen::VectorXd & direct) const
{
  const int n=durations_.size();
  if(gradient.rows()!=6*n || direct.size()!=n || !gradient.allFinite() || !direct.allFinite())
  {throw std::invalid_argument("Invalid MINCO cost partials");}
  // minco_adjoint_begin
  const Eigen::MatrixX2d lambda=factor_.solve(gradient,true);
  TrajectoryGradient result{Eigen::MatrixX2d::Zero(n-1,2),direct};
  for(int i=0;i<n;++i) {
    const auto C=coefficients_.middleRows<6>(6*i); const double T=durations_[i];
    const int row=i<n-1 ? 6*i+3 : 6*n-3;
    const std::vector<int> orders=i<n-1 ? std::vector<int>{4,5,1,1,2,3} : std::vector<int>{1,2,3};
    for(std::size_t j=0;j<orders.size();++j) {
      result.times(i)-=lambda.row(row+j).dot(polynomialBasis(T,orders[j])*C);
    }
    if(i<n-1) {result.waypoints.row(i)=lambda.row(6*i+5);}
  }
  return result;
  // minco_adjoint_end
}
}  // namespace motion2d
