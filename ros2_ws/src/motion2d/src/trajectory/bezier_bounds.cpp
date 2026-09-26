#include "motion2d/trajectory/bezier_bounds.hpp"
#include <cmath>
#include <stdexcept>
namespace motion2d {
namespace {
double choose(int n,int k) {double v=1;for(int j=1;j<=k;++j) v*=double(n-j+1)/j;return v;}
Eigen::MatrixXd mapDerivative(double T,int order) {
  Eigen::MatrixXd M=bezierMap(T,order);
  for(int k=order;k<6;++k) M.col(k)*=double(k-order)/T;
  return M;
}
using Controls=Eigen::Matrix<double,6,2>;
double vectorBound(const Controls & P,int depth) {
  if(!P.allFinite()) return std::numeric_limits<double>::infinity();
  if(depth==0) return P.rowwise().norm().maxCoeff();
  Controls work=P,left,right;left.row(0)=P.row(0);right.row(5)=P.row(5);
  for(int level=1;level<=5;++level) {
    for(int j=0;j<6-level;++j) work.row(j)=.5*(work.row(j)+work.row(j+1)).eval();
    left.row(level)=work.row(0);right.row(5-level)=work.row(5-level);
  }
  return std::max(vectorBound(left,depth-1),vectorBound(right,depth-1));
}
void bound(const Controls & P,double T,const ConvexRegion & region,int depth,BezierCertificate & result) {
  if(depth>0) {
    Controls work=P,left,right;left.row(0)=work.row(0);right.row(5)=work.row(5);
    for(int level=1;level<=5;++level) {
      for(int j=0;j<6-level;++j) work.row(j)=.5*(work.row(j)+work.row(j+1)).eval();
      left.row(level)=work.row(0);right.row(5-level)=work.row(5-level);
    }
    bound(left,T/2,region,depth-1,result);bound(right,T/2,region,depth-1,result);return;
  }
  const auto increase=[](double & maximum,double value) {
    maximum=std::isfinite(value) ? std::max(maximum,value) : std::numeric_limits<double>::infinity();
  };
  // bezier_certificate_begin
  for(int j=0;j<6;++j) {
    const Eigen::VectorXd h=region.A*P.row(j).transpose()-region.b;
    increase(result.corridor_residual,
      h.allFinite() ? h.maxCoeff() : std::numeric_limits<double>::infinity());
  }
  for(int j=0;j<5;++j) {
    const auto velocity=5/T*(P.row(j+1)-P.row(j));
    increase(result.speed_bound,velocity.norm());
  }
  for(int j=0;j<4;++j) {
    const auto acceleration=20/(T*T)*(P.row(j+2)-2*P.row(j+1)+P.row(j));
    increase(result.acceleration_bound,acceleration.norm());
  }
  // bezier_certificate_end
}
}
Eigen::MatrixXd bezierMap(double T,int order) {
  if(!std::isfinite(T) || T<=0 || order<0 || order>2) throw std::invalid_argument("Positive time and derivative order 0..2 required");
  const int degree=5-order;Eigen::MatrixXd M=Eigen::MatrixXd::Zero(degree+1,6);
  for(int j=0;j<=degree;++j) for(int k=0;k<=j;++k) {
    double factor=1;for(int d=0;d<order;++d) factor*=k+order-d;
    M(j,k+order)=choose(j,k)/choose(degree,k)*factor*std::pow(T,k);
  }
  return M;
}
CoefficientGradient bezierCost(const PolynomialTrajectory & curve,const std::vector<ConvexRegion> & regions,const TrajectoryCostConfig & c) {
  if(regions.size()!=curve.pieces().size()) throw std::invalid_argument("One corridor per piece required");
  const int n=curve.pieces().size();CoefficientGradient g;g.coefficients=Eigen::MatrixX2d::Zero(6*n,2);g.times=Eigen::VectorXd::Zero(n);
  for(int i=0;i<n;++i) {
    const auto & piece=curve.pieces()[i];const Eigen::Matrix<double,6,2> C=piece.coefficients.transpose();
    for(int order=0;order<=2;++order) {
      const auto M=bezierMap(piece.duration,order),Mt=mapDerivative(piece.duration,order);
      const Eigen::MatrixX2d P=M*C,Pt=Mt*C;
      for(int j=0;j<P.rows();++j) {
        Eigen::Vector2d gp=Eigen::Vector2d::Zero();
        const auto penalty=[&](double h,double w){const double v=std::max(0.,h);g.cost+=w*v*v*v;return 3*w*v*v;};
        if(order==0) {
          for(int k=0;k<regions[i].A.rows();++k)
            gp+=penalty(regions[i].A.row(k).dot(P.row(j))-regions[i].b(k)+c.corridor_margin,c.corridor_weight)*regions[i].A.row(k).transpose();
        } else {
          const double limit=order==1 ? c.speed_max : c.acceleration_max,weight=order==1 ? c.speed_weight : c.acceleration_weight;
          gp+=penalty(P.row(j).squaredNorm()-limit*limit,weight)*2*P.row(j).transpose();
        }
        g.coefficients.middleRows<6>(6*i)+=M.row(j).transpose()*gp.transpose();
        g.times(i)+=gp.dot(Pt.row(j));
      }
    }
  }
  if(!std::isfinite(g.cost) || !g.coefficients.allFinite() || !g.times.allFinite()) throw std::runtime_error("Nonfinite Bezier cost");
  return g;
}
ForceCertificate certifyForce(const PolynomialTrajectory & curve,const OmniDynamicLimits & d,int depth) {
  validateDynamicLimits(d);
  if(depth<0 || depth>10) throw std::invalid_argument("Force certificate depth must be 0..10");
  ForceCertificate out;
  for(const auto & piece:curve.pieces()) {
    // force_polynomial_begin
    Eigen::Matrix<double,2,6> F=Eigen::Matrix<double,2,6>::Zero(),rate=F;
    for(int k=0;k<5;++k) F.col(k)+=d.parameters.linear_drag*(k+1)*piece.coefficients.col(k+1);
    for(int k=0;k<4;++k) F.col(k)+=d.parameters.mass*(k+2)*(k+1)*piece.coefficients.col(k+2);
    for(int k=0;k<5;++k) rate.col(k)=(k+1)*F.col(k+1);
    const auto M=bezierMap(piece.duration);
    out.force_bound=std::max(out.force_bound,vectorBound(M*F.transpose(),depth));
    out.force_rate_bound=std::max(out.force_rate_bound,vectorBound(M*rate.transpose(),depth));
    // force_polynomial_end
  }
  out.certified=out.force_bound<=d.force+1e-9 && out.force_rate_bound<=d.force_rate+1e-9;
  return out;
}
BezierCertificate certifyBezier(const PolynomialTrajectory & curve,const std::vector<ConvexRegion> & regions,const TrajectoryLimits & limits,int depth) {
  if(regions.size()!=curve.pieces().size() || depth<0 || depth>10 || !std::isfinite(limits.speed) || limits.speed<=0 ||
     !std::isfinite(limits.acceleration) || limits.acceleration<=0) throw std::invalid_argument("Invalid Bezier certificate input");
  BezierCertificate result;
  for(std::size_t i=0;i<regions.size();++i) {
    const auto & piece=curve.pieces()[i];const Controls P=bezierMap(piece.duration)*piece.coefficients.transpose();
    if(!P.allFinite()) return result;
    bound(P,piece.duration,regions[i],depth,result);
  }
  result.certified=std::isfinite(result.corridor_residual) && std::isfinite(result.speed_bound) && std::isfinite(result.acceleration_bound) &&
    result.corridor_residual<=1e-9 && result.speed_bound<=limits.speed+1e-9 && result.acceleration_bound<=limits.acceleration+1e-9;
  if(limits.dynamics) {
    const auto physical=certifyForce(curve,*limits.dynamics,depth);
    result.force_bound=physical.force_bound;result.force_rate_bound=physical.force_rate_bound;
    result.certified=result.certified && physical.certified;
  }
  return result;
}
}
