#include "motion2d/trajectory/ackermann_trajectory.hpp"
#include "motion2d/trajectory/quintic.hpp"
#include "motion2d/trajectory/bezier_bounds.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace motion2d {
ProgressSample stopProgress(double time,double duration) {
  const double r=std::clamp(time/duration,0.,1.);
  return {r*r*r*(10+r*(-15+6*r)),30*r*r*(1-r)*(1-r)/duration,
    60*r*(1-r)*(1-2*r)/(duration*duration)};
}
// ackermann_warp_begin
AckermannFlatOutput warpAckermann(const AckermannFlatOutput & q,const ProgressSample & h,const AckermannParameters & p) {
  auto y=q;y.speed=q.speed*h.rate;y.yaw_rate=q.yaw_rate*h.rate;
  y.longitudinal_acceleration=q.longitudinal_acceleration*h.rate*h.rate+q.speed*h.acceleration;
  y.force=p.mass*y.longitudinal_acceleration+p.linear_drag*y.speed;
  y.steering_rate=q.steering_rate*h.rate;
  y.lateral_acceleration=q.lateral_acceleration*h.rate*h.rate;return y;
}
// ackermann_warp_end
WarpGradient warpAckermannBackward(const AckermannFlatOutput & q,const ProgressSample & h,
  const AckermannFlatOutput & g,const AckermannParameters & p) {
  WarpGradient out;auto & z=out.geometry;
  const double gs=g.speed+p.linear_drag*g.force,ga=g.longitudinal_acceleration+p.mass*g.force;
  z.speed=gs*h.rate+ga*h.acceleration;z.longitudinal_acceleration=ga*h.rate*h.rate;
  z.yaw=g.yaw;z.curvature=g.curvature;z.steering=g.steering;
  z.yaw_rate=g.yaw_rate*h.rate;z.steering_rate=g.steering_rate*h.rate;
  z.lateral_acceleration=g.lateral_acceleration*h.rate*h.rate;
  out.rate=gs*q.speed+2*ga*q.longitudinal_acceleration*h.rate+g.yaw_rate*q.yaw_rate+
    g.steering_rate*q.steering_rate+2*g.lateral_acceleration*q.lateral_acceleration*h.rate;
  out.acceleration=ga*q.speed;return out;
}
AckermannTrajectory::AckermannTrajectory(std::vector<AckermannPiece> pieces):pieces_(std::move(pieces)) {
  if(pieces_.empty())throw std::invalid_argument("Ackermann trajectory needs pieces");
  for(std::size_t i=0;i<pieces_.size();++i) {
    const auto & p=pieces_[i];
    if(p.geometry.duration!=1 || !p.geometry.coefficients.allFinite() || !std::isfinite(p.duration) || p.duration<=0)
      throw std::invalid_argument("Expected unit geometric interval and finite positive physical duration");
    const auto v0=derivative(p.geometry,0,1),v1=derivative(p.geometry,1,1);
    if(v0.norm()<1e-6 || v1.norm()<1e-6 || derivative(p.geometry,0,2).norm()>1e-7 || derivative(p.geometry,1,2).norm()>1e-7)
      throw std::invalid_argument("Stop pieces need regular endpoints and zero geometric endpoint acceleration");
    if(i && ((derivative(pieces_[i-1].geometry,1,0)-derivative(p.geometry,0,0)).norm()>1e-7 ||
      (derivative(pieces_[i-1].geometry,1,1).normalized()-v0.normalized()).norm()>1e-7))
      throw std::invalid_argument("Ackermann join must preserve position and heading");
    duration_+=p.duration;
  }
}
AckermannReference AckermannTrajectory::sample(double time,const AckermannParameters & p) const {
  if(!std::isfinite(time))throw std::invalid_argument("Finite sample time required");
  time=std::clamp(time,0.,duration_);std::size_t i=0;
  while(i+1<pieces_.size() && time>pieces_[i].duration)time-=pieces_[i++].duration;
  const auto & piece=pieces_[i];const auto h=stopProgress(time,piece.duration);
  const auto & q=piece.geometry;
  const auto geometry=ackermannForward({derivative(q,h.u,1),derivative(q,h.u,2),derivative(q,h.u,3)},p);
  const auto physical=warpAckermann(geometry,h,p);
  const AckermannState state{{derivative(q,h.u,0),physical.yaw},physical.speed,physical.steering};
  return {ackermannKinematics(state,{physical.force,physical.steering_rate},p),physical};
}
void validateAckermannLimits(const AckermannLimits & l) {
  validateAckermannParameters(l.vehicle);
  for(double x:{l.speed,l.force,l.steering,l.steering_rate,l.lateral_acceleration})
    if(!std::isfinite(x) || x<=0)throw std::invalid_argument("Positive finite Ackermann planning limits required");
  if(l.force>l.vehicle.force_max || l.steering>l.vehicle.steering_max || l.steering_rate>l.vehicle.steering_rate_max)
    throw std::invalid_argument("Planning limits must not exceed actuators");
}
AckermannPieceGradient ackermannPieceCost(const AckermannPiece & piece,const AckermannLimits & l,
  const ConvexRegion * region,int steps) {
  AckermannPieceGradient out;out.cost=piece.duration;out.time=1;
  for(int k=0;k<=steps;++k) {
    const double alpha=double(k)/steps,w=(k==0 || k==steps)?.5:1,dt=piece.duration/steps;
    const auto h=stopProgress(alpha*piece.duration,piece.duration);const auto & q=piece.geometry;
    const AckermannFlatInput x{derivative(q,h.u,1),derivative(q,h.u,2),derivative(q,h.u,3)};
    const auto geometry=ackermannForward(x,l.vehicle),physical=warpAckermann(geometry,h,l.vehicle);
    double cost=0;AckermannFlatOutput g;
    const auto penalty=[&](double value,double bound){
      const double target=.85*bound,r=std::max(0.,value*value/(target*target)-1);
      cost+=100*r*r*r;return 600*r*r*value/(target*target);
    };
    g.speed=penalty(physical.speed,l.speed);g.force=penalty(physical.force,l.force);
    g.steering=penalty(physical.steering,l.steering);g.steering_rate=penalty(physical.steering_rate,l.steering_rate);
    g.lateral_acceleration=penalty(physical.lateral_acceleration,l.lateral_acceleration);
    Eigen::Vector2d gp=Eigen::Vector2d::Zero();
    if(region)for(int j=0;j<region->A.rows();++j) {
      const double r=std::max(0.,region->A.row(j).dot(derivative(q,h.u,0))-region->b(j)+.02);
      cost+=1000*r*r*r;gp+=3000*r*r*region->A.row(j).transpose();
    }
    // ackermann_cost_reverse_begin
    const auto warped=warpAckermannBackward(geometry,h,g,l.vehicle);
    const auto flat=ackermannBackward(x,warped.geometry,l.vehicle);
    out.cost+=w*dt*cost;
    out.coefficients+=w*dt*(polynomialBasis(h.u,0).transpose()*gp.transpose()+
      polynomialBasis(h.u,1).transpose()*flat.velocity.transpose()+
      polynomialBasis(h.u,2).transpose()*flat.acceleration.transpose()+
      polynomialBasis(h.u,3).transpose()*flat.jerk.transpose());
    // At fixed alpha, u=h(alpha) is fixed; only rate and acceleration depend on T.
    out.time+=w*(cost/steps-dt/piece.duration*(warped.rate*h.rate+2*warped.acceleration*h.acceleration));
    // ackermann_cost_reverse_end
  }
  return out;
}
namespace {
using Controls=Eigen::Matrix<double,6,2>;
double cross(const Eigen::Vector2d & a,const Eigen::Vector2d & b){return a.x()*b.y()-a.y()*b.x();}
void bounds(const Controls & P,double du,double T,const ConvexRegion & region,const AckermannLimits & l,
  int depth,AckermannCertificate & out) {
  if(!P.allFinite()){out.min_tangent=0;out.corridor_residual=INFINITY;return;}
  if(depth>0) {
    Controls work=P,left,right;left.row(0)=P.row(0);right.row(5)=P.row(5);
    for(int level=1;level<=5;++level) {
      for(int j=0;j<6-level;++j)work.row(j)=.5*(work.row(j)+work.row(j+1)).eval();
      left.row(level)=work.row(0);right.row(5-level)=work.row(5-level);
    }
    bounds(left,du/2,T,region,l,depth-1,out);bounds(right,du/2,T,region,l,depth-1,out);return;
  }
  for(int i=0;i<6;++i)out.corridor_residual=std::max(out.corridor_residual,(region.A*P.row(i).transpose()-region.b).maxCoeff());
  Eigen::Matrix<double,5,2> V;Eigen::Matrix<double,4,2>A;Eigen::Matrix<double,3,2>J;
  for(int i=0;i<5;++i)V.row(i)=5/du*(P.row(i+1)-P.row(i));
  for(int i=0;i<4;++i)A.row(i)=4/du*(V.row(i+1)-V.row(i));
  for(int i=0;i<3;++i)J.row(i)=3/du*(A.row(i+1)-A.row(i));
  const Eigen::Vector2d chord=(P.row(5)-P.row(0)).transpose();
  const double nmin=chord.norm()>1e-12?(V*chord.normalized()).minCoeff():0;
  out.min_tangent=std::min(out.min_tangent,nmin);
  if(nmin<=1e-6)return;
  const double nmax=V.rowwise().norm().maxCoeff();double h=0,b=0,hp=0;
  for(int i=0;i<5;++i) {
    for(int j=0;j<4;++j) {h=std::max(h,std::abs(cross(V.row(i),A.row(j))));b=std::max(b,std::abs(V.row(i).dot(A.row(j))));}
    for(int j=0;j<3;++j)hp=std::max(hp,std::abs(cross(V.row(i),J.row(j))));
  }
  const double d=1.875/T,e=10/std::sqrt(3.)/(T*T),L=l.vehicle.wheelbase;
  out.speed=std::max(out.speed,nmax*d);
  out.steering=std::max(out.steering,std::atan(L*h/std::pow(nmin,3)));
  out.steering_rate=std::max(out.steering_rate,L*(hp/std::pow(nmin,3)+3*h*b/std::pow(nmin,5))*d);
  out.lateral_acceleration=std::max(out.lateral_acceleration,h/nmin*d*d);
  out.force=std::max(out.force,l.vehicle.mass*(b/nmin*d*d+nmax*e)+l.vehicle.linear_drag*nmax*d);
}
}
AckermannCertificate certifyAckermann(const AckermannTrajectory & trajectory,const std::vector<ConvexRegion> & regions,
  const AckermannLimits & l,int depth) {
  validateAckermannLimits(l);
  if(regions.size()!=trajectory.pieces().size() || depth<0 || depth>10)throw std::invalid_argument("One region per piece and depth 0..10 required");
  AckermannCertificate out;
  for(std::size_t i=0;i<regions.size();++i) {
    const auto & p=trajectory.pieces()[i];bounds(bezierMap(1)*p.geometry.coefficients.transpose(),1,p.duration,regions[i],l,depth,out);
  }
  out.geometry_valid=out.min_tangent>1e-6 && out.corridor_residual<=1e-9 && out.steering<=l.steering+1e-9;
  out.certified=out.geometry_valid && out.speed<=l.speed+1e-9 && out.force<=l.force+1e-9 &&
    out.steering_rate<=l.steering_rate+1e-9 && out.lateral_acceleration<=l.lateral_acceleration+1e-9;
  return out;
}
AckermannPlan planAckermann(const std::vector<Pose2D> & knots,const std::vector<double> & times,
  const std::vector<ConvexRegion> & regions,const AckermannPlanningConfig & config) {
  validateAckermannLimits(config.limits);const int n=times.size();
  if(n<1 || knots.size()!=std::size_t(n+1) || regions.size()!=std::size_t(n) || config.quadrature_steps<4 || config.quadrature_steps>1000)
    throw std::invalid_argument("Ackermann planning requires knots, times, verified regions and quadrature steps");
  std::vector<double> lengths(n+1);std::vector<Eigen::Vector2d> tangent(n+1);
  for(int i=0;i<=n;++i) {
    if(!knots[i].position.allFinite() || !std::isfinite(knots[i].yaw))throw std::invalid_argument("Finite knot poses required");
    lengths[i]=i==0?(knots[1].position-knots[0].position).norm():i==n?(knots[n].position-knots[n-1].position).norm():
      .5*((knots[i].position-knots[i-1].position).norm()+(knots[i+1].position-knots[i].position).norm());
    if(lengths[i]<=.05)throw std::invalid_argument("Distinct Ackermann knots required");
    tangent[i]={std::cos(knots[i].yaw),std::sin(knots[i].yaw)};
  }
  Eigen::VectorXd initial(2*n+1);
  for(int i=0;i<=n;++i)initial(i)=std::log(lengths[i]);
  for(int i=0;i<n;++i) {
    if(!std::isfinite(times[i]) || times[i]<=.1)throw std::invalid_argument("Initial time must exceed 0.1 s");
    initial(n+1+i)=std::log(times[i]-.1);
  }
  const auto decode=[&](const Eigen::VectorXd & x) {
    std::vector<AckermannPiece> pieces;
    for(int i=0;i<n;++i) {
      TranslationState a,b;a.position=knots[i].position;b.position=knots[i+1].position;
      a.velocity=std::exp(x(i))*tangent[i];b.velocity=std::exp(x(i+1))*tangent[i+1];
      pieces.push_back({interpolateQuintic(a,b,1),.1+std::exp(x(n+1+i))});
    }
    return AckermannTrajectory(std::move(pieces));
  };
  const auto objective=[&](const Eigen::VectorXd & x) {
    ObjectiveValue out;out.value=0;out.gradient=Eigen::VectorXd::Zero(x.size());
    try {
      const auto curve=decode(x);
      for(int i=0;i<=n;++i) {
        const double ell=std::exp(x(i)),error=ell-lengths[i];out.value+=.02*error*error;out.gradient(i)+=.04*error*ell;
      }
      for(int i=0;i<n;++i) {
        const auto g=ackermannPieceCost(curve.pieces()[i],config.limits,&regions[i],config.quadrature_steps);
        const auto & C=g.coefficients;out.value+=g.cost;
        out.gradient(i)+=(C.row(1)-6*C.row(3)+8*C.row(4)-3*C.row(5)).dot(tangent[i])*std::exp(x(i));
        out.gradient(i+1)+=(-4*C.row(3)+7*C.row(4)-3*C.row(5)).dot(tangent[i+1])*std::exp(x(i+1));
        out.gradient(n+1+i)+=g.time*std::exp(x(n+1+i));
      }
    }catch(const std::exception & e){out.error=e.what();out.value=INFINITY;}
    return out;
  };
  AckermannPlan result;result.solver=minimizeBfgs(objective,initial,config.solver);
  if(!result.solver.converged()){result.status="optimizer:"+result.solver.status;return result;}
  auto curve=decode(result.solver.x);
  for(int attempt=0;attempt<=20;++attempt) {
    result.certificate=certifyAckermann(curve,regions,config.limits);
    if(!result.certificate.geometry_valid){result.status="geometry_or_steering_not_certified";return result;}
    if(result.certificate.certified) {
      result.curve=std::move(curve);result.status=attempt?"certified_retimed":"certified";result.retiming_steps=attempt;return result;
    }
    auto pieces=curve.pieces();for(auto & p:pieces)p.duration*=1.2;curve=AckermannTrajectory(std::move(pieces));
  }
  result.status="dynamic_bounds_not_certified";return result;
}
}
