#include "motion2d/control/linear_mpc.hpp"
#include <algorithm>
#include <limits>
#include <stdexcept>
namespace motion2d {
namespace {
Eigen::Vector4d vector(const State2D & s) {Eigen::Vector4d x;x<<s.pose.position,s.velocity;return x;}
}
LinearModel linearInertialModel(const InertialParameters & p,double dt) {
  validateInertialParameters(p);
  if(!std::isfinite(dt) || dt<=0) throw std::invalid_argument("Invalid model timestep");
  LinearModel out;
  for(int i=0;i<4;++i) {State2D s;if(i<2) s.pose.position[i]=1.;else s.velocity[i-2]=1.;out.A.col(i)=vector(stepInertial(s,{},p,dt));}
  const double unit=std::min(1.,p.force_max);
  for(int i=0;i<2;++i) {Wrench2D u;u.force[i]=unit;out.B.col(i)=vector(stepInertial({},u,p,dt))/unit;}
  return out;
}
PredictionMatrices predictionMatrices(const LinearModel & m,int n) {
  if(n<1 || n>200 || !m.A.allFinite() || !m.B.allFinite()) throw std::invalid_argument("Invalid prediction horizon/model");
  PredictionMatrices out{Eigen::MatrixXd::Zero(4*n,4),Eigen::MatrixXd::Zero(4*n,2*n)};
  Eigen::Matrix4d power=Eigen::Matrix4d::Identity();
  for(int k=0;k<n;++k) {
    power=m.A*power;out.Sx.block<4,4>(4*k,0)=power;
    if(k) out.Su.middleRows(4*k,4)=m.A*out.Su.middleRows(4*(k-1),4);
    out.Su.block<4,2>(4*k,2*k)=m.B;
  }
  return out;
}
LinearMpc::LinearMpc(const InertialParameters & p,const MpcConfig & c):model_(p),config_(c),prediction_(predictionMatrices(linearInertialModel(p,c.dt),c.horizon)) {
  for(double value:{c.position_weight,c.velocity_weight,c.force_weight,c.velocity_max,c.force_rate_max})
    if(!std::isfinite(value) || value<=0) throw std::invalid_argument("Invalid positive MPC parameter");
  if(!std::isfinite(c.increment_weight) || c.increment_weight<0) throw std::invalid_argument("Invalid increment weight");
  const int n=c.horizon;
  Q_=Eigen::MatrixXd::Zero(4*n,4*n);D_=Eigen::MatrixXd::Identity(2*n,2*n);
  for(int k=0;k<n;++k) {
    Q_.diagonal().segment<4>(4*k)<<c.position_weight,c.position_weight,c.velocity_weight,c.velocity_weight;
    if(k) D_.block<2,2>(2*k,2*(k-1))=-Eigen::Matrix2d::Identity();
  }
  P_=2*(prediction_.Su.transpose()*Q_*prediction_.Su+c.force_weight*Eigen::MatrixXd::Identity(2*n,2*n)+c.increment_weight*D_.transpose()*D_);
}
BoxQp LinearMpc::problem(const State2D & state,const std::vector<State2D> & reference,
  const Eigen::Vector2d & previous,const std::vector<ConvexRegion> & regions) const {
  const int n=config_.horizon;
  if(reference.size()!=std::size_t(n) || (!regions.empty() && regions.size()!=std::size_t(n)) || !vector(state).allFinite() || !previous.allFinite())
    throw std::invalid_argument("MPC expects finite state, input and N future references/regions");
  Eigen::VectorXd target(4*n),feedforward(2*n),d=Eigen::VectorXd::Zero(2*n);d.head<2>()=previous;
  int extra=0;
  for(int k=0;k<n;++k) {
    if(!vector(reference[k]).allFinite() || !reference[k].acceleration.allFinite()) throw std::invalid_argument("Invalid MPC reference");
    target.segment<4>(4*k)=vector(reference[k]);feedforward.segment<2>(2*k)=model_.mass*reference[k].acceleration+model_.linear_drag*reference[k].velocity;
    if(!regions.empty()) {
      const auto & r=regions[k];
      if(r.A.cols()!=2 || r.A.rows()!=r.b.size() || !r.A.allFinite() || !r.b.allFinite()) throw std::invalid_argument("Invalid MPC halfspaces");
      extra+=r.b.size();
    }
  }
  const Eigen::VectorXd free=prediction_.Sx*vector(state);
  BoxQp out;out.P=P_;
  // mpc_condense_begin
  out.q=2*(prediction_.Su.transpose()*Q_*(free-target)
    -config_.force_weight*feedforward-config_.increment_weight*D_.transpose()*d);
  out.A=Eigen::MatrixXd::Zero(6*n+extra,2*n);
  out.lower=Eigen::VectorXd::Constant(6*n+extra,-std::numeric_limits<double>::infinity());
  out.upper=Eigen::VectorXd::Zero(6*n+extra);
  out.A.topRows(2*n).setIdentity();
  out.lower.head(2*n).setConstant(-model_.force_max);out.upper.head(2*n).setConstant(model_.force_max);
  out.A.middleRows(4*n,2*n)=D_;
  out.lower.segment(4*n,2*n)=d.array()-config_.force_rate_max*config_.dt;
  out.upper.segment(4*n,2*n)=d.array()+config_.force_rate_max*config_.dt;
  // mpc_condense_end
  int row=6*n;
  for(int k=0;k<n;++k) {
    out.A.middleRows(2*n+2*k,2)=prediction_.Su.middleRows(4*k+2,2);
    out.lower.segment<2>(2*n+2*k)=Eigen::Vector2d::Constant(-config_.velocity_max)-free.segment<2>(4*k+2);
    out.upper.segment<2>(2*n+2*k)=Eigen::Vector2d::Constant(config_.velocity_max)-free.segment<2>(4*k+2);
    if(!regions.empty()) {
      const auto & r=regions[k];const int count=r.b.size();
      out.A.middleRows(row,count)=r.A*prediction_.Su.middleRows(4*k,2);
      out.upper.segment(row,count)=r.b-r.A*free.segment<2>(4*k);row+=count;
    }
  }
  return out;
}
MpcResult LinearMpc::step(const State2D & state,const std::vector<State2D> & reference,
  const Eigen::Vector2d & previous,const std::vector<ConvexRegion> & regions) {
  const auto qp=problem(state,reference,previous,regions);
  MpcResult result;result.solver=solveBoxQp(qp,config_.solver,warm_);
  if(result.solver.solved()) {
    result.violation=qpViolation(qp,result.solver.x);
    result.force=result.solver.x.head<2>().cwiseMax(-model_.force_max).cwiseMin(model_.force_max);
    result.predicted_states=prediction_.Sx*vector(state)+prediction_.Su*result.solver.x;
    const int n=config_.horizon;
    warm_=result.solver.x;
    if(n>1) warm_.head(2*(n-1))=result.solver.x.tail(2*(n-1));
    warm_.tail<2>()=result.solver.x.tail<2>();
  } else {reset();result.force=dampingBrake(state,model_).force;}
  return result;
}
}
