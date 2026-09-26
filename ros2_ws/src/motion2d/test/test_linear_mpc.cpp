#include <gtest/gtest.h>
#include <Eigen/Cholesky>
#include "motion2d/control/linear_mpc.hpp"
using namespace motion2d;
TEST(BoxQp,UnconstrainedAnalyticAndBoxOptimum) {
  BoxQp p;p.P.resize(2,2);p.P<<4,1,1,2;p.q=Eigen::Vector2d(-1,-3);p.A.resize(0,2);p.lower.resize(0);p.upper.resize(0);
  auto r=solveBoxQp(p);ASSERT_TRUE(r.solved())<<r.status;EXPECT_LT((r.x+p.P.ldlt().solve(p.q)).norm(),1e-5);
  p.P=Eigen::Matrix2d::Identity();p.q=Eigen::Vector2d(-2,3);p.A=Eigen::Matrix2d::Identity();p.lower=Eigen::Vector2d(-1,-1);p.upper=Eigen::Vector2d(1,1);
  r=solveBoxQp(p);ASSERT_TRUE(r.solved())<<r.status;EXPECT_LT((r.x-Eigen::Vector2d(1,-1)).norm(),1e-4);EXPECT_LT(qpViolation(p,r.x),1e-4);
}
TEST(BoxQp,InfeasibleCertificateTimeoutAndNonconvergenceAreDifferent) {
  BoxQp p;p.P=Eigen::MatrixXd::Identity(1,1);p.q=Eigen::VectorXd::Zero(1);p.A=Eigen::MatrixXd::Ones(2,1);
  p.lower=Eigen::Vector2d(1,-10);p.upper=Eigen::Vector2d(10,0);
  EXPECT_EQ(solveBoxQp(p).status,"primal_infeasible");
  QpSettings s;s.max_iterations=1;EXPECT_EQ(solveBoxQp(p,s).status,"max_iterations");
  s.max_wall_seconds=1e-12;EXPECT_EQ(solveBoxQp(p,s).status,"time_limit");
  p.lower[0]=11;EXPECT_EQ(solveBoxQp(p).status,"primal_infeasible");
  p.P(0,0)=-1;EXPECT_THROW(solveBoxQp(p),std::invalid_argument);
}
TEST(LinearModel,ZohMatchesIndependentAnalyticLimitsAndCh04) {
  for(double drag:{0.,1e-10,.15,10.}) for(double dt:{.02,.1}) {
    InertialParameters model;model.mass=2;model.linear_drag=drag;model.force_max=.4;
    const auto m=linearInertialModel(model,dt);
    State2D s;s.pose.position={1,-2};s.velocity={.7,-.2};Wrench2D u;u.force={.3,-.35};
    const auto next=stepInertial(s,u,model,dt);Eigen::Vector4d x;x<<s.pose.position,s.velocity;
    Eigen::Vector4d expected;expected<<next.pose.position,next.velocity;
    EXPECT_LT((m.A*x+m.B*u.force-expected).norm(),1e-13);
    if(drag==0) {EXPECT_NEAR(m.A(0,2),dt,1e-14);EXPECT_NEAR(m.B(0,0),dt*dt/4,1e-14);EXPECT_NEAR(m.B(2,0),dt/2,1e-14);}
    const auto lift=predictionMatrices(m,7);Eigen::VectorXd inputs(14);inputs.setLinSpaced(-.3,.3);
    Eigen::Vector4d direct=x;
    for(int k=0;k<7;++k) {direct=m.A*direct+m.B*inputs.segment<2>(2*k);EXPECT_LT((lift.Sx.middleRows(4*k,4)*x+lift.Su.middleRows(4*k,4)*inputs-direct).norm(),1e-12);}
  }
}
TEST(LinearMpc,ConstraintBlocksAndFiniteHorizonFeasibilityLoss) {
  InertialParameters model;model.linear_drag=.15;model.force_max=.8;
  MpcConfig c;c.horizon=12;c.dt=.1;c.velocity_max=.5;c.force_rate_max=.8;c.solver.absolute_tolerance=1e-6;c.solver.relative_tolerance=1e-6;
  LinearMpc mpc(model,c);State2D s;Eigen::Vector2d previous=Eigen::Vector2d::Zero();
  std::vector<State2D> refs(c.horizon);for(auto & ref:refs) ref.pose.position={1.,.1};
  const auto box=makeRegion({{{-1,-1},{.3,-1},{.3,1},{-1,1}}});std::vector<ConvexRegion> regions(c.horizon,box);
  bool lost_feasibility=false;
  for(int i=0;i<20;++i) {
    const auto qp=mpc.problem(s,refs,previous,regions);const auto result=mpc.step(s,refs,previous,regions);
    if(!result.solver.solved()) {
      ASSERT_EQ(result.solver.status,"primal_infeasible");
      // Independent monotone lower bound: every admissible future x-force is
      // at least max(-Fmax, previous_x-(k+1)*slew*h). Even this sequence leaves
      // the corridor. A finite horizon without a terminal set is not recursive feasibility.
      State2D minimum=s;double force=previous.x();
      for(int k=0;k<c.horizon;++k) {
        force=std::max(-model.force_max,force-c.force_rate_max*c.dt);
        Wrench2D u;u.force.x()=force;minimum=stepInertial(minimum,u,model,c.dt);
      }
      EXPECT_GT(minimum.pose.position.x(),.3001);
      EXPECT_LT((result.force-dampingBrake(s,model).force).norm(),1e-12);
      lost_feasibility=true;break;
    }
    EXPECT_LE(qpViolation(qp,result.solver.x),1e-5);
    EXPECT_LE(result.force.cwiseAbs().maxCoeff(),.8+1e-8);EXPECT_LE((result.force-previous).cwiseAbs().maxCoeff(),.08001);
    for(int k=0;k<c.horizon;++k) {EXPECT_LE(result.predicted_states.segment<2>(4*k+2).cwiseAbs().maxCoeff(),.50001);EXPECT_LE(result.predicted_states[4*k],.30001);}
    Wrench2D u;u.force=result.force;s=stepInertial(s,u,model,c.dt);previous=result.force;
  }
  EXPECT_TRUE(lost_feasibility);
}
TEST(LinearMpc,FailureUsesFreshDampingNotPreviousOptimizedForce) {
  InertialParameters model;MpcConfig c;c.horizon=5;c.solver.max_wall_seconds=1e-12;LinearMpc mpc(model,c);
  State2D state;state.velocity={.1,-.2};std::vector<State2D> refs(c.horizon);
  const auto r=mpc.step(state,refs,{1,1});EXPECT_EQ(r.solver.status,"time_limit");EXPECT_LT((r.force-Eigen::Vector2d(-.2,.4)).norm(),1e-12);EXPECT_EQ(r.predicted_states.size(),0);
}
TEST(LinearMpc,CondensedGradientMatchesExplicitDynamicsCost) {
  InertialParameters model;model.linear_drag=.3;MpcConfig c;c.horizon=3;c.dt=.1;
  LinearMpc mpc(model,c);State2D initial;initial.pose.position={.2,-.1};initial.velocity={.3,.05};
  std::vector<State2D> refs(3);for(int k=0;k<3;++k) {refs[k].pose.position={.4+.02*k,-.3};refs[k].velocity={.2,.1};refs[k].acceleration={.1,-.2};}
  const Eigen::Vector2d previous(.1,-.05);const auto qp=mpc.problem(initial,refs,previous);
  Eigen::VectorXd u(6);u<<.2,-.3,.1,.05,-.2,.1;
  auto cost=[&](const Eigen::VectorXd & values) {
    State2D state=initial;Eigen::Vector2d last=previous;double sum=0;
    for(int k=0;k<3;++k) {Wrench2D command;command.force=values.segment<2>(2*k);state=stepInertial(state,command,model,c.dt);
      sum+=c.position_weight*(state.pose.position-refs[k].pose.position).squaredNorm()+c.velocity_weight*(state.velocity-refs[k].velocity).squaredNorm();
      sum+=c.force_weight*(command.force-model.mass*refs[k].acceleration-model.linear_drag*refs[k].velocity).squaredNorm();
      sum+=c.increment_weight*(command.force-last).squaredNorm();last=command.force;
    }return sum;
  };
  const Eigen::VectorXd gradient=qp.P*u+qp.q;
  for(int j=0;j<6;++j) {Eigen::VectorXd plus=u,minus=u;plus[j]+=1e-6;minus[j]-=1e-6;EXPECT_NEAR(gradient[j],(cost(plus)-cost(minus))/2e-6,1e-7);}
}
