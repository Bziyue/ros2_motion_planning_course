#include "motion2d/control/linear_mpc.hpp"
#include "motion2d/control/tracking_reference.hpp"
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
using namespace motion2d;
/** @brief Same 24s eight reference; cost and constraints fixed within each comparison. */
int main(int argc,char ** argv) {
  const std::filesystem::path out=argc>1 ? argv[1] : "tmp/ch18_mpc";std::filesystem::create_directories(out);
  std::ofstream samples(out/"samples.csv"),summary(out/"summary.csv");
  std::ofstream failed(out/"failures.csv");failed<<"mode,t,status,primal,dual,new_fx,new_fy\n";
  samples<<"mode,t,x_ref,y_ref,x,y,error,fx,fy,solve_ms,success\n";
  summary<<"mode,horizon,control_hz,force_limit,rms_error,max_error,force_peak,speed_axis_peak,slew_peak,min_clearance,solve_median_ms,solve_p95_ms,solve_max_ms,failures\n";
  for(int mode=0;mode<7;++mode) {
    const bool pd=mode==0 || mode==5;const bool limited=mode>=5;
    const int stride=mode==4 ? 10 : 4;
    InertialParameters model;model.linear_drag=.15;model.angular_drag=.01;model.force_max=limited ? .35 : 2;
    MpcConfig config;config.horizon=mode==1 ? 10 : mode==3 ? 40 : 20;config.dt=stride*.005;
    config.velocity_max=limited ? .5 : 1.;config.force_rate_max=limited ? 2 : 5;
    LinearMpc mpc(model,config);TrackingReferenceConfig ref;ref.shape=ReferenceShape::FigureEight;PdGains gains;
    State2D state;Wrench2D held;std::vector<double> times;double sum=0,maximum=0,force=0,speed=0,slew=0,clearance=10,last_ms=0;int failures=0;bool success=true;
    for(int k=0;k<4800;++k) {
      const double t=k*.005;
      if(k%stride==0) {
        const auto old=held;const auto current=sampleTrackingReference(ref,t);const auto command=trackPd(state,current,gains,model);
        held=command.applied;success=true;
        if(!pd) {
          std::vector<State2D> future;for(int j=1;j<=config.horizon;++j) future.push_back(sampleTrackingReference(ref,t+j*config.dt));
          const auto begin=std::chrono::steady_clock::now();const auto result=mpc.step(state,future,old.force);
          last_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();times.push_back(last_ms);
          held.force=result.force;success=result.solver.solved();
          if(!success) {
            ++failures;held.torque=dampingBrake(state,model).torque;
            failed<<mode<<','<<t<<','<<result.solver.status<<','<<result.solver.primal_residual<<','<<result.solver.dual_residual<<','<<held.force.x()<<','<<held.force.y()<<'\n';
          }
        }
        slew=std::max(slew,(held.force-old.force).cwiseAbs().maxCoeff()/config.dt);
      }
      state=stepInertial(state,held,model,.005);const auto desired=sampleTrackingReference(ref,(k+1)*.005);
      const double error=(desired.pose.position-state.pose.position).norm();sum+=error*error;maximum=std::max(maximum,error);
      force=std::max(force,held.force.cwiseAbs().maxCoeff());speed=std::max(speed,state.velocity.cwiseAbs().maxCoeff());
      clearance=std::min(clearance,4-state.pose.position.cwiseAbs().maxCoeff()-.2);
      if(k%stride==0) samples<<mode<<','<<(k+1)*.005<<','<<desired.pose.position.x()<<','<<desired.pose.position.y()<<','<<state.pose.position.x()<<','<<state.pose.position.y()<<','<<error<<','<<held.force.x()<<','<<held.force.y()<<','<<last_ms<<','<<success<<'\n';
    }
    std::sort(times.begin(),times.end());const auto quantile=[&](double q){return times.empty() ? 0. : times[std::size_t(q*(times.size()-1))];};
    summary<<mode<<','<<(pd ? 0 : config.horizon)<<','<<1/config.dt<<','<<model.force_max<<','<<std::sqrt(sum/4800)<<','<<maximum<<','<<force<<','<<speed<<','<<slew<<','<<clearance<<','<<quantile(.5)<<','<<quantile(.95)<<','<<quantile(1)<<','<<failures<<'\n';
    std::cout<<"mode="<<mode<<" RMS="<<std::sqrt(sum/4800)<<" max="<<maximum<<" solve_p95_ms="<<quantile(.95)<<" failures="<<failures<<'\n';
  }
}
