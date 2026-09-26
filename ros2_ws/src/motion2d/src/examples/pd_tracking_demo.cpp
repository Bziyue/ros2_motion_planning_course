#include "motion2d/control/pd_tracker.hpp"
#include "motion2d/control/tracking_reference.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
using namespace motion2d;
/** @brief Controlled known-reference experiment in an empty 8m square, not navigation. */
int main(int argc,char ** argv) {
  const std::filesystem::path out=argc>1 ? argv[1] : "tmp/ch17_pd";std::filesystem::create_directories(out);
  std::ofstream samples(out/"samples.csv"),summary(out/"summary.csv");
  samples<<"shape,mode,t,x_ref,y_ref,x,y,error,yaw_error,fx,fy,clearance\n";
  summary<<"shape,mode,rms_error,max_error,rms_yaw,force_peak,torque_peak,min_clearance,saturated_updates\n";
  const double dt=.005;const int steps=4800;
  for(int shape=0;shape<2;++shape) for(int mode=0;mode<5;++mode) {
    TrackingReferenceConfig ref;ref.shape=shape ? ReferenceShape::FigureEight : ReferenceShape::Circle;
    InertialParameters plant;plant.linear_drag=.15;plant.angular_drag=.01;
    if(mode==4) plant.force_max=.2;
    auto model=plant;if(mode==3) {model.mass=.6;model.inertia_z=.012;}
    PdGains gains;if(mode==2) gains.feedforward=false;
    State2D state;Wrench2D held;double sum=0,maximum=0,yaw_sum=0,force=0,torque=0,clearance=10;int saturated=0;
    for(int k=0;k<steps;++k) {
      const double t=k*dt;
      if(k%4==0 && mode!=0) {
        const auto command=trackPd(state,sampleTrackingReference(ref,t),gains,model);held=command.applied;
        if(command.saturated) ++saturated;
      }
      const auto desired=sampleTrackingReference(ref,(k+1)*dt);
      state=mode==0 ? desired : stepInertial(state,held,plant,dt);
      const double e=(desired.pose.position-state.pose.position).norm(),ey=wrapAngle(desired.pose.yaw-state.pose.yaw);
      const double free=4-state.pose.position.cwiseAbs().maxCoeff()-.2;
      sum+=e*e;maximum=std::max(maximum,e);yaw_sum+=ey*ey;clearance=std::min(clearance,free);
      force=std::max(force,held.force.cwiseAbs().maxCoeff());torque=std::max(torque,std::abs(held.torque));
      if(k%4==0) samples<<shape<<','<<mode<<','<<(k+1)*dt<<','<<desired.pose.position.x()<<','<<desired.pose.position.y()<<','<<state.pose.position.x()<<','<<state.pose.position.y()<<','<<e<<','<<ey<<','<<held.force.x()<<','<<held.force.y()<<','<<free<<'\n';
    }
    if(mode==0) force=torque=std::numeric_limits<double>::quiet_NaN();
    summary<<shape<<','<<mode<<','<<std::sqrt(sum/steps)<<','<<maximum<<','<<std::sqrt(yaw_sum/steps)<<','<<force<<','<<torque<<','<<clearance<<','<<saturated<<'\n';
    std::cout<<"shape="<<shape<<" mode="<<mode<<" RMS="<<std::sqrt(sum/steps)<<" max="<<maximum<<" clearance="<<clearance<<" force="<<force<<'\n';
  }
}
