#include "motion2d/control/ackermann_tracker.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
using namespace motion2d;
int main(int argc,char ** argv) {
  const std::filesystem::path dir=argc>1?argv[1]:"tmp/ackermann_planning";std::filesystem::create_directories(dir);
  const auto region=makeRegion({{{-1,-1},{4,-1},{4,2},{-1,2}}});
  AckermannPlanningConfig c;c.solver.max_iterations=400;c.solver.gradient_tolerance=1e-5;
  const auto result=planAckermann({{{0,0},0},{{2,.3},0}},{5},{region},c);
  std::cout<<result.status<<" solver="<<result.solver.status<<" retiming="<<result.retiming_steps<<'\n';
  if(!result.curve)return 1;
  std::ofstream data(dir/"tracking.csv");data<<"t,x,y,ref_x,ref_y,speed,force,steering,steering_rate,error\n";
  AckermannState x;double peak=0;
  for(double t=0;t<result.curve->duration()+3;t+=.005) {
    const auto ref=result.curve->sample(t,c.limits.vehicle);const auto u=trackAckermann(x,ref,c.limits.vehicle,{},.005);
    const double error=(x.pose.position-ref.state.pose.position).norm();peak=std::max(peak,error);
    data<<t<<','<<x.pose.position.x()<<','<<x.pose.position.y()<<','<<ref.state.pose.position.x()<<','<<ref.state.pose.position.y()<<','<<x.speed<<','<<u.force<<','<<x.steering<<','<<u.steering_rate<<','<<error<<'\n';
    x=stepAckermann(x,u,c.limits.vehicle,.005);
  }
  const auto & b=result.certificate;
  std::cout<<"duration="<<result.curve->duration()<<" bounds speed/force/steering/rate/lateral="<<b.speed<<'/'<<b.force<<'/'<<b.steering<<'/'<<b.steering_rate<<'/'<<b.lateral_acceleration
    <<" peak_tracking_error="<<peak<<" final_error="<<(x.pose.position-Eigen::Vector2d(2,.3)).norm()<<'\n';
  return peak<.03?0:2;
}
