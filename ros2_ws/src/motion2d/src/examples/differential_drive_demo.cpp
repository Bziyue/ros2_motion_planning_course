#include "motion2d/sim/differential_drive.hpp"
#include <iomanip>
#include <iostream>
/** @brief Appendix B exact no-slip circle: wheel rates left2/right6 rad/s. */
int main() {
  motion2d::State2D state;std::cout<<"t,x,y,yaw,vx,vy\n"<<std::setprecision(17);
  const double dt=4*motion2d::kPi/400;
  for(int i=0;i<=400;++i) {
    std::cout<<i*dt<<','<<state.pose.position.x()<<','<<state.pose.position.y()<<','<<state.pose.yaw<<','<<state.velocity.x()<<','<<state.velocity.y()<<'\n';
    if(i<400) state=motion2d::stepDifferential(state,2,6,dt);
  }
}
