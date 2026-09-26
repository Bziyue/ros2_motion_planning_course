#include "motion2d/dynamics/ackermann_flatness.hpp"
#include <iostream>
int main() {
  motion2d::AckermannParameters p;const double speed=.6,radius=1.2,omega=speed/radius;
  const motion2d::AckermannFlatInput jet{{speed,0},{0,speed*omega},{-speed*omega*omega,0}};
  const auto ref=motion2d::ackermannForward(jet,p);
  motion2d::AckermannState x{{{0,0},ref.yaw},ref.speed,ref.steering};
  for(int k=0;k<1000;++k)x=motion2d::stepAckermann(x,{ref.force,ref.steering_rate},p,.005);
  const Eigen::Vector2d exact(radius*std::sin(omega*5),radius*(1-std::cos(omega*5)));
  const double error=(x.pose.position-exact).norm();
  std::cout<<"steering="<<ref.steering<<" rad force="<<ref.force<<" N, circular integration error="<<error<<" m\n";
  return error<1e-9?0:1;
}
