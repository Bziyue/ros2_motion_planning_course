#include "motion2d/dynamics/omni_flatness.hpp"
#include <iostream>
int main()
{
  motion2d::InertialParameters p;p.mass=2;p.linear_drag=.3;
  motion2d::OmniFlatInput x{{.8,.1},{.4,-.2},{-.3,.5},0,0};
  const auto y=motion2d::omniForward(x,p);
  motion2d::OmniFlatOutput partials;partials.force=y.force;
  const auto g=motion2d::omniBackward(partials,p);
  std::cout<<"F [N] = "<<y.force.transpose()<<"\ndF/dt [N/s] = "<<y.force_rate.transpose()
    <<"\nFor J=0.5*|F|^2: dJ/dv = "<<g.velocity.transpose()
    <<", dJ/da = "<<g.acceleration.transpose()<<'\n';
}
