#include "point_jacobian.hpp"
#include <cmath>
#include <iostream>
/** @brief Independent central-difference experiment with metre/radian perturbations. */
int main() {
  const Eigen::Vector2d point{2,-1};const Eigen::Vector3d pose{.3,-.8,.7};
  auto transform=[&](const Eigen::Vector3d & p)->Eigen::Vector2d {
    return {p.x()+std::cos(p.z())*point.x()-std::sin(p.z())*point.y(),
      p.y()+std::sin(p.z())*point.x()+std::cos(p.z())*point.y()};
  };
  bool ok=false;
  for(double h:{1e-3,1e-5,1e-7,1e-9}) {
    Eigen::Matrix<double,2,3> numerical;
    for(int i=0;i<3;++i) {auto a=pose,b=pose;a[i]+=h;b[i]-=h;numerical.col(i)=(transform(a)-transform(b))/(2*h);}
    const double error=(studentPointJacobian(pose.z(),point)-numerical).norm();
    std::cout<<"h="<<h<<"; error="<<error<<'\n';if(h==1e-5) ok=error<1e-7;
  }
  std::cout<<(ok ? "PASS\n" : "FAIL\n");return ok ? 0 : 1;
}
