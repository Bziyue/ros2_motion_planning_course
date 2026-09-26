#include "curvature_rate.hpp"
#include <cmath>
#include <iostream>
int main() {
  const Eigen::Vector2d v(.7,.3),a(-.2,.6),j(.4,-.1);const double h=1e-6;
  const auto k=[](const Eigen::Vector2d & v,const Eigen::Vector2d & a){return (v.x()*a.y()-v.y()*a.x())/std::pow(v.norm(),3);};
  const double fd=(k(v+h*a,a+h*j)-k(v-h*a,a-h*j))/(2*h);
  if(std::abs(curvatureRate(v,a,j)-fd)>1e-8){std::cerr<<"curvature-rate mismatch\n";return 1;}
  std::cout<<"ch04-6 passed\n";
}
