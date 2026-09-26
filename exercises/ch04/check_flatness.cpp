#include "flat_force.hpp"
#include <iostream>
int main()
{
  const Eigen::Vector2d v(.8,-.3),a(-.4,.9);const double m=1.7,c=.4,h=1e-6;
  const auto g=forceGradient(m,c,v,a);
  for(int k=0;k<2;++k) {
    auto plus=a,minus=a;plus(k)+=h;minus(k)-=h;
    const double fd=((m*plus+c*v).squaredNorm()-(m*minus+c*v).squaredNorm())/(4*h);
    if(std::abs(fd-g(k))>1e-8){std::cerr<<"force gradient mismatch\n";return 1;}
  }
  std::cout<<"ch04-5 gradient passed\n";
}
