#include "body_velocity.hpp"
#include <iostream>
int main(){ if((bodyVelocity({.2,.3},std::acos(-1.)/2)-Eigen::Vector2d(-.3,.2)).norm()>1e-12) return 1;std::cout<<"PASS body velocity\n";}
