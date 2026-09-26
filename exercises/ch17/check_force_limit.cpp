#include "force_limit.hpp"
#include <iostream>
int main(){if((boundForce({.1,-.2},.3)-Eigen::Vector2d(.1,-.2)).norm()>1e-12)return 1;
 if((boundForce({.4,-.8},.3)-Eigen::Vector2d(.3,-.3)).norm()>1e-12)return 1;
 if((boundForce({.2,.4},.2)-Eigen::Vector2d(.2,.2)).norm()>1e-12)return 1;
 std::cout<<"ch17-1 PASS\n";}
