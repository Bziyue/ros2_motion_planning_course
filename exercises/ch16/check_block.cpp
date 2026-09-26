#include "block.hpp"
#include <Eigen/LU>
#include <iostream>
int main(){Eigen::Matrix2d a,b,c;a<<4,1,1,3;b<<.2,.3,-.1,.4;c<<3,.2,.2,2;
 Eigen::Matrix4d H;H<<a,b.transpose(),b,c;
 Eigen::Vector4d rhs;rhs<<1,2,3,4;
 const Eigen::Vector2d y=schurBlock(c,b,a.inverse()).fullPivLu().solve(rhs.tail<2>()-b*a.inverse()*rhs.head<2>());
 const Eigen::Vector4d ref=H.fullPivLu().solve(rhs);
 if((y-ref.tail<2>()).norm()>1e-12)return 1;std::cout<<"ch16-1 PASS\n";}
