#include "ray_circle.hpp"
#include <iostream>
int main() {
  bool ok=std::abs(studentCircleHit(0,0,1,0,3,0,1)-2)<1e-12 &&
    std::abs(studentCircleHit(0,1,1,0,3,0,1)-3)<1e-12 &&
    std::abs(studentCircleHit(3,0,1,0,3,0,1)-1)<1e-12 &&
    std::isinf(studentCircleHit(0,2,1,0,3,0,1)) && std::isinf(studentCircleHit(0,0,-1,0,3,0,1));
  std::cout<<(ok ? "PASS\n" : "FAIL\n");return ok ? 0 : 1;
}
