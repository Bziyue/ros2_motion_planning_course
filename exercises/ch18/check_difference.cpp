#include "input_difference.hpp"
#include <iostream>
int main() {
  Eigen::VectorXd u(6),expected(6);u<<.1,.2,.4,-.3,-.2,.1;expected<<.1,.2,.3,-.5,-.6,.4;
  if((inputDifference(3)*u-expected).norm()>1e-12) return 1;
  std::cout<<"PASS input difference\n";
}
