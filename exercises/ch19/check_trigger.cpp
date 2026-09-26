#include "replan_due.hpp"
#include <iostream>
int main() {
  bool ok=studentReplanDue(0,{},200,false) && !studentReplanDue(100,100,200,false) &&
    !studentReplanDue(299,100,200,false) && studentReplanDue(300,100,200,false) &&
    !studentReplanDue(300,100,200,true) && !studentReplanDue(50,100,200,false);
  std::cout<<(ok ? "PASS\n" : "FAIL\n");return ok ? 0 : 1;
}
