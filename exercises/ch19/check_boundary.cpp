#include "matching_boundary.hpp"
#include <iostream>
int main() {
  motion2d::TranslationState old;old.position={1,2};old.velocity={.3,.2};old.acceleration={.1,-.1};
  if(!matchingBoundary(old,old)) return 1;
  auto changed=old;changed.velocity.x()+=.2;if(matchingBoundary(old,changed)) return 1;
  changed=old;changed.acceleration.y()+=.2;if(matchingBoundary(old,changed)) return 1;
  std::cout<<"PASS p/v/a boundary\n";
}
