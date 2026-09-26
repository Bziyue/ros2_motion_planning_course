#include "motion2d/trajectory/reference_schedule.hpp"
#include "motion2d/trajectory/quintic.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
using namespace motion2d;
/** @brief Two reference curves join at a moving p/v/a state, including a map-frame round trip. */
int main(int argc,char ** argv) {
  const std::filesystem::path dir=argc>1 ? argv[1] : "tmp/ch19_handover";std::filesystem::create_directories(dir);
  TranslationState begin,end;end.position={2,0};
  NavigationReference first{{PolynomialTrajectory({interpolateQuintic(begin,end,4)}),0,0},{},0};
  ReferenceSchedule schedule;if(!schedule.accept(first,0).accepted) return 1;schedule.advance(1);
  const auto state=schedule.sample(1500000000);begin={state.pose.position,state.velocity,state.acceleration};end.position={3,1};
  NavigationReference next{{PolynomialTrajectory({interpolateQuintic(begin,end,4)}),1500000000,0},{},1200000000};
  const Pose2D correction{{1,-.5},.4};const auto in_map=transformReference(next,correction);
  if(!schedule.accept(transformReference(in_map,inverse(correction)),1200000000).accepted) return 1;
  std::ofstream csv(dir/"samples.csv");csv<<"t,old_x,old_y,x,y,vx,vy,ax,ay\n";
  for(int k=0;k<=600;++k) {
    const auto ns=std::int64_t(k)*10000000;const auto s=schedule.sample(ns),old=sampleHeldTrajectory(first.motion,ns);
    csv<<k*.01<<','<<old.pose.position.x()<<','<<old.pose.position.y()<<','<<s.pose.position.x()<<','<<s.pose.position.y()<<','<<s.velocity.x()<<','<<s.velocity.y()<<','<<s.acceleration.x()<<','<<s.acceleration.y()<<'\n';
  }
  std::cout<<"Accepted future handover at 1.5s: p="<<begin.position.transpose()<<" v="<<begin.velocity.transpose()<<" a="<<begin.acceleration.transpose()<<'\n';
}
