#include "motion2d/sim/world.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
/** @brief Evaluate saved truth samples against the ch19 physical world, offline only.
 * @details Same explicit generation settings as ch19.yaml. This executable never
 * publishes a map to navigation. Output is disk clearance, not centre distance.
 */
int main(int argc,char ** argv) {
  if(argc!=3) {std::cerr<<"Usage: navigation_clearance seed samples.csv\n";return 1;}
  try {
    motion2d::WorldConfig config;config.seed=std::stoul(argv[1]);config.goal={-4,-4};
    const auto world=motion2d::generateWorld(config);std::ifstream file(argv[2]);
    if(!file) throw std::runtime_error("Cannot open samples");
    std::string line;std::getline(file,line);std::cout<<"t,clearance_m\n"<<std::setprecision(17);
    while(std::getline(file,line)) {
      std::istringstream row(line);std::string value;double fields[3];
      for(auto & field:fields) {if(!std::getline(row,value,',')) throw std::runtime_error("Expected t,x,y");field=std::stod(value);}
      std::cout<<fields[0]<<','<<motion2d::clearance(world,{fields[1],fields[2]})-config.robot_radius<<'\n';
    }
  } catch(const std::exception & e) {std::cerr<<e.what()<<'\n';return 1;}
}
