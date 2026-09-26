#include <iomanip>
#include <iostream>
#include "motion2d/geometry/se2.hpp"

/** @brief A hand-checkable transform: (2,0) in body becomes (1,4) in map. */
int main()
{
  const motion2d::Pose2D map_from_body{{1.0, 2.0}, motion2d::kPi / 2.0};
  const Eigen::Vector2d point_body{2.0, 0.0};
  const auto point_map = motion2d::transformPoint(map_from_body, point_body);
  const auto recovered = motion2d::transformPoint(motion2d::inverse(map_from_body), point_map);
  std::cout << std::fixed << std::setprecision(6)
            << "body point [m]: " << point_body.transpose() << '\n'
            << "map point  [m]: " << point_map.transpose() << '\n'
            << "recovered  [m]: " << recovered.transpose() << '\n';
}
