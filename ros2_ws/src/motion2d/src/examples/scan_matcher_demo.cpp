#include <iostream>
#include <iomanip>
#include "motion2d/estimation/scan_matcher.hpp"
#include "motion2d/mapping/scan_projection.hpp"

int main()
{
  motion2d::PointCloud2D points;
  for (int i = 0; i <= 80; ++i) {
    points.emplace_back(-2 + .05 * i, 2);
    points.emplace_back(2, -2 + .05 * i);
  }
  const auto target = motion2d::makeMatchTarget(points);
  const motion2d::Pose2D known{{.12, -.08}, .04};
  const auto source = motion2d::registerPoints(points, motion2d::inverse(known));
  std::cout << "metric,initial,status,x,y,yaw,rmse,pairs,iterations\n" << std::setprecision(9);
  for (auto metric : {motion2d::MatchMetric::PointToPoint, motion2d::MatchMetric::PointToLine}) {
    for (bool nearby : {false, true}) {
      motion2d::MatchConfig config;
      config.metric = metric;
      const motion2d::Pose2D initial = nearby ?
        motion2d::Pose2D{{.11, -.075}, .038} : motion2d::Pose2D{};
      const auto result = motion2d::matchClouds(source, target, initial, config);
      std::cout << (metric == motion2d::MatchMetric::PointToPoint ? "point" : "line") << ','
        << (nearby ? "near" : "zero") << ',' << motion2d::matchStatusName(result.status) << ','
        << result.pose.position.x() << ',' << result.pose.position.y() << ','
        << result.pose.yaw << ',' << result.rmse << ',' << result.pairs << ','
        << result.iterations << '\n';
      if (!result.accepted()) {return 1;}
    }
  }
}
