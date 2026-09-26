#pragma once
#include <cstdint>
#include <deque>
#include <optional>
#include "motion2d/estimation/scan_matcher.hpp"

namespace motion2d
{
/** @brief Centroid of each occupied square voxel; finite points, size in m. */
PointCloud2D voxelCloud(const PointCloud2D & points, double size);

/** @brief Bounded scan-to-submap odometry parameters; distances in m, angles in rad. */
struct LidarOdometryConfig
{
  MatchConfig match;
  double voxel_size = .06, normal_radius = .4;
  double keyframe_distance = .2, keyframe_angle = .15;
  double max_gap = 1.0;  ///< No local tracking after a longer unobserved interval (s).
  int max_keyframes = 5;
  LidarOdometryConfig() {match.metric = MatchMetric::PointToLine;}
};

/** @brief A scan-time estimate; rejected scans must not be inserted into a map. */
struct OdometryUpdate
{
  Pose2D pose;
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero(); ///< Secant velocity in odom, m/s.
  double yaw_rate = 0;  ///< Secant angular velocity, rad/s; not an IMU reading.
  bool accepted = false;
  std::string status;
  MatchResult match;
};

/** @brief Scan-only local odometry with first accepted frame as its origin; ch08.
 * @details No truth, obstacle geometry or TF input. Failed matches leave the map
 * and accepted state unchanged. A time rollback/zero starts a new trial. A long
 * blackout requires reset, not silent reinitialization at the last position.
 */
class LidarOdometry
{
public:
  explicit LidarOdometry(LidarOdometryConfig config = {});
  /** @brief Estimate T_odom_laser at stamp_ns; optional prediction is in odom. */
  OdometryUpdate update(const PointCloud2D & points, std::int64_t stamp_ns,
    std::optional<Pose2D> prediction = std::nullopt);
  /** @brief Clear local history and first-frame origin. */
  void reset();
  /** @brief Current bounded reference cloud, in odom. */
  const PointCloud2D & submap() const {return target_.points;}
  std::size_t keyframeCount() const {return frames_.size();}

private:
  void addKeyframe(const PointCloud2D & points, const Pose2D & pose);
  LidarOdometryConfig config_;
  MatchTarget target_;
  std::deque<PointCloud2D> frames_;
  Pose2D pose_, keyframe_pose_;
  Eigen::Vector2d velocity_ = Eigen::Vector2d::Zero();
  double yaw_rate_ = 0;
  std::int64_t last_seen_ = -1, last_accepted_ = -1;
};
}  // namespace motion2d
