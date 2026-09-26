#include "motion2d/estimation/lidar_odometry.hpp"
#include <cmath>
#include <map>
#include <stdexcept>

namespace motion2d
{
PointCloud2D voxelCloud(const PointCloud2D & points, double size)
{
  if (!std::isfinite(size) || size <= 0) {throw std::invalid_argument("Invalid voxel size");}
  struct Cell {Eigen::Vector2d sum = Eigen::Vector2d::Zero(); int count = 0;};
  std::map<std::pair<double, double>, Cell> cells;
  for (const auto & point : points) {
    if (!point.allFinite()) {throw std::invalid_argument("Nonfinite cloud point");}
    auto & cell = cells[{std::floor(point.x() / size), std::floor(point.y() / size)}];
    cell.sum += point;
    ++cell.count;
  }
  PointCloud2D result;
  result.reserve(cells.size());
  for (const auto & item : cells) {result.push_back(item.second.sum / item.second.count);}
  return result;
}

LidarOdometry::LidarOdometry(LidarOdometryConfig config) : config_(config)
{
  for (double x : {config.voxel_size, config.normal_radius, config.keyframe_distance,
      config.keyframe_angle, config.max_gap}) {
    if (!std::isfinite(x) || x <= 0) {throw std::invalid_argument("Invalid odometry scale");}
  }
  if (config.max_keyframes < 1) {throw std::invalid_argument("Need at least one keyframe");}
  matchClouds({}, {}, {}, config_.match);  // Validate external matching parameters once.
}

void LidarOdometry::reset()
{
  target_ = {};
  frames_.clear();
  pose_ = {}; keyframe_pose_ = {};
  velocity_.setZero(); yaw_rate_ = 0;
  last_seen_ = last_accepted_ = -1;
}

void LidarOdometry::addKeyframe(const PointCloud2D & points, const Pose2D & pose)
{
  PointCloud2D registered;
  registered.reserve(points.size());
  for (const auto & point : points) {registered.push_back(transformPoint(pose, point));}
  frames_.push_back(std::move(registered));
  while (frames_.size() > static_cast<std::size_t>(config_.max_keyframes)) {frames_.pop_front();}
  PointCloud2D joined;
  for (const auto & frame : frames_) {joined.insert(joined.end(), frame.begin(), frame.end());}
  target_ = makeMatchTarget(voxelCloud(joined, config_.voxel_size), config_.normal_radius);
  keyframe_pose_ = pose;
}

OdometryUpdate LidarOdometry::update(const PointCloud2D & points, std::int64_t stamp,
  std::optional<Pose2D> prediction)
{
  if (stamp < 0) {throw std::invalid_argument("Negative sensor timestamp");}
  if (stamp == 0 || stamp < last_seen_) {reset();}
  OdometryUpdate result;
  result.pose = pose_;
  if (stamp == last_seen_) {result.status = "duplicate"; return result;}
  last_seen_ = stamp;
  const auto source = voxelCloud(points, config_.voxel_size);
  if (source.size() < static_cast<std::size_t>(config_.match.min_pairs)) {
    result.status = "too_few_points"; return result;
  }
  if (last_accepted_ < 0) {
    addKeyframe(source, pose_);
    last_accepted_ = stamp;
    result.accepted = true; result.status = "initialized";
    return result;
  }
  const double dt = (stamp - last_accepted_) * 1e-9;
  if (dt > config_.max_gap) {result.status = "tracking_lost"; return result;}
  Pose2D initial = prediction.value_or(pose_);
  if (!prediction) {
    initial.position += dt * velocity_;
    initial.yaw = wrapAngle(initial.yaw + dt * yaw_rate_);
  }
  // local_odometry_accept_begin
  result.match = matchClouds(source, target_, initial, config_.match);
  result.status = matchStatusName(result.match.status);
  if (!result.match.accepted()) {return result;}
  velocity_ = (result.match.pose.position - pose_.position) / dt;
  yaw_rate_ = wrapAngle(result.match.pose.yaw - pose_.yaw) / dt;
  pose_ = result.match.pose;
  last_accepted_ = stamp;
  if ((pose_.position - keyframe_pose_.position).norm() >= config_.keyframe_distance ||
      std::abs(wrapAngle(pose_.yaw - keyframe_pose_.yaw)) >= config_.keyframe_angle) {
    addKeyframe(source, pose_);
  }
  // local_odometry_accept_end
  result.pose = pose_; result.velocity = velocity_; result.yaw_rate = yaw_rate_;
  result.accepted = true;
  return result;
}
}  // namespace motion2d
