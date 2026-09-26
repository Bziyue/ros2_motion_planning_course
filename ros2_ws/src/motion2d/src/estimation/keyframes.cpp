#include "motion2d/estimation/keyframes.hpp"
#include "motion2d/estimation/lidar_odometry.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motion2d
{
KeyframeSlam::KeyframeSlam(KeyframeConfig config) : config_(config)
{
  for (double value : {config.distance, config.angle, config.max_seconds, config.voxel_size,
    config.normal_radius, config.candidate_distance, config.candidate_angle}) {
    if (!std::isfinite(value) || value <= 0) {throw std::invalid_argument("positive keyframe settings required");}
  }
  if (config.min_separation < 2 || config.loop_spacing < 1 || config.max_frames < 2) {
    throw std::invalid_argument("invalid keyframe separation/capacity");
  }
}

void KeyframeSlam::reset()
{
  frames_.clear(); edges_.clear(); alignment_ = {}; last_stamp_ = -1; loops_ = last_loop_ = 0;
}

SlamUpdate KeyframeSlam::update(const Scan2D & scan, const Pose2D & odom_pose, std::int64_t stamp)
{
  if (stamp < 0 || !odom_pose.position.allFinite() || !std::isfinite(odom_pose.yaw)) {
    throw std::invalid_argument("invalid keyframe timestamp/pose");
  }
  if (stamp == 0 || stamp < last_stamp_) {reset();}
  SlamUpdate update;
  if (stamp == last_stamp_) {update.status = "duplicate"; return update;}
  last_stamp_ = stamp;
  if (!frames_.empty()) {
    const auto & previous = frames_.back();
    if ((odom_pose.position-previous.odom_pose.position).norm() < config_.distance &&
      std::abs(wrapAngle(odom_pose.yaw-previous.odom_pose.yaw)) < config_.angle &&
      (stamp-previous.stamp) * 1e-9 < config_.max_seconds) {return update;}
  }
  if (frames_.size() >= config_.max_frames) {update.status = "capacity_reached"; return update;}
  const auto points = voxelCloud(projectScan(scan), config_.voxel_size);
  if (points.size() < static_cast<std::size_t>(config_.loop.match.min_pairs)) {
    update.status = "too_few_points"; return update;
  }
  const auto current = frames_.size();
  if (current > 0) {
    edges_.push_back({current-1, current, compose(inverse(frames_.back().odom_pose), odom_pose),
      {.03, .03, .02}, false});
  }
  frames_.push_back({stamp, odom_pose, compose(alignment_, odom_pose), scan,
    makeMatchTarget(points, config_.normal_radius)});
  update.keyframe_added = true; update.status = "keyframe_added";
  if (!config_.enable_loops || current < config_.min_separation ||
    current-last_loop_ < config_.loop_spacing) {return update;}
  std::vector<std::pair<double, std::size_t>> candidates;
  for (std::size_t i = 0; i + config_.min_separation <= current; ++i) {
    const double distance = (frames_[i].map_pose.position-frames_.back().map_pose.position).norm();
    const double angle = std::abs(wrapAngle(frames_[i].map_pose.yaw-frames_.back().map_pose.yaw));
    if (distance <= config_.candidate_distance && angle <= config_.candidate_angle) {
      candidates.emplace_back(distance, i);
    }
  }
  std::sort(candidates.begin(), candidates.end());
  // At most three nearest old candidates per keyframe; bounded teaching workload.
  if (candidates.size() > 3) {candidates.resize(3);}
  for (const auto & [distance, i] : candidates) {
    (void)distance; ++update.candidates;
    const auto initial = compose(inverse(frames_[i].map_pose), frames_.back().map_pose);
    const auto loop = verifyLoop(frames_.back().local, frames_[i].local, initial, config_.loop);
    if (!loop.accepted) {update.status = "loop_" + loop.status; continue;}
    edges_.push_back({i, current, loop.relative, {.02, .02, .01}, true});
    std::vector<Pose2D> poses;
    for (const auto & frame : frames_) {poses.push_back(frame.map_pose);}
    const auto result = optimizePoseGraph(poses, edges_);
    if (!result.converged) {edges_.pop_back(); update.status = "graph_" + result.status; continue;}
    // map_alignment_begin
    for (std::size_t k = 0; k < frames_.size(); ++k) {frames_[k].map_pose = result.poses[k];}
    alignment_ = compose(frames_.back().map_pose, inverse(frames_.back().odom_pose));
    ++loops_; last_loop_ = current;
    // map_alignment_end
    update.loop_added = true; update.status = "loop_accepted"; break;
  }
  return update;
}

SlamMap rebuildKeyframeMap(const std::vector<Keyframe> & frames,
  const GridConfig & config, double voxel_size)
{
  SlamMap map(config);
  for (const auto & frame : frames) {
    if (!map.grid.insertScan(frame.scan, frame.map_pose)) {++map.outside_scans;}
    const auto transformed = registerPoints(frame.local.points, frame.map_pose);
    map.cloud.insert(map.cloud.end(), transformed.begin(), transformed.end());
  }
  map.cloud = voxelCloud(map.cloud, voxel_size);
  return map;
}
}  // namespace motion2d
