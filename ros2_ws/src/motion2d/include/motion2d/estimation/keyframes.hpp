#pragma once
#include <cstdint>
#include "motion2d/estimation/loop_closure.hpp"
#include "motion2d/estimation/pose_graph.hpp"
#include "motion2d/mapping/occupancy_grid.hpp"

namespace motion2d
{
/** @brief Retain raw local observations so map correction can rebuild, not smear, walls. */
struct Keyframe
{
  std::int64_t stamp;
  Pose2D odom_pose, map_pose;
  Scan2D scan;
  MatchTarget local;
};

/** @brief Static small-scene SLAM parameters; finite graph, no marginalization framework. */
struct KeyframeConfig
{
  double distance = .3, angle = .25, max_seconds = 2.;
  double voxel_size = .08, normal_radius = .4;
  double candidate_distance = .8, candidate_angle = .6;
  std::size_t min_separation = 20, loop_spacing = 5, max_frames = 200;
  bool enable_loops = true;
  LoopConfig loop;
};

/** @brief One keyframe update; a failed loop never undoes valid local odometry. */
struct SlamUpdate
{
  bool keyframe_added = false, loop_added = false;
  std::string status = "not_keyframe";
  std::size_t candidates = 0;
};

/** @brief Keyframe graph fed ONLY observed scans and sensor-derived local poses. */
class KeyframeSlam
{
public:
  explicit KeyframeSlam(KeyframeConfig config = {});
  /** @brief Process an exact-time scan/odom pair; zero/rollback resets this trial. */
  SlamUpdate update(const Scan2D & scan, const Pose2D & odom_pose, std::int64_t stamp);
  /** @brief Remove keyframes, edges, alignment and last sensor time. */
  void reset();
  const std::vector<Keyframe> & frames() const {return frames_;}
  const std::vector<PoseGraphEdge> & edges() const {return edges_;}
  /** @brief T_map_odom = T_map_last * inverse(T_odom_last); odometry stays untouched. */
  Pose2D mapToOdom() const {return alignment_;}
  std::size_t loopCount() const {return loops_;}
private:
  KeyframeConfig config_;
  std::vector<Keyframe> frames_;
  std::vector<PoseGraphEdge> edges_;
  Pose2D alignment_;
  std::int64_t last_stamp_ = -1;
  std::size_t loops_ = 0, last_loop_ = 0;
};

/** @brief Rebuilt observation products; fixed-grid out-of-bounds scans are counted. */
struct SlamMap
{
  explicit SlamMap(const GridConfig & config) : grid(config) {}
  OccupancyGrid2D grid;
  PointCloud2D cloud;
  std::size_t outside_scans = 0;
};

/** @brief Reproject original keyframe scans at optimized poses into a NEW map.
 * @details No truth geometry is accepted. Unknown remains unknown. Voxel map
 * points are surface observations; grid free-space evidence still uses full rays.
 */
SlamMap rebuildKeyframeMap(const std::vector<Keyframe> & frames,
  const GridConfig & config = {}, double voxel_size = .08);
}  // namespace motion2d
