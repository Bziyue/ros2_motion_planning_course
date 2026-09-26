#pragma once
#include <cstdint>
#include <vector>
#include "motion2d/geometry/se2.hpp"
#include "motion2d/sim/world.hpp"

namespace motion2d
{
/** @brief Snapshot planar lidar settings; angles in rad and ranges in m. */
struct LidarConfig
{
  int beams = 720;
  double angle_min = -kPi;
  double fov = 2.0 * kPi;
  double range_min = 0.05, range_max = 10.0;
};

/** @brief Validate once at the configuration boundary; throws invalid_argument. */
void validateLidarConfig(const LidarConfig & config);

/** @brief Angular spacing: 2*pi/N for a full turn, FOV/(N-1) otherwise.
 * @pre Validated config. Full-turn comparison uses a 1e-12 rad tolerance.
 */
double beamIncrement(const LidarConfig & config);

/** @brief Compute one noiseless scan at a single robot pose in world coordinates.
 * @pre Valid world/config and a finite pose with origin in free space.
 * @return N distances: finite in [min,max], +inf for no in-range echo,
 * NaN for a true return nearer than min. The scanner is at the robot centre.
 * @details Each direction is yaw+angle_min+i*increment. No self-hit, no
 * rolling acquisition, no intensity model. See chapter 05.
 */
std::vector<float> scanCpu(const World2D & world, const Pose2D & pose,
  const LidarConfig & config);

/** @brief Number of simulation ticks per scan; reject nonaligned sample periods.
 * @param rate Scan rate in Hz, finite and >= 0.1.
 * @param dt Simulation step in s, finite and positive.
 * @return Positive integer ticks, requiring 1/rate = ticks*dt within 1 ns.
 * @details This chapter samples only existing states; it never silently rounds
 * a 7 Hz request to another frequency. Nonaligned scheduling needs interpolation.
 */
std::int64_t lidarPeriodTicks(double rate, double dt);
}  // namespace motion2d
