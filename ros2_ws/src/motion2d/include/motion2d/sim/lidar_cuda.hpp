#pragma once
#include <memory>
#include "motion2d/sim/lidar_cpu.hpp"
namespace motion2d {
/** @brief Synchronized wall seconds; kernel uses CUDA events on the device. */
struct CudaScanTiming {
  double kernel_seconds=0,download_seconds=0,scan_seconds=0;
};
/** @brief Optional ch20 snapshot scanner: one thread per ray, resident static geometry.
 * @details Same float/+inf/NaN and angular contract as scanCpu. Geometry is packed
 * as structure-of-arrays in double precision. No RNG on device: both backends use
 * addRangeNoise afterwards. Construction uploads once; buffers/events are reused.
 * This class is single-threaded and bound to the current CUDA device.
 */
class CudaLidar {
public:
  /** @brief Upload a valid static world and reserve 2..100000 output beams. */
  CudaLidar(const World2D & world,int capacity);
  ~CudaLidar();
  CudaLidar(const CudaLidar &)=delete;
  CudaLidar & operator=(const CudaLidar &)=delete;
  /** @brief Synchronous noiseless scan; optional timing includes launch/sync/copy.
   * @pre Valid pose/config, origin in free space, config.beams <= reserved capacity.
   */
  std::vector<float> scan(const Pose2D & pose,const LidarConfig & config,CudaScanTiming * timing=nullptr);
  /** @brief One-time host packing + allocation + geometry upload wall seconds. */
  double setupSeconds() const;
  /** @brief Synchronous geometry H2D copy wall seconds, excluding allocation/packing. */
  double uploadSeconds() const;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
