#pragma once
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include "motion2d/estimation/planar_ekf.hpp"

namespace motion2d
{
/** @brief Acquisition-time IMU sample; time is integer nanoseconds. */
struct TimedImu {std::int64_t stamp; PlanarImu sample;};

/** @brief EKF state at an explicit time, including uncertainty. */
struct FilterSnapshot {std::int64_t stamp; PlanarEkf filter;};

/** @brief Bounded held-IMU timeline for delayed, chronological laser corrections.
 * @details Predict with the sample on the LEFT of each interval. Laser times
 * may split an interval. Rewind to an anchor, correct at the laser stamp, then
 * replay IMU to the newest sample. No wall time or latest-pose substitution.
 * The caller queues future laser poses; reset is explicit for a new experiment.
 */
class FusionTimeline
{
public:
  explicit FusionTimeline(EkfConfig config = {}, double history_seconds = 2.0);
  /** @brief Clear gauge, history and timestamps; no prior estimate survives. */
  void reset();
  /** @brief Append a sample; duplicates/rollback rejected, gaps > .1 s lose initialization. */
  bool pushImu(const TimedImu & imu);
  /** @brief Correct at a bracketed stamp; returns scan pose only on accepted correction.
   * @details First laser initializes pose, zero velocity with 1 m/s uncertainty.
   * Corrections must be strictly chronological; future poses return waiting_imu.
   */
  std::optional<FilterSnapshot> correct(std::int64_t stamp, const Pose2D & pose);
  bool ready() const {return ready_;}
  std::int64_t newestStamp() const {return samples_.empty() ? -1 : samples_.back().stamp;}
  const FilterSnapshot & latest() const {return latest_;}
  const PlanarImu & latestImu() const {return samples_.back().sample;}
  const std::string & status() const {return status_;}
  std::size_t historySize() const {return samples_.size();}
  /** @brief Seconds since the most recent accepted laser correction; infinity before it. */
  double laserAge() const;

private:
  void advance(FilterSnapshot & state, std::int64_t target) const;
  void trim();
  EkfConfig config_;
  std::int64_t history_ns_;
  std::deque<TimedImu> samples_;
  FilterSnapshot anchor_, latest_;
  std::int64_t last_attempt_ = -1, last_correction_ = -1;
  bool ready_ = false;
  std::string status_ = "waiting_laser";
};
}  // namespace motion2d
