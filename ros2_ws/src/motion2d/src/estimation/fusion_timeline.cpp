#include "motion2d/estimation/fusion_timeline.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>

namespace motion2d
{
FusionTimeline::FusionTimeline(EkfConfig config, double history_seconds)
: config_(config), anchor_{-1, PlanarEkf(config)}, latest_(anchor_)
{
  if (!std::isfinite(history_seconds) || history_seconds < .1 || history_seconds > 10.) {
    throw std::invalid_argument("history_seconds must be in [.1,10]");
  }
  history_ns_ = std::llround(history_seconds * 1e9);
}

void FusionTimeline::reset()
{
  samples_.clear(); ready_ = false; last_attempt_ = last_correction_ = -1;
  anchor_ = {-1, PlanarEkf(config_)}; latest_ = anchor_; status_ = "waiting_laser";
}

bool FusionTimeline::pushImu(const TimedImu & imu)
{
  // Reuse the model's external measurement validation before changing history.
  predictImu(Vector8d::Zero(), imu.sample, .001);
  if (imu.stamp < 0 || imu.stamp <= newestStamp()) {status_ = "imu_not_increasing"; return false;}
  if (!samples_.empty() && imu.stamp - newestStamp() > 100000000) {
    reset(); status_ = "imu_gap";
  }
  if (ready_) {
    latest_.filter.predict(samples_.back().sample, (imu.stamp - latest_.stamp) * 1e-9);
    latest_.stamp = imu.stamp;
  }
  samples_.push_back(imu);
  trim();
  return true;
}

// fusion_replay_begin
void FusionTimeline::advance(FilterSnapshot & state, std::int64_t target) const
{
  for (std::size_t i = 0; i + 1 < samples_.size() && state.stamp < target; ++i) {
    if (samples_[i + 1].stamp <= state.stamp) {continue;}
    const auto end = std::min(target, samples_[i + 1].stamp);
    const double dt = (end - state.stamp) * 1e-9;
    state.filter.predict(samples_[i].sample, dt);
    state.stamp = end;
  }
}
// fusion_replay_end

void FusionTimeline::trim()
{
  while (samples_.size() > 1 && newestStamp() - samples_[1].stamp > history_ns_) {
    if (ready_ && anchor_.stamp < samples_[1].stamp) {advance(anchor_, samples_[1].stamp);}
    samples_.pop_front();
  }
}

std::optional<FilterSnapshot> FusionTimeline::correct(std::int64_t stamp, const Pose2D & pose)
{
  if (stamp > newestStamp()) {status_ = "waiting_imu"; return std::nullopt;}
  if (samples_.empty() || stamp < samples_.front().stamp ||
    (ready_ && stamp < anchor_.stamp)) {status_ = "laser_too_old"; return std::nullopt;}
  if (stamp <= last_attempt_) {status_ = "laser_not_increasing"; return std::nullopt;}
  if (!pose.position.allFinite() || !std::isfinite(pose.yaw)) {
    throw std::invalid_argument("nonfinite laser pose");
  }
  last_attempt_ = stamp;
  FilterSnapshot corrected = anchor_;
  if (!ready_) {
    corrected = {stamp, PlanarEkf(config_)}; corrected.filter.reset(pose);
  } else {
    advance(corrected, stamp);
    if (!corrected.filter.correct(pose)) {status_ = "innovation_rejected"; return std::nullopt;}
  }
  ready_ = true; last_correction_ = stamp; status_ = "laser_corrected";
  anchor_ = corrected;
  latest_ = anchor_; advance(latest_, newestStamp());
  // Retain the left sample even when a correction lies strictly inside an interval.
  while (samples_.size() > 1 && samples_[1].stamp <= stamp) {samples_.pop_front();}
  return corrected;
}

double FusionTimeline::laserAge() const
{
  return last_correction_ < 0 ? std::numeric_limits<double>::infinity() :
    (newestStamp() - last_correction_) * 1e-9;
}
}  // namespace motion2d
