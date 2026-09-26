#pragma once
#include "motion2d/trajectory/execution.hpp"
#include "motion2d/planning/corridor.hpp"
#include <optional>
#include <string>
namespace motion2d {
/** @brief One atomic navigation reference in odom, with a region per curve piece.
 * @details Empty regions are allowed for standalone reference experiments; navigation
 * must supply verified configuration-space regions. snapshot_ns identifies map data.
 */
struct NavigationReference {
  TimedTrajectory motion;
  std::vector<ConvexRegion> regions;
  std::int64_t snapshot_ns=0;
};
/** @brief Freeze a map-frame curve AND its regions through one constant SE(2) transform. */
NavigationReference transformReference(const NavigationReference & plan,const Pose2D & target_from_source);
struct ReferenceAcceptance {bool accepted=false;std::string reason;};
/** @brief One active curve plus at most one future C2 handover; no unbounded command queue.
 * @details sample() is const: looking ahead for MPC never activates the future plan early.
 * A reference is fixed in odom and does not follow subsequent map-to-odom corrections.
 */
class ReferenceSchedule {
public:
  explicit ReferenceSchedule(const Pose2D & hold={}) {reset(hold);}
  /** @brief Cancel both curves and hold a position, with zero derivatives.
   * @details An emergency reset is intentionally NOT claimed to preserve C2.
   */
  void reset(const Pose2D & hold);
  /** @brief Activate a due handover using actual observation time, not a prediction time. */
  void advance(std::int64_t now_ns);
  /** @brief Accept only future commands matching the old p/v/a/yaw at the handover.
   * @details End velocity/acceleration must be zero for the final hold. A rejected
   * candidate never changes an already scheduled valid reference.
   */
  ReferenceAcceptance accept(NavigationReference next,std::int64_t now_ns);
  State2D sample(std::int64_t stamp_ns) const;
  /** @brief Region assigned to the curve piece at this time, or null for no region. */
  const ConvexRegion * region(std::int64_t stamp_ns) const;
  /** @brief Reference used at a future/current instant, or null before the first start. */
  const NavigationReference * reference(std::int64_t stamp_ns) const;
  bool empty() const {return !active_ && !pending_;}
  bool pending() const {return pending_.has_value();}
private:
  Pose2D hold_;
  std::optional<NavigationReference> active_,pending_;
};
}
