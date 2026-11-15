#pragma once

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <string>
#include <stdexcept>

namespace so101_lerobot_follower {

// Reject an action that the follower's calibrated interval cannot represent.
// The conversion functions below retain their historical boundary clamp, but
// command handling must call this validation first so invalid leader actions
// pause without writing a clamped target.
inline std::string follower_action_range_violation(double value, size_t joint_index,
                                                    const char *joint_name,
                                                    int range_min, int range_max) {
  if (!std::isfinite(value)) return {};
  if (joint_index == 5) {
    if (value < 0.0 || value > 100.0)
      return std::string(joint_name) + " action=" + std::to_string(value) +
             " outside [0,100] percent";
    return {};
  }
  if (range_min >= range_max) throw std::invalid_argument("calibration range must have positive width");
  const double midpoint = (static_cast<double>(range_min) + range_max) / 2.0;
  const double lower = (range_min - midpoint) * 360.0 / 4095.0;
  const double upper = (range_max - midpoint) * 360.0 / 4095.0;
  if (value < lower || value > upper)
    return std::string(joint_name) + " action=" + std::to_string(value) + " deg outside follower calibrated range [" +
           std::to_string(lower) + "," + std::to_string(upper) + "] deg";
  return {};
}

// LeRobot 0.6.2's default DEGREES conversion, bounded to this arm's calibrated
// interval as required for the follower. Counts are homing-offset-adjusted.
inline int degrees_to_follower_count(double degrees, int range_min, int range_max) {
  if (!std::isfinite(degrees)) throw std::invalid_argument("degree action must be finite");
  if (range_min >= range_max) throw std::invalid_argument("calibration range must have positive width");
  const double midpoint = (static_cast<double>(range_min) + range_max) / 2.0;
  const double lower = (range_min - midpoint) * 360.0 / 4095.0;
  const double upper = (range_max - midpoint) * 360.0 / 4095.0;
  const double bounded = std::clamp(degrees, lower, upper);
  const int converted = static_cast<int>(bounded * 4095.0 / 360.0 + midpoint);
  return std::clamp(converted, range_min, range_max);
}

// LeRobot's RANGE_0_100 gripper conversion using the follower's own limits.
inline int percent_to_follower_count(double percent, int range_min, int range_max) {
  if (!std::isfinite(percent) || percent < 0.0 || percent > 100.0)
    throw std::invalid_argument("gripper action must be finite and in [0,100]");
  if (range_min >= range_max) throw std::invalid_argument("calibration range must have positive width");
  const int converted = static_cast<int>(percent / 100.0 * (range_max - range_min) + range_min);
  return std::clamp(converted, range_min, range_max);
}

}  // namespace so101_lerobot_follower
