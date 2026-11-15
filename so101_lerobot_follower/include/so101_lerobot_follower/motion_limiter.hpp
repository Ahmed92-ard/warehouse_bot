#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>

#include "so101_lerobot_follower/calibration.hpp"

namespace so101_lerobot_follower {

// Slew-limits the latest calibrated count target; it never queues old actions.
class JointMotionLimiter {
 public:
  using Counts = std::array<int, 6>;
  using Positions = std::array<double, 6>;

  void set_target(const Counts &target) {
    target_ = target;
    has_target_ = true;
  }

  void clear() noexcept {
    initialized_ = false;
    has_target_ = false;
  }

  std::optional<Counts> advance(const Counts &measured, const CalibrationSet &calibration,
                               double elapsed_seconds, double arm_speed_deg_s,
                               double gripper_speed_percent_s) {
    if (!std::isfinite(elapsed_seconds) || elapsed_seconds < 0.0 ||
        !std::isfinite(arm_speed_deg_s) || arm_speed_deg_s <= 0.0 ||
        !std::isfinite(gripper_speed_percent_s) || gripper_speed_percent_s <= 0.0) {
      throw std::invalid_argument("motion limiter elapsed time and speeds must be finite and positive");
    }
    if (!has_target_) return std::nullopt;
    if (!initialized_) {
      for (size_t i = 0; i < current_.size(); ++i) current_[i] = measured[i];
      initialized_ = true;
    }
    Counts output{};
    for (size_t i = 0; i < output.size(); ++i) {
      const double counts_per_second = i == 5
          ? gripper_speed_percent_s * (calibration[i].range_max - calibration[i].range_min) / 100.0
          : arm_speed_deg_s * 4095.0 / 360.0;
      const double max_step = counts_per_second * elapsed_seconds;
      const double difference = static_cast<double>(target_[i]) - current_[i];
      current_[i] += std::clamp(difference, -max_step, max_step);
      output[i] = static_cast<int>(std::lround(current_[i]));
    }
    return output;
  }

 private:
  Positions current_{};
  Counts target_{};
  bool initialized_{false};
  bool has_target_{false};
};

}  // namespace so101_lerobot_follower
