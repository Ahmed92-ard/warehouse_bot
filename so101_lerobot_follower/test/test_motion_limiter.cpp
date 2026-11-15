#include <cmath>
#include <iostream>

#include "so101_lerobot_follower/motion_limiter.hpp"

using so101_lerobot_follower::CalibrationSet;
using so101_lerobot_follower::JointMotionLimiter;

int main() {
  int failures = 0;
  const auto check = [&failures](bool condition, const char *message) {
    if (!condition) { std::cerr << "FAIL: " << message << '\n'; ++failures; }
  };
  CalibrationSet calibration{};
  for (auto &joint : calibration) { joint.range_min = 100; joint.range_max = 900; }
  calibration[5].range_min = 1000;
  calibration[5].range_max = 3000;

  JointMotionLimiter limiter;
  JointMotionLimiter::Counts measured{500, 500, 500, 500, 500, 2000};
  JointMotionLimiter::Counts far_target{900, 900, 900, 900, 900, 3000};
  limiter.set_target(far_target);
  auto first = limiter.advance(measured, calibration, 0.02, 20.0, 30.0);
  check(first.has_value(), "accepted target produces a shaped command");
  if (first) {
    for (size_t i = 0; i < 5; ++i)
      check((*first)[i] - measured[i] <= std::ceil(20.0 * 4095.0 / 360.0 * 0.02),
            "arm joint speed is bounded despite an abrupt target");
    check((*first)[5] - measured[5] <= std::ceil(30.0 * 2000.0 / 100.0 * 0.02),
          "gripper uses percent-per-second speed bound");
  }

  // A new target replaces the old one immediately and reverses the slew direction.
  JointMotionLimiter::Counts reverse_target{100, 100, 100, 100, 100, 1000};
  limiter.set_target(reverse_target);
  auto reversed = limiter.advance(measured, calibration, 0.02, 20.0, 30.0);
  check(first && reversed && (*reversed)[0] < (*first)[0],
        "direction reversal follows the latest target without draining queued targets");

  limiter.clear();  // Called while Stop/session-disable holds the follower command gate.
  check(!limiter.advance(measured, calibration, 0.02, 20.0, 30.0).has_value(),
        "Stop clears the target so later control ticks cannot issue motor updates");
  return failures == 0 ? 0 : 1;
}
