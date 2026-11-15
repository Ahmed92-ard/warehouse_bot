#include <limits>
#include <iostream>
#include <algorithm>
#include <so101_lerobot_follower/action_mapping.hpp>

using so101_lerobot_follower::degrees_to_follower_count;
using so101_lerobot_follower::percent_to_follower_count;
using so101_lerobot_follower::follower_action_range_violation;

int main() {
  int failures = 0;
  const auto check = [&failures](bool ok, const char *name) {
    if (!ok) { std::cerr << "FAIL: " << name << '\n'; ++failures; }
  };
  check(degrees_to_follower_count(0.0, 100, 900) == 500, "degree midpoint truncates like Python int");

  const double midpoint = (100.0 + 900.0) / 2.0;
  const double lower = (100.0 - midpoint) * 360.0 / 4095.0;
  const double upper = (900.0 - midpoint) * 360.0 / 4095.0;
  check(degrees_to_follower_count(lower, 100, 900) == 100, "lower degree limit maps to min count");
  check(degrees_to_follower_count(upper, 100, 900) == 900, "upper degree limit maps to max count");
  check(degrees_to_follower_count(-1000.0, 100, 900) == 100, "degree underflow clamps to min");
  check(degrees_to_follower_count(1000.0, 100, 900) == 900, "degree overflow clamps to max");

  check(follower_action_range_violation(lower, 0, "shoulder_pan", 100, 900).empty(),
        "calibrated lower action bound remains valid");
  check(follower_action_range_violation(upper, 0, "shoulder_pan", 100, 900).empty(),
        "calibrated upper action bound remains valid");
  const auto underflow = follower_action_range_violation(lower - 0.01, 0, "shoulder_pan", 100, 900);
  check(!underflow.empty(), "action below follower range is rejected before legacy mapper clamp");
  check(underflow.find("shoulder_pan") != std::string::npos && underflow.find("outside") != std::string::npos,
        "range rejection identifies joint and violation");
  check(!follower_action_range_violation(101.0, 5, "gripper", 200, 800).empty(),
        "gripper action above its native contract is rejected");

  bool session_enabled = true;
  bool movement_paused = false;
  int goal_writes = 0;
  const auto receive_action = [&](double degrees) {
    if (!session_enabled) return;
    const auto reason = follower_action_range_violation(degrees, 0, "shoulder_pan", 100, 900);
    if (!reason.empty()) {
      movement_paused = true;
      return;
    }
    (void)degrees_to_follower_count(degrees, 100, 900);
    ++goal_writes;
    movement_paused = false;
  };
  receive_action(0.0);
  check(goal_writes == 1, "valid action writes while the session is enabled");
  receive_action(lower - 0.01);
  check(goal_writes == 1, "out-of-range action causes no goal write");
  check(session_enabled && movement_paused, "range violation pauses movement without ending session");
  receive_action(lower - 0.02);
  check(goal_writes == 1 && session_enabled, "repeated range violations remain paused without disarming");
  receive_action(0.0);
  check(goal_writes == 2 && session_enabled && !movement_paused,
        "fresh valid action resumes movement in the same session");

  // Leader-side calibration mapping can provide a follower endpoint even while
  // the leader count is below or above its own saved interval. Those bounded
  // actions are valid to the follower and do not change session state.
  constexpr int leader_min = 875;
  constexpr int leader_max = 3097;
  constexpr int follower_min = 851;
  constexpr int follower_max = 3083;
  const auto mapped_follower_degrees = [=](int leader_count) {
    const double fraction = std::clamp(
        (leader_count - leader_min) / static_cast<double>(leader_max - leader_min), 0.0, 1.0);
    const double follower_count = follower_min + fraction * (follower_max - follower_min);
    const double follower_midpoint = (follower_min + follower_max) / 2.0;
    return (follower_count - follower_midpoint) * 360.0 / 4095.0;
  };
  const int leader_samples[] = {leader_min + 500, leader_min - 100, leader_max + 100};
  const int expected_targets[] = {
      follower_min + 500 * (follower_max - follower_min) / (leader_max - leader_min),
      follower_min, follower_max};
  for (size_t i = 0; i < 3; ++i) {
    const double mapped = mapped_follower_degrees(leader_samples[i]);
    const auto violation = follower_action_range_violation(mapped, 0, "shoulder_pan",
                                                             follower_min, follower_max);
    check(violation.empty(), "bounded mapped leader action is valid inside follower limits");
    const int target = degrees_to_follower_count(mapped, follower_min, follower_max);
    check(target == expected_targets[i], "mapped leader action reaches expected follower calibrated target");
    check(session_enabled, "leader calibration range status alone leaves follower session enabled");
  }
  check(expected_targets[0] != leader_samples[0],
        "different leader and follower calibration ranges are mapped proportionally");

  session_enabled = false;
  receive_action(0.0);
  check(goal_writes == 2, "explicit Stop prevents subsequent goal writes");

  check(percent_to_follower_count(0.0, 200, 800) == 200, "gripper zero maps to own min");
  check(percent_to_follower_count(50.0, 200, 800) == 500, "gripper midpoint maps to own interval");
  check(percent_to_follower_count(100.0, 200, 800) == 800, "gripper 100 maps to own max");

  bool threw = false;
  try { (void)degrees_to_follower_count(std::numeric_limits<double>::quiet_NaN(), 0, 4095); }
  catch (const std::invalid_argument &) { threw = true; }
  check(threw, "non-finite degrees are rejected");
  threw = false;
  try { (void)degrees_to_follower_count(0.0, 5, 5); }
  catch (const std::invalid_argument &) { threw = true; }
  check(threw, "zero-width calibration rejected");
  threw = false;
  try { (void)percent_to_follower_count(101.0, 0, 4095); }
  catch (const std::invalid_argument &) { threw = true; }
  check(threw, "out-of-range gripper percent rejected");
  return failures == 0 ? 0 : 1;
}
