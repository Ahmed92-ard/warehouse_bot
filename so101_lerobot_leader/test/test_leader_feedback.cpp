#include <array>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "../src/leader_feedback.hpp"

int main() {
  constexpr std::array<const char *, 6> names{
      "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"};
  constexpr std::array<int, 6> minimum{1059, 1055, 875, 914, 0, 1787};
  constexpr std::array<int, 6> maximum{3048, 3029, 3097, 3236, 4095, 2349};
  const std::array<int, 6> positions{2048, 969, 3098, 2000, 2048, 2000};
  std::vector<std::array<uint8_t, 2>> encoded;
  for (const int position : positions) {
    encoded.push_back({static_cast<uint8_t>(position & 0xff),
                       static_cast<uint8_t>((position >> 8) & 0xff)});
  }

  const auto feedback = so101_lerobot_leader::decode_leader_feedback(encoded, minimum, maximum);
  if (feedback.positions != positions) {
    std::cerr << "out-of-range readings must remain available as decoded feedback\n";
    return 1;
  }
  if (feedback.within_calibration[1] || feedback.within_calibration[2] ||
      !feedback.within_calibration[0] || !feedback.within_calibration[3]) {
    std::cerr << "calibration range status is incorrect\n";
    return 1;
  }
  const auto warning = so101_lerobot_leader::leader_feedback_range_warning(
      feedback, names, minimum, maximum);
  if (warning.find("shoulder_lift=969 outside [1055,3029]") == std::string::npos ||
      warning.find("elbow_flex=3098 outside [875,3097]") == std::string::npos) {
    std::cerr << "both offending joints must be diagnosed while feedback remains publishable\n";
    return 1;
  }
  if (!so101_lerobot_leader::is_recoverable_read_timeout("SerialPort::read_exact [ReadTimeout]") ||
      so101_lerobot_leader::is_recoverable_read_timeout("Port is not open")) {
    std::cerr << "only a documented serial read timeout is recoverable\n";
    return 1;
  }
  auto malformed = encoded;
  malformed[0] = {0xff, 0x7f};  // signed-magnitude position exceeds the 12-bit servo count range
  try {
    (void)so101_lerobot_leader::decode_leader_feedback(malformed, minimum, maximum);
    std::cerr << "out-of-format position counts must be rejected as malformed feedback\n";
    return 1;
  } catch (const std::runtime_error &) {
  }
  return 0;
}
