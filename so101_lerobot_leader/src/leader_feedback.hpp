#pragma once

#include <array>
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <feetech_driver/common.hpp>

namespace so101_lerobot_leader {

inline bool is_recoverable_read_timeout(const std::string &detail) {
  std::string normalized = detail;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                 [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
  return normalized.find("readtimeout") != std::string::npos ||
         normalized.find("read timeout") != std::string::npos;
}

struct LeaderFeedback {
  std::array<int, 6> positions{};
  std::array<bool, 6> within_calibration{};
};

inline LeaderFeedback decode_leader_feedback(
    const std::vector<std::array<uint8_t, 2>> &encoded,
    const std::array<int, 6> &range_min,
    const std::array<int, 6> &range_max) {
  if (encoded.size() != range_min.size())
    throw std::runtime_error("position feedback returned an unexpected motor count");
  LeaderFeedback feedback;
  for (size_t i = 0; i < encoded.size(); ++i) {
    const auto word = static_cast<uint16_t>(encoded[i][0] |
        (static_cast<uint16_t>(encoded[i][1]) << 8));
    feedback.positions[i] = feetech_driver::decode_sign_magnitude(word, 15);
    if (feedback.positions[i] < 0 || feedback.positions[i] > 4095)
      throw std::runtime_error("position feedback contains an invalid servo count");
    feedback.within_calibration[i] = feedback.positions[i] >= range_min[i] &&
                                     feedback.positions[i] <= range_max[i];
  }
  return feedback;
}

inline std::string leader_feedback_range_warning(
    const LeaderFeedback &feedback, const std::array<const char *, 6> &joint_names,
    const std::array<int, 6> &range_min, const std::array<int, 6> &range_max) {
  std::string warning;
  for (size_t i = 0; i < feedback.positions.size(); ++i) {
    if (feedback.within_calibration[i]) continue;
    if (!warning.empty()) warning += "; ";
    warning += std::string(joint_names[i]) + "=" + std::to_string(feedback.positions[i]) +
               " outside [" + std::to_string(range_min[i]) + "," +
               std::to_string(range_max[i]) + "]";
  }
  return warning;
}

}  // namespace so101_lerobot_leader
