#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

namespace so101_lerobot_follower {

struct Calibration {
  int id{};
  int drive_mode{};
  int homing_offset{};
  int range_min{};
  int range_max{};
};

using CalibrationSet = std::array<Calibration, 6>;

inline std::string require_follower_calibration_filename(const std::string &path) {
  std::filesystem::path resolved;
  try {
    resolved = std::filesystem::canonical(path);
  } catch (const std::filesystem::filesystem_error &error) {
    throw std::runtime_error("cannot resolve follower calibration file " + path + ": " + error.what());
  }
  if (resolved.filename() != "follower.json") {
    throw std::runtime_error("follower calibration path must resolve to the named follower.json file: " +
                             resolved.string());
  }
  return resolved.string();
}

inline int calibration_integer(const nlohmann::json &record, const std::string &joint,
                               const std::string &field) {
  if (!record.contains(field) || !(record.at(field).is_number_integer() || record.at(field).is_number_unsigned())) {
    throw std::runtime_error("follower calibration " + joint + "." + field + " must be an integer");
  }
  if (record.at(field).is_number_unsigned()) {
    const uint64_t value = record.at(field).get<uint64_t>();
    if (value > static_cast<uint64_t>(std::numeric_limits<int>::max())) {
      throw std::runtime_error("follower calibration " + joint + "." + field + " is outside integer range");
    }
    return static_cast<int>(value);
  }
  const int64_t value = record.at(field).get<int64_t>();
  if (value < std::numeric_limits<int>::min() || value > std::numeric_limits<int>::max())
    throw std::runtime_error("follower calibration " + joint + "." + field + " is outside integer range");
  return static_cast<int>(value);
}

inline CalibrationSet load_follower_calibration(const std::string &path,
                                                 const std::array<const char *, 6> &joint_names,
                                                 const std::array<int, 6> &motor_ids) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot read follower calibration file " + path);

  nlohmann::json document;
  try {
    input >> document;
  } catch (const nlohmann::json::exception &error) {
    throw std::runtime_error("cannot parse follower calibration " + path + ": " + error.what());
  }
  if (!document.is_object()) throw std::runtime_error("follower calibration root must be an object: " + path);

  std::set<std::string> expected_names;
  for (const auto *name : joint_names) expected_names.emplace(name);
  std::set<std::string> actual_names;
  for (auto it = document.begin(); it != document.end(); ++it) actual_names.insert(it.key());
  if (actual_names != expected_names) {
    throw std::runtime_error("follower calibration must contain exactly the six SO-101 joints; missing or extra keys in " + path);
  }

  CalibrationSet result{};
  for (size_t i = 0; i < joint_names.size(); ++i) {
    const std::string joint(joint_names[i]);
    const auto &record = document.at(joint);
    if (!record.is_object()) throw std::runtime_error("follower calibration entry for " + joint + " must be an object");
    Calibration c{
      calibration_integer(record, joint, "id"),
      calibration_integer(record, joint, "drive_mode"),
      calibration_integer(record, joint, "homing_offset"),
      calibration_integer(record, joint, "range_min"),
      calibration_integer(record, joint, "range_max")};
    if (c.id != motor_ids[i]) {
      throw std::runtime_error("follower calibration " + joint + ".id=" + std::to_string(c.id) +
                               " expected " + std::to_string(motor_ids[i]));
    }
    if (c.drive_mode != 0) throw std::runtime_error("follower calibration " + joint + ".drive_mode must be 0");
    if (c.homing_offset < -2047 || c.homing_offset > 2047) {
      throw std::runtime_error("follower calibration " + joint + ".homing_offset outside [-2047,2047]");
    }
    if (c.range_min < 0 || c.range_min >= c.range_max || c.range_max > 4095) {
      throw std::runtime_error("follower calibration " + joint + " position limits invalid: [" +
                               std::to_string(c.range_min) + "," + std::to_string(c.range_max) +
                               "] (expected 0 <= min < max <= 4095)");
    }
    result[i] = c;
  }
  return result;
}

inline bool calibration_matches_registers(const Calibration &expected, int actual_min, int actual_max,
                                          int actual_offset) {
  return actual_min == expected.range_min && actual_max == expected.range_max &&
         actual_offset == expected.homing_offset;
}

}  // namespace so101_lerobot_follower
