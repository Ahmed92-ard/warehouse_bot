#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "so101_lerobot_follower/calibration.hpp"

namespace so101_lerobot_follower {

struct CalibrationRegisterSnapshot {
  int minimum{};
  int maximum{};
  int homing_offset{};
  uint16_t homing_offset_raw{};
  int torque_enabled{};
  int operating_mode{};
  int lock{};
};

struct CalibrationSyncIssue {
  std::string joint;
  std::string register_name;
  std::string expected;
  std::string actual;
  std::string error;
};

struct CalibrationSyncReport {
  bool success{false};
  std::array<CalibrationRegisterSnapshot, 6> original{};
  std::vector<CalibrationSyncIssue> issues;
};

inline uint16_t encode_homing_offset(int value) {
  if (value < -2047 || value > 2047) throw std::invalid_argument("homing offset outside sign-magnitude range");
  return static_cast<uint16_t>((value < 0 ? 0x0800 : 0) | std::abs(value));
}

inline int decode_homing_offset(uint16_t value) {
  const int magnitude = static_cast<int>(value & 0x07FF);
  return (value & 0x0800) ? -magnitude : magnitude;
}

// Bus requirements: read_register(id,address,width), read_signed_register(id,address,width,sign_bit),
// and write_register(id,address,width,value). The supplied callbacks are used for the operator
// interlock, durable pre-write backup, and structured logging.
template <typename Bus, typename Backup, typename TeleopStopped, typename Logger>
CalibrationSyncReport synchronize_calibration(const CalibrationSet &calibration,
                                               const std::array<const char *, 6> &joint_names,
                                               const std::array<int, 6> &motor_ids,
                                               Bus &bus, Backup backup,
                                               TeleopStopped teleop_stopped, Logger log) {
  CalibrationSyncReport report;
  auto issue = [&report, &log](const std::string &joint, const std::string &reg,
                               const std::string &expected, const std::string &actual,
                               const std::string &error) {
    report.issues.push_back({joint, reg, expected, actual, error});
    log("ERROR", "joint=" + joint + " register=" + reg + " expected=" + expected +
                 " actual=" + actual + " error=" + error);
  };

  if (!teleop_stopped()) {
    issue("all", "teleop_enabled", "false", "true or unknown", "teleoperation stop was not confirmed");
    return report;
  }

  try {
    // Capture every original register before any operation can write to a motor.
    for (size_t i = 0; i < motor_ids.size(); ++i) {
      auto &saved = report.original[i];
      const int id = motor_ids[i];
      saved.minimum = bus.read_register(id, 9, 2);
      saved.maximum = bus.read_register(id, 11, 2);
      saved.homing_offset_raw = static_cast<uint16_t>(bus.read_register(id, 31, 2));
      saved.homing_offset = decode_homing_offset(saved.homing_offset_raw);
      saved.operating_mode = bus.read_register(id, 33, 1);
      saved.torque_enabled = bus.read_register(id, 40, 1);
      saved.lock = bus.read_register(id, 55, 1);
      log("INFO", "original joint=" + std::string(joint_names[i]) + " id=" + std::to_string(id) +
                   " Homing_Offset=" + std::to_string(saved.homing_offset) +
                   " Homing_Offset_raw=" + std::to_string(saved.homing_offset_raw) +
                   " Min_Position_Limit=" + std::to_string(saved.minimum) +
                   " Max_Position_Limit=" + std::to_string(saved.maximum) +
                   " Torque_Enable=" + std::to_string(saved.torque_enabled) +
                   " Operating_Mode=" + std::to_string(saved.operating_mode) +
                   " Lock=" + std::to_string(saved.lock));
    }
  } catch (const std::exception &error) {
    issue("all", "pre_write_read", "all original registers readable", "read failed", error.what());
    return report;
  }

  try {
    backup(report.original);
  } catch (const std::exception &error) {
    issue("all", "backup", "durable backup created", "not created", error.what());
    return report;
  }

  for (size_t i = 0; i < motor_ids.size(); ++i) {
    const auto &saved = report.original[i];
    if (saved.torque_enabled != 0) {
      issue(joint_names[i], "Torque_Enable(40)", "0", std::to_string(saved.torque_enabled),
            "torque must already be disabled; no torque write was attempted");
    }
    if (saved.operating_mode != 0) {
      issue(joint_names[i], "Operating_Mode(33)", "0", std::to_string(saved.operating_mode),
            "servo must already be in position mode; no mode write was attempted");
    }
    if (saved.lock != 0 && saved.lock != 1) {
      issue(joint_names[i], "Lock(55)", "0 or 1", std::to_string(saved.lock), "invalid EEPROM lock state");
    }
  }
  if (!report.issues.empty()) return report;

  bool write_failed = false;
  for (size_t i = 0; i < motor_ids.size() && !write_failed; ++i) {
    const auto &target = calibration[i];
    const auto &saved = report.original[i];
    const bool offset_changed = saved.homing_offset != target.homing_offset;
    const bool minimum_changed = saved.minimum != target.range_min;
    const bool maximum_changed = saved.maximum != target.range_max;
    if (!offset_changed && !minimum_changed && !maximum_changed) {
      log("INFO", "EEPROM already matches joint=" + std::string(joint_names[i]));
      continue;
    }

    if (!teleop_stopped()) {
      issue(joint_names[i], "teleop_enabled", "false", "true", "teleoperation became enabled during synchronization");
      write_failed = true;
      break;
    }

    bool lock_released = false;
    try {
      if (saved.lock == 1) {
        bus.write_register(motor_ids[i], 55, 1, 0);
        lock_released = true;
        const int lock_readback = bus.read_register(motor_ids[i], 55, 1);
        if (lock_readback != 0) {
          issue(joint_names[i], "Lock(55)", "0", std::to_string(lock_readback), "failed to unlock calibration registers");
          write_failed = true;
        }
      }

      // Match LeRobot 0.6.2 FeetechMotorsBus.write_calibration order and registers.
      const auto write_if_changed = [&](bool changed, const char *name, uint8_t address,
                                        int expected, uint16_t encoded) {
        if (!changed || write_failed) return;
        if (!teleop_stopped()) {
          issue(joint_names[i], "teleop_enabled", "false", "true", "teleoperation became enabled during synchronization");
          write_failed = true;
          return;
        }
        try {
          bus.write_register(motor_ids[i], address, 2, encoded);
          log("INFO", "EEPROM write joint=" + std::string(joint_names[i]) + " id=" +
                      std::to_string(motor_ids[i]) + " register=" + name + " address=" +
                      std::to_string(address) + " expected=" + std::to_string(expected));
        } catch (const std::exception &error) {
          std::string actual = "unavailable";
          std::string read_error;
          try {
            int value = bus.read_register(motor_ids[i], address, 2);
            if (address == 31) value = decode_homing_offset(static_cast<uint16_t>(value));
            actual = std::to_string(value);
          } catch (const std::exception &read_exception) {
            read_error = "; immediate read-back failed: " + std::string(read_exception.what());
          }
          issue(joint_names[i], std::string(name) + "(" + std::to_string(address) + ")",
                std::to_string(expected), actual, std::string("write failed: ") + error.what() + read_error);
          write_failed = true;
        }
      };
      write_if_changed(offset_changed, "Homing_Offset", 31, target.homing_offset,
                       encode_homing_offset(target.homing_offset));
      write_if_changed(minimum_changed, "Min_Position_Limit", 9, target.range_min,
                       static_cast<uint16_t>(target.range_min));
      write_if_changed(maximum_changed, "Max_Position_Limit", 11, target.range_max,
                       static_cast<uint16_t>(target.range_max));
    } catch (const std::exception &error) {
      issue(joint_names[i], "Lock(55)", std::to_string(saved.lock), "operation failed", error.what());
      write_failed = true;
    }

    if (lock_released && saved.lock == 1) {
      try {
        bus.write_register(motor_ids[i], 55, 1, 1);
        const int lock_readback = bus.read_register(motor_ids[i], 55, 1);
        if (lock_readback != 1) {
          issue(joint_names[i], "Lock(55)", "1", std::to_string(lock_readback), "failed to restore original EEPROM lock");
          write_failed = true;
        }
      } catch (const std::exception &error) {
        issue(joint_names[i], "Lock(55)", "1", "write/read failed", error.what());
        write_failed = true;
      }
    }
  }

  bool all_match = true;
  for (size_t i = 0; i < motor_ids.size(); ++i) {
    const auto &target = calibration[i];
    try {
      const int actual_offset = bus.read_signed_register(motor_ids[i], 31, 2, 11);
      const int actual_minimum = bus.read_register(motor_ids[i], 9, 2);
      const int actual_maximum = bus.read_register(motor_ids[i], 11, 2);
      const std::array<std::tuple<const char *, int, int>, 3> checks{{
        {"Homing_Offset(31)", target.homing_offset, actual_offset},
        {"Min_Position_Limit(9)", target.range_min, actual_minimum},
        {"Max_Position_Limit(11)", target.range_max, actual_maximum}}};
      for (const auto &[name, expected, actual] : checks) {
        if (expected != actual) {
          issue(joint_names[i], name, std::to_string(expected), std::to_string(actual), "read-back mismatch");
          all_match = false;
        } else {
          log("INFO", "EEPROM verified joint=" + std::string(joint_names[i]) + " register=" + name +
                      " expected=" + std::to_string(expected) + " actual=" + std::to_string(actual));
        }
      }
    } catch (const std::exception &error) {
      issue(joint_names[i], "calibration_readback", "all required registers readable", "read failed", error.what());
      all_match = false;
    }
  }

  report.success = !write_failed && all_match && report.issues.empty();
  return report;
}

}  // namespace so101_lerobot_follower
