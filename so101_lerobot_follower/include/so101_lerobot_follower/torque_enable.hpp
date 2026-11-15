#pragma once

#include <array>
#include <cstdint>
#include <exception>
#include <functional>
#include <string>
#include <vector>

#include "so101_lerobot_follower/calibration.hpp"

namespace so101_lerobot_follower {

struct TorqueEnableIssue {
  std::string joint;
  std::string register_name;
  std::string expected;
  std::string actual;
  std::string error;
};

struct TorqueEnableReport {
  bool success{false};
  bool write_attempted{false};
  bool already_enabled{false};
  bool rollback_failed{false};
  std::array<int, 6> final_torque_state{{-1, -1, -1, -1, -1, -1}};
  std::vector<TorqueEnableIssue> issues;
};

// Bus contract: verify_motors(), read_positions(), register reads/writes using
// (id,address,width), and signed register reads. No goal-position write exists
// in this operation by design.
template <typename Bus, typename StateIsFreshAndDisabled, typename Logger>
TorqueEnableReport enable_torque_safely(
    const CalibrationSet &calibration, const std::array<const char *, 6> &joint_names,
    const std::array<int, 6> &motor_ids, Bus &bus,
    StateIsFreshAndDisabled state_is_fresh_and_disabled, Logger log) {
  TorqueEnableReport report;
  auto issue = [&report, &log](const std::string &joint, const std::string &reg,
                               const std::string &expected, const std::string &actual,
                               const std::string &error) {
    report.issues.push_back({joint, reg, expected, actual, error});
    log("ERROR", "torque transition joint=" + joint + " register=" + reg +
                 " expected=" + expected + " actual=" + actual + " error=" + error);
  };

  if (!state_is_fresh_and_disabled()) {
    issue("all", "leader_teleop_enabled", "fresh explicit Start", "missing, stale, or disabled",
          "torque enable requires explicit Start and healthy leader diagnostics");
    return report;
  }

  try {
    bus.verify_motors();
  } catch (const std::exception &error) {
    issue("all", "motor_identity", "six expected STS3215 IDs", "verification failed", error.what());
    return report;
  }

  bool all_disabled = true;
  bool all_enabled = true;
  for (size_t i = 0; i < motor_ids.size(); ++i) {
    const int id = motor_ids[i];
    try {
      const auto &expected = calibration[i];
      const int actual_min = bus.read_register(id, 9, 2);
      const int actual_max = bus.read_register(id, 11, 2);
      const int actual_offset = bus.read_signed_register(id, 31, 2, 11);
      if (!calibration_matches_registers(expected, actual_min, actual_max, actual_offset)) {
        issue(joint_names[i], "calibration_registers", "file limits=" +
              std::to_string(expected.range_min) + "," + std::to_string(expected.range_max) +
              " offset=" + std::to_string(expected.homing_offset),
              "EEPROM limits=" + std::to_string(actual_min) + "," + std::to_string(actual_max) +
              " offset=" + std::to_string(actual_offset), "exact calibration parity failed");
      }
      const int mode = bus.read_register(id, 33, 1);
      if (mode != 0) issue(joint_names[i], "Operating_Mode(33)", "0", std::to_string(mode),
                           "servo must already be in position mode");
      const int torque = bus.read_register(id, 40, 1);
      report.final_torque_state[i] = torque;
      all_disabled = all_disabled && torque == 0;
      all_enabled = all_enabled && torque == 1;
      if (torque != 0 && torque != 1)
        issue(joint_names[i], "Torque_Enable(40)", "0 or 1", std::to_string(torque),
              "invalid torque register value");
    } catch (const std::exception &error) {
      issue(joint_names[i], "precondition_register_read", "all registers readable", "read failed", error.what());
    }
  }

  try {
    const auto positions = bus.read_positions();
    for (size_t i = 0; i < positions.size(); ++i) {
      if (positions[i] < 0 || positions[i] > 4095)
        issue(joint_names[i], "Present_Position(56)", "count in [0,4095]",
              std::to_string(positions[i]), "invalid current-position feedback");
      else
        log("INFO", "torque precondition feedback joint=" + std::string(joint_names[i]) +
                    " count=" + std::to_string(positions[i]));
    }
  } catch (const std::exception &error) {
    issue("all", "Present_Position(56)", "valid feedback from all six motors", "read failed", error.what());
  }

  if (!report.issues.empty()) return report;
  if (all_enabled) {
    if (!state_is_fresh_and_disabled()) {
      issue("all", "leader_teleop_enabled", "fresh explicit Start", "disabled or stale",
            "leader readiness expired during precondition validation");
      return report;
    }
    report.success = true;
    report.already_enabled = true;
    log("INFO", "all six Torque_Enable registers already read back as 1; no torque writes needed");
    return report;
  }
  if (!all_disabled) {
    issue("all", "Torque_Enable(40)", "all six motors disabled before transition",
          "mixed torque state", "refusing to alter a partial pre-existing torque state");
    return report;
  }

  std::array<bool, 6> may_have_been_enabled{};
  bool failed = false;
  for (size_t i = 0; i < motor_ids.size(); ++i) {
    if (!state_is_fresh_and_disabled()) {
      issue(joint_names[i], "leader_teleop_enabled", "fresh explicit Start", "disabled or stale",
            "leader readiness changed or expired during torque transition");
      failed = true;
      break;
    }
    report.write_attempted = true;
    may_have_been_enabled[i] = true;  // A lost ACK does not prove the write was rejected.
    try {
      bus.write_register(motor_ids[i], 40, 1, 1);
      const int actual = bus.read_register(motor_ids[i], 40, 1);
      report.final_torque_state[i] = actual;
      if (actual != 1) {
        issue(joint_names[i], "Torque_Enable(40)", "1", std::to_string(actual),
              "torque-enable readback mismatch");
        failed = true;
        break;
      }
      log("INFO", "torque enabled and verified joint=" + std::string(joint_names[i]) +
                  " id=" + std::to_string(motor_ids[i]) + " Torque_Enable(40)=1");
    } catch (const std::exception &error) {
      std::string actual = "unreadable";
      try {
        const int value = bus.read_register(motor_ids[i], 40, 1);
        report.final_torque_state[i] = value;
        actual = std::to_string(value);
        may_have_been_enabled[i] = value == 1;
      } catch (const std::exception &read_error) {
        issue(joint_names[i], "Torque_Enable(40)", "1", actual,
              std::string("write failed: ") + error.what() + "; read-back failed: " + read_error.what());
        failed = true;
        break;
      }
      issue(joint_names[i], "Torque_Enable(40)", "1", actual,
            std::string("write/verification failed: ") + error.what());
      failed = true;
      break;
    }
  }

  if (!failed) {
    for (size_t i = 0; i < motor_ids.size(); ++i) {
      try {
        const int actual = bus.read_register(motor_ids[i], 40, 1);
        report.final_torque_state[i] = actual;
        if (actual != 1) {
          issue(joint_names[i], "Torque_Enable(40)", "1", std::to_string(actual),
                "final all-motor torque verification failed");
          failed = true;
        }
      } catch (const std::exception &error) {
        issue(joint_names[i], "Torque_Enable(40)", "1", "read failed", error.what());
        failed = true;
      }
    }
    if (!state_is_fresh_and_disabled()) {
      issue("all", "leader_teleop_enabled", "fresh explicit Start", "disabled or stale",
            "leader readiness changed during torque verification");
      failed = true;
    }
  }

  if (failed) {
    for (size_t reverse = motor_ids.size(); reverse > 0; --reverse) {
      const size_t i = reverse - 1;
      if (!may_have_been_enabled[i]) continue;
      try {
        bus.write_register(motor_ids[i], 40, 1, 0);
        const int actual = bus.read_register(motor_ids[i], 40, 1);
        report.final_torque_state[i] = actual;
        if (actual != 0)
          report.rollback_failed = true;
        if (actual != 0)
          issue(joint_names[i], "Torque_Enable(40) rollback", "0", std::to_string(actual),
                "rollback readback mismatch");
        else
          log("WARN", "rolled back torque enable joint=" + std::string(joint_names[i]) +
                      " id=" + std::to_string(motor_ids[i]) + " Torque_Enable(40)=0");
      } catch (const std::exception &error) {
        report.rollback_failed = true;
        issue(joint_names[i], "Torque_Enable(40) rollback", "0", "unverified",
              std::string("rollback failed: ") + error.what());
      }
    }
    return report;
  }

  report.success = true;
  return report;
}

}  // namespace so101_lerobot_follower
