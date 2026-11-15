#include <array>
#include <cstdint>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>

#include "so101_lerobot_follower/calibration_sync.hpp"

namespace {
constexpr std::array<const char *, 6> names{
  "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"};
constexpr std::array<int, 6> ids{1, 2, 3, 4, 5, 6};
const so101_lerobot_follower::CalibrationSet targets{{
  {1, 0, -20, 100, 900}, {2, 0, 30, 200, 1000}, {3, 0, -40, 300, 1100},
  {4, 0, 50, 400, 1200}, {5, 0, -60, 500, 1300}, {6, 0, 70, 600, 1400}}};
int failures = 0;
void check(bool condition, const std::string &name) {
  if (!condition) { std::cerr << "FAIL: " << name << '\n'; ++failures; }
}

struct FakeBus {
  std::map<std::pair<int, int>, uint16_t> registers;
  int writes{0};
  int fail_id{0};
  int fail_address{-1};
  int corrupt_read_id{0};
  int corrupt_read_address{-1};
  bool backup_completed{false};

  FakeBus() {
    for (int id : ids) {
      registers[{id, 9}] = 1;
      registers[{id, 11}] = 2;
      registers[{id, 31}] = 0;
      registers[{id, 33}] = 0;
      registers[{id, 40}] = 0;
      registers[{id, 55}] = 1;
    }
  }
  int read_register(int id, int address, int) {
    auto value = registers.at({id, address});
    if (writes && id == corrupt_read_id && address == corrupt_read_address) ++value;
    return value;
  }
  int read_signed_register(int id, int address, int, int sign_bit) {
    const auto raw = static_cast<uint16_t>(read_register(id, address, 2));
    const auto bit = static_cast<uint16_t>(1U << sign_bit);
    const int magnitude = static_cast<int>(raw & (bit - 1U));
    return (raw & bit) ? -magnitude : magnitude;
  }
  void write_register(int id, int address, int, uint16_t value) {
    if (!backup_completed) throw std::runtime_error("write attempted before backup");
    ++writes;
    if (id == fail_id && address == fail_address) throw std::runtime_error("injected write failure");
    registers[{id, address}] = value;
  }
};

auto backup_for(FakeBus &bus) {
  return [&bus](const auto &) { bus.backup_completed = true; };
}
auto stopped() { return [] { return true; }; }
auto quiet_log() { return [](const std::string &, const std::string &) {}; }
}

int main() {
  {
    FakeBus bus;
    auto report = so101_lerobot_follower::synchronize_calibration(
      targets, names, ids, bus, backup_for(bus), stopped(), quiet_log());
    check(report.success, "synchronizes six motors and verifies every target register");
    check(bus.backup_completed && report.original[0].lock == 1,
          "captures original register state and backs it up before writes");
    check(bus.registers[{1, 31}] == so101_lerobot_follower::encode_homing_offset(-20) &&
          bus.registers[{6, 11}] == 1400, "writes signed offset encoding and little-endian register values");
    check(bus.registers[{1, 55}] == 1, "restores original EEPROM lock state");
    bool torque_untouched = true;
    for (int id : ids) torque_untouched = torque_untouched && bus.registers[{id, 40}] == 0;
    check(torque_untouched, "calibration synchronization never enables torque");
  }
  {
    FakeBus bus;
    bus.fail_id = 4; bus.fail_address = 31;
    auto report = so101_lerobot_follower::synchronize_calibration(
      targets, names, ids, bus, backup_for(bus), stopped(), quiet_log());
    check(!report.success && !report.issues.empty(), "write failure prevents synchronization success");
    check(report.issues.front().joint == "wrist_flex" &&
          report.issues.front().expected == "50" && report.issues.front().actual == "0",
          "write failure reports joint, expected value, and actual register value");
  }
  {
    FakeBus bus;
    bus.corrupt_read_id = 2; bus.corrupt_read_address = 9;
    auto report = so101_lerobot_follower::synchronize_calibration(
      targets, names, ids, bus, backup_for(bus), stopped(), quiet_log());
    check(!report.success, "read-back mismatch prevents synchronization success");
    check(!report.issues.empty() && report.issues.back().error == "read-back mismatch",
          "read-back mismatch reports a register-specific issue");
  }
  {
    FakeBus bus;
    bus.registers[{3, 40}] = 1;
    auto report = so101_lerobot_follower::synchronize_calibration(
      targets, names, ids, bus, backup_for(bus), stopped(), quiet_log());
    check(!report.success && bus.writes == 0, "refuses every write when any motor torque is enabled");
  }
  {
    FakeBus bus;
    auto report = so101_lerobot_follower::synchronize_calibration(
      targets, names, ids, bus, backup_for(bus), [] { return false; }, quiet_log());
    check(!report.success && bus.writes == 0, "refuses writes without confirmed stopped teleoperation");
  }
  return failures == 0 ? 0 : 1;
}
