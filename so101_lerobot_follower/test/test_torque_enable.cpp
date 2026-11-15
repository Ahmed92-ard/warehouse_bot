#include <array>
#include <cstdlib>
#include <cstdint>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>

#include "so101_lerobot_follower/torque_enable.hpp"

namespace {
constexpr std::array<const char *, 6> names{
  "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"};
constexpr std::array<int, 6> ids{1, 2, 3, 4, 5, 6};
const so101_lerobot_follower::CalibrationSet calibration{{
  {1, 0, -20, 100, 900}, {2, 0, 30, 200, 1000}, {3, 0, -40, 300, 1100},
  {4, 0, 50, 400, 1200}, {5, 0, -60, 500, 1300}, {6, 0, 70, 600, 1400}}};
int failures = 0;
void check(bool value, const std::string &what) {
  if (!value) { std::cerr << "FAIL: " << what << '\n'; ++failures; }
}

struct FakeBus {
  std::map<std::pair<int, int>, uint16_t> registers;
  std::array<int, 6> positions{500, 500, 500, 500, 500, 500};
  int fail_enable_id{0};
  int fail_disable_id{0};
  int mismatch_readback_id{0};
  int goal_writes{0};
  int torque_writes{0};
  bool identity_ok{true};

  FakeBus() {
    for (size_t i = 0; i < ids.size(); ++i) {
      registers[{ids[i], 9}] = static_cast<uint16_t>(calibration[i].range_min);
      registers[{ids[i], 11}] = static_cast<uint16_t>(calibration[i].range_max);
      const int offset = calibration[i].homing_offset;
      registers[{ids[i], 31}] = static_cast<uint16_t>((offset < 0 ? 0x0800 : 0) | std::abs(offset));
      registers[{ids[i], 33}] = 0;
      registers[{ids[i], 40}] = 0;
    }
  }
  void verify_motors() {
    if (!identity_ok) throw std::runtime_error("motor ID/model verification failed");
  }
  std::array<int, 6> read_positions() { return positions; }
  int read_register(int id, int address, int) {
    const int value = registers.at({id, address});
    if (address == 40 && id == mismatch_readback_id && value == 1) return 0;
    return value;
  }
  int read_signed_register(int id, int address, int, int sign_bit) {
    const auto raw = static_cast<uint16_t>(read_register(id, address, 2));
    const auto bit = static_cast<uint16_t>(1U << sign_bit);
    const int magnitude = static_cast<int>(raw & (bit - 1U));
    return (raw & bit) ? -magnitude : magnitude;
  }
  void write_register(int id, int address, int, uint16_t value) {
    if (address == 42) { ++goal_writes; return; }
    if (address != 40) throw std::runtime_error("unexpected register write");
    ++torque_writes;
    if (value == 1 && id == fail_enable_id) throw std::runtime_error("injected torque-enable write failure");
    if (value == 0 && id == fail_disable_id) throw std::runtime_error("injected rollback write failure");
    registers[{id, address}] = value;
  }
};

auto logger() { return [](const std::string &, const std::string &) {}; }
auto run(FakeBus &bus, bool fresh_disabled) {
  return so101_lerobot_follower::enable_torque_safely(
    calibration, names, ids, bus, [fresh_disabled] { return fresh_disabled; }, logger());
}
}

int main() {
  {
    FakeBus bus;
    auto result = run(bus, true);
    check(result.success && !result.already_enabled, "valid preconditions enable and verify all six motors");
    check(bus.torque_writes == 6, "one acknowledged torque enable write per motor");
    check(bus.goal_writes == 0, "torque transition never sends goal-position writes");
    for (int id : ids) check(bus.registers[{id, 40}] == 1, "all motor torque registers enabled");
  }
  {
    FakeBus bus;
    auto result = run(bus, false);
    check(!result.success && bus.torque_writes == 0, "missing or stale disabled-session state blocks torque writes");
  }
  {
    FakeBus bus;
    bus.identity_ok = false;
    auto result = run(bus, true);
    check(!result.success && bus.torque_writes == 0, "motor ID/model verification failure blocks torque writes");
  }
  {
    FakeBus bus;
    bus.registers[{4, 11}]++;
    auto result = run(bus, true);
    check(!result.success && bus.torque_writes == 0, "calibration mismatch blocks torque writes");
  }
  {
    FakeBus bus;
    bus.registers[{3, 33}] = 1;
    auto result = run(bus, true);
    check(!result.success && bus.torque_writes == 0, "wrong operating mode blocks torque writes");
  }
  {
    FakeBus bus;
    bus.positions[2] = 4096;
    auto result = run(bus, true);
    check(!result.success && bus.torque_writes == 0, "invalid feedback blocks torque writes");
  }
  {
    FakeBus bus;
    bus.fail_enable_id = 4;
    auto result = run(bus, true);
    check(!result.success, "write failure blocks readiness");
    check(bus.registers[{1, 40}] == 0 && bus.registers[{2, 40}] == 0 &&
          bus.registers[{3, 40}] == 0, "partial enable is rolled back on earlier motors");
    check(bus.goal_writes == 0, "partial failure never sends goal-position writes");
  }
  {
    FakeBus bus;
    int state_checks = 0;
    auto result = so101_lerobot_follower::enable_torque_safely(
      calibration, names, ids, bus, [&state_checks] { return ++state_checks < 3; }, logger());
    check(!result.success && bus.registers[{1, 40}] == 0,
          "leader session changing from disabled aborts and rolls back already-enabled motor");
  }
  {
    FakeBus bus;
    bus.mismatch_readback_id = 2;
    auto result = run(bus, true);
    check(!result.success, "torque readback mismatch blocks readiness");
    check(bus.registers[{1, 40}] == 0 && bus.registers[{2, 40}] == 0,
          "readback mismatch rolls back torque enabled in this attempt");
  }
  {
    FakeBus bus;
    bus.fail_enable_id = 4;
    bus.fail_disable_id = 2;
    auto result = run(bus, true);
    check(!result.success && !result.issues.empty(), "rollback failure is surfaced and cannot report ready");
  }
  {
    FakeBus bus;
    for (int id : ids) bus.registers[{id, 40}] = 1;
    auto result = run(bus, true);
    check(result.success && result.already_enabled && bus.torque_writes == 0,
          "already enabled verified state does not cause extra torque writes");
  }
  return failures == 0 ? 0 : 1;
}
