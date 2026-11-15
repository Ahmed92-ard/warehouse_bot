#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "so101_lerobot_follower/calibration.hpp"

namespace {
constexpr std::array<const char *, 6> kNames{
  "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"};
constexpr std::array<int, 6> kIds{1, 2, 3, 4, 5, 6};

std::string valid_json() {
  return R"({
    "shoulder_pan":{"id":1,"drive_mode":0,"homing_offset":104,"range_min":10,"range_max":900},
    "shoulder_lift":{"id":2,"drive_mode":0,"homing_offset":-120,"range_min":20,"range_max":910},
    "elbow_flex":{"id":3,"drive_mode":0,"homing_offset":-200,"range_min":30,"range_max":920},
    "wrist_flex":{"id":4,"drive_mode":0,"homing_offset":70,"range_min":40,"range_max":930},
    "wrist_roll":{"id":5,"drive_mode":0,"homing_offset":-100,"range_min":50,"range_max":940},
    "gripper":{"id":6,"drive_mode":0,"homing_offset":120,"range_min":60,"range_max":950}
  })";
}

int failures = 0;
void check(bool condition, const std::string &name) {
  if (!condition) { std::cerr << "FAIL: " << name << '\n'; ++failures; }
}

std::filesystem::path write_file(const std::filesystem::path &directory, const std::string &name,
                                 const std::string &contents) {
  const auto path = directory / name;
  std::ofstream output(path);
  output << contents;
  output.close();
  return path;
}

bool throws_for(const std::string &contents, const std::filesystem::path &directory, const std::string &name) {
  const auto path = write_file(directory, name, contents);
  try { (void)so101_lerobot_follower::load_follower_calibration(path.string(), kNames, kIds); }
  catch (const std::exception &) { return true; }
  return false;
}
}  // namespace

int main(int argc, char **argv) {
  const auto directory = std::filesystem::temp_directory_path() /
    ("so101_calibration_regression_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directories(directory);
  const auto valid_path = write_file(directory, "follower.json", valid_json());
  try {
    const auto calibration = so101_lerobot_follower::load_follower_calibration(valid_path.string(), kNames, kIds);
    check(calibration[0].id == 1 && calibration[5].id == 6, "loads six joint records in LeRobot order");
  } catch (const std::exception &error) {
    check(false, std::string("valid calibration loads: ") + error.what());
  }

  check(so101_lerobot_follower::require_follower_calibration_filename(valid_path.string()) == valid_path.string(),
        "accepts named follower.json path");
  bool wrong_name_rejected = false;
  try { (void)so101_lerobot_follower::require_follower_calibration_filename((directory / "None.json").string()); }
  catch (const std::exception &) { wrong_name_rejected = true; }
  check(wrong_name_rejected, "rejects None.json and any non-follower filename");

  bool missing_rejected = false;
  try { (void)so101_lerobot_follower::load_follower_calibration((directory / "missing.json").string(), kNames, kIds); }
  catch (const std::exception &) { missing_rejected = true; }
  check(missing_rejected, "missing calibration file rejected");
  check(throws_for("{ malformed", directory, "follower.json"), "malformed JSON rejected");

  auto missing_joint = valid_json();
  const auto joint_start = missing_joint.find("    \"gripper\"");
  missing_joint.erase(joint_start, missing_joint.find('\n', joint_start) - joint_start + 1);
  check(throws_for(missing_joint, directory, "follower.json"), "missing joint rejected");

  for (const auto &mutation : std::vector<std::pair<std::string, std::string>>{
         {"\"id\":1", "\"id\":9"},
         {"\"drive_mode\":0", "\"drive_mode\":1"},
         {"\"homing_offset\":104", "\"homing_offset\":2048"},
         {"\"range_min\":10", "\"range_min\":900"},
         {"\"range_max\":900", "\"range_max\":4096"},
         {"\"range_min\":10", "\"range_min\":10.5"}}) {
    auto invalid = valid_json();
    const auto at = invalid.find(mutation.first);
    invalid.replace(at, mutation.first.size(), mutation.second);
    check(throws_for(invalid, directory, "follower.json"), "invalid calibration values rejected: " + mutation.second);
  }

  so101_lerobot_follower::Calibration wrist_flex{4, 0, 70, 40, 930};
  check(so101_lerobot_follower::calibration_matches_registers(wrist_flex, 40, 930, 70),
        "matching hardware calibration accepted");
  check(!so101_lerobot_follower::calibration_matches_registers(wrist_flex, 40, 925, 70),
        "wrist_flex register mismatch remains a fault");
  so101_lerobot_follower::Calibration wrist_roll{5, 0, -100, 50, 940};
  check(!so101_lerobot_follower::calibration_matches_registers(wrist_roll, 55, 940, -100),
        "wrist_roll register mismatch remains a fault");
  so101_lerobot_follower::Calibration gripper{6, 0, 120, 60, 950};
  check(!so101_lerobot_follower::calibration_matches_registers(gripper, 65, 950, 120),
        "gripper register mismatch remains a fault");

  if (argc == 2) {
    try {
      const std::string path = so101_lerobot_follower::require_follower_calibration_filename(argv[1]);
      const auto actual_cache = so101_lerobot_follower::load_follower_calibration(path, kNames, kIds);
      std::cout << "validated LeRobot follower calibration path=" << path
                << " joints=" << actual_cache.size() << '\n';
    } catch (const std::exception &error) {
      check(false, std::string("actual follower cache calibration validates: ") + error.what());
    }
  }

  std::filesystem::remove_all(directory);
  return failures == 0 ? 0 : 1;
}
