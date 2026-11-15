#include <array>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <signal.h>
#include <stdexcept>
#include <string>
#include <sys/prctl.h>
#include <unistd.h>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <feetech_driver/SMS_STS.h>
#include <feetech_driver/common.hpp>
#include <feetech_driver/communication_protocol.hpp>
#include <feetech_driver/serial_port.hpp>
#include <libserial/SerialPortConstants.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "leader_feedback.hpp"

namespace {
using namespace std::chrono_literals;
constexpr std::array<const char *, 6> kJointNames{
    "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"};
constexpr std::array<uint8_t, 6> kMotorIds{1, 2, 3, 4, 5, 6};
constexpr int kExpectedModel = 777;

void set_parent_death_signal() {
  const pid_t parent_pid = getppid();
  if (prctl(PR_SET_PDEATHSIG, SIGTERM) == -1) {
    return;
  }
  if (getppid() != parent_pid) {
    kill(getpid(), SIGTERM);
  }
}
}

class LeaderStateNode final : public rclcpp::Node {
 public:
  LeaderStateNode() : Node("leader_state", "so101") {
    port_ = declare_parameter<std::string>("leader_port", "/dev/ttyACM0");
    calibration_file_ = declare_parameter<std::string>("leader_calibration_file", "");
    baud_rate_ = declare_parameter<int>("baud_rate", 1000000);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 30.0);
    state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/so101/leader/joint_states", 10);
    diagnostic_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/so101/leader/driver_diagnostics", 10);
    reconnect_service_ = create_service<std_srvs::srv::Trigger>(
        "/so101/leader_state/reconnect",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          if (protocol_) {
            response->success = true;
            response->message = "Leader serial driver is already connected";
            return;
          }
          retry_disabled_ = false;
          next_attempt_ = std::chrono::steady_clock::now();
          response->success = true;
          response->message = "Leader serial reconnect scheduled";
          RCLCPP_WARN(get_logger(), "Reconnect requested from leader GUI; retrying serial verification");
        });
    timer_ = create_wall_timer(10ms, [this] { tick(); });
    publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::WARN, "waiting for leader device");
  }

 private:
  std::string port_, calibration_file_;
  int baud_rate_{1000000};
  double publish_rate_hz_{30.0};
  bool calibration_loaded_{false};
  bool retry_disabled_{false};
  bool connection_established_once_{false};
  bool read_pause_active_{false};
  std::array<int, 6> range_min_{}, range_max_{}, homing_offset_{}, drive_mode_{};
  std::unique_ptr<feetech_driver::CommunicationProtocol> protocol_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reconnect_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::steady_clock::time_point next_attempt_{};
  std::chrono::steady_clock::time_point next_read_{};

  void publish_diagnostic(uint8_t level, const std::string &message) {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "feetech_read_only_state";
    status.hardware_id = port_;
    status.level = level;
    status.message = message;
    diagnostic_msgs::msg::KeyValue device;
    device.key = "device";
    device.value = port_;
    diagnostic_msgs::msg::KeyValue access;
    access.key = "access";
    access.value = "read-only; no motor writes issued";
    status.values = {device, access};
    array.status.push_back(std::move(status));
    diagnostic_pub_->publish(array);
  }

  void load_calibration() {
    if (calibration_loaded_) return;
    if (calibration_file_.empty()) throw std::runtime_error("leader_calibration_file is required");
    std::ifstream input(calibration_file_);
    if (!input) throw std::runtime_error("cannot open leader calibration file: " + calibration_file_);
    nlohmann::json document;
    try {
      input >> document;
      for (size_t i = 0; i < kJointNames.size(); ++i) {
        const auto &joint = document.at(kJointNames[i]);
        if (joint.at("id").get<int>() != kMotorIds[i])
          throw std::runtime_error(std::string("calibration motor ID mismatch for ") + kJointNames[i]);
        range_min_[i] = joint.at("range_min").get<int>();
        range_max_[i] = joint.at("range_max").get<int>();
        homing_offset_[i] = joint.at("homing_offset").get<int>();
        drive_mode_[i] = joint.at("drive_mode").get<int>();
        if (range_min_[i] < 0 || range_max_[i] > 4095 || range_min_[i] >= range_max_[i] ||
            homing_offset_[i] < -2047 || homing_offset_[i] > 2047 || drive_mode_[i] != 0)
          throw std::runtime_error(std::string("invalid calibration values for ") + kJointNames[i]);
      }
    } catch (const nlohmann::json::exception &e) {
      throw std::runtime_error("invalid leader calibration " + calibration_file_ + ": " + e.what());
    }
    calibration_loaded_ = true;
  }

  void validate_device_path() const {
    std::error_code ec;
    if (!std::filesystem::exists(port_, ec) || ec)
      throw std::runtime_error("leader device does not exist: " + port_);
    if (!std::filesystem::is_character_file(port_, ec) || ec)
      throw std::runtime_error("leader path is not a serial character device: " + port_);
    if (::access(port_.c_str(), R_OK | W_OK) != 0)
      throw std::runtime_error("leader device is not readable and writable by this user: " + port_ +
                               "; check device permissions/group membership");
  }

  int read_word(uint8_t id, uint8_t address) {
    std::array<uint8_t, 2> bytes{};
    const auto result = protocol_->read<2>(id, address, &bytes);
    if (!result)
      throw std::runtime_error("read motor " + std::to_string(id) + " register " +
                               std::to_string(address) + " failed: " + result.error());
    return static_cast<int>(bytes[0]) | (static_cast<int>(bytes[1]) << 8);
  }

  void verify_motors() {
    for (size_t i = 0; i < kMotorIds.size(); ++i) {
      const uint8_t id = kMotorIds[i];
      const auto ping = protocol_->ping(id);
      if (!ping) throw std::runtime_error("motor " + std::to_string(id) + " did not respond: " + ping.error());
      const auto model = protocol_->read_model_number(id);
      if (!model) throw std::runtime_error("model read failed for motor " + std::to_string(id) + ": " + model.error());
      if (model.value() != kExpectedModel)
        throw std::runtime_error("motor " + std::to_string(id) + " model mismatch: expected STS3215/777, got " +
                                 std::to_string(model.value()));

      const int actual_min = read_word(id, SMS_STS_MIN_ANGLE_LIMIT_L);
      const int actual_max = read_word(id, SMS_STS_MAX_ANGLE_LIMIT_L);
      const int actual_offset = feetech_driver::decode_sign_magnitude(
          static_cast<uint16_t>(read_word(id, SMS_STS_OFS_L)), SMS_STS_SIGN_BIT_HOMING_OFFSET);
      std::array<uint8_t, 1> torque{}, mode{};
      const auto torque_result = protocol_->read<1>(id, SMS_STS_TORQUE_ENABLE, &torque);
      const auto mode_result = protocol_->read<1>(id, SMS_STS_MODE, &mode);
      if (!torque_result || !mode_result)
        throw std::runtime_error("torque/mode readback failed for motor " + std::to_string(id));
      if (actual_min != range_min_[i] || actual_max != range_max_[i] || actual_offset != homing_offset_[i] ||
          static_cast<int>(mode[0]) != drive_mode_[i] || torque[0] != 0)
        throw std::runtime_error("calibration, position mode, or torque-off readback failed for motor " +
                                 std::to_string(id));
    }
  }

  void connect_read_only() {
    const bool initial_connection = !connection_established_once_;
    load_calibration();
    validate_device_path();
    const auto baud = feetech_driver::to_baudrate(static_cast<size_t>(baud_rate_));
    if (!baud) throw std::runtime_error("unsupported Feetech baud rate " + std::to_string(baud_rate_));

    auto serial = std::make_unique<feetech_driver::SerialPort>(port_);
    // The public driver lifecycle is configure(); it opens the selected device before setting its baud rate.
    feetech_driver::Result configured;
    try {
      configured = serial->configure(baud.value());
    } catch (const LibSerial::NotOpen &e) {
      // The existing SerialPort destructor calls Close() unconditionally. Retain this unopened wrapper
      // until process exit because destroying it would throw a second NotOpen from the shared driver.
      (void)serial.release();
      retry_disabled_ = true;
      throw std::runtime_error("shared driver reported NotOpen while configuring " + port_ + ": " + e.what());
    }
    if (!configured) {
      const std::string detail = configured.error();
      if (detail.starts_with("Open [")) {
        // Open failed, so the wrapper is unopened; its current destructor is not safe in that state.
        (void)serial.release();
        retry_disabled_ = true;
      }
      throw std::runtime_error("cannot open/configure " + port_ + ": " + detail);
    }
    protocol_ = std::make_unique<feetech_driver::CommunicationProtocol>(std::move(serial));
    try {
      verify_motors();
    } catch (const std::exception &e) {
      protocol_.reset();
      throw std::runtime_error(std::string("required motor verification failed: ") + e.what());
    }
    connection_established_once_ = true;
    if (initial_connection) RCLCPP_INFO(get_logger(), "READY — Leader serial connected");
    publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::OK, "connected; all six motors verified read-only");
    next_read_ = std::chrono::steady_clock::now();
  }

  void tick() {
    const auto now_steady = std::chrono::steady_clock::now();
    if (retry_disabled_ && !protocol_) return;
    if (!protocol_) {
      if (now_steady < next_attempt_) return;
      try {
        connect_read_only();
      } catch (const LibSerial::NotOpen &e) {
        (void)protocol_.release();
        retry_disabled_ = true;
        publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                           std::string("FATAL_LEADER_DRIVER_FAULT: serial port closed during verification: ") + e.what() +
                           "; restart the leader node after checking the USB connection");
        RCLCPP_ERROR(get_logger(), "DISARMED — Leader serial connection failed");
      } catch (const std::exception &e) {
        const std::string detail = e.what();
        protocol_.reset();
        // Only a transport timeout during a post-startup re-verification is
        // recoverable. Initial verification and all ambiguous failures are fatal.
        if (connection_established_once_ &&
            so101_lerobot_leader::is_recoverable_read_timeout(detail)) {
          next_attempt_ = std::chrono::steady_clock::now() + 1s;
          publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                             "RECOVERABLE_LEADER_FEEDBACK_LOSS: reconnect verification timed out; retry scheduled: " + detail);
          if (!read_pause_active_) RCLCPP_WARN(get_logger(), "PAUSED — Waiting for leader feedback");
          read_pause_active_ = true;
        } else {
          retry_disabled_ = true;
          publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                             "FATAL_LEADER_DRIVER_FAULT: " + detail);
          RCLCPP_ERROR(get_logger(), "DISARMED — Leader serial connection or startup verification failed: %s",
                       detail.c_str());
        }
      }
      return;
    }
    if (now_steady < next_read_) return;
    next_read_ = now_steady + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / publish_rate_hz_));
    try {
      std::vector<uint8_t> ids(kMotorIds.begin(), kMotorIds.end());
      std::vector<std::array<uint8_t, 2>> data;
      const auto result = protocol_->sync_read<2>(ids, SMS_STS_PRESENT_POSITION_L, &data);
      if (!result) {
        const std::string detail = result.error();
        if (so101_lerobot_leader::is_recoverable_read_timeout(detail))
          throw std::runtime_error("RECOVERABLE_READ_TIMEOUT: " + detail);
        throw std::runtime_error("FATAL_AMBIGUOUS_POSITION_READ: " + detail);
      }
      const auto feedback = so101_lerobot_leader::decode_leader_feedback(data, range_min_, range_max_);
      sensor_msgs::msg::JointState msg;
      msg.header.stamp = now();
      msg.name.assign(kJointNames.begin(), kJointNames.end());
      for (size_t i = 0; i < data.size(); ++i) {
        msg.position.push_back(feedback.positions[i]);
      }
      state_pub_->publish(msg);
      if (read_pause_active_) {
        RCLCPP_INFO(get_logger(), "RESUMED — Leader feedback restored");
        read_pause_active_ = false;
      }
      const auto range_warning = so101_lerobot_leader::leader_feedback_range_warning(
          feedback, kJointNames, range_min_, range_max_);
      if (range_warning.empty()) {
        publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::OK, "read-only feedback healthy");
      } else {
        publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                           "feedback published; " + range_warning +
                           "; leader-to-follower mapping clamps targets to follower calibration limits");
      }
    } catch (const LibSerial::NotOpen &e) {
      // NotOpen means the underlying SerialPort is already closed. Releasing the wrapper avoids
      // triggering its current unconditional Close() destructor; restart the node after fixing USB.
      (void)protocol_.release();
      retry_disabled_ = true;
      publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                         std::string("FATAL_LEADER_DRIVER_FAULT: serial port closed unexpectedly: ") + e.what() +
                         "; restart the leader node after checking the USB connection");
      RCLCPP_ERROR(get_logger(), "DISARMED — Leader serial connection failed");
    } catch (const std::exception &e) {
      const std::string detail = e.what();
      protocol_.reset();
      if (detail.starts_with("RECOVERABLE_READ_TIMEOUT:")) {
        next_attempt_ = std::chrono::steady_clock::now() + 1s;
        publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                           "RECOVERABLE_LEADER_FEEDBACK_LOSS: read timeout; reconnect retry scheduled: " + detail);
        if (!read_pause_active_) RCLCPP_WARN(get_logger(), "PAUSED — Waiting for leader feedback");
        read_pause_active_ = true;
      } else {
        retry_disabled_ = true;
        publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                           "FATAL_LEADER_DRIVER_FAULT: " + detail);
        RCLCPP_ERROR(get_logger(), "DISARMED — Leader feedback invalid or driver fault confirmed: %s",
                     detail.c_str());
      }
    }
  }
};

int main(int argc, char **argv) {
  set_parent_death_signal();
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LeaderStateNode>());
  rclcpp::shutdown();
  return 0;
}
