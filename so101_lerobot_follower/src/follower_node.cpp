#include <algorithm>
#include <array>
#include <atomic>
#include <condition_variable>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <poll.h>
#include <queue>
#include <sys/ioctl.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <ctime>
#include <cstdio>
#include <cstdlib>
#include <sys/file.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <vector>

#include <nlohmann/json.hpp>
#include "so101_lerobot_follower/action_mapping.hpp"
#include "so101_lerobot_follower/calibration.hpp"
#include "so101_lerobot_follower/calibration_sync.hpp"
#include "so101_lerobot_follower/leader_session_state.hpp"
#include "so101_lerobot_follower/motion_limiter.hpp"
#include "so101_lerobot_follower/torque_enable.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include "so101_lerobot_follower/msg/so101_action.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <feetech_driver/common.hpp>
#include <feetech_driver/communication_protocol.hpp>
#include <feetech_driver/serial_port.hpp>
#include <feetech_driver/SMS_STS.h>

namespace {
using namespace std::chrono_literals;
using Action = so101_lerobot_follower::msg::SO101Action;
constexpr std::array<const char *, 6> kNames{
    "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"};
constexpr std::array<int, 6> kIds{1, 2, 3, 4, 5, 6};
constexpr int kModelSts3215 = 777;
constexpr int kPositionAddress = 56;
constexpr int kGoalAddress = 42;
constexpr int kBroadcastId = 0xFE;
constexpr int kSyncRead = 0x82;
constexpr int kSyncWrite = 0x83;

struct LogEntry {
  std::string level;
  std::string message;
  size_t source_thread;
};

std::string expand_home(std::string value) {
  if (!value.empty() && value[0] == '~') {
    const char *home = std::getenv("HOME");
    value.replace(0, 1, home ? home : "");
  }
  return value;
}

std::string hex_bytes(const std::vector<uint8_t> &bytes) {
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (uint8_t b : bytes) out << std::setw(2) << static_cast<int>(b) << ' ';
  return out.str();
}

// Feetech bus interface backed by feetech_ros2_driver library.
class FeetechBus {
 public:
  FeetechBus(std::string device, int baud, bool debug,
             std::function<void(const std::string &, bool)> logger)
      : device_(std::move(device)), baud_(baud), debug_(debug), logger_(std::move(logger)) {}
  ~FeetechBus() { close(); }

  void verify_motors() {
    for (int id : kIds) {
      const int model = ping(id);
      if (model != kModelSts3215)
        throw std::runtime_error("motor ID " + std::to_string(id) +
          " model mismatch: expected STS3215/777, got " + std::to_string(model));
      logger_("Torque preflight motor identified id=" + std::to_string(id) +
              " model=STS3215 model_number=777 protocol=0", false);
    }
  }

  void open() {
    logger_("Opening serial device=" + device_ + " baud=" + std::to_string(baud_), false);
    auto baud_result = feetech_driver::to_baudrate(baud_);
    if (!baud_result) throw std::runtime_error("invalid baud rate: " + std::to_string(baud_));
    auto serial_port = std::make_unique<feetech_driver::SerialPort>(device_);
    auto conf_result = serial_port->configure(baud_result.value());
    if (!conf_result) {
      throw std::runtime_error("feetech_driver serial configuration failed: " + conf_result.error());
    }
    comm_ = std::make_unique<feetech_driver::CommunicationProtocol>(std::move(serial_port));
    logger_("Serial opened via feetech_ros2_driver; no torque, EEPROM, calibration, or operating-mode writes issued", false);
    for (int id : kIds) {
      auto model = ping(id);
      if (model != kModelSts3215) {
        throw std::runtime_error("motor ID " + std::to_string(id) + " model mismatch: expected STS3215/777, got " +
                                 std::to_string(model));
      }
      logger_("Motor identified id=" + std::to_string(id) + " model=STS3215 model_number=777 protocol=0", false);
    }
  }

  void close() noexcept {
    comm_.reset();
  }

  void set_debug(bool value) { debug_ = value; }

  int ping(int id) {
    if (!comm_) throw std::runtime_error("serial bus is closed");
    auto res = comm_->ping(id);
    if (!res) throw std::runtime_error("ping motor " + std::to_string(id) + " failed: " + res.error());
    auto model_res = comm_->read_model_number(static_cast<uint8_t>(id));
    if (!model_res) throw std::runtime_error("read model motor " + std::to_string(id) + " failed: " + model_res.error());
    return model_res.value();
  }

  std::array<int, 6> read_positions() {
    if (!comm_) throw std::runtime_error("serial bus is closed");
    std::vector<uint8_t> ids(kIds.begin(), kIds.end());
    std::vector<std::array<uint8_t, 2>> data;
    auto res = comm_->sync_read<2>(ids, SMS_STS_PRESENT_POSITION_L, &data);
    if (!res) throw std::runtime_error("sync_read positions failed: " + res.error());
    std::array<int, 6> positions{};
    for (size_t i = 0; i < kIds.size(); ++i) {
      uint16_t encoded = static_cast<uint16_t>(data[i][0]) | (static_cast<uint16_t>(data[i][1]) << 8);
      positions[i] = feetech_driver::decode_sign_magnitude(encoded, 15);
      if (debug_) logger_("Present_Position addr=56 id=" + std::to_string(kIds[i]) +
                           " decoded=" + std::to_string(positions[i]), true);
      if (positions[i] < 0 || positions[i] > 4095)
        throw std::runtime_error("invalid Present_Position for " + std::string(kNames[i]) +
                                 ": decoded count=" + std::to_string(positions[i]) + " outside [0,4095]");
    }
    return positions;
  }

  void write_goals(const std::array<int, 6> &goals) {
    if (!comm_) throw std::runtime_error("serial bus is closed");
    std::vector<uint8_t> ids(kIds.begin(), kIds.end());
    std::vector<std::array<uint8_t, 2>> params(goals.size());
    for (size_t i = 0; i < goals.size(); ++i) {
      if (goals[i] < 0 || goals[i] > 4095)
        throw std::runtime_error("refusing Goal_Position outside servo count range for " + std::string(kNames[i]));
      uint16_t encoded = static_cast<uint16_t>(feetech_driver::encode_sign_magnitude(goals[i], 15));
      params[i][0] = static_cast<uint8_t>(encoded & 0xFF);
      params[i][1] = static_cast<uint8_t>((encoded >> 8) & 0xFF);
    }
    auto res = comm_->sync_write<2>(ids, kGoalAddress, params);
    if (!res) throw std::runtime_error("sync_write Goal_Position failed: " + res.error());
    if (debug_) logger_("SyncWrite Goal_Position addr=42 goals=" + format_counts(goals), true);
  }

  int read_register(int id, uint8_t address, uint8_t width) {
    if (!comm_) throw std::runtime_error("serial bus is closed");
    if (width == 1) {
      std::array<uint8_t, 1> buf{};
      auto res = comm_->read<1>(static_cast<uint8_t>(id), address, &buf);
      if (!res) throw std::runtime_error("read register id=" + std::to_string(id) + " addr=" + std::to_string(address) + " failed: " + res.error());
      return buf[0];
    } else if (width == 2) {
      std::array<uint8_t, 2> buf{};
      auto res = comm_->read<2>(static_cast<uint8_t>(id), address, &buf);
      if (!res) throw std::runtime_error("read register id=" + std::to_string(id) + " addr=" + std::to_string(address) + " failed: " + res.error());
      return static_cast<int>(static_cast<uint16_t>(buf[0]) | (static_cast<uint16_t>(buf[1]) << 8));
    }
    throw std::runtime_error("unsupported register width " + std::to_string(width));
  }

  int read_signed_register(int id, uint8_t address, uint8_t width, int sign_bit) {
    return feetech_driver::decode_sign_magnitude(static_cast<uint16_t>(read_register(id, address, width)), sign_bit);
  }

  void write_register(int id, uint8_t address, uint8_t width, uint16_t value) {
    if (!comm_) throw std::runtime_error("serial bus is closed");
    if (width == 1) {
      if (value > 0xFF) throw std::runtime_error("value exceeds one-byte register width");
      std::array<uint8_t, 1> buf{static_cast<uint8_t>(value & 0xFF)};
      auto res = comm_->write<1>(static_cast<uint8_t>(id), address, buf);
      if (!res) throw std::runtime_error("write register id=" + std::to_string(id) + " failed: " + res.error());
    } else if (width == 2) {
      std::array<uint8_t, 2> buf{static_cast<uint8_t>(value & 0xFF), static_cast<uint8_t>((value >> 8) & 0xFF)};
      auto res = comm_->write<2>(static_cast<uint8_t>(id), address, buf);
      if (!res) throw std::runtime_error("write register id=" + std::to_string(id) + " failed: " + res.error());
    } else {
      throw std::runtime_error("unsupported register width " + std::to_string(width));
    }
  }

  static std::string format_counts(const std::array<int, 6> &values) {
    std::ostringstream s;
    for (size_t i = 0; i < values.size(); ++i) s << (i ? "," : "") << kNames[i] << "=" << values[i];
    return s.str();
  }

 private:
  std::string device_;
  int baud_;
  bool debug_;
  std::function<void(const std::string &, bool)> logger_;
  std::unique_ptr<feetech_driver::CommunicationProtocol> comm_;
};

class CalibrationSyncMonitor final : public rclcpp::Node {
 public:
  CalibrationSyncMonitor() : Node("follower_calibration_sync", "so101") {
    sub_ = create_subscription<std_msgs::msg::Bool>(
      "/so101/leader/teleop_enabled", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        teleop_enabled_.store(msg->data);
        have_state_.store(true);
        cv_.notify_all();
      });
  }

  bool wait_for_stopped(std::chrono::seconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout, [this] {
      return have_state_.load() && !teleop_enabled_.load();
    });
  }
  bool stopped() const { return have_state_.load() && !teleop_enabled_.load(); }
  bool enabled() const { return have_state_.load() && teleop_enabled_.load(); }

 private:
  std::atomic<bool> have_state_{false};
  std::atomic<bool> teleop_enabled_{false};
  std::mutex mutex_;
  std::condition_variable cv_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
};

std::string resolve_default_follower_calibration() {
  try {
    const std::string share_path = ament_index_cpp::get_package_share_directory("so101_lerobot_follower") + "/config/follower.json";
    if (std::filesystem::exists(share_path)) return share_path;
  } catch (...) {}
  return "config/follower.json";
}

int run_calibration_sync(const rclcpp::Node::SharedPtr &monitor) {
  const auto serial_port = expand_home(monitor->declare_parameter<std::string>(
    "serial_port", "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14114210-if00"));
  const int baud_rate = monitor->declare_parameter<int>("baud_rate", 1000000);
  auto cal_param = monitor->declare_parameter<std::string>("calibration_file", "");
  if (cal_param.empty() || cal_param.find("huggingface") != std::string::npos) {
    cal_param = resolve_default_follower_calibration();
  }
  const auto calibration_path = expand_home(cal_param);
  const auto backup_directory = expand_home(monitor->declare_parameter<std::string>(
    "calibration_backup_directory", "~/.ros/so101_lerobot_follower/calibration_backups"));
  const auto resolved_path = so101_lerobot_follower::require_follower_calibration_filename(calibration_path);
  const auto calibration = so101_lerobot_follower::load_follower_calibration(resolved_path, kNames, kIds);

  auto logger = [monitor](const std::string &level, const std::string &message) {
    if (level == "ERROR") RCLCPP_ERROR(monitor->get_logger(), "%s", message.c_str());
    else RCLCPP_INFO(monitor->get_logger(), "%s", message.c_str());
  };
  logger("INFO", "maintenance EEPROM synchronization requested; calibration=" + resolved_path +
                  " serial=" + serial_port + " baud=" + std::to_string(baud_rate));
  logger("INFO", "waiting for explicit /so101/leader/teleop_enabled=false; no EEPROM writes until confirmed");
  auto typed_monitor = std::static_pointer_cast<CalibrationSyncMonitor>(monitor);
  if (!typed_monitor->wait_for_stopped(10s)) {
    logger("ERROR", "synchronization refused: no explicit stopped teleoperation state received within 10 seconds");
    return 2;
  }

  FeetechBus bus(serial_port, baud_rate, false,
    [logger](const std::string &message, bool debug) {
      if (!debug) logger("INFO", message);
    });
  try {
    bus.open();
  } catch (const std::exception &error) {
    logger("ERROR", std::string("synchronization failed opening motor bus: ") + error.what());
    return 2;
  }

  auto backup = [backup_directory, resolved_path, serial_port, &logger](
      const std::array<so101_lerobot_follower::CalibrationRegisterSnapshot, 6> &original) {
    namespace fs = std::filesystem;
    fs::create_directories(backup_directory);
    const auto stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    const fs::path path = fs::path(backup_directory) /
      ("follower_eeprom_before_sync_" + std::to_string(stamp) + ".json");
    nlohmann::json document;
    document["calibration_file"] = resolved_path;
    document["serial_device"] = serial_port;
    document["created_unix_ns"] = stamp;
    for (size_t i = 0; i < kNames.size(); ++i) {
      const auto &saved = original[i];
      document["motors"][kNames[i]] = {
        {"id", kIds[i]}, {"homing_offset", saved.homing_offset},
        {"homing_offset_raw", saved.homing_offset_raw},
        {"min_position_limit", saved.minimum}, {"max_position_limit", saved.maximum},
        {"torque_enable", saved.torque_enabled}, {"operating_mode", saved.operating_mode},
        {"lock", saved.lock}};
    }
    const fs::path temporary = path.string() + ".tmp";
    {
      std::ofstream output(temporary, std::ios::out | std::ios::trunc);
      if (!output) throw std::runtime_error("cannot create EEPROM backup " + temporary.string());
      output << document.dump(2) << '\n';
      output.flush();
      if (!output) throw std::runtime_error("failed writing EEPROM backup " + temporary.string());
    }
    fs::rename(temporary, path);
    logger("INFO", "original EEPROM register backup saved path=" + path.string());
  };
  auto stopped = [typed_monitor] { return typed_monitor->stopped(); };
  auto report = so101_lerobot_follower::synchronize_calibration(
    calibration, kNames, kIds, bus, backup, stopped, logger);
  bus.close();
  if (!report.success) {
    logger("ERROR", "CALIBRATION_SYNC_FAILED issues=" + std::to_string(report.issues.size()) +
                     "; normal follower readiness remains blocked until every register matches");
    return 1;
  }
  logger("INFO", "CALIBRATION_SYNC_SUCCESS all six motors verified against " + resolved_path);
  return 0;
}

class FollowerNode final : public rclcpp::Node {
 public:
  FollowerNode() : Node("follower", "so101") {
    serial_port_ = declare_parameter<std::string>("serial_port", "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14114210-if00");
    baud_rate_ = declare_parameter<int>("baud_rate", 1000000);
    auto cal_param = declare_parameter<std::string>("calibration_file", "");
    if (cal_param.empty() || cal_param.find("huggingface") != std::string::npos) {
      cal_param = resolve_default_follower_calibration();
    }
    calibration_file_ = expand_home(cal_param);
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 50.0);
    max_action_age_ms_ = declare_parameter<int>("max_action_age_ms", 500);
    max_joint_speed_deg_s_ = declare_parameter<double>("max_joint_speed_deg_s", 20.0);
    max_gripper_speed_percent_s_ = declare_parameter<double>("max_gripper_speed_percent_s", 30.0);
    debug_logging_ = declare_parameter<bool>("debug_logging", false);
    log_path_ = expand_home(declare_parameter<std::string>("log_file", "~/.ros/so101_lerobot_follower.log"));
    validate_parameters();
    open_log();
    log_line("INFO", "startup ROS 2 Jazzy node=/so101/follower package=so101_lerobot_follower/0.1.0");
    log_line("INFO", "serial=" + serial_port_ + " baud=" + std::to_string(baud_rate_) +
                      " protocol=Feetech SCS/STS v0 timeout=35ms");

    // Startup hardware validation can publish the first feedback sample below,
    // so every publisher must exist before opening or probing the serial bus.
    feedback_pub_ = create_publisher<Action>("/so101/follower/action_feedback", rclcpp::QoS(1).reliable());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/so101/follower/diagnostics", 10);

    try {
      load_calibration();
      bus_ = std::make_unique<FeetechBus>(serial_port_, baud_rate_, debug_logging_,
        [this](const std::string &m, bool debug) { this->motor_log(m, debug); });
      bus_->open();
      inspect_motor_configuration();
      hardware_ready_ = true;
      auto actual = bus_->read_positions();
      feedback_valid_ = true;
      if (calibration_valid_) publish_feedback(actual);
      log_line("INFO", "hardware connected; read-only LeRobot calibration parity, mode, torque, and feedback checks completed");
    } catch (const std::exception &e) {
      log_line("ERROR", std::string("hardware startup failed: ") + e.what());
      operator_line("ERROR", "DISARMED — Follower hardware not ready");
      if (bus_) bus_->close();
      hardware_ready_ = false;
    }

    session_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions session_options;
    session_options.callback_group = session_callback_group_;
    session_sub_ = create_subscription<std_msgs::msg::Bool>("/so101/leader/teleop_enabled", rclcpp::QoS(1).reliable().durability_volatile(),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) { on_session(msg->data); }, session_options);
    leader_diag_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/so101/leader/driver_diagnostics", rclcpp::QoS(10).reliable(),
      [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) { on_leader_diagnostics(*msg); });
    log_line("INFO", "disarmed; torque enable requires explicit Start plus fresh healthy read-only leader diagnostics");
    action_sub_ = create_subscription<Action>("/so101/leader/action", rclcpp::QoS(1).reliable(),
      [this](Action::ConstSharedPtr msg) { on_action(*msg); });
    log_line("INFO", "ROS pub=/so101/follower/action_feedback custom SO101Action; diagnostics=/so101/follower/diagnostics");
    log_line("INFO", "ROS sub=/so101/leader/action custom SO101Action and /so101/leader/teleop_enabled std_msgs/Bool");
    log_line("INFO", "joint order=" + joint_list() + " action units=degrees for joints 1-5, percent [0,100] for gripper");
    status_timer_ = create_wall_timer(1s, [this] { enforce_session_freshness(); publish_diagnostics(); });
    feedback_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / control_rate_hz_), [this] { read_cycle(); });
    reconnect_timer_ = create_wall_timer(3s, [this] { try_reconnect(); });
    param_cb_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &params) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto &p : params) {
        if (p.get_name() == "debug_logging") {
          debug_logging_ = p.as_bool();
          const auto logging_result = rcutils_logging_set_logger_level(get_logger().get_name(),
            debug_logging_ ? RCUTILS_LOG_SEVERITY_DEBUG : RCUTILS_LOG_SEVERITY_INFO);
          if (logging_result != RCUTILS_RET_OK)
            log_line("WARN", "could not update ROS console logger severity");
          if (bus_) bus_->set_debug(debug_logging_);
          log_line("INFO", std::string("debug logging ") + (debug_logging_ ? "enabled" : "disabled"));
        }
      }
      return result;
    });
    torque_worker_ = std::thread([this] { torque_enable_worker(); });
  }

  ~FollowerNode() override {
    {
      std::lock_guard<std::mutex> guard(torque_worker_mutex_);
      stop_torque_worker_ = true;
      torque_check_pending_ = false;
    }
    torque_worker_cv_.notify_one();
    if (torque_worker_.joinable()) torque_worker_.join();
    {
      std::lock_guard<std::mutex> gate(command_gate_mutex_);
      disarm_session_locked("node shutdown", "~FollowerNode");
    }
    if (bus_) bus_->close();
    log_line("INFO", "shutdown: command session stopped; serial lock released without torque writes");
    {
      std::lock_guard<std::mutex> guard(log_queue_mutex_);
      stop_log_thread_ = true;
    }
    log_cv_.notify_one();
    if (log_thread_.joinable()) log_thread_.join();
    if (log_.is_open()) log_.close();
  }

 private:
  void disarm_session_locked(const std::string &reason, const std::string &source) {
    const bool was_active = active_session_.exchange(false);
    if (!was_active) return;
    clear_motion_target_locked(reason);
    const std::string lower_reason = reason;
    if (lower_reason.find("heartbeat stale") != std::string::npos) {
      operator_line("WARN", "DISARMED — Session heartbeat lost");
    } else if (lower_reason.find("fatal leader driver") != std::string::npos) {
      operator_line("ERROR", "DISARMED — Leader driver fault confirmed");
    } else if (lower_reason.find("leader published teleop_enabled=false") != std::string::npos) {
      operator_line("WARN", "STOPPED — Leader disabled teleoperation");
    } else if (lower_reason.find("Start rejected") != std::string::npos) {
      operator_line("WARN", "WARNING — Start blocked by readiness checks");
    } else if (lower_reason.find("follower") != std::string::npos ||
               lower_reason.find("torque") != std::string::npos ||
               lower_reason.find("Goal_Position") != std::string::npos) {
      operator_line("ERROR", "DISARMED — Follower hardware fault confirmed");
    } else if (lower_reason.find("shutdown") == std::string::npos) {
      operator_line("WARN", "STOPPED — Teleoperation session ended");
    }

    const int64_t steady_now = steady_now_ns();
    const int64_t diagnostics_receipt = leader_diagnostics_receipt_ns_.load();
    const int64_t diagnostics_age_ms = diagnostics_receipt > 0 && steady_now >= diagnostics_receipt
        ? (steady_now - diagnostics_receipt) / 1000000LL : -1;
    std::string hardware_error;
    {
      std::lock_guard<std::mutex> error_guard(hardware_error_mutex_);
      hardware_error = last_hardware_error_;
    }
    std::ostringstream detail;
    detail << "SESSION_DISARM source=" << source << " reason=" << reason
           << " previous=true new=false"
           << " leader_heartbeat_age_ms=" << session_state_.last_state_age_ms(steady_now)
           << " leader_heartbeat_last_received_value=" << (session_state_.enabled() ? "true" : "false")
           << " leader_diagnostics_age_ms=" << diagnostics_age_ms
           << " leader_diagnostics_readiness=" << (leader_diagnostics_healthy_.load() ? "healthy" : "unhealthy")
           << " leader_diagnostics_fresh=" << (leader_ready_fresh(steady_now) ? "true" : "false")
           << " last_received_action_sequence=";
    if (last_received_sequence_seen_.load()) detail << last_received_sequence_.load();
    else detail << "none";
    detail << " last_accepted_action_sequence=";
    if (received_sequence_) detail << last_sequence_;
    else detail << "none";
    detail << " limiter_target_present=" << (motion_target_active_ ? "true" : "false")
           << " serial_read_health={hardware_ready=" << (hardware_ready_.load() ? "true" : "false")
           << ",feedback_valid=" << (feedback_valid_.load() ? "true" : "false")
           << ",successful_reads=" << successful_reads_.load() << ",read_errors=" << read_errors_.load() << "}"
           << " serial_write_health={successful_goal_writes=" << successful_writes_.load()
           << ",last_hardware_error=" << (hardware_error.empty() ? "none" : hardware_error) << "}";
    log_line("ERROR", detail.str());
  }

  void validate_parameters() {
    if (baud_rate_ <= 0 || control_rate_hz_ < 1.0 || control_rate_hz_ > 100.0 || max_action_age_ms_ < 100 ||
        !std::isfinite(max_joint_speed_deg_s_) || max_joint_speed_deg_s_ <= 0.0 || max_joint_speed_deg_s_ > 360.0 ||
        !std::isfinite(max_gripper_speed_percent_s_) || max_gripper_speed_percent_s_ <= 0.0 ||
        max_gripper_speed_percent_s_ > 100.0)
      throw std::runtime_error("invalid baud/control/action-age or motion-speed parameters");
  }
  void open_log() {
    const auto path = std::filesystem::path(log_path_);
    std::filesystem::create_directories(path.parent_path());
    if (std::filesystem::exists(path) && std::filesystem::file_size(path) > 5 * 1024 * 1024) {
      std::error_code ec;
      std::filesystem::remove(path.string() + ".1", ec);
      std::filesystem::rename(path, path.string() + ".1", ec);
    }
    log_.open(path, std::ios::app);
    if (!log_) throw std::runtime_error("cannot open log file " + path.string());
    log_thread_ = std::thread([this] { log_worker(); });
    log_line("INFO", "file logging path=" + path.string() + " rotation=5MiB/one backup");
  }
  void log_line(const std::string &level, const std::string &message) {
    {
      std::lock_guard<std::mutex> guard(log_queue_mutex_);
      if (stop_log_thread_) return;
      log_queue_.push(LogEntry{level, message, std::hash<std::thread::id>{}(std::this_thread::get_id())});
    }
    log_cv_.notify_one();
  }
  void operator_line(const std::string &level, const std::string &message) {
    log_line(level, "OPERATOR: " + message);
  }
  void log_worker() {
    for (;;) {
      LogEntry item;
      {
        std::unique_lock<std::mutex> guard(log_queue_mutex_);
        log_cv_.wait(guard, [this] { return stop_log_thread_ || !log_queue_.empty(); });
        if (log_queue_.empty() && stop_log_thread_) break;
        item = std::move(log_queue_.front());
        log_queue_.pop();
      }
      const auto now = std::chrono::system_clock::now();
      const auto t = std::chrono::system_clock::to_time_t(now);
      std::tm tm{};
      localtime_r(&t, &tm);
      std::ostringstream line;
      line << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S%z") << " [" << item.level << "] [follower] [pid="
           << ::getpid() << " event_thread=" << item.source_thread << " writer_thread="
           << std::this_thread::get_id() << "] " << item.message;
      if (item.message.rfind("OPERATOR: ", 0) == 0) {
        const std::string concise = item.message.substr(10);
        if (item.level == "ERROR") RCLCPP_ERROR(get_logger(), "%s", concise.c_str());
        else if (item.level == "WARN") RCLCPP_WARN(get_logger(), "%s", concise.c_str());
        else RCLCPP_INFO(get_logger(), "%s", concise.c_str());
      }
      if (log_.is_open()) {
        log_ << line.str() << '\n';
        if (item.level != "DEBUG") log_.flush();
        const auto position = log_.tellp();
        if (position > static_cast<std::streampos>(5 * 1024 * 1024)) {
          log_.flush(); log_.close();
          std::error_code ec;
          std::filesystem::remove(log_path_ + ".1", ec);
          std::filesystem::rename(log_path_, log_path_ + ".1", ec);
          log_.open(log_path_, std::ios::trunc);
        }
      }
    }
    if (log_.is_open()) log_.flush();
  }
  void motor_log(const std::string &message, bool debug) {
    if (!debug || debug_logging_) log_line(debug ? "DEBUG" : "INFO", message);
  }
  void load_calibration() {
    calibration_file_ = so101_lerobot_follower::require_follower_calibration_filename(calibration_file_);
    calibration_ = so101_lerobot_follower::load_follower_calibration(calibration_file_, kNames, kIds);
    log_line("INFO", "loaded follower calibration id=follower path=" + calibration_file_ +
                      " (read-only; no EEPROM writes)");
    for (size_t i = 0; i < kNames.size(); ++i) {
      const auto &c = calibration_[i];
      log_line("INFO", "joint=" + std::string(kNames[i]) + " id=" + std::to_string(c.id) +
        " model=STS3215 drive_mode=" + std::to_string(c.drive_mode) + " homing_offset=" +
        std::to_string(c.homing_offset) + " offset-adjusted limits=[" + std::to_string(c.range_min) +
        "," + std::to_string(c.range_max) + "] counts");
    }
  }
  void inspect_motor_configuration() {
    calibration_valid_ = false;
    position_mode_ready_ = false;
    torque_ready_ = false;
    bool calibration_ok = true;
    bool position_mode_ok = true;
    bool torque_all_enabled = true;
    for (size_t i = 0; i < kNames.size(); ++i) {
      const int id = kIds[i];
      const int actual_min = bus_->read_register(id, 9, 2);
      const int actual_max = bus_->read_register(id, 11, 2);
      const int actual_offset = bus_->read_signed_register(id, 31, 2, 11);
      const int mode = bus_->read_register(id, 33, 1);
      const int torque = bus_->read_register(id, 40, 1);
      {
        std::lock_guard<std::mutex> guard(torque_status_mutex_);
        torque_states_[i] = torque;
      }
      const auto &expected = calibration_[i];
      const bool match = so101_lerobot_follower::calibration_matches_registers(
        expected, actual_min, actual_max, actual_offset);
      if (!match) calibration_ok = false;
      if (mode != 0) position_mode_ok = false;
      if (torque != 1) torque_all_enabled = false;
      log_line(match ? "INFO" : "ERROR", "motor configuration read-only id=" + std::to_string(id) +
        " joint=" + kNames[i] + " EEPROM_limits=[" + std::to_string(actual_min) + "," +
        std::to_string(actual_max) + "] EEPROM_homing_offset=" + std::to_string(actual_offset) +
        " calibration_file_limits=[" + std::to_string(expected.range_min) + "," +
        std::to_string(expected.range_max) + "] calibration_file_homing_offset=" +
        std::to_string(expected.homing_offset) + " exact_LeRobot_calibration_match=" + (match ? "true" : "false") +
        " Operating_Mode=" + std::to_string(mode) + " Torque_Enable=" + std::to_string(torque));
    }
    calibration_valid_ = calibration_ok;
    position_mode_ready_ = position_mode_ok;
    torque_ready_ = torque_all_enabled && !torque_transition_fault_.load();
    if (!position_mode_ready_) log_line("ERROR", "one or more motors are not in existing position mode; no mode writes will be attempted");
    if (torque_transition_fault_.load()) torque_ready_ = false;
    if (!torque_ready_) log_line("WARN", "one or more follower motors have torque disabled; motors remain disarmed until explicit GUI Start");
  }
  static std::string joint_list() {
    std::ostringstream s;
    for (size_t i = 0; i < kNames.size(); ++i) s << (i ? "," : "") << kNames[i];
    return s.str();
  }
  double feedback_value(size_t i, int count) const {
    const auto &c = calibration_[i];
    if (i == 5) return 100.0 * static_cast<double>(std::clamp(count, c.range_min, c.range_max) - c.range_min) /
                           static_cast<double>(c.range_max - c.range_min);
    const double mid = (static_cast<double>(c.range_min) + static_cast<double>(c.range_max)) / 2.0;
    return (static_cast<double>(count) - mid) * 360.0 / 4095.0;
  }
  std::array<int, 6> map_action(const Action &msg) {
    std::array<int, 6> goals{};
    for (size_t i = 0; i < kNames.size(); ++i) {
      if (msg.joint_names[i] != kNames[i])
        throw std::runtime_error("unexpected joint at index " + std::to_string(i) + ": expected " +
                                 kNames[i] + ", got " + msg.joint_names[i]);
      const double value = msg.values[i];
      if (!std::isfinite(value)) throw std::runtime_error("non-finite action for " + std::string(kNames[i]));
      const auto &c = calibration_[i];
      if (i == 5 && (value < 0.0 || value > 100.0))
        throw std::runtime_error("gripper action outside [0,100] percent: " + std::to_string(value));
      // LeRobot 0.6.2 DEGREES unnormalization: int(degrees * 4095 / 360 + midpoint).
      // The follower-specific calibration midpoint and limits are used independently.
      const int truncated = i == 5
        ? so101_lerobot_follower::percent_to_follower_count(value, c.range_min, c.range_max)
        : so101_lerobot_follower::degrees_to_follower_count(value, c.range_min, c.range_max);
      goals[i] = truncated;
      if (debug_logging_) log_line("DEBUG", "action joint=" + std::string(kNames[i]) +
        " source=" + std::to_string(value) + (i == 5 ? " percent" : " deg") +
        " converted_offset_adjusted_count=" + std::to_string(truncated) + " clamped_goal=" +
        std::to_string(goals[i]) + " follower_limits=[" + std::to_string(c.range_min) + "," +
        std::to_string(c.range_max) + "]");
    }
    return goals;
  }
  std::string action_range_violation(const Action &msg, size_t *violating_joint = nullptr) const {
    for (size_t i = 0; i < kNames.size(); ++i) {
      if (msg.joint_names[i] != kNames[i] || !std::isfinite(msg.values[i])) continue;
      const auto &c = calibration_[i];
      const auto violation = so101_lerobot_follower::follower_action_range_violation(
        msg.values[i], i, kNames[i], c.range_min, c.range_max);
      if (!violation.empty()) {
        if (violating_joint) *violating_joint = i;
        return violation;
      }
    }
    return {};
  }
  void on_session(bool enabled) {
    const int64_t receipt = steady_now_ns();
    session_state_seen_ = true;
    const bool rising_edge = session_state_.update(enabled, receipt);
    if (!enabled) {
      {
        std::lock_guard<std::mutex> gate(command_gate_mutex_);
        disarm_session_locked("leader published teleop_enabled=false", "on_session(false) leader Bool callback");
      }
      return;
    }
    if (!rising_edge) return;  // Heartbeats renew the session; they never initiate arming.
    const bool follower_preflight = hardware_ready_.load() && calibration_valid_.load() &&
                                    position_mode_ready_.load() && feedback_valid_.load() &&
                                    !session_fault_latched_.load() && !torque_transition_fault_.load();
    if (!so101_lerobot_follower::torque_enable_authorized(
          rising_edge, leader_ready_fresh(receipt), follower_preflight)) {
      {
        std::lock_guard<std::mutex> gate(command_gate_mutex_);
        disarm_session_locked("explicit Start rejected by preflight", "on_session(true) leader Bool callback");
      }
      log_line("ERROR", "explicit Start rejected: leader diagnostics are stale/unhealthy or follower preflight is incomplete");
      return;
    }
    {
      std::lock_guard<std::mutex> gate(command_gate_mutex_);
      session_started_ros_ns_.store(now().nanoseconds());
      received_sequence_ = false;
      ++session_generation_;
      clear_motion_target_locked("new leader session started");
      active_session_ = true;
      log_line("INFO", "SESSION_REARM source=on_session(true) leader false-to-true Bool edge previous=false new=true");
    }
    {
      std::lock_guard<std::mutex> guard(torque_worker_mutex_);
      torque_check_pending_ = true;
    }
    torque_worker_cv_.notify_one();
    log_line("INFO", "explicit Start accepted with fresh leader/follower preflight; torque arming requested");
  }

  void on_leader_diagnostics(const diagnostic_msgs::msg::DiagnosticArray &msg) {
    using so101_lerobot_follower::LeaderDiagnosticState;
    LeaderDiagnosticState state = LeaderDiagnosticState::kUnknown;
    for (const auto &status : msg.status) {
      const auto candidate = so101_lerobot_follower::classify_leader_diagnostic(
        status.name, status.level, status.message);
      if (candidate == LeaderDiagnosticState::kFatal) { state = candidate; break; }
      if (candidate == LeaderDiagnosticState::kRecoverableFeedbackLoss) state = candidate;
      else if (candidate == LeaderDiagnosticState::kHealthy && state == LeaderDiagnosticState::kUnknown)
        state = candidate;
    }
    leader_diagnostics_receipt_ns_ = steady_now_ns();
    if (state == LeaderDiagnosticState::kFatal) {
      driver_fault_fatal_ = true;
      leader_diagnostics_healthy_ = false;
      leader_feedback_recoverable_ = false;
      std::lock_guard<std::mutex> gate(command_gate_mutex_);
      disarm_session_locked("fatal leader driver diagnostic", "on_leader_diagnostics");
      return;
    }
    if (state == LeaderDiagnosticState::kRecoverableFeedbackLoss) {
      leader_diagnostics_healthy_ = false;
      const bool preserve_active_session =
        !driver_fault_fatal_.load() &&
        so101_lerobot_follower::active_session_survives_leader_diagnostic(
          active_session_.load(), state);
      leader_feedback_recoverable_ = preserve_active_session;
      if (preserve_active_session) {
        std::lock_guard<std::mutex> gate(command_gate_mutex_);
        clear_motion_target_locked("recoverable leader feedback interruption");
      }
      return;
    }
    if (state == LeaderDiagnosticState::kHealthy) {
      leader_diagnostics_healthy_ = true;
      leader_feedback_recoverable_ = false;
      if (!active_session_) driver_fault_fatal_ = false;
      return;
    }
    leader_diagnostics_healthy_ = false;
    leader_feedback_recoverable_ = false;
  }

  bool leader_ready_fresh(int64_t now_ns) const {
    const int64_t received = leader_diagnostics_receipt_ns_.load();
    return so101_lerobot_follower::leader_diagnostics_fresh(
      leader_diagnostics_healthy_.load(), received, now_ns,
      so101_lerobot_follower::LeaderSessionState::kFreshnessNs);
  }

  void enforce_session_freshness() {
    std::lock_guard<std::mutex> gate(command_gate_mutex_);
    if (active_session_ && !session_state_.fresh_enabled(steady_now_ns())) {
      disarm_session_locked("leader session heartbeat stale", "enforce_session_freshness");
    }
    if (active_session_ && !leader_feedback_recoverable_ && !leader_ready_fresh(steady_now_ns())) {
      disarm_session_locked("leader readiness diagnostics stale or unhealthy", "enforce_session_freshness");
    }
  }

  void torque_enable_worker() {
    std::unique_lock<std::mutex> lock(torque_worker_mutex_);
    while (!stop_torque_worker_) {
      torque_worker_cv_.wait(lock, [this] { return stop_torque_worker_ || torque_check_pending_; });
      if (stop_torque_worker_) break;
      torque_check_pending_ = false;
      lock.unlock();
      try {
        maybe_enable_torque();
      } catch (const std::exception &error) {
        torque_transition_fault_ = true;
        torque_ready_ = false;
        hardware_ready_ = false;
        feedback_valid_ = false;
        const std::string failure = std::string("unexpected torque worker failure: ") + error.what();
        {
          std::lock_guard<std::mutex> error_guard(hardware_error_mutex_);
          last_hardware_error_ = failure;
        }
        {
          std::lock_guard<std::mutex> gate(command_gate_mutex_);
          disarm_session_locked(failure, "torque_enable_worker");
        }
        {
          std::lock_guard<std::mutex> bus_guard(bus_mutex_);
          if (bus_) bus_->close();
        }
        {
          std::lock_guard<std::mutex> status_guard(torque_status_mutex_);
          torque_enable_error_ = std::string("unexpected torque worker failure: ") + error.what();
        }
        log_line("ERROR", "TORQUE_ENABLE_FAILED unexpected worker exception; serial closed and readiness latched false: " +
                           std::string(error.what()));
      }
      lock.lock();
    }
  }

  static int64_t steady_now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  bool leader_enable_state_fresh() const {
    return !stop_torque_worker_ && active_session_.load() && session_state_.fresh_enabled(steady_now_ns()) &&
           leader_ready_fresh(steady_now_ns());
  }

  void maybe_enable_torque() {
    if (!leader_enable_state_fresh() || torque_ready_.load() || torque_transition_fault_.load()) return;
    if (!hardware_ready_.load() || !bus_) return;
    std::unique_lock<std::mutex> transition_lock(torque_transition_mutex_, std::try_to_lock);
    if (!transition_lock.owns_lock() || torque_ready_.load() || torque_transition_fault_.load()) return;
    std::lock_guard<std::mutex> bus_lock(bus_mutex_);
    if (!leader_enable_state_fresh() || !hardware_ready_.load() || !bus_) return;

    auto transition_logger = [this](const std::string &level, const std::string &message) {
      if (level == "ERROR") {
        std::lock_guard<std::mutex> guard(torque_status_mutex_);
        if (message == last_torque_issue_log_) return;
        last_torque_issue_log_ = message;
      }
      log_line(level, message);
    };
    const auto report = so101_lerobot_follower::enable_torque_safely(
      calibration_, kNames, kIds, *bus_, [this] { return leader_enable_state_fresh(); }, transition_logger);
    {
      std::lock_guard<std::mutex> guard(torque_status_mutex_);
      for (size_t i = 0; i < report.final_torque_state.size(); ++i)
        if (report.final_torque_state[i] >= 0) torque_states_[i] = report.final_torque_state[i];
    }
    if (report.success) {
      torque_ready_ = true;
      operator_line("INFO", "READY — Initial checks passed");
      operator_line("INFO", "STARTED — Teleoperation active");
      feedback_valid_ = true;
      {
        std::lock_guard<std::mutex> guard(torque_status_mutex_);
        torque_enable_error_.clear();
        last_torque_issue_log_.clear();
      }
      log_line("INFO", report.already_enabled
        ? "all six torque registers verified enabled; no torque writes were needed"
        : "TORQUE_ENABLE_SUCCESS: all six motors verified Torque_Enable=1; no Goal_Position writes were sent");
      return;
    }

    std::ostringstream details;
    for (size_t i = 0; i < report.issues.size(); ++i) {
      const auto &item = report.issues[i];
      if (i) details << "; ";
      details << item.joint << ' ' << item.register_name << " expected=" << item.expected
              << " actual=" << item.actual << " error=" << item.error;
    }
    const std::string failure = details.str().empty() ? "torque transition failed without a diagnostic detail" : details.str();
    torque_ready_ = false;
    if (report.write_attempted) {
      torque_transition_fault_ = true;
      {
        std::lock_guard<std::mutex> error_guard(hardware_error_mutex_);
        last_hardware_error_ = failure;
      }
      std::lock_guard<std::mutex> gate(command_gate_mutex_);
      disarm_session_locked("torque transition failed: " + failure, "maybe_enable_torque");
    }
    bool failure_changed = false;
    {
      std::lock_guard<std::mutex> guard(torque_status_mutex_);
      failure_changed = torque_enable_error_ != failure;
      torque_enable_error_ = failure;
    }
    if (failure_changed) {
      operator_line("WARN", "WARNING — Follower torque check failed");
      log_line("ERROR", "TORQUE_ENABLE_FAILED: " + failure +
        (report.write_attempted ? "; readiness latched false after torque-write attempt" :
                                  "; no torque writes were attempted"));
    }
    if (report.rollback_failed) {
      hardware_ready_ = false;
      feedback_valid_ = false;
      bus_->close();
      log_line("ERROR", "torque rollback could not be verified; serial closed, readiness blocked, teleoperation remains disabled");
    }
  }
  void on_action(const Action &msg) {
    last_received_sequence_ = msg.sequence;
    last_received_sequence_seen_ = true;
    if (!active_session_) { reject("teleoperation session is disabled", &msg); return; }
    const uint64_t observed_session = session_generation_.load();
    if (!hardware_ready_ || !bus_) { reject("follower hardware is not connected/ready", &msg); return; }
    if (!calibration_valid_) { reject("read-only motor calibration registers do not match follower calibration file", &msg); return; }
    if (!position_mode_ready_) { reject("one or more motors are not already in position mode", &msg); return; }
    if (!torque_ready_) { reject("torque is not verified enabled on all six motors; guarded startup transition incomplete", &msg); return; }
    if (!feedback_valid_) { reject("latest follower feedback is invalid or missing", &msg); return; }
    if (msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0) { reject("action has zero timestamp", &msg); return; }
    const rclcpp::Time stamp(msg.header.stamp);
    const int64_t stamp_ns = stamp.nanoseconds();
    if (!so101_lerobot_follower::action_is_newer_than_session(stamp_ns, session_started_ros_ns_.load())) {
      reject("action timestamp predates current teleoperation session", &msg); return;
    }
    const auto age = (now() - stamp).nanoseconds();
    if (age > static_cast<int64_t>(max_action_age_ms_) * 1000000LL) {
      reject("stale action age_ms=" + std::to_string(age / 1000000LL), &msg); return;
    }
    if (age < -5000000000LL) log_line("WARN", "leader timestamp is >5 seconds in the future; check laptop/Pi clock synchronization");
    size_t violating_joint = kNames.size();
    const auto range_violation = action_range_violation(msg, &violating_joint);
    if (!range_violation.empty()) {
      const auto reason = range_violation +
          "; action rejected, session remains enabled and movement paused until a fresh valid action";
      bool first_rejection;
      {
        std::lock_guard<std::mutex> guard(action_range_mutex_);
        first_rejection = !action_range_paused_;
        action_range_paused_ = true;
        action_range_reason_ = range_violation;
      }
      {
        std::lock_guard<std::mutex> gate(command_gate_mutex_);
        if (active_session_ && observed_session == session_generation_.load()) {
          if (first_rejection)
            clear_motion_target_locked(reason, &msg, violating_joint);
          else {
            motion_limiter_.clear();
            motion_target_active_ = false;
          }
        }
      }
      reject(reason, &msg, violating_joint);
      return;
    }
    std::array<int, 6> goals{};
    try {
      goals = map_action(msg);
    } catch (const std::exception &e) {
      reject(e.what(), &msg);
      return;
    }
    {
      std::lock_guard<std::mutex> gate(command_gate_mutex_);
      if (!active_session_ || observed_session != session_generation_.load()) {
        reject("teleoperation session stopped or changed before target update", &msg); return;
      }
      if (!so101_lerobot_follower::action_is_newer_than_session(stamp_ns, session_started_ros_ns_.load())) {
        reject("action timestamp predates current teleoperation session", &msg); return;
      }
      if (received_sequence_ && !so101_lerobot_follower::sequence_is_newer(msg.sequence, last_sequence_)) {
        reject("duplicate or out-of-order action sequence=" + std::to_string(msg.sequence), &msg); return;
      }
      motion_limiter_.set_target(goals);
      motion_target_active_ = true;
      last_target_action_ = msg;
      have_last_target_action_ = true;
      last_sequence_ = msg.sequence;
      received_sequence_ = true;
      last_action_steady_ns_ = steady_now_ns();
    }
    {
      std::lock_guard<std::mutex> guard(action_range_mutex_);
      if (action_range_paused_) {
        action_range_paused_ = false;
        action_range_reason_.clear();
        log_line("INFO", "fresh valid follower-range action received; movement resumed within the existing teleoperation session");
      }
    }
    ++actions_received_;
    if (debug_logging_) log_line("DEBUG", "action accepted sequence=" + std::to_string(msg.sequence) +
      " six joints; latest target replaced, speed-limited at control rate");
  }
  std::string action_diagnostic_context(const Action &msg, size_t joint_index) const {
    std::ostringstream context;
    if (joint_index < kNames.size()) {
      const auto &calibration = calibration_[joint_index];
      context << "joint=" << kNames[joint_index]
              << " received_value=" << msg.values[joint_index]
              << (joint_index == 5 ? " percent" : " deg")
              << " follower_bounds=[" << calibration.range_min << "," << calibration.range_max << "]";
    } else {
      context << "joint=all received_values={";
      for (size_t i = 0; i < kNames.size(); ++i) {
        if (i) context << "; ";
        const auto &calibration = calibration_[i];
        context << kNames[i] << ":" << msg.values[i] << (i == 5 ? "%" : "deg")
                << " bounds=[" << calibration.range_min << "," << calibration.range_max << "]";
      }
      context << "}";
    }
    return context.str();
  }
  void reject(const std::string &why, const Action *msg = nullptr,
              size_t joint_index = kNames.size()) {
    ++rejected_actions_;
    const auto now_steady = std::chrono::steady_clock::now();
    if (last_reject_log_.time_since_epoch().count() == 0 || now_steady - last_reject_log_ > 1s) {
      std::string detail = "command rejected reason=" + why +
          " session_active=" + (active_session_ ? "true" : "false") + " motor_write=false";
      if (msg) detail += " " + action_diagnostic_context(*msg, joint_index);
      log_line("WARN", detail);
      last_reject_log_ = now_steady;
    }
  }
  void clear_motion_target_locked(const std::string &reason, const Action *rejected = nullptr,
                                  size_t joint_index = kNames.size(),
                                  const char *motor_write = "false") {
    const bool had_target = motion_target_active_;
    if (had_target || rejected) {
      std::string detail = "motion target cleared reason=" + reason +
          " target_present=" + (had_target ? "true" : "false") +
          " session_active=" + (active_session_ ? "true" : "false") +
          " motor_write=" + motor_write;
      if (rejected) detail += " " + action_diagnostic_context(*rejected, joint_index);
      else if (have_last_target_action_)
        detail += " " + action_diagnostic_context(last_target_action_, kNames.size());
      log_line("WARN", detail);
    }
    motion_limiter_.clear();
    motion_target_active_ = false;
  }
  void read_cycle() {
    if (!hardware_ready_ || !bus_) return;
    const auto start = std::chrono::steady_clock::now();
    std::array<int, 6> counts{};
    std::string failure;
    bool success = false;
    for (int attempt = 0; attempt <= 2; ++attempt) {
      try {
        std::lock_guard<std::mutex> bus_lock(bus_mutex_);
        if (!hardware_ready_ || !bus_) return;
        counts = bus_->read_positions();
        success = true;
        if (attempt > 0) log_line("WARN", "feedback read recovered on immediate retry " + std::to_string(attempt));
        break;
      } catch (const std::exception &e) {
        failure = e.what();
        if (attempt < 2 && debug_logging_)
          log_line("DEBUG", "feedback retry=" + std::to_string(attempt + 1) + " reason=" + failure);
      }
    }
    if (success) {
      feedback_valid_ = true;
      ++successful_reads_;
      if (calibration_valid_) publish_feedback(counts);
      const auto now_steady = std::chrono::steady_clock::now();
      const double elapsed_seconds = last_motion_update_.time_since_epoch().count() == 0
          ? 1.0 / control_rate_hz_
          : std::chrono::duration<double>(now_steady - last_motion_update_).count();
      last_motion_update_ = now_steady;
      try {
        std::lock_guard<std::mutex> gate(command_gate_mutex_);
        if (active_session_ && hardware_ready_ && bus_) {
          const int64_t last_action = last_action_steady_ns_.load();
          const bool action_fresh = last_action > 0 &&
              steady_now_ns() - last_action <= static_cast<int64_t>(max_action_age_ms_) * 1000000LL;
          if (!action_fresh) {
            clear_motion_target_locked("accepted leader action exceeded max_action_age_ms");
          } else {
            const auto shaped = motion_limiter_.advance(counts, calibration_, elapsed_seconds,
                max_joint_speed_deg_s_, max_gripper_speed_percent_s_);
            if (shaped) {
            std::lock_guard<std::mutex> bus_lock(bus_mutex_);
            if (active_session_ && hardware_ready_ && bus_) {
              bus_->write_goals(*shaped);
              ++successful_writes_;
            }
            }
          }
        }
      } catch (const std::exception &e) {
        std::lock_guard<std::mutex> gate(command_gate_mutex_);
        hardware_ready_ = false;
        feedback_valid_ = false;
        session_fault_latched_ = true;
        ++session_generation_;
        {
          std::lock_guard<std::mutex> error_guard(hardware_error_mutex_);
          last_hardware_error_ = e.what();
        }
        disarm_session_locked("speed-limited motor write or control update failed",
                              "read_cycle Goal_Position write/control update");
        std::lock_guard<std::mutex> bus_lock(bus_mutex_);
        if (bus_) bus_->close();
        log_line("ERROR", std::string("speed-limited Goal_Position sync write failed; serial released and reconnect scheduled: ") + e.what());
        return;
      }
      if (debug_logging_) {
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        log_line("DEBUG", "feedback read/conversion/publish ms=" + std::to_string(ms) +
                          " counts=" + FeetechBus::format_counts(counts));
      }
      return;
    }
    feedback_valid_ = false;
    ++read_errors_;
    bool read_error_changed = false;
    {
      std::lock_guard<std::mutex> gate(command_gate_mutex_);
      hardware_ready_ = false;
      session_fault_latched_ = true;
      ++session_generation_;
      {
        std::lock_guard<std::mutex> error_guard(hardware_error_mutex_);
        read_error_changed = last_hardware_error_ != failure;
        last_hardware_error_ = failure;
      }
      disarm_session_locked("follower feedback read failed", "read_cycle feedback read");
      std::lock_guard<std::mutex> bus_lock(bus_mutex_);
      if (bus_) bus_->close();
    }
    if (read_error_changed || std::chrono::steady_clock::now() - last_error_log_ > 1s) {
      log_line("ERROR", "feedback failed after two immediate retries; no feedback published, serial released, and new motor writes blocked: " + failure);
      last_error_log_ = std::chrono::steady_clock::now();
    }
  }
  void try_reconnect() {
    std::lock_guard<std::mutex> bus_lock(bus_mutex_);
    if (hardware_ready_ || !bus_) return;
    try {
      log_line("INFO", "retrying follower serial connection");
      bus_->open();
      inspect_motor_configuration();
      auto counts = bus_->read_positions();
      feedback_valid_ = true;
      hardware_ready_ = true;
      if (calibration_valid_) publish_feedback(counts);
      log_line("INFO", "follower hardware reconnected; six-motor feedback validated");
    } catch (const std::exception &e) {
      feedback_valid_ = false;
      bus_->close();
      const std::string reason(e.what());
      bool changed = false;
      {
        std::lock_guard<std::mutex> error_guard(hardware_error_mutex_);
        changed = reason != last_hardware_error_;
        last_hardware_error_ = reason;
      }
      if (changed) log_line("ERROR", "serial reconnect attempt failed: " + reason);
    }
  }
  void publish_feedback(const std::array<int, 6> &counts) {
    Action msg;
    msg.header.stamp = now();
    msg.sequence = feedback_sequence_++;
    for (size_t i = 0; i < kNames.size(); ++i) {
      msg.joint_names[i] = kNames[i];
      msg.values[i] = feedback_value(i, counts[i]);
    }
    feedback_pub_->publish(msg);
  }
  void publish_diagnostics() {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "/so101/follower";
    status.hardware_id = serial_port_;
    const int64_t action_ns = last_action_steady_ns_.load();
    const int64_t action_age_ms = action_ns == 0 ? -1 : (steady_now_ns() - action_ns) / 1000000LL;
    const int64_t leader_state_age_ms = session_state_.last_state_age_ms(steady_now_ns());
    const bool armable = hardware_ready_ && feedback_valid_ && calibration_valid_ && position_mode_ready_ &&
                         !session_fault_latched_ && !torque_transition_fault_;
    const bool ready = armable && torque_ready_;
    bool range_paused;
    std::string range_reason;
    {
      std::lock_guard<std::mutex> guard(action_range_mutex_);
      range_paused = action_range_paused_;
      range_reason = action_range_reason_;
    }
    std::string torque_error;
    std::array<int, 6> torque_states{};
    {
      std::lock_guard<std::mutex> guard(torque_status_mutex_);
      torque_error = torque_enable_error_;
      torque_states = torque_states_;
    }
    status.level = ready ? (range_paused ? diagnostic_msgs::msg::DiagnosticStatus::WARN : diagnostic_msgs::msg::DiagnosticStatus::OK) :
                   armable ? diagnostic_msgs::msg::DiagnosticStatus::WARN : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = session_fault_latched_ ? "session safety/communication fault; fresh disabled heartbeats and explicit restart required" :
      !hardware_ready_ ? "hardware not connected" : !calibration_valid_ ? "EEPROM calibration does not match follower calibration file" :
      !position_mode_ready_ ? "motors are not already in position mode" : !torque_ready_ ?
      (torque_error.empty() ? "torque is not verified enabled on all six motors" : "torque transition failed: " + torque_error) :
      !feedback_valid_ ? "motor feedback invalid" : range_paused ? "action out of follower calibrated range; session remains enabled and movement is paused" :
      active_session_ ? "teleoperation active" :
      "ready; teleoperation stopped";
    add_value(status, "ready", ready ? "true" : "false");
    add_value(status, "armable", armable ? "true" : "false");
    add_value(status, "calibration_valid", calibration_valid_ ? "true" : "false");
    add_value(status, "position_mode_ready", position_mode_ready_ ? "true" : "false");
    add_value(status, "torque_enabled_all", torque_ready_ ? "true" : "false");
    add_value(status, "leader_ready_fresh", leader_ready_fresh(steady_now_ns()) ? "true" : "false");
    add_value(status, "torque_enable_error", torque_error);
    for (size_t i = 0; i < kNames.size(); ++i)
      add_value(status, std::string("torque_enabled_") + kNames[i],
        torque_states[i] == 1 ? "true" : torque_states[i] == 0 ? "false" : "unknown");
    add_value(status, "feedback_valid", feedback_valid_ ? "true" : "false");
    add_value(status, "teleop_enabled", active_session_ ? "true" : "false");
    add_value(status, "leader_session_state_age_ms", std::to_string(leader_state_age_ms));
    add_value(status, "session_fault_latched", session_fault_latched_ ? "true" : "false");
    add_value(status, "action_range_paused", range_paused ? "true" : "false");
    add_value(status, "action_range_pause_reason", range_reason);
    add_value(status, "last_action_age_ms", std::to_string(action_age_ms));
    add_value(status, "last_sequence", received_sequence_ ? std::to_string(last_sequence_) : "none");
    add_value(status, "actions_received", std::to_string(actions_received_));
    add_value(status, "actions_rejected", std::to_string(rejected_actions_));
    add_value(status, "successful_feedback_reads", std::to_string(successful_reads_));
    add_value(status, "feedback_read_errors", std::to_string(read_errors_));
    add_value(status, "serial", serial_port_);
    add_value(status, "calibration_file", calibration_file_);
    add_value(status, "maximum_single_action_age_ms", std::to_string(max_action_age_ms_));
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }
  static void add_value(diagnostic_msgs::msg::DiagnosticStatus &status, const std::string &key, const std::string &value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key; kv.value = value;
    status.values.push_back(kv);
  }

  std::string serial_port_, calibration_file_, log_path_;
  int baud_rate_{1000000}, max_action_age_ms_{500};
  double control_rate_hz_{50.0};
  double max_joint_speed_deg_s_{20.0}, max_gripper_speed_percent_s_{30.0};
  std::atomic<bool> debug_logging_{false};
  std::atomic<bool> hardware_ready_{false}, feedback_valid_{false}, active_session_{false};
  std::atomic<bool> calibration_valid_{false}, position_mode_ready_{false}, torque_ready_{false};
  std::atomic<bool> torque_transition_fault_{false}, torque_fault_session_log_{false}, session_state_seen_{false};
  std::atomic<bool> session_fault_latched_{false}, session_fault_log_{false};
  bool action_range_paused_{false};
  std::string action_range_reason_;
  so101_lerobot_follower::LeaderSessionState session_state_;
  so101_lerobot_follower::CalibrationSet calibration_{};
  so101_lerobot_follower::JointMotionLimiter motion_limiter_;
  Action last_target_action_{};
  bool have_last_target_action_{false};
  bool motion_target_active_{false};  // guarded by command_gate_mutex_
  std::chrono::steady_clock::time_point last_motion_update_{};
  std::unique_ptr<FeetechBus> bus_;
  std::mutex bus_mutex_, torque_transition_mutex_, torque_status_mutex_, action_range_mutex_, hardware_error_mutex_;
  std::mutex command_gate_mutex_;
  std::mutex torque_worker_mutex_;
  std::condition_variable torque_worker_cv_;
  std::atomic<bool> stop_torque_worker_{false};
  bool torque_check_pending_{false};
  std::thread torque_worker_;
  std::array<int, 6> torque_states_{{-1, -1, -1, -1, -1, -1}};
  std::string torque_enable_error_{"waiting for explicit Start and fresh healthy leader diagnostics"};
  std::string last_torque_issue_log_;
  std::ofstream log_;
  std::mutex log_mutex_;
  std::mutex log_queue_mutex_;
  std::condition_variable log_cv_;
  std::queue<LogEntry> log_queue_;
  std::thread log_thread_;
  bool stop_log_thread_{false};
  uint32_t feedback_sequence_{0}, last_sequence_{0};
  std::atomic<uint32_t> last_received_sequence_{0};
  std::atomic<bool> last_received_sequence_seen_{false};
  bool received_sequence_{false};
  uint64_t actions_received_{0}, rejected_actions_{0};
  std::atomic<uint64_t> successful_reads_{0}, read_errors_{0}, successful_writes_{0};
  std::atomic<int64_t> last_action_steady_ns_{0}, session_started_ros_ns_{0};
  std::atomic<uint64_t> session_generation_{0};
  std::chrono::steady_clock::time_point last_error_log_{}, last_reject_log_{};
  std::string last_hardware_error_;
  rclcpp::Publisher<Action>::SharedPtr feedback_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr session_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr leader_diag_sub_;
  rclcpp::CallbackGroup::SharedPtr session_callback_group_;
  rclcpp::Subscription<Action>::SharedPtr action_sub_;
  std::atomic<bool> leader_diagnostics_healthy_{false};
  std::atomic<bool> leader_feedback_recoverable_{false}, driver_fault_fatal_{false};
  std::atomic<int64_t> leader_diagnostics_receipt_ns_{0};
  rclcpp::TimerBase::SharedPtr feedback_timer_, status_timer_, reconnect_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};
}  // namespace

int main(int argc, char **argv) {
  if (std::getenv("ROS_DOMAIN_ID") == nullptr) ::setenv("ROS_DOMAIN_ID", "42", 0);
  bool sync_calibration = false;
  int output_argc = 1;
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--sync-calibration") {
      sync_calibration = true;
      continue;
    }
    argv[output_argc++] = argv[i];
  }
  argc = output_argc;
  argv[argc] = nullptr;
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    if (sync_calibration) {
      auto node = std::make_shared<CalibrationSyncMonitor>();
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(node);
      std::thread spin_thread([&executor] { executor.spin(); });
      try {
        result = run_calibration_sync(node);
      } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "CALIBRATION_SYNC_FAILED validation/startup error: %s", e.what());
        result = 2;
      }
      executor.cancel();
      if (spin_thread.joinable()) spin_thread.join();
      executor.remove_node(node);
      node.reset();
      rclcpp::shutdown();
      return result;
    }
    auto node = std::make_shared<FollowerNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
    node.reset();
  } catch (const std::exception &e) {
    std::fprintf(stderr, "[so101_lerobot_follower] fatal startup error: %s\n", e.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
