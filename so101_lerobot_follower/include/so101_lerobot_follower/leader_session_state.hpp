#pragma once

#include <cstdint>
#include <mutex>
#include <string>

namespace so101_lerobot_follower {

inline bool action_is_newer_than_session(int64_t action_ros_ns, int64_t session_start_ros_ns) {
  return action_ros_ns > session_start_ros_ns;
}

inline bool sequence_is_newer(uint32_t candidate, uint32_t previous) {
  return static_cast<int32_t>(candidate - previous) > 0;
}

inline bool torque_enable_authorized(bool explicit_enable_edge, bool leader_ready_fresh,
                                    bool follower_preflight_valid) {
  return explicit_enable_edge && leader_ready_fresh && follower_preflight_valid;
}

enum class LeaderDiagnosticState { kHealthy, kRecoverableFeedbackLoss, kFatal, kUnknown };

inline LeaderDiagnosticState classify_leader_diagnostic(const std::string &name, uint8_t level,
                                                        const std::string &message) {
  if (name != "feetech_read_only_state") return LeaderDiagnosticState::kUnknown;
  if (level >= 2) return LeaderDiagnosticState::kFatal;
  if (level == 1 && message.compare(0, 33, "RECOVERABLE_LEADER_FEEDBACK_LOSS:") == 0)
    return LeaderDiagnosticState::kRecoverableFeedbackLoss;
  if (level == 0 && (message == "read-only feedback healthy" ||
                     message == "connected; all six motors verified read-only"))
    return LeaderDiagnosticState::kHealthy;
  constexpr uint8_t kWarn = 1;
  constexpr char kRangePrefix[] = "feedback published; ";
  constexpr char kClampSuffix[] =
      "; leader-to-follower mapping clamps targets to follower calibration limits";
  if (level == kWarn && message.compare(0, sizeof(kRangePrefix) - 1, kRangePrefix) == 0 &&
      message.size() > (sizeof(kRangePrefix) - 1) + (sizeof(kClampSuffix) - 1) &&
      message.find(" outside [") != std::string::npos &&
      message.compare(message.size() - (sizeof(kClampSuffix) - 1), sizeof(kClampSuffix) - 1,
                      kClampSuffix) == 0) return LeaderDiagnosticState::kHealthy;
  return LeaderDiagnosticState::kUnknown;
}

inline bool leader_feedback_diagnostic_healthy(const std::string &name, uint8_t level,
                                               const std::string &message) {
  return classify_leader_diagnostic(name, level, message) == LeaderDiagnosticState::kHealthy;
}

inline bool active_session_survives_leader_diagnostic(bool active_session,
                                                       LeaderDiagnosticState state) {
  return active_session && state == LeaderDiagnosticState::kRecoverableFeedbackLoss;
}

inline bool leader_diagnostics_fresh(bool healthy, int64_t receipt_steady_ns,
                                     int64_t now_steady_ns, int64_t freshness_ns) {
  return healthy && receipt_steady_ns > 0 && now_steady_ns >= receipt_steady_ns &&
         now_steady_ns - receipt_steady_ns <= freshness_ns;
}

// Session heartbeats are state, not authorization. Only a false-to-true edge
// can request arming; the caller additionally requires fresh leader diagnostics.
class LeaderSessionState {
 public:
  // Torque may only be requested by a deliberate rising edge. Repeated false
  // heartbeats (including retained startup state) can never request it.
  bool update(bool enabled, int64_t receipt_steady_ns) {
    std::lock_guard<std::mutex> guard(mutex_);
    const bool rising_edge = enabled && !enabled_ && have_seen_disabled_;
    if (!enabled) have_seen_disabled_ = true;
    enabled_ = enabled;
    last_receipt_ns_ = receipt_steady_ns;
    return rising_edge;
  }

  bool fresh_enabled(int64_t now_steady_ns) const {
    std::lock_guard<std::mutex> guard(mutex_);
    const int64_t age = now_steady_ns - last_receipt_ns_;
    return enabled_ && age >= 0 && age <= kFreshnessNs;
  }

  bool enabled() const {
    std::lock_guard<std::mutex> guard(mutex_);
    return enabled_;
  }

  int64_t last_state_age_ms(int64_t now_steady_ns) const {
    std::lock_guard<std::mutex> guard(mutex_);
    const int64_t age = now_steady_ns - last_receipt_ns_;
    return last_receipt_ns_ == 0 || age < 0 ? -1 : age / 1000000LL;
  }

  static constexpr int64_t kFreshnessNs = 1500000000LL;

 private:
  mutable std::mutex mutex_;
  bool enabled_{false};
  bool have_seen_disabled_{false};
  int64_t last_receipt_ns_{0};
};

}  // namespace so101_lerobot_follower
