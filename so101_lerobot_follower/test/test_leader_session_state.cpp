#include <iostream>
#include <string>

#include "so101_lerobot_follower/leader_session_state.hpp"

int main() {
  int failures = 0;
  const auto check = [&failures](bool ok, const std::string &name) {
    if (!ok) { std::cerr << "FAIL: " << name << '\n'; ++failures; }
  };

  so101_lerobot_follower::LeaderSessionState state;
  check(!state.fresh_enabled(1000000000LL), "missing session is not fresh enabled");
  check(!state.update(true, 1000000000LL), "retained or startup true before fresh disabled state is ignored");
  check(!so101_lerobot_follower::torque_enable_authorized(false, true, true),
        "startup true cannot authorize torque before the disabled baseline");
  for (int i = 1; i <= 100; ++i) {
    const bool edge = state.update(false, i * 500000000LL);
    check(!edge, "disabled heartbeat cannot request torque enable");
    check(!so101_lerobot_follower::torque_enable_authorized(edge, true, true),
          "even repeated disabled heartbeats are not torque authorization");
  }

  check(state.update(true, 51000000000LL), "explicit false-to-true Start after disabled baseline is an enable edge");
  check(state.fresh_enabled(51000000000LL), "explicit Start state is fresh");
  check(so101_lerobot_follower::torque_enable_authorized(true, true, true),
        "explicit Start with fresh leader and follower preflight authorizes arming");
  check(!so101_lerobot_follower::torque_enable_authorized(true, false, true),
        "stale or unhealthy leader diagnostics block arming");
  check(!so101_lerobot_follower::torque_enable_authorized(true, true, false),
        "failed follower preflight blocks arming");
  check(!state.update(true, 51500000000LL), "true heartbeat renews state but is not a new enable edge");
  check(!state.fresh_enabled(53000000001LL), "expired true heartbeat is stale");
  check(!state.update(true, 53000000002LL), "fresh true heartbeat after stale disarm does not rearm");
  check(!state.update(true, 53500000000LL), "repeated true heartbeat after stale disarm stays disarmed");
  check(!state.update(false, 53500000000LL), "explicit Stop disarms immediately");
  check(state.update(true, 54000000000LL), "a later deliberate Start creates a new edge");

  const auto leader_status_healthy = so101_lerobot_follower::leader_feedback_diagnostic_healthy;
  check(leader_status_healthy("feetech_read_only_state", 0, "read-only feedback healthy"),
        "in-range feedback diagnostic is healthy");
  check(leader_status_healthy("feetech_read_only_state", 1,
        "feedback published; wrist_flex=874 outside [914,3236]; leader-to-follower mapping clamps targets to follower calibration limits"),
        "decoded out-of-range feedback warning remains healthy for session readiness");
  check(leader_status_healthy("feetech_read_only_state", 0, "read-only feedback healthy"),
        "return to in-range feedback restores the normal healthy diagnostic");
  check(!leader_status_healthy("feetech_read_only_state", 1, "feedback read warning"),
        "unrelated warning is not accepted as healthy");
  check(!leader_status_healthy("feetech_read_only_state", 1,
        "feedback published; unclassified warning; leader-to-follower mapping clamps targets to follower calibration limits"),
        "range-shaped message without an explicit calibration violation is not accepted");
  check(!leader_status_healthy("feetech_read_only_state", 2, "leader feedback failed; reconnect will be attempted"),
        "feedback communication error remains unhealthy");
  check(!leader_status_healthy("other_status", 0, "read-only feedback healthy"),
        "wrong diagnostic status name is not accepted");
  check(!leader_status_healthy("feetech_read_only_state", 0, "unknown healthy-looking message"),
        "unknown status message is not accepted");

  using so101_lerobot_follower::LeaderDiagnosticState;
  const auto classify = so101_lerobot_follower::classify_leader_diagnostic;
  const std::string recovery_message = "RECOVERABLE_LEADER_FEEDBACK_LOSS: read timeout; reconnect retry scheduled";
  const std::string fatal_message = "FATAL_LEADER_DRIVER_FAULT: serial port closed; restart required";
  check(classify("feetech_read_only_state", 1, recovery_message) ==
            LeaderDiagnosticState::kRecoverableFeedbackLoss,
        "only the explicit recoverable diagnostic is classified as a feedback pause");
  check(classify("feetech_read_only_state", 2, fatal_message) == LeaderDiagnosticState::kFatal,
        "explicit closed-port diagnostic is fatal");
  check(classify("feetech_read_only_state", 2, recovery_message) == LeaderDiagnosticState::kFatal,
        "recoverable wording at ERROR level is treated as fatal, never recoverable");
  check(classify("feetech_read_only_state", 2, "unspecified driver error") == LeaderDiagnosticState::kFatal,
        "unclassified driver errors fail closed");
  check(classify("feetech_read_only_state", 1, "feedback read warning") == LeaderDiagnosticState::kUnknown,
        "arbitrary warning is not accepted as recoverable");
  check(!so101_lerobot_follower::active_session_survives_leader_diagnostic(false,
            LeaderDiagnosticState::kRecoverableFeedbackLoss),
        "recoverable diagnostics cannot authorize startup or initial arming");
  check(so101_lerobot_follower::active_session_survives_leader_diagnostic(true,
            LeaderDiagnosticState::kRecoverableFeedbackLoss),
        "recoverable diagnostics preserve only an already active session");
  check(!so101_lerobot_follower::active_session_survives_leader_diagnostic(true,
            LeaderDiagnosticState::kFatal),
        "fatal diagnostics never preserve an active session");
  check(!so101_lerobot_follower::active_session_survives_leader_diagnostic(true,
            LeaderDiagnosticState::kUnknown),
        "unknown diagnostics do not preserve an active session");
  check(classify("feetech_read_only_state", 0, "read-only feedback healthy") ==
            LeaderDiagnosticState::kHealthy,
        "healthy diagnostics restore feedback readiness after recovery");
  check(!state.update(true, 54500000000LL),
        "healthy feedback recovery during an enabled heartbeat is not a new session edge");
  check(!so101_lerobot_follower::torque_enable_authorized(false, true, true),
        "feedback recovery cannot issue another torque enable request without a Start edge");

  constexpr int64_t diagnostics_freshness_ns = 1500000000LL;
  const auto readiness_at = [diagnostics_freshness_ns](const std::string &name, uint8_t level,
                                                        const std::string &message, int64_t receipt_ns) {
    return so101_lerobot_follower::leader_diagnostics_fresh(
        so101_lerobot_follower::leader_feedback_diagnostic_healthy(name, level, message),
        receipt_ns, receipt_ns + 1000000LL, diagnostics_freshness_ns);
  };
  check(readiness_at("feetech_read_only_state", 0, "read-only feedback healthy", 100LL),
        "in-range diagnostic yields fresh leader readiness");
  check(readiness_at("feetech_read_only_state", 1,
        "feedback published; wrist_flex=874 outside [914,3236]; leader-to-follower mapping clamps targets to follower calibration limits", 200LL),
        "out-of-range diagnostic transition retains fresh leader readiness");
  check(readiness_at("feetech_read_only_state", 0, "read-only feedback healthy", 300LL),
        "range recovery retains fresh leader readiness");
  check(so101_lerobot_follower::leader_diagnostics_fresh(true, 100LL, 100LL, diagnostics_freshness_ns),
        "healthy diagnostics received now are fresh");
  check(!so101_lerobot_follower::leader_diagnostics_fresh(false, 100LL, 100LL, diagnostics_freshness_ns),
        "unhealthy diagnostics are never fresh");
  check(!so101_lerobot_follower::leader_diagnostics_fresh(true, 100LL, 100LL + diagnostics_freshness_ns + 1,
                                                          diagnostics_freshness_ns),
        "healthy but stale diagnostics are not fresh");
  check(!so101_lerobot_follower::leader_diagnostics_fresh(true, 0LL, 100LL, diagnostics_freshness_ns),
        "missing diagnostics are not fresh");
  check(!so101_lerobot_follower::leader_diagnostics_fresh(true, 101LL, 100LL, diagnostics_freshness_ns),
        "future diagnostics receipt is invalid");

  check(!so101_lerobot_follower::action_is_newer_than_session(999, 1000),
        "previous-session action timestamp is rejected");
  check(so101_lerobot_follower::action_is_newer_than_session(1001, 1000),
        "current-session action timestamp passes the session boundary");
  check(!so101_lerobot_follower::sequence_is_newer(8, 8), "duplicate action sequence is rejected");
  check(!so101_lerobot_follower::sequence_is_newer(7, 8), "old session sequence is rejected");
  check(so101_lerobot_follower::sequence_is_newer(9, 8), "new sequence is accepted");
  check(so101_lerobot_follower::sequence_is_newer(0, 0xffffffffU), "sequence wrap remains ordered");
  return failures == 0 ? 0 : 1;
}
