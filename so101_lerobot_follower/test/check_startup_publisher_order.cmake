file(READ "${SOURCE_FILE}" follower_source)

string(FIND "${follower_source}" "feedback_pub_ = create_publisher<Action>" feedback_publisher_pos)
string(FIND "${follower_source}" "diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>" diagnostics_publisher_pos)
string(FIND "${follower_source}" "try {\n      load_calibration();" hardware_start_pos)
string(FIND "${follower_source}" "if (calibration_valid_) publish_feedback(actual);" startup_feedback_pos)

if(feedback_publisher_pos LESS 0 OR diagnostics_publisher_pos LESS 0 OR
   hardware_start_pos LESS 0 OR startup_feedback_pos LESS 0)
  message(FATAL_ERROR "Follower startup publisher or hardware initialization pattern was not found")
endif()

if(NOT feedback_publisher_pos LESS hardware_start_pos OR
   NOT diagnostics_publisher_pos LESS hardware_start_pos OR
   NOT hardware_start_pos LESS startup_feedback_pos)
  message(FATAL_ERROR "Follower publishers must be created before hardware startup can publish feedback")
endif()
