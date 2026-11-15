cmake_minimum_required(VERSION 3.8)
project(so101_lerobot_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(feetech_ros2_driver REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SO101Action.msg"
  "msg/AudioChunk.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

add_executable(so101_lerobot_follower_node src/follower_node.cpp)
target_compile_features(so101_lerobot_follower_node PUBLIC cxx_std_20)
target_include_directories(so101_lerobot_follower_node PRIVATE include)
ament_target_dependencies(so101_lerobot_follower_node
  rclcpp rcutils rcl_interfaces std_msgs sensor_msgs diagnostic_msgs feetech_ros2_driver ament_index_cpp)
target_link_libraries(so101_lerobot_follower_node nlohmann_json::nlohmann_json)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(so101_lerobot_follower_node "${cpp_typesupport_target}")

install(TARGETS so101_lerobot_follower_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/camera_streamer_node.py
  scripts/audio_streamer_node.py
  scripts/audio_mic_streamer_node.py
  scripts/audio_playback_node.py
  scripts/audio_tcp_sender_node.py
  scripts/audio_tcp_receiver_node.py
  scripts/inspect_camera.py
  scripts/action_feedback_to_joint_states.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY description meshes DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

enable_testing()
add_executable(test_action_mapping test/test_action_mapping.cpp)
target_compile_features(test_action_mapping PUBLIC cxx_std_17)
target_include_directories(test_action_mapping PRIVATE include)
add_test(NAME action_mapping COMMAND test_action_mapping)
add_executable(test_calibration test/test_calibration.cpp)
target_compile_features(test_calibration PUBLIC cxx_std_17)
target_include_directories(test_calibration PRIVATE include)
target_link_libraries(test_calibration nlohmann_json::nlohmann_json)
add_test(NAME calibration_validation COMMAND test_calibration)
add_executable(test_calibration_sync test/test_calibration_sync.cpp)
target_compile_features(test_calibration_sync PUBLIC cxx_std_17)
target_include_directories(test_calibration_sync PRIVATE include)
add_test(NAME calibration_sync COMMAND test_calibration_sync)
add_executable(test_torque_enable test/test_torque_enable.cpp)
target_compile_features(test_torque_enable PUBLIC cxx_std_17)
target_include_directories(test_torque_enable PRIVATE include)
add_test(NAME torque_enable COMMAND test_torque_enable)
add_executable(test_leader_session_state test/test_leader_session_state.cpp)
target_compile_features(test_leader_session_state PUBLIC cxx_std_17)
target_include_directories(test_leader_session_state PRIVATE include)
add_test(NAME leader_session_state COMMAND test_leader_session_state)
add_executable(test_motion_limiter test/test_motion_limiter.cpp)
target_compile_features(test_motion_limiter PUBLIC cxx_std_17)
target_include_directories(test_motion_limiter PRIVATE include)
add_test(NAME motion_limiter COMMAND test_motion_limiter)
add_test(
  NAME startup_publisher_order
  COMMAND ${CMAKE_COMMAND}
    -DSOURCE_FILE=${CMAKE_CURRENT_SOURCE_DIR}/src/follower_node.cpp
    -P ${CMAKE_CURRENT_SOURCE_DIR}/test/check_startup_publisher_order.cmake
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
