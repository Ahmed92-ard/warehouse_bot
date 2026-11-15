# SO-101 Self-Contained Leader Teleoperation Station

Modernized ROS 2 Jazzy leader teleoperation station with integrated follower camera streaming, Feetech ROS 2 driver communication, local calibration management, and responsive live joint monitoring.

## Key Architecture & Enhancements

1. **Self-Contained & LeRobot-Independent**:
   - Runtime dependencies on the official LeRobot Python package have been completely eliminated.
   - The leader-local ROS node calls the existing `feetech_ros2_driver` public C++ API. It does not implement packet handling, and the GUI never opens the motor port.
2. **Local Package Calibration**:
   - Calibration values are copied directly into the package configuration (`config/leader.json`).
   - Zero runtime reliance on Hugging Face cache directories (`~/.cache/huggingface`) or internet downloads.
3. **Hardware Integration & Torque Safety**:
   - Reuses the existing `feetech_ros2_driver`.
   - One leader-local C++ node owns the leader serial port and makes no motor writes.
   - EEPROM registers (homing offsets, angle limits) and motor model (STS3215/777) are verified against local calibration before connecting.
   - Joint limits are strictly enforced: moving out of range temporarily pauses publication (`range_pause`) without aborting the session.
4. **Remote Follower Media**:
   - The optional media window subscribes to both compressed camera topics and renders the latest frame from each.
   - Plays 16 kHz mono PCM received from the follower's USB microphone using `aplay`.
   - Bounded best-effort queues keep stale video and audio from building up during network congestion.
   - Runs independently from arm teleoperation.
5. **Modernized Leader GUI**:
   - Clean slate/dark theme with responsive layout.
   - Follower network check and ping tool.
   - Leader connect / disconnect controls.
   - Prominent Start / Stop teleoperation buttons with armed/disarmed indicators.
   - Live 6-joint telemetry card showing Leader angle/percent, Follower angle/percent, and real-time tracking error ($\Delta$).
   - Real-time diagnostics log viewer with debug logging toggle.

## ROS 2 Topics

| Topic | Type | Direction | Description |
| :--- | :--- | :--- | :--- |
| `/so101/leader/action` | `so101_lerobot_follower/msg/SO101Action` | Output (30 Hz) | 6-DOF leader actions (deg x5, gripper 0-100%) |
| `/so101/leader/teleop_enabled` | `std_msgs/msg/Bool` | Output (2 Hz / Event) | Heartbeat session activation state |
| `/so101/leader/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Output (1 Hz) | Leader bus health and status |
| `/so101/leader/joint_states` | `sensor_msgs/msg/JointState` | Input to GUI (30 Hz) | Raw position counts from the C++ driver node |
| `/so101/leader/driver_diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Output | Leader connection, validation, and feedback health |
| `/so101/follower/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Input | Follower arm readiness and health |
| `/so101/follower/action_feedback` | `so101_lerobot_follower/msg/SO101Action` | Input (30 Hz) | Live follower joint feedback |
| `/so101/follower/camera/image/compressed` | `sensor_msgs/msg/CompressedImage` | Input (30 Hz) | Remote follower USB camera feed |
| `/so101/follower/camera2/image/compressed` | `sensor_msgs/msg/CompressedImage` | Input (15 Hz) | Remote follower second USB camera feed |
| `/so101/follower/audio/pcm` | `so101_lerobot_follower/msg/AudioChunk` | Input (50 Hz) | Remote follower microphone audio |

## Launch & Run

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42

# Launch modernized Leader GUI
ros2 launch so101_lerobot_leader leader_gui.launch.py \
  leader_port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B8E116792-if00 \
  follower_ip:=10.104.219.64 \
  enable_camera_viewer:=true
```

The optional media window shows both follower cameras and plays microphone audio through the leader computer's default ALSA output. Set `audio_playback_device:=<alsa-device>` to select another output.

On the Raspberry Pi, keep the current `rover` combined bringup running for the arm and first camera, then start the second camera and USB microphone:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch so101_lerobot_follower media_streamer.launch.py \
  camera2_device:=/dev/video2 \
  audio_device:=plughw:CARD=CAM,DEV=0
```

Build both packages and their dependencies in the workspace before running either command:

```bash
cd ~/ros2_ws
colcon build --packages-up-to so101_lerobot_leader so101_lerobot_follower
```
