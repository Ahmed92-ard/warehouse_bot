# SO-101 Self-Contained Follower & USB Camera Streaming

Native C++ ROS 2 Jazzy follower arm controller and asynchronous USB camera streamer for the Raspberry Pi.

## Key Features & Enhancements

1. **Integrated `feetech_ros2_driver`**:
   - Reuses `feetech_driver::CommunicationProtocol` and `feetech_driver::SerialPort` from `feetech_ros2_driver`.
   - Eliminates duplicate Feetech packet framing and byte-level checksum logic.
2. **Local Package Calibration**:
   - Packaged calibration copied directly into `config/follower.json`.
   - Completely removes runtime reads from Hugging Face cache paths (`~/.cache/huggingface`).
3. **Guarded Torque & Calibration Safety**:
   - EEPROM registers (homing offsets and angle limits) are verified read-only against local calibration at startup.
   - Disabled heartbeats only keep the session disarmed. Torque activation requires an explicit GUI Start transition, fresh healthy leader driver diagnostics, and complete follower hardware/calibration/mode/feedback preflight.
   - Verifies torque readback sequentially per motor before reporting readiness.
4. **Dual USB Camera Streaming (`camera_streamer_node.py`)**:
   - Captures the two arm cameras by stable `/dev/v4l/by-id/` names so USB re-enumeration does not swap camera assignments. The combined MARS launch puts the WCM gripper-facing camera on arm camera 2, which is the QR scanning feed.
   - Publishes JPEG-compressed `sensor_msgs/msg/CompressedImage` frames on separate camera topics.
   - Best-effort depth=1 QoS drops old frames to keep latency bounded.
   - Completely asynchronous: runs in a separate process, independent of the arm control loop, and never alters torque state.
   - Automatically reconnects if camera is disconnected or plugged in late.
5. **USB Microphone Streaming (`audio_streamer_node.py`)**:
   - Captures mono 16 kHz S16_LE PCM from ALSA with `arecord` and publishes 20 ms chunks.
   - Audio uses best-effort QoS and does not interact with motor control.
6. **Standalone Camera Inspection (`inspect_camera.py`)**:
   - Tool to inspect V4L2 supported resolutions, pixel formats, and frame rates.

## ROS 2 Topics

| Topic | Type | Direction | Description |
| :--- | :--- | :--- | :--- |
| `/so101/leader/action` | `so101_lerobot_follower/msg/SO101Action` | Input (30 Hz) | Sequenced 6-DOF leader actions (deg x5, gripper 0-100%) |
| `/so101/leader/teleop_enabled` | `std_msgs/msg/Bool` | Input (2 Hz / Event) | Session activation heartbeat from leader |
| `/so101/follower/action_feedback` | `so101_lerobot_follower/msg/SO101Action` | Output (50 Hz) | Present position feedback in native units |
| `/so101/follower/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Output (10 Hz) | Readiness, bus health, and motor diagnostics |
| `/so101/follower/camera/image/compressed` | `sensor_msgs/msg/CompressedImage` | Output (30 Hz) | JPEG-compressed USB camera stream |
| `/so101/follower/camera2/image/compressed` | `sensor_msgs/msg/CompressedImage` | Output (15 Hz) | Second USB camera stream |
| `/so101/follower/audio/pcm` | `so101_lerobot_follower/msg/AudioChunk` | Output (50 Hz) | 16 kHz mono PCM microphone chunks |

## Launch Commands

### 1. Full Follower Launch (Arm + USB Camera)
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch so101_lerobot_follower follower.launch.py \
  serial_port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14114210-if00 \
  audio_device:=plughw:CARD=CAM,DEV=0 \
  enable_camera:=true
```

If the rover is already running `rover/combined_mars_so101.launch.py` (which starts camera 1), start camera 2 and microphone capture separately on the Pi:

```bash
ros2 launch so101_lerobot_follower media_streamer.launch.py \
  audio_device:=plughw:CARD=CAM,DEV=0
```

The launch files use stable camera by-ID paths by default. Check that the IDs exist with `ls -l /dev/v4l/by-id/`; only override them if a camera is replaced or its USB identifier changes. Audio capture applies 3× digital gain by default before publishing, with clipping protection. Check the ALSA capture device with `arecord -l`; install `alsa-utils` if `arecord` is unavailable.

### 2. Camera-Only Streaming (No Motors / No Torque)
```bash
ros2 launch so101_lerobot_follower camera_streamer.launch.py \
  fps:=30.0 \
  jpeg_quality:=75
```

### 3. V4L2 Camera Mode Inspection
```bash
ros2 run so101_lerobot_follower inspect_camera.py
```
