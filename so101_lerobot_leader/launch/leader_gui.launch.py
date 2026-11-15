"""Launch file for SO-101 Leader GUI teleoperation station."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('so101_lerobot_leader')
    default_calibration = PathJoinSubstitution([pkg_share, 'config', 'leader.json'])
    follower_pkg_share = FindPackageShare('so101_lerobot_follower')
    default_follower_calibration = PathJoinSubstitution(
        [follower_pkg_share, 'config', 'follower.json'])

    leader_state = Node(
        package='so101_lerobot_leader',
        executable='so101_leader_state_node',
        name='leader_state',
        namespace='so101',
        output='screen',
        parameters=[{
            'leader_port': LaunchConfiguration('leader_port'),
            'leader_calibration_file': LaunchConfiguration('leader_calibration_file'),
            'baud_rate': 1000000,
            'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
        }],
    )
    leader_gui = Node(
        package='so101_lerobot_leader',
        executable='leader_gui',
        name='leader_gui',
        namespace='so101',
        output='screen',
        parameters=[{
            'leader_port': LaunchConfiguration('leader_port'),
            'leader_calibration_file': LaunchConfiguration('leader_calibration_file'),
            'follower_calibration_file': LaunchConfiguration('follower_calibration_file'),
            'follower_ip': LaunchConfiguration('follower_ip'),
            'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
            'leader_feedback_pause_timeout_sec': LaunchConfiguration('leader_feedback_pause_timeout_sec'),
            'debug_logging': LaunchConfiguration('debug_logging'),
            'warehouse_api_url': LaunchConfiguration('warehouse_api_url'),
            'warehouse_env_file': LaunchConfiguration('warehouse_env_file'),
            'qr_camera_topic': LaunchConfiguration('qr_camera_topic'),
        }],
    )
    camera_viewer = Node(
        package='so101_lerobot_leader',
        executable='leader_camera_viewer',
        name='leader_camera_viewer',
        namespace='so101',
        output='screen',
        parameters=[{
            'camera_topic': LaunchConfiguration('camera_topic'),
            'camera2_topic': LaunchConfiguration('camera2_topic'),
            'rover_camera_topic': LaunchConfiguration('rover_camera_topic'),
        }],
    )
    audio_player = Node(
        package='so101_lerobot_leader',
        executable='leader_audio_player',
        name='leader_audio_player',
        namespace='so101',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_follower_audio')),
        parameters=[{
            'audio_topic': LaunchConfiguration('audio_topic'),
            'playback_device': ParameterValue(LaunchConfiguration('audio_playback_device'), value_type=str),
        }],
    )
    laptop_audio_sender = Node(
        package='so101_lerobot_follower',
        executable='audio_tcp_sender_node.py',
        name='laptop_audio_tcp_sender',
        namespace='mars',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_laptop_mic_to_rover')),
        parameters=[{
            'source': LaunchConfiguration('laptop_mic_source'),
            'host': LaunchConfiguration('laptop_audio_host'),
            'port': LaunchConfiguration('laptop_audio_port'),
            'sample_rate': 16000,
            'channels': 1,
            'chunk_ms': 10,
        }],
    )

    stop_launch_on_exit = lambda node: RegisterEventHandler(
        OnProcessExit(
            target_action=node,
            on_exit=[EmitEvent(event=Shutdown(reason='A leader launch process exited'))],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('ros_domain_id', default_value='42',
                              description='ROS domain shared by the leader GUI, driver, and follower'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        DeclareLaunchArgument('leader_port', default_value='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B8E116792-if00',
                              description='Serial device for the leader Feetech arm'),
        DeclareLaunchArgument('leader_calibration_file', default_value=default_calibration,
                              description='Path to local leader calibration JSON'),
        DeclareLaunchArgument('follower_calibration_file', default_value=default_follower_calibration,
                              description='Local copy of the paired follower calibration JSON'),
        DeclareLaunchArgument('follower_ip', default_value='10.104.219.64',
                              description='Follower arm / Raspberry Pi IP address'),
        DeclareLaunchArgument('publish_rate_hz', default_value='30.0',
                              description='Leader feedback and action publishing target rate in Hz'),
        DeclareLaunchArgument('leader_feedback_pause_timeout_sec', default_value='0.25',
                              description='Pause action output after this feedback age; the session remains active'),
        DeclareLaunchArgument('debug_logging', default_value='false',
                              description='Enable verbose debug logging'),
        DeclareLaunchArgument('enable_follower_audio', default_value='true',
                              description='Play follower microphone audio on the laptop'),
        DeclareLaunchArgument('camera_topic', default_value='/so101/follower/camera/image/compressed',
                              description='ROS topic for follower camera 1'),
        DeclareLaunchArgument('camera2_topic', default_value='/so101/follower/camera2/image/compressed',
                              description='ROS topic for follower camera 2'),
        DeclareLaunchArgument('rover_camera_topic', default_value='/mars/camera/image/compressed',
                              description='ROS topic for the rover front camera'),
        DeclareLaunchArgument('qr_camera_topic', default_value='/so101/follower/camera2/image/compressed',
                              description='ROS topic for Arm camera 2 used for QR scanning'),
        DeclareLaunchArgument('warehouse_api_url', default_value='http://127.0.0.1:8000',
                              description='Local warehouse API used for operator task confirmations'),
        DeclareLaunchArgument('warehouse_env_file', default_value='/home/ahmed/ros2_ws/src/rover/rover/warehouse_backend/.env',
                              description='Local environment file containing the operator API key'),
        DeclareLaunchArgument('audio_topic', default_value='/so101/follower/audio/pcm',
                              description='ROS topic for follower microphone PCM'),
        DeclareLaunchArgument('audio_playback_device', default_value='default',
                              description='ALSA output device on the leader computer'),
        DeclareLaunchArgument('enable_laptop_mic_to_rover', default_value='true',
                              description='Send this laptop microphone to the rover speaker'),
        DeclareLaunchArgument('laptop_mic_source', default_value='',
                              description='PulseAudio/PipeWire source name; empty uses the default mic'),
        DeclareLaunchArgument('laptop_audio_host', default_value='192.168.0.113',
                              description='Raspberry Pi address for the low-latency audio stream'),
        DeclareLaunchArgument('laptop_audio_port', default_value='5004',
                              description='TCP port for laptop microphone audio'),

        leader_state,
        leader_gui,
        camera_viewer,
        audio_player,
        laptop_audio_sender,
        stop_launch_on_exit(leader_state),
        stop_launch_on_exit(leader_gui),
        stop_launch_on_exit(audio_player),
        stop_launch_on_exit(laptop_audio_sender),
    ])
