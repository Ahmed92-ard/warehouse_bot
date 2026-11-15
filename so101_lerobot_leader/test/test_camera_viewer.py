import inspect
import importlib.util
import unittest
from unittest.mock import Mock
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from sensor_msgs.msg import CompressedImage

from so101_lerobot_leader import gui as leader_gui
from so101_lerobot_leader.camera_viewer import CAMERA_TOPIC, CameraViewer, CameraViewerNode, LatestFrameBuffer

_launch_path = Path(__file__).resolve().parents[1] / "launch" / "leader_gui.launch.py"
_launch_spec = importlib.util.spec_from_file_location("leader_gui_launch", _launch_path)
_launch_module = importlib.util.module_from_spec(_launch_spec)
_launch_spec.loader.exec_module(_launch_module)
generate_launch_description = _launch_module.generate_launch_description


class CameraViewerTest(unittest.TestCase):
    def test_leader_gui_has_no_camera_subscription_or_rendering(self):
        gui_source = inspect.getsource(leader_gui.LeaderNode) + inspect.getsource(leader_gui.LeaderGui)
        self.assertNotIn("CompressedImage", gui_source)
        self.assertNotIn("_on_camera_image", gui_source)
        self.assertNotIn("_render_camera_loop", gui_source)
        self.assertNotIn("camera_lock", gui_source)
        self.assertNotIn("camera_condition", gui_source)

    def test_viewer_uses_existing_compressed_camera_topic(self):
        self.assertEqual(CAMERA_TOPIC, "/so101/follower/camera/image/compressed")
        self.assertEqual(CameraViewerNode.topic, CAMERA_TOPIC)

    def test_frame_buffer_retains_only_the_latest_message_without_copying(self):
        frames = LatestFrameBuffer()
        messages = [CompressedImage() for _ in range(100)]
        for index, message in enumerate(messages):
            message.data = [index]
            self.assertTrue(frames.put(message))

        latest = frames.take_latest()
        self.assertIsNotNone(latest)
        self.assertIs(latest[0], messages[-1])
        self.assertEqual(list(latest[0].data), [99])
        self.assertIsNone(frames.take_latest())

    def test_camera_viewer_is_optional_and_control_nodes_are_unconditional(self):
        description = generate_launch_description()
        entities = description.entities
        viewer_arg = next(
            entity for entity in entities
            if isinstance(entity, DeclareLaunchArgument) and entity.name == "enable_camera_viewer")
        context = LaunchContext()
        default_value = "".join(item.perform(context) for item in viewer_arg.default_value)
        self.assertEqual(default_value, "false")

        nodes = [entity for entity in entities if isinstance(entity, Node)]
        viewer = next(node for node in nodes if node._Node__node_executable == "leader_camera_viewer")
        control_nodes = [node for node in nodes if node._Node__node_executable in (
            "so101_leader_state_node", "leader_gui")]
        self.assertEqual(len(control_nodes), 2)
        self.assertTrue(all(node.condition is None for node in control_nodes))

        context.launch_configurations["enable_camera_viewer"] = "false"
        self.assertFalse(viewer.condition.evaluate(context))
        context.launch_configurations["enable_camera_viewer"] = "true"
        self.assertTrue(viewer.condition.evaluate(context))

    def test_viewer_close_discards_pending_frame_and_returns_cleanly(self):
        frames = LatestFrameBuffer()
        frames.put(CompressedImage())
        viewer = CameraViewer.__new__(CameraViewer)
        viewer.frames = frames
        viewer.root = Mock()
        viewer.closed = False

        viewer.close()
        viewer.close()

        self.assertTrue(frames.closed)
        self.assertIsNone(frames.take_latest())
        viewer.root.destroy.assert_called_once()


if __name__ == "__main__":
    unittest.main()
