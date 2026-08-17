"""
Smoke test: launches sim_node + perception_node together and asserts
/detected_objects actually gets published with a real detection. Proves the
ROS2 wiring itself works end-to-end (camera frame -> segmentation detection
-> published message) -- not full coverage of every node's behavior.
"""
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray


@pytest.mark.launch_test
def generate_test_description():
    sim_node = launch_ros.actions.Node(
        package='arm_ros2', executable='sim_node', name='sim_node',
        parameters=[{'mode': 'single'}],
    )
    perception_node = launch_ros.actions.Node(
        package='arm_ros2', executable='perception_node', name='perception_node',
    )
    return launch.LaunchDescription([
        sim_node,
        perception_node,
        launch_testing.actions.ReadyToTest(),
    ])


class TestDetectedObjectsPublished(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node('test_bringup_smoke')

    def tearDown(self):
        self.node.destroy_node()

    def test_red_cube_is_detected(self):
        received = []
        self.node.create_subscription(
            Detection3DArray, '/detected_objects', lambda msg: received.append(msg), 10
        )

        # Give sim_node + perception_node time to start, load the scene,
        # settle physics, and publish at least one detection frame.
        end_time = self.node.get_clock().now().nanoseconds + int(20e9)
        while self.node.get_clock().now().nanoseconds < end_time and not received:
            rclpy.spin_once(self.node, timeout_sec=0.5)

        self.assertTrue(len(received) > 0, "No message received on /detected_objects within 20s")
        colors = [det.results[0].hypothesis.class_id for det in received[-1].detections if det.results]
        self.assertIn('red', colors, f"Expected 'red' among detections, got {colors}")
