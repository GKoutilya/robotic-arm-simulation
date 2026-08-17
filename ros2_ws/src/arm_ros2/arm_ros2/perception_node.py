"""
perception_node: subscribes to the raw camera frame published by sim_node,
reconstructs a CameraFrame, and reuses the existing segmentation-based
detection pipeline (find_objects_by_segmentation) completely unchanged,
publishing vision_msgs/Detection3DArray.
"""
import pybullet as p
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Int32MultiArray
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from arm_interfaces.msg import CameraFrame as CameraFrameMsg

from src import config
from src.camera.object_detector import find_objects_by_segmentation

from arm_ros2.ros_conversions import msg_to_camera_frame


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # A throwaway DIRECT connection purely to compute the camera's
        # view/projection matrices via the same helper calls
        # camera_sim.capture_camera_image uses -- these are pure math utility
        # functions, and the camera is a fixed known constant (src.config.CAMERA),
        # so recomputing them here is simpler than transmitting matrices over ROS.
        p.connect(p.DIRECT)
        cam = config.CAMERA
        self._view_matrix = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=cam.target_pos, distance=cam.distance,
            yaw=cam.yaw, pitch=cam.pitch, roll=cam.roll, upAxisIndex=2,
        )
        self._projection_matrix = p.computeProjectionMatrixFOV(
            cam.fov, cam.aspect, cam.near, cam.far,
        )

        self._ignore_body_ids = frozenset()
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Int32MultiArray, '/sim/known_body_ids', self._known_ids_cb, latched_qos)

        self.create_subscription(CameraFrameMsg, '/camera/frame', self._camera_frame_cb, 5)
        self.detections_pub = self.create_publisher(Detection3DArray, '/detected_objects', 10)

        self.get_logger().info('perception_node ready')

    def _known_ids_cb(self, msg):
        self._ignore_body_ids = frozenset(msg.data)

    def _camera_frame_cb(self, msg):
        frame = msg_to_camera_frame(msg, self._view_matrix, self._projection_matrix)
        detections = find_objects_by_segmentation(frame, ignore_body_ids=self._ignore_body_ids)

        array_msg = Detection3DArray()
        array_msg.header = msg.header

        for color_name, info in detections.items():
            det = Detection3D()
            det.header = msg.header
            det.id = str(info['body_id'])

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = color_name
            hyp.hypothesis.score = 1.0
            hyp.pose.pose.position.x, hyp.pose.pose.position.y, hyp.pose.pose.position.z = info['position']
            det.results.append(hyp)

            array_msg.detections.append(det)

        self.detections_pub.publish(array_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
