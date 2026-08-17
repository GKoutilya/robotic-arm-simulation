"""
sim_node: owns the PyBullet simulation and is the ROS2 graph's "hardware
interface" -- publishes joint states and camera frames, executes trajectories,
and grasps/releases objects. Wraps the existing src.demo.pick_and_place
functions (load_scene, hold_initial_pose, attach_object_to_ee, release_grasp)
unchanged; nothing about the underlying simulation logic is reimplemented here.
"""
import os

import pybullet as p
import pybullet_data
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Header, Int32MultiArray
from std_srvs.srv import Trigger
from control_msgs.action import FollowJointTrajectory

from arm_interfaces.msg import CameraFrame as CameraFrameMsg
from arm_interfaces.srv import Grasp

from src import config
from src.camera.camera_sim import capture_camera_image
from src.demo import pick_and_place as pap

from arm_ros2.ros_conversions import camera_frame_to_msg, numpy_to_image_msg

# pick_and_place.load_scene() uses paths relative to the repo root (matching
# how it's always been invoked: `python -m src.demo.pick_and_place`, which
# naturally sets CWD to the repo root). A ROS2 launch context has an arbitrary
# CWD, so replicate that assumption explicitly rather than touching load_scene.
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(pap.__file__), '..', '..'))


class SimNode(Node):
    def __init__(self):
        super().__init__('sim_node')
        os.chdir(REPO_ROOT)
        pap.configure_logging(verbose=False, quiet=False)

        self.declare_parameter('mode', 'single')
        mode = self.get_parameter('mode').get_parameter_value().string_value

        p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        self.robot_id, self.cube_ids, self.table_id, self.plane_id = pap.load_scene(mode=mode)
        pap.hold_initial_pose(self.robot_id)

        self.grasp_constraint_id = None

        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.camera_pub = self.create_publisher(CameraFrameMsg, '/camera/frame', 5)
        self.camera_rgb_preview_pub = self.create_publisher(Image, '/camera/rgb', 5)

        # Transient-local ("latched"): perception_node needs these IDs to
        # exclude plane/table/robot from detection, even if it subscribes
        # after this is published.
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.known_ids_pub = self.create_publisher(Int32MultiArray, '/sim/known_body_ids', latched_qos)
        ids_msg = Int32MultiArray()
        ids_msg.data = [self.plane_id, self.table_id, self.robot_id]
        self.known_ids_pub.publish(ids_msg)

        self._camera_tick = 0
        self.timer = self.create_timer(1.0 / 30.0, self._on_timer)

        self._action_server = ActionServer(
            self, FollowJointTrajectory, '/execute_trajectory', self._execute_trajectory_cb
        )
        self._grasp_srv = self.create_service(Grasp, '/grasp', self._grasp_cb)
        self._release_srv = self.create_service(Trigger, '/release', self._release_cb)

        self.get_logger().info(f'sim_node ready (mode={mode}), robot_id={self.robot_id}, '
                                f'cube_ids={self.cube_ids}, table_id={self.table_id}, plane_id={self.plane_id}')

    def _joint_index_by_name(self, name):
        for i in range(p.getNumJoints(self.robot_id)):
            if p.getJointInfo(self.robot_id, i)[1].decode('utf-8') == name:
                return i
        return None

    def _publish_joint_states(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(p.getNumJoints(self.robot_id)):
            info = p.getJointInfo(self.robot_id, i)
            state = p.getJointState(self.robot_id, i)
            msg.name.append(info[1].decode('utf-8'))
            msg.position.append(state[0])
            msg.velocity.append(state[1])
        self.joint_state_pub.publish(msg)

    def _publish_camera_frame(self):
        frame = capture_camera_image()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera'
        self.camera_pub.publish(camera_frame_to_msg(frame, header))
        # Also as a plain sensor_msgs/Image for RViz2/rqt, which can't
        # subscribe to the nested .rgb field inside the custom CameraFrame msg.
        self.camera_rgb_preview_pub.publish(numpy_to_image_msg(frame.rgb.astype('uint8'), 'rgb8', header))

    def _on_timer(self):
        for _ in range(8):
            p.stepSimulation()
        self._publish_joint_states()
        self._camera_tick += 1
        if self._camera_tick % 6 == 0:  # ~5 Hz camera publish at a 30 Hz timer
            self._publish_camera_frame()

    def _execute_trajectory_cb(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        joint_names = trajectory.joint_names
        self.get_logger().info(f'Executing trajectory with {len(trajectory.points)} points')

        for point in trajectory.points:
            for name, target in zip(joint_names, point.positions):
                joint_index = self._joint_index_by_name(name)
                if joint_index is not None:
                    p.setJointMotorControl2(
                        self.robot_id, joint_index, p.POSITION_CONTROL,
                        targetPosition=target, force=config.JOINT_FORCE,
                    )
            for _ in range(120):
                p.stepSimulation()
            self._publish_joint_states()

            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = joint_names
            goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def _grasp_cb(self, request, response):
        cid = pap.attach_object_to_ee(self.robot_id, config.EE_LINK, request.body_id)
        if cid is None:
            response.success = False
            response.message = 'Grasp failed: object too far from end-effector'
        else:
            self.grasp_constraint_id = cid
            response.success = True
            response.message = f'Grasped body {request.body_id}'
        return response

    def _release_cb(self, request, response):
        pap.release_grasp(self.grasp_constraint_id)
        self.grasp_constraint_id = None
        response.success = True
        response.message = 'Released'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SimNode()
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
