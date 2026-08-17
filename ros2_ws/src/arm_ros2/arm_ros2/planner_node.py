"""
planner_node: serves /plan_path, wrapping the existing calculate_ik and
motion_planner.plan_path/shortcut_path unchanged. Maintains its own
"kinematic twin" of the robot -- a separate DIRECT PyBullet connection with
just the robot URDF loaded (no plane/table/objects, no physics stepping) --
purely for IK computation, the same separation MoveIt2 uses between a
planning model and the "real" simulated/hardware robot. sim_node owns the
one true physical simulation; this node never touches it directly.
"""
import numpy as np
import pybullet as p
import pybullet_data
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from arm_interfaces.srv import PlanPath

from src import config
from src.control.inverse_kinematics import calculate_ik
from src.control.motion_planner import Obstacle, plan_path, shortcut_path
from src.control.planner import get_current_ee_position

ASSUMED_SPEED_M_PER_S = 0.1  # for estimating time_from_start between waypoints


class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')

        p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.robot_id = p.loadURDF(config.ROBOT_URDF, basePosition=config.ROBOT_POSITION, useFixedBase=True)
        self.joint_names = [
            p.getJointInfo(self.robot_id, i)[1].decode('utf-8')
            for i in range(p.getNumJoints(self.robot_id))
        ]
        self._down_orn = p.getQuaternionFromEuler([np.pi, 0, 0])

        self._latest_joint_state = None
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)
        self.create_service(PlanPath, '/plan_path', self._plan_path_cb)

        self.get_logger().info(f'planner_node ready (kinematic twin robot_id={self.robot_id})')

    def _joint_state_cb(self, msg):
        self._latest_joint_state = msg

    def _sync_twin_to_latest_state(self):
        if self._latest_joint_state is None:
            return
        name_to_pos = dict(zip(self._latest_joint_state.name, self._latest_joint_state.position))
        for i in range(p.getNumJoints(self.robot_id)):
            name = p.getJointInfo(self.robot_id, i)[1].decode('utf-8')
            if name in name_to_pos:
                p.resetJointState(self.robot_id, i, name_to_pos[name])

    def _plan_path_cb(self, request, response):
        self._sync_twin_to_latest_state()

        start = get_current_ee_position(self.robot_id, config.EE_LINK)
        goal = [request.goal.position.x, request.goal.position.y, request.goal.position.z]
        obstacles = [
            Obstacle(position=[pt.x, pt.y, pt.z], radius=radius)
            for pt, radius in zip(request.obstacle_positions, request.obstacle_radii)
        ]

        path = plan_path(
            start, goal, obstacles, config.PLANNING_BOUNDS,
            clearance=config.PLANNER_CLEARANCE, step_size=config.PLANNER_STEP_SIZE,
            max_iterations=config.PLANNER_MAX_ITERATIONS, goal_bias=config.PLANNER_GOAL_BIAS,
        )
        if path is None:
            response.success = False
            response.message = f'No collision-free path found around {len(obstacles)} obstacle(s)'
            self.get_logger().warning(response.message)
            return response

        path = shortcut_path(path, obstacles, clearance=config.PLANNER_CLEARANCE)
        self.get_logger().info(f'Planned path with {len(path)} waypoints around {len(obstacles)} obstacle(s)')

        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        elapsed = 0.0
        prev_point = path[0]
        for waypoint in path:
            joint_angles = calculate_ik(self.robot_id, waypoint, self._down_orn, end_effector_index=config.EE_LINK)
            if joint_angles is None:
                response.success = False
                response.message = f'IK failed for waypoint {waypoint}'
                self.get_logger().error(response.message)
                return response

            for i, angle in enumerate(joint_angles[:p.getNumJoints(self.robot_id)]):
                p.resetJointState(self.robot_id, i, angle)

            elapsed += np.linalg.norm(np.array(waypoint) - np.array(prev_point)) / ASSUMED_SPEED_M_PER_S
            prev_point = waypoint

            point = JointTrajectoryPoint()
            point.positions = list(joint_angles[:len(self.joint_names)])
            point.time_from_start = Duration(sec=int(elapsed), nanosec=int((elapsed % 1.0) * 1e9))
            trajectory.points.append(point)

        response.trajectory = trajectory
        response.success = True
        response.message = f'Planned {len(path)} waypoints'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
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
