"""
task_node: the orchestrator. Exposes /pick_and_place (PickAndPlace action)
and re-expresses the sequence run_single_mode/run_clutter_mode already
encode -- detect -> plan -> execute -> grasp -> plan -> execute -> release --
as a sequence of client calls to the other nodes instead of direct Python
function calls. This is the one node with no direct existing-function
equivalent to wrap: the original orchestration is tightly coupled to
in-process PyBullet calls, so the *sequence* is reused, not the code.

v1 scope: a single-object pick-and-place by target color (mirrors run_single_mode's
complexity). Porting the full clutter-mode nuance (staged gentle-place descent,
obstacle-relocation fallback, closed-loop re-alignment) into this orchestration
is a reasonable follow-up, not attempted here.
"""
import asyncio

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection3DArray

from arm_interfaces.action import PickAndPlace
from arm_interfaces.srv import Grasp, PlanPath

from src import config

APPROACH_HEIGHT_OFFSET = 0.15   # matches pick_object's approach offset in pick_and_place.py
GRASP_HEIGHT_OFFSET = 0.03      # matches move_ee_to_object's default height_offset
PLACE_TRANSPORT_HEIGHT = config.TABLE_HEIGHT + 0.25   # matches place_object's transport height
PLACE_LOWER_HEIGHT = config.TABLE_HEIGHT + 0.08       # close enough above the table for a soft release


class TaskNode(Node):
    def __init__(self):
        super().__init__('task_node')

        self._latest_detections = None
        self.create_subscription(Detection3DArray, '/detected_objects', self._detections_cb, 10)

        self._plan_path_client = self.create_client(PlanPath, '/plan_path')
        self._grasp_client = self.create_client(Grasp, '/grasp')
        self._release_client = self.create_client(Trigger, '/release')
        self._trajectory_client = ActionClient(self, FollowJointTrajectory, '/execute_trajectory')

        self._action_server = ActionServer(
            self, PickAndPlace, '/pick_and_place', self._execute_cb
        )

        self.get_logger().info('task_node ready')

    def _detections_cb(self, msg):
        self._latest_detections = msg

    def _find_detection(self, color_name):
        if self._latest_detections is None:
            return None
        for det in self._latest_detections.detections:
            if det.results and det.results[0].hypothesis.class_id == color_name:
                pos = det.results[0].pose.pose.position
                return {'body_id': int(det.id), 'position': [pos.x, pos.y, pos.z]}
        return None

    def _other_obstacles(self, exclude_color):
        obstacles_positions, obstacles_radii = [], []
        if self._latest_detections is not None:
            for det in self._latest_detections.detections:
                if not det.results or det.results[0].hypothesis.class_id == exclude_color:
                    continue
                pos = det.results[0].pose.pose.position
                obstacles_positions.append(pos)
                obstacles_radii.append(config.OBSTACLE_COLLISION_RADIUS)
        return obstacles_positions, obstacles_radii

    async def _plan(self, goal_xyz, exclude_color):
        request = PlanPath.Request()
        request.goal = Pose()
        request.goal.position.x, request.goal.position.y, request.goal.position.z = goal_xyz
        request.obstacle_positions, request.obstacle_radii = self._other_obstacles(exclude_color)
        response = await self._plan_path_client.call_async(request)
        return response

    async def _execute(self, trajectory):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        goal_handle = await self._trajectory_client.send_goal_async(goal_msg)
        if not goal_handle.accepted:
            return False
        result = await goal_handle.get_result_async()
        return result.status == GoalStatus.STATUS_SUCCEEDED

    async def _plan_and_execute(self, goal_xyz, exclude_color, phase_name, feedback_fn, progress):
        """Returns None on success, or an error message string on failure."""
        feedback_fn(f'PLANNING_{phase_name}', progress)
        plan = await self._plan(goal_xyz, exclude_color=exclude_color)
        if not plan.success:
            return f'{phase_name} planning failed: {plan.message}'

        feedback_fn(f'EXECUTING_{phase_name}', progress + 0.05)
        if not await self._execute(plan.trajectory):
            return f'{phase_name} trajectory execution failed'
        return None

    async def _execute_cb(self, goal_handle):
        request = goal_handle.request
        target_color = request.target_color or 'red'
        self.get_logger().info(f'Goal received: mode={request.mode}, target_color={target_color}')

        def feedback(phase, progress):
            fb = PickAndPlace.Feedback()
            fb.phase = phase
            fb.progress = progress
            goal_handle.publish_feedback(fb)

        def fail(message):
            goal_handle.abort()
            result = PickAndPlace.Result()
            result.success = False
            result.message = message
            self.get_logger().error(message)
            return result

        # 1. Wait briefly for a fresh detection of the target color
        feedback('DETECTING', 0.05)
        detection = None
        for _ in range(50):  # up to ~5s at the ~10Hz /detected_objects rate
            detection = self._find_detection(target_color)
            if detection is not None:
                break
            await self._sleep(0.1)
        if detection is None:
            return fail(f'Could not detect target color "{target_color}"')

        # 2. Approach above the target, then descend close enough to grasp
        # (mirrors pick_object's approach + move_ee_to_object's descent in pick_and_place.py)
        approach = [detection['position'][0], detection['position'][1],
                    detection['position'][2] + APPROACH_HEIGHT_OFFSET]
        error = await self._plan_and_execute(approach, target_color, 'PICK_APPROACH', feedback, 0.10)
        if error:
            return fail(error)

        grasp_pos = [detection['position'][0], detection['position'][1],
                     detection['position'][2] + GRASP_HEIGHT_OFFSET]
        error = await self._plan_and_execute(grasp_pos, target_color, 'PICK_DESCEND', feedback, 0.30)
        if error:
            return fail(error)

        # 3. Grasp
        feedback('GRASPING', 0.5)
        grasp_response = await self._grasp_client.call_async(Grasp.Request(body_id=detection['body_id']))
        if not grasp_response.success:
            return fail(f'Grasp failed: {grasp_response.message}')

        # 4. Transport above the place position, then lower before releasing
        # (mirrors place_object's transport + gentle_place's descent)
        place_xy = config.DESIRED_PLACE_POSITION[:2]
        transport_goal = place_xy + [PLACE_TRANSPORT_HEIGHT]
        error = await self._plan_and_execute(transport_goal, target_color, 'PLACE_TRANSPORT', feedback, 0.60)
        if error:
            return fail(error)

        lower_goal = place_xy + [PLACE_LOWER_HEIGHT]
        error = await self._plan_and_execute(lower_goal, target_color, 'PLACE_LOWER', feedback, 0.85)
        if error:
            return fail(error)

        # 5. Release
        feedback('RELEASING', 0.95)
        await self._release_client.call_async(Trigger.Request())

        goal_handle.succeed()
        result = PickAndPlace.Result()
        result.success = True
        result.message = 'Pick and place complete'
        result.final_position.x, result.final_position.y, result.final_position.z = lower_goal
        return result

    async def _sleep(self, seconds):
        await asyncio.sleep(seconds)


def main(args=None):
    rclpy.init(args=args)
    node = TaskNode()
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
