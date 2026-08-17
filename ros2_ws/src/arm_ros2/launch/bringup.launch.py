"""
Starts the full node graph: sim_node (PyBullet + camera + trajectory
execution + grasp/release), perception_node (vision detection),
planner_node (IK + RRT path planning), task_node (pick-and-place
orchestration), plus robot_state_publisher so RViz2 can show a live 3D
model of the arm driven by the real /joint_states.

Usage: ros2 launch arm_ros2 bringup.launch.py mode:=single
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _kuka_urdf_path():
    # pick_and_place.py loads "kuka_iiwa/model.urdf" via PyBullet's own
    # search path (pybullet_data), which lives inside the venv used to run
    # these nodes. $VIRTUAL_ENV is inherited from the sourced venv even
    # though `ros2 launch` itself runs under system Python.
    venv = os.environ.get('VIRTUAL_ENV', os.path.expanduser('~/.venvs/ros2_arm'))
    candidate = os.path.join(venv, 'lib', 'python3.12', 'site-packages',
                              'pybullet_data', 'kuka_iiwa', 'model.urdf')
    return candidate


def generate_launch_description():
    mode_arg = DeclareLaunchArgument('mode', default_value='single',
                                      description='single | multi | sort | clutter')

    urdf_path = _kuka_urdf_path()
    robot_description = ''
    if os.path.exists(urdf_path):
        with open(urdf_path, 'r') as f:
            robot_description = f.read()

    return LaunchDescription([
        mode_arg,

        Node(
            package='arm_ros2', executable='sim_node', name='sim_node', output='screen',
            parameters=[{'mode': LaunchConfiguration('mode')}],
        ),
        Node(
            package='arm_ros2', executable='perception_node', name='perception_node', output='screen',
        ),
        Node(
            package='arm_ros2', executable='planner_node', name='planner_node', output='screen',
        ),
        Node(
            package='arm_ros2', executable='task_node', name='task_node', output='screen',
        ),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
    ])
