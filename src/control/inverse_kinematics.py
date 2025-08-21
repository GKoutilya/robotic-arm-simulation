import pybullet as p
import numpy as np

def calculate_ik(robot_id, end_effector_index=6, target_pos=None, target_ori=None):
    """
        Computes inverse kinematics to reach the given position (and optional orientation).

        Args:
            robot_id (int): ID of the robot in PyBullet.
            target_pos (list): [x, y, z] target position.
            target_ori (list or None): [x, y, z, w] quaternion orientation or None for position-only IK.
            end_effector_index (int): Index of the robot's end-effector link.
        
        Returns:
            list: Joint angles for the robot to reach the target.
    """
    if target_ori is None:
        joint_angles = p.calculateInverseKinematics(robot_id, end_effector_index, target_pos)
    else:
        joint_angles = p.calculateInverseKinematics(robot_id, end_effector_index, target_pos, target_ori)
    
    return joint_angles