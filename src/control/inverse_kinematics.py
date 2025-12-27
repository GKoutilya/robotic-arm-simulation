import pybullet as p
import numpy as np

def calculate_ik(robot_id, target_pos, target_ori=None, end_effector_index=6):
    """
    Computes inverse kinematics to reach the given position (and optional orientation).

    Args:
        robot_id (int): ID of the robot in PyBullet.
        target_pos (list): [x, y, z] target position.
        target_ori (list or None): [x, y, z, w] quaternion orientation or None for position-only IK.
        end_effector_index (int): Index of the robot's end-effector link.
    
    Returns:
        list: Joint angles for the robot to reach the target, or None if failed.
    """
    try:
        if target_pos is None:
            print("[ERROR] target_pos is None")
            return None
            
        if target_ori is None:
            # Gripper pointing downward
            target_ori = p.getQuaternionFromEuler([np.pi, 0, 0])

        joint_positions = p.calculateInverseKinematics(
            robot_id,
            end_effector_index,
            target_pos,
            target_ori,
            maxNumIterations=100,
            residualThreshold=1e-5
        )

        return list(joint_positions)
    
    except Exception as e:
        print(f'[ERROR] IK Calculation Failed: {e}')
        return None