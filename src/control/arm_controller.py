import pybullet as p
import numpy as np
from inverse_kinematics import calculate_ik

class ArmController:
    def __init__(self, robot_id, end_effector_index, joint_indices):
        self.robot_id = robot_id
        self.end_effector_index = end_effector_index
        self.joint_indices = joint_indices

    def move_to_pose(self, target_position, target_orientation=None):
        """
            Moves the robotic arm to the desired end-effector pose.
            Orientation is optional - if None, uses default orientation.
        """
        if target_orientation is None:
            target_orientation = p.getQuaternionFromEuler([0, 0, 0])

        joint_positions = calculate_ik(
            self.robot_id,
            self.end_effector_index,
            target_position,
            target_orientation
        )

        if joint_positions is None:
            print("Inverse kinematics failed to find a solution.")
            return False
        
        for joint_index, joint_angle in zip(self.joint_indices, joint_positions):
            p.setJointMotorControl2(
                bodyIndex=self.robot_id,
                jointIndex=joint_index,
                controlMode=p.POSITION_CONTROL,
                targetPosition=joint_angle,
                force=500
            )
        
        return True
    
    def get_end_effector_position(self):
        """
            Returns the current position (x, y, z) of the end-effector.
        """
        state = p.getLinkState(self.robot_id, self.end_effector_index)
        return state[4]
    
    def get_joint_angles(self):
        """
            Returns the current joint angles.
        """
        joint_states = p.getJointStates(self.robot_id, self.joint_indices)
        return [state[0] for state in joint_states]