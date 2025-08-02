import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.simulation.world import launch_world
from src.control.inverse_kinematics import calculate_ik
import pybullet as p
import pybullet_data
import time

def main():
    robot_id, table_id, cube_id = launch_world(gui=True)

    for _ in range(100):
        p.stepSimulation()
        time.sleep(1./240.)
    
    cube_pos, _ = p.getBasePositionAndOrientation(cube_id)
    target_pos = [cube_pos[0], cube_pos[1], cube_pos[2] + 0.1]
    target_ori = p.getQuaternionFromEuler([0, -3.14, 0])

    end_effector_link_index = 6
    joint_angles = calculate_ik(robot_id, target_pos, target_ori, end_effector_index=end_effector_link_index)

    num_joints = p.getNumJoints(robot_id)
    for i in range(num_joints):
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=i,
            controlMode=p.POSITION_CONTROL,
            targetPosition=joint_angles[i],
            force=500
        )

    for _ in range(1000):
        p.stepSimulation()
        time.sleep(1./240.)

    print("Target Position:", target_pos)
    print("Calculated Joint Positions:", joint_angles)

    for i, j in enumerate(joint_angles):
        p.resetJointState(robot_id, i, j)

    ee_state = p.getLinkState(robot_id, end_effector_link_index)
    actual_pos = ee_state[4]

    print("Actual End-Effector Position:", actual_pos)

    p.disconnect()

if __name__ == "__main__":
    main()