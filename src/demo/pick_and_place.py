import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.control.arm_controller import ArmController
from src.simulation.world import add_clutter
from src.camera.camera_sim import capture_camera_image
import src.camera.object_detector
import pybullet as p
import pybullet_data
import time
import os

def connect_sim(gui=True):
    if gui:
        physics_client = p.connect(p.GUI)
    else:
        physics_client = p.connect(p.DIRECT)
    
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    return physics_client

def load_scene():
    plane_id = p.loadURDF("plane.urdf")
    table_path = os.path.join("assets", "table", "table.urdf")
    table_id = p.loadURDF(table_path, basePosition=[0.5, 0, 0.325], useFixedBase=True)
    cube_path = os.path.join("assets", "objects", "cube.urdf")
    cube_id = p.loadURDF(cube_path, basePosition=[0.7, 0, 0.35])
    robot_path = "kuka_iiwa/model.urdf"
    robot_id = p.loadURDF(robot_path, basePosition=[0, 0, 0], useFixedBase=True)
    return robot_id, cube_id, table_id

def get_object_pose(obj_id):
    pos, ori = p.getBasePositionAndOrientation(obj_id)
    return pos, ori

def main():
    connect_sim(gui=True)
    robot_id, cube_id, table_id = load_scene()
    add_clutter(num_objects=8, table_id=table_id)

    # Perception - Get cube pose
    cube_pos, cube_ori = get_object_pose(cube_id)
    print(f"[INFO] Detected cube at: {cube_pos}")

    # Control - Create ArmController
    arm = ArmController(robot_id, end_effector_index=6, joint_indices=list(range(p.getNumJoints(robot_id))))

    pre_grasp = [cube_pos[0], cube_pos[1], cube_pos[2] + 0.15]
    grasp = [cube_pos[0], cube_pos[1], cube_pos[2] + 0.02]
    place = [cube_pos[0] - 0.2, cube_pos[1], cube_pos[2] + 0.15]

    # Execution
    arm.move_to_pose(pre_grasp)
    for _ in range(120): p.stepSimulation(); time.sleep(1./240.)
    arm.move_to_pose(grasp)
    for _ in range(120): p.stepSimulation(); time.sleep(1./240.)
    arm.move_to_pose(pre_grasp)
    for _ in range(120): p.stepSimulation(); time.sleep(1./240.)
    arm.move_to_pose(place)
    for _ in range(120): p.stepSimulation(); time.sleep(1./240.)

    print("[INFO] Pick and place complete.")

    try:
        while True:
            p.stepSimulation()
            time.sleep(1.0 / 240.0)
    except KeyboardInterrupt:
        print("\n[INFO] Simulation terminated by user.")
        p.disconnect()


if __name__ == "__main__":
    main()