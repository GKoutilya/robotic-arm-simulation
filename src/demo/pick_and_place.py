from src.control.inverse_kinematics import calculate_ik
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
    table_id = p.loadURDF(table_path, basePosition=[0.5, 0, 0.325])

    cube_path = os.path.join("assets", "objects", "cube.urdf")
    cube_id = p.loadURDF(cube_path, basePosition=[0.7, 0, 0.35])

    robot_path = os.path.join("assets", "robot", "urSe.urdf")
    robot_id = p.loadURDF(robot_path, basePosition=[0, 0, 0], useFixedBase=True)

    return robot_id, cube_id

def main():
    connect_sim(gui=True)
    robot_id, cube_id = load_scene()

    print("[INFO] simulation initialized.")
    print(f"[INFO] Robot ID: {robot_id}, Cube ID: {cube_id}")

    try:
        while True:
            p.stepSimulation()
            time.sleep(1.0 / 240.0)
    except KeyboardInterrupt:
        print("\n[INFO] Simulation terminated by user.")
        p.disconnect()


if __name__ == "__main__":
    main()