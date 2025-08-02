import pybullet as p
import pybullet_data
import time
import os

def launch_world():
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    p.setGravity(0, 0, -9.81)

    plane_id = p.loadURDF("plane.urdf")

    kuka_path = "kuka_iiwa/model.urdf"
    kuka_id = p.loadURDF(kuka_path, useFixedBase=False)

    print(f"Loaded KUKA robot with ID: {kuka_id}")

    for _ in range(1000):
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

    p.disconnect()


if __name__ == "__main__":
    launch_world()