import pybullet as p
import pybullet_data
import random
import time
import os

def launch_world(gui=True, run_time=None):
    if gui:
        physics_client = p.connect(p.GUI_SERVER)
    else:
        physics_client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    plane_id = p.loadURDF("plane.urdf")
    robot_start_pos = [0, 0, 0]
    robot_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    kuka_path = "kuka_iiwa/model.urdf"
    kuka_id = p.loadURDF(kuka_path, robot_start_pos, robot_start_orientation, useFixedBase=True)

    print(f"Loaded KUKA robot with ID: {kuka_id}")    

    table_position = [0.6, 0, 0]
    table_orientation = p.getQuaternionFromEuler([0, 0, 0])
    table_id = p.loadURDF("table/table.urdf", table_position, table_orientation)

    cube_start_pos = [0.5, 0, 0.645]
    cube_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    cube_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.03, 0.03, 0.03])
    cube_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.03, 0.03, 0.03], rgbaColor=[1, 0, 0, 1])
    cube_id = p.createMultiBody(
        baseMass=0.1,
        baseCollisionShapeIndex=cube_collision_shape,
        baseVisualShapeIndex=cube_visual_shape,
        basePosition=cube_start_pos,
        baseOrientation=cube_start_orientation
    )

    print(f"Table ID: {table_id}, Cube ID: {cube_id}")

    hold_default_pose(kuka_id)
    add_clutter(num_objects=5, table_id=table_id)

    p.setRealTimeSimulation(1)

    '''try:
        print("Running simulation. Press CTRL+C to exit.")
        if run_time is None:
            while True:
                time.sleep(0.01)
        else:
            start = time.time()
            while time.time() - start < run_time:
                time.sleep(0.01)
    except KeyboardInterrupt:
        print("Simulation ended by user.")
    finally:
        p.disconnect()'''

    return kuka_id, table_id, cube_id

def hold_default_pose(robot_id):
    num_joints = p.getNumJoints(robot_id)

    target_positions = [0, 0.3, 0, -1.0, 0, 1.2, 0.5]

    for joint_index in range(num_joints):
        joint_info = p.getJointInfo(robot_id, joint_index)
        joint_type = joint_info[2]

        if joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
            p.setJointMotorControl2(
                bodyUniqueId=robot_id,
                jointIndex=joint_index,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target_positions[joint_index],
                force=500
            )

def add_clutter(num_objects=5, table_id=None):
    if table_id is None:
        raise ValueError("You must pass the table_id so clutter height can be calculated.")

    aabb_min, aabb_max = p.getAABB(table_id)
    table_top_z = aabb_max[2]

    for _ in range(num_objects):
        x = random.uniform(0.4, 0.7)
        y = random.uniform(-0.35, 0.35)
        z = table_top_z + 0.025
        obj_path = random.choice([
            "cube_small.urdf",
            "sphere_small.urdf",
            "duck_vhacd.urdf"
        ])
        clutter_id = p.loadURDF(obj_path, [x, y, z])
        p.changeDynamics(clutter_id, -1, mass=0)

if __name__ == "__main__":
    launch_world()