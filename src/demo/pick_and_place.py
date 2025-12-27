import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from src.control.arm_controller import ArmController
from src.simulation.world import add_clutter
from src.camera.camera_sim import capture_camera_image
from src.camera.object_detector import find_target_object
from src.control.inverse_kinematics import calculate_ik
import pybullet_data
import pybullet as p
import numpy as np
import time

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
    cube_id = p.loadURDF(cube_path, basePosition=[0.5, 0, 0.7])  # Start higher, will fall onto table
    robot_path = "kuka_iiwa/model.urdf"
    robot_id = p.loadURDF(robot_path, basePosition=[0, 0, 0], useFixedBase=True)
    return robot_id, cube_id, table_id

def hold_initial_pose(robot_id):
    """
    Set the robot to a safe initial pose and hold it.
    """
    num_joints = p.getNumJoints(robot_id)
    # Safe initial joint positions (arm in a neutral, upright position)
    initial_positions = [0.0, 0.5, 0.0, -1.0, 0.0, 1.0, 0.0]

    for i in range(min(num_joints, len(initial_positions))):
        p.resetJointState(robot_id, i, initial_positions[i])
        p.setJointMotorControl2(
            robot_id,
            i,
            p.POSITION_CONTROL,
            targetPosition=initial_positions[i],
            force=500
        )

    # Let the robot stabilize
    for _ in range(200):
        p.stepSimulation()
        time.sleep(1./240.)

def move_joints(robot_id, q, steps=240):
    """
    Move robot joints to target positions.
    """
    if q is None:
        print("[ERROR] Invalid joint positions (None)")
        return False
    
    num_joints = p.getNumJoints(robot_id)
    for i in range(num_joints):
        target = q[i] if i < len(q) else 0.0
        p.setJointMotorControl2(
            robot_id,
            i,
            p.POSITION_CONTROL,
            targetPosition=target,
            force=500
        )
    
    for _ in range(steps):
        p.stepSimulation()
        time.sleep(1./240.)
    
    return True

def go_to_pose(robot_id, ee_link, pos, orn=None):
    """
    Move the robot end-effector to a target position.
    """
    if orn is None:
        orn = p.getQuaternionFromEuler([np.pi, 0, 0])  # Gripper pointing down
    
    q = calculate_ik(robot_id, pos, orn, end_effector_index=ee_link)
    
    if q is None:
        print(f"[ERROR] IK failed for position {pos}")
        return False
    
    return move_joints(robot_id, q)

def check_collisions(robot_id, obj_id, distance_threshold=0.01):
    """
    Check for potential collisions between the robot and an object.
    """
    closest_points = p.getClosestPoints(robot_id, obj_id, distance_threshold)
    if closest_points:
        print(f"[WARNING] Potential collision detected between robot and object {obj_id}")
        return True
    return False

def fake_grasp(robot_id, ee_link, obj_id):
    """
    Attach object to end-effector via a fixed constraint (suction-like).
    """
    cid = p.createConstraint(
        parentBodyUniqueId=robot_id,
        parentLinkIndex=ee_link,
        childBodyUniqueId=obj_id,
        childLinkIndex=-1,  # -1 for base link of the cube
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=[0, 0, 0.05],  # Offset from end-effector
        childFramePosition=[0, 0, 0],
    )
    print(f"[INFO] Created grasp constraint: {cid}")
    return cid

def release_grasp(constraint_id):
    """
    Release the grasped object.
    """
    if constraint_id is not None:
        p.removeConstraint(constraint_id)
        print(f"[INFO] Released grasp constraint: {constraint_id}")

def main():
    try:
        print("[INFO] Starting vision-guided pick-and-place pipeline...")

        print("[DEBUG] Connecting to PyBullet...")
        physics_client = connect_sim(gui=True)
        print(f"[DEBUG] Connected to PyBullet (client: {physics_client}).")

        print("[DEBUG] Loading scene...")
        robot_id, cube_id, table_id = load_scene()
        print(f"[DEBUG] Scene loaded: robot={robot_id}, cube={cube_id}, table={table_id}")

        print("[DEBUG] Holding initial pose...")
        hold_initial_pose(robot_id)
        print("[DEBUG] Initial pose set.")

        print("[DEBUG] Adding clutter...")
        add_clutter(num_objects=5, table_id=table_id)
        print("[DEBUG] Clutter added.")

        # Let simulation settle (cube falls onto table)
        print("[DEBUG] Letting simulation settle...")
        for _ in range(300):
            p.stepSimulation()
            time.sleep(1./240.)

        # Get cube position after it settles
        cube_pos, cube_orn = p.getBasePositionAndOrientation(cube_id)
        cube_pos = list(cube_pos)
        print(f"[INFO] Cube position after settling: {cube_pos}")

        # Try vision-based detection (looking for RED cube)
        print("[INFO] Detecting target object...")
        target_pos = find_target_object(target_color=[255, 0, 0], threshold=80)

        if target_pos is None:
            print("[WARNING] Vision failed to detect object. Using fallback position.")
            target_pos = cube_pos
        
        print(f"[INFO] Target position: {target_pos}")

        # Setup end-effector link
        ee_link = 6
        print(f"[DEBUG] Robot has {p.getNumJoints(robot_id)} joints, using ee_link={ee_link}")

        # Define waypoints based on target position
        pre_grasp = [target_pos[0], target_pos[1], target_pos[2] + 0.15]
        grasp_pos = [target_pos[0], target_pos[1], target_pos[2] + 0.05]
        lift_pos = [target_pos[0], target_pos[1], target_pos[2] + 0.25]
        place_pos = [target_pos[0] - 0.15, target_pos[1] + 0.1, target_pos[2] + 0.25]
        place_down = [target_pos[0] - 0.15, target_pos[1] + 0.1, target_pos[2] + 0.05]

        print("[INFO] Executing pick-and-place sequence...")

        # Step 1: Move to pre-grasp position
        print("[INFO] Step 1: Moving to pre-grasp position...")
        success = go_to_pose(robot_id, ee_link, pre_grasp)
        if not success:
            print("[ERROR] Failed to reach pre-grasp position")
        
        # Step 2: Move down to grasp position
        print("[INFO] Step 2: Moving to grasp position...")
        success = go_to_pose(robot_id, ee_link, grasp_pos)
        if not success:
            print("[ERROR] Failed to reach grasp position")

        # Step 3: Grasp the object
        print("[INFO] Step 3: Grasping object...")
        grasp_constraint = fake_grasp(robot_id, ee_link, cube_id)
        for _ in range(100):
            p.stepSimulation()
            time.sleep(1./240.)

        # Step 4: Lift the object
        print("[INFO] Step 4: Lifting object...")
        success = go_to_pose(robot_id, ee_link, lift_pos)
        if not success:
            print("[ERROR] Failed to lift object")

        # Step 5: Move to place position
        print("[INFO] Step 5: Moving to place position...")
        success = go_to_pose(robot_id, ee_link, place_pos)
        if not success:
            print("[ERROR] Failed to reach place position")

        # Step 6: Lower to place
        print("[INFO] Step 6: Lowering object...")
        success = go_to_pose(robot_id, ee_link, place_down)
        if not success:
            print("[ERROR] Failed to lower object")

        # Step 7: Release the object
        print("[INFO] Step 7: Releasing object...")
        release_grasp(grasp_constraint)
        for _ in range(100):
            p.stepSimulation()
            time.sleep(1./240.)

        # Step 8: Move back up
        print("[INFO] Step 8: Moving back up...")
        go_to_pose(robot_id, ee_link, place_pos)

        print("[SUCCESS] Pick and place complete!")

        # Keep simulation running
        print("[INFO] Simulation running. Press CTRL+C to exit...")
        while True:
            p.stepSimulation()
            time.sleep(1.0 / 240.0)

    except KeyboardInterrupt:
        print("\n[INFO] Simulation terminated by user.")
    except Exception as e:
        print(f"[ERROR] An exception occurred: {e}")
        import traceback
        traceback.print_exc()
        
        # Keep window open for debugging
        print("[INFO] Keeping simulation open for debugging. Press CTRL+C to exit...")
        try:
            while True:
                p.stepSimulation()
                time.sleep(1.0 / 240.0)
        except KeyboardInterrupt:
            pass
    finally:
        p.disconnect()


if __name__ == "__main__":
    main()