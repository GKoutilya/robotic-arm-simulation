import sys
import os
import argparse
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from src.control.arm_controller import ArmController
from src.simulation.world import add_clutter
from src.camera.camera_sim import capture_camera_image
from src.camera.object_detector import find_target_object, find_all_objects, COLORS
from src.control.inverse_kinematics import calculate_ik
from src.control.planner import interpolate_positions, plan_arc_trajectory, get_current_ee_position, plan_pick_and_place_trajectory
from src.simulation.utils import create_colored_cube, create_sorting_bins, wait_for_settle, print_joint_info
import pybullet_data
import pybullet as p
import numpy as np
import time

# ============== CONFIGURATION ==============
# Set your desired place location here!
DESIRED_PLACE_POSITION = [0.5, 0.0, 0.65]  # Center of table - where obstacle will be
OBSTACLE_CLEAR_OFFSET = [0.0, 0.25, 0.0]   # Move obstacles to the side (y direction)
TABLE_HEIGHT = 0.65  # Height of the table surface
CUBE_HALF_SIZE = 0.025  # Half the size of the cube (cube is 0.05m)
# ===========================================

# Global to track the obstacle ID
OBSTACLE_ID = None

# Home position for the robot joints
HOME_POSITION = [0.0, 0.5, 0.0, -1.0, 0.0, 1.0, 0.0]


def connect_sim(gui=True):
    if gui:
        physics_client = p.connect(p.GUI)
    else:
        physics_client = p.connect(p.DIRECT)
    
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    return physics_client


def load_scene(mode='single'):
    """
    Load the simulation scene.
    """
    plane_id = p.loadURDF("plane.urdf")
    table_path = os.path.join("assets", "table", "table.urdf")
    table_id = p.loadURDF(table_path, basePosition=[0.5, 0, 0.325], useFixedBase=True)
    
    robot_path = "kuka_iiwa/model.urdf"
    robot_id = p.loadURDF(robot_path, basePosition=[0, 0, 0], useFixedBase=True)
    
    cube_ids = []
    
    if mode == 'single':
        cube_path = os.path.join("assets", "objects", "cube.urdf")
        # Place red cube FAR from the target location (0.5, 0.0)
        red_cube_pos = [0.35, -0.20, 0.7]
        cube_id = p.loadURDF(cube_path, basePosition=red_cube_pos)
        cube_ids.append(cube_id)
        print(f"[DEBUG] Red cube spawned at {red_cube_pos}, ID: {cube_id}")
    elif mode == 'multi' or mode == 'sort':
        colors = [
            [1, 0, 0, 1],    # Red
            [0, 1, 0, 1],    # Green
            [0, 0, 1, 1],    # Blue
        ]
        positions = [
            [0.45, -0.1, 0.7],
            [0.5, 0, 0.7],
            [0.55, 0.1, 0.7],
        ]
        for pos, color in zip(positions, colors):
            cube_id = create_colored_cube(pos, color, size=0.05)
            cube_ids.append(cube_id)
    
    return robot_id, cube_ids, table_id


def hold_initial_pose(robot_id):
    """
    Set the robot to a safe initial pose and hold it.
    """
    num_joints = p.getNumJoints(robot_id)

    for i in range(min(num_joints, len(HOME_POSITION))):
        p.resetJointState(robot_id, i, HOME_POSITION[i])
        p.setJointMotorControl2(
            robot_id,
            i,
            p.POSITION_CONTROL,
            targetPosition=HOME_POSITION[i],
            force=500
        )

    wait_for_settle(200)


def return_to_home(robot_id):
    """
    Smoothly return the robot to its home/initial pose.
    """
    print(f"\n{'='*60}")
    print("[ACTION] RETURNING TO HOME POSITION")
    print(f"{'='*60}")
    
    num_joints = p.getNumJoints(robot_id)
    
    # Get current joint positions
    current_positions = []
    for i in range(min(num_joints, len(HOME_POSITION))):
        joint_state = p.getJointState(robot_id, i)
        current_positions.append(joint_state[0])
    
    # Smoothly interpolate to home position
    num_steps = 60
    for step in range(num_steps + 1):
        t = step / num_steps
        # Smooth ease-in-out interpolation
        t_smooth = t * t * (3 - 2 * t)
        
        for i in range(min(num_joints, len(HOME_POSITION))):
            target = current_positions[i] + t_smooth * (HOME_POSITION[i] - current_positions[i])
            p.setJointMotorControl2(
                robot_id,
                i,
                p.POSITION_CONTROL,
                targetPosition=target,
                force=500
            )
        
        for _ in range(4):
            p.stepSimulation()
            time.sleep(1./240.)
    
    wait_for_settle(100)
    print("[SUCCESS] Robot returned to home position!")


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


def go_to_pose(robot_id, ee_link, pos, orn=None, steps=240):
    """
    Move the robot end-effector to a target position.
    """
    if orn is None:
        orn = p.getQuaternionFromEuler([np.pi, 0, 0])
    
    q = calculate_ik(robot_id, pos, orn, end_effector_index=ee_link)
    
    if q is None:
        print(f"[ERROR] IK failed for position {pos}")
        return False
    
    return move_joints(robot_id, q, steps=steps)


def go_to_pose_smooth(robot_id, ee_link, target_pos, orn=None, num_steps=5):
    """
    Move the robot end-effector to a target position with smooth interpolation.
    """
    if orn is None:
        orn = p.getQuaternionFromEuler([np.pi, 0, 0])
    
    current_pos = get_current_ee_position(robot_id, ee_link)
    waypoints = interpolate_positions(current_pos, target_pos, num_steps)
    
    for waypoint in waypoints[1:]:
        success = go_to_pose(robot_id, ee_link, waypoint, orn, steps=120)
        if not success:
            return False
    
    return True


def move_ee_to_object(robot_id, ee_link, obj_id, height_offset=0.03, max_iterations=5):
    """
    Move the end-effector directly above and then down to the object.
    Uses the actual object position to ensure accurate alignment.
    """
    for iteration in range(max_iterations):
        obj_pos, _ = p.getBasePositionAndOrientation(obj_id)
        obj_pos = list(obj_pos)
        
        target_pos = [obj_pos[0], obj_pos[1], obj_pos[2] + height_offset]
        
        print(f"[INFO] Iteration {iteration + 1}: Moving EE to {target_pos}")
        
        success = go_to_pose(robot_id, ee_link, target_pos, steps=200)
        if not success:
            print(f"[WARNING] IK failed for position {target_pos}")
            continue
        
        wait_for_settle(100)
        
        ee_state = p.getLinkState(robot_id, ee_link)
        ee_pos = ee_state[4]
        
        horizontal_dist = np.sqrt(
            (ee_pos[0] - obj_pos[0])**2 + 
            (ee_pos[1] - obj_pos[1])**2
        )
        
        print(f"[INFO] EE position: {ee_pos}")
        print(f"[INFO] Object position: {obj_pos}")
        print(f"[INFO] Horizontal distance: {horizontal_dist:.4f}m")
        
        if horizontal_dist < 0.03:
            print("[INFO] End-effector aligned with object!")
            return True
    
    print("[WARNING] Could not perfectly align with object, proceeding anyway...")
    return True


def attach_object_to_ee(robot_id, ee_link, obj_id):
    """
    Attach the object directly below the end-effector.
    The object will be centered under the EE, not offset to the side.
    """
    ee_state = p.getLinkState(robot_id, ee_link)
    ee_pos = ee_state[4]
    obj_pos, obj_orn = p.getBasePositionAndOrientation(obj_id)
    
    dist = np.sqrt(sum((e - o)**2 for e, o in zip(ee_pos, obj_pos)))
    print(f"[INFO] Distance between EE and object: {dist:.4f}m")
    
    if dist > 0.15:
        print(f"[ERROR] Object too far from end-effector ({dist:.3f}m)")
        return None
    
    z_offset = ee_pos[2] - obj_pos[2]
    
    cid = p.createConstraint(
        parentBodyUniqueId=robot_id,
        parentLinkIndex=ee_link,
        childBodyUniqueId=obj_id,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=[0, 0, -z_offset],
        childFramePosition=[0, 0, 0],
    )
    
    p.changeConstraint(cid, maxForce=1000)
    
    print(f"[INFO] Created grasp constraint: {cid}")
    print(f"[INFO] Object attached {z_offset:.3f}m below end-effector")
    
    return cid


def release_grasp(constraint_id):
    """
    Release the grasped object.
    """
    if constraint_id is not None:
        p.removeConstraint(constraint_id)
        print(f"[INFO] Released grasp constraint: {constraint_id}")


def gentle_place(robot_id, ee_link, obj_id, target_xy, table_height, grasp_constraint):
    """
    Gently place an object on the table surface.
    Lowers the object until it's just above the table, then releases.
    """
    ee_state = p.getLinkState(robot_id, ee_link)
    ee_pos = ee_state[4]
    obj_pos, _ = p.getBasePositionAndOrientation(obj_id)
    
    ee_to_obj_offset = ee_pos[2] - obj_pos[2]
    target_obj_z = table_height + CUBE_HALF_SIZE + 0.005
    target_ee_z = target_obj_z + ee_to_obj_offset
    
    print(f"[INFO] Gentle place calculation:")
    print(f"[INFO]   Table height: {table_height}")
    print(f"[INFO]   Target object Z: {target_obj_z}")
    print(f"[INFO]   EE to object offset: {ee_to_obj_offset}")
    print(f"[INFO]   Target EE Z: {target_ee_z}")
    
    above_target = [target_xy[0], target_xy[1], target_ee_z + 0.10]
    print(f"[INFO] Moving above target: {above_target}")
    go_to_pose(robot_id, ee_link, above_target, steps=200)
    wait_for_settle(50)
    
    current_z = target_ee_z + 0.10
    descent_step = 0.02
    
    while current_z > target_ee_z:
        current_z = max(current_z - descent_step, target_ee_z)
        descent_pos = [target_xy[0], target_xy[1], current_z]
        print(f"[INFO] Descending to: {descent_pos}")
        go_to_pose(robot_id, ee_link, descent_pos, steps=100)
        wait_for_settle(30)
        
        obj_pos_now, _ = p.getBasePositionAndOrientation(obj_id)
        if obj_pos_now[2] < table_height + CUBE_HALF_SIZE + 0.02:
            print(f"[INFO] Object near table surface at z={obj_pos_now[2]:.4f}")
            break
    
    final_pos = [target_xy[0], target_xy[1], target_ee_z]
    print(f"[INFO] Final descent to: {final_pos}")
    go_to_pose(robot_id, ee_link, final_pos, steps=150)
    wait_for_settle(50)
    
    obj_pos_final, _ = p.getBasePositionAndOrientation(obj_id)
    print(f"[INFO] Object position before release: {obj_pos_final}")
    print(f"[INFO] Height above table: {obj_pos_final[2] - table_height:.4f}m")
    
    print("[INFO] Releasing object gently...")
    release_grasp(grasp_constraint)
    
    wait_for_settle(150)
    
    obj_pos_settled, _ = p.getBasePositionAndOrientation(obj_id)
    print(f"[INFO] Object settled at: {obj_pos_settled}")
    
    drift = np.sqrt(
        (obj_pos_settled[0] - target_xy[0])**2 +
        (obj_pos_settled[1] - target_xy[1])**2
    )
    print(f"[INFO] Horizontal drift after release: {drift:.4f}m")
    
    return True


def move_obstacle_to_clear_position(robot_id, ee_link, obstacle_id, clear_position):
    """
    Move the obstacle object to a clear position.
    This is the core function that actually moves the obstacle.
    """
    print(f"\n{'='*60}")
    print(f"[ACTION] MOVING OBSTACLE (ID: {obstacle_id}) OUT OF THE WAY")
    print(f"{'='*60}")
    
    # Get current obstacle position
    obs_pos, _ = p.getBasePositionAndOrientation(obstacle_id)
    obs_pos = list(obs_pos)
    print(f"[INFO] Obstacle current position: {obs_pos}")
    print(f"[INFO] Target clear position: {clear_position}")
    
    # Step 1: Approach from above
    approach = [obs_pos[0], obs_pos[1], obs_pos[2] + 0.15]
    print(f"[INFO] Step 1: Approaching obstacle at {approach}")
    go_to_pose_smooth(robot_id, ee_link, approach)
    wait_for_settle(50)
    
    # Step 2: Align and descend to obstacle
    print("[INFO] Step 2: Aligning with obstacle...")
    move_ee_to_object(robot_id, ee_link, obstacle_id, height_offset=0.03)
    
    # Step 3: Grasp the obstacle
    print("[INFO] Step 3: Grasping obstacle...")
    grasp_cid = attach_object_to_ee(robot_id, ee_link, obstacle_id)
    
    if grasp_cid is None:
        print("[ERROR] Failed to grasp obstacle!")
        go_to_pose(robot_id, ee_link, approach, steps=150)
        return False
    
    wait_for_settle(50)
    
    # Step 4: Lift the obstacle
    lift = [obs_pos[0], obs_pos[1], obs_pos[2] + 0.20]
    print(f"[INFO] Step 4: Lifting obstacle to {lift}")
    go_to_pose(robot_id, ee_link, lift, steps=180)
    wait_for_settle(50)
    
    # Step 5: Transport to clear position
    transport = [clear_position[0], clear_position[1], TABLE_HEIGHT + 0.20]
    print(f"[INFO] Step 5: Transporting obstacle to {transport}")
    go_to_pose_smooth(robot_id, ee_link, transport)
    wait_for_settle(50)
    
    # Step 6: Gently place the obstacle
    print("[INFO] Step 6: Gently placing obstacle...")
    gentle_place(robot_id, ee_link, obstacle_id, [clear_position[0], clear_position[1]], TABLE_HEIGHT, grasp_cid)
    
    # Step 7: Retreat
    retreat = [clear_position[0], clear_position[1], TABLE_HEIGHT + 0.15]
    print(f"[INFO] Step 7: Retreating to {retreat}")
    go_to_pose(robot_id, ee_link, retreat, steps=180)
    wait_for_settle(50)
    
    # Verify obstacle was moved
    new_pos, _ = p.getBasePositionAndOrientation(obstacle_id)
    print(f"\n[SUCCESS] Obstacle moved!")
    print(f"[INFO]   From: {obs_pos}")
    print(f"[INFO]   To: {new_pos}")
    print(f"{'='*60}\n")
    
    return True


def check_and_clear_obstacle(robot_id, ee_link, target_position, red_cube_id, obstacle_id):
    """
    Check if there's an obstacle at the target position and move it if needed.
    Uses the known obstacle_id directly instead of searching.
    """
    print(f"\n{'='*60}")
    print(f"[CHECK] IS THERE AN OBSTACLE AT TARGET POSITION?")
    print(f"[INFO] Target position: {target_position}")
    print(f"[INFO] Red cube ID (to place): {red_cube_id}")
    print(f"[INFO] Known obstacle ID: {obstacle_id}")
    print(f"{'='*60}")
    
    if obstacle_id is None:
        print("[INFO] No obstacle was created. Target is clear!")
        return True
    
    # Get obstacle position
    obs_pos, _ = p.getBasePositionAndOrientation(obstacle_id)
    obs_pos = list(obs_pos)
    print(f"[INFO] Obstacle position: {obs_pos}")
    
    # Calculate distance from target
    dist = np.sqrt(
        (target_position[0] - obs_pos[0])**2 + 
        (target_position[1] - obs_pos[1])**2
    )
    print(f"[INFO] Distance from target to obstacle: {dist:.4f}m")
    
    # Check if obstacle is blocking (within 12cm of target)
    if dist > 0.12:
        print(f"[INFO] Obstacle is NOT blocking target (distance > 0.12m)")
        return True
    
    print(f"\n[ALERT] OBSTACLE IS BLOCKING TARGET POSITION!")
    print(f"[INFO] Must move obstacle before placing red cube.")
    
    # Calculate clear position
    clear_position = [
        target_position[0] + OBSTACLE_CLEAR_OFFSET[0],
        target_position[1] + OBSTACLE_CLEAR_OFFSET[1],
        TABLE_HEIGHT
    ]
    
    # Ensure within table bounds
    table_bounds = {
        'x_min': 0.30, 'x_max': 0.65,
        'y_min': -0.30, 'y_max': 0.30
    }
    clear_position[0] = np.clip(clear_position[0], table_bounds['x_min'], table_bounds['x_max'])
    clear_position[1] = np.clip(clear_position[1], table_bounds['y_min'], table_bounds['y_max'])
    
    print(f"[INFO] Will move obstacle to: {clear_position}")
    
    # Move the obstacle
    success = move_obstacle_to_clear_position(robot_id, ee_link, obstacle_id, clear_position)
    
    return success


def pick_object(robot_id, ee_link, obj_id, use_smooth=True):
    """
    Pick up an object and return the grasp constraint.
    """
    obj_pos, _ = p.getBasePositionAndOrientation(obj_id)
    obj_pos = list(obj_pos)
    
    print(f"\n{'='*60}")
    print(f"[ACTION] PICKING UP OBJECT (ID: {obj_id})")
    print(f"[INFO] Object position: {obj_pos}")
    print(f"{'='*60}")
    
    # Approach from above
    approach = [obj_pos[0], obj_pos[1], obj_pos[2] + 0.15]
    print(f"[INFO] Approaching at {approach}")
    if use_smooth:
        go_to_pose_smooth(robot_id, ee_link, approach)
    else:
        go_to_pose(robot_id, ee_link, approach)
    wait_for_settle(50)
    
    # Align and descend
    print("[INFO] Aligning with object...")
    move_ee_to_object(robot_id, ee_link, obj_id, height_offset=0.03)
    
    # Grasp
    print("[INFO] Grasping object...")
    grasp_constraint = attach_object_to_ee(robot_id, ee_link, obj_id)
    
    if grasp_constraint is None:
        print("[ERROR] Failed to grasp object!")
        return None
    
    wait_for_settle(50)
    
    # Lift
    obj_pos_new, _ = p.getBasePositionAndOrientation(obj_id)
    lift = [obj_pos_new[0], obj_pos_new[1], obj_pos[2] + 0.25]
    print(f"[INFO] Lifting to {lift}")
    go_to_pose(robot_id, ee_link, lift, steps=200)
    wait_for_settle(50)
    
    print("[SUCCESS] Object picked up!")
    return grasp_constraint


def place_object(robot_id, ee_link, obj_id, place_pos, grasp_constraint, use_smooth=True):
    """
    Place an object at the target position.
    """
    print(f"\n{'='*60}")
    print(f"[ACTION] PLACING OBJECT AT TARGET")
    print(f"[INFO] Target position: {place_pos}")
    print(f"{'='*60}")
    
    # Transport to above target
    transport = [place_pos[0], place_pos[1], TABLE_HEIGHT + 0.25]
    print(f"[INFO] Transporting to {transport}")
    if use_smooth:
        go_to_pose_smooth(robot_id, ee_link, transport)
    else:
        go_to_pose(robot_id, ee_link, transport)
    wait_for_settle(50)
    
    # Gently place
    print("[INFO] Gently placing object...")
    gentle_place(robot_id, ee_link, obj_id, [place_pos[0], place_pos[1]], TABLE_HEIGHT, grasp_constraint)
    
    # Retreat
    retreat = [place_pos[0], place_pos[1], TABLE_HEIGHT + 0.15]
    print(f"[INFO] Retreating to {retreat}")
    go_to_pose(robot_id, ee_link, retreat, steps=200)
    wait_for_settle(50)
    
    print("[SUCCESS] Object placed!")
    return True


def run_single_mode(robot_id, cube_ids, table_id, ee_link, obstacle_id):
    """
    Run single object pick-and-place demo.
    """
    red_cube_id = cube_ids[0]
    
    # Get positions
    red_pos, _ = p.getBasePositionAndOrientation(red_cube_id)
    red_pos = list(red_pos)
    
    place_pos = DESIRED_PLACE_POSITION.copy()
    place_pos[2] = TABLE_HEIGHT
    
    print(f"\n{'#'*70}")
    print(f"[TASK] PICK AND PLACE DEMONSTRATION")
    print(f"{'#'*70}")
    print(f"[INFO] Red cube (ID: {red_cube_id}) is at: {red_pos}")
    if obstacle_id is not None:
        obs_pos, _ = p.getBasePositionAndOrientation(obstacle_id)
        print(f"[INFO] Yellow obstacle (ID: {obstacle_id}) is at: {obs_pos}")
    print(f"[INFO] Target place position: {place_pos}")
    print(f"[INFO] ")
    print(f"[INFO] SEQUENCE:")
    print(f"[INFO]   1. Check if obstacle is blocking target")
    print(f"[INFO]   2. If yes, move obstacle out of the way")
    print(f"[INFO]   3. Pick up red cube")
    print(f"[INFO]   4. Place red cube at target")
    print(f"[INFO]   5. Return to home position")
    print(f"{'#'*70}\n")
    
    time.sleep(2)
    
    # PHASE 1: Clear obstacle if needed
    print("\n" + "="*70)
    print("PHASE 1: CLEAR OBSTACLE IF BLOCKING")
    print("="*70)
    check_and_clear_obstacle(robot_id, ee_link, place_pos, red_cube_id, obstacle_id)
    
    # PHASE 2: Pick up the red cube
    print("\n" + "="*70)
    print("PHASE 2: PICK UP RED CUBE")
    print("="*70)
    grasp_constraint = pick_object(robot_id, ee_link, red_cube_id, use_smooth=True)
    
    if grasp_constraint is None:
        print("[FAILED] Could not pick up red cube!")
        return_to_home(robot_id)
        return False
    
    # PHASE 3: Place the red cube
    print("\n" + "="*70)
    print("PHASE 3: PLACE RED CUBE AT TARGET")
    print("="*70)
    success = place_object(robot_id, ee_link, red_cube_id, place_pos, grasp_constraint, use_smooth=True)
    
    # PHASE 4: Return to home position
    print("\n" + "="*70)
    print("PHASE 4: RETURN TO HOME POSITION")
    print("="*70)
    return_to_home(robot_id)
    
    if success:
        print(f"\n{'#'*70}")
        print("[SUCCESS] TASK COMPLETE!")
        final_pos, _ = p.getBasePositionAndOrientation(red_cube_id)
        print(f"[INFO] Red cube final position: {final_pos}")
        print(f"{'#'*70}\n")
    else:
        print("[FAILED] Task failed!")
    
    return success


def run_multi_mode(robot_id, cube_ids, table_id, ee_link):
    """
    Run multi-object pick-and-place demo.
    """
    place_positions = [
        [0.35, -0.15, TABLE_HEIGHT],
        [0.35, 0.0, TABLE_HEIGHT],
        [0.35, 0.15, TABLE_HEIGHT],
    ]
    
    for i, cube_id in enumerate(cube_ids):
        print(f"\n[INFO] Processing object {i + 1}/{len(cube_ids)}...")
        
        place_pos = place_positions[i % len(place_positions)]
        
        grasp = pick_object(robot_id, ee_link, cube_id)
        if grasp:
            place_object(robot_id, ee_link, cube_id, place_pos, grasp)
    
    # Return to home after all objects placed
    print("\n" + "="*70)
    print("RETURNING TO HOME POSITION")
    print("="*70)
    return_to_home(robot_id)
    
    print("\n[SUCCESS] Multi-object pick-and-place complete!")


def run_sort_mode(robot_id, cube_ids, table_id, ee_link):
    """
    Run object sorting demo - sort by color.
    """
    sort_bins = {
        'red': [0.35, -0.15, TABLE_HEIGHT],
        'green': [0.35, 0.0, TABLE_HEIGHT],
        'blue': [0.35, 0.15, TABLE_HEIGHT],
    }
    
    color_names = ['red', 'green', 'blue']
    
    for i, cube_id in enumerate(cube_ids):
        color = color_names[i % len(color_names)]
        place_pos = sort_bins[color]
        
        print(f"\n[INFO] Sorting {color} cube to {place_pos}")
        
        grasp = pick_object(robot_id, ee_link, cube_id)
        if grasp:
            place_object(robot_id, ee_link, cube_id, place_pos, grasp)
    
    # Return to home after sorting
    print("\n" + "="*70)
    print("RETURNING TO HOME POSITION")
    print("="*70)
    return_to_home(robot_id)
    
    print("\n[SUCCESS] Object sorting complete!")


def main():
    global OBSTACLE_ID
    
    parser = argparse.ArgumentParser(description='Vision-guided pick-and-place demo')
    parser.add_argument('--mode', type=str, default='single', choices=['single', 'multi', 'sort'],
                        help='Demo mode: single, multi, or sort')
    parser.add_argument('--no-clutter', action='store_true', help='Disable clutter objects')
    parser.add_argument('--no-obstacle', action='store_true', help='Do not place obstacle at target')
    parser.add_argument('--place-x', type=float, default=None, help='X coordinate for place position')
    parser.add_argument('--place-y', type=float, default=None, help='Y coordinate for place position')
    args = parser.parse_args()
    
    global DESIRED_PLACE_POSITION
    if args.place_x is not None:
        DESIRED_PLACE_POSITION[0] = args.place_x
    if args.place_y is not None:
        DESIRED_PLACE_POSITION[1] = args.place_y
    
    try:
        print(f"\n{'*'*70}")
        print(f"  VISION-GUIDED PICK-AND-PLACE WITH OBSTACLE CLEARING")
        print(f"{'*'*70}")
        print(f"[INFO] Mode: {args.mode}")
        print(f"[INFO] Target place position: {DESIRED_PLACE_POSITION}")
        print(f"[INFO] Obstacle at target: {not args.no_obstacle}")
        print(f"{'*'*70}\n")

        print("[SETUP] Connecting to PyBullet...")
        physics_client = connect_sim(gui=True)

        print("[SETUP] Loading scene...")
        robot_id, cube_ids, table_id = load_scene(mode=args.mode)
        print(f"[SETUP] Robot ID: {robot_id}, Red cube ID: {cube_ids}, Table ID: {table_id}")

        print("[SETUP] Setting initial pose...")
        hold_initial_pose(robot_id)

        # Skip clutter for cleaner demo
        if not args.no_clutter and args.mode == 'single':
            print("[SETUP] Skipping clutter for cleaner obstacle demo...")
        
        # Create yellow obstacle at target location
        obstacle_id = None
        if not args.no_obstacle and args.mode == 'single':
            print(f"\n[SETUP] Creating YELLOW OBSTACLE at target {DESIRED_PLACE_POSITION}...")
            obstacle_pos = [DESIRED_PLACE_POSITION[0], DESIRED_PLACE_POSITION[1], 0.7]
            obstacle_id = create_colored_cube(obstacle_pos, [1.0, 1.0, 0.0, 1.0], size=0.05)
            OBSTACLE_ID = obstacle_id
            print(f"[SETUP] Yellow obstacle created at {obstacle_pos}, ID: {obstacle_id}")

        print("[SETUP] Letting simulation settle...")
        wait_for_settle(300)
        
        # Print final positions
        print("\n[SETUP] Object positions after settling:")
        for cid in cube_ids:
            pos, _ = p.getBasePositionAndOrientation(cid)
            print(f"[SETUP]   Red cube (ID: {cid}): {[round(p, 3) for p in pos]}")
        if obstacle_id is not None:
            pos, _ = p.getBasePositionAndOrientation(obstacle_id)
            print(f"[SETUP]   Yellow obstacle (ID: {obstacle_id}): {[round(p, 3) for p in pos]}")

        ee_link = 6

        # Run the appropriate mode
        if args.mode == 'single':
            run_single_mode(robot_id, cube_ids, table_id, ee_link, obstacle_id)
        elif args.mode == 'multi':
            run_multi_mode(robot_id, cube_ids, table_id, ee_link)
        elif args.mode == 'sort':
            run_sort_mode(robot_id, cube_ids, table_id, ee_link)

        print("\n[INFO] Simulation complete. Press CTRL+C to exit...")
        while True:
            p.stepSimulation()
            time.sleep(1.0 / 240.0)

    except KeyboardInterrupt:
        print("\n[INFO] Simulation terminated by user.")
    except Exception as e:
        print(f"[ERROR] An exception occurred: {e}")
        import traceback
        traceback.print_exc()
        
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