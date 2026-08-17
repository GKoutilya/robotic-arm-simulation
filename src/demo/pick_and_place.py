import sys
import os
import argparse
import logging
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from src.camera.camera_sim import capture_camera_image
from src.camera.object_detector import find_objects_by_segmentation
from src.control.inverse_kinematics import calculate_ik
from src.control.planner import interpolate_positions, get_current_ee_position
from src.control.motion_planner import Obstacle, plan_path, shortcut_path, find_blocking_obstacle
from src.simulation.utils import create_colored_cube, create_sorting_bins, wait_for_settle
from src import config
import pybullet_data
import pybullet as p
import numpy as np
import random
import time

logger = logging.getLogger("pick_and_place")

# Mutable copy: CLI args (--place-x/--place-y) may override this at runtime.
DESIRED_PLACE_POSITION = list(config.DESIRED_PLACE_POSITION)

# Global to track the obstacle ID
OBSTACLE_ID = None


def configure_logging(verbose=False, quiet=False):
    level = logging.DEBUG if verbose else logging.WARNING if quiet else logging.INFO
    logging.basicConfig(level=level, format="%(message)s")


def connect_sim(gui=True):
    if gui:
        physics_client = p.connect(p.GUI)
    else:
        physics_client = p.connect(p.DIRECT)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    return physics_client


def _random_clutter_xy(avoid_points, min_separation, max_attempts=50):
    """
    Sample an (x, y) within the table bounds (with a small edge margin) that
    stays at least `min_separation` from every point in `avoid_points`.
    Returns None if no valid spot is found within max_attempts.
    """
    margin = 0.03
    for _ in range(max_attempts):
        x = random.uniform(config.TABLE_BOUNDS['x_min'] + margin, config.TABLE_BOUNDS['x_max'] - margin)
        y = random.uniform(config.TABLE_BOUNDS['y_min'] + margin, config.TABLE_BOUNDS['y_max'] - margin)
        if all(np.hypot(x - ax, y - ay) >= min_separation for ax, ay in avoid_points):
            return x, y
    return None


def load_scene(mode='single'):
    """
    Load the simulation scene.

    Returns:
        (robot_id, cube_ids, table_id, plane_id)
    """
    plane_id = p.loadURDF("plane.urdf")
    table_path = os.path.join("assets", "table", "table.urdf")
    table_id = p.loadURDF(table_path, basePosition=config.TABLE_POSITION, useFixedBase=True)

    robot_id = p.loadURDF(config.ROBOT_URDF, basePosition=config.ROBOT_POSITION, useFixedBase=True)

    cube_ids = []

    if mode == 'single':
        cube_path = os.path.join("assets", "objects", "cube.urdf")
        # Place red cube FAR from the target location (0.5, 0.0)
        red_cube_pos = [0.35, -0.20, 0.7]
        cube_id = p.loadURDF(cube_path, basePosition=red_cube_pos)
        cube_ids.append(cube_id)
        logger.debug(f"[DEBUG] Red cube spawned at {red_cube_pos}, ID: {cube_id}")
    elif mode == 'multi' or mode == 'sort':
        positions = [
            [0.45, -0.1, 0.7],
            [0.5, 0, 0.7],
            [0.55, 0.1, 0.7],
        ]
        for pos, color_name in zip(positions, ['red', 'green', 'blue']):
            cube_id = create_colored_cube(pos, config.COLORS_RGBA[color_name], size=config.CUBE_SIZE)
            cube_ids.append(cube_id)
    elif mode == 'clutter':
        cube_path = os.path.join("assets", "objects", "cube.urdf")
        red_cube_pos = [0.35, -0.20, 0.7]
        cube_id = p.loadURDF(cube_path, basePosition=red_cube_pos)
        cube_ids.append(cube_id)
        logger.debug(f"[DEBUG] Red target cube spawned at {red_cube_pos}, ID: {cube_id}")

        avoid_points = [(red_cube_pos[0], red_cube_pos[1]),
                         (DESIRED_PLACE_POSITION[0], DESIRED_PLACE_POSITION[1])]
        clutter_colors = [name for name in config.COLORS_RGBA if name != 'red']
        num_obstacles = random.randint(*config.NUM_CLUTTER_OBJECTS_RANGE)

        for i in range(num_obstacles):
            xy = _random_clutter_xy(avoid_points, config.CLUTTER_MIN_SEPARATION)
            if xy is None:
                logger.warning("[SETUP] Could not find a clear spot for another clutter object; skipping")
                continue
            x, y = xy
            color_name = clutter_colors[i % len(clutter_colors)]
            obstacle_id = create_colored_cube([x, y, 0.7], config.COLORS_RGBA[color_name], size=config.CUBE_SIZE)
            cube_ids.append(obstacle_id)
            avoid_points.append((x, y))
            logger.debug(f"[DEBUG] {color_name} clutter cube spawned at [{x:.3f}, {y:.3f}], ID: {obstacle_id}")

        logger.info(f"[SETUP] Spawned {len(cube_ids) - 1} clutter obstacles around the red target cube")

    return robot_id, cube_ids, table_id, plane_id


def locate_object_by_color(color_name, ignore_body_ids=frozenset()):
    """
    Vision-based localization: capture a frame, segment it, and return the
    detection for `color_name` (or None if not currently visible).

    Returns:
        {"position": [x, y, z], "body_id": int, "pixel": (x, y)} or None
    """
    frame = capture_camera_image()
    detections = find_objects_by_segmentation(frame, ignore_body_ids=ignore_body_ids)
    return detections.get(color_name)


def make_vision_locator(color_name, ignore_body_ids=frozenset()):
    """Returns a zero-arg callable that re-runs vision detection for `color_name`."""
    def locate():
        detection = locate_object_by_color(color_name, ignore_body_ids=ignore_body_ids)
        return detection["position"] if detection else None
    return locate


def hold_initial_pose(robot_id):
    """
    Set the robot to a safe initial pose and hold it.
    """
    num_joints = p.getNumJoints(robot_id)

    for i in range(min(num_joints, len(config.HOME_POSITION))):
        p.resetJointState(robot_id, i, config.HOME_POSITION[i])
        p.setJointMotorControl2(
            robot_id,
            i,
            p.POSITION_CONTROL,
            targetPosition=config.HOME_POSITION[i],
            force=config.JOINT_FORCE
        )

    wait_for_settle(200)


def return_to_home(robot_id):
    """
    Smoothly return the robot to its home/initial pose.
    """
    logger.info(f"\n{'='*60}")
    logger.info("[ACTION] RETURNING TO HOME POSITION")
    logger.info(f"{'='*60}")

    num_joints = p.getNumJoints(robot_id)

    # Get current joint positions
    current_positions = []
    for i in range(min(num_joints, len(config.HOME_POSITION))):
        joint_state = p.getJointState(robot_id, i)
        current_positions.append(joint_state[0])

    # Smoothly interpolate to home position
    num_steps = 60
    for step in range(num_steps + 1):
        t = step / num_steps
        # Smooth ease-in-out interpolation
        t_smooth = t * t * (3 - 2 * t)

        for i in range(min(num_joints, len(config.HOME_POSITION))):
            target = current_positions[i] + t_smooth * (config.HOME_POSITION[i] - current_positions[i])
            p.setJointMotorControl2(
                robot_id,
                i,
                p.POSITION_CONTROL,
                targetPosition=target,
                force=config.JOINT_FORCE
            )

        for _ in range(4):
            p.stepSimulation()
            time.sleep(1./240.)

    wait_for_settle(100)
    logger.info("[SUCCESS] Robot returned to home position!")


def move_joints(robot_id, q, steps=240):
    """
    Move robot joints to target positions.
    """
    if q is None:
        logger.error("[ERROR] Invalid joint positions (None)")
        return False

    num_joints = p.getNumJoints(robot_id)
    for i in range(num_joints):
        target = q[i] if i < len(q) else 0.0
        p.setJointMotorControl2(
            robot_id,
            i,
            p.POSITION_CONTROL,
            targetPosition=target,
            force=config.JOINT_FORCE
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
        logger.error(f"[ERROR] IK failed for position {pos}")
        return False

    return move_joints(robot_id, q, steps=steps)


def follow_path(robot_id, ee_link, path, orn=None, num_steps=5):
    """
    Move the end-effector through a sequence of via-points (e.g. from the
    RRT planner), smoothly interpolating each consecutive segment.
    """
    if orn is None:
        orn = p.getQuaternionFromEuler([np.pi, 0, 0])

    for i in range(len(path) - 1):
        waypoints = interpolate_positions(path[i], path[i + 1], num_steps)
        for waypoint in waypoints[1:]:
            success = go_to_pose(robot_id, ee_link, waypoint, orn, steps=120)
            if not success:
                return False

    return True


def go_to_pose_smooth(robot_id, ee_link, target_pos, orn=None, num_steps=5):
    """
    Move the robot end-effector to a target position with smooth interpolation.
    """
    current_pos = get_current_ee_position(robot_id, ee_link)
    return follow_path(robot_id, ee_link, [current_pos, target_pos], orn=orn, num_steps=num_steps)


def move_ee_to_object(robot_id, ee_link, obj_id, height_offset=0.03, max_iterations=5, locate_fn=None):
    """
    Move the end-effector directly above and then down to the object.

    If `locate_fn` is given, it's called each iteration to re-estimate the
    object's position (closed-loop visual servoing via fresh camera +
    segmentation detection). Otherwise falls back to PyBullet ground truth.
    Also falls back to ground truth for any iteration where `locate_fn`
    fails to detect the object (e.g. occluded by the gripper).
    """
    for iteration in range(max_iterations):
        obj_pos = locate_fn() if locate_fn is not None else None
        source = "VISION"
        if obj_pos is None:
            obj_pos = list(p.getBasePositionAndOrientation(obj_id)[0])
            source = "SIM (fallback)"

        target_pos = [obj_pos[0], obj_pos[1], obj_pos[2] + height_offset]

        logger.info(f"[INFO] Iteration {iteration + 1}: Moving EE to {target_pos} (source: {source})")

        success = go_to_pose(robot_id, ee_link, target_pos, steps=200)
        if not success:
            logger.warning(f"[WARNING] IK failed for position {target_pos}")
            continue

        wait_for_settle(100)

        ee_state = p.getLinkState(robot_id, ee_link)
        ee_pos = ee_state[4]

        horizontal_dist = np.sqrt(
            (ee_pos[0] - obj_pos[0])**2 +
            (ee_pos[1] - obj_pos[1])**2
        )

        logger.info(f"[INFO] EE position: {ee_pos}")
        logger.info(f"[INFO] Object position ({source}): {obj_pos}")
        logger.info(f"[INFO] Horizontal distance: {horizontal_dist:.4f}m")

        if horizontal_dist < config.ALIGNMENT_TOLERANCE:
            logger.info("[INFO] End-effector aligned with object!")
            return True

    logger.warning("[WARNING] Could not perfectly align with object, proceeding anyway...")
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
    logger.info(f"[INFO] Distance between EE and object: {dist:.4f}m")

    if dist > config.GRASP_MAX_DISTANCE:
        logger.error(f"[ERROR] Object too far from end-effector ({dist:.3f}m)")
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

    logger.info(f"[INFO] Created grasp constraint: {cid}")
    logger.info(f"[INFO] Object attached {z_offset:.3f}m below end-effector")

    return cid


def release_grasp(constraint_id):
    """
    Release the grasped object.
    """
    if constraint_id is not None:
        p.removeConstraint(constraint_id)
        logger.info(f"[INFO] Released grasp constraint: {constraint_id}")


def gentle_place(robot_id, ee_link, obj_id, target_xy, table_height, grasp_constraint):
    """
    Gently place an object on the table surface.
    Lowers the object until it's just above the table, then releases.
    """
    ee_state = p.getLinkState(robot_id, ee_link)
    ee_pos = ee_state[4]
    obj_pos, _ = p.getBasePositionAndOrientation(obj_id)

    ee_to_obj_offset = ee_pos[2] - obj_pos[2]
    target_obj_z = table_height + config.CUBE_HALF_SIZE + 0.005
    target_ee_z = target_obj_z + ee_to_obj_offset

    logger.info(f"[INFO] Gentle place calculation:")
    logger.info(f"[INFO]   Table height: {table_height}")
    logger.info(f"[INFO]   Target object Z: {target_obj_z}")
    logger.info(f"[INFO]   EE to object offset: {ee_to_obj_offset}")
    logger.info(f"[INFO]   Target EE Z: {target_ee_z}")

    above_target = [target_xy[0], target_xy[1], target_ee_z + 0.10]
    logger.info(f"[INFO] Moving above target: {above_target}")
    go_to_pose(robot_id, ee_link, above_target, steps=200)
    wait_for_settle(50)

    current_z = target_ee_z + 0.10

    while current_z > target_ee_z:
        current_z = max(current_z - config.DESCENT_STEP, target_ee_z)
        descent_pos = [target_xy[0], target_xy[1], current_z]
        logger.info(f"[INFO] Descending to: {descent_pos}")
        go_to_pose(robot_id, ee_link, descent_pos, steps=100)
        wait_for_settle(30)

        obj_pos_now, _ = p.getBasePositionAndOrientation(obj_id)
        if obj_pos_now[2] < table_height + config.CUBE_HALF_SIZE + 0.02:
            logger.info(f"[INFO] Object near table surface at z={obj_pos_now[2]:.4f}")
            break

    final_pos = [target_xy[0], target_xy[1], target_ee_z]
    logger.info(f"[INFO] Final descent to: {final_pos}")
    go_to_pose(robot_id, ee_link, final_pos, steps=150)
    wait_for_settle(50)

    obj_pos_final, _ = p.getBasePositionAndOrientation(obj_id)
    logger.info(f"[INFO] Object position before release: {obj_pos_final}")
    logger.info(f"[INFO] Height above table: {obj_pos_final[2] - table_height:.4f}m")

    logger.info("[INFO] Releasing object gently...")
    release_grasp(grasp_constraint)

    wait_for_settle(150)

    obj_pos_settled, _ = p.getBasePositionAndOrientation(obj_id)
    logger.info(f"[INFO] Object settled at: {obj_pos_settled}")

    drift = np.sqrt(
        (obj_pos_settled[0] - target_xy[0])**2 +
        (obj_pos_settled[1] - target_xy[1])**2
    )
    logger.info(f"[INFO] Horizontal drift after release: {drift:.4f}m")

    return True


def move_obstacle_to_clear_position(robot_id, ee_link, obstacle_id, clear_position):
    """
    Move the obstacle object to a clear position.
    This is the core function that actually moves the obstacle.
    """
    logger.info(f"\n{'='*60}")
    logger.info(f"[ACTION] MOVING OBSTACLE (ID: {obstacle_id}) OUT OF THE WAY")
    logger.info(f"{'='*60}")

    # Get current obstacle position
    obs_pos, _ = p.getBasePositionAndOrientation(obstacle_id)
    obs_pos = list(obs_pos)
    logger.info(f"[INFO] Obstacle current position: {obs_pos}")
    logger.info(f"[INFO] Target clear position: {clear_position}")

    # Step 1: Approach from above
    approach = [obs_pos[0], obs_pos[1], obs_pos[2] + 0.15]
    logger.info(f"[INFO] Step 1: Approaching obstacle at {approach}")
    go_to_pose_smooth(robot_id, ee_link, approach)
    wait_for_settle(50)

    # Step 2: Align and descend to obstacle
    logger.info("[INFO] Step 2: Aligning with obstacle...")
    move_ee_to_object(robot_id, ee_link, obstacle_id, height_offset=0.03)

    # Step 3: Grasp the obstacle
    logger.info("[INFO] Step 3: Grasping obstacle...")
    grasp_cid = attach_object_to_ee(robot_id, ee_link, obstacle_id)

    if grasp_cid is None:
        logger.error("[ERROR] Failed to grasp obstacle!")
        go_to_pose(robot_id, ee_link, approach, steps=150)
        return False

    wait_for_settle(50)

    # Step 4: Lift the obstacle
    lift = [obs_pos[0], obs_pos[1], obs_pos[2] + 0.20]
    logger.info(f"[INFO] Step 4: Lifting obstacle to {lift}")
    go_to_pose(robot_id, ee_link, lift, steps=180)
    wait_for_settle(50)

    # Step 5: Transport to clear position
    transport = [clear_position[0], clear_position[1], config.TABLE_HEIGHT + 0.20]
    logger.info(f"[INFO] Step 5: Transporting obstacle to {transport}")
    go_to_pose_smooth(robot_id, ee_link, transport)
    wait_for_settle(50)

    # Step 6: Gently place the obstacle
    logger.info("[INFO] Step 6: Gently placing obstacle...")
    gentle_place(robot_id, ee_link, obstacle_id, [clear_position[0], clear_position[1]], config.TABLE_HEIGHT, grasp_cid)

    # Step 7: Retreat
    retreat = [clear_position[0], clear_position[1], config.TABLE_HEIGHT + 0.15]
    logger.info(f"[INFO] Step 7: Retreating to {retreat}")
    go_to_pose(robot_id, ee_link, retreat, steps=180)
    wait_for_settle(50)

    # Verify obstacle was moved
    new_pos, _ = p.getBasePositionAndOrientation(obstacle_id)
    logger.info(f"\n[SUCCESS] Obstacle moved!")
    logger.info(f"[INFO]   From: {obs_pos}")
    logger.info(f"[INFO]   To: {new_pos}")
    logger.info(f"{'='*60}\n")

    return True


def compute_clear_position(target_position):
    """
    A position `OBSTACLE_CLEAR_OFFSET` away from `target_position`, clipped
    to stay within the table bounds. Shared by the single-obstacle path
    (check_and_clear_obstacle) and the general clutter path (clear_target_if_occupied).
    """
    clear_position = [
        target_position[0] + config.OBSTACLE_CLEAR_OFFSET[0],
        target_position[1] + config.OBSTACLE_CLEAR_OFFSET[1],
        config.TABLE_HEIGHT
    ]
    clear_position[0] = np.clip(clear_position[0], config.TABLE_BOUNDS['x_min'], config.TABLE_BOUNDS['x_max'])
    clear_position[1] = np.clip(clear_position[1], config.TABLE_BOUNDS['y_min'], config.TABLE_BOUNDS['y_max'])
    return clear_position


def check_and_clear_obstacle(robot_id, ee_link, target_position, red_cube_id, obstacle_id):
    """
    Check if there's an obstacle at the target position and move it if needed.
    Uses the known obstacle_id directly instead of searching.

    Obstacle handling stays on PyBullet ground truth rather than vision --
    it's a separate concern from vision-guided target-object localization.
    """
    logger.info(f"\n{'='*60}")
    logger.info(f"[CHECK] IS THERE AN OBSTACLE AT TARGET POSITION?")
    logger.info(f"[INFO] Target position: {target_position}")
    logger.info(f"[INFO] Red cube ID (to place): {red_cube_id}")
    logger.info(f"[INFO] Known obstacle ID: {obstacle_id}")
    logger.info(f"{'='*60}")

    if obstacle_id is None:
        logger.info("[INFO] No obstacle was created. Target is clear!")
        return True

    # Get obstacle position
    obs_pos, _ = p.getBasePositionAndOrientation(obstacle_id)
    obs_pos = list(obs_pos)
    logger.info(f"[INFO] Obstacle position: {obs_pos}")

    # Calculate distance from target
    dist = np.sqrt(
        (target_position[0] - obs_pos[0])**2 +
        (target_position[1] - obs_pos[1])**2
    )
    logger.info(f"[INFO] Distance from target to obstacle: {dist:.4f}m")

    # Check if obstacle is blocking
    if dist > config.OBSTACLE_BLOCK_DISTANCE:
        logger.info(f"[INFO] Obstacle is NOT blocking target (distance > {config.OBSTACLE_BLOCK_DISTANCE}m)")
        return True

    logger.warning(f"\n[ALERT] OBSTACLE IS BLOCKING TARGET POSITION!")
    logger.info(f"[INFO] Must move obstacle before placing red cube.")

    clear_position = compute_clear_position(target_position)
    logger.info(f"[INFO] Will move obstacle to: {clear_position}")

    # Move the obstacle
    success = move_obstacle_to_clear_position(robot_id, ee_link, obstacle_id, clear_position)

    return success


def detect_obstacles(ignore_body_ids, exclude_color='red'):
    """
    Vision-based obstacle detection: one camera capture + segmentation,
    turned into Obstacle instances for every detected object other than
    `exclude_color` (the pick target). Called fresh before each planning
    step so obstacle positions reflect the current scene state.
    """
    frame = capture_camera_image()
    detections = find_objects_by_segmentation(frame, ignore_body_ids=ignore_body_ids)
    return [
        Obstacle(position=info["position"], radius=config.OBSTACLE_COLLISION_RADIUS, body_id=info["body_id"])
        for color, info in detections.items() if color != exclude_color
    ]


def clear_target_if_occupied(robot_id, ee_link, target_position, obstacles):
    """
    Generalizes check_and_clear_obstacle from "one known obstacle_id" to "any
    currently-detected obstacle sitting on the target spot" -- relocates the
    first one found within OBSTACLE_BLOCK_DISTANCE of target_position.
    """
    for obs in obstacles:
        dist = np.sqrt(
            (target_position[0] - obs.position[0])**2 +
            (target_position[1] - obs.position[1])**2
        )
        if dist <= config.OBSTACLE_BLOCK_DISTANCE:
            logger.warning(f"\n[ALERT] Obstacle {obs.body_id} is sitting on the target spot (distance {dist:.4f}m); relocating")
            clear_position = compute_clear_position(target_position)
            move_obstacle_to_clear_position(robot_id, ee_link, obs.body_id, clear_position)
            return True

    logger.info("[INFO] Target spot is clear of clutter.")
    return False


def plan_and_execute_to(robot_id, ee_link, goal_pos, obstacles, orn=None):
    """
    Plan a collision-aware path from the current EE position to goal_pos
    around `obstacles` and execute it. Falls back to relocating the single
    nearest blocking obstacle and retrying once, then to a direct
    best-effort move, so the demo never simply halts.
    """
    start = get_current_ee_position(robot_id, ee_link)
    path = plan_path(
        start, goal_pos, obstacles, config.PLANNING_BOUNDS,
        clearance=config.PLANNER_CLEARANCE, step_size=config.PLANNER_STEP_SIZE,
        max_iterations=config.PLANNER_MAX_ITERATIONS, goal_bias=config.PLANNER_GOAL_BIAS,
    )

    if path is not None:
        path = shortcut_path(path, obstacles, clearance=config.PLANNER_CLEARANCE)
        logger.info(f"[PLANNER] Found collision-free path with {len(path)} waypoints around {len(obstacles)} obstacle(s)")
        return follow_path(robot_id, ee_link, path, orn=orn)

    logger.warning("[PLANNER] No collision-free path found; relocating nearest blocking obstacle")
    blocker = find_blocking_obstacle(start, goal_pos, obstacles)
    if blocker is not None:
        clear_position = compute_clear_position(blocker.position)
        move_obstacle_to_clear_position(robot_id, ee_link, blocker.body_id, clear_position)

        remaining = [o for o in obstacles if o.body_id != blocker.body_id]
        path = plan_path(
            start, goal_pos, remaining, config.PLANNING_BOUNDS,
            clearance=config.PLANNER_CLEARANCE, step_size=config.PLANNER_STEP_SIZE,
            max_iterations=config.PLANNER_MAX_ITERATIONS, goal_bias=config.PLANNER_GOAL_BIAS,
        )
        if path is not None:
            path = shortcut_path(path, remaining, clearance=config.PLANNER_CLEARANCE)
            logger.info(f"[PLANNER] Path found after relocating obstacle {blocker.body_id} ({len(path)} waypoints)")
            return follow_path(robot_id, ee_link, path, orn=orn)

    logger.error("[PLANNER] Still no path after fallback; moving directly (best-effort)")
    return go_to_pose_smooth(robot_id, ee_link, goal_pos, orn=orn)


def pick_object(robot_id, ee_link, obj_id, initial_estimate=None, locate_fn=None, use_smooth=True, reach_fn=None):
    """
    Pick up an object and return the grasp constraint.

    `initial_estimate` (if given) is a vision-estimated [x, y, z] used for the
    approach waypoint instead of ground truth. `locate_fn`, if given, is
    threaded into move_ee_to_object for closed-loop visual re-alignment.
    """
    if initial_estimate is not None:
        obj_pos = list(initial_estimate)
    else:
        obj_pos = list(p.getBasePositionAndOrientation(obj_id)[0])

    logger.info(f"\n{'='*60}")
    logger.info(f"[ACTION] PICKING UP OBJECT (ID: {obj_id})")
    logger.info(f"[INFO] Object position: {obj_pos}")
    logger.info(f"{'='*60}")

    # Approach from above (planner-driven if reach_fn given, otherwise the usual straight-line move)
    approach = [obj_pos[0], obj_pos[1], obj_pos[2] + 0.15]
    logger.info(f"[INFO] Approaching at {approach}")
    if reach_fn is not None:
        reach_fn(approach)
    elif use_smooth:
        go_to_pose_smooth(robot_id, ee_link, approach)
    else:
        go_to_pose(robot_id, ee_link, approach)
    wait_for_settle(50)

    # Align and descend (closed-loop if locate_fn provided)
    logger.info("[INFO] Aligning with object...")
    move_ee_to_object(robot_id, ee_link, obj_id, height_offset=0.03, locate_fn=locate_fn)

    # Grasp
    logger.info("[INFO] Grasping object...")
    grasp_constraint = attach_object_to_ee(robot_id, ee_link, obj_id)

    if grasp_constraint is None:
        logger.error("[ERROR] Failed to grasp object!")
        return None

    wait_for_settle(50)

    # Lift
    obj_pos_new, _ = p.getBasePositionAndOrientation(obj_id)
    lift = [obj_pos_new[0], obj_pos_new[1], obj_pos[2] + 0.25]
    logger.info(f"[INFO] Lifting to {lift}")
    go_to_pose(robot_id, ee_link, lift, steps=200)
    wait_for_settle(50)

    logger.info("[SUCCESS] Object picked up!")
    return grasp_constraint


def place_object(robot_id, ee_link, obj_id, place_pos, grasp_constraint, use_smooth=True, transport_fn=None):
    """
    Place an object at the target position.

    `transport_fn`, if given, replaces the usual straight-line transport
    move with a planner-driven one (see plan_and_execute_to).
    """
    logger.info(f"\n{'='*60}")
    logger.info(f"[ACTION] PLACING OBJECT AT TARGET")
    logger.info(f"[INFO] Target position: {place_pos}")
    logger.info(f"{'='*60}")

    # Transport to above target
    transport = [place_pos[0], place_pos[1], config.TABLE_HEIGHT + 0.25]
    logger.info(f"[INFO] Transporting to {transport}")
    if transport_fn is not None:
        transport_fn(transport)
    elif use_smooth:
        go_to_pose_smooth(robot_id, ee_link, transport)
    else:
        go_to_pose(robot_id, ee_link, transport)
    wait_for_settle(50)

    # Gently place
    logger.info("[INFO] Gently placing object...")
    gentle_place(robot_id, ee_link, obj_id, [place_pos[0], place_pos[1]], config.TABLE_HEIGHT, grasp_constraint)

    # Retreat
    retreat = [place_pos[0], place_pos[1], config.TABLE_HEIGHT + 0.15]
    logger.info(f"[INFO] Retreating to {retreat}")
    go_to_pose(robot_id, ee_link, retreat, steps=200)
    wait_for_settle(50)

    logger.info("[SUCCESS] Object placed!")
    return True


def run_single_mode(robot_id, cube_ids, table_id, plane_id, ee_link, obstacle_id):
    """
    Run single object pick-and-place demo.

    Target localization is vision-driven: the red cube's position is
    estimated via camera capture + segmentation-based detection before any
    ground-truth position is consulted. Ground truth is logged alongside
    for a concrete vision-accuracy comparison, and used as a fallback if
    detection fails.
    """
    red_cube_id = cube_ids[0]
    ignore_ids = {plane_id, table_id, robot_id}

    vision_detection = locate_object_by_color('red', ignore_body_ids=ignore_ids)
    ground_truth_pos, _ = p.getBasePositionAndOrientation(red_cube_id)

    if vision_detection is not None:
        vision_pos = vision_detection["position"]
        error_cm = 100 * np.linalg.norm(np.array(vision_pos) - np.array(ground_truth_pos))
        logger.info(f"[VISION] Estimated red cube position: {[round(v, 3) for v in vision_pos]}")
        logger.info(f"[SIM]    Actual red cube position:    {[round(v, 3) for v in ground_truth_pos]}")
        logger.info(f"[VISION] Localization error: {error_cm:.2f} cm")
    else:
        vision_pos = None
        logger.warning("[VISION] Could not visually detect the red cube; falling back to simulation ground truth")

    place_pos = DESIRED_PLACE_POSITION.copy()
    place_pos[2] = config.TABLE_HEIGHT

    logger.info(f"\n{'#'*70}")
    logger.info(f"[TASK] PICK AND PLACE DEMONSTRATION")
    logger.info(f"{'#'*70}")
    logger.info(f"[INFO] Red cube (ID: {red_cube_id}) is at: {list(ground_truth_pos)}")
    if obstacle_id is not None:
        obs_pos, _ = p.getBasePositionAndOrientation(obstacle_id)
        logger.info(f"[INFO] Yellow obstacle (ID: {obstacle_id}) is at: {obs_pos}")
    logger.info(f"[INFO] Target place position: {place_pos}")
    logger.info(f"[INFO] ")
    logger.info(f"[INFO] SEQUENCE:")
    logger.info(f"[INFO]   1. Check if obstacle is blocking target")
    logger.info(f"[INFO]   2. If yes, move obstacle out of the way")
    logger.info(f"[INFO]   3. Vision-detect and pick up red cube")
    logger.info(f"[INFO]   4. Place red cube at target")
    logger.info(f"[INFO]   5. Return to home position")
    logger.info(f"{'#'*70}\n")

    time.sleep(2)

    # PHASE 1: Clear obstacle if needed
    logger.info("\n" + "="*70)
    logger.info("PHASE 1: CLEAR OBSTACLE IF BLOCKING")
    logger.info("="*70)
    check_and_clear_obstacle(robot_id, ee_link, place_pos, red_cube_id, obstacle_id)

    # PHASE 2: Pick up the red cube (vision-guided)
    logger.info("\n" + "="*70)
    logger.info("PHASE 2: PICK UP RED CUBE")
    logger.info("="*70)
    grasp_constraint = pick_object(
        robot_id, ee_link, red_cube_id,
        initial_estimate=vision_pos,
        locate_fn=make_vision_locator('red', ignore_body_ids=ignore_ids),
        use_smooth=True,
    )

    if grasp_constraint is None:
        logger.error("[FAILED] Could not pick up red cube!")
        return_to_home(robot_id)
        return False

    # PHASE 3: Place the red cube
    logger.info("\n" + "="*70)
    logger.info("PHASE 3: PLACE RED CUBE AT TARGET")
    logger.info("="*70)
    success = place_object(robot_id, ee_link, red_cube_id, place_pos, grasp_constraint, use_smooth=True)

    # PHASE 4: Return to home position
    logger.info("\n" + "="*70)
    logger.info("PHASE 4: RETURN TO HOME POSITION")
    logger.info("="*70)
    return_to_home(robot_id)

    if success:
        logger.info(f"\n{'#'*70}")
        logger.info("[SUCCESS] TASK COMPLETE!")
        final_pos, _ = p.getBasePositionAndOrientation(red_cube_id)
        logger.info(f"[INFO] Red cube final position: {final_pos}")
        logger.info(f"{'#'*70}\n")
    else:
        logger.error("[FAILED] Task failed!")

    return success


def run_multi_mode(robot_id, cube_ids, table_id, plane_id, ee_link):
    """
    Run multi-object pick-and-place demo.

    Iterates by color (not by pre-known cube_id) -- each cube's position and
    identity are resolved via vision (segmentation + color classification)
    before it's picked, exactly as run_single_mode does for the red cube.
    """
    ignore_ids = {plane_id, table_id, robot_id}
    color_order = ['red', 'green', 'blue']
    place_positions = [
        [0.35, -0.15, config.TABLE_HEIGHT],
        [0.35, 0.0, config.TABLE_HEIGHT],
        [0.35, 0.15, config.TABLE_HEIGHT],
    ]

    logger.info(f"[INFO] Scene contains {len(cube_ids)} cubes to process")

    for i, color_name in enumerate(color_order):
        logger.info(f"\n[INFO] Processing {color_name} cube ({i + 1}/{len(color_order)})...")

        detection = locate_object_by_color(color_name, ignore_body_ids=ignore_ids)
        if detection is None:
            logger.warning(f"[WARNING] Could not visually detect the {color_name} cube; skipping")
            continue

        place_pos = place_positions[i % len(place_positions)]
        grasp = pick_object(
            robot_id, ee_link, detection["body_id"],
            initial_estimate=detection["position"],
            locate_fn=make_vision_locator(color_name, ignore_body_ids=ignore_ids),
        )
        if grasp:
            place_object(robot_id, ee_link, detection["body_id"], place_pos, grasp)

    # Return to home after all objects placed
    logger.info("\n" + "="*70)
    logger.info("RETURNING TO HOME POSITION")
    logger.info("="*70)
    return_to_home(robot_id)

    logger.info("\n[SUCCESS] Multi-object pick-and-place complete!")


def run_sort_mode(robot_id, cube_ids, table_id, plane_id, ee_link):
    """
    Run object sorting demo - sort by color, vision-guided (see run_multi_mode).
    """
    bin_base_position = [0.35, 0.0, config.TABLE_HEIGHT]
    color_order = ['red', 'green', 'blue']
    sort_bin_positions = create_sorting_bins(bin_base_position, num_bins=len(color_order), bin_spacing=0.15)
    sort_bins = dict(zip(color_order, sort_bin_positions))

    logger.info(f"[INFO] Scene contains {len(cube_ids)} cubes to sort")
    ignore_ids = {plane_id, table_id, robot_id}

    for color_name in color_order:
        place_pos = sort_bins[color_name]

        detection = locate_object_by_color(color_name, ignore_body_ids=ignore_ids)
        if detection is None:
            logger.warning(f"[WARNING] Could not visually detect the {color_name} cube; skipping")
            continue

        logger.info(f"\n[INFO] Sorting {color_name} cube to {place_pos}")

        grasp = pick_object(
            robot_id, ee_link, detection["body_id"],
            initial_estimate=detection["position"],
            locate_fn=make_vision_locator(color_name, ignore_body_ids=ignore_ids),
        )
        if grasp:
            place_object(robot_id, ee_link, detection["body_id"], place_pos, grasp)

    # Return to home after sorting
    logger.info("\n" + "="*70)
    logger.info("RETURNING TO HOME POSITION")
    logger.info("="*70)
    return_to_home(robot_id)

    logger.info("\n[SUCCESS] Object sorting complete!")


def run_clutter_mode(robot_id, cube_ids, table_id, plane_id, ee_link):
    """
    Run the collision-aware clutter-navigation demo.

    Unlike run_single_mode's scripted single-obstacle handling, this mode
    detects every object on the table via vision and plans a genuine
    collision-free path (see src.control.motion_planner) around whichever
    ones are in the way -- both en route to the red target cube and while
    transporting it to the place position. Falls back to physically
    relocating the nearest blocking obstacle only if no path is found
    (see plan_and_execute_to).
    """
    red_cube_id = cube_ids[0]
    ignore_ids = {plane_id, table_id, robot_id}

    place_pos = DESIRED_PLACE_POSITION.copy()
    place_pos[2] = config.TABLE_HEIGHT

    vision_detection = locate_object_by_color('red', ignore_body_ids=ignore_ids)
    ground_truth_pos, _ = p.getBasePositionAndOrientation(red_cube_id)

    if vision_detection is not None:
        vision_pos = vision_detection["position"]
        error_cm = 100 * np.linalg.norm(np.array(vision_pos) - np.array(ground_truth_pos))
        logger.info(f"[VISION] Estimated red cube position: {[round(v, 3) for v in vision_pos]}")
        logger.info(f"[VISION] Localization error: {error_cm:.2f} cm")
    else:
        vision_pos = None
        logger.warning("[VISION] Could not visually detect the red cube; falling back to simulation ground truth")

    logger.info(f"\n{'#'*70}")
    logger.info(f"[TASK] CLUTTER-NAVIGATION PICK AND PLACE")
    logger.info(f"{'#'*70}")
    logger.info(f"[INFO] Scene contains {len(cube_ids) - 1} clutter obstacles")
    logger.info(f"[INFO] Target place position: {place_pos}")
    logger.info(f"{'#'*70}\n")

    time.sleep(2)

    # PHASE 1: clear the placement spot if something happens to be sitting on it
    logger.info("\n" + "="*70)
    logger.info("PHASE 1: CLEAR TARGET SPOT IF OCCUPIED")
    logger.info("="*70)
    clear_target_if_occupied(robot_id, ee_link, place_pos, detect_obstacles(ignore_ids))

    # PHASE 2: plan a collision-free path to the red cube and pick it up
    logger.info("\n" + "="*70)
    logger.info("PHASE 2: PLAN + PICK UP RED CUBE")
    logger.info("="*70)
    grasp_constraint = pick_object(
        robot_id, ee_link, red_cube_id,
        initial_estimate=vision_pos,
        locate_fn=make_vision_locator('red', ignore_body_ids=ignore_ids),
        reach_fn=lambda pos: plan_and_execute_to(robot_id, ee_link, pos, detect_obstacles(ignore_ids)),
    )

    if grasp_constraint is None:
        logger.error("[FAILED] Could not pick up red cube!")
        return_to_home(robot_id)
        return False

    # PHASE 3: plan a collision-free path to the place position and place it
    logger.info("\n" + "="*70)
    logger.info("PHASE 3: PLAN + PLACE RED CUBE AT TARGET")
    logger.info("="*70)
    success = place_object(
        robot_id, ee_link, red_cube_id, place_pos, grasp_constraint,
        transport_fn=lambda pos: plan_and_execute_to(robot_id, ee_link, pos, detect_obstacles(ignore_ids)),
    )

    # PHASE 4: return home
    logger.info("\n" + "="*70)
    logger.info("PHASE 4: RETURN TO HOME POSITION")
    logger.info("="*70)
    return_to_home(robot_id)

    if success:
        logger.info(f"\n{'#'*70}")
        logger.info("[SUCCESS] TASK COMPLETE!")
        final_pos, _ = p.getBasePositionAndOrientation(red_cube_id)
        logger.info(f"[INFO] Red cube final position: {final_pos}")
        logger.info(f"{'#'*70}\n")
    else:
        logger.error("[FAILED] Task failed!")

    return success


def main():
    global OBSTACLE_ID

    parser = argparse.ArgumentParser(description='Vision-guided pick-and-place demo')
    parser.add_argument('--mode', type=str, default='single', choices=['single', 'multi', 'sort', 'clutter'],
                        help='Demo mode: single, multi, sort, or clutter (collision-aware path planning)')
    parser.add_argument('--no-clutter', action='store_true', help='Disable clutter objects')
    parser.add_argument('--no-obstacle', action='store_true', help='Do not place obstacle at target')
    parser.add_argument('--place-x', type=float, default=None, help='X coordinate for place position')
    parser.add_argument('--place-y', type=float, default=None, help='Y coordinate for place position')
    parser.add_argument('--seed', type=int, default=None, help='Random seed (reproducible clutter layout in --mode clutter)')
    parser.add_argument('--verbose', action='store_true', help='Enable debug-level logging')
    parser.add_argument('--quiet', action='store_true', help='Only log warnings and errors')
    args = parser.parse_args()

    configure_logging(verbose=args.verbose, quiet=args.quiet)

    if args.seed is not None:
        random.seed(args.seed)

    global DESIRED_PLACE_POSITION
    if args.place_x is not None:
        DESIRED_PLACE_POSITION[0] = args.place_x
    if args.place_y is not None:
        DESIRED_PLACE_POSITION[1] = args.place_y

    try:
        logger.info(f"\n{'*'*70}")
        logger.info(f"  VISION-GUIDED PICK-AND-PLACE WITH OBSTACLE CLEARING")
        logger.info(f"{'*'*70}")
        logger.info(f"[INFO] Mode: {args.mode}")
        logger.info(f"[INFO] Target place position: {DESIRED_PLACE_POSITION}")
        logger.info(f"[INFO] Obstacle at target: {not args.no_obstacle}")
        logger.info(f"{'*'*70}\n")

        logger.info("[SETUP] Connecting to PyBullet...")
        physics_client = connect_sim(gui=True)

        logger.info("[SETUP] Loading scene...")
        robot_id, cube_ids, table_id, plane_id = load_scene(mode=args.mode)
        logger.info(f"[SETUP] Robot ID: {robot_id}, Cube IDs: {cube_ids}, Table ID: {table_id}")

        logger.info("[SETUP] Setting initial pose...")
        hold_initial_pose(robot_id)

        # Skip clutter for cleaner demo
        if not args.no_clutter and args.mode == 'single':
            logger.info("[SETUP] Skipping clutter for cleaner obstacle demo...")

        # Create yellow obstacle at target location
        obstacle_id = None
        if not args.no_obstacle and args.mode == 'single':
            logger.info(f"\n[SETUP] Creating YELLOW OBSTACLE at target {DESIRED_PLACE_POSITION}...")
            obstacle_pos = [DESIRED_PLACE_POSITION[0], DESIRED_PLACE_POSITION[1], 0.7]
            obstacle_id = create_colored_cube(obstacle_pos, config.COLORS_RGBA['yellow'], size=config.CUBE_SIZE)
            OBSTACLE_ID = obstacle_id
            logger.info(f"[SETUP] Yellow obstacle created at {obstacle_pos}, ID: {obstacle_id}")

        logger.info("[SETUP] Letting simulation settle...")
        wait_for_settle(300)

        # Print final positions
        logger.info("\n[SETUP] Object positions after settling:")
        for cid in cube_ids:
            pos, _ = p.getBasePositionAndOrientation(cid)
            logger.info(f"[SETUP]   Cube (ID: {cid}): {[round(v, 3) for v in pos]}")
        if obstacle_id is not None:
            pos, _ = p.getBasePositionAndOrientation(obstacle_id)
            logger.info(f"[SETUP]   Yellow obstacle (ID: {obstacle_id}): {[round(v, 3) for v in pos]}")

        ee_link = config.EE_LINK

        # Run the appropriate mode
        if args.mode == 'single':
            run_single_mode(robot_id, cube_ids, table_id, plane_id, ee_link, obstacle_id)
        elif args.mode == 'multi':
            run_multi_mode(robot_id, cube_ids, table_id, plane_id, ee_link)
        elif args.mode == 'sort':
            run_sort_mode(robot_id, cube_ids, table_id, plane_id, ee_link)
        elif args.mode == 'clutter':
            run_clutter_mode(robot_id, cube_ids, table_id, plane_id, ee_link)

        logger.info("\n[INFO] Simulation complete. Press CTRL+C to exit...")
        while True:
            p.stepSimulation()
            time.sleep(1.0 / 240.0)

    except KeyboardInterrupt:
        logger.info("\n[INFO] Simulation terminated by user.")
    except Exception as e:
        logger.error(f"[ERROR] An exception occurred: {e}")
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
