"""
Single source of truth for values that were previously hardcoded and
duplicated (with disagreeing numbers) across camera_sim.py, object_detector.py,
world.py, pick_and_place.py, inverse_kinematics.py, and tests/test_ik.py.
"""
from dataclasses import dataclass, field


# ---- Table ----
TABLE_POSITION = [0.5, 0.0, 0.325]   # basePosition passed to p.loadURDF for table.urdf
TABLE_HEIGHT = 0.65                  # world-frame Z of the table surface
TABLE_BOUNDS = {
    "x_min": 0.30, "x_max": 0.65,
    "y_min": -0.30, "y_max": 0.30,
}

# ---- Robot ----
ROBOT_URDF = "kuka_iiwa/model.urdf"
ROBOT_POSITION = [0, 0, 0]
EE_LINK = 6                          # KUKA IIWA end-effector link index
JOINT_FORCE = 500                    # N*m applied by every POSITION_CONTROL joint command
HOME_POSITION = [0.0, 0.5, 0.0, -1.0, 0.0, 1.0, 0.0]

# ---- Objects ----
CUBE_SIZE = 0.05
CUBE_HALF_SIZE = CUBE_SIZE / 2

COLORS = {
    "red": [255, 0, 0],
    "green": [0, 255, 0],
    "blue": [0, 0, 255],
    "yellow": [255, 255, 0],
    "cyan": [0, 255, 255],
    "magenta": [255, 0, 255],
}
# RGBA (0-1 range) used when spawning cubes in PyBullet, keyed the same as COLORS.
COLORS_RGBA = {
    "red": [1, 0, 0, 1],
    "green": [0, 1, 0, 1],
    "blue": [0, 0, 1, 1],
    "yellow": [1, 1, 0, 1],
    "cyan": [0, 1, 1, 1],
    "magenta": [1, 0, 1, 1],
}

# ---- Grasping / motion thresholds ----
GRASP_MAX_DISTANCE = 0.15      # max EE-to-object distance (m) to accept a grasp
ALIGNMENT_TOLERANCE = 0.03     # horizontal distance (m) considered "aligned" with target
OBSTACLE_BLOCK_DISTANCE = 0.12 # distance (m) within which an obstacle is considered blocking
DESCENT_STEP = 0.02            # m per step during gentle_place descent
OBSTACLE_CLEAR_OFFSET = [0.0, 0.25, 0.0]

# ---- Default demo config ----
DESIRED_PLACE_POSITION = [0.5, 0.0, TABLE_HEIGHT]

# ---- Motion planning (clutter mode) ----
PLANNER_STEP_SIZE = 0.05
PLANNER_MAX_ITERATIONS = 1500
PLANNER_GOAL_BIAS = 0.1
PLANNER_CLEARANCE = 0.03            # safety margin added on top of obstacle radius
OBSTACLE_COLLISION_RADIUS = 0.05    # bounding-sphere radius for a clutter cube
PLANNING_BOUNDS = {
    **TABLE_BOUNDS,
    # Absolute (not TABLE_HEIGHT-derived) because objects physically settle
    # well below TABLE_HEIGHT in this scene (~0.375m, not 0.65m -- TABLE_HEIGHT
    # is used as a Cartesian placement-target constant, not the true table
    # surface height). These bounds must cover the actual altitude range
    # pick_object/place_object operate at: grasp/approach around obj_z+0.15
    # (~0.5-0.55m) up through place_object's transport waypoint at
    # TABLE_HEIGHT + 0.25 (0.9m).
    "z_min": 0.30,
    "z_max": 1.00,
}
NUM_CLUTTER_OBJECTS_RANGE = (3, 5)
CLUTTER_MIN_SEPARATION = 0.12       # min center-to-center spacing between randomly placed clutter


@dataclass
class CameraConfig:
    target_pos: list = field(default_factory=lambda: [0.6, 0, 0.35])
    distance: float = 1.0
    yaw: float = 90
    pitch: float = -45
    roll: float = 0
    fov: float = 60
    aspect: float = 1.0
    near: float = 0.1
    far: float = 3.0
    img_width: int = 256
    img_height: int = 256


CAMERA = CameraConfig()
