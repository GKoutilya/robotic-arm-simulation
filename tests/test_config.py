import numpy as np

from src import config


def test_table_bounds_are_ordered_correctly():
    assert config.TABLE_BOUNDS['x_min'] < config.TABLE_BOUNDS['x_max']
    assert config.TABLE_BOUNDS['y_min'] < config.TABLE_BOUNDS['y_max']


def test_desired_place_position_lies_within_table_bounds():
    x, y, _ = config.DESIRED_PLACE_POSITION
    assert config.TABLE_BOUNDS['x_min'] <= x <= config.TABLE_BOUNDS['x_max']
    assert config.TABLE_BOUNDS['y_min'] <= y <= config.TABLE_BOUNDS['y_max']


def test_cube_half_size_matches_cube_size():
    assert config.CUBE_HALF_SIZE == config.CUBE_SIZE / 2


def test_home_position_has_seven_joint_values():
    assert len(config.HOME_POSITION) == 7


def test_colors_and_colors_rgba_have_matching_keys():
    assert set(config.COLORS.keys()) == set(config.COLORS_RGBA.keys())


def test_colors_rgba_channels_are_normalized():
    for name, rgba in config.COLORS_RGBA.items():
        assert len(rgba) == 4
        assert all(0.0 <= c <= 1.0 for c in rgba), f"{name} channel out of [0,1] range"


def test_grasp_threshold_is_looser_than_alignment_tolerance():
    # The grasp check happens after alignment, so it must tolerate at least
    # the residual error the alignment loop is allowed to leave behind.
    assert config.GRASP_MAX_DISTANCE > config.ALIGNMENT_TOLERANCE


def test_camera_config_has_positive_intrinsics():
    cam = config.CAMERA
    assert cam.fov > 0
    assert cam.aspect > 0
    assert 0 < cam.near < cam.far
    assert cam.img_width > 0 and cam.img_height > 0


def test_planner_clearance_and_radius_are_non_negative():
    assert config.PLANNER_CLEARANCE >= 0
    assert config.OBSTACLE_COLLISION_RADIUS > 0
    assert config.PLANNER_STEP_SIZE > 0
    assert config.PLANNER_MAX_ITERATIONS > 0
    assert 0 <= config.PLANNER_GOAL_BIAS <= 1


def test_planning_bounds_are_ordered_correctly():
    b = config.PLANNING_BOUNDS
    assert b['x_min'] < b['x_max']
    assert b['y_min'] < b['y_max']
    assert b['z_min'] < b['z_max']


def test_planning_bounds_cover_the_altitudes_pick_and_place_actually_use():
    # place_object's transport waypoint is TABLE_HEIGHT + 0.25; the grasp/
    # approach waypoint is roughly (object rest height) + 0.15. Both must
    # fall inside PLANNING_BOUNDS or the RRT's random sampling band won't
    # cover where the arm actually needs to move.
    b = config.PLANNING_BOUNDS
    transport_z = config.TABLE_HEIGHT + 0.25
    assert b['z_min'] <= transport_z <= b['z_max']


def test_clutter_object_count_range_is_valid():
    low, high = config.NUM_CLUTTER_OBJECTS_RANGE
    assert 0 < low <= high


def test_clutter_min_separation_exceeds_obstacle_footprint():
    # otherwise randomly-placed clutter could spawn overlapping each other
    assert config.CLUTTER_MIN_SEPARATION >= config.OBSTACLE_COLLISION_RADIUS
