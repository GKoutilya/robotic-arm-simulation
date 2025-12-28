import numpy as np
import pybullet as p

def interpolate_positions(start_pos, end_pos, num_steps=10):
    """
    Generate interpolated positions between start and end.
    
    Args:
        start_pos: Starting position [x, y, z]
        end_pos: Ending position [x, y, z]
        num_steps: Number of intermediate steps
    
    Returns:
        List of interpolated positions
    """
    positions = []
    for i in range(num_steps + 1):
        t = i / num_steps
        # Smooth interpolation using ease-in-out
        t_smooth = t * t * (3 - 2 * t)
        pos = [
            start_pos[0] + (end_pos[0] - start_pos[0]) * t_smooth,
            start_pos[1] + (end_pos[1] - start_pos[1]) * t_smooth,
            start_pos[2] + (end_pos[2] - start_pos[2]) * t_smooth
        ]
        positions.append(pos)
    return positions

def plan_arc_trajectory(start_pos, end_pos, arc_height=0.1, num_steps=20):
    """
    Plan an arc trajectory to avoid obstacles.
    
    Args:
        start_pos: Starting position [x, y, z]
        end_pos: Ending position [x, y, z]
        arc_height: Height of the arc above the straight line
        num_steps: Number of waypoints
    
    Returns:
        List of waypoint positions
    """
    positions = []
    mid_z = max(start_pos[2], end_pos[2]) + arc_height
    
    for i in range(num_steps + 1):
        t = i / num_steps
        # Parabolic arc for z-coordinate
        arc_factor = 4 * t * (1 - t)  # Peaks at t=0.5
        
        pos = [
            start_pos[0] + (end_pos[0] - start_pos[0]) * t,
            start_pos[1] + (end_pos[1] - start_pos[1]) * t,
            start_pos[2] + (end_pos[2] - start_pos[2]) * t + arc_height * arc_factor
        ]
        positions.append(pos)
    return positions

def get_current_ee_position(robot_id, ee_link):
    """
    Get the current end-effector position.
    
    Args:
        robot_id: PyBullet robot ID
        ee_link: End-effector link index
    
    Returns:
        Current position [x, y, z]
    """
    state = p.getLinkState(robot_id, ee_link)
    return list(state[4])

def plan_pick_and_place_trajectory(current_pos, target_pos, place_pos, approach_height=0.15, lift_height=0.25):
    """
    Plan a complete pick-and-place trajectory.
    
    Args:
        current_pos: Current end-effector position
        target_pos: Object position to pick
        place_pos: Position to place the object
        approach_height: Height above object for approach
        lift_height: Height to lift object
    
    Returns:
        Dictionary of waypoints for each phase
    """
    waypoints = {
        'approach': [target_pos[0], target_pos[1], target_pos[2] + approach_height],
        'pre_grasp': [target_pos[0], target_pos[1], target_pos[2] + 0.08],
        'grasp': [target_pos[0], target_pos[1], target_pos[2] + 0.03],
        'lift': [target_pos[0], target_pos[1], target_pos[2] + lift_height],
        'transport': [place_pos[0], place_pos[1], place_pos[2] + lift_height],
        'pre_place': [place_pos[0], place_pos[1], place_pos[2] + 0.08],
        'place': [place_pos[0], place_pos[1], place_pos[2] + 0.03],
        'retreat': [place_pos[0], place_pos[1], place_pos[2] + approach_height]
    }
    return waypoints