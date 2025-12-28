import pybullet as p
import numpy as np
import time

def create_colored_cube(position, color_rgba, size=0.05):
    """
    Create a colored cube at the specified position.
    
    Args:
        position: [x, y, z] position
        color_rgba: [r, g, b, a] color (0-1 range)
        size: Size of the cube
    
    Returns:
        Body ID of the created cube
    """
    half_size = size / 2
    collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[half_size, half_size, half_size])
    visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[half_size, half_size, half_size], rgbaColor=color_rgba)
    
    cube_id = p.createMultiBody(
        baseMass=0.1,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=position
    )
    return cube_id

def create_sorting_bins(base_position, num_bins=3, bin_spacing=0.15):
    """
    Create sorting bins for object sorting tasks.
    
    Args:
        base_position: [x, y, z] base position for bins
        num_bins: Number of bins to create
        bin_spacing: Spacing between bins
    
    Returns:
        List of bin positions
    """
    bin_positions = []
    colors = [
        [1, 0, 0, 0.3],  # Red (transparent)
        [0, 1, 0, 0.3],  # Green
        [0, 0, 1, 0.3],  # Blue
    ]
    
    for i in range(num_bins):
        pos = [
            base_position[0],
            base_position[1] + (i - num_bins // 2) * bin_spacing,
            base_position[2]
        ]
        bin_positions.append(pos)
        
        # Create visual marker for bin
        visual_shape = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=0.05,
            length=0.02,
            rgbaColor=colors[i % len(colors)]
        )
        p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=visual_shape,
            basePosition=[pos[0], pos[1], pos[2] + 0.01]
        )
    
    return bin_positions

def get_object_color(obj_id):
    """
    Get the color of an object.
    
    Args:
        obj_id: PyBullet body ID
    
    Returns:
        Color name ('red', 'green', 'blue', 'unknown')
    """
    visual_data = p.getVisualShapeData(obj_id)
    if visual_data:
        rgba = visual_data[0][7]  # RGBA color
        r, g, b = rgba[0], rgba[1], rgba[2]
        
        if r > 0.7 and g < 0.3 and b < 0.3:
            return 'red'
        elif g > 0.7 and r < 0.3 and b < 0.3:
            return 'green'
        elif b > 0.7 and r < 0.3 and g < 0.3:
            return 'blue'
    
    return 'unknown'

def wait_for_settle(steps=100, sleep_time=1./240.):
    """
    Wait for the simulation to settle.
    
    Args:
        steps: Number of simulation steps
        sleep_time: Time between steps
    """
    for _ in range(steps):
        p.stepSimulation()
        time.sleep(sleep_time)

def print_joint_info(robot_id):
    """
    Print information about all joints in the robot.
    
    Args:
        robot_id: PyBullet robot ID
    """
    num_joints = p.getNumJoints(robot_id)
    print(f"\nRobot has {num_joints} joints:")
    print("-" * 60)
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        print(f"Joint {i}: {joint_info[1].decode('utf-8')}")
        print(f"  Type: {joint_info[2]}")
        print(f"  Lower limit: {joint_info[8]}")
        print(f"  Upper limit: {joint_info[9]}")
    print("-" * 60)