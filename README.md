# Simulated Robotic Arm with Vision-Guided Object Manipulation

A PyBullet-based simulation of a KUKA IIWA robotic arm performing vision-guided pick-and-place operations with intelligent obstacle avoidance.

![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)
![PyBullet](https://img.shields.io/badge/PyBullet-Physics-green.svg)
![OpenCV](https://img.shields.io/badge/OpenCV-Vision-red.svg)

## Overview

This project demonstrates core robotics concepts including inverse kinematics, motion planning, computer vision, and autonomous decision-making. The robot arm can detect objects using color-based vision, clear obstacles blocking target locations, and perform precise pick-and-place operations.

## Key Features

| Feature | Description |
|---------|-------------|
| **Vision System** | Camera-based object detection using color segmentation |
| **Inverse Kinematics** | Real-time IK solver for 7-DOF robot arm |
| **Obstacle Clearing** | Automatically detects and relocates obstacles blocking target positions |
| **Smooth Motion** | Interpolated trajectories with ease-in-out curves |
| **Gentle Placement** | Objects are lowered carefully to prevent bouncing |
| **Multiple Modes** | Single object, multi-object, and color sorting demos |
| **Home Position** | Robot returns to starting pose after task completion |

## Demo Sequence

```
1. Detect yellow obstacle at target location
2. Pick up and move obstacle aside
3. Pick up red cube from starting position
4. Place red cube at cleared target location
5. Return robot to home position
```

## Installation

### Prerequisites
- Python 3.8 or higher
- pip package manager

### Setup
```bash
# Clone the repository
git clone https://github.com/yourusername/robotic-arm-simulation.git
cd robotic-arm-simulation

# Install dependencies
pip install -r requirements.txt
```

### Dependencies
- `pybullet` - Physics simulation engine
- `numpy` - Numerical computations
- `opencv-python` - Computer vision and image processing

## Usage

### Basic Demo (with obstacle clearing)
```bash
python src/demo/pick_and_place.py
```

### Without Obstacle
```bash
python src/demo/pick_and_place.py --no-obstacle
```

### Multi-Object Mode
```bash
python src/demo/pick_and_place.py --mode multi
```

### Color Sorting Mode
```bash
python src/demo/pick_and_place.py --mode sort
```

### Custom Target Location
```bash
python src/demo/pick_and_place.py --place-x 0.4 --place-y 0.15
```

### Command-Line Arguments

| Argument | Description | Default |
|----------|-------------|---------|
| `--mode` | Demo mode: `single`, `multi`, or `sort` | `single` |
| `--no-obstacle` | Run without obstacle at target | `False` |
| `--no-clutter` | Disable random clutter objects | `False` |
| `--place-x` | X coordinate for target position | `0.5` |
| `--place-y` | Y coordinate for target position | `0.0` |

## Project Structure

```
📁 Simulated Robotic Arm with Vision-Guided Object Manipulation/
├── 📁 assets/
│   ├── 📁 objects/
│   │   └── cube.urdf              # Cube object model
│   └── 📁 table/
│       └── table.urdf             # Table model
├── 📁 src/
│   ├── 📁 camera/
│   │   ├── camera_sim.py          # Simulated RGB-D camera
│   │   └── object_detector.py     # Color-based object detection
│   ├── 📁 control/
│   │   ├── arm_controller.py      # Robot arm control interface
│   │   ├── inverse_kinematics.py  # IK solver using PyBullet
│   │   └── planner.py             # Trajectory planning utilities
│   ├── 📁 demo/
│   │   └── pick_and_place.py      # Main demonstration script ⭐
│   └── 📁 simulation/
│       ├── world.py               # Scene and environment setup
│       └── utils.py               # Helper functions
├── 📁 tests/
│   └── test_ik.py                 # Inverse kinematics tests
├── requirements.txt
└── README.md
```

## Technical Details

### Robot
- **Model**: KUKA IIWA 7-DOF robotic arm
- **End Effector**: Simulated suction gripper (constraint-based grasping)
- **Control**: Position control with 500N force limit

### Vision System
- **Camera**: Simulated RGB-D camera mounted above workspace
- **Detection**: HSV color space thresholding
- **Supported Colors**: Red, Green, Blue, Yellow

### Motion Planning
- **IK Solver**: PyBullet's built-in damped least squares solver
- **Trajectory**: Linear interpolation with smooth ease-in-out curves
- **Collision Avoidance**: Height-based approach and retreat paths

### Coordinate System
- **Origin**: Robot base
- **X-axis**: Forward (toward table)
- **Y-axis**: Left
- **Z-axis**: Up

## Controls

- **Mouse**: Rotate and zoom the 3D view
- **CTRL+C**: Exit simulation

## Sample Output

```
VISION-GUIDED PICK-AND-PLACE WITH OBSTACLE CLEARING

   [INFO] Mode: single
   [INFO] Target place position: [0.5, 0.0, 0.65]
   [INFO] Obstacle at target: True

PHASE 1: CLEAR OBSTACLE IF BLOCKING

   [ALERT] OBSTACLE IS BLOCKING TARGET POSITION!
   [ACTION] MOVING OBSTACLE (ID: 4) OUT OF THE WAY
   [SUCCESS] Obstacle moved!

PHASE 2: PICK UP RED CUBE

   [ACTION] PICKING UP OBJECT (ID: 3)
   [SUCCESS] Object picked up!

PHASE 3: PLACE RED CUBE AT TARGET

   [ACTION] PLACING OBJECT AT TARGET
   [SUCCESS] Object placed!

PHASE 4: RETURN TO HOME POSITION

   [SUCCESS] Robot returned to home position!


[SUCCESS] TASK COMPLETE!

```

## Future Improvements

- [ ] Add ROS/ROS2 integration for real robot deployment
- [ ] Implement path planning with RRT/RRT*
- [ ] Add reinforcement learning for adaptive grasping
- [ ] Support more complex object shapes (cylinders, irregular objects)
- [ ] Implement force/torque feedback for sensitive manipulation
- [ ] Add multi-robot coordination

## License

MIT License - feel free to use this project for learning and portfolio purposes.

## Author

- **Name**: Koutilya Ganapathiraju
- **Email**: gkoutilyaraju@gmail.com
- **LinkedIn**: www.linkedin.com/in/koutilya-ganapathiraju-0a3350182