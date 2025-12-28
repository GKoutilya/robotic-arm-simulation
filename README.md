# Simulated Robotic Arm with Vision-Guided Object Manipulation

A complete physics-based simulation of a 7-DOF robotic arm performing autonomous, vision-guided pick-and-place in a cluttered tabletop environment using a modular perception–planning–control pipeline.

![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)
![PyBullet](https://img.shields.io/badge/PyBullet-Physics-green.svg)
![Computer Vision](https://img.shields.io/badge/Vision-RGB--D-red.svg)

---

## Overview

This project implements an end-to-end robotic manipulation system in simulation. A robotic arm observes a cluttered tabletop scene using simulated RGB, depth, and segmentation cameras, detects a target object using vision, reasons about workspace clutter, and executes a smooth pick-and-place operation using inverse kinematics and physics-aware motion primitives.

The system mirrors the structure of real robotic manipulation stacks used in industry, with a clean separation between **perception**, **planning**, and **control**, making it both technically rigorous and interview-defensible.

---

## System Architecture

The pipeline follows a standard robotics control loop:

### Perception

* Simulated RGB, depth, and segmentation cameras
* Color-based object detection
* Projection from image space to world coordinates

### Planning

* Task-level sequencing (approach → grasp → lift → place → retreat)
* Rule-based clutter handling when the target location is obstructed
* Height-aware motion strategies to avoid collisions without full motion planning

### Control

* Inverse kinematics using PyBullet’s damped least-squares solver
* Smooth joint-space trajectory interpolation
* Position-controlled execution with bounded forces

Each component is modular and independently replaceable, reflecting real robotic system design.

---

## Key Capabilities

| Capability                     | Description                                                           |
| ------------------------------ | --------------------------------------------------------------------- |
| **Vision-Guided Manipulation** | Uses RGB, depth, and segmentation cameras for object localization     |
| **Inverse Kinematics Control** | Real-time IK for a 7-DOF articulated arm                              |
| **Clutter-Aware Execution**    | Detects and relocates blocking objects when necessary                 |
| **Stable Physics Interaction** | Fixed table, stable clutter placement, and controlled object dynamics |
| **Smooth Motion Execution**    | Interpolated joint trajectories for natural arm motion                |
| **Modular Architecture**       | Clean separation of perception, planning, and control logic           |

---

## Demonstrated Behavior

A full task execution proceeds as follows:

```
1. Capture RGB, depth, and segmentation images
2. Detect target object in the scene
3. Estimate target position in world coordinates
4. Check for clutter blocking the target location
5. Relocate blocking objects if present
6. Pick up the target object
7. Place the object at the goal position
8. Return the robot to a home configuration
```

All actions are executed under physics simulation with gravity, contact dynamics, and joint constraints enabled.

---

## Installation

### Prerequisites

* Python 3.8 or newer
* pip

### Setup

```bash
git clone https://github.com/yourusername/simulated-robotic-arm.git
cd simulated-robotic-arm
pip install -r requirements.txt
```

### Dependencies

* `pybullet`
* `numpy`
* `opencv-python`
* `matplotlib`

---

## Running the Simulation

Launch the complete vision-guided pick-and-place pipeline:

```bash
python -m src.demo.pick_and_place
```

This opens a PyBullet GUI window showing:

* The robotic arm
* A fixed tabletop environment
* Randomly placed clutter objects
* Autonomous manipulation behavior driven by vision

---

## Project Structure

```
📁 Simulated Robotic Arm with Vision-Guided Object Manipulation/
├── 📁 assets/
│   ├── 📁 objects/
│   │   └── cube.urdf
│   ├── 📁 table/
│   │   └── table.urdf
│   └── ur5e.urdf
├── 📁 src/
│   ├── 📁 camera/
│   │   ├── camera_sim.py          # RGB / depth / segmentation cameras
│   │   └── object_detector.py     # Vision-based object detection
│   ├── 📁 control/
│   │   ├── arm_controller.py      # High-level arm control interface
│   │   └── inverse_kinematics.py  # Inverse kinematics solver
│   ├── 📁 simulation/
│   │   └── world.py               # Physics world and scene setup
│   └── 📁 demo/
│       └── pick_and_place.py      # Main executable pipeline ⭐
├── requirements.txt
└── README.md
```

---

## Technical Specifications

### Robot

* **Type**: 7-DOF articulated robotic arm
* **Control Mode**: Joint position control
* **End Effector**: Constraint-based grasping

### Vision System

* **Sensors**: Simulated RGB, depth, and segmentation cameras
* **Detection Method**: Color-based segmentation
* **Output**: Object positions in world coordinates

### Motion Generation

* **IK Solver**: PyBullet damped least-squares
* **Trajectory Generation**: Linear interpolation in joint space
* **Collision Strategy**: Height-based approach and retreat motions

### Coordinate System

* **Origin**: Robot base frame
* **X-axis**: Forward (toward table)
* **Y-axis**: Left
* **Z-axis**: Up

---

## Design Rationale

* PyBullet was chosen for rapid prototyping with realistic physics
* Rule-based planning ensures interpretability and deterministic behavior
* Height-based motion primitives avoid the need for full global path planning
* Modular structure mirrors real robotic software stacks used in production systems

---

## License

MIT License — free to use for learning, research, and portfolio purposes.

---

## Author

**Koutilya Ganapathiraju**
Email: [gkoutilyaraju@gmail.com](mailto:gkoutilyaraju@gmail.com)
LinkedIn: [https://www.linkedin.com/in/koutilya-ganapathiraju-0a3350182](https://www.linkedin.com/in/koutilya-ganapathiraju-0a3350182)

---