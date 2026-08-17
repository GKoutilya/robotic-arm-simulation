# Simulated Robotic Arm with Vision-Guided Object Manipulation

A complete physics-based simulation of a 7-DOF robotic arm performing autonomous, vision-guided pick-and-place in a cluttered tabletop environment using a modular perception–planning–control pipeline.

![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)
![PyBullet](https://img.shields.io/badge/PyBullet-Physics-green.svg)
![Computer Vision](https://img.shields.io/badge/Vision-RGB--D-red.svg)

---

## Overview

This project implements an end-to-end robotic manipulation system in simulation. A robotic arm observes a tabletop scene using a simulated RGB-D camera, localizes a target object with a segmentation-mask-based perception pipeline and true pinhole-camera unprojection, reasons about workspace clutter, and executes a smooth pick-and-place operation using inverse kinematics and physics-aware motion primitives.

The system separates **perception**, **planning**, and **control** into independent modules, mirroring the structure of real robotic manipulation stacks. That separation is not just conceptual: the same logic also runs as an actual multi-node **ROS2** computation graph (see [ROS2 Integration](#ros2-integration)), communicating over real topics/services/actions instead of in-process function calls, the standalone script below still works on its own and doesn't require ROS2 at all.

---

## System Architecture

The pipeline follows a standard robotics control loop:

### Perception

* Simulated RGB-D camera with a real pinhole camera model (explicit intrinsics from FOV/aspect/resolution, extrinsics from the camera's view matrix)
* Per-object localization via PyBullet's segmentation mask (a stand-in for a trained instance-segmentation network) rather than flat color-threshold pixel search, this avoids the color-bleed/ambiguity issues a naive color mask has with multiple similarly-colored objects
* Color classification of each detected object from its own RGB pixels
* True depth-based pixel → world unprojection (inverse of projection × view, not an assumed flat table plane)
* Closed-loop re-detection during final alignment (the arm re-runs vision each iteration while homing in on the object, not just once)

### Planning

* Task-level sequencing (approach → grasp → lift → place → retreat)
* `single`/`multi`/`sort` modes: rule-based clutter handling, a scripted distance check that relocates a single known blocking object, plus height-aware approach/retreat motions
* `clutter` mode: genuine collision-aware path planning, a goal-biased RRT ([`src/control/motion_planner.py`](src/control/motion_planner.py)) plans a 3D route around every vision-detected object standing between the arm and its target, with path shortcutting for smoother motion, falling back to physically relocating the nearest blocking object only if no path can be found

### Control

* Inverse kinematics using PyBullet's damped least-squares solver
* Smooth joint-space trajectory interpolation
* Position-controlled execution with bounded forces

Each component is modular and independently replaceable, reflecting real robotic system design. All camera, robot, and threshold constants live in one place ([`src/config.py`](src/config.py)) rather than being duplicated across modules.

**What's still ground truth, and why:** the physical grasp (the PyBullet constraint that attaches an object to the end-effector) and obstacle clearing both use PyBullet's simulation state directly. Vision drives *where to go and what to pick*; physics and a simple distance check handle *making and confirming contact* and *is something in the way*, respectively. It's the same division of labor a real stack has between a perception module and low-level force/contact sensing.

---

## Key Capabilities

| Capability                     | Description                                                                 |
| ------------------------------ | ----------------------------------------------------------------------------|
| **Vision-Guided Manipulation** | Segmentation + color classification + true camera projection drive target localization, including closed-loop re-detection during alignment |
| **Inverse Kinematics Control** | Real-time IK for a 7-DOF articulated arm                                    |
| **Collision-Aware Path Planning** (`clutter` mode) | RRT-based planning routes the arm around several vision-detected obstacles at once, not just a single scripted one |
| **Clutter-Aware Execution**    | Detects and relocates blocking objects when necessary (all modes); `clutter` mode plans around them first and only relocates as a fallback |
| **Stable Physics Interaction** | Fixed table, stable clutter placement, and controlled object dynamics       |
| **Smooth Motion Execution**    | Interpolated joint trajectories for natural arm motion                      |
| **Modular Architecture**       | Clean separation of perception, planning, and control logic, backed by a single config module and a pytest suite |
| **ROS2 Node Graph** (optional) | The same perception/planning/control logic also runs as 4 real ROS2 nodes communicating over topics/services/actions, with RViz2 visualization — see [ROS2 Integration](#ros2-integration) |

---

## Demonstrated Behavior

A full task execution proceeds as follows:

```
1. Capture an RGB-D frame from the simulated camera
2. Segment the frame and classify each detected object's color
3. Unproject each object's pixel centroid to a world position
4. Check for clutter blocking the target location
5. Relocate blocking objects if present
6. Pick up the target object, re-running vision each alignment step
7. Place the object at the goal position
8. Return the robot to a home configuration
```

All actions are executed under physics simulation with gravity, contact dynamics, and joint constraints enabled. Each run logs the vision-estimated object position alongside PyBullet's ground truth and the localization error between them, e.g.:

```
[VISION] Estimated red cube position: [0.352, -0.198, 0.725]
[SIM]    Actual red cube position:    [0.35, -0.2, 0.725]
[VISION] Localization error: 0.36 cm
```

### Clutter mode: real path planning

`--mode clutter` spawns 3-5 randomly-placed obstacle cubes around a target and asks the arm to actually route around them, rather than relying on the single-obstacle script the other modes use. Each pick/place step:

1. Detects every object on the table via one camera capture (target + obstacles)
2. Plans a collision-free 3D path around the obstacles with a goal-biased RRT, then shortcuts it into a smoother route
3. Executes that path; if no path exists, relocates the single nearest blocking obstacle and retries once before falling back to a direct best-effort move

```
[PLANNER] Found collision-free path with 14 waypoints around 5 obstacle(s)
```

---

## Installation

### Prerequisites

* Python 3.8 or newer
* pip

### Setup

```bash
git clone <this-repository-url>
cd "(8) Simulated Robotic Arm with Vision-Guided Object Manipulation"
pip install -r requirements.txt
```

### Dependencies

* `pybullet`
* `numpy`
* `matplotlib`
* `pytest` (dev/test only)

---

## Running the Simulation

Launch the vision-guided pick-and-place pipeline:

```bash
python -m src.demo.pick_and_place --mode single
```

This opens a PyBullet GUI window showing the robotic arm, a fixed tabletop, and autonomous manipulation driven by the vision pipeline described above.

### CLI options

| Flag              | Description                                                |
| ------------------ | ----------------------------------------------------------|
| `--mode`            | `single` (one cube + obstacle-clearing demo), `multi`, `sort`, or `clutter` (RRT path planning around several obstacles) |
| `--no-clutter`       | Disable clutter objects                                    |
| `--no-obstacle`      | Don't place an obstacle at the target position (`single` mode) |
| `--place-x/--place-y`| Override the target place position                         |
| `--seed`             | Random seed, for a reproducible obstacle layout (`clutter` mode) |
| `--verbose`          | Debug-level logging                                        |
| `--quiet`            | Only warnings and errors                                   |

### Running the tests

```bash
pytest
```

The suite covers the pure-math planning/detection logic plus headless (`p.DIRECT`) PyBullet integration tests for IK and the vision pipeline — no GUI or display required, so it also runs in CI (see [`.github/workflows/tests.yml`](.github/workflows/tests.yml)).

---

## ROS2 Integration

The perception/planning/control separation described above also exists as a real **ROS2 Jazzy** node graph, not a toy demo, an actual multi-process system where each node is a thin wrapper around the *same* functions the standalone script uses (`load_scene`, `calculate_ik`, `plan_path`/`shortcut_path`, `find_objects_by_segmentation`, `attach_object_to_ee`). This is additive: `python -m src.demo.pick_and_place` still works completely on its own and needs none of this.

### Node graph

| Node | Role | Wraps |
| --- | --- | --- |
| `sim_node` | Owns the PyBullet simulation ("hardware interface"); publishes `/joint_states`, `/camera/frame`, `/camera/rgb`; serves trajectory execution + grasp/release | `load_scene`, `hold_initial_pose`, `attach_object_to_ee`, `release_grasp` |
| `perception_node` | Detects objects from camera frames, publishes `/detected_objects` (`vision_msgs/Detection3DArray`) | `find_objects_by_segmentation`, `pixel_to_world_coords` |
| `planner_node` | Serves `/plan_path`; runs IK + the RRT planner against its own "kinematic twin" PyBullet instance (the same planning/execution-model separation MoveIt2 uses) | `calculate_ik`, `motion_planner.plan_path`/`shortcut_path` |
| `task_node` | Orchestrates a full pick-and-place as a `/pick_and_place` action (goal/feedback/result): detect → approach → descend → grasp → transport → lower → release | New code — re-expresses `run_single_mode`'s sequence as async service/action calls |

Custom interfaces (`arm_interfaces` package) are kept to the minimum that doesn't already have a good standard ROS type: `CameraFrame.msg` (RGB + raw depth buffer + PyBullet segmentation buffer, segmentation has no real-camera analogue), `PlanPath.srv`, `Grasp.srv`, and the top-level `PickAndPlace.action`. Everything else reuses standard types: `sensor_msgs/JointState`, `sensor_msgs/Image`, `vision_msgs/Detection3DArray`, `trajectory_msgs/JointTrajectory`, `std_srvs/Trigger`, and `control_msgs/action/FollowJointTrajectory` (the same action `ros2_control`/MoveIt2 use for trajectory execution).

**Current scope:** `task_node` handles single-object pick-and-place by target color (matching `single` mode's complexity). Porting `clutter` mode's obstacle-relocation fallback and closed-loop re-alignment into the ROS2 orchestration is a reasonable follow-up, not done yet.

### Setup (one-time)

ROS2 needs Linux. On Windows this means WSL2:

```bash
wsl --install -d Ubuntu   # then reboot, launch Ubuntu, finish first-run setup
```

Inside Ubuntu, install ROS2 Jazzy (desktop + dev tools — includes RViz2, `colcon`, `rosdep`), then create a Python venv that can see both `rclpy` and this project's dependencies:

```bash
# ROS2 Jazzy: follow https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
sudo apt install -y ros-jazzy-vision-msgs ros-jazzy-control-msgs  # not in the desktop install by default

python3 -m venv ~/.venvs/ros2_arm --system-site-packages
source ~/.venvs/ros2_arm/bin/activate
pip install pybullet numpy matplotlib pytest
pip install -e .   # from this repo's root — makes `src` importable anywhere (see pyproject.toml)
```

### Build

**Why a sync step:** CMake/`colcon` cannot parse a build path containing literal parentheses — and this repo's folder name has them (`(8) Simulated Robotic Arm...`). This is a hard syntax limitation in generated Makefiles, not a performance issue, so it can't be worked around by choosing a faster filesystem. [`ros2_ws/sync_and_build.sh`](ros2_ws/sync_and_build.sh) copies `ros2_ws/src` (the git-tracked source of truth, edited here) into a clean path (`~/ros2_ws`) and builds it there:

```bash
source /opt/ros/jazzy/setup.bash
source ~/.venvs/ros2_arm/bin/activate
bash ros2_ws/sync_and_build.sh
source ~/ros2_ws/install/setup.bash
```

Re-run this after any change under `ros2_ws/src/` in this repo.

### Run

```bash
ros2 launch arm_ros2 bringup.launch.py mode:=single
```

This starts all 4 nodes plus `robot_state_publisher` (for RViz2). In another sourced terminal, trigger a task:

```bash
ros2 action send_goal /pick_and_place arm_interfaces/action/PickAndPlace "{mode: 'single', target_color: 'red'}" --feedback
```

Open RViz2 (`rviz2 -d ros2_ws/src/arm_ros2/rviz/arm.rviz`, from a location where that relative path resolves, or just `rviz2` and load it manually) to watch the arm model move live and see the camera feed, driven by real `/joint_states` and `/camera/rgb` topics.

---

## Project Structure

```
📁 Simulated Robotic Arm with Vision-Guided Object Manipulation/
├── 📁 assets/
│   ├── 📁 objects/
│   │   └── cube.urdf
│   └── 📁 table/
│       └── table.urdf
├── 📁 src/
│   ├── config.py                      # Single source of truth for shared constants
│   ├── 📁 camera/
│   │   ├── camera_sim.py              # RGB-D-segmentation camera capture (CameraFrame)
│   │   └── object_detector.py         # Segmentation-based detection + real unprojection
│   ├── 📁 control/
│   │   ├── inverse_kinematics.py      # Inverse kinematics solver
│   │   ├── planner.py                 # Trajectory interpolation helpers
│   │   └── motion_planner.py          # RRT collision-aware path planning (clutter mode)
│   ├── 📁 simulation/
│   │   └── utils.py                   # Cube/bin spawning, settle-wait helpers
│   └── 📁 demo/
│       └── pick_and_place.py          # Main executable pipeline ⭐
├── 📁 tests/                          # pytest suite (pure-math + headless PyBullet)
├── 📁 ros2_ws/                        # ROS2 node graph (see ROS2 Integration below)
│   ├── sync_and_build.sh              # syncs src/ to a clean WSL2 path and builds it there
│   └── 📁 src/
│       ├── 📁 arm_interfaces/         # custom msg/srv/action definitions
│       └── 📁 arm_ros2/               # sim_node, perception_node, planner_node, task_node
├── 📁 .github/workflows/              # CI: runs pytest on push/PR
├── pyproject.toml                     # makes `src` pip-installable (`pip install -e .`)
├── requirements.txt
└── README.md
```

---

## Technical Specifications

### Robot

* **Type**: 7-DOF articulated robotic arm (KUKA IIWA)
* **Control Mode**: Joint position control
* **End Effector**: Constraint-based grasping

### Vision System

* **Sensor**: Simulated RGB-D camera (PyBullet TinyRenderer, so behavior is identical in GUI and headless/CI runs)
* **Detection Method**: Per-pixel segmentation mask for object localization + RGB-based color classification
* **Projection**: True pinhole unprojection (pixel + depth + inverse(projection × view) → world point), not an assumed flat table plane
* **Output**: Object positions in world coordinates, logged alongside ground truth for accuracy comparison

### Motion Generation

* **IK Solver**: PyBullet damped least-squares
* **Trajectory Generation**: Linear interpolation in joint space
* **Collision Strategy**: Height-based approach/retreat motions (`single`/`multi`/`sort`); goal-biased RRT over 3D end-effector positions with inflated-sphere obstacle checks and path shortcutting (`clutter`) — see [`src/control/motion_planner.py`](src/control/motion_planner.py)

### Coordinate System

* **Origin**: Robot base frame
* **X-axis**: Forward (toward table)
* **Y-axis**: Left
* **Z-axis**: Up

---

## Design Rationale

* PyBullet was chosen for rapid prototyping with realistic physics
* Segmentation-based detection avoids the multi-object ambiguity of flat color thresholding, while staying simple enough to reason about and test
* `single`/`multi`/`sort` modes use rule-based planning for interpretability and deterministic behavior
* `clutter` mode's RRT plans in 3D task space over inflated bounding-sphere obstacles rather than full 7-DOF joint-space planning with per-node collision queries, collision-aware and adaptive to a randomized scene each run, without the cost/fragility of the full-arm version, and pure geometry means it's fast, deterministic-when-seeded, and unit-testable without PyBullet
* Modular structure mirrors real robotic software stacks used in production systems, closely enough that the same logic runs unmodified as ROS2 nodes (see [ROS2 Integration](#ros2-integration)), each one a thin wrapper reusing the existing functions rather than a reimplementation
* `planner_node` runs IK/planning against its own separate "kinematic twin" PyBullet instance rather than the live simulated robot, mirroring the planning-model/execution-model separation real systems like MoveIt2 use
* ROS2 interfaces reuse standard message/service/action types wherever one genuinely fits (`sensor_msgs`, `vision_msgs/Detection3DArray`, `control_msgs/action/FollowJointTrajectory`), with custom types defined only where nothing standard covers the concept (e.g. `CameraFrame.msg`, which carries PyBullet's segmentation buffer, a sim-only privilege with no real-camera equivalent)

---

## License

MIT License — free to use for learning, research, and portfolio purposes.

---

## Author

**Koutilya Ganapathiraju**
- **Email**: [gkoutilyaraju@gmail.com](mailto:gkoutilyaraju@gmail.com)
- **LinkedIn**: [https://www.linkedin.com/in/koutilya-ganapathiraju-0a3350182](https://www.linkedin.com/in/koutilya-ganapathiraju-0a3350182)

---
