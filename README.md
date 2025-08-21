# Vision-Guided Robotic Arm Simulation

This project is a **simulation of a robotic arm performing vision-based pick-and-place tasks**.  
It is designed as a sandbox for experimenting with computer vision, robotics control, and reinforcement learning techniques, without needing real hardware.

## Features (Current Progress)
- **Simulation environment** built in [PyBullet](https://pybullet.org/).
- **Table setup with clutter objects** for manipulation tasks.
- **Robotic arm model** (URDF-based) for interaction.
- **Camera system** with three simulated modalities:
  - RGB camera
  - Depth camera
  - Segmentation camera

## Planned Features
- Object detection and pose estimation from camera feeds.
- Grasp planning for cluttered environments.
- Pick-and-place task execution with trajectory planning.
- Reinforcement learning integration for policy-based control.
- Logging and visualization of simulation runs.

## Project Structure
```

project\_root/
│── camera\_sim.py        # Camera setup and rendering (RGB, Depth, Segmentation)
│── robot\_sim.py         # Robotic arm setup and control
│── table\_sim.py         # Table and clutter generation
│── main.py              # Entry point for running the full simulation
│── README.md            # Project documentation

````

## Getting Started
1. Clone this repository:
   ```bash
   git clone <repo-url>
   cd vision-guided-robotic-arm
    ```

2. Install dependencies:

   ```bash
   pip install pybullet numpy
   ```
3. Run the simulation:

   ```bash
   python main.py
   ```


## Future Directions

* Integration of real-time vision-based object detection.
* Domain randomization for sim-to-real transfer.
* More advanced robotic arm models and end-effectors.
* Multi-camera setups and sensor fusion.

---

### Status

Currently in **early development phase**. The environment, robot, table, clutter, and cameras are functional.
Next step: **integrating perception and control for object manipulation.**
