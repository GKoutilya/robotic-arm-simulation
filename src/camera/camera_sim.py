import pybullet as p
import pybullet_data
import numpy as np
import time
from dataclasses import dataclass

from src.config import CAMERA


@dataclass
class CameraFrame:
    """A single RGB-D-segmentation capture plus the matrices needed to unproject it."""
    rgb: np.ndarray                # (H, W, 3) uint8
    depth: np.ndarray              # (H, W) float, linearized real-world distance (meters)
    depth_buffer: np.ndarray       # (H, W) float, raw OpenGL depth buffer in [0, 1]
    segmentation: np.ndarray       # (H, W) int, PyBullet body unique ID per pixel
    view_matrix: tuple             # raw 16-tuple from PyBullet (column-major)
    projection_matrix: tuple       # raw 16-tuple from PyBullet (column-major)
    width: int
    height: int


def capture_camera_image(camera=CAMERA):
    """
    Capture an RGB / depth / segmentation frame from the simulated camera
    described by `camera` (see src.config.CameraConfig).

    Uses PyBullet's software (TinyRenderer) renderer rather than hardware
    OpenGL so captures are identical whether connected via p.GUI or the
    headless p.DIRECT mode used in tests/CI.
    """
    view_matrix = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=camera.target_pos,
        distance=camera.distance,
        yaw=camera.yaw,
        pitch=camera.pitch,
        roll=camera.roll,
        upAxisIndex=2
    )
    projection_matrix = p.computeProjectionMatrixFOV(
        camera.fov, camera.aspect, camera.near, camera.far
    )

    width, height, rgb_px, depth_px, seg_px = p.getCameraImage(
        width=camera.img_width,
        height=camera.img_height,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
        renderer=p.ER_TINY_RENDERER
    )

    rgb = np.reshape(rgb_px, (height, width, 4))[:, :, :3].astype(np.uint8)
    depth_buffer = np.reshape(depth_px, (height, width)).astype(np.float64)
    segmentation = np.reshape(seg_px, (height, width)).astype(np.int64)

    depth_real = camera.far * camera.near / (camera.far - (camera.far - camera.near) * depth_buffer)

    return CameraFrame(
        rgb=rgb,
        depth=depth_real,
        depth_buffer=depth_buffer,
        segmentation=segmentation,
        view_matrix=view_matrix,
        projection_matrix=projection_matrix,
        width=width,
        height=height,
    )


def main():
    import matplotlib.pyplot as plt

    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")
    p.loadURDF("table/table.urdf", [0.5, 0, 0.325])

    for _ in range(100):
        p.stepSimulation()
        time.sleep(1. / 240.)

    frame = capture_camera_image()

    plt.imshow(frame.rgb)
    plt.title("RGB Camera View")
    plt.axis('off')
    plt.show()

    p.disconnect()

if __name__ == "__main__":
    main()
