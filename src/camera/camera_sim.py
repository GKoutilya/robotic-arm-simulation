import pybullet as p
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
import time

def capture_camera_image():
    cam_target_pos = [0.6, 0, 0.35]
    cam_distance = 1.0
    cam_yaw = 90
    cam_pitch = -90
    cam_roll = 0
    up_vector = [0, 1, 0]

    view_matrix = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=cam_target_pos,
        distance=cam_distance,
        yaw=cam_yaw,
        pitch=cam_pitch,
        roll=cam_roll,
        upAxisIndex=2
    )

    fov = 60
    aspect = 1
    near = 0.1
    far = 3
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

    img_width = 256
    img_height = 256
    img_arr = p.getCameraImage(
        width=img_width,
        height=img_height,
        viewMatrix=view_matrix,
        projection_matrix=projection_matrix,
        renderer=p.ER_BULLET_HARDWARE_OPENGL
    )

    rgb = np.reshape(img_arr[2], (img_height, img_width, 4))
    depth = np.reshape(img_arr[3], (img_height, img_width))

    depth_real = far * near / (far - (far - near) * depth)
    rgb_uint8 = (rgb[:, :, :3] * 255).astype('uint8')

    return rgb_uint8, depth_real

def main():
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")
    p.loadURDF("table/table.urdf", [0.5, 0, 0.325])

    for _ in range(100):
        p.stepSimulation()
        time.sleep(1./240.)

    rgb_img, depth_img = capture_camera_image()

    plt.imshow(rgb_img)
    plt.title("RGB Camera View")
    plt.axis('off')
    plt.show()

    p.disconnect()

if __name__ == "__main__":
    main()