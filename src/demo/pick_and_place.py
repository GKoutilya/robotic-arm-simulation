from src.control.arm_controller import ArmController
from src.simulation.world import add_clutter
from src.camera.camera_sim import capture_camera_image
from src.camera.object_detector import find_target_object
from src.control.inverse_kinematics import calculate_ik
import pybullet_data
import pybullet as p
import numpy as np
import time
import os

def connect_sim(gui=True):
    if gui:
        physics_client = p.connect(p.GUI)
    else:
        physics_client = p.connect(p.DIRECT)
    
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    return physics_client

def load_scene():
    plane_id = p.loadURDF("plane.urdf")
    table_path = os.path.join("assets", "table", "table.urdf")
    table_id = p.loadURDF(table_path, basePosition=[0.5, 0, 0.325], useFixedBase=True)
    cube_path = os.path.join("assets", "objects", "cube.urdf")
    cube_id = p.loadURDF(cube_path, basePosition=[0.7, 0, 0.35])
    robot_path = "kuka_iiwa/model.urdf"
    robot_id = p.loadURDF(robot_path, basePosition=[0, 0, 0], useFixedBase=True)
    return robot_id, cube_id, table_id

def _mat3(x):
    return np.array(x, dtype=np.float64).reshape(4, 4)

def _inv(x):
    return np.linalg.inv(x)

def pixel_to_world(u, v, depth_val, view_matrix, proj_matrix, img_w, img_h):
    """
        Unproject a pixel (u, v) with depth into world coordinates.
    """
    x_ndc = (2.0 * u / img_w) - 1.0
    y_ndc = 1.0 - (2.0 * v / img_h)
    z_ndc = 2.0 * depth_val - 1.0

    ndc = np.array([x_ndc, y_ndc, z_ndc, 1.0])

    P = _mat3(proj_matrix)
    V = _mat3(view_matrix)
    VP_inv = _inv(P @ V)

    world_h = VP_inv @ ndc
    world_h /= world_h[3] + 1e-9

    return world_h[:3]

def get_camera_frame(get_viewproj_fn, img_w=256, img_h=256):
    """
        Capture RGB, depth, seg + matrices using your existing camera setup.
    """
    view_matrix, proj_matrix = get_viewproj_fn()
    img = p.getCameraImage(img_w, img_h, view_matrix, proj_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    rgba = np.array(img[2]).reshape(img_h, img_w, 4)
    depth = np.array(img[3]).reshape(img_h, img_w)
    seg = np.array(img[4]).reshape(img_h, img_w)
    rgb_u8 = (rgba[:, :, :3] * 255).astype(np.uint8)
    return rgb_u8, depth, seg, view_matrix, proj_matrix

def seg_to_body_and_link(seg_val):
    """
        Pybullet encodes: (bodyUniquID << 24) + (linkIndex & ((1<<24)-1))
    """
    if seg_val < 0:
        return -1 -1
    body_id = seg_val >> 24
    link_id = seg_val & ((1 << 24) - 1)
    return body_id, link_id

def find_object_centroid_2d_by_body(seg_img, target_body_id):
    ys, xs, = np.where((seg_img >> 24) == target_body_id)
    if xs.size == 0:
        return None
    u = int(xs.mean())
    v = int(ys.mean())
    return (u, v)

def find_target_3d(target_body_id, get_viewproj_fn, img_w=256, img_h=256):
    """
        Return 3D world positioon of the target body (centroid via segmentation).
    """
    rgb, depth, seg, V, P = get_camera_frame(get_viewproj_fn, img_w, img_h)
    uv = find_object_centroid_2d_by_body(seg, target_body_id)
    if uv is None:
        return None
    u, v = uv
    pos_w = pixel_to_world(u, v, depth[v, u], V, P, img_w, img_h)
    return pos_w

def move_joints(robot_id, q, steps=240):
    nj = p.getNumJoints(robot_id)
    for i in range(nj):
        p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, targetPosition=q[i] if i < len(q) else 0.0, force=200)
    for _ in range(steps):
        p.stepSimulation()

def go_to_pose(robot_id, ee_link, pos, orn=None):
    if orn is None:
        orn = p.getQuaternionFromEuler([0, 0, 0])
    q = calculate_ik(robot_id, pos, orn, end_effector_index=ee_link)
    move_joints(robot_id, q)

def check_collisions(robot_id, obj_id, distance_threshold=0.01):
    """
        Check for potential collisions between the robot and an object.

        Args:
            robot_id: ID of the robot.
            obj_id: ID of the object to check collisions with.
            distance_threshold: Minimum distance to consider as a collision.

        Returns:
            True if a collision is detected, False otherwise.
    """
    closest_points = p.getClosestPoints(robot_id, obj_id, distance_threshold)
    if closest_points:
        print(f"[WARNING] Potential collision detected between robot and object {obj_id}")
        return True
    return False

def fake_grasp(robot_id, ee_link, obj_id):
    """
        Attach object to end-effector via a fixed constraint (suction-like).
    """
    ee_state = p.getLinkState(robot_id, ee_link)
    ee_pos, ee_orn = ee_state[4], ee_state[5]
    obj_pos, obj_orn = p.getBasePositionAndOrientation(obj_id)
    cid = p.createConstraint(
        parentBodyUniqueId=robot_id,
        parentLinkIndex=ee_link,
        childBodyUniqueId=obj_id,
        childLinkIndex=1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=[0, 0, 0],
        childFramePosition=[obj_pos[0]-ee_pos[0], obj_pos[1]-ee_pos[1], obj_pos[2]-ee_pos[2]],
        parentFrameOrientation=ee_orn,
        childFrameOrientation=obj_orn
    )
    return cid

def release_grasp(constraint_id):
    if constraint_id is not None:
        p.removeConstraint(constraint_id)

def pick_and_place(robot_id, ee_link, target_xyz, place_xyz, approach_offset=0.12, lift_offset=0.15, obj_id=None):
    pre = [target_xyz[0], target_xyz[1], target_xyz[2] + approach_offset]
    go_to_pose(robot_id, ee_link, pre)

    contact = [target_xyz[0], target_xyz[1], target_xyz[2] + 0.01]
    go_to_pose(robot_id, ee_link, contact)

    grasp_cid = None
    if obj_id is not None:
        grasp_cid = fake_grasp(robot_id, ee_link, obj_id)

    lift = [target_xyz[0], target_xyz[1], target_xyz[2] + lift_offset]
    go_to_pose(robot_id, ee_link, lift)

    pre_place = [place_xyz[0], place_xyz[1], place_xyz[2] + lift_offset]
    go_to_pose(robot_id, ee_link, pre_place)

    place_contact = [place_xyz[0], place_xyz[1], place_xyz[2] + 0.02]
    go_to_pose(robot_id, ee_link, place_contact)
    release_grasp(grasp_cid)

    go_to_pose(robot_id, ee_link, pre_place)

def main():
    try:
        print("[INFO] Starting vision-guided pick-and-place pipeline...")

        print("[DEBUG] Connecting to PyBullet...")
        physics_client = connect_sim(gui=True)
        print("[DEBUG] Connected to PyBullet.")

        print("[DEBUG] Loading scene...")
        robot_id, cube_id, table_id = load_scene()
        print("[DEBUG] Scene loaded successfully.")

        print("[DEBUG] Adding clutter...")
        add_clutter(num_objects=8, table_id=table_id)
        print("[DEBUG] Clutter added.")

        for _ in range(100):
            p.stepSimulation()
            time.sleep(1./240.)

        print("[INFO] Detecting target object...")
        target_pos = find_target_object(target_color=[255, 0, 0], threshold=50)

        if target_pos is None:
            print("[WARNING] Vision failed to detect object. Using fallback position.")
            cube_pos, _ = p.getBasePositionAndOrientation(cube_id)
            target_pos = list(cube_pos)

        print(f"[INFO] Target object detected at: {target_pos}")

        # Control - Create ArmController
        arm = ArmController(robot_id, end_effector_index=6, joint_indices=list(range(p.getNumJoints(robot_id))))

        pre_grasp = [cube_pos[0], cube_pos[1], cube_pos[2] + 0.15]
        grasp = [cube_pos[0], cube_pos[1], cube_pos[2] + 0.02]
        lift = [target_pos[0], target_pos[1], target_pos[2] + 0.20]
        place = [cube_pos[0] - 0.2, cube_pos[1], cube_pos[2] + 0.15]

        print("[INFO] Executing pick-and-place sequence...")

        # Execution
        arm.move_to_pose(pre_grasp)
        for _ in range(120): p.stepSimulation(); time.sleep(1./240.)
        arm.move_to_pose(grasp)
        for _ in range(120): p.stepSimulation(); time.sleep(1./240.)
        arm.move_to_pose(pre_grasp)
        for _ in range(120): p.stepSimulation(); time.sleep(1./240.)
        arm.move_to_pose(place)
        for _ in range(120): p.stepSimulation(); time.sleep(1./240.)

        print("[SUCCESS] Pick and place complete.")

        print("[INFO] Simulation running. Press CTRL + C to exit...")
        while True:
            p.stepSimulation()
            time.sleep(1.0 / 240.0)
    except Exception as e:
        print(f"[ERROR] An exception occurred: {e}")
    finally:
        p.disconnect()


if __name__ == "__main__":
    main()