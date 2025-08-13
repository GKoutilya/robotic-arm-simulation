import numpy as np
from src.camera.camera_sim import capture_camera_image

def detect_objects(rgb_image, target_color=[255, 0, 0], threshold=50):
    """
        Detect objects in the RGB image based on color.

        Args:
            rgb_image: RGB image array
            target_color: Target RGB color to detect [R, G, B]
            threshold: Color matching threshold

        Returns:
            object_positions: List of detected object centers in pixel coordinates
    """
    rgb = np.array(rgb_image)

    color_diff = np.abs(rgb - target_color)
    mask = np.all(color_diff < threshold, axis=2)

    if np.any(mask):
        y_coords, x_coords = np.where(mask)
        if len(x_coords) < 0:
            center_x = int(np.mean(x_coords))
            center_y = int(np.mean(y_coords))
            return [(center_x, center_y)]
        
    return []

def pixel_to_world_coords(pixel_x, pixel_y, depth_value, camera_params=None):
    """
        Convert pixel coordinates to world coordinates using depth information.

        Args:
            pixel_x: X-coordinate in the image
            pixel_y: Y-coordinate in the image
            depth_value: Depth value at the pixel
            camera_params: Camera parameters (optional, for calibration)

        Returns:
            world_coords: [x, y, z] world coordinates
    """
    img_width, img_height = 256, 256
    table_center = [0.6, 0, 0.35]
    table_size = 0.6

    norm_x = (pixel_x - img_width / 2) / (img_width / 2)
    norm_y = (pixel_y - img_height / 2) / (img_height / 2)

    world_x = table_center[0] + norm_x * table_size / 2
    world_y = table_center[1] + norm_y * table_size / 2
    world_z = table_center[2] + 0.02

    return [world_x, world_y, world_z]

def find_target_object(target_color=[255, 0, 0], threshold=50):
    """
        Use the camera to find the target object in the scene.

        Args:
            target_color: Target RGB color to detect [R, G, B]
            threshold: Color matching threshold
        
        Returns:
            object_pose: [x, y, z] world coordinates of the detected object, or None
    """
    print("[INFO] Capturing camera image...")
    rgb_img, depth_img = capture_camera_image()

    print("[INFO] Detecting object...")
    detected_objects = detect_objects(rgb_img, target_color=target_color, threshold=threshold)

    if detected_objects:
        pixel_x, pixel_y = detected_objects[0]
        depth_value = depth_img[pixel_y, pixel_x]

        world_pos = pixel_to_world_coords(pixel_x, pixel_y, depth_value)

        print(f"[INFO] Detected object at pixel ({pixel_x}, {pixel_y})")
        print(f"[INFO] Estimated world position: {world_pos}")
        
        return world_pos
    
    print("[WARNING] No target object detected.")
    return None