import numpy as np

from src.camera.camera_sim import capture_camera_image
from src.config import COLORS


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

    color_diff = np.abs(rgb.astype(np.int16) - np.array(target_color, dtype=np.int16))
    mask = np.all(color_diff < threshold, axis=2)

    if np.any(mask):
        y_coords, x_coords = np.where(mask)
        if len(x_coords) > 0:
            center_x = int(np.mean(x_coords))
            center_y = int(np.mean(y_coords))
            return [(center_x, center_y)]

    return []


def detect_all_colored_objects(rgb_image, threshold=50):
    """
    Detect all colored objects in the image.

    Args:
        rgb_image: RGB image array
        threshold: Color matching threshold

    Returns:
        Dictionary of color -> list of pixel positions
    """
    detected = {}

    for color_name, color_rgb in COLORS.items():
        objects = detect_objects(rgb_image, target_color=color_rgb, threshold=threshold)
        if objects:
            detected[color_name] = objects

    return detected


def _closest_color(mean_rgb, threshold=60):
    """
    Classify a mean RGB sample against the known COLORS palette.
    `threshold` is a per-channel-style tolerance, scaled to a Euclidean
    bound (threshold * sqrt(3)) to match the semantics of detect_objects's
    per-channel `< threshold` comparison.
    """
    mean_rgb = np.array(mean_rgb, dtype=np.float64)
    best_name, best_dist = None, None
    for name, ref in COLORS.items():
        dist = np.linalg.norm(mean_rgb - np.array(ref, dtype=np.float64))
        if best_dist is None or dist < best_dist:
            best_name, best_dist = name, dist

    if best_dist is not None and best_dist < threshold * np.sqrt(3):
        return best_name
    return None


def pixel_to_world_coords(frame, pixel_x, pixel_y):
    """
    Unproject a pixel (using its true depth-buffer value) back to world
    coordinates via the camera's actual view/projection matrices, i.e. real
    pinhole-camera unprojection rather than an assumed flat table plane.

    Args:
        frame: a camera_sim.CameraFrame (must carry the raw depth_buffer,
            view_matrix and projection_matrix used to capture it)
        pixel_x, pixel_y: pixel coordinates (image space, y-down)

    Returns:
        world_coords: [x, y, z] world coordinates
    """
    proj = np.asarray(frame.projection_matrix, dtype=np.float64).reshape(4, 4, order="F")
    view = np.asarray(frame.view_matrix, dtype=np.float64).reshape(4, 4, order="F")
    pixel_to_world_matrix = np.linalg.inv(proj @ view)

    x_ndc = (2.0 * pixel_x - frame.width) / frame.width
    y_ndc = -(2.0 * pixel_y - frame.height) / frame.height
    z_ndc = 2.0 * frame.depth_buffer[int(pixel_y), int(pixel_x)] - 1.0

    clip_space = np.array([x_ndc, y_ndc, z_ndc, 1.0])
    world = pixel_to_world_matrix @ clip_space
    world /= world[3]

    return world[:3].tolist()


def find_objects_by_segmentation(frame, ignore_body_ids=frozenset(), color_threshold=60):
    """
    Localize every object visible in `frame` using PyBullet's per-pixel
    segmentation mask (a stand-in for a trained instance-segmentation
    network) for robust pixel localization, then classify each detected
    object's color from its own RGB pixels and unproject its centroid to a
    world position via pixel_to_world_coords.

    Args:
        frame: a camera_sim.CameraFrame
        ignore_body_ids: body unique IDs to skip (e.g. plane, table, robot)
        color_threshold: passed through to _closest_color

    Returns:
        Dictionary: color_name -> {"position": [x,y,z], "body_id": int, "pixel": (x, y)}
    """
    # capture_camera_image() does not request ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX,
    # so frame.segmentation already holds plain body unique IDs, with -1 for
    # "no object" background pixels -- filter those out before doing anything else.
    unique_ids = np.unique(frame.segmentation)

    results = {}
    for body_id in unique_ids:
        body_id = int(body_id)
        if body_id < 0 or body_id in ignore_body_ids:
            continue

        mask = frame.segmentation == body_id
        ys, xs = np.where(mask)
        if len(xs) == 0:
            continue

        center_x, center_y = int(np.mean(xs)), int(np.mean(ys))
        mean_rgb = frame.rgb[mask].mean(axis=0)

        color_name = _closest_color(mean_rgb, threshold=color_threshold)
        if color_name is None:
            continue

        results[color_name] = {
            "position": pixel_to_world_coords(frame, center_x, center_y),
            "body_id": body_id,
            "pixel": (center_x, center_y),
        }

    return results


def find_target_object(target_color=[255, 0, 0], threshold=50):
    """
    Use the camera to find the target object in the scene via color-threshold
    detection (kept for simple single-object cases; find_objects_by_segmentation
    is the more robust multi-object path used by the live demo).

    Args:
        target_color: Target RGB color to detect [R, G, B]
        threshold: Color matching threshold

    Returns:
        object_pose: [x, y, z] world coordinates of the detected object, or None
    """
    frame = capture_camera_image()

    detected_objects = detect_objects(frame.rgb, target_color=target_color, threshold=threshold)

    if detected_objects:
        pixel_x, pixel_y = detected_objects[0]
        world_pos = pixel_to_world_coords(frame, pixel_x, pixel_y)
        return world_pos

    return None


def find_all_objects(threshold=80):
    """
    Find all colored objects in the scene via color-threshold detection.

    Args:
        threshold: Color matching threshold

    Returns:
        Dictionary of color -> world position
    """
    frame = capture_camera_image()

    detected = detect_all_colored_objects(frame.rgb, threshold=threshold)

    objects = {}
    for color_name, pixel_positions in detected.items():
        pixel_x, pixel_y = pixel_positions[0]
        objects[color_name] = pixel_to_world_coords(frame, pixel_x, pixel_y)

    return objects
