import os

import numpy as np
import pybullet as p
import pybullet_data
import pytest

from src.camera.camera_sim import capture_camera_image
from src.camera.object_detector import find_objects_by_segmentation, pixel_to_world_coords
from src import config


@pytest.fixture
def scene():
    """Headless plane + table + one red cube, matching load_scene('single')."""
    client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    plane_id = p.loadURDF("plane.urdf")
    table_id = p.loadURDF(
        os.path.join("assets", "table", "table.urdf"),
        basePosition=config.TABLE_POSITION,
        useFixedBase=True,
    )
    cube_pos = [0.55, 0.05, 0.7]
    cube_id = p.loadURDF(
        os.path.join("assets", "objects", "cube.urdf"),
        basePosition=cube_pos,
    )

    for _ in range(50):
        p.stepSimulation()

    yield {"plane_id": plane_id, "table_id": table_id, "cube_id": cube_id, "cube_pos": cube_pos}
    p.disconnect(client)


def test_capture_returns_frame_with_expected_shape():
    p.connect(p.DIRECT)
    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        frame = capture_camera_image()
        assert frame.rgb.shape == (config.CAMERA.img_height, config.CAMERA.img_width, 3)
        assert frame.segmentation.shape == (config.CAMERA.img_height, config.CAMERA.img_width)
        assert frame.depth_buffer.shape == (config.CAMERA.img_height, config.CAMERA.img_width)
    finally:
        p.disconnect()


def test_segmentation_detects_the_cube_by_color(scene):
    frame = capture_camera_image()
    ignore_ids = {scene["plane_id"], scene["table_id"]}

    detections = find_objects_by_segmentation(frame, ignore_body_ids=ignore_ids)

    assert 'red' in detections
    assert detections['red']['body_id'] == scene["cube_id"]


def test_segmentation_position_estimate_is_close_to_ground_truth(scene):
    frame = capture_camera_image()
    ignore_ids = {scene["plane_id"], scene["table_id"]}

    detections = find_objects_by_segmentation(frame, ignore_body_ids=ignore_ids)
    estimated_pos = np.array(detections['red']['position'])

    actual_pos, _ = p.getBasePositionAndOrientation(scene["cube_id"])
    actual_pos = np.array(actual_pos)

    # TinyRenderer + a low-res 256x256 capture introduces a few cm of
    # centroid/unprojection error; this just guards against gross breakage
    # (e.g. an inverted axis or wrong matrix convention), not sub-cm accuracy.
    assert np.linalg.norm(estimated_pos - actual_pos) < 0.08


def test_ignored_body_ids_are_excluded_from_detections(scene):
    frame = capture_camera_image()
    all_ids = {scene["plane_id"], scene["table_id"], scene["cube_id"]}

    detections = find_objects_by_segmentation(frame, ignore_body_ids=all_ids)

    assert detections == {}


def test_background_pixels_never_produce_a_phantom_detection(scene):
    # Regression test: segmentation background pixels are -1. An earlier
    # version bit-masked the segmentation buffer with `& 0xFFFFFF` to strip a
    # link-index encoding that was never actually requested from PyBullet,
    # which corrupted -1 (two's complement) into 16777215 -- a fake "body id"
    # that only got excluded by accident (its color happened not to match).
    # Only real body IDs found in the raw segmentation buffer may be detected.
    frame = capture_camera_image()
    valid_ids = {int(v) for v in np.unique(frame.segmentation) if v >= 0}

    detections = find_objects_by_segmentation(frame, ignore_body_ids=frozenset())

    for info in detections.values():
        assert info["body_id"] in valid_ids
