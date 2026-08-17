import numpy as np

from src.camera.object_detector import detect_objects, detect_all_colored_objects, _closest_color
from src.config import COLORS


def make_image(width=20, height=20, background=(0, 0, 0)):
    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[:, :] = background
    return img


def paint_block(img, color, y0, y1, x0, x1):
    img[y0:y1, x0:x1] = color
    return img


class TestDetectObjects:
    def test_finds_no_object_in_empty_image(self):
        img = make_image()
        assert detect_objects(img, target_color=[255, 0, 0]) == []

    def test_finds_centroid_of_solid_block(self):
        img = make_image()
        img = paint_block(img, [255, 0, 0], y0=4, y1=8, x0=10, x1=14)
        # block spans rows [4,8) -> center row 5.5, cols [10,14) -> center col 11.5
        detected = detect_objects(img, target_color=[255, 0, 0])
        assert len(detected) == 1
        cx, cy = detected[0]
        assert 10 <= cx <= 13
        assert 4 <= cy <= 7

    def test_ignores_colors_outside_threshold(self):
        img = make_image()
        img = paint_block(img, [0, 255, 0], y0=4, y1=8, x0=10, x1=14)  # green block
        assert detect_objects(img, target_color=[255, 0, 0], threshold=50) == []

    def test_within_threshold_still_detected(self):
        img = make_image()
        # close to red but not exact
        img = paint_block(img, [230, 20, 10], y0=0, y1=5, x0=0, x1=5)
        detected = detect_objects(img, target_color=[255, 0, 0], threshold=50)
        assert len(detected) == 1


class TestDetectAllColoredObjects:
    def test_detects_multiple_distinct_colors(self):
        img = make_image(width=30, height=10)
        img = paint_block(img, COLORS['red'], y0=0, y1=5, x0=0, x1=5)
        img = paint_block(img, COLORS['blue'], y0=0, y1=5, x0=20, x1=25)

        detected = detect_all_colored_objects(img, threshold=50)

        assert 'red' in detected
        assert 'blue' in detected
        assert 'green' not in detected


class TestClosestColor:
    def test_exact_match(self):
        assert _closest_color(COLORS['red']) == 'red'

    def test_near_match_within_threshold(self):
        assert _closest_color([250, 5, 5], threshold=60) == 'red'

    def test_far_from_any_known_color_returns_none(self):
        # mid-gray is far from every saturated primary in COLORS
        assert _closest_color([128, 128, 128], threshold=60) is None
