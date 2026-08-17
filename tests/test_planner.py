import numpy as np
import pytest

from src.control.planner import interpolate_positions, plan_arc_trajectory


class TestInterpolatePositions:
    def test_starts_at_start_pos(self):
        waypoints = interpolate_positions([0, 0, 0], [1, 2, 3], num_steps=10)
        assert waypoints[0] == [0, 0, 0]

    def test_ends_at_end_pos(self):
        end = [1.0, 2.0, 3.0]
        waypoints = interpolate_positions([0, 0, 0], end, num_steps=10)
        assert np.allclose(waypoints[-1], end)

    def test_returns_num_steps_plus_one_waypoints(self):
        waypoints = interpolate_positions([0, 0, 0], [1, 1, 1], num_steps=7)
        assert len(waypoints) == 8

    def test_monotonic_progress_along_each_axis(self):
        start, end = [0, 0, 0], [2, -3, 1]
        waypoints = interpolate_positions(start, end, num_steps=20)
        for axis in range(3):
            values = [wp[axis] for wp in waypoints]
            diffs = np.diff(values)
            sign = np.sign(end[axis] - start[axis])
            # every step should move in the same direction as start->end (or stay put)
            assert np.all(diffs * sign >= -1e-9)

    def test_ease_in_out_is_slower_at_endpoints_than_midpoint(self):
        waypoints = interpolate_positions([0, 0, 0], [10, 0, 0], num_steps=10)
        first_step = waypoints[1][0] - waypoints[0][0]
        mid_step = waypoints[6][0] - waypoints[5][0]
        assert mid_step > first_step


class TestPlanArcTrajectory:
    def test_endpoints_match_start_and_end(self):
        start, end = [0, 0, 0], [1, 0, 0]
        waypoints = plan_arc_trajectory(start, end, arc_height=0.2, num_steps=10)
        assert np.allclose(waypoints[0], start)
        assert np.allclose(waypoints[-1], end)

    def test_peaks_above_start_and_end_height(self):
        start, end = [0, 0, 0.0], [1, 0, 0.0]
        arc_height = 0.15
        waypoints = plan_arc_trajectory(start, end, arc_height=arc_height, num_steps=20)
        max_z = max(wp[2] for wp in waypoints)
        assert max_z == pytest.approx(arc_height, abs=1e-6)

    def test_peak_occurs_near_midpoint(self):
        waypoints = plan_arc_trajectory([0, 0, 0], [1, 0, 0], arc_height=0.1, num_steps=20)
        z_values = [wp[2] for wp in waypoints]
        peak_index = int(np.argmax(z_values))
        assert 8 <= peak_index <= 12
