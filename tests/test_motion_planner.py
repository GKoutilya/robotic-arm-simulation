import numpy as np
import pytest

from src.control.motion_planner import (
    Obstacle,
    is_segment_clear,
    plan_path,
    shortcut_path,
    find_blocking_obstacle,
)

BOUNDS = {"x_min": 0.0, "x_max": 1.0, "y_min": -0.5, "y_max": 0.5, "z_min": 0.0, "z_max": 0.5}


def path_len_waypoints(path):
    return sum(np.linalg.norm(np.array(path[i + 1]) - np.array(path[i])) for i in range(len(path) - 1))


def path_is_collision_free(path, obstacles, clearance=0.0):
    return all(
        is_segment_clear(path[i], path[i + 1], obstacles, clearance)
        for i in range(len(path) - 1)
    )


class TestIsSegmentClear:
    def test_far_away_obstacle_is_clear(self):
        obs = [Obstacle(position=[10, 10, 10], radius=0.05)]
        assert is_segment_clear([0, 0, 0], [1, 0, 0], obs, clearance=0.0)

    def test_obstacle_directly_on_segment_blocks(self):
        obs = [Obstacle(position=[0.5, 0, 0], radius=0.05)]
        assert not is_segment_clear([0, 0, 0], [1, 0, 0], obs, clearance=0.0)

    def test_obstacle_within_clearance_blocks(self):
        # 0.08m off the line; radius 0.05 alone doesn't reach it, +0.05 clearance does.
        obs = [Obstacle(position=[0.5, 0.08, 0], radius=0.05)]
        assert is_segment_clear([0, 0, 0], [1, 0, 0], obs, clearance=0.0)
        assert not is_segment_clear([0, 0, 0], [1, 0, 0], obs, clearance=0.05)

    def test_obstacle_beyond_segment_endpoints_does_not_block(self):
        # obstacle sits "behind" p1's endpoint direction, not between p1 and p2
        obs = [Obstacle(position=[-1, 0, 0], radius=0.05)]
        assert is_segment_clear([0, 0, 0], [1, 0, 0], obs, clearance=0.0)


class TestPlanPath:
    def test_connects_start_and_goal_with_no_obstacles(self):
        rng = np.random.default_rng(0)
        start, goal = [0.1, 0, 0.1], [0.9, 0, 0.1]
        path = plan_path(start, goal, [], BOUNDS, rng=rng)
        assert path is not None
        assert np.allclose(path[0], start)
        assert np.allclose(path[-1], goal)

    def test_avoids_a_blocking_obstacle(self):
        rng = np.random.default_rng(1)
        start, goal = [0.1, 0, 0.1], [0.9, 0, 0.1]
        obstacles = [Obstacle(position=[0.5, 0, 0.1], radius=0.1)]
        clearance = 0.02

        path = plan_path(start, goal, obstacles, BOUNDS, clearance=clearance, rng=rng)

        assert path is not None
        assert path_is_collision_free(path, obstacles, clearance)

    def test_returns_none_when_goal_is_fully_enclosed(self):
        rng = np.random.default_rng(2)
        start = [0.1, 0, 0.1]
        goal = [0.5, 0, 0.1]
        # ring of obstacles tightly surrounding the goal within a small bounded region
        tight_bounds = {"x_min": 0.0, "x_max": 0.6, "y_min": -0.3, "y_max": 0.3, "z_min": 0.0, "z_max": 0.3}
        obstacles = [
            Obstacle(position=[0.5 + 0.06, 0, 0.1], radius=0.08),
            Obstacle(position=[0.5 - 0.06, 0, 0.1], radius=0.08),
            Obstacle(position=[0.5, 0.06, 0.1], radius=0.08),
            Obstacle(position=[0.5, -0.06, 0.1], radius=0.08),
            Obstacle(position=[0.5, 0, 0.16], radius=0.08),
            Obstacle(position=[0.5, 0, 0.04], radius=0.08),
        ]
        path = plan_path(start, goal, obstacles, tight_bounds, clearance=0.0, max_iterations=300, rng=rng)
        assert path is None

    def test_deterministic_given_seeded_rng(self):
        start, goal = [0.1, 0, 0.1], [0.9, 0, 0.1]
        obstacles = [Obstacle(position=[0.5, 0, 0.1], radius=0.1)]

        path_a = plan_path(start, goal, obstacles, BOUNDS, clearance=0.02, rng=np.random.default_rng(42))
        path_b = plan_path(start, goal, obstacles, BOUNDS, clearance=0.02, rng=np.random.default_rng(42))

        assert path_a == path_b


class TestShortcutPath:
    def test_never_increases_waypoint_count(self):
        rng = np.random.default_rng(3)
        start, goal = [0.1, 0, 0.1], [0.9, 0, 0.1]
        obstacles = [Obstacle(position=[0.5, 0, 0.1], radius=0.1)]
        raw_path = plan_path(start, goal, obstacles, BOUNDS, clearance=0.02, rng=rng)

        smoothed = shortcut_path(raw_path, obstacles, clearance=0.02, rng=rng)

        assert len(smoothed) <= len(raw_path)

    def test_stays_collision_free(self):
        rng = np.random.default_rng(4)
        start, goal = [0.1, 0, 0.1], [0.9, 0, 0.1]
        obstacles = [Obstacle(position=[0.5, 0, 0.1], radius=0.1)]
        raw_path = plan_path(start, goal, obstacles, BOUNDS, clearance=0.02, rng=rng)

        smoothed = shortcut_path(raw_path, obstacles, clearance=0.02, rng=rng)

        assert path_is_collision_free(smoothed, obstacles, 0.02)

    def test_shortcuts_a_needlessly_zigzag_path_when_no_obstacle_blocks_directly(self):
        zigzag = [[0, 0, 0], [0.2, 0.2, 0], [0.4, -0.2, 0], [0.6, 0.2, 0], [1.0, 0, 0]]
        smoothed = shortcut_path(zigzag, [], clearance=0.0, iterations=200, rng=np.random.default_rng(5))
        assert len(smoothed) < len(zigzag)
        assert np.allclose(smoothed[0], zigzag[0])
        assert np.allclose(smoothed[-1], zigzag[-1])

    def test_handles_short_paths_without_error(self):
        assert shortcut_path([[0, 0, 0], [1, 0, 0]], []) == [[0, 0, 0], [1, 0, 0]]
        assert shortcut_path(None, []) is None


class TestFindBlockingObstacle:
    def test_returns_none_for_empty_obstacle_list(self):
        assert find_blocking_obstacle([0, 0, 0], [1, 0, 0], []) is None

    def test_picks_the_obstacle_closest_to_the_line(self):
        near = Obstacle(position=[0.5, 0.02, 0], radius=0.05, body_id=1)
        far = Obstacle(position=[0.5, 0.3, 0], radius=0.05, body_id=2)
        result = find_blocking_obstacle([0, 0, 0], [1, 0, 0], [far, near])
        assert result.body_id == 1
