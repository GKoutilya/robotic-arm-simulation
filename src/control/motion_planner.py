"""
Task-space collision-aware motion planning: a goal-biased RRT over 3D
end-effector positions, treating obstacles as inflated bounding spheres.

Pure Python/numpy -- no PyBullet dependency -- so it's fast, deterministic
when seeded, and fully unit-testable headless (see tests/test_motion_planner.py).
"""
import numpy as np
from dataclasses import dataclass


@dataclass
class Obstacle:
    position: list   # [x, y, z]
    radius: float
    body_id: int = None  # opaque metadata for the caller; unused in collision math


def _closest_point_on_segment(p1, p2, point):
    """Point on segment p1->p2 closest to `point`, via clamped projection."""
    p1 = np.asarray(p1, dtype=np.float64)
    p2 = np.asarray(p2, dtype=np.float64)
    point = np.asarray(point, dtype=np.float64)

    seg = p2 - p1
    seg_len_sq = np.dot(seg, seg)
    if seg_len_sq < 1e-12:
        return p1

    t = np.dot(point - p1, seg) / seg_len_sq
    t = np.clip(t, 0.0, 1.0)
    return p1 + t * seg


def is_segment_clear(p1, p2, obstacles, clearance=0.0):
    """True if the straight segment p1->p2 stays outside every obstacle's
    (radius + clearance)."""
    for obs in obstacles:
        closest = _closest_point_on_segment(p1, p2, obs.position)
        dist = np.linalg.norm(closest - np.asarray(obs.position, dtype=np.float64))
        if dist < obs.radius + clearance:
            return False
    return True


def _random_point(bounds, rng):
    return np.array([
        rng.uniform(bounds["x_min"], bounds["x_max"]),
        rng.uniform(bounds["y_min"], bounds["y_max"]),
        rng.uniform(bounds["z_min"], bounds["z_max"]),
    ])


def plan_path(start, goal, obstacles, bounds, clearance=0.0, step_size=0.05,
              max_iterations=1500, goal_bias=0.1, rng=None):
    """
    Goal-biased single-tree RRT from `start` to `goal` within `bounds`
    (dict with x_min/x_max/y_min/y_max/z_min/z_max), avoiding `obstacles`.

    Returns a waypoint list [start, ..., goal] or None if no path is found
    within max_iterations.
    """
    if rng is None:
        rng = np.random.default_rng()

    start = np.asarray(start, dtype=np.float64)
    goal = np.asarray(goal, dtype=np.float64)

    if is_segment_clear(start, goal, obstacles, clearance):
        return [start.tolist(), goal.tolist()]

    nodes = [start]
    parents = [-1]

    for _ in range(max_iterations):
        sample = goal if rng.uniform(0, 1) < goal_bias else _random_point(bounds, rng)

        # nearest node in the tree
        dists = [np.linalg.norm(sample - n) for n in nodes]
        nearest_idx = int(np.argmin(dists))
        nearest = nodes[nearest_idx]

        direction = sample - nearest
        dist = np.linalg.norm(direction)
        if dist < 1e-9:
            continue
        new_node = nearest + direction / dist * min(step_size, dist)

        if not is_segment_clear(nearest, new_node, obstacles, clearance):
            continue

        nodes.append(new_node)
        parents.append(nearest_idx)

        if np.linalg.norm(new_node - goal) < step_size and is_segment_clear(new_node, goal, obstacles, clearance):
            nodes.append(goal)
            parents.append(len(nodes) - 2)

            # walk back from goal to start
            path = []
            idx = len(nodes) - 1
            while idx != -1:
                path.append(nodes[idx].tolist())
                idx = parents[idx]
            path.reverse()
            return path

    return None


def shortcut_path(path, obstacles, clearance=0.0, iterations=100, rng=None):
    """
    Iteratively try to splice out intermediate waypoints when a direct
    segment between two (non-adjacent) points in the path is collision-free.
    Never lengthens the path (in waypoint count) and always stays collision-free.
    """
    if path is None or len(path) <= 2:
        return path

    if rng is None:
        rng = np.random.default_rng()

    path = [np.asarray(p, dtype=np.float64) for p in path]

    for _ in range(iterations):
        if len(path) <= 2:
            break
        i, j = sorted(rng.choice(len(path), size=2, replace=False))
        if j - i <= 1:
            continue
        if is_segment_clear(path[i], path[j], obstacles, clearance):
            path = path[:i + 1] + path[j:]

    return [p.tolist() for p in path]


def find_blocking_obstacle(start, goal, obstacles):
    """Return the obstacle whose center is closest to the straight
    start->goal segment (used to pick which object to relocate when
    planning fails). None if `obstacles` is empty."""
    if not obstacles:
        return None

    best_obs, best_dist = None, None
    for obs in obstacles:
        closest = _closest_point_on_segment(start, goal, obs.position)
        dist = np.linalg.norm(closest - np.asarray(obs.position, dtype=np.float64))
        if best_dist is None or dist < best_dist:
            best_obs, best_dist = obs, dist

    return best_obs
