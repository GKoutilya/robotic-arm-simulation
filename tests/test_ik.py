import numpy as np
import pybullet as p
import pybullet_data
import pytest

from src.control.inverse_kinematics import calculate_ik
from src.config import EE_LINK


@pytest.fixture
def robot_id():
    """Headless PyBullet connection with the KUKA IIWA loaded, torn down after the test."""
    client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")
    rid = p.loadURDF("kuka_iiwa/model.urdf", basePosition=[0, 0, 0], useFixedBase=True)
    yield rid
    p.disconnect(client)


def test_ee_link_index_is_valid_for_the_loaded_robot(robot_id):
    assert 0 <= EE_LINK < p.getNumJoints(robot_id)


def test_calculate_ik_returns_a_joint_angle_per_joint(robot_id):
    target_pos = [0.5, 0.0, 0.5]
    joint_angles = calculate_ik(robot_id, target_pos, end_effector_index=EE_LINK)

    assert joint_angles is not None
    assert len(joint_angles) == p.getNumJoints(robot_id)


def test_calculate_ik_solution_reaches_target_within_tolerance(robot_id):
    target_pos = [0.5, 0.0, 0.5]
    target_orn = p.getQuaternionFromEuler([np.pi, 0, 0])

    joint_angles = calculate_ik(robot_id, target_pos, target_orn, end_effector_index=EE_LINK)
    assert joint_angles is not None

    for i, angle in enumerate(joint_angles):
        p.resetJointState(robot_id, i, angle)

    ee_state = p.getLinkState(robot_id, EE_LINK)
    actual_pos = np.array(ee_state[4])

    assert np.linalg.norm(actual_pos - np.array(target_pos)) < 0.02


def test_calculate_ik_returns_none_for_invalid_target_pos(robot_id):
    assert calculate_ik(robot_id, None, end_effector_index=EE_LINK) is None
