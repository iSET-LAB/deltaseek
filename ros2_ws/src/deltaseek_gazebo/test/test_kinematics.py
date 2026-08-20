"""Tests for URDF-driven forward kinematics."""

import math

import numpy as np
import pytest

from deltaseek_gazebo.kinematics import Chain, base_pose, pose_to_xyz_rpy


URDF = """<?xml version="1.0"?>
<robot name="test">
  <link name="base"/>
  <link name="shoulder"/>
  <link name="arm"/>
  <link name="tip"/>
  <joint name="fixed_riser" type="fixed">
    <parent link="base"/><child link="shoulder"/>
    <origin xyz="0 0 1" rpy="0 0 0"/>
  </joint>
  <joint name="yaw_joint" type="revolute">
    <parent link="shoulder"/><child link="arm"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.5" upper="1.5"/>
  </joint>
  <joint name="reach" type="fixed">
    <parent link="arm"/><child link="tip"/>
    <origin xyz="2 0 0" rpy="0 0 0"/>
  </joint>
</robot>
"""


@pytest.fixture()
def chain(tmp_path):
    path = tmp_path / 'test.urdf'
    path.write_text(URDF, encoding='utf-8')
    return Chain.from_urdf(path, 'base', 'tip')


def test_only_moving_joints_are_exposed(chain):
    assert chain.joint_names == ['yaw_joint']


def test_zero_configuration_applies_fixed_offsets(chain):
    pose = chain.forward({})
    assert np.allclose(pose[:3, 3], [2.0, 0.0, 1.0])


def test_revolute_joint_rotates_the_tip(chain):
    pose = chain.forward({'yaw_joint': math.pi / 2.0})
    assert np.allclose(pose[:3, 3], [0.0, 2.0, 1.0], atol=1e-9)


def test_joint_limits_are_read_from_the_urdf(chain):
    assert chain.within_limits([0.5]) is True
    assert chain.within_limits([2.0]) is False


def test_missing_link_is_reported(tmp_path):
    path = tmp_path / 'test.urdf'
    path.write_text(URDF, encoding='utf-8')
    with pytest.raises(ValueError, match='no path'):
        Chain.from_urdf(path, 'nonexistent', 'tip')


def test_pose_round_trip_through_xyz_rpy(chain):
    pose = chain.forward({'yaw_joint': 0.7})
    xyz, rpy = pose_to_xyz_rpy(pose)
    assert np.allclose(xyz, pose[:3, 3])
    assert rpy[2] == pytest.approx(0.7)


def test_base_pose_composes_with_the_chain(chain):
    # A base rotated 90 degrees turns the arm's +x reach into world +y.
    world = base_pose([1.0, 0.0], math.pi / 2.0) @ chain.forward({})
    assert np.allclose(world[:3, 3], [1.0, 2.0, 1.0], atol=1e-9)
