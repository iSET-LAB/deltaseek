"""Tests for multi-sensor observation.

The sensor ablation only means anything if the chassis camera really is
independent of the arm.  If a refactor ever routes its pose through forward
kinematics over the manipulator, every ablation number silently becomes a
statement about the arm again, and nothing else in the suite would notice.
"""

import math

import numpy as np
import pytest

from deltaseek_gazebo.detection import (
    CHASSIS, WRIST, ObservationParams, normalise_station, sensor_params)
from deltaseek_gazebo.kinematics import Chain
from deltaseek_gazebo.viewpoints import (
    ARM_POSTURES, CHASSIS_CAMERA_LINK, build_viewpoints, chassis_chain)
from deltaseek_gazebo.compare import resolve_urdf


@pytest.fixture(scope='module')
def urdf():
    try:
        return resolve_urdf(None)
    except Exception as exc:                      # pragma: no cover
        pytest.skip(f'generated robot description unavailable: {exc}')


def test_chassis_pose_is_invariant_across_every_arm_posture(urdf):
    chain = Chain.from_urdf(urdf, 'base_link', 'camera_0_link')
    poses = build_viewpoints(
        chain, [((1.0, 2.0), 0.7)], chassis=chassis_chain(urdf))
    assert len(poses) == len(ARM_POSTURES)

    chassis = [viewpoint.sensor_poses[CHASSIS] for viewpoint in poses]
    reference = chassis[0]
    for xyz, rpy in chassis[1:]:
        np.testing.assert_allclose(xyz, reference[0], atol=1e-12)
        np.testing.assert_allclose(rpy, reference[1], atol=1e-12)

    # ... while the wrist camera genuinely moves, or the test above would pass
    # for the trivial reason that nothing moves at all.
    wrist = {tuple(np.round(vp.sensor_poses[WRIST][0], 6)) for vp in poses}
    assert len(wrist) > 1


def test_chassis_pose_follows_the_base(urdf):
    chain = Chain.from_urdf(urdf, 'base_link', 'camera_0_link')
    arch = chassis_chain(urdf)
    at_origin = build_viewpoints(chain, [((0.0, 0.0), 0.0)], chassis=arch)
    shifted = build_viewpoints(chain, [((3.0, -1.0), 0.0)], chassis=arch)
    a = np.array(at_origin[0].sensor_poses[CHASSIS][0])
    b = np.array(shifted[0].sensor_poses[CHASSIS][0])
    np.testing.assert_allclose(b - a, [3.0, -1.0, 0.0], atol=1e-9)


def test_chassis_camera_sits_below_the_arm_and_on_the_platform(urdf):
    xyz = chassis_chain(urdf).forward()[:3, 3]
    assert 0.2 < xyz[2] < 0.45, 'chassis camera should be at platform height'
    assert abs(xyz[1]) < 0.349, 'must stay inside the navigation footprint'


def test_chassis_link_name_exists_in_the_description(urdf):
    # A missing link would otherwise surface as an opaque failure deep in FK.
    Chain.from_urdf(urdf, 'base_link', CHASSIS_CAMERA_LINK)


def test_a_bare_pose_is_read_as_the_wrist_camera():
    station = normalise_station(([1.0, 2.0, 3.0], [0.0, 0.0, 0.0]))
    assert set(station) == {WRIST}


def test_a_mapping_station_is_passed_through():
    station = {CHASSIS: ([0.0] * 3, [0.0] * 3), WRIST: ([1.0] * 3, [0.0] * 3)}
    assert normalise_station(station) == station


def test_per_sensor_params_resolve_and_missing_ones_raise():
    wide = ObservationParams(far=20.0)
    narrow = ObservationParams(far=5.0)
    table = {CHASSIS: wide, WRIST: narrow}
    assert sensor_params(table, CHASSIS).far == pytest.approx(20.0)
    assert sensor_params(table, WRIST).far == pytest.approx(5.0)
    with pytest.raises(KeyError):
        sensor_params(table, 'nonexistent')
    # A single params object applies to every sensor.
    assert sensor_params(narrow, CHASSIS) is narrow
