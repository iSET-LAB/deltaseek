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


# AMP enclosure mesh extents, measured from observer_enclosure.stl. Its top
# face is the deck the chassis sensors stand on.
DECK_Z, DECK_HALF_X, DECK_HALF_Y = 0.407, 0.486, 0.226


@pytest.mark.parametrize('link', ['camera_1_link', 'lidar3d_0_link'])
def test_chassis_sensors_stand_on_the_enclosure_deck(urdf, link):
    """Both must sit on the robot, not float beside it.

    An earlier revision bracketed them at y = 0.30, which is 0.074 m past the
    enclosure side face: inside the navigation footprint, and therefore
    invisible to every other check, but hanging in mid-air next to the body.
    Only a test against the enclosure's own extents catches that.
    """
    xyz = Chain.from_urdf(urdf, 'base_link', link).forward()[:3, 3]
    assert abs(xyz[0]) <= DECK_HALF_X, f'{link} overhangs the deck in x'
    assert abs(xyz[1]) <= DECK_HALF_Y, f'{link} overhangs the deck in y'
    assert DECK_Z - 0.01 <= xyz[2] <= DECK_Z + 0.25, (
        f'{link} should rest on the deck at z={DECK_Z}, found {xyz[2]:.3f}')


def test_chassis_sensors_clear_the_arm_in_every_posture(urdf):
    arm_links = ['arm_0_shoulder_link', 'arm_0_upper_arm_link',
                 'arm_0_forearm_link', 'arm_0_wrist_1_link',
                 'arm_0_wrist_2_link', 'arm_0_wrist_3_link']
    chains = {name: Chain.from_urdf(urdf, 'base_link', name)
              for name in arm_links}
    for link in ('camera_1_link', 'lidar3d_0_link'):
        sensor = Chain.from_urdf(urdf, 'base_link', link).forward()[:3, 3]
        for values in ARM_POSTURES.values():
            joints = dict(zip(chains[arm_links[0]].joint_names, values))
            for name, chain in chains.items():
                gap = np.linalg.norm(chain.forward(joints)[:3, 3] - sensor)
                # Link origins only; real clearance is smaller by the link
                # radii, so this is a floor rather than the true gap.
                assert gap > 0.15, f'{link} is {gap:.3f} m from {name}'


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
