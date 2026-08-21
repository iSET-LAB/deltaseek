"""Tests for the IFC importer's geometry handling.

IfcOpenShell lives in a separate Conda environment and is not importable from
the ROS test runner, so these cover the pure-geometry half: recovering
roll-pitch-yaw from a placement, and reading the placement matrix layout.
``ifc_to_manifest`` imports IfcOpenShell lazily for exactly this reason.
"""

import math

import numpy as np

from deltaseek_gazebo.ifc_to_manifest import _placement, _rpy_from_matrix
from deltaseek_gazebo.visibility import rotation_matrix


def _round_trip(rpy):
    return _rpy_from_matrix(rotation_matrix(rpy))


def test_rpy_round_trips_through_the_rotation_used_downstream():
    for rpy in [
        (0.0, 0.0, 0.0),
        (0.0, 0.0, -0.0157),          # the ~0.9 deg site rotation seen in Revit
        (0.0, 0.0, math.pi),
        (0.3, -0.4, 1.2),
        (-1.1, 0.7, -2.4),
    ]:
        recovered = _round_trip(rpy)
        np.testing.assert_allclose(
            rotation_matrix(recovered), rotation_matrix(rpy), atol=1e-12)


def test_gimbal_lock_still_yields_the_requested_orientation():
    # Pitch at +/- 90 deg makes roll and yaw degenerate; the angles may differ
    # from the input but the orientation they describe must not.
    for pitch in (math.pi / 2.0, -math.pi / 2.0):
        rpy = (0.6, pitch, -0.9)
        np.testing.assert_allclose(
            rotation_matrix(_round_trip(rpy)), rotation_matrix(rpy), atol=1e-9)


def test_placement_reads_a_column_major_matrix():
    class _Shape:
        class transformation:
            # IfcOpenShell hands back 16 values, column major, translation last.
            matrix = (0.0, 1.0, 0.0, 0.0,
                      -1.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 1.0, 0.0,
                      2.0, 3.0, 4.0, 1.0)

    rotation, translation = _placement(_Shape())
    assert translation == [2.0, 3.0, 4.0]
    np.testing.assert_allclose(rotation, [[0.0, -1.0, 0.0],
                                          [1.0, 0.0, 0.0],
                                          [0.0, 0.0, 1.0]])
    # A quarter turn about +z, expressed the way the manifest stores it.
    np.testing.assert_allclose(
        _rpy_from_matrix(rotation), [0.0, 0.0, math.pi / 2.0], atol=1e-12)
