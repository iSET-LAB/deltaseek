"""Tests for traversal cost that routes around obstacles."""

import math

import pytest

from deltaseek_gazebo.navigation import CostGrid, PathCost, path_cost_for
from deltaseek_gazebo.planner import euclidean
from deltaseek_gazebo.viewpoints import ROBOT_RADIUS


def _element(element_id, xyz, size, ifc_class='IfcWall'):
    return {
        'id': element_id,
        'ifc_class': ifc_class,
        'name': element_id,
        'pose': {'xyz': list(xyz), 'rpy': [0.0, 0.0, 0.0]},
        'geometry': {'type': 'box', 'size': list(size)},
        'material': {'rgba': [0.5, 0.5, 0.5, 1.0]},
        'static': True,
        'collision': True,
    }


# A room bisected by a wall that stops short of the east end, leaving a gap.
FLOOR = _element('floor', [0.0, 0.0, -0.1], [24.0, 24.0, 0.2], 'IfcSlab')
BAFFLE = _element('baffle', [0.0, 0.0, 1.5], [0.2, 16.0, 3.0])


def test_robot_radius_matches_the_platform_footprint():
    # Clearpath's a300 Nav2 footprint is 0.495 x 0.349 half-extents.
    assert ROBOT_RADIUS == pytest.approx(math.hypot(0.495, 0.349), abs=1e-9)


def test_floor_is_drivable_and_walls_are_not():
    grid = CostGrid([FLOOR, BAFFLE])
    assert grid.is_free((5.0, 0.0))
    assert not grid.is_free((0.0, 0.0))


def test_staying_put_costs_nothing():
    """A plan revisits one base pose from many arm postures.

    Any per-step offset here accumulates over hundreds of viewpoints and
    silently inflates the reported traversal budget.
    """
    cost = path_cost_for([FLOOR, BAFFLE])
    for point in ((5.0, 0.0), (-5.0, 3.0), (7.25, -2.5)):
        assert cost(point, point) == 0.0


def test_a_detour_costs_more_than_the_straight_line():
    cost = path_cost_for([FLOOR, BAFFLE])
    west, east = (-3.0, 0.0), (3.0, 0.0)
    straight = euclidean(west, east)
    around = cost(west, east)
    assert math.isfinite(around)
    # The baffle spans y in [-8, 8]; getting past it is far longer than 6 m.
    assert around > straight * 2.0


def test_cost_never_undercuts_the_straight_line():
    cost = path_cost_for([FLOOR, BAFFLE])
    points = [(-6.0, -6.0), (-2.0, 4.0), (4.0, -3.0), (9.0, 7.0), (2.0, 9.5)]
    for a in points:
        for b in points:
            value = cost(a, b)
            if math.isfinite(value):
                assert value >= euclidean(a, b) - 1e-6


def test_unreachable_target_is_infinite():
    # A sealed box: nothing inside connects to the rest of the floor.
    walls = [
        _element('n', [0.0, 2.0, 1.5], [4.4, 0.2, 3.0]),
        _element('s', [0.0, -2.0, 1.5], [4.4, 0.2, 3.0]),
        _element('e', [2.0, 0.0, 1.5], [0.2, 4.4, 3.0]),
        _element('w', [-2.0, 0.0, 1.5], [0.2, 4.4, 3.0]),
    ]
    cost = path_cost_for([FLOOR] + walls)
    assert cost((0.0, 0.0), (8.0, 0.0)) == math.inf


def test_repeated_queries_reuse_the_source_field():
    cost = path_cost_for([FLOOR, BAFFLE])
    first = cost((-5.0, 0.0), (-5.0, 4.0))
    again = cost((-5.0, 0.0), (-5.0, 4.0))
    assert first == again
    assert len(cost._fields) == 1
