"""Tests for viewpoint generation and the planners."""

import numpy as np
import pytest

from deltaseek_gazebo.planner import (
    Belief,
    BeliefParams,
    euclidean,
    plan_coverage,
    plan_deviation_seeking,
    plan_goal_directed,
)
from deltaseek_gazebo.viewpoints import Viewpoint, footprints, free_base_poses, is_blocked


def _element(element_id, xyz, size):
    return {
        'id': element_id,
        'ifc_class': 'IfcWall',
        'name': element_id,
        'pose': {'xyz': list(xyz), 'rpy': [0.0, 0.0, 0.0]},
        'geometry': {'type': 'box', 'size': list(size)},
        'material': {'rgba': [0.5, 0.5, 0.5, 1.0]},
        'static': True,
        'collision': True,
    }


FLOOR = _element('slab', [0.0, 0.0, -0.1], [18.0, 12.0, 0.2])
WALL = _element('wall', [0.0, 5.5, 1.5], [18.0, 0.2, 3.0])
PIPE = _element('pipe', [2.0, 0.0, 2.2], [0.24, 5.0, 0.24])


def test_floor_and_overhead_elements_do_not_block_the_base():
    assert footprints([FLOOR]) == []
    assert footprints([PIPE]) == []
    assert len(footprints([WALL])) == 1


def test_long_wall_blocks_a_rectangle_not_a_circle():
    # Regression: a circumscribing circle around an 18 m wall would sweep a
    # ~9 m radius and wrongly block the entire storey.
    blocked = footprints([WALL], inflation=0.55)
    assert is_blocked((0.0, 5.5), blocked) is True      # inside the wall
    assert is_blocked((0.0, 0.0), blocked) is False     # 5.5 m to the side
    assert is_blocked((8.0, 0.0), blocked) is False     # along its length


def test_free_base_poses_avoid_obstacles_and_respect_yaws():
    poses = free_base_poses(
        [FLOOR, WALL], ((-4.0, 4.0), (-3.0, 3.0)), spacing=2.0,
        yaws=(0.0, 1.57))
    assert poses
    assert all(not is_blocked(xy, footprints([FLOOR, WALL]))
               for xy, _ in poses)
    yaws = {yaw for _, yaw in poses}
    assert yaws == {0.0, 1.57}


def _viewpoint(x, y):
    return Viewpoint(base_xy=(x, y), base_yaw=0.0,
                     camera_xyz=(x, y, 1.0), camera_rpy=(0.0, 0.0, 0.0))


def test_resolution_ramp_ignores_glimpses_and_saturates():
    params = BeliefParams(min_fraction=0.05, full_fraction=0.35)
    assert params.resolution([0.0])[0] == 0.0
    assert params.resolution([0.04])[0] == 0.0
    assert params.resolution([0.5])[0] == 1.0
    assert 0.0 < params.resolution([0.2])[0] < 1.0


def test_observing_an_element_reduces_later_gain():
    belief = Belief(2)
    fractions = np.array([1.0, 0.0])
    first = belief.gain(fractions)
    belief.update(fractions)
    assert belief.gain(fractions) < first
    assert belief.gain(fractions) == pytest.approx(0.0, abs=1e-9)


def test_deviation_seeking_prefers_the_informative_viewpoint():
    viewpoints = [_viewpoint(1.0, 0.0), _viewpoint(1.2, 0.0)]
    # The second viewpoint sees both elements, the first only one.
    matrix = np.array([[1.0, 0.0], [1.0, 1.0]])
    order, _ = plan_deviation_seeking(viewpoints, matrix, budget=50.0)
    assert order[0] == 1


def test_planners_respect_the_traversal_budget():
    viewpoints = [_viewpoint(float(x), 0.0) for x in range(0, 40, 2)]
    matrix = np.ones((len(viewpoints), 3))
    for planner, kwargs in (
        (plan_deviation_seeking, {'matrix': matrix}),
        (plan_coverage, {}),
        (plan_goal_directed, {}),
    ):
        if 'matrix' in kwargs:
            _, info = planner(viewpoints, kwargs['matrix'], budget=10.0)
        else:
            _, info = planner(viewpoints, budget=10.0)
        assert info['distance'] <= 10.0 + 1e-9


def test_zero_budget_yields_no_plan():
    viewpoints = [_viewpoint(5.0, 0.0), _viewpoint(9.0, 0.0)]
    matrix = np.ones((2, 2))
    order, _ = plan_deviation_seeking(viewpoints, matrix, budget=0.0)
    assert order == []


def test_euclidean_ignores_orientation():
    assert euclidean((0.0, 0.0), (3.0, 4.0)) == pytest.approx(5.0)
