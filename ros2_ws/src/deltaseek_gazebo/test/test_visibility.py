"""Tests for the geometric visibility and detection model."""

import math

import numpy as np
import pytest

from deltaseek_gazebo.detection import (
    ObservationParams,
    Scene,
    detect_along,
    observe_elements,
)
from deltaseek_gazebo.visibility import Camera, OrientedBox, visible_fraction


def _element(element_id, xyz, size, rpy=(0.0, 0.0, 0.0)):
    return {
        'id': element_id,
        'ifc_class': 'IfcWall',
        'name': element_id,
        'pose': {'xyz': list(xyz), 'rpy': list(rpy)},
        'geometry': {'type': 'box', 'size': list(size)},
        'material': {'rgba': [0.5, 0.5, 0.5, 1.0]},
        'static': True,
        'collision': True,
    }


def _camera_at(x, yaw=0.0, **kwargs):
    return Camera.from_pose([x, 0.0, 1.0], [0.0, 0.0, yaw], **kwargs)


def test_box_in_front_is_visible_and_behind_is_not():
    box = OrientedBox([2.0, 0.0, 1.0], [0, 0, 0], [0.5, 0.5, 0.5])
    assert visible_fraction(box, _camera_at(0.0), []) > 0.0
    # Same distance, opposite side: the frustum only opens along +x.
    assert visible_fraction(box, _camera_at(4.0), []) == 0.0


def test_element_beyond_usable_range_is_not_observed():
    box = OrientedBox([5.0, 0.0, 1.0], [0, 0, 0], [0.5, 0.5, 0.5])
    assert visible_fraction(box, _camera_at(0.0, far=6.0), []) > 0.0
    assert visible_fraction(box, _camera_at(0.0, far=3.0), []) == 0.0


def test_occluder_blocks_the_element_behind_it():
    target = OrientedBox([3.0, 0.0, 1.0], [0, 0, 0], [0.5, 0.5, 0.5])
    camera = _camera_at(0.0)
    clear = visible_fraction(target, camera, [])
    assert clear > 0.0

    wall = OrientedBox([1.5, 0.0, 1.0], [0, 0, 0], [0.1, 4.0, 3.0])
    assert visible_fraction(target, camera, [wall]) == 0.0


def test_grazing_incidence_is_rejected():
    # A thin slab seen almost edge-on returns no usable surface.
    slab = OrientedBox([2.0, 0.0, 1.0], [0, 0, 0], [4.0, 0.02, 3.0])
    camera = Camera.from_pose([2.0, -2.0, 1.0], [0.0, 0.0, math.pi / 2.0])
    head_on = visible_fraction(slab, camera, [])
    edge_on = visible_fraction(
        slab, camera, [], max_incidence=math.radians(1.0))
    assert head_on > 0.0
    assert edge_on < head_on


def test_visible_fraction_is_bounded_and_deterministic():
    box = OrientedBox([2.0, 0.0, 1.0], [0, 0, 0.3], [0.6, 0.4, 0.8])
    camera = _camera_at(0.0)
    first = visible_fraction(box, camera, [])
    second = visible_fraction(box, camera, [])
    assert first == second
    assert 0.0 <= first <= 1.0


def test_observe_elements_excludes_self_occlusion():
    scene = Scene.from_elements([
        _element('near', [2.0, 0.0, 1.0], [0.5, 0.5, 0.5]),
        _element('far', [4.0, 0.0, 1.0], [0.5, 0.5, 0.5]),
    ])
    fractions = observe_elements(_camera_at(0.0), scene)
    assert fractions['near'] > 0.0
    # 'far' sits directly behind 'near' but is wider than the shadow it casts.
    assert fractions['far'] < fractions['near']


def _ground_truth(discrepancies):
    return {'schema_version': 1, 'scenario_id': 't', 'discrepancies': discrepancies}


def test_missing_element_is_detected_by_seeing_the_empty_volume():
    nominal = [_element('beam', [3.0, 0.0, 1.0], [0.5, 0.5, 0.5])]
    actual = []
    truth = _ground_truth([
        {'id': 'd0', 'type': 'missing', 'element_id': 'beam', 'severity': 0.5},
    ])
    records = detect_along([([0.0, 0.0, 1.0], [0.0, 0.0, 0.0])],
                           nominal, actual, truth)
    assert records[0]['detected'] is True
    assert records[0]['evidence'] == 'nominal'


def test_deviation_outside_the_frustum_is_not_detected():
    nominal = [_element('beam', [3.0, 0.0, 1.0], [0.5, 0.5, 0.5])]
    actual = [_element('beam', [3.0, 0.0, 1.0], [0.5, 0.5, 0.5])]
    truth = _ground_truth([
        {'id': 'd0', 'type': 'rotated', 'element_id': 'beam', 'severity': 0.5},
    ])
    # Facing away from the element.
    records = detect_along([([0.0, 0.0, 1.0], [0.0, 0.0, math.pi])],
                           nominal, actual, truth)
    assert records[0]['detected'] is False
    assert records[0]['first_viewpoint'] is None


def test_detection_records_the_first_viewpoint_that_sees_it():
    nominal = [_element('beam', [3.0, 0.0, 1.0], [0.6, 0.6, 0.6])]
    actual = [_element('beam', [3.0, 0.0, 1.0], [0.6, 0.6, 0.6])]
    truth = _ground_truth([
        {'id': 'd0', 'type': 'resized', 'element_id': 'beam', 'severity': 0.5},
    ])
    viewpoints = [
        ([0.0, 0.0, 1.0], [0.0, 0.0, math.pi]),   # looking away
        ([0.0, 0.0, 1.0], [0.0, 0.0, math.pi]),   # still looking away
        ([0.0, 0.0, 1.0], [0.0, 0.0, 0.0]),       # now facing it
    ]
    records = detect_along(viewpoints, nominal, actual, truth)
    assert records[0]['first_viewpoint'] == 2


def test_threshold_controls_sensitivity():
    nominal = [_element('beam', [5.5, 0.0, 1.0], [0.3, 0.3, 0.3])]
    actual = [_element('beam', [5.5, 0.0, 1.0], [0.3, 0.3, 0.3])]
    truth = _ground_truth([
        {'id': 'd0', 'type': 'rotated', 'element_id': 'beam', 'severity': 0.5},
    ])
    viewpoints = [([0.0, 0.0, 1.0], [0.0, 0.0, 0.0])]
    lenient = detect_along(viewpoints, nominal, actual, truth,
                           ObservationParams(min_visible_fraction=0.01))
    strict = detect_along(viewpoints, nominal, actual, truth,
                          ObservationParams(min_visible_fraction=0.99))
    assert lenient[0]['detected'] is True
    assert strict[0]['detected'] is False
