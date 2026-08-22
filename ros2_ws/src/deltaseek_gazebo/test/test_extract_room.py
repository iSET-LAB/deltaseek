"""Tests for clipping a building manifest down to one room."""

import math

import pytest

from deltaseek_gazebo.extract_room import clip_element, extract


def _wall(center, rpy, size, name='wall'):
    return {
        'id': name, 'ifc_class': 'IfcWallStandardCase', 'name': name,
        'pose': {'xyz': list(center), 'rpy': list(rpy)},
        'geometry': {'type': 'box', 'size': list(size)},
        'material': {'rgba': [0.5, 0.5, 0.5, 1.0]},
        'static': True, 'collision': True,
    }


def test_a_long_wall_is_cut_to_the_room_and_keeps_its_thickness():
    # A 40 m wall through a 6 m room: the room takes 6 m of it, and the
    # 0.2 m thickness must survive untouched.
    wall = _wall((0.0, 0.0, 1.5), (0.0, 0.0, 0.0), (40.0, 0.2, 3.0))
    clipped = clip_element(wall, (-3.0, -1.0, 3.0, 1.0))
    assert clipped['geometry']['size'][0] == pytest.approx(6.0)
    assert clipped['geometry']['size'][1] == pytest.approx(0.2)
    assert clipped['pose']['xyz'][0] == pytest.approx(0.0)


def test_the_clip_follows_the_element_frame_not_the_world_axes():
    # The reason the importer emits oriented boxes: clipping a rotated wall
    # must not silently re-align or thicken it.
    yaw = math.radians(30.0)
    wall = _wall((0.0, 0.0, 1.5), (0.0, 0.0, yaw), (40.0, 0.2, 3.0))
    clipped = clip_element(wall, (-20.0, -20.0, 20.0, 20.0))
    assert clipped['pose']['rpy'][2] == pytest.approx(yaw)
    assert clipped['geometry']['size'][1] == pytest.approx(0.2)


def test_an_element_outside_the_room_is_dropped():
    wall = _wall((50.0, 0.0, 1.5), (0.0, 0.0, 0.0), (2.0, 0.2, 3.0))
    assert clip_element(wall, (-3.0, -1.0, 3.0, 1.0)) is None


def test_a_thin_element_survives_but_a_shaved_sliver_does_not():
    # A 0.12 m partition is genuinely thin and must be kept; a wall the room
    # boundary shaves down to 0.05 m is an artefact and must not be.
    partition = _wall((0.0, 0.0, 1.3), (0.0, 0.0, 0.0), (4.0, 0.12, 2.6))
    assert clip_element(partition, (-2.0, -1.0, 2.0, 1.0)) is not None

    grazing = _wall((0.0, 0.0, 1.3), (0.0, 0.0, 0.0), (4.0, 2.0, 2.6))
    assert clip_element(grazing, (-2.0, 0.95, 2.0, 4.0)) is None


def test_vertical_clipping_trims_the_element_and_recentres_it():
    slab = _wall((0.0, 0.0, 3.0), (0.0, 0.0, 0.0), (4.0, 4.0, 2.0))
    clipped = clip_element(slab, (-9.0, -9.0, 9.0, 9.0), z_range=(0.0, 2.5))
    assert clipped['geometry']['size'][2] == pytest.approx(0.5)
    assert clipped['pose']['xyz'][2] == pytest.approx(2.25)


def test_extract_records_the_room_and_renames_the_world():
    manifest = {
        'schema_version': 1,
        'world': {'name': 'storey', 'frame_id': 'world',
                  'gravity': [0.0, 0.0, -9.81]},
        'elements': [
            _wall((0.0, 0.0, 1.5), (0.0, 0.0, 0.0), (40.0, 0.2, 3.0), 'keep'),
            _wall((80.0, 0.0, 1.5), (0.0, 0.0, 0.0), (2.0, 0.2, 3.0), 'drop'),
        ],
    }
    room = extract(manifest, (-3.0, -1.0, 3.0, 1.0), room_id='hall')
    assert [e['id'] for e in room['elements']] == ['keep']
    assert room['world']['name'] == 'storey_hall'
    assert room['source']['room']['parent_elements'] == 2
    assert room['source']['room']['bounds'] == [-3.0, -1.0, 3.0, 1.0]
