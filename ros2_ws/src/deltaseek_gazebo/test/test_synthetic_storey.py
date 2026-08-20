"""Tests for the synthetic storey generator."""

import pytest

from deltaseek_gazebo.benchmark import validate_manifest
from deltaseek_gazebo.synthetic_storey import DOOR_HEIGHT, _parser, build_storey
from deltaseek_gazebo.viewpoints import free_base_poses, footprints, is_blocked, scene_bounds


def _args(*argv):
    return _parser().parse_args(['--output', 'unused', *argv])


@pytest.fixture()
def storey():
    return build_storey(_args('--seed', '1'))


def test_manifest_is_valid_and_ids_are_unique(storey):
    validated = validate_manifest(storey)
    ids = [element['id'] for element in validated['elements']]
    assert len(ids) == len(set(ids))
    assert len(ids) > 50


def test_generation_is_reproducible_and_seed_sensitive():
    first = build_storey(_args('--seed', '3'))
    again = build_storey(_args('--seed', '3'))
    other = build_storey(_args('--seed', '4'))
    assert first == again
    assert first['elements'] != other['elements']


def test_scene_scales_with_room_count():
    small = build_storey(_args('--rooms-per-side', '2'))
    large = build_storey(_args('--rooms-per-side', '5'))
    assert len(large['elements']) > len(small['elements'])


def test_doorways_are_real_openings(storey):
    """A door must be a gap in the geometry, not a thinner wall.

    This is the property the bounding-box IFC adapter cannot preserve, and the
    reason the scene is built from primitives instead of imported.
    """
    args = _args('--seed', '1')
    door_x = args.room_width / 2.0          # centre of the first north room
    corridor_face = args.corridor_width / 2.0

    spanning = []
    for element in storey['elements']:
        if element['ifc_class'] != 'IfcWall':
            continue
        x, y, z = element['pose']['xyz']
        sx, sy, sz = element['geometry']['size']
        if abs(y - corridor_face) > 0.01:
            continue
        if abs(x - door_x) > sx / 2.0:
            continue
        # Anything overlapping the doorway below head height blocks it.
        if z - sz / 2.0 < DOOR_HEIGHT - 0.01:
            spanning.append(element['name'])
    assert spanning == [], f'doorway is blocked by {spanning}'


def test_a_lintel_spans_above_each_doorway(storey):
    lintels = [e for e in storey['elements'] if e['name'].endswith('lintel')]
    assert lintels
    for lintel in lintels:
        bottom = lintel['pose']['xyz'][2] - lintel['geometry']['size'][2] / 2.0
        assert bottom >= DOOR_HEIGHT - 1e-6


def test_room_interiors_are_reachable(storey):
    """Rooms must offer standing room, or nothing can be inspected inside."""
    elements = validate_manifest(storey)['elements']
    poses = free_base_poses(
        elements, scene_bounds(elements), spacing=1.0, yaws=(0.0,))
    assert poses
    args = _args()
    corridor_half = args.corridor_width / 2.0
    inside_rooms = [
        xy for xy, _ in poses if abs(xy[1]) > corridor_half + 0.5
    ]
    assert inside_rooms, 'no base pose falls inside any room'


def test_slab_and_overhead_services_are_not_obstacles(storey):
    """The robot drives on the slab and beneath the services."""
    elements = validate_manifest(storey)['elements']

    slabs = [e for e in elements if e['ifc_class'] == 'IfcSlab']
    assert slabs
    assert footprints(slabs) == []

    overhead = [
        e for e in elements
        if e['ifc_class'] == 'IfcFlowSegment'
        and e['pose']['xyz'][2] - e['geometry']['size'][2] / 2.0 >= 1.0
    ]
    assert overhead
    assert footprints(overhead) == []


def test_walls_do_block_driving(storey):
    elements = validate_manifest(storey)['elements']
    walls = [e for e in elements if e['ifc_class'] == 'IfcWall']
    blocked = footprints(walls)
    assert blocked
    # A point in the middle of a wall panel must be rejected.
    panel = next(e for e in walls if e['name'].endswith('panel'))
    assert is_blocked(panel['pose']['xyz'][:2], blocked)
