"""Clip a building manifest down to a single room.

A whole storey is the wrong unit for studying how a robot decides what it is
looking at.  Its elements are building-scale -- one wall entity runs the full
39 m of a facade, one slab is the whole floor -- so a room cut out of it by
membership alone still drags the entire building in with it.

Clipping happens in each element's own frame rather than in world axes.  That
keeps the 0.9 deg site rotation intact: a wall clipped in world axes would come
back axis-aligned and thickened, which is the error ``ifc_to_manifest`` exists
to avoid.  Because the room box is nearly aligned with the elements, the local
clip is tight; where it is not, it errs by keeping material, never by removing
material the robot would have collided with.
"""

import argparse
import math
from pathlib import Path

import yaml

from deltaseek_gazebo.benchmark import SCHEMA_VERSION, validate_manifest


def _rotation_2d(rpy):
    yaw = float(rpy[2])
    cos, sin = math.cos(yaw), math.sin(yaw)
    return ((cos, -sin), (sin, cos))


def clip_element(element, bounds, z_range=None, minimum_extent=0.2):
    """Return the element trimmed to ``bounds``, or None if it falls outside.

    ``bounds`` is ``(x0, y0, x1, y1)`` in world coordinates.  Only box geometry
    is clipped; anything else is kept whole when its centre is inside, because
    trimming a cylinder or a mesh is not a box operation.  ``minimum_extent``
    discards elements the clip reduced to a sliver, judged against what the
    clip removed rather than against the surviving size, so genuinely thin
    elements are kept.
    """
    x0, y0, x1, y1 = (float(value) for value in bounds)
    x0, x1 = min(x0, x1), max(x0, x1)
    y0, y1 = min(y0, y1), max(y0, y1)
    center = [float(value) for value in element['pose']['xyz']]
    inside_xy = x0 <= center[0] <= x1 and y0 <= center[1] <= y1

    if element['geometry']['type'] != 'box':
        return dict(element) if inside_xy else None

    rotation = _rotation_2d(element['pose']['rpy'])
    size = [float(value) for value in element['geometry']['size']]
    half = [value / 2.0 for value in size]

    # Express the room rectangle in the element's own frame and keep the part
    # of the element that survives.  Rotation about z only: x and y mix, z does
    # not, so the vertical extent is handled separately.
    corners = ((x0, y0), (x1, y0), (x1, y1), (x0, y1))
    local = []
    for corner in corners:
        dx, dy = corner[0] - center[0], corner[1] - center[1]
        local.append((rotation[0][0] * dx + rotation[1][0] * dy,
                      rotation[0][1] * dx + rotation[1][1] * dy))
    lows = [min(point[axis] for point in local) for axis in (0, 1)]
    highs = [max(point[axis] for point in local) for axis in (0, 1)]

    offsets, extents, trimmed = [], [], []
    for axis in (0, 1):
        low = max(-half[axis], lows[axis])
        high = min(half[axis], highs[axis])
        if high <= low:
            return None
        offsets.append((low + high) / 2.0)
        extents.append(high - low)
        trimmed.append(high - low < size[axis] - 1e-6)

    low_z, high_z = center[2] - half[2], center[2] + half[2]
    if z_range is not None:
        low_z, high_z = max(low_z, float(z_range[0])), min(high_z, float(z_range[1]))
        if high_z <= low_z:
            return None

    # A sliver is an element the clip cut down to nothing, not a genuinely
    # thin one: a 0.12 m partition must survive, a wall shaved to 0.09 m by the
    # room boundary must not.
    for axis in (0, 1):
        if trimmed[axis] and extents[axis] < minimum_extent:
            return None

    clipped = {key: value for key, value in element.items()}
    clipped['pose'] = {
        'xyz': [
            round(center[0] + rotation[0][0] * offsets[0]
                  + rotation[0][1] * offsets[1], 6),
            round(center[1] + rotation[1][0] * offsets[0]
                  + rotation[1][1] * offsets[1], 6),
            round((low_z + high_z) / 2.0, 6),
        ],
        'rpy': [round(float(value), 9) for value in element['pose']['rpy']],
    }
    clipped['geometry'] = {
        'type': 'box',
        'size': [round(extents[0], 6), round(extents[1], 6),
                 round(high_z - low_z, 6)],
    }
    return clipped


def extract(manifest, bounds, z_range=None, minimum_size=0.2, room_id='room'):
    """Clip every element of a validated manifest to a room."""
    nominal = validate_manifest(manifest)
    kept = []
    for element in nominal['elements']:
        clipped = clip_element(element, bounds, z_range, minimum_size)
        if clipped is None:
            continue
        kept.append(clipped)

    source = dict(nominal.get('source', {}))
    source['room'] = {
        'id': room_id,
        'bounds': [round(float(value), 6) for value in bounds],
        'z_range': ([round(float(value), 6) for value in z_range]
                    if z_range is not None else None),
        'parent_elements': len(nominal['elements']),
    }
    world = dict(nominal['world'])
    world['name'] = f"{world['name']}_{room_id}"
    return {
        'schema_version': SCHEMA_VERSION,
        'source': source,
        'world': world,
        'elements': kept,
    }


def _parser():
    parser = argparse.ArgumentParser(
        description='Clip a building manifest down to one room.')
    parser.add_argument('--manifest', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--room-id', default='room')
    parser.add_argument(
        '--bounds', nargs=4, type=float, required=True,
        metavar=('X0', 'Y0', 'X1', 'Y1'),
        help='World-frame rectangle to keep.')
    parser.add_argument(
        '--z-range', nargs=2, type=float, default=None, metavar=('LOW', 'HIGH'),
        help='Optional vertical clip, e.g. to drop the roof and keep a ceiling.')
    parser.add_argument(
        '--minimum-size', type=float, default=0.2,
        help='Discard elements the clip reduced to a sliver thinner than this.')
    return parser


def main(argv=None):
    args = _parser().parse_args(argv)
    manifest = yaml.safe_load(args.manifest.read_text(encoding='utf-8'))
    room = extract(manifest, args.bounds, args.z_range,
                   args.minimum_size, args.room_id)
    if not room['elements']:
        print('error: the room bounds kept no elements')
        return 1
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        yaml.safe_dump(room, sort_keys=False), encoding='utf-8')
    print(f'Wrote {len(room["elements"])} elements '
          f'(from {room["source"]["room"]["parent_elements"]}) to {args.output}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
