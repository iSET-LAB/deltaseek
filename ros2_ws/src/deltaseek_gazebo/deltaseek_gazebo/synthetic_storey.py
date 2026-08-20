"""Generate a synthetic office storey as a DeltaSeek element manifest.

The demo fixture is seven elements in one open room, where every planner sees
everything and saturates. Evaluating a deviation-seeking planner needs a scene
with three properties the fixture cannot supply: enough elements for a density
sweep to mean anything, occlusion that makes viewpoint choice matter, and
elements that are only visible from particular heights, so that including the
manipulator is worth something measurable.

Walls are emitted as segments around their openings rather than as solid
slabs. A doorway a robot can see through is the whole point of the scene, and
it is also what the IFC adapter cannot yet produce: that adapter approximates
each product by its world-aligned bounding box, which turns a wall with a door
into a solid barrier. Building the geometry here from primitives sidesteps that
until a tessellating IFC importer exists.

The layout is a central corridor with rooms down both sides, which is the
cheapest arrangement that forces a robot to enter a space to inspect it.

Output is an ordinary manifest, so the sampler, world generator, ground truth
and scorer all consume it unchanged.
"""

import argparse
import math
from pathlib import Path

import yaml

from deltaseek_gazebo.benchmark import SCHEMA_VERSION
from deltaseek_gazebo.ifc_to_manifest import CLASS_COLORS


DOOR_HEIGHT = 2.1


def _parser():
    parser = argparse.ArgumentParser(
        description='Generate a synthetic storey manifest for evaluation.')
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--world-name', default='deltaseek_synthetic_storey')
    parser.add_argument('--rooms-per-side', type=int, default=4)
    parser.add_argument('--room-width', type=float, default=6.0)
    parser.add_argument('--room-depth', type=float, default=5.0)
    parser.add_argument('--corridor-width', type=float, default=2.6)
    parser.add_argument('--wall-height', type=float, default=3.0)
    parser.add_argument('--wall-thickness', type=float, default=0.2)
    parser.add_argument('--door-width', type=float, default=1.3)
    parser.add_argument(
        '--equipment-per-room', type=int, default=3,
        help='Floor-standing items placed in each room.')
    return parser


class _Builder:
    """Accumulates manifest elements with stable, unique identifiers."""

    def __init__(self):
        self.elements = []
        self._counts = {}

    def add(self, ifc_class, name, xyz, size, rgba=None):
        index = self._counts.get(ifc_class, 0)
        self._counts[ifc_class] = index + 1
        size = [max(float(value), 0.01) for value in size]
        self.elements.append({
            'id': f'SYN-{ifc_class}-{index:04d}',
            'ifc_class': ifc_class,
            'name': name,
            'pose': {
                'xyz': [round(float(value), 6) for value in xyz],
                'rpy': [0.0, 0.0, 0.0],
            },
            'geometry': {'type': 'box', 'size': [round(v, 6) for v in size]},
            'material': {
                'rgba': list(rgba or CLASS_COLORS.get(
                    ifc_class, [0.65, 0.68, 0.72, 1.0]))
            },
            'static': True,
            'collision': True,
        })


def _wall_with_door(builder, name, along, fixed, start, end, thickness,
                    height, door_center=None, door_width=0.0):
    """Emit a wall as segments, leaving a real opening where a door sits.

    ``along`` is the axis the wall runs along, 0 for x and 1 for y; ``fixed``
    is its position on the other axis.
    """
    def box(low, high, z_low, z_high, suffix):
        length = high - low
        if length <= 0.01:
            return
        center = [0.0, 0.0, (z_low + z_high) / 2.0]
        center[along] = (low + high) / 2.0
        center[1 - along] = fixed
        size = [0.0, 0.0, z_high - z_low]
        size[along] = length
        size[1 - along] = thickness
        builder.add('IfcWall', f'{name} {suffix}', center, size)

    if door_width <= 0.0 or door_center is None:
        box(start, end, 0.0, height, 'panel')
        return
    left = door_center - door_width / 2.0
    right = door_center + door_width / 2.0
    box(start, left, 0.0, height, 'jamb west')
    box(right, end, 0.0, height, 'jamb east')
    # The lintel is what makes the opening a doorway rather than a gap, and it
    # is exactly the geometry a bounding-box import destroys.
    box(left, right, DOOR_HEIGHT, height, 'lintel')


def build_storey(args):
    """Return a manifest for a corridor-and-rooms storey."""
    rng = _Rng(args.seed)
    builder = _Builder()

    span_x = args.rooms_per_side * args.room_width
    half_corridor = args.corridor_width / 2.0
    outer_y = half_corridor + args.room_depth

    builder.add('IfcSlab', 'Storey slab',
                [span_x / 2.0, 0.0, -0.1],
                [span_x + 2.0, 2.0 * outer_y + 2.0, 0.2])

    for side, sign in (('north', 1.0), ('south', -1.0)):
        corridor_face = sign * half_corridor
        outer_face = sign * outer_y
        for index in range(args.rooms_per_side):
            x0 = index * args.room_width
            x1 = x0 + args.room_width
            room = f'{side} room {index}'

            # Corridor-facing wall carries the door; the rest are solid.
            _wall_with_door(
                builder, f'{room} corridor wall', 0, corridor_face, x0, x1,
                args.wall_thickness, args.wall_height,
                door_center=x0 + args.room_width / 2.0,
                door_width=args.door_width)
            _wall_with_door(
                builder, f'{room} outer wall', 0, outer_face, x0, x1,
                args.wall_thickness, args.wall_height)
            _wall_with_door(
                builder, f'{room} west wall', 1, x0,
                min(corridor_face, outer_face), max(corridor_face, outer_face),
                args.wall_thickness, args.wall_height)

            _populate_room(builder, rng, args, room, x0, x1,
                           corridor_face, outer_face, sign)

    # Closing wall at the far end of the row.
    _wall_with_door(
        builder, 'east end wall', 1, span_x, -outer_y, outer_y,
        args.wall_thickness, args.wall_height)

    # Corridor services: a pipe run and a duct the length of the storey, high
    # enough that only a raised camera resolves them.
    builder.add('IfcFlowSegment', 'Corridor pipe run',
                [span_x / 2.0, -0.6, 2.55], [span_x, 0.22, 0.22])
    builder.add('IfcFlowSegment', 'Corridor duct',
                [span_x / 2.0, 0.7, 2.45], [span_x, 0.55, 0.4])

    for index in range(args.rooms_per_side + 1):
        x = index * args.room_width
        for y in (-half_corridor - 0.35, half_corridor + 0.35):
            builder.add('IfcColumn', f'Column {index} at {y:.1f}',
                        [x, y, args.wall_height / 2.0],
                        [0.4, 0.4, args.wall_height])

    return {
        'schema_version': SCHEMA_VERSION,
        'source': {
            'type': 'synthetic',
            'generator': 'deltaseek_gazebo.synthetic_storey',
            'seed': args.seed,
            'units': 'metre',
            'geometry_approximation': 'primitive_boxes_with_openings',
            'layout': 'central corridor, rooms both sides',
        },
        'world': {
            'name': args.world_name,
            'frame_id': 'world',
            'gravity': [0.0, 0.0, -9.81],
        },
        'elements': builder.elements,
    }


def _populate_room(builder, rng, args, room, x0, x1, corridor_face,
                   outer_face, sign):
    """Fill a room with contents at heights that reward different postures."""
    inner_near = corridor_face + sign * 0.6
    inner_far = outer_face - sign * 0.6

    for slot in range(args.equipment_per_room):
        fraction = (slot + 0.5) / args.equipment_per_room
        x = x0 + 0.8 + fraction * (args.room_width - 1.6)
        y = inner_near + (inner_far - inner_near) * rng.uniform(0.25, 0.85)
        width = rng.uniform(0.6, 1.1)
        depth = rng.uniform(0.5, 0.9)
        height = rng.uniform(0.8, 1.6)
        builder.add('IfcFurnishingElement', f'{room} equipment {slot}',
                    [x, y, height / 2.0], [width, depth, height])

        # A fitting tucked against the far side of the equipment: visible only
        # from a viewpoint that gets past the cabinet, which is what the arm
        # is for.
        if slot % 2 == 0:
            builder.add(
                'IfcBuildingElementProxy', f'{room} low fitting {slot}',
                [x, y + sign * (depth / 2.0 + 0.28), 0.22],
                [0.3, 0.24, 0.3])

    # A branch drop from the corridor services into the room, high up.
    branch_x = x0 + args.room_width / 2.0
    builder.add('IfcFlowSegment', f'{room} branch duct',
                [branch_x, (corridor_face + inner_far) / 2.0, 2.5],
                [0.3, abs(inner_far - corridor_face), 0.3])

    # A wall-mounted panel, only resolvable from inside the room.
    builder.add('IfcBuildingElementProxy', f'{room} wall panel',
                [x0 + 0.5, outer_face - sign * 0.18, 1.5],
                [0.5, 0.12, 0.7])


class _Rng:
    """Small deterministic generator, kept separate so layout stays stable.

    Using an explicit stream rather than `random` module state means adding a
    later element type cannot shift the values drawn for existing ones.
    """

    def __init__(self, seed):
        self._state = (seed * 6364136223846793005 + 1442695040888963407) % (2 ** 64)

    def _next(self):
        self._state = (self._state * 6364136223846793005 + 1442695040888963407) % (2 ** 64)
        return (self._state >> 11) / float(2 ** 53)

    def uniform(self, low, high):
        return low + (high - low) * self._next()


def main(argv=None):
    args = _parser().parse_args(argv)
    manifest = build_storey(args)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        yaml.safe_dump(manifest, sort_keys=False), encoding='utf-8')
    counts = {}
    for element in manifest['elements']:
        counts[element['ifc_class']] = counts.get(element['ifc_class'], 0) + 1
    print(f'Wrote {len(manifest["elements"])} elements to {args.output}')
    for ifc_class, count in sorted(counts.items()):
        print(f'  {ifc_class:28s} {count}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
