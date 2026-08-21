"""Convert IFC products to a DeltaSeek primitive element manifest.

Each product is approximated by a box.  By default the box is oriented in the
product's own placement frame, which matters because building models are
routinely exported on a slightly rotated site grid: a 39 m wall sitting 0.9 deg
off the world axes has a world-aligned box four times its true thickness, and
that error lands straight in the navigation costmap.  ``--geometry aabb``
restores the older world-aligned behaviour for comparison.
"""

import math

import argparse
from pathlib import Path
import sys

import yaml


DEFAULT_CLASSES = {
    'IfcBeam',
    'IfcBuildingElementProxy',
    'IfcColumn',
    'IfcCovering',
    'IfcCurtainWall',
    'IfcDoor',
    'IfcFlowSegment',
    'IfcFooting',
    'IfcFurnishingElement',
    'IfcMember',
    'IfcPile',
    'IfcPlate',
    'IfcRailing',
    'IfcRamp',
    'IfcRoof',
    'IfcSlab',
    'IfcStair',
    'IfcWall',
    'IfcWallStandardCase',
    'IfcWindow',
}

CLASS_COLORS = {
    'IfcBeam': [0.55, 0.58, 0.62, 1.0],
    'IfcColumn': [0.55, 0.58, 0.62, 1.0],
    'IfcDoor': [0.55, 0.32, 0.14, 1.0],
    'IfcFlowSegment': [0.80, 0.34, 0.10, 1.0],
    'IfcSlab': [0.50, 0.50, 0.48, 1.0],
    'IfcWall': [0.72, 0.69, 0.62, 1.0],
    'IfcWallStandardCase': [0.72, 0.69, 0.62, 1.0],
    'IfcWindow': [0.25, 0.55, 0.72, 0.55],
}


def _parser():
    parser = argparse.ArgumentParser(
        description=(
            'Approximate IFC products with world-aligned bounding boxes. '
            'GlobalIds and IFC classes are retained for deviation labels.'))
    parser.add_argument('--ifc', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--world-name', default='deltaseek_ifc_nominal')
    parser.add_argument(
        '--origin', nargs=3, type=float, default=[0.0, 0.0, 0.0],
        metavar=('X', 'Y', 'Z'),
        help='IFC world-coordinate origin to subtract, in metres.')
    parser.add_argument(
        '--classes', default=','.join(sorted(DEFAULT_CLASSES)),
        help='Comma-separated IFC classes to include.')
    parser.add_argument(
        '--geometry', choices=('obb', 'aabb'), default='obb',
        help=(
            'obb: box oriented in the product placement frame (default). '
            'aabb: world-aligned box, which inflates anything off-axis.'))
    parser.add_argument('--threads', type=int, default=1)
    parser.add_argument(
        '--minimum-size', type=float, default=0.005,
        help='Reject degenerate products smaller than this many metres.')
    return parser


def _load_ifcopenshell():
    try:
        import ifcopenshell
        import ifcopenshell.geom
    except ImportError as exc:
        raise RuntimeError(
            'IfcOpenShell is required only for IFC import. Install it in a '
            'separate conversion environment with: pip install ifcopenshell'
        ) from exc
    return ifcopenshell, ifcopenshell.geom


def _rpy_from_matrix(rotation):
    """Recover roll-pitch-yaw from a rotation matrix built as ``Rz @ Ry @ Rx``.

    This is the inverse of ``visibility.rotation_matrix``, so the angles written
    here reproduce the same orientation downstream and in SDF.
    """
    sp = max(-1.0, min(1.0, -float(rotation[2][0])))
    pitch = math.asin(sp)
    if abs(sp) > 1.0 - 1e-9:
        # Gimbal lock: roll and yaw are not separable, so fold both into yaw.
        roll = 0.0
        yaw = math.atan2(-float(rotation[0][1]), float(rotation[1][1]))
    else:
        roll = math.atan2(float(rotation[2][1]), float(rotation[2][2]))
        yaw = math.atan2(float(rotation[1][0]), float(rotation[0][0]))
    return [roll, pitch, yaw]


def _placement(shape):
    """Return (rotation, translation) from a shape's 4x4 column-major matrix."""
    values = [float(value) for value in shape.transformation.matrix]
    if len(values) != 16:
        raise RuntimeError(
            f'expected a 4x4 placement matrix, got {len(values)} values')
    columns = [values[index:index + 4] for index in range(0, 16, 4)]
    rotation = [[columns[col][row] for col in range(3)] for row in range(3)]
    translation = columns[3][:3]
    return rotation, translation


def _clean_float(value):
    value = round(float(value), 9)
    return 0.0 if value == 0.0 else value


def convert(args):
    """Convert an IFC file into a conservative box manifest."""
    ifcopenshell, ifc_geom = _load_ifcopenshell()
    ifc_file = ifcopenshell.open(str(args.ifc))
    oriented = args.geometry == 'obb'
    settings = ifc_geom.settings()
    # World coordinates bake the placement into the vertices, which is what an
    # axis-aligned box wants and exactly what an oriented one must not have.
    settings.set('use-world-coords', not oriented)
    iterator = ifc_geom.iterator(
        settings, ifc_file, max(1, int(args.threads)))
    included_classes = {
        item.strip() for item in args.classes.split(',') if item.strip()
    }
    elements = []
    skipped = 0

    if iterator.initialize():
        while True:
            shape = iterator.get()
            product = ifc_file.by_id(shape.id)
            if product.is_a() in included_classes:
                vertices = list(shape.geometry.verts)
                axes = [vertices[offset::3] for offset in range(3)]
                if vertices and all(axes):
                    minimum = [min(axis) for axis in axes]
                    maximum = [max(axis) for axis in axes]
                    size = [
                        _clean_float(high - low)
                        for low, high in zip(minimum, maximum)
                    ]
                    if all(value >= args.minimum_size for value in size):
                        local = [
                            (low + high) / 2.0
                            for low, high in zip(minimum, maximum)
                        ]
                        if oriented:
                            rotation, translation = _placement(shape)
                            world = [
                                sum(rotation[row][col] * local[col]
                                    for col in range(3)) + translation[row]
                                for row in range(3)
                            ]
                            rpy = _rpy_from_matrix(rotation)
                        else:
                            world = local
                            rpy = [0.0, 0.0, 0.0]
                        center = [
                            _clean_float(value - origin)
                            for value, origin in zip(world, args.origin)
                        ]
                        ifc_class = product.is_a()
                        elements.append({
                            'id': str(product.GlobalId),
                            'ifc_class': ifc_class,
                            'name': str(product.Name or product.GlobalId),
                            'pose': {
                                'xyz': center,
                                'rpy': [_clean_float(value) for value in rpy],
                            },
                            'geometry': {'type': 'box', 'size': size},
                            'material': {
                                'rgba': list(CLASS_COLORS.get(
                                    ifc_class, [0.65, 0.68, 0.72, 1.0]))
                            },
                            'static': True,
                            'collision': True,
                        })
                    else:
                        skipped += 1
            if not iterator.next():
                break

    elements.sort(key=lambda item: item['id'])
    return {
        'schema_version': 1,
        'source': {
            'type': 'ifc',
            'file': str(args.ifc),
            'ifc_schema': str(ifc_file.schema),
            'units': 'metre',
            'geometry_approximation': (
                'oriented_bounding_box' if oriented
                else 'world_aligned_bounding_box'),
            'origin_offset': [_clean_float(value) for value in args.origin],
            'skipped_degenerate_products': skipped,
        },
        'world': {
            'name': args.world_name,
            'frame_id': 'world',
            'gravity': [0.0, 0.0, -9.81],
        },
        'elements': elements,
    }


def main(argv=None):
    args = _parser().parse_args(argv)
    try:
        manifest = convert(args)
    except RuntimeError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2
    if not manifest['elements']:
        print('error: no supported products with geometry were found', file=sys.stderr)
        return 1
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        yaml.safe_dump(manifest, sort_keys=False), encoding='utf-8')
    print(f'Wrote {len(manifest["elements"])} elements to {args.output}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
