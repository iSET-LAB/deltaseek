"""Sample randomized deviation scenarios at a controlled density.

Hand-written scenarios cannot support a density sweep, and a sweep is what
shows whether a deviation-seeking planner keeps its advantage as deviations
become rare.  This module turns a nominal manifest plus a seed into a scenario
in the same schema the hand-written fixtures use, so downstream generation and
ground-truth code is shared.

Density is the fraction of nominal elements carrying a deviation.  Unexpected
elements are counted separately because they add to the scene instead of
modifying it, so they cannot consume a nominal element's single-deviation slot.

Every draw comes from one seeded generator, so a scenario id plus a seed fully
reproduces a benchmark.
"""

import argparse
import math
from pathlib import Path
import random

import yaml

from deltaseek_gazebo.benchmark import SCHEMA_VERSION, validate_manifest


MODIFYING_TYPES = ('displaced', 'rotated', 'missing', 'resized')
DEFAULT_WEIGHTS = {
    'displaced': 0.4,
    'rotated': 0.2,
    'missing': 0.2,
    'resized': 0.2,
}


def _parser():
    parser = argparse.ArgumentParser(
        description='Sample a reproducible deviation scenario from a manifest.')
    parser.add_argument('--manifest', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--scenario-id', default='sampled')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument(
        '--density', type=float, default=0.25,
        help='Fraction of nominal elements that receive a deviation.')
    parser.add_argument(
        '--unexpected-rate', type=float, default=0.05,
        help='Unexpected elements added, as a fraction of nominal count.')
    parser.add_argument(
        '--displacement-range', nargs=2, type=float, default=[0.15, 0.60],
        metavar=('MIN', 'MAX'), help='Displacement magnitude in metres.')
    parser.add_argument(
        '--rotation-range', nargs=2, type=float, default=[0.10, 0.45],
        metavar=('MIN', 'MAX'), help='Rotation magnitude in radians.')
    parser.add_argument(
        '--scale-range', nargs=2, type=float, default=[0.15, 0.45],
        metavar=('MIN', 'MAX'), help='Fractional size change.')
    parser.add_argument(
        '--types', default=','.join(MODIFYING_TYPES),
        help='Comma-separated subset of modifying deviation types.')
    return parser


def _weighted_type(rng, types):
    weights = [DEFAULT_WEIGHTS.get(name, 0.25) for name in types]
    return rng.choices(list(types), weights=weights, k=1)[0]


def _severity(rng):
    return round(rng.uniform(0.2, 0.95), 3)


def _displaced(rng, args):
    magnitude = rng.uniform(*args.displacement_range)
    # Deviations on a construction site are predominantly in-plane, so the
    # vertical component is deliberately smaller than the lateral one.
    angle = rng.uniform(0.0, 2.0 * math.pi)
    vertical = rng.uniform(-0.25, 0.25) * magnitude
    return {
        'translation': [
            round(magnitude * math.cos(angle), 6),
            round(magnitude * math.sin(angle), 6),
            round(vertical, 6),
        ],
    }


def _rotated(rng, args):
    magnitude = rng.uniform(*args.rotation_range)
    # Most site elements are installed off-axis about the vertical, not tipped.
    return {
        'rotation': [
            round(rng.uniform(-0.1, 0.1) * magnitude, 6),
            round(rng.uniform(-0.1, 0.1) * magnitude, 6),
            round(rng.choice((-1.0, 1.0)) * magnitude, 6),
        ],
    }


def _resized(rng, args):
    change = rng.uniform(*args.scale_range)
    axis = rng.randrange(3)
    factors = [1.0, 1.0, 1.0]
    factors[axis] = round(1.0 + rng.choice((-1.0, 1.0)) * change, 6)
    if factors[axis] <= 0.1:
        factors[axis] = 1.0 + change
    return {'scale_factors': factors}


def _unexpected_element(rng, index, elements):
    """Place an extra object on the ground near an existing element."""
    anchor = rng.choice(elements)
    angle = rng.uniform(0.0, 2.0 * math.pi)
    radius = rng.uniform(1.0, 2.5)
    size = round(rng.uniform(0.35, 0.9), 3)
    center = anchor['pose']['xyz']
    return {
        'id': f'unexpected-{index:03d}',
        'ifc_class': 'IfcBuildingElementProxy',
        'name': f'Unexpected site object {index:03d}',
        'pose': {
            'xyz': [
                round(center[0] + radius * math.cos(angle), 6),
                round(center[1] + radius * math.sin(angle), 6),
                round(size / 2.0, 6),
            ],
            'rpy': [0.0, 0.0, round(rng.uniform(0.0, math.pi), 6)],
        },
        'geometry': {'type': 'box', 'size': [size, size, size]},
        'material': {'rgba': [0.55, 0.28, 0.08, 1.0]},
        'static': True,
        'collision': True,
    }


def sample_scenario(manifest, args):
    """Return a scenario mapping for the given manifest and sampling options."""
    nominal = validate_manifest(manifest)
    elements = nominal['elements']
    types = tuple(
        name.strip() for name in args.types.split(',')
        if name.strip() in MODIFYING_TYPES
    )
    if not types:
        raise ValueError('at least one modifying deviation type is required')

    rng = random.Random(args.seed)
    count = int(round(max(0.0, min(1.0, args.density)) * len(elements)))
    targets = rng.sample(elements, count) if count else []
    # Sampling order must not depend on the manifest's ordering.
    targets.sort(key=lambda item: item['id'])

    builders = {
        'displaced': _displaced,
        'rotated': _rotated,
        'resized': _resized,
    }
    deviations = []
    for index, element in enumerate(targets):
        kind = _weighted_type(rng, types)
        deviation = {
            'id': f'{kind}-{index:03d}',
            'type': kind,
            'target': element['id'],
            'severity': _severity(rng),
        }
        if kind in builders:
            deviation.update(builders[kind](rng, args))
        deviations.append(deviation)

    extra = int(round(max(0.0, args.unexpected_rate) * len(elements)))
    for index in range(extra):
        deviations.append({
            'id': f'unexpected-{index:03d}',
            'type': 'unexpected',
            'severity': _severity(rng),
            'element': _unexpected_element(rng, index, elements),
        })

    return {
        'schema_version': SCHEMA_VERSION,
        'scenario': {
            'id': args.scenario_id,
            'seed': args.seed,
            'description': (
                f'Sampled at density {args.density} with seed {args.seed}.'),
            'density': args.density,
            'unexpected_rate': args.unexpected_rate,
        },
        'deviations': deviations,
    }


def main(argv=None):
    args = _parser().parse_args(argv)
    manifest = yaml.safe_load(args.manifest.read_text(encoding='utf-8'))
    scenario = sample_scenario(manifest, args)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        yaml.safe_dump(scenario, sort_keys=False), encoding='utf-8')
    print(
        f'Wrote {len(scenario["deviations"])} deviations to {args.output}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
