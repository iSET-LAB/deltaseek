"""Create the serpentine sweep used to exercise the evaluation harness.

This is a harness fixture, not a baseline. It drives a fixed lawnmower pattern
and looks around from each stop, which is what a coverage planner would do if
it knew nothing about the model. The real coverage, frontier, and
goal-directed baselines belong in their own module, scored through the same
`evaluate` entry point.
"""

import argparse
import math
from pathlib import Path

import yaml


def sweep(rows=(-3.0, 0.0, 3.0), x_range=(-6.0, 6.0), spacing=3.0,
          camera_height=1.45, pitch=-0.35):
    """Return viewpoints for a serpentine drive with a look-around at each stop."""
    viewpoints = []
    forward = True
    for row_index, y in enumerate(rows):
        steps = int(round((x_range[1] - x_range[0]) / spacing)) + 1
        xs = [x_range[0] + index * spacing for index in range(steps)]
        if not forward:
            xs.reverse()
        forward = not forward
        for x in xs:
            # Yaws are relative to the direction of travel so the sweep looks
            # ahead, to each side, and behind without stopping to spin.
            for yaw in (0.0, math.pi / 2.0, -math.pi / 2.0):
                viewpoints.append({
                    'base': {'xyz': [round(x, 3), round(y, 3), 0.0],
                             'yaw': 0.0},
                    'camera': {
                        'xyz': [round(x, 3), round(y, 3), camera_height],
                        'rpy': [0.0, round(pitch, 4), round(yaw, 4)],
                    },
                })
    return viewpoints


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path)
    args = parser.parse_args()

    document = {
        'schema_version': 1,
        'trajectory': {
            'id': 'demo_sweep',
            'frame_id': 'world',
            'description': (
                'Fixed serpentine sweep with a three-way look-around, used to '
                'exercise the metrics harness.'),
        },
        'viewpoints': sweep(),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        yaml.safe_dump(document, sort_keys=False), encoding='utf-8')
    print(f'{args.output}: {len(document["viewpoints"])} viewpoints')


if __name__ == '__main__':
    main()
