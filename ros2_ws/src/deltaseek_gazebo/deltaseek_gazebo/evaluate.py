"""Score an inspection trajectory against exact deviation ground truth.

This produces the abstract's headline claim: what fraction of injected
deviations a run detects, and how much traversal distance it spent doing so.
Reporting detections against a *distance budget* rather than against time or
viewpoint count is what makes a deviation-seeking planner comparable with
coverage, frontier, and goal-directed baselines, which all pay for distance in
the same currency.

Traversal distance accumulates over base positions only.  Arm motion is
deliberately excluded: the claim under test is that including the manipulator
in viewpoint selection buys detections *without* extra driving.
"""

import argparse
import json
from pathlib import Path

import numpy as np
import yaml

from deltaseek_gazebo.benchmark import apply_scenario
from deltaseek_gazebo.detection import ObservationParams, detect_along


DEFAULT_BUDGETS = (0.25, 0.5, 0.75, 1.0)


def _load(path):
    return yaml.safe_load(Path(path).read_text(encoding='utf-8'))


def load_trajectory(document):
    """Return base positions and camera poses from a trajectory document."""
    viewpoints = document.get('viewpoints', [])
    if not viewpoints:
        raise ValueError('trajectory contains no viewpoints')
    bases = []
    cameras = []
    for index, viewpoint in enumerate(viewpoints):
        camera = viewpoint.get('camera')
        if not camera:
            raise ValueError(f'viewpoints[{index}] has no camera pose')
        cameras.append((
            [float(value) for value in camera['xyz']],
            [float(value) for value in camera.get('rpy', [0.0, 0.0, 0.0])],
        ))
        base = viewpoint.get('base', {})
        bases.append([float(value) for value in base.get('xyz', camera['xyz'])])
    return np.asarray(bases, dtype=float), cameras


def cumulative_distance(bases, cost_fn=None):
    """Return traversal distance up to and including each viewpoint.

    ``cost_fn`` must be the same cost the planner selected against. Choosing
    viewpoints by drivable distance and then scoring them by straight-line
    distance would report a budget the robot never actually spent.
    """
    if len(bases) < 2:
        return np.zeros(len(bases))
    if cost_fn is None:
        steps = np.linalg.norm(np.diff(bases[:, :2], axis=0), axis=1)
    else:
        steps = np.array([
            cost_fn(tuple(a[:2]), tuple(b[:2]))
            for a, b in zip(bases[:-1], bases[1:])
        ])
    return np.concatenate([[0.0], np.cumsum(steps)])


def summarize(records, distances, budgets=DEFAULT_BUDGETS):
    """Return detection counts and the distance each detection cost."""
    total = len(records)
    detected = [record for record in records if record['detected']]
    total_distance = float(distances[-1]) if len(distances) else 0.0

    for record in records:
        index = record['first_viewpoint']
        record['distance_at_detection'] = (
            round(float(distances[index]), 6) if index is not None else None)

    curve = []
    for index, distance in enumerate(distances):
        found = sum(
            1 for record in records
            if record['first_viewpoint'] is not None
            and record['first_viewpoint'] <= index
        )
        curve.append({
            'viewpoint_index': int(index),
            'distance': round(float(distance), 6),
            'detections': found,
            'detection_rate': round(found / total, 6) if total else 0.0,
        })

    at_budget = {}
    for budget in budgets:
        limit = total_distance * budget
        found = sum(
            1 for record in detected
            if record['distance_at_detection'] is not None
            and record['distance_at_detection'] <= limit + 1.0e-9
        )
        at_budget[f'{int(budget * 100)}%'] = {
            'distance': round(limit, 6),
            'detections': found,
            'detection_rate': round(found / total, 6) if total else 0.0,
        }

    return {
        'summary': {
            'deviation_count': total,
            'detected_count': len(detected),
            'detection_rate': round(len(detected) / total, 6) if total else 0.0,
            'viewpoint_count': int(len(distances)),
            'total_traversal_distance': round(total_distance, 6),
        },
        'detection_rate_at_budget': at_budget,
        'budget_curve': curve,
        'detections': records,
    }


def evaluate(manifest, scenario, trajectory, params=None, cost_fn=None):
    """Replay a trajectory against a scenario and return its metrics."""
    nominal, actual_elements, ground_truth = apply_scenario(manifest, scenario)
    bases, cameras = load_trajectory(trajectory)
    records = detect_along(
        cameras, nominal['elements'], actual_elements, ground_truth, params)
    distances = cumulative_distance(bases, cost_fn)
    report = summarize(records, distances)
    report['schema_version'] = 1
    report['scenario_id'] = ground_truth['scenario_id']
    report['trajectory_id'] = trajectory.get('trajectory', {}).get('id', 'run')
    return report


def _parser():
    parser = argparse.ArgumentParser(
        description='Score an inspection trajectory against deviation labels.')
    parser.add_argument('--manifest', required=True, type=Path)
    parser.add_argument('--scenario', required=True, type=Path)
    parser.add_argument('--trajectory', required=True, type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument(
        '--min-visible-fraction', type=float, default=0.05,
        help='Surface fraction an element must show to count as observed.')
    parser.add_argument(
        '--max-range', type=float, default=6.0,
        help='Usable depth range in metres, not the render clip distance.')
    parser.add_argument(
        '--samples-per-face', type=int, default=3,
        help='Grid resolution per box face; cost grows with its square.')
    return parser


def main(argv=None):
    args = _parser().parse_args(argv)
    params = ObservationParams(
        samples_per_face=args.samples_per_face,
        min_visible_fraction=args.min_visible_fraction,
        far=args.max_range,
    )
    report = evaluate(
        _load(args.manifest), _load(args.scenario), _load(args.trajectory),
        params)

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            yaml.safe_dump(report, sort_keys=False), encoding='utf-8')
        print(f'Wrote {args.output}')

    summary = report['summary']
    print(json.dumps(summary, indent=2))
    print('detection rate at traversal budget:')
    for label, entry in report['detection_rate_at_budget'].items():
        print(
            f'  {label:>5}  {entry["distance"]:7.2f} m  '
            f'{entry["detections"]}/{summary["deviation_count"]}  '
            f'{entry["detection_rate"] * 100:5.1f}%')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
