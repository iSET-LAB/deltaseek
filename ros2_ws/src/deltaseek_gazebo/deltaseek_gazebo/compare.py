"""Run every planner on one scenario and report detections per unit distance.

This is the experiment behind the abstract's claim.  All planners share the
same candidate viewpoints, the same nominal model, and the same distance
budget, so the only variable is how each one chooses where to look.

``--fixed-sensor`` repeats the comparison with the arm frozen at a single
posture, which isolates what including the manipulator in viewpoint selection
is actually worth.
"""

import argparse
from pathlib import Path
import subprocess
import tempfile

import yaml

from deltaseek_gazebo.benchmark import validate_manifest
from deltaseek_gazebo.detection import ObservationParams
from deltaseek_gazebo.evaluate import SENSOR_SETS, evaluate
from deltaseek_gazebo.kinematics import Chain
from deltaseek_gazebo.navigation import path_cost_for
from deltaseek_gazebo.paths import default_setup_path
from deltaseek_gazebo.planner import PLANNERS, euclidean, visibility_matrix
from deltaseek_gazebo.viewpoints import (
    build_viewpoints,
    chassis_chain,
    fixed_sensor_viewpoints,
    free_base_poses,
    scene_bounds,
    to_trajectory,
)


def _load(path):
    return yaml.safe_load(Path(path).read_text(encoding='utf-8'))


def resolve_urdf(explicit=None):
    """Return a path to an expanded URDF, expanding the xacro if needed."""
    if explicit:
        return Path(explicit)
    source = Path(default_setup_path()) / 'robot.urdf.xacro'
    if not source.is_file():
        raise FileNotFoundError(
            f'{source} not found; launch the simulation once to generate it, '
            'or pass --urdf')
    output = Path(tempfile.gettempdir()) / 'deltaseek_robot.urdf'
    result = subprocess.run(
        ['xacro', str(source)], capture_output=True, check=False)
    if result.returncode != 0:
        raise RuntimeError(
            f'xacro failed on {source}. A PYTHONPATH that replaces the ROS '
            f'entry instead of extending it is the usual cause.\n'
            f'{result.stderr.decode(errors="replace").strip()}')
    output.write_bytes(result.stdout)
    return output


def run(manifest, scenario, chain, budget, params, fixed_sensor=False,
        spacing=1.5, yaws=(0.0, 1.5707963, 3.1415927, -1.5707963),
        path_cost='grid', start=None, sensors=None, chassis=None):
    """Return one report per planner for this scenario."""
    nominal = validate_manifest(manifest)
    elements = nominal['elements']
    bases = free_base_poses(
        elements, scene_bounds(elements), spacing=spacing, yaws=yaws)
    builder = fixed_sensor_viewpoints if fixed_sensor else build_viewpoints
    viewpoints = builder(chain, bases, chassis=chassis)
    if not viewpoints:
        raise ValueError('no reachable viewpoints were generated')

    # The planner must see what the evaluation will score, so the same sensor
    # set drives both.
    keys, matrix = visibility_matrix(viewpoints, elements, params, sensors)

    # Straight-line distance lets every planner reach a room without paying
    # for the doorway, which flatters all of them and distorts the metric the
    # project is judged on.
    cost_fn = euclidean if path_cost == 'euclidean' else path_cost_for(elements)
    # Start from the reachable pose nearest the west end of the storey rather
    # than the origin, which need not be drivable.
    if start is None:
        start = min((xy for xy, _ in bases), key=lambda xy: (xy[0], abs(xy[1])))
    else:
        # Snap the requested start onto the candidate set, so every planner
        # begins from a pose the traversal grid actually knows about.
        target = (float(start[0]), float(start[1]))
        start = min((xy for xy, _ in bases),
                    key=lambda xy: (xy[0] - target[0]) ** 2
                    + (xy[1] - target[1]) ** 2)

    reports = {}
    for name, planner in PLANNERS.items():
        if name == 'deviation_seeking':
            order, info = planner(viewpoints, matrix, budget, start=start,
                                  params=None, cost_fn=cost_fn)
        else:
            order, info = planner(viewpoints, budget, start=start,
                                  cost_fn=cost_fn)
        if not order:
            reports[name] = None
            continue
        trajectory = to_trajectory([viewpoints[i] for i in order], name)
        report = evaluate(manifest, scenario, trajectory, params, cost_fn,
                          sensors=sensors)
        report['planner'] = name
        report['planned_distance'] = info.get('distance')
        reports[name] = report

    return reports, {'viewpoints': len(viewpoints), 'base_poses': len(bases),
                     'elements': len(keys), 'path_cost': path_cost,
                     'start': [round(v, 3) for v in start]}


def detections_within(report, limit):
    """Return how many deviations were found within a distance limit."""
    return sum(
        1 for record in report['detections']
        if record.get('distance_at_detection') is not None
        and record['distance_at_detection'] <= limit + 1.0e-9
    )


def _parser():
    parser = argparse.ArgumentParser(
        description='Compare deviation-seeking planning against baselines.')
    parser.add_argument('--manifest', required=True, type=Path)
    parser.add_argument('--scenario', required=True, type=Path)
    parser.add_argument('--urdf', type=Path)
    parser.add_argument('--budget', type=float, default=40.0,
                        help='Traversal distance budget in metres.')
    parser.add_argument('--spacing', type=float, default=1.5,
                        help='Base-pose grid spacing in metres.')
    parser.add_argument('--fixed-sensor', action='store_true',
                        help='Freeze the arm to isolate the manipulator gain.')
    parser.add_argument('--min-visible-fraction', type=float, default=0.05)
    parser.add_argument('--max-range', type=float, default=6.0)
    parser.add_argument('--output', type=Path)
    parser.add_argument(
        '--sensors', choices=['chassis', 'wrist', 'both'], default='both',
        help='Which sensors are active for both planning and scoring.')
    parser.add_argument(
        '--start', nargs=2, type=float, default=None, metavar=('X', 'Y'),
        help='World-frame start position, snapped to the nearest candidate '
             'base pose. Defaults to the westmost candidate, which suits a '
             'whole storey; a single room needs a start inside it.')
    parser.add_argument(
        '--path-cost', choices=['grid', 'euclidean'], default='grid',
        help='grid routes around walls at the platform Nav2 footprint and '
             'costmap resolution; euclidean is the straight-line baseline.')
    return parser


def main(argv=None):
    args = _parser().parse_args(argv)
    params = ObservationParams(
        min_visible_fraction=args.min_visible_fraction, far=args.max_range)
    chain = Chain.from_urdf(
        resolve_urdf(args.urdf), 'base_link', 'camera_0_link')

    reports, info = run(
        _load(args.manifest), _load(args.scenario), chain, args.budget,
        params, fixed_sensor=args.fixed_sensor, spacing=args.spacing,
        path_cost=args.path_cost, start=args.start,
        sensors=SENSOR_SETS[args.sensors],
        chassis=chassis_chain(resolve_urdf(args.urdf)))

    print(f'candidates: {info["viewpoints"]} viewpoints over '
          f'{info["base_poses"]} base poses, {info["elements"]} elements')
    print(f'budget: {args.budget:.1f} m'
          f'{"  (fixed sensor)" if args.fixed_sensor else "  (arm included)"}'
          f'   traversal cost: {info["path_cost"]}   start: {info["start"]}'
          f'   sensors: {args.sensors}')
    print()
    # Planners spend different amounts, so their own totals are not
    # comparable. Score every planner at the same absolute distances.
    checkpoints = [args.budget * f for f in (0.25, 0.5, 0.75, 1.0)]
    columns = ''.join(f'{c:>9.1f}m' for c in checkpoints)
    header = f'{"planner":<20}{"spent":>8}{"found":>7}{columns}'
    print(f'{"":<20}{"":>8}{"":>7}{"  detections within shared budget":<40}')
    print(header)
    print('-' * len(header))
    for name, report in reports.items():
        if report is None:
            print(f'{name:<20}{"-":>8}{"no plan":>7}')
            continue
        summary = report['summary']
        cells = ''.join(
            f'{detections_within(report, c):>10d}' for c in checkpoints)
        print(f'{name:<20}{summary["total_traversal_distance"]:>8.1f}'
              f'{summary["detected_count"]:>7d}{cells}')
    total = next(
        (r['summary']['deviation_count'] for r in reports.values() if r), 0)
    print(f'\n(of {total} injected deviations)')

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            yaml.safe_dump({'candidates': info, 'reports': reports},
                           sort_keys=False), encoding='utf-8')
        print(f'\nWrote {args.output}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
