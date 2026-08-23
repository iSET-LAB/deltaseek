"""Which deviations actually need the wrist camera?

A geometric observability study, not a planner comparison. One room, one
hand-placed deviation set, three sensor configurations, one table.

The question is narrow on purpose. The project's claim is that mounting the
camera on the manipulator buys detections a fixed platform cannot get, and the
comparison used to support that froze the arm at one posture -- leaving the
camera on a parked tool flange, which is not a robot anyone builds. Running the
same stations with only the chassis sensor active asks the honest version:
would the sensor the platform already carries have resolved this anyway?

The experiment is designed to be able to answer "yes, all of them". Nothing
here is tuned to prevent that.
"""

import argparse
import csv
import json
import math
from pathlib import Path
import statistics

import numpy as np
import yaml

from deltaseek_gazebo.benchmark import apply_scenario, validate_manifest
from deltaseek_gazebo.detection import (
    CHASSIS, WRIST, ObservationParams, Scene, detect_along, evidence_fractions,
    sensor_params)
from deltaseek_gazebo.evaluate import cumulative_distance
from deltaseek_gazebo.kinematics import Chain
from deltaseek_gazebo.navigation import path_cost_for
from deltaseek_gazebo.planner import plan_deviation_seeking, visibility_matrix
from deltaseek_gazebo.viewpoints import (
    build_viewpoints, chassis_chain, free_base_poses, scene_bounds)
from deltaseek_gazebo.visibility import OrientedBox


CONFIGURATIONS = {
    'chassis': (CHASSIS,),
    'wrist': (WRIST,),
    'both': (CHASSIS, WRIST),
}
CLASS_ORDER = ('trivial', 'height', 'incidence', 'occluded', 'unclassified')


def _load(path):
    return yaml.safe_load(Path(path).read_text(encoding='utf-8'))


def room_base_poses(elements, bounds, spacing, yaws, room=None):
    """Candidate base poses, restricted to the room when one is given.

    Without this the grid spills outside the walls. Those poses are unreachable
    once the solid-box door seals the room, but they still enter the candidate
    set and distort the planner's accounting.
    """
    poses = free_base_poses(elements, bounds, spacing=spacing, yaws=yaws)
    if room is None:
        return poses
    x0, y0, x1, y1 = room
    return [((x, y), yaw) for (x, y), yaw in poses
            if x0 <= x <= x1 and y0 <= y <= y1]


def run_configuration(manifest, scenario, chain, chassis, sensors, args):
    """Plan and score one sensor configuration, returning per-deviation rows."""
    nominal = validate_manifest(manifest)
    elements = nominal['elements']
    bases = room_base_poses(
        elements, scene_bounds(elements), args.spacing,
        (0.0, 1.5707963, 3.1415927, -1.5707963), args.room)
    if not bases:
        raise ValueError('no base poses inside the room bounds')

    viewpoints = build_viewpoints(chain, bases, chassis=chassis)
    params = ObservationParams(
        far=args.max_range, min_visible_fraction=args.min_visible_fraction)

    # The planner sees exactly what the evaluation will score, so a
    # configuration is never credited with information it did not have.
    _, matrix = visibility_matrix(viewpoints, elements, params, sensors)
    cost_fn = path_cost_for(elements)
    start = min((xy for xy, _ in bases),
                key=lambda xy: (xy[0] - args.start[0]) ** 2
                + (xy[1] - args.start[1]) ** 2)
    order, info = plan_deviation_seeking(
        viewpoints, matrix, args.budget, start=start, params=None,
        cost_fn=cost_fn)
    if not order:
        raise ValueError('the planner selected no viewpoints')

    selected = [viewpoints[index] for index in order]
    stations = [viewpoint.poses(sensors) for viewpoint in selected]
    bases_xyz = [viewpoint.base_xyz for viewpoint in selected]

    _, actual_elements, ground_truth = apply_scenario(manifest, scenario)
    records = detect_along(
        stations, nominal['elements'], actual_elements, ground_truth, params,
        sensors=sensors)
    distances = cumulative_distance(
        np.asarray(bases_xyz, dtype=float), cost_fn)

    rows = []
    for record in records:
        index = record['first_viewpoint']
        rows.append({
            'deviation': record['id'],
            'class': record['class'],
            'type': record['type'],
            'detected': bool(record['detected']),
            'distance_m': (round(float(distances[index]), 3)
                           if index is not None else math.inf),
            'resolved_by': record['sensor'] or '',
            'best_visible_fraction': record['best_visible_fraction'],
        })
    return rows, {'viewpoints': len(viewpoints), 'selected': len(selected),
                  'planned_distance': info.get('distance'),
                  'spent': float(distances[-1]) if len(distances) else 0.0}


def observation_counts(manifest, scenario, viewpoints, sensors, params):
    """How many candidate viewpoints observe each deviation, per sensor set.

    The count is the difference between "impossible" and "awkward". A deviation
    seen from 0 poses is a capability limit; one seen from 14 of 240 is a
    routing problem wearing the same clothes in a single planned run.
    """
    nominal = validate_manifest(manifest)
    _, actual_elements, ground_truth = apply_scenario(manifest, scenario)
    scene = Scene.from_elements(actual_elements)
    nominal_boxes = {element['id']: OrientedBox.from_element(element)
                     for element in nominal['elements']}
    counts = {d['id']: 0 for d in ground_truth['discrepancies']}

    # Count distinct sensor poses, not viewpoints. A chassis sensor does not
    # move with the arm, so the same pose recurs once per posture; counting
    # viewpoints would inflate its coverage fivefold and make it look far more
    # capable than it is.
    seen = set()
    total = 0
    for viewpoint in viewpoints:
        for name, (xyz, rpy) in viewpoint.poses(sensors).items():
            key = (name, tuple(round(v, 6) for v in xyz),
                   tuple(round(v, 6) for v in rpy))
            if key in seen:
                continue
            seen.add(key)
            total += 1
            active = sensor_params(params, name)
            camera = active.camera(xyz, rpy)
            for discrepancy in ground_truth['discrepancies']:
                fractions = evidence_fractions(
                    camera, discrepancy, nominal_boxes, scene, active)
                if fractions and max(fractions.values()) >= \
                        active.min_visible_fraction:
                    counts[discrepancy['id']] += 1
    return {'counts': counts, 'distinct_poses': total}


def observable_anywhere(manifest, scenario, viewpoints, sensors, params):
    """Deviations a sensor set can resolve from *any* candidate viewpoint.

    Whether the planner reached a viewpoint inside its budget is a routing
    outcome; whether one exists at all is a capability. Reporting only the
    planned run confuses the two, and the headline then moves with routing
    noise: a panel visible from 14 of 240 chassis poses is not evidence that
    the wrist camera is required, however the route happened to fall.
    """
    nominal = validate_manifest(manifest)
    _, actual_elements, ground_truth = apply_scenario(manifest, scenario)
    stations = [viewpoint.poses(sensors) for viewpoint in viewpoints]
    records = detect_along(
        stations, nominal['elements'], actual_elements, ground_truth, params,
        sensors=sensors)
    return {record['id'] for record in records if record['detected']}


def summarise(results):
    """Return the per-class table and the headline counts."""
    classes = sorted(
        {row['class'] for rows in results.values() for row in rows},
        key=lambda name: CLASS_ORDER.index(name)
        if name in CLASS_ORDER else len(CLASS_ORDER))
    table = []
    for name in classes:
        entry = {'class': name}
        for config, rows in results.items():
            group = [row for row in rows if row['class'] == name]
            entry[config] = (sum(1 for row in group if row['detected']),
                             len(group))
        table.append(entry)
    return classes, table


def _median_distance(rows):
    found = [row['distance_m'] for row in rows if row['detected']]
    return statistics.median(found) if found else math.inf


def _fmt(value):
    return 'never' if math.isinf(value) else f'{value:.1f} m'


def distance_penalty(results):
    """Distance the chassis-only configuration pays for what it can still see.

    A deviation both configurations resolve is not evidence that the arm is
    useless there: occlusion can convert into extra driving rather than into a
    missed detection, and only the distances show which happened.
    """
    chassis = {row['deviation']: row for row in results['chassis']}
    wrist = {row['deviation']: row for row in results['wrist']}
    shared = [name for name in chassis
              if chassis[name]['detected'] and wrist[name]['detected']]
    rows = []
    for name in shared:
        rows.append((name, chassis[name]['class'],
                     chassis[name]['distance_m'], wrist[name]['distance_m']))
    return rows


def report(results, info, out_dir):
    """Write the CSV and the markdown table, and print the summary."""
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / 'sensor_ablation.csv'
    with csv_path.open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=[
            'configuration', 'deviation', 'class', 'type', 'detected',
            'distance_m', 'resolved_by', 'best_visible_fraction'])
        writer.writeheader()
        for config, rows in results.items():
            for row in rows:
                writer.writerow({'configuration': config, **row})

    reachable = info.get('chassis_capability', set())

    classes, table = summarise(results)
    lines = ['# Sensor-configuration ablation, Hall B', '',
             'Detections by observability class, as `found / total`.', '',
             '| class | ' + ' | '.join(results) + ' |',
             '| --- | ' + ' | '.join('---' for _ in results) + ' |']
    for entry in table:
        cells = ' | '.join(f'{entry[c][0]}/{entry[c][1]}' for c in results)
        lines.append(f'| {entry["class"]} | {cells} |')
    totals = ' | '.join(
        f'**{sum(1 for r in rows if r["detected"])}/{len(rows)}**'
        for rows in results.values())
    lines += [f'| **all** | {totals} |', '',
              '| configuration | median distance to detection |',
              '| --- | --- |']
    for config, rows in results.items():
        value = _median_distance(rows)
        lines.append(f'| {config} | '
                     + ('never' if math.isinf(value) else f'{value:.1f} m')
                     + ' |')
    lines += ['',
              '## Capability versus routing',
              '',
              'Deviations the chassis camera cannot resolve from any candidate '
              'viewpoint, as opposed to ones it merely failed to reach inside '
              'the budget.', '',
              '| deviation | class | chassis can ever see it |',
              '| --- | --- | --- |']
    for row in results['both']:
        lines.append(f'| {row["deviation"]} | {row["class"]} | '
                     + ('yes' if row['deviation'] in reachable else '**no**')
                     + ' |')
    lines += ['',
              '## What the chassis camera pays instead',
              '',
              'Deviations both configurations resolve, and the distance each '
              'spent getting there.', '',
              '| deviation | class | chassis | wrist |',
              '| --- | --- | --- | --- |']
    for name, cls, c_d, w_d in distance_penalty(results):
        lines.append(f'| {name} | {cls} | {_fmt(c_d)} | {_fmt(w_d)} |')
    lines += ['', 'Per-deviation detail is in `sensor_ablation.csv`; the '
              'machine-readable form, including per-viewpoint observation '
              'counts, is in `sensor_ablation.json`.', '']
    md_path = out_dir / 'sensor_ablation.md'
    md_path.write_text('\n'.join(lines), encoding='utf-8')

    json_path = out_dir / 'sensor_ablation.json'
    json_path.write_text(json.dumps({
        'configurations': {name: rows for name, rows in results.items()},
        'chassis_capability': sorted(reachable),
        'pose_counts': info.get('pose_counts', {}),
        'candidate_base_poses': info.get('candidate_base_poses'),
        'viewpoints': info['both']['viewpoints'],
    }, indent=2, default=str), encoding='utf-8')

    print(f'candidates: {info["both"]["viewpoints"]} viewpoints, '
          f'{len(next(iter(results.values())))} deviations\n')
    header = f'{"class":<14}' + ''.join(f'{c:>12}' for c in results)
    print(header)
    print('-' * len(header))
    for entry in table:
        print(f'{entry["class"]:<14}'
              + ''.join(f'{entry[c][0]:>7}/{entry[c][1]:<4}' for c in results))
    print('-' * len(header))
    print(f'{"all":<14}' + ''.join(
        f'{sum(1 for r in rows if r["detected"]):>7}/{len(rows):<4}'
        for rows in results.values()))
    print(f'\n{"median dist":<14}' + ''.join(
        f'{_fmt(_median_distance(rows)):>12}' for rows in results.values()))
    print(f'{"distance spent":<14}' + ''.join(
        f'{info[c]["spent"]:>11.1f}m' for c in results))

    chassis_rows = {r['deviation']: r for r in results['chassis']}
    both_rows = {r['deviation']: r for r in results['both']}
    classes = {r['deviation']: r['class'] for r in results['both']}
    reachable = info.get('chassis_capability')
    only_arm = [d for d in both_rows if d not in reachable]
    budget_only = [d for d, r in both_rows.items()
                   if d in reachable and not chassis_rows[d]['detected']]
    missed = [d for d, r in both_rows.items() if not r['detected']]

    print('\n' + '=' * len(header))
    if not only_arm:
        print('FINDING: every deviation is visible to the chassis camera from')
        print('somewhere in the candidate set. On this scene the wrist camera')
        print('adds no capability, which contradicts the project premise.')
    else:
        print(f'FINDING: {len(only_arm)} of {len(both_rows)} deviations are '
              f'invisible to the chassis camera from every')
        print('candidate viewpoint, so they require the wrist camera:')
        for name in only_arm:
            print(f'  - {name}  [{classes[name]}]')
    if budget_only:
        print(f'\n{len(budget_only)} further deviations the chassis-only run '
              f'missed are visible to it somewhere,')
        print('and were simply not reached within the budget. These are '
              'routing outcomes,')
        print('not evidence for the arm:')
        for name in budget_only:
            print(f'  - {name}  [{classes[name]}]')
    shared = distance_penalty(results)
    worse = [row for row in shared if row[2] > row[3] + 1.0e-9]
    if shared:
        print(f'\nOf the {len(shared)} deviations both configurations resolved, '
              f'the chassis-only run reached {len(worse)} of them later:')
        for name, cls, c_d, w_d in sorted(shared, key=lambda r: r[3] - r[2]):
            mark = '  <-- ' if c_d > w_d + 1.0e-9 else '      '
            print(f'  {name:<24}[{cls:<9}] chassis {_fmt(c_d):>8}   '
                  f'wrist {_fmt(w_d):>8}{mark}')
        print('Where both succeed, occlusion is a routing cost rather than a')
        print('sensing limit: it is paid in metres, not in missed deviations.')
    if missed:
        print(f'\nNot resolved by any configuration within the budget: '
              f'{", ".join(missed)}')
    print(f'\nWrote {csv_path}\nWrote {md_path}\nWrote {json_path}')
    return only_arm


def _parser():
    parser = argparse.ArgumentParser(
        description='Which deviations require the wrist-mounted camera?')
    parser.add_argument('--manifest', required=True, type=Path)
    parser.add_argument('--scenario', required=True, type=Path)
    parser.add_argument('--urdf', type=Path)
    parser.add_argument('--budget', type=float, default=25.0)
    parser.add_argument('--max-range', type=float, default=5.0)
    parser.add_argument('--min-visible-fraction', type=float, default=0.05)
    parser.add_argument('--spacing', type=float, default=1.0)
    parser.add_argument('--start', nargs=2, type=float, default=[6.4, 4.5])
    parser.add_argument(
        '--room', nargs=4, type=float, default=None,
        metavar=('X0', 'Y0', 'X1', 'Y1'),
        help='Restrict candidate base poses to this rectangle.')
    parser.add_argument('--output-dir', type=Path, default=Path('results'))
    return parser


def main(argv=None):
    from deltaseek_gazebo.compare import resolve_urdf
    args = _parser().parse_args(argv)
    urdf = resolve_urdf(args.urdf)
    chain = Chain.from_urdf(urdf, 'base_link', 'camera_0_link')
    chassis = chassis_chain(urdf)
    manifest, scenario = _load(args.manifest), _load(args.scenario)

    results, info = {}, {}
    for config, sensors in CONFIGURATIONS.items():
        rows, meta = run_configuration(
            manifest, scenario, chain, chassis, sensors, args)
        results[config], info[config] = rows, meta

    nominal = validate_manifest(manifest)
    bases = room_base_poses(
        nominal['elements'], scene_bounds(nominal['elements']), args.spacing,
        (0.0, 1.5707963, 3.1415927, -1.5707963), args.room)
    sweep = build_viewpoints(chain, bases, chassis=chassis)
    params = ObservationParams(far=args.max_range,
                               min_visible_fraction=args.min_visible_fraction)
    info['chassis_capability'] = observable_anywhere(
        manifest, scenario, sweep, CONFIGURATIONS['chassis'], params)
    info['pose_counts'] = {
        name: observation_counts(manifest, scenario, sweep, sensors, params)
        for name, sensors in (('chassis', CONFIGURATIONS['chassis']),
                              ('wrist', CONFIGURATIONS['wrist']))
    }
    info['candidate_base_poses'] = len(bases)
    report(results, info, args.output_dir)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
