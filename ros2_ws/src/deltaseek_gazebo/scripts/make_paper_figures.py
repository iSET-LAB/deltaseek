"""Produce the data and figures reported in the workshop abstract.

Everything the paper claims is generated here, so a number in the text can be
traced to a command rather than to a spreadsheet. Running this script
regenerates both the JSON summary and the figures.
"""

import argparse
import json
import math
from pathlib import Path
import statistics

import matplotlib
matplotlib.use('Agg')
import matplotlib.patches as patches            # noqa: E402
import matplotlib.pyplot as plt                 # noqa: E402
import numpy as np                              # noqa: E402
import yaml                                     # noqa: E402

from deltaseek_gazebo.benchmark import apply_scenario, validate_manifest
from deltaseek_gazebo.detection import ObservationParams
from deltaseek_gazebo.deviation_sampler import _parser as sampler_parser
from deltaseek_gazebo.deviation_sampler import sample_scenario
from deltaseek_gazebo.evaluate import evaluate
from deltaseek_gazebo.kinematics import Chain
from deltaseek_gazebo.navigation import path_cost_for
from deltaseek_gazebo.planner import PLANNERS, euclidean, visibility_matrix
from deltaseek_gazebo.synthetic_storey import _parser as storey_parser
from deltaseek_gazebo.synthetic_storey import build_storey
from deltaseek_gazebo.viewpoints import (
    build_viewpoints,
    fixed_sensor_viewpoints,
    free_base_poses,
    scene_bounds,
    to_trajectory,
)
from deltaseek_gazebo.visibility import OrientedBox


LABELS = {
    'deviation_seeking': 'Deviation-seeking (ours)',
    'coverage': 'Coverage',
    'frontier': 'Frontier',
    'goal_directed': 'Goal-directed',
}
COLOURS = {
    'deviation_seeking': '#0b6fa4',
    'coverage': '#c2571a',
    'frontier': '#4a7c3f',
    'goal_directed': '#8a8f98',
}
YAWS = (0.0, 1.5707963, 3.1415927, -1.5707963)


def _style():
    plt.rcParams.update({
        'font.size': 8,
        'axes.labelsize': 8,
        'axes.titlesize': 8,
        'legend.fontsize': 7,
        'xtick.labelsize': 7,
        'ytick.labelsize': 7,
        'axes.spines.top': False,
        'axes.spines.right': False,
        'figure.dpi': 200,
        'savefig.bbox': 'tight',
        'savefig.pad_inches': 0.02,
    })


def detection_curve(report, grid):
    """Return detection rate sampled on a common distance axis."""
    total = report['summary']['deviation_count']
    found = sorted(
        record['distance_at_detection']
        for record in report['detections']
        if record.get('distance_at_detection') is not None
    )
    counts = np.searchsorted(found, grid, side='right')
    return 100.0 * counts / max(total, 1)


def run_experiment(args):
    """Sweep planners over seeds, sensor configurations and cost models."""
    storey_args = storey_parser().parse_args(
        ['--output', 'unused', '--rooms-per-side', str(args.rooms),
         '--seed', str(args.layout_seed)])
    manifest = build_storey(storey_args)
    elements = validate_manifest(manifest)['elements']
    params = ObservationParams(far=args.max_range)

    chain = Chain.from_urdf(args.urdf, 'base_link', 'camera_0_link')
    bases = free_base_poses(
        elements, scene_bounds(elements), spacing=args.spacing, yaws=YAWS)
    start = min((xy for xy, _ in bases), key=lambda xy: (xy[0], abs(xy[1])))
    grid = np.linspace(0.0, args.budget, 121)

    scenarios = {}
    for seed in args.seeds:
        sampled = sampler_parser().parse_args(
            ['--manifest', 'x', '--output', 'x', '--density', str(args.density),
             '--unexpected-rate', '0.05', '--seed', str(seed)])
        scenarios[seed] = sample_scenario(manifest, sampled)

    results = {}
    tours = {}
    for sensor, builder in (('arm', build_viewpoints),
                            ('fixed', fixed_sensor_viewpoints)):
        viewpoints = builder(chain, bases)
        _, matrix = visibility_matrix(viewpoints, elements, params)
        for cost_name, cost_fn in (('drivable', path_cost_for(elements)),
                                   ('straight', euclidean)):
            for name, planner in PLANNERS.items():
                curves, finals = [], []
                for seed, scenario in scenarios.items():
                    if name == 'deviation_seeking':
                        order, _ = planner(viewpoints, matrix, args.budget,
                                           start=start, params=None,
                                           cost_fn=cost_fn)
                    else:
                        order, _ = planner(viewpoints, args.budget,
                                           start=start, cost_fn=cost_fn)
                    if not order:
                        curves.append(np.zeros_like(grid))
                        finals.append(0.0)
                        continue
                    trajectory = to_trajectory(
                        [viewpoints[i] for i in order], name)
                    report = evaluate(manifest, scenario, trajectory,
                                      params, cost_fn)
                    curves.append(detection_curve(report, grid))
                    finals.append(100.0 * report['summary']['detection_rate'])
                    if (sensor == 'arm' and cost_name == 'drivable'
                            and seed == args.seeds[0]):
                        tours[name] = [
                            viewpoints[i].base_xy for i in order]
                results[f'{sensor}|{cost_name}|{name}'] = {
                    'grid': grid.tolist(),
                    'mean': np.mean(curves, axis=0).tolist(),
                    'std': np.std(curves, axis=0).tolist(),
                    'final_mean': statistics.mean(finals),
                    'final_std': statistics.pstdev(finals),
                    'finals': finals,
                }

    _, _, ground_truth = apply_scenario(manifest, scenarios[args.seeds[0]])
    return {
        'results': results,
        'tours': {k: [list(map(float, xy)) for xy in v]
                  for k, v in tours.items()},
        'manifest': manifest,
        'ground_truth': ground_truth,
        'config': {
            'budget': args.budget, 'density': args.density,
            'max_range': args.max_range, 'spacing': args.spacing,
            'seeds': args.seeds, 'elements': len(elements),
            'base_poses': len(bases),
        },
    }


def figure_pipeline(path):
    """Draw the benchmark pipeline as a labelled block diagram."""
    _style()
    fig, ax = plt.subplots(figsize=(7.0, 1.5))
    ax.set_axis_off()
    blocks = [
        ('IFC model\nor synthetic\nstorey', '#dbe7f0'),
        ('Element\nmanifest\n(GlobalId)', '#dbe7f0'),
        ('Deviation\nsampler\n(density, seed)', '#f6e0cf'),
        ('Gazebo world\n+ exact\nground truth', '#f6e0cf'),
        ('Viewpoint\nplanner\n(nominal only)', '#d9e8d4'),
        ('Scoring vs\ntraversal\nbudget', '#e8e2ef'),
    ]
    width, gap = 1.0, 0.30
    for index, (text, colour) in enumerate(blocks):
        x = index * (width + gap)
        ax.add_patch(patches.FancyBboxPatch(
            (x, 0), width, 0.85, boxstyle='round,pad=0.02',
            facecolor=colour, edgecolor='#44484d', linewidth=0.7))
        ax.text(x + width / 2, 0.425, text, ha='center', va='center',
                fontsize=6.6, linespacing=1.35)
        if index < len(blocks) - 1:
            ax.annotate('', xy=(x + width + gap, 0.425),
                        xytext=(x + width, 0.425),
                        arrowprops=dict(arrowstyle='-|>', linewidth=0.8,
                                        color='#44484d'))
    ax.text(3.9, -0.22, 'ground truth is withheld from the planner and used only for scoring',
            ha='center', fontsize=6.2, style='italic', color='#55595e')
    ax.set_xlim(-0.1, len(blocks) * (width + gap))
    ax.set_ylim(-0.42, 0.95)
    fig.savefig(path)
    plt.close(fig)


def figure_scene(path, manifest, ground_truth, tours):
    """Plan view of the storey with deviations and one planned tour."""
    _style()
    fig, ax = plt.subplots(figsize=(3.4, 2.5))
    elements = validate_manifest(manifest)['elements']
    deviated = {d['element_id'] for d in ground_truth['discrepancies']}

    for element in elements:
        box = OrientedBox.from_element(element)
        size = 2.0 * box.half_extents
        low = box.center[:2] - box.half_extents[:2]
        ifc = element['ifc_class']
        if ifc == 'IfcSlab':
            continue
        overhead = box.center[2] - box.half_extents[2] >= 1.0
        face = '#c9ced4' if not overhead else '#e9edf1'
        ax.add_patch(patches.Rectangle(
            low, size[0], size[1], facecolor=face,
            edgecolor='#9aa1a9', linewidth=0.25, zorder=1))

    for discrepancy in ground_truth['discrepancies']:
        pose = discrepancy.get('actual_pose') or discrepancy.get('reference_pose')
        if pose is None:
            continue
        ax.plot(pose['xyz'][0], pose['xyz'][1], marker='o', markersize=3.0,
                markerfacecolor='#d1495b', markeredgecolor='white',
                markeredgewidth=0.4, zorder=4)

    tour = tours.get('deviation_seeking', [])
    if tour:
        xs = [p[0] for p in tour]
        ys = [p[1] for p in tour]
        ax.plot(xs, ys, color='#0b6fa4', linewidth=0.9, alpha=0.9, zorder=3)
        ax.plot(xs, ys, linestyle='none', marker='.', markersize=2.0,
                color='#0b6fa4', zorder=3)
        ax.plot(xs[0], ys[0], marker='s', markersize=3.5, color='#0b6fa4',
                zorder=5)

    ax.plot([], [], marker='o', linestyle='none', markersize=3,
            markerfacecolor='#d1495b', markeredgecolor='white',
            label='injected deviation')
    ax.plot([], [], color='#0b6fa4', linewidth=0.9, label='planned tour')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.18), ncol=2,
              frameon=False, handlelength=1.4, columnspacing=1.2)
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_yticks([-5, 0, 5])
    fig.savefig(path)
    plt.close(fig)


def figure_curves(path, results, budget):
    """Detection rate against traversal budget, per cost model."""
    _style()
    fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.35), sharey=True)
    panels = [('drivable', 'Drivable distance (around walls)'),
              ('straight', 'Straight-line distance')]
    for ax, (cost_name, title) in zip(axes, panels):
        for name in ('deviation_seeking', 'coverage', 'frontier',
                     'goal_directed'):
            entry = results[f'arm|{cost_name}|{name}']
            grid = np.array(entry['grid'])
            mean = np.array(entry['mean'])
            std = np.array(entry['std'])
            ax.plot(grid, mean, color=COLOURS[name], linewidth=1.3,
                    label=LABELS[name])
            ax.fill_between(grid, mean - std, mean + std, color=COLOURS[name],
                            alpha=0.13, linewidth=0)
        ax.set_title(title)
        ax.set_xlabel('traversal budget [m]')
        ax.set_xlim(0, budget)
        ax.set_ylim(0, 100)
        ax.grid(alpha=0.16, linewidth=0.4)
    axes[0].set_ylabel('deviations detected [%]')
    axes[0].legend(loc='upper left', frameon=False)
    fig.savefig(path)
    plt.close(fig)


def figure_ablation(path, results):
    """Final detection rate with the arm included versus frozen."""
    _style()
    fig, ax = plt.subplots(figsize=(3.4, 2.0))
    names = ['deviation_seeking', 'coverage', 'frontier', 'goal_directed']
    positions = np.arange(len(names))
    width = 0.36
    for offset, sensor, colour, label in (
            (-width / 2, 'arm', '#0b6fa4', 'arm included'),
            (width / 2, 'fixed', '#b9c4cd', 'arm frozen')):
        means = [results[f'{sensor}|drivable|{n}']['final_mean'] for n in names]
        errs = [results[f'{sensor}|drivable|{n}']['final_std'] for n in names]
        ax.bar(positions + offset, means, width, yerr=errs, capsize=2,
               color=colour, edgecolor='none', label=label,
               error_kw={'elinewidth': 0.7, 'capthick': 0.7})
    ax.set_xticks(positions)
    ax.set_xticklabels(['deviation-\nseeking', 'coverage', 'frontier',
                        'goal-\ndirected'], fontsize=6.4)
    ax.set_ylabel('detected [%]')
    ax.set_ylim(0, 100)
    ax.grid(axis='y', alpha=0.16, linewidth=0.4)
    ax.legend(frameon=False, loc='upper right')
    fig.savefig(path)
    plt.close(fig)


def _parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out-dir', type=Path, required=True)
    parser.add_argument('--urdf', type=Path,
                        default=Path('/tmp/deltaseek_robot.urdf'))
    parser.add_argument('--budget', type=float, default=60.0)
    parser.add_argument('--density', type=float, default=0.2)
    parser.add_argument('--max-range', type=float, default=5.0)
    parser.add_argument('--spacing', type=float, default=1.5)
    parser.add_argument('--rooms', type=int, default=4)
    parser.add_argument('--layout-seed', type=int, default=1)
    parser.add_argument('--seeds', type=int, nargs='+',
                        default=[5, 6, 7, 8, 9])
    return parser


def main(argv=None):
    args = _parser().parse_args(argv)
    args.out_dir.mkdir(parents=True, exist_ok=True)

    data = run_experiment(args)
    results = data['results']

    summary = {
        'config': data['config'],
        'final': {
            key: {'mean': round(value['final_mean'], 2),
                  'std': round(value['final_std'], 2),
                  'per_seed': [round(v, 2) for v in value['finals']]}
            for key, value in results.items()
        },
    }
    (args.out_dir / 'results.json').write_text(
        json.dumps(summary, indent=2), encoding='utf-8')

    figure_pipeline(args.out_dir / 'fig_pipeline.pdf')
    figure_scene(args.out_dir / 'fig_scene.pdf', data['manifest'],
                 data['ground_truth'], data['tours'])
    figure_curves(args.out_dir / 'fig_curves.pdf', results, args.budget)
    figure_ablation(args.out_dir / 'fig_ablation.pdf', results)

    print(f'wrote figures and results.json to {args.out_dir}')
    print(f"{'configuration':38s} {'detected %':>12s}")
    for key in sorted(summary['final'],
                      key=lambda k: -summary['final'][k]['mean']):
        entry = summary['final'][key]
        print(f"  {key:36s} {entry['mean']:6.1f} +/- {entry['std']:4.1f}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
