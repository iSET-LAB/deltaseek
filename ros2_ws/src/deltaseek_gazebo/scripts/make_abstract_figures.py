"""Figures for the DeltaSeek extended abstract.

Three figures, each carrying one claim:

``fig_scene``    what the benchmark scene is, and where the eight changes sit
``fig_reach``    why exactly two of them are outside the chassis camera's reach
``fig_result``   capability against acquisition cost, per change

Everything is read from the committed results rather than recomputed, so the
figures cannot drift from the numbers in the text.
"""

from decimal import Decimal, ROUND_HALF_UP
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle
import matplotlib.patheffects as pe
import numpy as np
import yaml

from deltaseek_gazebo.benchmark import apply_scenario
from deltaseek_gazebo.visibility import OrientedBox

ROOT = Path('/home/acharjee07/deltaseek')
RESULTS = ROOT / 'results'
FIGURES = ROOT / 'ieeeconf' / 'figures'
BENCH = Path(__file__).resolve().parents[1] / 'config' / 'benchmarks'

CLASS_COLOUR = {'trivial': '#4c72b0', 'height': '#c44e52',
                'incidence': '#dd8452', 'occluded': '#55a868'}
CLASS_ORDER = ('trivial', 'height', 'incidence', 'occluded')
SHELL, FLAT = '#8a8175', {'IfcSlab', 'IfcCovering'}
plt.rcParams.update({'font.size': 8, 'axes.titlesize': 8.5,
                     'axes.labelsize': 8, 'legend.fontsize': 7.5})


def r2(value):
    """Round half up to two decimals, so figure and text agree."""
    return float(Decimal(repr(value)).quantize(Decimal('0.01'), ROUND_HALF_UP))


def load():
    manifest = yaml.safe_load((BENCH / 'hall_b_ablation_nominal.yaml').read_text())
    scenario = yaml.safe_load((BENCH / 'hall_b_ablation_deviations.yaml').read_text())
    nominal, actual, truth = apply_scenario(manifest, scenario)
    data = json.loads((RESULTS / 'sensor_ablation.json').read_text())
    envelope = json.loads((RESULTS / 'sensing_envelope.json').read_text())
    return nominal, actual, truth, data, envelope


def footprint(element):
    box = OrientedBox.from_element(element)
    local = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * box.half_extents[:2]
    return local @ box.rotation[:2, :2].T + box.center[:2]


def fig_scene(nominal, actual, truth):
    classes = {d['element_id']: d['class'] for d in truth['discrepancies']}
    labels = {d['element_id']: d['id'].split('-')[0] for d in truth['discrepancies']}
    fig, ax = plt.subplots(figsize=(3.4, 4.6))
    # Shell first, changes second: a change sitting on top of a modelled unit
    # is smaller than it and would otherwise be painted over.
    for element in actual:
        if element['ifc_class'] in FLAT or element['id'] in classes:
            continue
        ax.add_patch(Polygon(footprint(element), closed=True, fc=SHELL,
                             ec='#3a3a3a', lw=0.4, zorder=2))
    for element in actual:
        eid = element['id']
        if eid not in classes:
            continue
        ax.add_patch(Polygon(footprint(element), closed=True,
                             fc=CLASS_COLOUR[classes[eid]], ec='#111',
                             lw=0.9, zorder=4))
        centre = OrientedBox.from_element(element).center[:2]
        ax.annotate(labels[eid], centre, fontsize=6.5, ha='center',
                    va='center', color='white', weight='bold', zorder=6,
                    path_effects=[pe.withStroke(linewidth=1.6,
                                                foreground=CLASS_COLOUR[classes[eid]])])
    ax.plot(6.3, 4.7, marker='*', ms=11, color='#f0c419', mec='#3a3000', zorder=5)
    ax.annotate('start', (6.3, 4.7), xytext=(6, 5), fontsize=6.5,
                textcoords='offset points')
    ax.set_xlim(2.4, 10.4)
    ax.set_ylim(-6.9, 6.9)
    ax.set_aspect('equal')
    ax.set_xlabel('$x$ [m]')
    ax.set_ylabel('$y$ [m]')
    ax.grid(alpha=0.15, lw=0.4)
    ax.legend(handles=[plt.Rectangle((0, 0), 1, 1, fc=CLASS_COLOUR[c], ec='#1a1a1a',
                                     lw=0.4, label=c) for c in CLASS_ORDER],
              # Lower left: the north-west corner of the room is where H2
              # sits, and a legend there hides it.
              loc='lower left', bbox_to_anchor=(0.01, 0.01), framealpha=0.92,
              handlelength=1.1, borderpad=0.35)
    fig.tight_layout(pad=0.3)
    fig.savefig(FIGURES / 'fig_scene.pdf')
    plt.close(fig)


def fig_reach(truth, actual, envelope):
    # Analytic reaches, not swept ones: the sweep resolves to its grid step
    # and, for the wrist, an earlier sweep reported its own z-range bound.
    chassis = envelope['chassis_reach_analytic']
    wrist = envelope['wrist_reach_analytic']
    ceiling = envelope['ceiling']
    boxes = {e['id']: OrientedBox.from_element(e) for e in actual}
    fig, ax = plt.subplots(figsize=(3.4, 3.0))

    ax.axhspan(chassis, ceiling, xmin=0.0, xmax=0.62, color='#c44e52',
               alpha=0.18, zorder=1)
    ax.axhline(0.0, color='#555', lw=1.2)
    ax.annotate('floor', (0.02, 0.06), fontsize=7, color='#444')
    ax.axhline(ceiling, color='#555', lw=1.2)
    ax.annotate(f'suspended ceiling {r2(ceiling):.2f}', (0.02, ceiling + 0.06),
                fontsize=7, color='#444')

    for value, colour, label in ((chassis, '#c44e52', 'chassis reach'),
                                 (wrist, '#4c72b0', 'wrist reach')):
        ax.axhline(value, color=colour, lw=1.4, ls='--')
        ax.annotate(f'{label} {r2(value):.2f}', (0.02, value - 0.19),
                    fontsize=7, color=colour)

    # Where each camera actually sits.
    ax.plot([0.10], [envelope['chassis_height'][0]], marker='o', ms=5,
            color='#c44e52', zorder=5)
    lo, hi = min(envelope['wrist_heights']), max(envelope['wrist_heights'])
    ax.plot([0.18, 0.18], [lo, hi], color='#4c72b0', lw=3.2, solid_capstyle='butt',
            zorder=5)
    ax.annotate('camera\nheights', (0.145, -0.40), fontsize=6.5, ha='center',
                color='#444')

    for d in truth['discrepancies']:
        if d['class'] != 'height':
            continue
        box = boxes[d['element_id']]
        low = box.center[2] - box.half_extents[2]
        ax.add_patch(Rectangle((0.40, low), 0.14, 2 * box.half_extents[2],
                               fc='#c44e52', ec='#111', lw=0.6, zorder=6))
    ax.annotate('H1, H2\nat 2.25', (0.47, 2.72), fontsize=6.5, ha='center',
                color='#8c2f33')

    ax.annotate(f'wrist-only band\n{r2(ceiling - chassis):.2f} m',
                xy=(0.62, (chassis + ceiling) / 2), xytext=(0.72, 1.30),
                fontsize=7, color='#8c2f33', ha='left',
                arrowprops=dict(arrowstyle='->', color='#8c2f33', lw=0.9))

    ax.set_xlim(0, 1.0)
    ax.set_ylim(-0.72, 5.15)
    ax.set_xticks([])
    ax.set_yticks(np.arange(0, 5.1, 1.0))
    ax.set_ylabel('height above floor [m]')
    ax.set_title('Vertical sensing envelope, 5 m range')
    fig.tight_layout(pad=0.3)
    fig.savefig(FIGURES / 'fig_reach.pdf')
    plt.close(fig)


def fig_result(truth, data):
    order = [d['id'] for d in truth['discrepancies']]
    classes = {d['id']: d['class'] for d in truth['discrepancies']}
    counts = data['pose_counts']
    n_chassis = counts['chassis']['distinct_poses']
    n_wrist = counts['wrist']['distinct_poses']
    rows = {c['deviation']: c for c in data['configurations']['chassis']}
    wrows = {c['deviation']: c for c in data['configurations']['wrist']}

    fig, axes = plt.subplots(1, 2, figsize=(7.1, 2.7))
    y = np.arange(len(order))[::-1]
    short = [n.split('-', 1)[0] for n in order]

    ax = axes[0]
    ax.barh(y + 0.19, [100 * counts['chassis']['counts'][n] / n_chassis for n in order],
            height=0.36, color='#c44e52', label=f'chassis ({n_chassis} poses)')
    ax.barh(y - 0.19, [100 * counts['wrist']['counts'][n] / n_wrist for n in order],
            height=0.36, color='#4c72b0', label=f'wrist ({n_wrist} poses)')
    for index, name in enumerate(order):
        if counts['chassis']['counts'][name] == 0:
            ax.annotate('never', (0.25, y[index] + 0.19), va='center',
                        fontsize=6.5, color='#8c2f33', weight='bold')
    ax.set_yticks(y)
    ax.set_yticklabels(short, fontsize=7)
    ax.set_xlabel('admissible sensor poses that observe it [%]')
    ax.set_title('(a) Capability')
    ax.legend(loc='lower right', framealpha=0.9)
    ax.grid(axis='x', alpha=0.15, lw=0.4)

    ax = axes[1]
    cap = 26.0
    for index, name in enumerate(order):
        c, w = rows[name]['distance_m'], wrows[name]['distance_m']
        c = cap if not np.isfinite(float(c)) else float(c)
        w = cap if not np.isfinite(float(w)) else float(w)
        ax.plot([w, c], [y[index]] * 2, color='#999', lw=0.9, zorder=1)
        ax.scatter(w, y[index], s=26, color='#4c72b0', zorder=3)
        ax.scatter(c, y[index], s=26, color='#c44e52', zorder=3,
                   marker='X' if not np.isfinite(float(rows[name]['distance_m'])) else 'o')
    ax.axvline(cap, color='#c44e52', lw=0.8, ls=':')
    ax.annotate('never', (cap - 0.4, 6.72), fontsize=6.5, ha='right',
                color='#8c2f33')
    ax.set_yticks(y)
    ax.set_yticklabels(short, fontsize=7)
    for tick, name in zip(ax.get_yticklabels(), order):
        tick.set_color(CLASS_COLOUR[classes[name]])
    ax.scatter([], [], s=26, color='#4c72b0', label='wrist')
    ax.scatter([], [], s=26, color='#c44e52', label='chassis')
    ax.scatter([], [], s=30, color='#c44e52', marker='X', label='chassis, never')
    ax.legend(loc='center left', framealpha=0.9, handletextpad=0.2)
    ax.set_xlabel('drivable distance to first observation [m]')
    ax.set_title('(b) Acquisition cost')
    ax.set_xlim(-1, cap + 1)
    ax.grid(axis='x', alpha=0.15, lw=0.4)
    fig.tight_layout(pad=0.4)
    fig.savefig(FIGURES / 'fig_result.pdf')
    plt.close(fig)


def main():
    FIGURES.mkdir(parents=True, exist_ok=True)
    nominal, actual, truth, data, envelope = load()
    fig_scene(nominal, actual, truth)
    fig_reach(truth, actual, envelope)
    fig_result(truth, data)
    print('wrote fig_scene.pdf, fig_reach.pdf, fig_result.pdf')


if __name__ == '__main__':
    main()
