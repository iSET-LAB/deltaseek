"""Figures for the DeltaSeek extended abstract.

Two figures, each carrying one claim:

``fig_reach``    why exactly two of them are outside the chassis camera's reach
``fig_result``   capability against acquisition cost, per change

Everything is read from the committed results rather than recomputed, so the
figures cannot drift from the numbers in the text.
"""

from decimal import Decimal, ROUND_HALF_UP
import json
from pathlib import Path
import subprocess

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
FIGURES = ROOT / 'publication' / 'ieeeconf' / 'figures'
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
    ax.annotate('Floor', (0.02, 0.06), fontsize=7, color='#444')
    ax.axhline(ceiling, color='#555', lw=1.2)
    ax.annotate(f'Suspended ceiling {r2(ceiling):.2f}', (0.02, ceiling + 0.06),
                fontsize=7, color='#444')

    for value, colour, label in ((chassis, '#c44e52', 'Chassis reach'),
                                 (wrist, '#4c72b0', 'Wrist reach')):
        ax.axhline(value, color=colour, lw=1.4, ls='--')
        ax.annotate(f'{label} {r2(value):.2f}', (0.02, value - 0.19),
                    fontsize=7, color=colour)

    # Where each camera actually sits.
    ax.plot([0.10], [envelope['chassis_height'][0]], marker='o', ms=5,
            color='#c44e52', zorder=5)
    lo, hi = min(envelope['wrist_heights']), max(envelope['wrist_heights'])
    ax.plot([0.18, 0.18], [lo, hi], color='#4c72b0', lw=3.2, solid_capstyle='butt',
            zorder=5)
    ax.annotate('Camera\nheights', (0.145, -0.40), fontsize=6.5, ha='center',
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

    ax.annotate(f'Wrist-only band\n{r2(ceiling - chassis):.2f} m',
                xy=(0.62, (chassis + ceiling) / 2), xytext=(0.72, 1.30),
                fontsize=7, color='#8c2f33', ha='left',
                arrowprops=dict(arrowstyle='->', color='#8c2f33', lw=0.9))

    ax.set_xlim(0, 1.0)
    ax.set_ylim(-0.72, 5.15)
    ax.set_xticks([])
    ax.set_yticks(np.arange(0, 5.1, 1.0))
    ax.set_ylabel('Height above floor [m]')
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
            height=0.36, color='#c44e52', label=f'Chassis ({n_chassis} poses)')
    ax.barh(y - 0.19, [100 * counts['wrist']['counts'][n] / n_wrist for n in order],
            height=0.36, color='#4c72b0', label=f'Wrist ({n_wrist} poses)')
    for index, name in enumerate(order):
        if counts['chassis']['counts'][name] == 0:
            ax.annotate('Never', (0.25, y[index] + 0.19), va='center',
                        fontsize=6.5, color='#8c2f33', weight='bold')
    ax.set_yticks(y)
    ax.set_yticklabels(short, fontsize=7)
    ax.set_xlabel('Admissible sensor poses that observe it [%]')
    ax.set_title('(a) Capability')
    # Headroom above the top bar, so the legend sits clear of the data rather
    # than over it. The bars run to the right edge, so there is no free corner.
    ax.set_ylim(-0.75, len(order) + 0.85)
    ax.legend(loc='upper center', ncol=2, framealpha=0.95, fontsize=7,
              handlelength=1.2, columnspacing=1.1, borderpad=0.35)
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
    ax.annotate('Never', (cap - 0.4, len(order) - 0.55), fontsize=6.5,
                ha='right', color='#8c2f33')
    ax.set_yticks(y)
    ax.set_yticklabels(short, fontsize=7)
    for tick, name in zip(ax.get_yticklabels(), order):
        tick.set_color(CLASS_COLOUR[classes[name]])
    ax.scatter([], [], s=26, color='#4c72b0', label='Wrist')
    ax.scatter([], [], s=26, color='#c44e52', label='Chassis')
    ax.scatter([], [], s=30, color='#c44e52', marker='X', label='Chassis, never')
    ax.set_ylim(-0.75, len(order) + 0.85)
    ax.legend(loc='upper center', ncol=3, framealpha=0.95, fontsize=7,
              handletextpad=0.2, columnspacing=0.9, borderpad=0.35)
    ax.set_xlabel('Drivable distance to first observation [m]')
    ax.set_title('(b) Acquisition cost')
    ax.set_xlim(-1, cap + 1)
    ax.grid(axis='x', alpha=0.15, lw=0.4)
    fig.tight_layout(pad=0.4)
    fig.savefig(FIGURES / 'fig_result.pdf')
    plt.close(fig)


def export_png(dpi=300):
    """Mirror each figure to PNG for viewing outside a LaTeX build.

    The PNG copies are what get opened when checking a figure, and the Word
    build inlines them, so they are
    regenerated here rather than by hand at the 300 dpi that build expects; a
    stale or low-resolution copy is worse than none.
    """
    out = FIGURES / 'png'
    out.mkdir(parents=True, exist_ok=True)
    for name in ('fig_reach', 'fig_result', 'fig_platform'):
        source = FIGURES / f'{name}.pdf'
        if source.exists():
            subprocess.run(['pdftoppm', '-r', str(dpi), '-png', '-singlefile',
                            str(source), str(out / name)], check=True)


def main():
    FIGURES.mkdir(parents=True, exist_ok=True)
    nominal, actual, truth, data, envelope = load()
    fig_reach(truth, actual, envelope)
    fig_result(truth, data)
    export_png()
    print('wrote fig_reach.pdf, fig_result.pdf and png/ copies')


if __name__ == '__main__':
    main()
