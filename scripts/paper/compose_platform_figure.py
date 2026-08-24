"""Assemble the three-panel platform figure from the captured renders."""

from pathlib import Path
import subprocess

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle
import numpy as np
import yaml
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
SHOTS = ROOT / 'publication' / 'ieeeconf' / 'figures' / 'shots'
OUT = ROOT / 'publication' / 'ieeeconf' / 'figures' / 'fig_platform.pdf'
CAM_HFOV, CAM_W, CAM_H = 1.05, 1600, 1100
BLUE, ORANGE = '#2f6fb5', '#d1660f'
# Panel box aspects (width / height). Every panel is cropped to its own box so
# the three sit at one height.
A_ASPECT, B_ASPECT, SUB_ASPECT = 1.26, 1.52, 1.00
plt.rcParams.update({'font.size': 8})


def project(points, eye, R):
    local = (np.atleast_2d(points) - np.asarray(eye)) @ np.asarray(R)
    fx = (CAM_W / 2.0) / np.tan(CAM_HFOV / 2.0)
    fwd = local[:, 0]
    return np.column_stack([CAM_W/2.0 - fx*local[:, 1]/fwd,
                            CAM_H/2.0 - fx*local[:, 2]/fwd]), fwd


def draw_frustum(ax, meta, colour, label):
    eye, R = meta['eye'], meta['R']
    origin = np.array(meta['camera'])
    fars = np.array([r[1] for r in meta['rays']])
    o_px, _ = project(origin[None, :], eye, R)
    f_px, fwd = project(fars, eye, R)
    if (fwd <= 0).any():                      # a corner behind the render camera
        return
    # Order the four far corners into a quadrilateral by angle about their mean.
    centre = f_px.mean(axis=0)
    order = np.argsort(np.arctan2(f_px[:, 1]-centre[1], f_px[:, 0]-centre[0]))
    quad = f_px[order]
    ax.add_patch(Polygon(quad, closed=True, fc=colour, alpha=0.16, ec='none',
                         zorder=3))
    for corner in quad:
        ax.plot([o_px[0, 0], corner[0]], [o_px[0, 1], corner[1]],
                color=colour, lw=1.0, alpha=0.85, zorder=4)
    ax.add_patch(Polygon(quad, closed=True, fill=False, ec=colour, lw=1.2,
                         ls='--', zorder=4))
    ax.plot(o_px[0, 0], o_px[0, 1], marker='o', ms=5, color=colour, zorder=6)
    ax.annotate(label, (o_px[0, 0], o_px[0, 1]), textcoords='offset points',
                xytext=(8, -12), color=colour, fontsize=7.5, weight='bold',
                zorder=6)


def mark_target(ax, meta):
    centre = np.array(meta['target'])
    half = np.array(meta['target_extent'])
    corners = centre + np.array([[sx*half[0], sy*half[1], sz*half[2]]
                                 for sx in (-1, 1) for sy in (-1, 1)
                                 for sz in (-1, 1)])
    px, fwd = project(corners, meta['eye'], meta['R'])
    if (fwd <= 0).any():
        return
    lo, hi = px.min(axis=0), px.max(axis=0)
    pad = 14
    ax.add_patch(Rectangle((lo[0]-pad, lo[1]-pad), hi[0]-lo[0]+2*pad,
                           hi[1]-lo[1]+2*pad, fill=False, ec='#111', lw=1.4,
                           zorder=7))
    ax.annotate('Target change', ((lo[0]+hi[0])/2, lo[1]-pad-8), ha='center',
                va='bottom', fontsize=7.5, weight='bold', color='#111', zorder=7)


def crop_to(image, aspect):
    """Centre-crop to a target width/height so every panel shares a height.

    imshow forces the data aspect and shrinks the axes to match, so panels of
    differing image aspect end up different heights however the gridspec is
    weighted. Cropping to the box aspect is what keeps the row aligned.
    """
    w, h = image.size
    if w / h > aspect:
        new_w = int(round(h * aspect))
        left = (w - new_w) // 2
        return image.crop((left, 0, left + new_w, h))
    new_h = int(round(w / aspect))
    top = (h - new_h) // 2
    return image.crop((0, top, w, top + new_h))


def panel(ax, image_path, title=None, aspect=None):
    image = Image.open(image_path)
    if aspect is not None:
        image = crop_to(image, aspect)
    ax.imshow(image)
    ax.set_xticks([]); ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_edgecolor('#333'); spine.set_linewidth(0.8)
    if title:
        ax.set_title(title, fontsize=8.5, pad=4)


def main():
    meta = yaml.safe_load((SHOTS / 'frustum_meta.yaml').read_text())
    fig = plt.figure(figsize=(7.16, 2.05))
    gs = fig.add_gridspec(1, 3,
                          width_ratios=[A_ASPECT, B_ASPECT, 2*SUB_ASPECT],
                          wspace=0.055, left=0.006, right=0.994, top=0.955,
                          bottom=0.155)

    ax = fig.add_subplot(gs[0, 0])
    panel(ax, SHOTS / 'raw_platform.png', aspect=A_ASPECT)
    ax.set_xlabel('(a) Platform', fontsize=8, labelpad=4)

    ax = fig.add_subplot(gs[0, 1])
    panel(ax, SHOTS / 'raw_environment.png', aspect=B_ASPECT)
    ax.set_xlabel('(b) IFC-derived environment', fontsize=8, labelpad=4)

    inner = gs[0, 2].subgridspec(1, 2, wspace=0.03)
    for column, (sensor, colour, name) in enumerate(
            ((('chassis'), BLUE, 'Chassis-mounted RGB-D'),
             (('wrist'), ORANGE, 'Wrist-mounted RGB-D'))):
        sub = fig.add_subplot(inner[0, column])
        panel(sub, SHOTS / f'raw_frustum_{sensor}.png')
        draw_frustum(sub, meta[sensor], colour, sensor.capitalize())
        mark_target(sub, meta[sensor])
        # Keep the overlay in full-image coordinates and crop with the limits,
        # so the projected frusta stay registered to the render.
        half = CAM_H * SUB_ASPECT / 2.0
        sub.set_xlim(CAM_W/2.0 - half, CAM_W/2.0 + half)
        sub.set_ylim(CAM_H, 0)
        sub.set_title(name, fontsize=7.5, color=colour, pad=3)
        if column == 0:
            sub.set_xlabel('(c) Perceptual action spaces', fontsize=8,
                           labelpad=4)
            sub.xaxis.set_label_coords(1.03, -0.055)

    fig.savefig(OUT, dpi=300)
    # Mirror to the tracked PNG the README shows.
    png = ROOT / 'docs' / 'figures'
    png.mkdir(parents=True, exist_ok=True)
    subprocess.run(['pdftoppm', '-r', '300', '-png', '-singlefile',
                    str(OUT), str(png / 'fig_platform')], check=True)
    print('wrote', OUT, 'and', png / 'fig_platform.png')


if __name__ == '__main__':
    main()
