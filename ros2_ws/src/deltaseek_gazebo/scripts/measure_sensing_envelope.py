"""Measure each sensor's vertical reach, analytically and by sweep.

The reach is the highest world point a camera can place inside its frustum at
the configured range. It is what decides whether a change is observable at all
on height grounds, so it is derived in closed form and cross-checked against a
numeric sweep rather than taken from either alone.

An earlier sweep reported the wrist reach as its own z-range upper bound. The
closed form has no such failure mode, and the sweep here is bounded by the
closed form plus a margin so it cannot silently saturate again.
"""

import json
import math
from pathlib import Path

import numpy as np

from deltaseek_gazebo.compare import resolve_urdf
from deltaseek_gazebo.detection import CHASSIS, WRIST, ObservationParams
from deltaseek_gazebo.kinematics import Chain
from deltaseek_gazebo.viewpoints import build_viewpoints, chassis_chain
from deltaseek_gazebo.visibility import rotation_matrix

OUT = Path('/home/acharjee07/deltaseek/results/sensing_envelope.json')
CEILING = 2.44          # underside of the suspended ceiling in Hall B
FAR = 5.0


def closed_form(xyz, rpy, params):
    """Highest world z inside the frustum.

    A frustum point is (f, y, z) in camera axes with f <= far, |y| <= f tan_h,
    |z| <= f tan_v, so world height is linear in f and maximised at the far
    plane, at whichever corner the rotation's z-row favours.
    """
    tan_h = math.tan(params.hfov / 2.0)
    tan_v = tan_h / params.aspect
    row = rotation_matrix(rpy)[2]
    gain = row[0] + abs(row[1]) * tan_h + abs(row[2]) * tan_v
    return float(xyz[2]) + params.far * max(gain, 0.0)


def main():
    urdf = resolve_urdf(None)
    chain = Chain.from_urdf(urdf, 'base_link', 'camera_0_link')
    params = ObservationParams(far=FAR)

    analytic = {CHASSIS: -np.inf, WRIST: -np.inf}
    heights = {CHASSIS: set(), WRIST: set()}
    for yaw in np.linspace(0.0, 2*np.pi, 24, endpoint=False):
        for viewpoint in build_viewpoints(chain, [((0.0, 0.0), yaw)],
                                          chassis=chassis_chain(urdf)):
            for name, (xyz, rpy) in viewpoint.poses().items():
                analytic[name] = max(analytic[name],
                                     closed_form(xyz, rpy, params))
                heights[name].add(round(float(xyz[2]), 3))

    # Numeric cross-check, bounded above the closed form so it cannot saturate.
    radial = np.arange(0.05, FAR + 1.01, 0.02)
    angles = np.linspace(0.0, 2*np.pi, 96, endpoint=False)
    xs = np.outer(radial, np.cos(angles)).ravel()
    ys = np.outer(radial, np.sin(angles)).ravel()
    swept = {}
    for name in (CHASSIS, WRIST):
        ceiling = analytic[name]
        grid = np.arange(ceiling - 0.30, ceiling + 0.20, 0.001)
        found = 0.0
        for yaw in np.linspace(0.0, 2*np.pi, 8, endpoint=False):
            for viewpoint in build_viewpoints(chain, [((0.0, 0.0), yaw)],
                                              chassis=chassis_chain(urdf)):
                pose = viewpoint.poses().get(name)
                if pose is None:
                    continue
                camera = params.camera(*pose)
                for z in grid:
                    if z <= found:
                        continue
                    if camera.contains(
                            np.stack([xs, ys, np.full_like(xs, z)], -1)).any():
                        found = float(z)
        swept[name] = found

    envelope = {
        'far': FAR,
        'ceiling': CEILING,
        'chassis_reach_analytic': analytic[CHASSIS],
        'chassis_reach_swept': swept[CHASSIS],
        'wrist_reach_analytic': analytic[WRIST],
        'wrist_reach_swept': swept[WRIST],
        'chassis_height': sorted(heights[CHASSIS]),
        'wrist_heights': sorted(heights[WRIST]),
        'band': CEILING - analytic[CHASSIS],
        'height_margin_mm': (2.250 - analytic[CHASSIS]) * 1000.0,
    }
    OUT.write_text(json.dumps(envelope, indent=2), encoding='utf-8')
    for name in (CHASSIS, WRIST):
        print(f'{name:8s} analytic {analytic[name]:.4f}  swept {swept[name]:.4f}'
              f'  delta {abs(analytic[name]-swept[name])*1000:.2f} mm')
    print(f'band {envelope["band"]:.4f} m, height margin '
          f'{envelope["height_margin_mm"]:.1f} mm')
    print('wrote', OUT)


if __name__ == '__main__':
    main()
