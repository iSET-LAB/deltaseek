"""Three-panel platform figure, built from real Gazebo renders.

Panels: (a) the platform, (b) the IFC-derived environment, (c) the perceptual
action space of each sensor. Panel (c) overlays each camera's true frustum,
projected into the render camera using the same poses the evaluation uses, so
the drawing and the numbers cannot disagree.

The arm rotates the wrist camera 90 deg off the base heading in these
postures, so the two sensors cannot be aimed at a target from one base yaw.
Each sub-panel therefore uses the base yaw that aims its own sensor at the
target -- which is the fair comparison, and is itself part of what the
manipulator buys.

Requires a running simulation of the Hall B ablation world.
"""

import subprocess, time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from PIL import Image
from rclpy.node import Node
from sensor_msgs.msg import Image as ImageMsg

from deltaseek_gazebo.benchmark import apply_scenario
from deltaseek_gazebo.compare import resolve_urdf
from deltaseek_gazebo.detection import CHASSIS, WRIST, ObservationParams
from deltaseek_gazebo.kinematics import Chain
from deltaseek_gazebo.viewpoints import ARM_POSTURES, build_viewpoints, chassis_chain
from deltaseek_gazebo.visibility import OrientedBox, rotation_matrix

WORLD = 'deltaseek_hall_b_ablation_hall_b_ablation'
SHOTS = Path('/home/acharjee07/deltaseek/publication/ieeeconf/figures/shots')
BENCH = Path(__file__).resolve().parents[1] / 'config' / 'benchmarks'
CAM_HFOV, CAM_W, CAM_H = 1.05, 1600, 1100
TARGET = 'H2-cabinet-carton'


def quat(eye, target, up=(0, 0, 1)):
    eye, target = np.array(eye, float), np.array(target, float)
    f = target - eye; f /= np.linalg.norm(f)
    l = np.cross(np.array(up, float), f); l /= np.linalg.norm(l)
    R = np.column_stack([f, l, np.cross(f, l)])
    t = np.trace(R)
    if t > 0:
        s = 0.5 / np.sqrt(t + 1.0)
        q = np.array([0.25/s, (R[2,1]-R[1,2])*s, (R[0,2]-R[2,0])*s, (R[1,0]-R[0,1])*s])
    else:
        i = int(np.argmax(np.diag(R))); j, k = (i+1) % 3, (i+2) % 3
        s = 2.0*np.sqrt(1.0+R[i,i]-R[j,j]-R[k,k]); q = np.zeros(4)
        q[0] = (R[k,j]-R[j,k])/s; q[i+1] = 0.25*s
        q[j+1] = (R[j,i]+R[i,j])/s; q[k+1] = (R[k,i]+R[i,k])/s
    return q/np.linalg.norm(q), R


def gz_set_pose(name, xyz, q):
    subprocess.run(['gz', 'service', '-s', f'/world/{WORLD}/set_pose',
                    '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '5000', '--req',
                    f'name: "{name}", position: {{x: {xyz[0]}, y: {xyz[1]}, '
                    f'z: {xyz[2]}}}, orientation: {{w: {q[0]}, x: {q[1]}, '
                    f'y: {q[2]}, z: {q[3]}}}'], capture_output=True, timeout=30)


def place_robot(xy, yaw):
    subprocess.run(['ros2', 'topic', 'pub', '--once', '/a300_00000/cmd_vel',
                    'geometry_msgs/msg/TwistStamped',
                    '{twist: {linear: {x: 0.0}, angular: {z: 0.0}}}'],
                   capture_output=True, timeout=20)
    gz_set_pose('a300_00000/robot', (xy[0], xy[1], 0.16),
                (np.cos(yaw/2), 0.0, 0.0, np.sin(yaw/2)))
    time.sleep(3.5)


class Shooter(Node):
    def __init__(self):
        super().__init__('platform_figure')
        self.frame = None
        self.create_subscription(ImageMsg, '/figure_cam/image',
                                 lambda m: setattr(self, 'frame', m), 3)

    def grab(self, timeout=25.0):
        self.frame = None
        end = time.time() + timeout
        while self.frame is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        m = self.frame
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        return Image.fromarray(a[:, :, :3], 'RGB')


def project(points, eye, R):
    """World points to pixels for the render camera (+x forward, +y left)."""
    local = (np.atleast_2d(points) - np.asarray(eye)) @ R
    fx = (CAM_W / 2.0) / np.tan(CAM_HFOV / 2.0)
    forward = local[:, 0]
    u = CAM_W / 2.0 - fx * local[:, 1] / forward
    v = CAM_H / 2.0 - fx * local[:, 2] / forward
    return np.column_stack([u, v]), forward


def frustum_rays(xyz, rpy, params, length):
    """The four corner rays of a camera frustum, as world-space segments."""
    R = rotation_matrix(rpy)
    th = np.tan(params.hfov / 2.0)
    tv = th / params.aspect
    corners = [(1, s*th, t*tv) for s in (-1, 1) for t in (-1, 1)]
    origin = np.asarray(xyz, float)
    return [(origin, origin + length * (R @ np.array(c))) for c in corners]


def sensor_pose(xy, yaw, sensor, urdf):
    chain = Chain.from_urdf(urdf, 'base_link', 'camera_0_link')
    vp = build_viewpoints(chain, [(tuple(xy), yaw)],
                          postures={'raised': ARM_POSTURES['raised']},
                          chassis=chassis_chain(urdf))[0]
    return vp.poses()[sensor]


def yaw_aiming(sensor, xy, target_xy, urdf):
    """Base yaw that points the given sensor at a target.

    Solved rather than assumed: the wrist camera carries a fixed offset from
    the base heading that depends on the arm posture, and guessing it wrong
    would silently mis-aim the figure.
    """
    want = np.arctan2(target_xy[1] - xy[1], target_xy[0] - xy[0])
    _, rpy = sensor_pose(xy, 0.0, sensor, urdf)
    return float(want - rpy[2])


def main():
    urdf = resolve_urdf(None)
    manifest = yaml.safe_load((BENCH / 'hall_b_ablation_nominal.yaml').read_text())
    scenario = yaml.safe_load((BENCH / 'hall_b_ablation_deviations.yaml').read_text())
    _, actual, truth = apply_scenario(manifest, scenario)
    boxes = {e['id']: OrientedBox.from_element(e) for e in actual}
    target = boxes[TARGET].center
    params = ObservationParams(far=5.0)

    rclpy.init()
    node = Shooter()
    meta = {}

    # Context panels first, with the robot posed for them. Capturing these
    # after the frustum shots would photograph it at whatever yaw the last
    # sensor alignment happened to need.
    place_robot((6.20, 0.60), 0.75)
    for name, (eye, look) in {
        'platform': ((4.55, -1.15, 1.30), (6.25, 0.72, 0.95)),
        'environment': ((3.35, 5.90, 2.25), (8.20, -2.60, 0.95)),
    }.items():
        q, _ = quat(eye, look)
        gz_set_pose('figure_cam', eye, q)
        time.sleep(2.0)
        node.grab().save(SHOTS / f'raw_{name}.png')
        print('saved', name)

    base_xy = (5.10, 2.40)
    for sensor in (CHASSIS, WRIST):
        yaw = yaw_aiming(sensor, base_xy, target[:2], urdf)
        place_robot(base_xy, yaw)
        xyz, rpy = sensor_pose(base_xy, yaw, sensor, urdf)
        eye = (base_xy[0] + 3.30, base_xy[1] - 2.60, 1.95)
        look = ((base_xy[0] + target[0]) / 2.0, (base_xy[1] + target[1]) / 2.0, 1.30)
        q, R = quat(eye, look)
        gz_set_pose('figure_cam', eye, q)
        time.sleep(2.0)
        node.grab().save(SHOTS / f'raw_frustum_{sensor}.png')
        # Draw the frustum out to the target's own depth, so its far face
        # lands in the target's plane and whether the change falls inside is
        # readable directly off the figure rather than inferred.
        forward = rotation_matrix(rpy)[:, 0]
        depth = float(np.dot(np.asarray(target) - np.asarray(xyz), forward))
        rays = frustum_rays(xyz, rpy, params, max(0.6, min(depth, params.far)))
        meta[sensor] = {
            'eye': eye, 'R': R.tolist(), 'camera': list(map(float, xyz)),
            'rays': [[list(map(float, a)), list(map(float, b))] for a, b in rays],
            'target': list(map(float, target)),
            'target_extent': list(map(float, boxes[TARGET].half_extents)),
            'base_yaw': yaw,
        }
        print(f'{sensor}: base yaw {yaw:+.3f}, camera at {np.round(xyz, 3)}')

    (SHOTS / 'frustum_meta.yaml').write_text(yaml.safe_dump(meta), encoding='utf-8')
    node.destroy_node(); rclpy.shutdown()
    print('wrote frustum_meta.yaml')


if __name__ == '__main__':
    main()
