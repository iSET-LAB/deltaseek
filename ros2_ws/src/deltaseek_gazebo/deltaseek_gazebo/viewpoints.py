"""Generate candidate viewpoints over base poses and arm configurations.

The project's claim is that including the manipulator in viewpoint selection
reaches vantage points a fixed-sensor platform cannot.  Testing that claim
requires both candidate sets to be produced the same way and scored by the same
model, so this module builds them from one generator: a *fixed* sensor is just
the arm frozen at a single posture.

Base poses are laid on a grid of navigable free space rather than sampled
randomly, so that two planners compared on the same scene see exactly the same
candidate set and differ only in which candidates they choose.
"""

from dataclasses import dataclass, field
import itertools
import math

import numpy as np

from deltaseek_gazebo.detection import CHASSIS, WRIST
from deltaseek_gazebo.kinematics import Chain, base_pose, pose_to_xyz_rpy
from deltaseek_gazebo.visibility import OrientedBox


# Half-extents of the A300 footprint declared in the platform's Nav2
# configuration. Base-pose feasibility and the traversal cost grid must inflate
# obstacles by the same amount, or the planner selects viewpoints it cannot
# drive to.
FOOTPRINT_HALF_EXTENTS = (0.495, 0.349)
ROBOT_RADIUS = math.hypot(*FOOTPRINT_HALF_EXTENTS)

# Clearpath indexes cameras positionally from robot.yaml, so the eye-in-hand
# sensor is camera_0 and the chassis sensor camera_1.  Both mount transforms
# are read out of the generated description by forward kinematics rather than
# written down here: a guessed mount height would decide the outcome of the
# sensor ablation by itself.
WRIST_CAMERA_LINK = 'camera_0_link'
CHASSIS_CAMERA_LINK = 'camera_1_link'


@dataclass
class Viewpoint:
    """A base pose plus arm configuration, and the camera pose it produces."""

    base_xy: tuple
    base_yaw: float
    joint_values: dict = field(default_factory=dict)
    camera_xyz: tuple = (0.0, 0.0, 0.0)
    camera_rpy: tuple = (0.0, 0.0, 0.0)
    sensor_poses: dict = field(default_factory=dict)

    @property
    def base_xyz(self):
        return [float(self.base_xy[0]), float(self.base_xy[1]), 0.0]

    def poses(self, sensors=None):
        """Return ``{sensor_name: (xyz, rpy)}`` for the active sensors."""
        available = self.sensor_poses or {
            WRIST: (self.camera_xyz, self.camera_rpy)}
        if sensors is None:
            return dict(available)
        return {name: pose for name, pose in available.items()
                if name in sensors}


# Representative inspection postures.  These are chosen for what they let the
# camera see rather than to span the joint space: a neutral forward look, a
# raised look for high services, a lowered look under assemblies, and two
# side-reaches that see around an obstruction without moving the base.
ARM_POSTURES = {
    'forward': [0.0, -0.6, 0.9, -0.3, 0.0, 0.0],
    'raised': [0.0, -1.1, 0.7, 0.4, 0.0, 0.0],
    'lowered': [0.0, -0.2, 1.0, -0.8, 0.0, 0.0],
    'left_reach': [1.2, -0.7, 0.9, -0.2, 0.0, 0.0],
    'right_reach': [-1.2, -0.7, 0.9, -0.2, 0.0, 0.0],
}
FIXED_POSTURE = 'forward'


def footprints(elements, inflation=ROBOT_RADIUS, drivable_height=0.2,
               platform_height=1.0):
    """Return inflated 2D bounds for elements that block the base.

    Two classes of element are not obstacles. Slabs and floors, whose top
    surface is at or below ``drivable_height``, are what the robot drives on;
    treating a storey slab as an obstacle would block the entire scene.
    Overhead services, whose underside clears ``platform_height``, are driven
    beneath.
    """
    blocked = []
    for element in elements:
        box = OrientedBox.from_element(element)
        top = box.center[2] + box.half_extents[2]
        bottom = box.center[2] - box.half_extents[2]
        if top <= drivable_height or bottom >= platform_height:
            continue
        # A circumscribing circle is unusable here: a long thin wall would
        # sweep a radius that swallows the whole storey. Keep the rectangle.
        blocked.append((
            box.center[:2].copy(),
            box.rotation[:2, :2].copy(),
            box.half_extents[:2] + inflation,
        ))
    return blocked


def is_blocked(point, blocked):
    """Return whether a base position lies inside any inflated footprint."""
    position = np.asarray(point, dtype=float)
    for center, rotation, extents in blocked:
        local = rotation.T @ (position - center)
        if abs(local[0]) <= extents[0] and abs(local[1]) <= extents[1]:
            return True
    return False


def free_base_poses(elements, bounds, spacing=1.5, yaws=(0.0,),
                    inflation=ROBOT_RADIUS):
    """Return grid base poses that clear every blocking element footprint."""
    blocked = footprints(elements, inflation)
    (x_min, x_max), (y_min, y_max) = bounds
    xs = np.arange(x_min, x_max + 1.0e-9, spacing)
    ys = np.arange(y_min, y_max + 1.0e-9, spacing)
    poses = []
    for x, y in itertools.product(xs, ys):
        if is_blocked((x, y), blocked):
            continue
        for yaw in yaws:
            poses.append(((float(x), float(y)), float(yaw)))
    return poses


def scene_bounds(elements, margin=1.0):
    """Return xy bounds covering every element, padded by a margin."""
    centers = np.array([element['pose']['xyz'][:2] for element in elements])
    return (
        (float(centers[:, 0].min() - margin), float(centers[:, 0].max() + margin)),
        (float(centers[:, 1].min() - margin), float(centers[:, 1].max() + margin)),
    )


def chassis_chain(urdf_path, link=CHASSIS_CAMERA_LINK):
    """Return the base_link -> chassis camera chain from the description."""
    return Chain.from_urdf(urdf_path, 'base_link', link)


def build_viewpoints(chain, base_poses, postures=None, min_camera_height=0.25,
                     chassis=None):
    """Compose base poses with arm postures into camera viewpoints.

    ``chassis`` is an optional base_link -> chassis camera chain.  Its pose is
    computed once, outside the posture loop, because a chassis sensor does not
    move with the arm; that invariance is the whole point of the comparison.
    """
    postures = ARM_POSTURES if postures is None else postures
    chassis_local = None if chassis is None else chassis.forward()
    viewpoints = []
    cache = {}
    for name, values in postures.items():
        joints = dict(zip(chain.joint_names, values))
        if not chain.within_limits(joints):
            continue
        cache[name] = (joints, chain.forward(joints))

    for (xy, yaw) in base_poses:
        world_base = base_pose(xy, yaw)
        for name, (joints, arm_pose) in cache.items():
            camera = world_base @ arm_pose
            xyz, rpy = pose_to_xyz_rpy(camera)
            if xyz[2] < min_camera_height:
                continue
            poses = {WRIST: (tuple(xyz), tuple(rpy))}
            if chassis_local is not None:
                c_xyz, c_rpy = pose_to_xyz_rpy(world_base @ chassis_local)
                poses[CHASSIS] = (tuple(c_xyz), tuple(c_rpy))
            viewpoints.append(Viewpoint(
                base_xy=xy, base_yaw=yaw, joint_values=joints,
                camera_xyz=tuple(xyz), camera_rpy=tuple(rpy),
                sensor_poses=poses))
    return viewpoints


def fixed_sensor_viewpoints(chain, base_poses, posture=FIXED_POSTURE,
                            chassis=None):
    """Return the comparison set: one frozen arm posture, base motion only.

    Note this freezes the *arm*, leaving the camera on a parked tool flange.
    It is not the same comparison as running with only the chassis sensor
    active, which is what a real platform without a manipulator would have.
    """
    return build_viewpoints(
        chain, base_poses, postures={posture: ARM_POSTURES[posture]},
        chassis=chassis)


def to_trajectory(viewpoints, trajectory_id='run'):
    """Serialize an ordered viewpoint list into the evaluation schema."""
    return {
        'schema_version': 1,
        'trajectory': {'id': trajectory_id, 'frame_id': 'world'},
        'viewpoints': [
            {
                'base': {
                    'xyz': [round(v, 6) for v in viewpoint.base_xyz],
                    'yaw': round(float(viewpoint.base_yaw), 6),
                },
                'camera': {
                    'xyz': [round(float(v), 6) for v in viewpoint.camera_xyz],
                    'rpy': [round(float(v), 6) for v in viewpoint.camera_rpy],
                },
                'posture': [
                    round(float(value), 6)
                    for value in viewpoint.joint_values.values()
                ],
                'sensors': {
                    name: {
                        'xyz': [round(float(v), 6) for v in xyz],
                        'rpy': [round(float(v), 6) for v in rpy],
                    }
                    for name, (xyz, rpy) in viewpoint.poses().items()
                },
            }
            for viewpoint in viewpoints
        ],
    }
