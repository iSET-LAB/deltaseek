"""Forward kinematics for the arm-mounted camera, read from the robot URDF.

Viewpoint planning has to know where the camera ends up for a given arm
configuration.  Deriving that from the generated URDF rather than from
hard-coded UR5e parameters keeps one source of truth: the arm mount, the
enclosure offset, and the camera transform all come from ``robot.yaml`` through
the Clearpath generator, so a change there cannot silently invalidate the
planner's geometry.

This is deliberately a small, dependency-free chain evaluator rather than a
KDL or MoveIt binding.  The planner scores thousands of candidate
configurations per viewpoint, so FK sits in the inner loop and must stay cheap
and importable without a ROS runtime.
"""

import xml.etree.ElementTree as ET

import numpy as np

from deltaseek_gazebo.visibility import rotation_matrix


REVOLUTE = ('revolute', 'continuous')
MOVING = REVOLUTE + ('prismatic',)


def transform(xyz, rpy):
    """Return the 4x4 homogeneous transform for a URDF origin."""
    matrix = np.eye(4)
    matrix[:3, :3] = rotation_matrix(rpy)
    matrix[:3, 3] = np.asarray(xyz, dtype=float)
    return matrix


def _origin_of(joint):
    origin = joint.find('origin')
    if origin is None:
        return transform([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    xyz = [float(v) for v in (origin.get('xyz') or '0 0 0').split()]
    rpy = [float(v) for v in (origin.get('rpy') or '0 0 0').split()]
    return transform(xyz, rpy)


def _axis_of(joint):
    axis = joint.find('axis')
    values = [float(v) for v in (axis.get('xyz') if axis is not None else '1 0 0').split()]
    vector = np.asarray(values, dtype=float)
    norm = np.linalg.norm(vector)
    return vector / norm if norm > 0.0 else np.array([1.0, 0.0, 0.0])


def _limits_of(joint):
    limit = joint.find('limit')
    if joint.get('type') == 'continuous' or limit is None:
        return (-np.pi, np.pi)
    return (float(limit.get('lower', -np.pi)), float(limit.get('upper', np.pi)))


def _axis_transform(axis, kind, value):
    matrix = np.eye(4)
    if kind in REVOLUTE:
        # Rodrigues' rotation about the joint axis.
        skew = np.array([
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ])
        matrix[:3, :3] = (
            np.eye(3)
            + np.sin(value) * skew
            + (1.0 - np.cos(value)) * (skew @ skew)
        )
    else:
        matrix[:3, 3] = axis * value
    return matrix


class Chain:
    """An ordered kinematic chain between two links of a URDF."""

    def __init__(self, joints):
        self.joints = joints
        self.joint_names = [j['name'] for j in joints if j['type'] in MOVING]
        self.limits = [j['limits'] for j in joints if j['type'] in MOVING]

    @classmethod
    def from_urdf(cls, path, base_link, tip_link):
        """Build the chain connecting ``base_link`` to ``tip_link``."""
        root = ET.parse(str(path)).getroot()
        by_child = {}
        for joint in root.findall('joint'):
            child = joint.find('child').get('link')
            by_child[child] = joint

        ordered = []
        link = tip_link
        while link != base_link:
            joint = by_child.get(link)
            if joint is None:
                raise ValueError(
                    f'no path from {base_link!r} to {tip_link!r}: '
                    f'{link!r} has no parent joint')
            ordered.append({
                'name': joint.get('name'),
                'type': joint.get('type'),
                'origin': _origin_of(joint),
                'axis': _axis_of(joint),
                'limits': _limits_of(joint),
            })
            link = joint.find('parent').get('link')
        ordered.reverse()
        return cls(ordered)

    def forward(self, values=None):
        """Return the tip pose relative to the base for these joint values."""
        values = {} if values is None else values
        if not isinstance(values, dict):
            values = dict(zip(self.joint_names, np.asarray(values, float)))

        pose = np.eye(4)
        for joint in self.joints:
            pose = pose @ joint['origin']
            if joint['type'] in MOVING:
                pose = pose @ _axis_transform(
                    joint['axis'], joint['type'],
                    float(values.get(joint['name'], 0.0)))
        return pose

    def within_limits(self, values):
        """Return whether every joint value lies inside its URDF limit."""
        if isinstance(values, dict):
            values = [values.get(name, 0.0) for name in self.joint_names]
        return all(
            low <= value <= high
            for value, (low, high) in zip(values, self.limits)
        )


def pose_to_xyz_rpy(pose):
    """Split a 4x4 transform into translation and roll-pitch-yaw angles."""
    matrix = pose[:3, :3]
    pitch = float(np.arctan2(
        -matrix[2, 0], np.hypot(matrix[0, 0], matrix[1, 0])))
    if abs(np.cos(pitch)) < 1.0e-9:
        # Gimbal lock: fold the degenerate roll into yaw.
        roll = 0.0
        yaw = float(np.arctan2(-matrix[0, 1], matrix[1, 1]))
    else:
        roll = float(np.arctan2(matrix[2, 1], matrix[2, 2]))
        yaw = float(np.arctan2(matrix[1, 0], matrix[0, 0]))
    return list(pose[:3, 3]), [roll, pitch, yaw]


def base_pose(xy, yaw, z=0.0):
    """Return the world transform of a differential-drive base pose."""
    return transform([float(xy[0]), float(xy[1]), float(z)],
                     [0.0, 0.0, float(yaw)])
