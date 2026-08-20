"""Geometric visibility model for BIM element observation.

Viewpoint planning has to score thousands of candidate viewpoints, which rules
out rendering each one.  This module answers the cheaper question the planner
actually needs: *what fraction of an element's surface would this camera see?*

Elements are treated as oriented boxes, which is the representation the IFC
adapter already produces.  An element sample is counted as seen when it lies in
the view frustum, faces the camera closely enough to give a usable depth return,
and no other element blocks the line of sight.

The camera frame follows the ROS body convention used by the Clearpath sensor
description's ``<name>_link``: +x forward along the view axis, +y left, +z up.
This is *not* the optical frame, which is z-forward.

Scaling: occlusion is broad-phase filtered by bounding sphere, but the model is
still quadratic in element count.  It is sized for the hundreds of elements in a
storey, not for an entire building at once.
"""

import math

import numpy as np


def rotation_matrix(rpy):
    """Return the rotation matrix for roll-pitch-yaw angles, as ``Rz @ Ry @ Rx``."""
    roll, pitch, yaw = (float(value) for value in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


class OrientedBox:
    """An element approximated by an oriented bounding box."""

    def __init__(self, center, rpy, size):
        self.center = np.asarray(center, dtype=float)
        self.rotation = rotation_matrix(rpy)
        self.half_extents = np.abs(np.asarray(size, dtype=float)) / 2.0
        self.radius = float(np.linalg.norm(self.half_extents))

    @classmethod
    def from_element(cls, element):
        """Build a box from a normalized manifest element.

        Non-box primitives are replaced by their bounding box, which keeps the
        visibility model uniform.  Meshes fall back to their scale, so a mesh
        manifest needs real extents before its numbers mean anything.
        """
        geometry = element['geometry']
        kind = geometry['type']
        if kind == 'box':
            size = geometry['size']
        elif kind == 'cylinder':
            diameter = 2.0 * float(geometry['radius'])
            size = [diameter, diameter, float(geometry['length'])]
        elif kind == 'sphere':
            diameter = 2.0 * float(geometry['radius'])
            size = [diameter, diameter, diameter]
        else:
            size = geometry.get('scale', [1.0, 1.0, 1.0])
        pose = element['pose']
        return cls(pose['xyz'], pose['rpy'], size)

    def surface_samples(self, per_face=3):
        """Return a deterministic grid of surface points and outward normals."""
        steps = (np.arange(per_face) + 0.5) / per_face * 2.0 - 1.0
        local_points = []
        local_normals = []
        for axis in range(3):
            first, second = [index for index in range(3) if index != axis]
            for sign in (-1.0, 1.0):
                normal = np.zeros(3)
                normal[axis] = sign
                for a in steps:
                    for b in steps:
                        point = np.zeros(3)
                        point[axis] = sign
                        point[first] = a
                        point[second] = b
                        local_points.append(point * self.half_extents)
                        local_normals.append(normal)
        points = np.asarray(local_points) @ self.rotation.T + self.center
        normals = np.asarray(local_normals) @ self.rotation.T
        return points, normals

    def blocks(self, origin, points, epsilon=1.0e-4):
        """Return which origin-to-point segments this box intersects."""
        directions = points - origin
        lengths = np.linalg.norm(directions, axis=1)
        safe = np.where(lengths > 0.0, lengths, 1.0)
        directions = directions / safe[:, None]

        local_origin = (origin - self.center) @ self.rotation
        local_directions = directions @ self.rotation
        local_directions = np.where(
            np.abs(local_directions) < 1.0e-12, 1.0e-12, local_directions)

        inverse = 1.0 / local_directions
        near = (-self.half_extents - local_origin) * inverse
        far = (self.half_extents - local_origin) * inverse
        t_min = np.max(np.minimum(near, far), axis=1)
        t_max = np.min(np.maximum(near, far), axis=1)
        return (t_max >= t_min) & (t_max > epsilon) & (t_min < lengths - epsilon)

    def may_block(self, origin, target):
        """Cheap bounding-sphere reject for the origin-to-target corridor."""
        axis = target.center - origin
        length = float(np.linalg.norm(axis))
        reach = self.radius + target.radius
        offset = self.center - origin
        if length < 1.0e-9:
            return float(np.linalg.norm(offset)) <= reach
        unit = axis / length
        along = float(np.dot(offset, unit))
        if along < -self.radius or along > length + self.radius:
            return False
        perpendicular = float(np.linalg.norm(offset - along * unit))
        return perpendicular <= reach


class Camera:
    """A pinhole view frustum with a usable depth range."""

    def __init__(self, position, rotation, hfov=1.25, aspect=4.0 / 3.0,
                 near=0.3, far=6.0):
        self.position = np.asarray(position, dtype=float)
        self.rotation = np.asarray(rotation, dtype=float)
        self.hfov = float(hfov)
        self.near = float(near)
        self.far = float(far)
        self._tan_h = math.tan(self.hfov / 2.0)
        self._tan_v = self._tan_h / float(aspect)

    @classmethod
    def from_pose(cls, xyz, rpy, **kwargs):
        return cls(xyz, rotation_matrix(rpy), **kwargs)

    def contains(self, points):
        """Return which world points fall inside the frustum."""
        local = (np.atleast_2d(points) - self.position) @ self.rotation
        forward = local[:, 0]
        return (
            (forward >= self.near)
            & (forward <= self.far)
            & (np.abs(local[:, 1]) <= forward * self._tan_h)
            & (np.abs(local[:, 2]) <= forward * self._tan_v)
        )


def visible_fraction(box, camera, occluders, samples_per_face=3,
                     max_incidence=math.radians(75.0)):
    """Return the fraction of a box's surface this camera observes.

    ``occluders`` must not contain ``box`` itself; a box always hides its own
    far faces, and self-occlusion is handled by the incidence test instead.
    """
    points, normals = box.surface_samples(samples_per_face)
    to_camera = camera.position - points
    distances = np.linalg.norm(to_camera, axis=1)
    safe = np.where(distances > 0.0, distances, 1.0)
    directions = to_camera / safe[:, None]

    facing = np.sum(normals * directions, axis=1) >= math.cos(max_incidence)
    candidate = facing & camera.contains(points)
    if not candidate.any():
        return 0.0

    visible = candidate.copy()
    subset = points[candidate]
    blocked = np.zeros(len(subset), dtype=bool)
    for occluder in occluders:
        if blocked.all():
            break
        if not occluder.may_block(camera.position, box):
            continue
        blocked |= occluder.blocks(camera.position, subset)
    visible[candidate] = ~blocked
    return float(visible.sum()) / float(len(points))
