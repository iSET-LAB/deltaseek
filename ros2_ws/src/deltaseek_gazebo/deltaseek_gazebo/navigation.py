"""Traversal cost that has to go around walls.

Straight-line distance is fine in an open room and wrong in a building: it
lets a planner reach a room without paying for the doorway. Since detections
per unit distance is the quantity the project claims to improve, the
denominator has to be a distance the robot could actually drive.

Nav2's global planner for this platform is NavfnPlanner, which is Dijkstra
over an inflated costmap. Rather than call its action server, which would cost
a round trip for each of the tens of thousands of pairwise queries a planner
makes, this module computes the same quantity directly, using the same
parameters as the platform's Nav2 configuration:

    costmap resolution   0.06 m
    footprint            0.495 x 0.349 m half-extents
    circumscribed radius 0.606 m

Obstacles are inflated by the circumscribed radius and treated as lethal.
That is the conservative reading of an inflation layer: Nav2 would let a path
graze an inflated cell at higher cost, so these lengths are an upper bound on
what Nav2 would return, never an optimistic one.
"""

import heapq
import math

import numpy as np

from deltaseek_gazebo.viewpoints import ROBOT_RADIUS, footprints
from deltaseek_gazebo.visibility import OrientedBox


COSTMAP_RESOLUTION = 0.06

_NEIGHBOURS = [
    (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
    (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)),
]


class CostGrid:
    """An inflated occupancy grid built from the nominal element manifest."""

    def __init__(self, elements, resolution=COSTMAP_RESOLUTION,
                 robot_radius=ROBOT_RADIUS, margin=1.5):
        self.resolution = float(resolution)
        blocked = footprints(elements, inflation=robot_radius)

        # Bound the grid by element extents, not centres: a single wide slab
        # has one centre but covers the whole storey, and sizing from centres
        # would clip the drivable area down to nothing.
        corners = []
        for element in elements:
            box = OrientedBox.from_element(element)
            reach = np.abs(box.rotation[:2, :2]) @ box.half_extents[:2]
            corners.append(box.center[:2] - reach)
            corners.append(box.center[:2] + reach)
        points = np.asarray(corners, dtype=float)
        self.origin = points.min(axis=0) - margin
        extent = points.max(axis=0) + margin - self.origin
        self.shape = (
            int(math.ceil(extent[1] / self.resolution)) + 1,
            int(math.ceil(extent[0] / self.resolution)) + 1,
        )
        self.occupied = np.zeros(self.shape, dtype=bool)
        for center, rotation, extents in blocked:
            self._mark(center, rotation, extents)

    def _mark(self, center, rotation, extents):
        """Rasterize one inflated rectangle, testing only its bounding box."""
        # AABB half-extent of the rotated rectangle, per axis.
        reach = np.abs(rotation) @ extents
        low = self.cell_of(center - reach)
        high = self.cell_of(center + reach)
        rows = slice(max(low[0], 0), min(high[0] + 1, self.shape[0]))
        cols = slice(max(low[1], 0), min(high[1] + 1, self.shape[1]))
        if rows.start >= rows.stop or cols.start >= cols.stop:
            return

        ys = self.origin[1] + np.arange(rows.start, rows.stop) * self.resolution
        xs = self.origin[0] + np.arange(cols.start, cols.stop) * self.resolution
        grid_x, grid_y = np.meshgrid(xs, ys)
        offset = np.stack(
            [grid_x - center[0], grid_y - center[1]], axis=-1)
        local = offset @ rotation
        inside = (
            (np.abs(local[..., 0]) <= extents[0])
            & (np.abs(local[..., 1]) <= extents[1])
        )
        self.occupied[rows, cols] |= inside

    def cell_of(self, xy):
        """Return the (row, col) containing a world position."""
        col = int(round((float(xy[0]) - self.origin[0]) / self.resolution))
        row = int(round((float(xy[1]) - self.origin[1]) / self.resolution))
        return row, col

    def center_of(self, cell):
        """Return the world position of a cell centre."""
        row, col = cell
        return (self.origin[0] + col * self.resolution,
                self.origin[1] + row * self.resolution)

    def in_bounds(self, cell):
        row, col = cell
        return 0 <= row < self.shape[0] and 0 <= col < self.shape[1]

    def is_free(self, xy):
        cell = self.cell_of(xy)
        return self.in_bounds(cell) and not self.occupied[cell]

    def nearest_free(self, xy, search_radius=1.0):
        """Snap a position to the closest drivable cell.

        Base poses are sampled on a coarse grid and can land just inside an
        inflated obstacle. Refusing them outright would silently delete
        viewpoints, so snap instead and let the caller pay the real distance.
        """
        cell = self.cell_of(xy)
        if self.in_bounds(cell) and not self.occupied[cell]:
            return cell
        steps = int(math.ceil(search_radius / self.resolution))
        for radius in range(1, steps + 1):
            best = None
            for row in range(cell[0] - radius, cell[0] + radius + 1):
                for col in range(cell[1] - radius, cell[1] + radius + 1):
                    if max(abs(row - cell[0]), abs(col - cell[1])) != radius:
                        continue
                    if not self.in_bounds((row, col)) or self.occupied[row, col]:
                        continue
                    distance = math.hypot(row - cell[0], col - cell[1])
                    if best is None or distance < best[0]:
                        best = (distance, (row, col))
            if best is not None:
                return best[1]
        return None


class PathCost:
    """Geodesic traversal cost between base positions, memoized per source."""

    def __init__(self, grid):
        self.grid = grid
        self._fields = {}

    def _field(self, source):
        """Return the distance field from one source cell, in metres."""
        cached = self._fields.get(source)
        if cached is not None:
            return cached

        occupied = self.grid.occupied
        rows, cols = self.grid.shape
        distance = np.full(self.grid.shape, np.inf)
        distance[source] = 0.0
        queue = [(0.0, source)]
        resolution = self.grid.resolution

        while queue:
            cost, (row, col) = heapq.heappop(queue)
            if cost > distance[row, col]:
                continue
            for d_row, d_col, step in _NEIGHBOURS:
                next_row, next_col = row + d_row, col + d_col
                if not (0 <= next_row < rows and 0 <= next_col < cols):
                    continue
                if occupied[next_row, next_col]:
                    continue
                candidate = cost + step * resolution
                if candidate < distance[next_row, next_col]:
                    distance[next_row, next_col] = candidate
                    heapq.heappush(queue, (candidate, (next_row, next_col)))

        self._fields[source] = distance
        return distance

    def __call__(self, a, b):
        """Return drivable distance in metres, or infinity if unreachable.

        A query point can fall inside an inflated obstacle, in which case it is
        snapped to the nearest drivable cell. The snap distance is charged, so
        the result can never come out below the straight-line distance and
        report a shortcut that does not exist.
        """
        start = tuple(float(v) for v in a[:2])
        goal = tuple(float(v) for v in b[:2])
        source = self.grid.nearest_free(start)
        target = self.grid.nearest_free(goal)
        if source is None or target is None:
            return math.inf
        if source == target:
            # Same cell: the grid has nothing to say, and charging the
            # rounding to cell centres would bill a stationary robot. This
            # matters because a plan revisits one base pose from many arm
            # postures, so any per-step offset accumulates into the budget.
            return math.dist(start, goal)
        along = float(self._field(source)[target])
        if not math.isfinite(along):
            return math.inf
        # Only a query point that had to be snapped out of an obstacle pays an
        # approach; for a drivable point the offset is discretization noise.
        approach = 0.0
        if not self.grid.is_free(start):
            approach += math.dist(start, self.grid.center_of(source))
        if not self.grid.is_free(goal):
            approach += math.dist(goal, self.grid.center_of(target))
        return along + approach


def path_cost_for(elements, **kwargs):
    """Build a ready-to-use cost function for a set of nominal elements."""
    return PathCost(CostGrid(elements, **kwargs))
