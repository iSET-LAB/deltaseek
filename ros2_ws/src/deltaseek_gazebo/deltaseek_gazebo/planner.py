"""Deviation-seeking viewpoint planning and the baselines it is measured against.

Every planner here sees only the *nominal* model, which is all a real robot has
before it inspects anything.  Ground truth is used exclusively afterwards, by
the evaluation harness, to score what a plan actually found.  Keeping that
boundary strict is what stops the planner from quietly cheating.

The deviation-seeking planner spends a traversal budget to maximize expected
information about the deviation state of modelled elements.  Each element
carries an independent Bernoulli belief; a viewpoint's value is the expected
entropy it removes, and viewpoints are chosen by benefit per unit distance,
which is the standard greedy rule for a budgeted orienteering problem.

Travel cost is Euclidean by default.  ``cost_fn`` is pluggable so a Nav2 path
length can be substituted once a planner server is running, without changing
the selection logic.
"""

from dataclasses import dataclass
import math

import numpy as np

from deltaseek_gazebo.detection import ObservationParams, Scene
from deltaseek_gazebo.visibility import visible_fraction


def euclidean(a, b):
    """Return straight-line distance between two base positions."""
    return float(math.hypot(b[0] - a[0], b[1] - a[1]))


def visibility_matrix(viewpoints, elements, params=None):
    """Return the visible surface fraction of each element from each viewpoint.

    Computed once against the nominal model and reused by every planner, so
    planners are compared on identical information.
    """
    params = params or ObservationParams()
    scene = Scene.from_elements(elements)
    keys = list(scene.boxes)
    matrix = np.zeros((len(viewpoints), len(keys)))
    for row, viewpoint in enumerate(viewpoints):
        camera = params.camera(viewpoint.camera_xyz, viewpoint.camera_rpy)
        for column, key in enumerate(keys):
            matrix[row, column] = visible_fraction(
                scene.boxes[key], camera, scene.occluders(exclude=key),
                samples_per_face=params.samples_per_face,
                max_incidence=params.max_incidence)
    return keys, matrix


@dataclass
class BeliefParams:
    """How an observation converts into resolved uncertainty."""

    prior: float = 0.15
    min_fraction: float = 0.05
    full_fraction: float = 0.35

    def resolution(self, fractions):
        """Return how completely each observation settles an element's state.

        Below ``min_fraction`` a glimpse settles nothing; by ``full_fraction``
        the element is considered fully inspected.  The ramp between them keeps
        the planner preferring viewpoints that see elements properly over ones
        that merely clip them.
        """
        span = max(self.full_fraction - self.min_fraction, 1.0e-9)
        return np.clip((np.asarray(fractions) - self.min_fraction) / span,
                       0.0, 1.0)


def _entropy(probability):
    p = min(max(float(probability), 1.0e-9), 1.0 - 1.0e-9)
    return -(p * math.log2(p) + (1.0 - p) * math.log2(1.0 - p))


class Belief:
    """Independent per-element uncertainty about deviation state."""

    def __init__(self, element_count, params=None):
        self.params = params or BeliefParams()
        self.entropy = np.full(element_count, _entropy(self.params.prior))
        self.residual = np.ones(element_count)

    def gain(self, fractions):
        """Return the expected entropy this observation would remove."""
        return float(np.sum(
            self.entropy * self.residual * self.params.resolution(fractions)))

    def update(self, fractions):
        """Consume an observation, leaving the uncertainty it did not resolve."""
        self.residual = self.residual * (1.0 - self.params.resolution(fractions))

    @property
    def resolved_fraction(self):
        return float(1.0 - self.residual.mean())


def plan_deviation_seeking(viewpoints, matrix, budget, start=(0.0, 0.0),
                           params=None, cost_fn=euclidean, min_gain=1.0e-3):
    """Select viewpoints maximizing expected information per unit distance."""
    belief = Belief(matrix.shape[1], params)
    remaining = set(range(len(viewpoints)))
    position = tuple(start)
    spent = 0.0
    order = []

    while remaining:
        best = None
        for index in remaining:
            gain = belief.gain(matrix[index])
            if gain < min_gain:
                continue
            step = cost_fn(position, viewpoints[index].base_xy)
            if spent + step > budget:
                continue
            # Benefit per unit distance, with a floor so that a valuable
            # viewpoint at the current position cannot divide by zero.
            score = gain / max(step, 0.25)
            if best is None or score > best[0]:
                best = (score, index, step, gain)
        if best is None:
            break
        _, index, step, _ = best
        order.append(index)
        remaining.discard(index)
        belief.update(matrix[index])
        spent += step
        position = viewpoints[index].base_xy

    return order, {'distance': round(spent, 6),
                   'resolved_fraction': round(belief.resolved_fraction, 6)}


def plan_coverage(viewpoints, budget, start=(0.0, 0.0), cost_fn=euclidean):
    """Baseline: visit every base pose in a nearest-neighbour sweep.

    Coverage inspection ignores the verification objective entirely; it only
    tries to stand everywhere, which is the behaviour the project argues is
    wasteful under a distance budget.
    """
    by_base = {}
    for index, viewpoint in enumerate(viewpoints):
        by_base.setdefault(viewpoint.base_xy, []).append(index)

    remaining = set(by_base)
    position = tuple(start)
    spent = 0.0
    order = []
    while remaining:
        nearest = min(remaining, key=lambda xy: cost_fn(position, xy))
        step = cost_fn(position, nearest)
        if spent + step > budget:
            break
        spent += step
        position = nearest
        remaining.discard(nearest)
        order.extend(by_base[nearest])
    return order, {'distance': round(spent, 6)}


def plan_goal_directed(viewpoints, budget, start=(0.0, 0.0), goal=None,
                       cost_fn=euclidean):
    """Baseline: drive toward a goal, observing only from poses along the way.

    This is the navigation-first behaviour the abstract contrasts with: the
    verification objective never influences where the robot goes.
    """
    if goal is None:
        goal = max(
            (viewpoint.base_xy for viewpoint in viewpoints),
            key=lambda xy: cost_fn(start, xy))

    bases = sorted(
        {viewpoint.base_xy for viewpoint in viewpoints},
        key=lambda xy: cost_fn(start, xy))
    corridor = [
        xy for xy in bases
        # Keep poses that do not meaningfully detour from the straight run.
        if cost_fn(start, xy) + cost_fn(xy, goal) <= cost_fn(start, goal) * 1.15
    ]
    corridor.sort(key=lambda xy: cost_fn(start, xy))

    by_base = {}
    for index, viewpoint in enumerate(viewpoints):
        by_base.setdefault(viewpoint.base_xy, []).append(index)

    position = tuple(start)
    spent = 0.0
    order = []
    for xy in corridor:
        step = cost_fn(position, xy)
        if spent + step > budget:
            break
        spent += step
        position = xy
        order.extend(by_base.get(xy, []))
    return order, {'distance': round(spent, 6), 'goal': list(goal)}


def plan_frontier(viewpoints, budget, start=(0.0, 0.0), sensor_range=4.0,
                  cell=1.0, cost_fn=euclidean):
    """Baseline: greedily drive to the nearest pose that reveals unseen space.

    Frontier exploration maximizes newly observed *area*, not information about
    modelled elements, so it keeps moving into open space long after the
    modelled elements there have been resolved.
    """
    bases = sorted({viewpoint.base_xy for viewpoint in viewpoints})
    grid = {
        (round(xy[0] / cell), round(xy[1] / cell)): xy for xy in bases
    }
    seen = set()

    def revealed(xy):
        return {
            key for key, other in grid.items()
            if key not in seen and cost_fn(xy, other) <= sensor_range
        }

    by_base = {}
    for index, viewpoint in enumerate(viewpoints):
        by_base.setdefault(viewpoint.base_xy, []).append(index)

    remaining = set(bases)
    position = tuple(start)
    spent = 0.0
    order = []
    while remaining:
        best = None
        for xy in remaining:
            new = len(revealed(xy))
            if not new:
                continue
            step = cost_fn(position, xy)
            if spent + step > budget:
                continue
            score = new / max(step, 0.25)
            if best is None or score > best[0]:
                best = (score, xy, step)
        if best is None:
            break
        _, xy, step = best
        seen |= revealed(xy)
        spent += step
        position = xy
        remaining.discard(xy)
        order.extend(by_base[xy])
    return order, {'distance': round(spent, 6)}


PLANNERS = {
    'deviation_seeking': plan_deviation_seeking,
    'coverage': plan_coverage,
    'frontier': plan_frontier,
    'goal_directed': plan_goal_directed,
}
