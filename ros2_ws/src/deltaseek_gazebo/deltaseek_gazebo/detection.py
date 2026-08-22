"""Turn camera viewpoints into deviation detections against exact ground truth.

Each deviation type becomes visible through a different observation:

``missing``
    Nothing occupies the modelled volume, so the deviation is revealed by
    seeing *into* the nominal element's region and finding it empty.
``unexpected``, ``rotated``, ``resized``
    The discrepancy sits where the as-built element is, so observing that
    element is what reveals it.
``displaced``
    Either observation suffices: seeing the element in the wrong place, or
    seeing that the modelled place is empty.

A station may carry more than one sensor.  An element is observed when *any*
active sensor observes it, which is what makes "would the chassis camera have
seen this anyway?" a question the model can answer: run the same stations with
different sensors enabled and compare.

A deviation counts as detected the first time its governing observation clears
``min_visible_fraction``.  That threshold stands in for a detector's sensitivity
and is the main knob separating this geometric model from a real perception
pipeline.
"""

from dataclasses import dataclass, field
import math

from deltaseek_gazebo.visibility import Camera, OrientedBox, visible_fraction


WRIST = 'wrist'
CHASSIS = 'chassis'

ACTUAL = 'actual'
NOMINAL = 'nominal'
EITHER = 'either'

DETECTION_EVIDENCE = {
    'missing': NOMINAL,
    'displaced': EITHER,
    'rotated': ACTUAL,
    'resized': ACTUAL,
    'unexpected': ACTUAL,
}


@dataclass
class ObservationParams:
    """Thresholds separating a usable observation from a glimpse."""

    samples_per_face: int = 3
    max_incidence: float = math.radians(75.0)
    min_visible_fraction: float = 0.05
    hfov: float = 1.25
    aspect: float = 4.0 / 3.0
    near: float = 0.3
    far: float = 6.0

    def camera(self, xyz, rpy):
        return Camera.from_pose(
            xyz, rpy, hfov=self.hfov, aspect=self.aspect,
            near=self.near, far=self.far)


@dataclass
class Scene:
    """The as-built world, indexed for repeated visibility queries."""

    boxes: dict = field(default_factory=dict)

    @classmethod
    def from_elements(cls, elements):
        return cls({
            element['id']: OrientedBox.from_element(element)
            for element in elements
        })

    def occluders(self, exclude=None):
        return [
            box for key, box in self.boxes.items() if key != exclude
        ]


def normalise_station(station):
    """Return ``{sensor_name: (xyz, rpy)}`` for one observation station.

    A bare ``(xyz, rpy)`` pair is the single-camera form used before sensor
    configurations existed, and is read as the wrist camera.
    """
    if isinstance(station, dict):
        return station
    xyz, rpy = station
    return {WRIST: (xyz, rpy)}


def sensor_params(params, name):
    """Resolve the observation thresholds for one sensor."""
    if isinstance(params, dict):
        resolved = params.get(name)
        if resolved is None:
            raise KeyError(f'no ObservationParams supplied for sensor {name!r}')
        return resolved
    return params


def _fraction(box, camera, occluders, params):
    return visible_fraction(
        box, camera, occluders,
        samples_per_face=params.samples_per_face,
        max_incidence=params.max_incidence,
    )


def observe_elements(camera, scene, params=None):
    """Return the visible surface fraction of every as-built element."""
    params = params or ObservationParams()
    return {
        key: _fraction(box, camera, scene.occluders(exclude=key), params)
        for key, box in scene.boxes.items()
    }


def evidence_fractions(camera, discrepancy, nominal_boxes, scene, params):
    """Return the visible fractions backing one deviation, by evidence source."""
    element_id = discrepancy['element_id']
    kind = discrepancy['type']
    evidence = DETECTION_EVIDENCE.get(kind, ACTUAL)
    results = {}

    if evidence in (ACTUAL, EITHER):
        box = scene.boxes.get(element_id)
        if box is not None:
            results[ACTUAL] = _fraction(
                box, camera, scene.occluders(exclude=element_id), params)

    if evidence in (NOMINAL, EITHER):
        box = nominal_boxes.get(element_id)
        if box is not None:
            # The nominal volume is not part of the as-built scene, so every
            # as-built element is a candidate occluder of the line of sight.
            results[NOMINAL] = _fraction(
                box, camera, scene.occluders(), params)

    return results


def detect_along(viewpoints, nominal_elements, actual_elements, ground_truth,
                 params=None, sensors=None):
    """Replay viewpoints and record when each deviation first becomes visible.

    ``viewpoints`` is a sequence of observation stations, each either a bare
    ``(xyz, rpy)`` camera pose or a ``{sensor_name: (xyz, rpy)}`` mapping, in
    the world frame.  ``sensors`` restricts which of them are active;
    ``params`` is either one ``ObservationParams`` for all sensors or a mapping
    from sensor name to its own.  Returns one record per ground-truth
    deviation.
    """
    params = params or ObservationParams()
    scene = Scene.from_elements(actual_elements)
    nominal_boxes = {
        element['id']: OrientedBox.from_element(element)
        for element in nominal_elements
    }

    records = []
    for discrepancy in ground_truth.get('discrepancies', []):
        records.append({
            'id': discrepancy['id'],
            'type': discrepancy['type'],
            'element_id': discrepancy['element_id'],
            'element_class': discrepancy.get('element_class'),
            'class': discrepancy.get('class', 'unclassified'),
            'severity': discrepancy.get('severity'),
            'detected': False,
            'first_viewpoint': None,
            'best_visible_fraction': 0.0,
            'evidence': None,
            'sensor': None,
        })

    for index, station in enumerate(viewpoints):
        poses = normalise_station(station)
        if sensors is not None:
            poses = {name: pose for name, pose in poses.items()
                     if name in sensors}
        for name, (xyz, rpy) in poses.items():
            active = sensor_params(params, name)
            camera = active.camera(xyz, rpy)
            for record, discrepancy in zip(
                    records, ground_truth['discrepancies']):
                if record['detected']:
                    continue
                fractions = evidence_fractions(
                    camera, discrepancy, nominal_boxes, scene, active)
                if not fractions:
                    continue
                source, value = max(fractions.items(), key=lambda i: i[1])
                if value > record['best_visible_fraction']:
                    record['best_visible_fraction'] = round(float(value), 6)
                if value >= active.min_visible_fraction:
                    record['detected'] = True
                    record['first_viewpoint'] = index
                    record['evidence'] = source
                    record['sensor'] = name

    return records
