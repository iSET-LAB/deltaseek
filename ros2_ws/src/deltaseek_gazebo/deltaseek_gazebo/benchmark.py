"""Deterministic nominal-world and deviation generation for DeltaSeek."""

from copy import deepcopy
import hashlib
import math
import re
from xml.etree import ElementTree as ET


SCHEMA_VERSION = 1
SUPPORTED_GEOMETRY = {'box', 'cylinder', 'sphere', 'mesh'}
SUPPORTED_DEVIATIONS = {
    'displaced', 'rotated', 'missing', 'unexpected', 'resized'
}


class BenchmarkError(ValueError):
    """Raised when a benchmark manifest or scenario is invalid."""


def _numbers(value, length, field):
    if not isinstance(value, list) or len(value) != length:
        raise BenchmarkError(f'{field} must be a list of {length} numbers')
    try:
        return [float(item) for item in value]
    except (TypeError, ValueError) as exc:
        raise BenchmarkError(f'{field} must contain only numbers') from exc


def _pose(mapping, field='pose'):
    if not isinstance(mapping, dict):
        raise BenchmarkError(f'{field} must be a mapping')
    return {
        'xyz': _numbers(mapping.get('xyz', [0, 0, 0]), 3, f'{field}.xyz'),
        'rpy': _numbers(mapping.get('rpy', [0, 0, 0]), 3, f'{field}.rpy'),
    }


def _geometry(mapping, field='geometry'):
    if not isinstance(mapping, dict):
        raise BenchmarkError(f'{field} must be a mapping')
    geometry = deepcopy(mapping)
    geometry_type = geometry.get('type')
    if geometry_type not in SUPPORTED_GEOMETRY:
        raise BenchmarkError(
            f'{field}.type must be one of {sorted(SUPPORTED_GEOMETRY)}')
    if geometry_type == 'box':
        geometry['size'] = _numbers(geometry.get('size'), 3, f'{field}.size')
        if any(value <= 0 for value in geometry['size']):
            raise BenchmarkError(f'{field}.size values must be positive')
    elif geometry_type == 'cylinder':
        geometry['radius'] = float(geometry.get('radius', 0))
        geometry['length'] = float(geometry.get('length', 0))
        if geometry['radius'] <= 0 or geometry['length'] <= 0:
            raise BenchmarkError(f'{field} radius and length must be positive')
    elif geometry_type == 'sphere':
        geometry['radius'] = float(geometry.get('radius', 0))
        if geometry['radius'] <= 0:
            raise BenchmarkError(f'{field}.radius must be positive')
    else:
        if not geometry.get('uri'):
            raise BenchmarkError(f'{field}.uri is required for a mesh')
        geometry['scale'] = _numbers(
            geometry.get('scale', [1, 1, 1]), 3, f'{field}.scale')
    return geometry


def _normalise_element(element, field):
    if not isinstance(element, dict):
        raise BenchmarkError(f'{field} must be a mapping')
    result = deepcopy(element)
    result['id'] = str(result.get('id', '')).strip()
    if not result['id']:
        raise BenchmarkError(f'{field}.id is required')
    result['ifc_class'] = str(result.get('ifc_class', 'IfcBuildingElementProxy'))
    result['name'] = str(result.get('name') or result['id'])
    result['pose'] = _pose(result.get('pose', {}), f'{field}.pose')
    result['geometry'] = _geometry(
        result.get('geometry', {}), f'{field}.geometry')
    result['static'] = bool(result.get('static', True))
    result['collision'] = bool(result.get('collision', True))
    result['material'] = {
        'rgba': _numbers(
            result.get('material', {}).get('rgba', [0.65, 0.68, 0.72, 1.0]),
            4,
            f'{field}.material.rgba',
        )
    }
    return result


def validate_manifest(manifest):
    """Validate and normalize a nominal element manifest."""
    if not isinstance(manifest, dict):
        raise BenchmarkError('manifest must be a mapping')
    if manifest.get('schema_version') != SCHEMA_VERSION:
        raise BenchmarkError(f'schema_version must be {SCHEMA_VERSION}')
    world = deepcopy(manifest.get('world', {}))
    world['name'] = str(world.get('name', 'deltaseek_nominal'))
    world['frame_id'] = str(world.get('frame_id', 'world'))
    world['gravity'] = _numbers(
        world.get('gravity', [0, 0, -9.81]), 3, 'world.gravity')
    elements = []
    ids = set()
    for index, raw in enumerate(manifest.get('elements', [])):
        element = _normalise_element(raw, f'elements[{index}]')
        if element['id'] in ids:
            raise BenchmarkError(f'duplicate element id: {element["id"]}')
        ids.add(element['id'])
        elements.append(element)
    if not elements:
        raise BenchmarkError('manifest must contain at least one element')
    return {
        'schema_version': SCHEMA_VERSION,
        'source': deepcopy(manifest.get('source', {})),
        'world': world,
        'elements': elements,
    }


def _scale_geometry(geometry, factors):
    scaled = deepcopy(geometry)
    if geometry['type'] == 'box':
        scaled['size'] = [
            value * factor for value, factor in zip(geometry['size'], factors)
        ]
    elif geometry['type'] == 'cylinder':
        scaled['radius'] *= (factors[0] + factors[1]) / 2.0
        scaled['length'] *= factors[2]
    elif geometry['type'] == 'sphere':
        scaled['radius'] *= sum(factors) / 3.0
    else:
        scaled['scale'] = [
            value * factor
            for value, factor in zip(geometry.get('scale', [1, 1, 1]), factors)
        ]
    return scaled


def _magnitude(deviation, reference, actual):
    deviation_type = deviation['type']
    if deviation_type == 'displaced':
        value = math.sqrt(sum(item ** 2 for item in deviation['translation']))
        return round(value, 9)
    if deviation_type == 'rotated':
        value = math.sqrt(sum(item ** 2 for item in deviation['rotation']))
        return round(value, 9)
    if deviation_type == 'resized':
        value = max(
            abs(item - 1.0) for item in deviation['scale_factors'])
        return round(value, 9)
    return 1.0


def apply_scenario(manifest, scenario):
    """Apply deviation operations and return actual elements and ground truth."""
    nominal = validate_manifest(manifest)
    if not isinstance(scenario, dict):
        raise BenchmarkError('scenario must be a mapping')
    if scenario.get('schema_version') != SCHEMA_VERSION:
        raise BenchmarkError(f'scenario schema_version must be {SCHEMA_VERSION}')

    scenario_info = scenario.get('scenario', {})
    scenario_id = str(scenario_info.get('id', 'scenario'))
    actual_by_id = {item['id']: deepcopy(item) for item in nominal['elements']}
    reference_by_id = {item['id']: deepcopy(item) for item in nominal['elements']}
    discrepancies = []
    used_targets = set()

    for index, raw in enumerate(scenario.get('deviations', [])):
        if not isinstance(raw, dict):
            raise BenchmarkError(f'deviations[{index}] must be a mapping')
        deviation = deepcopy(raw)
        deviation_id = str(deviation.get('id', '')).strip()
        deviation_type = deviation.get('type')
        if not deviation_id:
            raise BenchmarkError(f'deviations[{index}].id is required')
        if deviation_type not in SUPPORTED_DEVIATIONS:
            raise BenchmarkError(
                f'{deviation_id}.type must be one of '
                f'{sorted(SUPPORTED_DEVIATIONS)}')

        target_id = deviation.get('target')
        if deviation_type == 'unexpected':
            actual = _normalise_element(
                deviation.get('element', {}), f'{deviation_id}.element')
            if actual['id'] in actual_by_id:
                raise BenchmarkError(
                    f'{deviation_id} adds existing element {actual["id"]}')
            actual_by_id[actual['id']] = actual
            reference = None
            target_id = actual['id']
        else:
            target_id = str(target_id or '')
            if target_id not in reference_by_id:
                raise BenchmarkError(
                    f'{deviation_id} targets unknown element {target_id}')
            if target_id in used_targets:
                raise BenchmarkError(
                    f'multiple deviations target element {target_id}')
            used_targets.add(target_id)
            reference = reference_by_id[target_id]
            actual = deepcopy(actual_by_id[target_id])
            if deviation_type == 'missing':
                del actual_by_id[target_id]
                actual = None
            elif deviation_type == 'displaced':
                deviation['translation'] = _numbers(
                    deviation.get('translation'), 3,
                    f'{deviation_id}.translation')
                actual['pose']['xyz'] = [
                    value + delta for value, delta in zip(
                        actual['pose']['xyz'], deviation['translation'])
                ]
                actual_by_id[target_id] = actual
            elif deviation_type == 'rotated':
                deviation['rotation'] = _numbers(
                    deviation.get('rotation'), 3,
                    f'{deviation_id}.rotation')
                actual['pose']['rpy'] = [
                    value + delta for value, delta in zip(
                        actual['pose']['rpy'], deviation['rotation'])
                ]
                actual_by_id[target_id] = actual
            elif deviation_type == 'resized':
                deviation['scale_factors'] = _numbers(
                    deviation.get('scale_factors'), 3,
                    f'{deviation_id}.scale_factors')
                if any(value <= 0 for value in deviation['scale_factors']):
                    raise BenchmarkError(
                        f'{deviation_id}.scale_factors must be positive')
                actual['geometry'] = _scale_geometry(
                    actual['geometry'], deviation['scale_factors'])
                actual_by_id[target_id] = actual

        discrepancy = {
            'id': deviation_id,
            'type': deviation_type,
            'element_id': target_id,
            'element_class': (
                reference or actual or {}
            ).get('ifc_class', 'IfcBuildingElementProxy'),
            'severity': float(deviation.get('severity', 0.5)),
            'magnitude': _magnitude(deviation, reference, actual),
            'reference_pose': reference['pose'] if reference else None,
            'actual_pose': actual['pose'] if actual else None,
            'reference_geometry': reference['geometry'] if reference else None,
            'actual_geometry': actual['geometry'] if actual else None,
        }
        discrepancies.append(discrepancy)

    actual_elements = [actual_by_id[key] for key in sorted(actual_by_id)]
    ground_truth = {
        'schema_version': SCHEMA_VERSION,
        'scenario_id': scenario_id,
        'seed': int(scenario_info.get('seed', 0)),
        'frame_id': nominal['world']['frame_id'],
        'source': deepcopy(nominal['source']),
        'summary': {
            'nominal_element_count': len(nominal['elements']),
            'actual_element_count': len(actual_elements),
            'deviation_count': len(discrepancies),
        },
        'discrepancies': discrepancies,
    }
    return nominal, actual_elements, ground_truth


def _format(values):
    if isinstance(values, (list, tuple)):
        return ' '.join(f'{float(value):.9g}' for value in values)
    return f'{float(values):.9g}'


def _safe_name(value):
    cleaned = re.sub(r'[^A-Za-z0-9_]+', '_', value).strip('_')
    if not cleaned:
        cleaned = 'element'
    digest = hashlib.sha1(value.encode('utf-8')).hexdigest()[:8]
    return f'ifc_{cleaned[:48]}_{digest}'


def _append_geometry(parent, geometry):
    geometry_xml = ET.SubElement(parent, 'geometry')
    geometry_type = geometry['type']
    shape = ET.SubElement(geometry_xml, geometry_type)
    if geometry_type == 'box':
        ET.SubElement(shape, 'size').text = _format(geometry['size'])
    elif geometry_type == 'cylinder':
        ET.SubElement(shape, 'radius').text = _format(geometry['radius'])
        ET.SubElement(shape, 'length').text = _format(geometry['length'])
    elif geometry_type == 'sphere':
        ET.SubElement(shape, 'radius').text = _format(geometry['radius'])
    else:
        ET.SubElement(shape, 'uri').text = str(geometry['uri'])
        ET.SubElement(shape, 'scale').text = _format(geometry['scale'])


def _append_element(world, element):
    model = ET.SubElement(world, 'model', {'name': _safe_name(element['id'])})
    ET.SubElement(model, 'static').text = str(element['static']).lower()
    pose = element['pose']['xyz'] + element['pose']['rpy']
    ET.SubElement(model, 'pose').text = _format(pose)
    link = ET.SubElement(model, 'link', {'name': 'link'})
    if element['collision']:
        collision = ET.SubElement(link, 'collision', {'name': 'collision'})
        _append_geometry(collision, element['geometry'])
    visual = ET.SubElement(link, 'visual', {'name': 'visual'})
    _append_geometry(visual, element['geometry'])
    material = ET.SubElement(visual, 'material')
    rgba = element['material']['rgba']
    ET.SubElement(material, 'ambient').text = _format(rgba)
    ET.SubElement(material, 'diffuse').text = _format(rgba)


def make_world_sdf(world_config, elements, world_name):
    """Create an SDF document for a collection of normalized elements."""
    sdf = ET.Element('sdf', {'version': '1.10'})
    world = ET.SubElement(sdf, 'world', {'name': world_name})
    physics = ET.SubElement(
        world, 'physics', {'name': 'default_physics', 'type': 'ignored'})
    ET.SubElement(physics, 'max_step_size').text = '0.003'
    ET.SubElement(physics, 'real_time_factor').text = '1.0'
    ET.SubElement(
        world, 'plugin', {
            'filename': 'gz-sim-physics-system',
            'name': 'gz::sim::systems::Physics',
        })
    ET.SubElement(
        world, 'plugin', {
            'filename': 'gz-sim-user-commands-system',
            'name': 'gz::sim::systems::UserCommands',
        })
    ET.SubElement(
        world, 'plugin', {
            'filename': 'gz-sim-scene-broadcaster-system',
            'name': 'gz::sim::systems::SceneBroadcaster',
        })
    ET.SubElement(world, 'gravity').text = _format(world_config['gravity'])
    scene = ET.SubElement(world, 'scene')
    ET.SubElement(scene, 'ambient').text = '0.55 0.55 0.55 1'
    ET.SubElement(scene, 'background').text = '0.78 0.84 0.92 1'
    ET.SubElement(scene, 'shadows').text = 'false'
    light = ET.SubElement(world, 'light', {'type': 'directional', 'name': 'sun'})
    ET.SubElement(light, 'cast_shadows').text = 'false'
    ET.SubElement(light, 'pose').text = '0 0 10 0 0 0'
    ET.SubElement(light, 'diffuse').text = '0.9 0.9 0.9 1'
    ET.SubElement(light, 'direction').text = '-0.4 0.2 -0.9'
    for element in sorted(elements, key=lambda item: item['id']):
        _append_element(world, element)
    ET.indent(sdf, space='  ')
    return '<?xml version="1.0"?>\n' + ET.tostring(
        sdf, encoding='unicode') + '\n'
