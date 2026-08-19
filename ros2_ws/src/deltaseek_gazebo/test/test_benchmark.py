"""Tests for deterministic DeltaSeek benchmark generation."""

from pathlib import Path
from xml.etree import ElementTree as ET

import pytest
import yaml

from deltaseek_gazebo.benchmark import (
    BenchmarkError,
    apply_scenario,
    make_world_sdf,
)


PACKAGE_ROOT = Path(__file__).parents[1]


def _load(name):
    path = PACKAGE_ROOT / 'config' / 'benchmarks' / name
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def test_demo_scenario_is_exact_and_deterministic():
    manifest = _load('demo_nominal.yaml')
    scenario = _load('demo_deviations.yaml')
    nominal, actual, ground_truth = apply_scenario(manifest, scenario)

    assert len(nominal['elements']) == 7
    assert len(actual) == 7
    assert ground_truth['summary'] == {
        'nominal_element_count': 7,
        'actual_element_count': 7,
        'deviation_count': 4,
    }

    actual_by_id = {item['id']: item for item in actual}
    assert '1oTetZHTbSC89CoF_sOIS7' not in actual_by_id
    assert actual_by_id['3pEp8vIzrQ48x8Cb1N6zbE']['pose']['xyz'][1] == 0.45
    assert actual_by_id['0fZ$MiOnzIshQgTqMC2x8T']['geometry']['size'] == [
        3.2, 0.7, 0.4]
    assert 'unexpected-crate-01' in actual_by_id

    first = make_world_sdf(
        nominal['world'], actual, 'deltaseek_ifc_nominal_demo')
    second = make_world_sdf(
        nominal['world'], actual, 'deltaseek_ifc_nominal_demo')
    assert first == second
    root = ET.fromstring(first)
    assert root.find("world[@name='deltaseek_ifc_nominal_demo']") is not None


def test_unknown_target_is_rejected():
    manifest = _load('demo_nominal.yaml')
    scenario = {
        'schema_version': 1,
        'scenario': {'id': 'bad'},
        'deviations': [{
            'id': 'missing_unknown',
            'type': 'missing',
            'target': 'not-an-ifc-id',
        }],
    }
    with pytest.raises(BenchmarkError, match='unknown element'):
        apply_scenario(manifest, scenario)
