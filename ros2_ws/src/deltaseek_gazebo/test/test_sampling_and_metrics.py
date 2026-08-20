"""Tests for scenario sampling and trajectory scoring."""

from pathlib import Path

import numpy as np
import pytest
import yaml

from deltaseek_gazebo.benchmark import apply_scenario
from deltaseek_gazebo.deviation_sampler import _parser, sample_scenario
from deltaseek_gazebo.evaluate import cumulative_distance, evaluate, summarize


PACKAGE_ROOT = Path(__file__).parents[1]


def _manifest():
    path = PACKAGE_ROOT / 'config' / 'benchmarks' / 'demo_nominal.yaml'
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def _args(*argv):
    return _parser().parse_args(
        ['--manifest', 'unused', '--output', 'unused', *argv])


def test_density_controls_how_many_elements_deviate():
    manifest = _manifest()          # seven nominal elements
    sparse = sample_scenario(manifest, _args('--density', '0.0',
                                             '--unexpected-rate', '0.0'))
    half = sample_scenario(manifest, _args('--density', '0.5',
                                           '--unexpected-rate', '0.0'))
    full = sample_scenario(manifest, _args('--density', '1.0',
                                           '--unexpected-rate', '0.0'))
    assert len(sparse['deviations']) == 0
    assert len(half['deviations']) == 4      # round(0.5 * 7)
    assert len(full['deviations']) == 7


def test_unexpected_elements_are_added_on_top_of_density():
    manifest = _manifest()
    scenario = sample_scenario(
        manifest, _args('--density', '0.0', '--unexpected-rate', '1.0'))
    kinds = {item['type'] for item in scenario['deviations']}
    assert kinds == {'unexpected'}
    assert len(scenario['deviations']) == 7


def test_sampling_is_reproducible_and_seed_sensitive():
    manifest = _manifest()
    first = sample_scenario(manifest, _args('--seed', '7'))
    again = sample_scenario(manifest, _args('--seed', '7'))
    other = sample_scenario(manifest, _args('--seed', '8'))
    assert first == again
    assert first['deviations'] != other['deviations']


def test_sampled_scenarios_survive_the_benchmark_generator():
    manifest = _manifest()
    scenario = sample_scenario(manifest, _args('--density', '1.0', '--seed', '3'))
    nominal, actual, ground_truth = apply_scenario(manifest, scenario)
    assert ground_truth['summary']['deviation_count'] == len(scenario['deviations'])
    assert len(nominal['elements']) == 7


def test_each_element_receives_at_most_one_deviation():
    manifest = _manifest()
    scenario = sample_scenario(manifest, _args('--density', '1.0', '--seed', '5'))
    targets = [
        item['target'] for item in scenario['deviations'] if 'target' in item
    ]
    assert len(targets) == len(set(targets))


def test_traversal_distance_ignores_height_changes():
    bases = np.array([[0.0, 0.0, 0.0], [3.0, 4.0, 0.0], [3.0, 4.0, 9.0]])
    distances = cumulative_distance(bases)
    assert distances[0] == 0.0
    assert distances[1] == pytest.approx(5.0)
    assert distances[2] == pytest.approx(5.0)


def test_budget_report_counts_only_detections_within_the_budget():
    records = [
        {'id': 'a', 'detected': True, 'first_viewpoint': 0},
        {'id': 'b', 'detected': True, 'first_viewpoint': 3},
        {'id': 'c', 'detected': False, 'first_viewpoint': None},
    ]
    distances = np.array([0.0, 4.0, 8.0, 12.0])
    report = summarize(records, distances, budgets=(0.5, 1.0))
    assert report['summary']['detection_rate'] == pytest.approx(2 / 3)
    assert report['detection_rate_at_budget']['50%']['detections'] == 1
    assert report['detection_rate_at_budget']['100%']['detections'] == 2


def test_evaluate_end_to_end_on_the_demo_fixture():
    manifest = _manifest()
    scenario = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'benchmarks' / 'demo_deviations.yaml')
        .read_text(encoding='utf-8'))
    trajectory = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'trajectories' / 'demo_sweep.yaml')
        .read_text(encoding='utf-8'))

    report = evaluate(manifest, scenario, trajectory)
    assert report['summary']['deviation_count'] == 4
    assert report['summary']['total_traversal_distance'] > 0.0
    # The sweep is only a harness exercise, but it must find something.
    assert report['summary']['detected_count'] >= 1
    rates = [
        entry['detection_rate']
        for entry in report['detection_rate_at_budget'].values()
    ]
    assert rates == sorted(rates)   # detections accumulate with distance
