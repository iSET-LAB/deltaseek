"""CLI for generating nominal and deviated Gazebo benchmark worlds."""

import argparse
from pathlib import Path

import yaml

from deltaseek_gazebo.benchmark import apply_scenario, make_world_sdf


def _load_yaml(path):
    with path.open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def _write_text(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding='utf-8')


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Generate matching SDF worlds and exact deviation labels.')
    parser.add_argument('--manifest', required=True, type=Path)
    parser.add_argument('--scenario', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    args = parser.parse_args(argv)

    manifest = _load_yaml(args.manifest)
    scenario = _load_yaml(args.scenario)
    nominal, actual_elements, ground_truth = apply_scenario(manifest, scenario)
    scenario_id = ground_truth['scenario_id']
    nominal_name = nominal['world']['name']
    deviated_name = f'{nominal_name}_{scenario_id}'

    nominal_path = args.output_dir / f'{scenario_id}_nominal.sdf'
    deviated_path = args.output_dir / f'{scenario_id}_deviated.sdf'
    ground_truth_path = args.output_dir / f'{scenario_id}_ground_truth.yaml'

    _write_text(
        nominal_path,
        make_world_sdf(nominal['world'], nominal['elements'], nominal_name),
    )
    _write_text(
        deviated_path,
        make_world_sdf(nominal['world'], actual_elements, deviated_name),
    )
    _write_text(
        ground_truth_path,
        yaml.safe_dump(ground_truth, sort_keys=False),
    )

    print(f'Nominal world: {nominal_path}')
    print(f'Deviated world: {deviated_path}')
    print(f'Ground truth: {ground_truth_path}')
    print(f'Gazebo world name: {deviated_name}')


if __name__ == '__main__':
    main()
