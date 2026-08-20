"""Guards for launch wiring that fails silently rather than loudly."""

import importlib.util
from pathlib import Path

from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
import pytest


LAUNCH_DIR = Path(__file__).parents[1] / 'launch'


def _load(name):
    spec = importlib.util.spec_from_file_location(
        name, LAUNCH_DIR / f'{name}.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _has_clearpath():
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory('clearpath_gz')
    except Exception:
        return False
    return True


@pytest.fixture()
def benchmark():
    if not _has_clearpath():
        pytest.skip('needs ros-jazzy-clearpath-simulator installed')
    return _load('benchmark')


def test_rviz_is_evaluated_before_the_clearpath_include(benchmark):
    """The include's 'rviz' argument leaks into this file's scope.

    IncludeLaunchDescription does not open a scope, so passing 'rviz': 'false'
    to the Clearpath launch overwrites this file's own rviz configuration for
    everything evaluated afterwards. When that happened, rviz:=true was
    accepted on the command line and then silently ignored.
    """
    entities = benchmark.entities
    rviz_index = next(
        index for index, entity in enumerate(entities)
        if isinstance(entity, Node) and entity._Action__condition is not None
    )
    include_index = next(
        index for index, entity in enumerate(entities)
        if isinstance(entity, IncludeLaunchDescription)
    )
    assert rviz_index < include_index


def test_benchmark_rviz_layout_shows_the_ground_truth_markers():
    config = (Path(__file__).parents[1] / 'config' / 'benchmark.rviz'
              ).read_text(encoding='utf-8')
    assert '/deltaseek/ground_truth/discrepancies' in config
    # The benchmark runs the Clearpath-generated robot, not the legacy overlay.
    assert '/a300_00000/robot_description' in config
    assert '/a300_0000/robot_description' not in config


def test_generated_worlds_load_the_sensor_systems():
    """Gazebo advertises sensor topics with or without these systems.

    Missing them produces no error, just topics that never carry data: the
    camera renders nothing and the EKF loses its IMU.
    """
    world = (Path(__file__).parents[1] / 'worlds' / 'generated'
             / 'demo_deviated.sdf').read_text(encoding='utf-8')
    assert 'gz-sim-sensors-system' in world
    assert 'gz-sim-imu-system' in world
