"""Run an IFC-derived DeltaSeek benchmark with the official Clearpath robot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from deltaseek_gazebo.paths import default_setup_path


def generate_launch_description():
    package_share = get_package_share_directory('deltaseek_gazebo')
    default_output = os.path.join(package_share, 'worlds', 'generated')

    setup_path = LaunchConfiguration('setup_path')
    world_file = LaunchConfiguration('world_file')
    world_name = LaunchConfiguration('world_name')
    ground_truth_file = LaunchConfiguration('ground_truth_file')
    headless = LaunchConfiguration('headless')
    rviz = LaunchConfiguration('rviz')
    spawn_x = LaunchConfiguration('x')
    spawn_y = LaunchConfiguration('y')
    spawn_yaw = LaunchConfiguration('yaw')

    official_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'clearpath_sim.launch.py')),
        launch_arguments={
            'setup_path': setup_path,
            'world_file': world_file,
            'world_name': world_name,
            'generate': 'true',
            'headless': headless,
            'x': spawn_x,
            'y': spawn_y,
            'yaw': spawn_yaw,
            # Clearpath's RViz layout has no marker display; this file starts
            # its own below.
            'rviz': 'false',
        }.items(),
    )

    ground_truth = Node(
        package='deltaseek_gazebo',
        executable='ground_truth_publisher',
        name='ground_truth_publisher',
        output='screen',
        parameters=[{
            'config_file': ground_truth_file,
            'use_sim_time': True,
        }],
    )

    world_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odom',
        output='screen',
        arguments=[
            '--frame-id', 'world',
            '--child-frame-id', 'odom',
        ],
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/tf', '/a300_00000/tf'),
            ('/tf_static', '/a300_00000/tf_static'),
        ],
    )

    # Clearpath's own RViz layout has no marker display, so the ground-truth
    # discrepancies would be published but invisible. Use this package's
    # benchmark layout instead.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(rviz),
        arguments=['-d', os.path.join(package_share, 'config', 'benchmark.rviz')],
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/tf', '/a300_00000/tf'),
            ('/tf_static', '/a300_00000/tf_static'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'setup_path',
            default_value=default_setup_path(),
            description='Clearpath setup directory containing robot.yaml.'),
        DeclareLaunchArgument(
            'world_file',
            default_value=os.path.join(default_output, 'demo_deviated.sdf'),
            description='Generated deviated SDF world.'),
        DeclareLaunchArgument(
            'world_name',
            default_value='deltaseek_ifc_nominal_demo',
            description='World name embedded in the generated SDF.'),
        DeclareLaunchArgument(
            'ground_truth_file',
            default_value=os.path.join(
                default_output, 'demo_ground_truth.yaml'),
            description='Ground-truth YAML generated with the SDF.'),
        DeclareLaunchArgument(
            'headless', default_value='false', choices=['true', 'false'],
            description='Run the Gazebo server without its GUI.'),
        DeclareLaunchArgument(
            'x', default_value='0.0', description='Spawn x in the world frame.'),
        DeclareLaunchArgument(
            'y', default_value='0.0', description='Spawn y in the world frame.'),
        DeclareLaunchArgument(
            'yaw', default_value='0.0', description='Spawn yaw in radians.'),
        DeclareLaunchArgument(
            'rviz', default_value='false', choices=['true', 'false'],
            description='Start RViz with the benchmark layout and ground-truth markers.'),
        # rviz_node must precede official_robot. IncludeLaunchDescription
        # does not open a scope, so the include's 'rviz': 'false' argument
        # overwrites this file's rviz configuration for everything evaluated
        # after it. Scoping the include instead would stop Clearpath's own
        # arguments from propagating into it.
        rviz_node,
        official_robot,
        ground_truth,
        world_to_odom,
    ])
