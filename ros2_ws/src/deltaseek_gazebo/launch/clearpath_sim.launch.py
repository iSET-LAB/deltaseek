"""Run the official Clearpath robot generator and spawner in Gazebo.

The Gazebo GUI is started by default.  Pass ``headless:=true`` for a
server-only run, which is what a machine reached over SSH needs.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from deltaseek_gazebo.paths import default_setup_path


def generate_launch_description():
    clearpath_gz_share = get_package_share_directory('clearpath_gz')
    ros_gz_share = get_package_share_directory('ros_gz_sim')

    setup_path = LaunchConfiguration('setup_path')
    world_file = LaunchConfiguration('world_file')
    world_name = LaunchConfiguration('world_name')
    generate = LaunchConfiguration('generate')
    headless = LaunchConfiguration('headless')
    rviz = LaunchConfiguration('rviz')

    resource_paths = [
        os.path.join(clearpath_gz_share, 'worlds'),
        os.path.join(clearpath_gz_share, 'meshes'),
    ]
    for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep):
        if prefix:
            resource_paths.append(os.path.join(prefix, 'share'))

    gz_launch = os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')
    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={
            'gz_args': ['-r -v 3 ', world_file],
        }.items(),
        condition=UnlessCondition(headless),
    )
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={
            'gz_args': ['-r -s -v 3 ', world_file],
        }.items(),
        condition=IfCondition(headless),
    )

    clearpath_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(clearpath_gz_share, 'launch', 'robot_spawn.launch.py')),
        launch_arguments={
            'setup_path': setup_path,
            'world': world_name,
            'use_sim_time': 'true',
            'rviz': rviz,
            'generate': generate,
            'x': '0.0',
            'y': '0.0',
            'z': '0.3',
            'yaw': '0.0',
        }.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    # clearpath_generator_gz 2.9.2 emits an empty bridge configuration for the
    # simulated Phidgets Spatial IMU even though the official sensor Xacro
    # publishes this Gazebo Transport topic. Keep this simulation-only bridge
    # in the wrapper; robot.yaml remains the hardware source of truth.
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='imu_0_sim_bridge',
        output='screen',
        arguments=[
            '/a300_00000/sensors/imu_0/data'
            '@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
    )

    # The Clearpath realsense macro runs with use_nominal_extrinsics disabled,
    # so it never emits optical frames, yet the Gazebo sensor stamps every
    # image and point cloud with camera_0_color_optical_frame. Without this
    # transform any TF-aware consumer silently drops the camera stream, which
    # is what makes RViz's Camera display stay blank.
    camera_optical_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_0_optical_frame',
        output='screen',
        arguments=[
            '--frame-id', 'camera_0_link',
            '--child-frame-id', 'camera_0_color_optical_frame',
            '--roll', '-1.5707963', '--pitch', '0', '--yaw', '-1.5707963',
        ],
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
            description='Directory containing the authoritative Clearpath robot.yaml.'),
        DeclareLaunchArgument(
            'world_file',
            default_value=os.path.join(clearpath_gz_share, 'worlds', 'construction.sdf'),
            description='Absolute SDF world path.'),
        DeclareLaunchArgument(
            'world_name',
            default_value='office_construction',
            description='The <world name> inside world_file, used by sensor bridges.'),
        DeclareLaunchArgument(
            'generate', default_value='true', choices=['true', 'false'],
            description='Regenerate all Clearpath files from robot.yaml.'),
        DeclareLaunchArgument(
            'headless', default_value='false', choices=['true', 'false'],
            description='Run the Gazebo server without its GUI.'),
        DeclareLaunchArgument(
            'rviz', default_value='false', choices=['true', 'false'],
            description='Start the Clearpath RViz configuration.'),
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH', os.pathsep.join(resource_paths)),
        gz_gui,
        gz_server,
        clearpath_spawn,
        clock_bridge,
        imu_bridge,
        camera_optical_frame,
    ])
