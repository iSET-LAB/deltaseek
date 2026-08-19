"""Launch the DeltaSeek A300 + UR5e simulation in Gazebo Harmonic."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('deltaseek_gazebo')
    ros_gz_share = get_package_share_directory('ros_gz_sim')

    robot_xacro = os.path.join(package_share, 'urdf', 'husky_a300_ur5e.urdf.xacro')
    controller_config = os.path.join(package_share, 'config', 'controllers.yaml')
    discrepancy_config = os.path.join(package_share, 'config', 'discrepancies.yaml')
    default_world = os.path.join(package_share, 'worlds', 'deltaseek_construction.sdf')

    namespace = LaunchConfiguration('namespace')
    world_file = LaunchConfiguration('world_file')
    headless = LaunchConfiguration('headless')
    rviz = LaunchConfiguration('rviz')

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ', robot_xacro,
            ' namespace:=', namespace,
            ' gazebo_controllers:=', controller_config,
        ]),
        value_type=str,
    )

    resource_paths = [os.path.join(package_share, 'worlds')]
    for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep):
        if prefix:
            resource_paths.append(os.path.join(prefix, 'share'))

    gz_launch = os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': ['-r -v 3 ', world_file]}.items(),
        condition=UnlessCondition(headless),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': ['-r -s -v 3 ', world_file]}.items(),
        condition=IfCondition(headless),
    )

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        remappings=[
            ('joint_states', 'platform/joint_states'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    spawn_robot = TimerAction(
        period=2.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            namespace=namespace,
            output='screen',
            arguments=[
                '-name', 'deltaseek_robot',
                '-allow_renaming', 'false',
                '-x', '0.0', '-y', '0.0', '-z', '0.35',
                '-topic', 'robot_description',
            ],
        )],
    )

    spawn_controllers = TimerAction(
        period=7.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[
                '--controller-manager', ['/', namespace, '/controller_manager'],
                '--controller-manager-timeout', '60',
                'joint_state_broadcaster',
                'platform_velocity_controller',
                'arm_controller',
            ],
        )],
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    camera_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='inspection_camera_info_bridge',
        output='screen',
        arguments=[
            ['/', namespace, '/sensors/inspection_camera/camera_info',
             '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
            ['/', namespace, '/sensors/inspection_camera/points',
             '@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'],
        ],
    )

    color_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='inspection_color_bridge',
        output='screen',
        arguments=[['/', namespace, '/sensors/inspection_camera/image']],
    )
    depth_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='inspection_depth_bridge',
        output='screen',
        arguments=[['/', namespace, '/sensors/inspection_camera/depth_image']],
    )

    ground_truth = Node(
        package='deltaseek_gazebo',
        executable='ground_truth_publisher',
        name='ground_truth_publisher',
        output='screen',
        parameters=[{
            'config_file': discrepancy_config,
            'use_sim_time': True,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(rviz),
        arguments=['-d', os.path.join(package_share, 'config', 'deltaseek.rviz')],
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/tf', ['/', namespace, '/tf']),
            ('/tf_static', ['/', namespace, '/tf_static']),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='a300_0000',
            description='ROS namespace for the mobile manipulator.'),
        DeclareLaunchArgument(
            'world_file', default_value=default_world,
            description='Absolute path to an SDF world.'),
        DeclareLaunchArgument(
            'headless', default_value='false', choices=['true', 'false'],
            description='Run the Gazebo server without its GUI.'),
        DeclareLaunchArgument(
            'rviz', default_value='false', choices=['true', 'false'],
            description='Start RViz with the DeltaSeek configuration.'),
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH', os.pathsep.join(resource_paths)),
        gazebo_gui,
        gazebo_headless,
        state_publisher,
        spawn_robot,
        spawn_controllers,
        clock_bridge,
        camera_info_bridge,
        color_bridge,
        depth_bridge,
        ground_truth,
        rviz_node,
    ])

