import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import logging as launch_logging
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


LOGGER = launch_logging.get_logger('mapping_launch')

def generate_launch_description():
    slam_config = os.path.join(
        get_package_share_directory('f1tenth'), 'config', 'mapping', 'slam_toolbox.yaml')
    rviz_config = os.path.join(
        get_package_share_directory('f1tenth'), 'rviz', 'map.rviz')
    localization_launch = os.path.join(
        get_package_share_directory('f1tenth'), 'launch', 'localization', 'ekf_launch.py')

    use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time',
    )

    def launch_setup(context, *args, **kwargs):
        use_sim_time = LaunchConfiguration('use_sim_time')

        slam_node = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_config,
                {'use_sim_time': use_sim_time},
            ],
        )

        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='slam_toolbox_rviz',
            arguments=['-d', rviz_config],
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
            ],
        )

        ekf_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )

        actions = [
            slam_node,
            rviz_node,
            ekf_launch,
        ]

        return actions

    return LaunchDescription([
        use_sim_time_argument,
        OpaqueFunction(function=launch_setup),
    ])