import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_directory = os.path.join(
        get_package_share_directory('mapping'), 'config')
    config_basename = 'mapping.lua'

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=[
            '-configuration_directory', config_directory,
            '-configuration_basename', config_basename,
        ],
        remappings=[
            ('imu', '/camera/imu'),
        ],
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'resolution': 0.05}],
    )

    return LaunchDescription([
        cartographer_node,
        occupancy_grid_node,
    ])
