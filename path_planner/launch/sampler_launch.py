from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('path_planner')
    csv_path = os.path.join(pkg_dir, 'centerline.csv')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'csv_path',
            default_value=csv_path,
            description='Path to centerline CSV file'
        ),
        DeclareLaunchArgument(
            'sampling_distance',
            default_value='3.0',
            description='Distance between sampled points in meters'
        ),
        DeclareLaunchArgument(
            'update_hz',
            default_value='100',
            description='Update frequency in Hz'
        ),
        
        Node(
            package='path_planner',
            executable='path_sampler_node',
            name='path_sampler_node',
            output='screen',
            parameters=[
                {'csv_path': LaunchConfiguration('csv_path')},
                {'sampling_distance': LaunchConfiguration('sampling_distance')},
                {'update_hz': LaunchConfiguration('update_hz')},
            ],
        ),
    ])