import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    slam_config = os.path.join(get_package_share_directory('mapping'), 'config', 'slam_toolbox.yaml')
    rviz_config = os.path.join(get_package_share_directory('mapping'), 'rviz', 'map.rviz')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        # EKF 노드를 별도 런치에서 포함
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('localization'), 'launch', 'ekf_launch.py')
            ),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
        ),
        
        # slam_toolbox 노드 - map->odom 발행
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_config,
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='slam_toolbox_rviz',
            arguments=['-d', rviz_config],
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),
    ])
