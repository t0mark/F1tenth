#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    output_csv = LaunchConfiguration('output_csv')
    auto_save = ParameterValue(LaunchConfiguration('auto_save_on_add'), value_type=bool)
    map_frame = LaunchConfiguration('map_frame')
    publish_topic = LaunchConfiguration('publish_topic')
    map_path = LaunchConfiguration('map_path')

    gym_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('f1tenth_gym_ros'), 'launch', 'gym_bridge_launch.py'])
        ),
        launch_arguments={'map_path': map_path}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'output_csv',
            default_value=PathJoinSubstitution([FindPackageShare('f1tenth_path_planner'), 'data', 'checkpoints.csv']),
            description='Destination CSV file for recorded checkpoints'),
        DeclareLaunchArgument(
            'auto_save_on_add',
            default_value='false',
            description='Persist to CSV automatically each time a checkpoint is recorded'),
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='Global frame that checkpoints are expressed in'),
        DeclareLaunchArgument(
            'publish_topic',
            default_value='/checkpoint_path',
            description='Topic name to publish Path visualization'),
        DeclareLaunchArgument(
            'map_path',
            default_value='',
            description='Path to the map YAML file (with .yaml extension). If empty, uses the map from gym bridge sim.yaml config file.'),
        gym_launch,
        Node(
            package='f1tenth_path_planner',
            executable='checkpoint_recorder_node',
            name='checkpoint_recorder_node',
            output='screen',
            parameters=[{
                'output_csv_path': output_csv,
                'auto_save_on_add': auto_save,
                'map_frame': map_frame,
                'publish_topic': publish_topic,
            }]
        ),
    ])
