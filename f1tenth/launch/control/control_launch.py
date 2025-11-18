#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Vehicle parameters
    wheelbase_arg = DeclareLaunchArgument('wheelbase', default_value='0.3302')
    max_steer_arg = DeclareLaunchArgument('max_steering_angle', default_value='0.4189')

    # Topic parameters
    path_topic_arg = DeclareLaunchArgument('path_topic', default_value='/local_path')
    odom_topic_arg = DeclareLaunchArgument('odom_topic', default_value='/odom')
    drive_topic_arg = DeclareLaunchArgument('drive_topic', default_value='/drive')

    # Speed parameters
    vmin_arg = DeclareLaunchArgument('v_min', default_value='3.0')
    vmax_arg = DeclareLaunchArgument('v_max', default_value='9.0')
    max_lat_accel_arg = DeclareLaunchArgument('max_lateral_accel', default_value='14.715')  # 1.5 * g (F1TENTH benchmark)

    # Lookahead parameters
    ld_min_arg = DeclareLaunchArgument('ld_min', default_value='0.5')
    ld_max_arg = DeclareLaunchArgument('ld_max', default_value='1.8')
    ld_gain_arg = DeclareLaunchArgument('ld_speed_gain', default_value='0.18')

    # Control parameters
    rate_arg = DeclareLaunchArgument('control_rate_hz', default_value='50.0')
    smooth_arg = DeclareLaunchArgument('steer_smooth_alpha', default_value='0.3')

    node = Node(
        package='f1tenth',
        executable='pure_pursuit_node',
        name='pure_pursuit_controller',
        output='screen',
        parameters=[{
            'wheelbase': LaunchConfiguration('wheelbase'),
            'max_steering_angle': LaunchConfiguration('max_steering_angle'),
            'path_topic': LaunchConfiguration('path_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'drive_topic': LaunchConfiguration('drive_topic'),
            'v_min': LaunchConfiguration('v_min'),
            'v_max': LaunchConfiguration('v_max'),
            'max_lateral_accel': LaunchConfiguration('max_lateral_accel'),
            'ld_min': LaunchConfiguration('ld_min'),
            'ld_max': LaunchConfiguration('ld_max'),
            'ld_speed_gain': LaunchConfiguration('ld_speed_gain'),
            'control_rate_hz': LaunchConfiguration('control_rate_hz'),
            'steer_smooth_alpha': LaunchConfiguration('steer_smooth_alpha'),
        }]
    )

    return LaunchDescription([
        wheelbase_arg, max_steer_arg,
        path_topic_arg, odom_topic_arg, drive_topic_arg,
        vmin_arg, vmax_arg, max_lat_accel_arg,
        ld_min_arg, ld_max_arg, ld_gain_arg,
        rate_arg, smooth_arg,
        node
    ])
