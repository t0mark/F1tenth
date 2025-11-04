#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 기본 인자(시작값) — 이후 노드가 자동으로 조정함
    wheelbase_arg = DeclareLaunchArgument('wheelbase', default_value='0.3302')
    maxsteer_arg  = DeclareLaunchArgument('max_steering_angle', default_value='0.418879')  # 24°

    path_topic_arg      = DeclareLaunchArgument('path_topic', default_value='/local_path')
    fallback_topic_arg  = DeclareLaunchArgument('fallback_path_topic', default_value='/global_path')
    odom_topic_arg      = DeclareLaunchArgument('odom_topic', default_value='/odom')
    drive_topic_arg     = DeclareLaunchArgument('drive_topic', default_value='/drive')

    # Speed control parameters
    vmin_arg   = DeclareLaunchArgument('v_min', default_value='0.5')
    vmax_arg   = DeclareLaunchArgument('v_max', default_value='6.0')
    max_curvature_arg = DeclareLaunchArgument('max_curvature', default_value='0.8')

    # Lookahead parameters
    ld_min_arg = DeclareLaunchArgument('ld_min', default_value='0.5')
    ld_max_arg = DeclareLaunchArgument('ld_max', default_value='1.0')

    # Control parameters
    ltimeout_arg = DeclareLaunchArgument('local_path_timeout', default_value='1.0')
    rate_arg     = DeclareLaunchArgument('control_rate_hz', default_value='50.0')
    smooth_arg   = DeclareLaunchArgument('steer_smooth_alpha', default_value='0.3') 
    curvature_exponent_arg = DeclareLaunchArgument('curvature_exponent', default_value='1.5')
    ref_velocity_arg = DeclareLaunchArgument('ref_velocity', default_value=LaunchConfiguration('v_max'))

    node = Node(
        package='control',
        executable='pure_pursuit',
        name='pure_pursuit_controller',
        output='screen',
        parameters=[{
            'wheelbase':             LaunchConfiguration('wheelbase'),
            'max_steering_angle':    LaunchConfiguration('max_steering_angle'),
            'path_topic':            LaunchConfiguration('path_topic'),
            'fallback_path_topic':   LaunchConfiguration('fallback_path_topic'),
            'odom_topic':            LaunchConfiguration('odom_topic'),
            'drive_topic':           LaunchConfiguration('drive_topic'),
            'v_min':                 LaunchConfiguration('v_min'),
            'v_max':                 LaunchConfiguration('v_max'),
            'max_curvature':         LaunchConfiguration('max_curvature'),
            'ld_min':                LaunchConfiguration('ld_min'),
            'ld_max':                LaunchConfiguration('ld_max'),
            'local_path_timeout':    LaunchConfiguration('local_path_timeout'),
            'control_rate_hz':       LaunchConfiguration('control_rate_hz'),
            'steer_smooth_alpha':    LaunchConfiguration('steer_smooth_alpha'),
            'curvature_exponent':    LaunchConfiguration('curvature_exponent'),
            'ref_velocity':          LaunchConfiguration('ref_velocity'),
        }]
    )

    return LaunchDescription([
        wheelbase_arg, maxsteer_arg,
        path_topic_arg, fallback_topic_arg, odom_topic_arg, drive_topic_arg,
        vmin_arg, vmax_arg, max_curvature_arg,
        ld_min_arg, ld_max_arg,
        ltimeout_arg, rate_arg, smooth_arg,
        curvature_exponent_arg, ref_velocity_arg,
        node
    ])
