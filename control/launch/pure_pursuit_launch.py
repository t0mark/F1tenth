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
    vmin_arg   = DeclareLaunchArgument('v_min', default_value='0.5')  # Safety minimum (emergency/obstacle)
    vmax_arg   = DeclareLaunchArgument('v_max', default_value='9.0')
    max_lateral_accel_arg = DeclareLaunchArgument('max_lateral_accel', default_value='4.0')  # m/s²
    max_curvature_arg = DeclareLaunchArgument('max_curvature', default_value='0.25')

    # Lookahead parameters
    ld_min_arg = DeclareLaunchArgument('ld_min', default_value='1.0')
    ld_max_arg = DeclareLaunchArgument('ld_max', default_value='4.0')
    lookahead_time_arg = DeclareLaunchArgument('lookahead_time', default_value='0.35')  # steady curvature
    lookahead_time_accel_arg = DeclareLaunchArgument('lookahead_time_accel', default_value='0.25')  # steady curvature (responsive)
    lookahead_time_decel_arg = DeclareLaunchArgument('lookahead_time_decel', default_value='0.6')  # changing curvature (predictive)
    lookahead_preview_distance_arg = DeclareLaunchArgument('lookahead_preview_distance', default_value='3.0')  # preview distance (m)
    curvature_change_threshold_arg = DeclareLaunchArgument('curvature_change_threshold', default_value='0.05')  # rad/m
    curvature_sensitivity_arg = DeclareLaunchArgument('curvature_sensitivity', default_value='0.65')  # 0-1

    # Control parameters
    ltimeout_arg = DeclareLaunchArgument('local_path_timeout', default_value='1.0')
    rate_arg     = DeclareLaunchArgument('control_rate_hz', default_value='50.0')
    smooth_arg   = DeclareLaunchArgument('steer_smooth_alpha', default_value='0.25')
    curvature_exponent_arg = DeclareLaunchArgument('curvature_exponent', default_value='1.4')  # reduced from 1.8
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
            'max_lateral_accel':     LaunchConfiguration('max_lateral_accel'),
            'max_curvature':         LaunchConfiguration('max_curvature'),
            'ld_min':                LaunchConfiguration('ld_min'),
            'ld_max':                LaunchConfiguration('ld_max'),
            'lookahead_time':        LaunchConfiguration('lookahead_time'),
            'lookahead_time_accel':  LaunchConfiguration('lookahead_time_accel'),
            'lookahead_time_decel':  LaunchConfiguration('lookahead_time_decel'),
            'lookahead_preview_distance': LaunchConfiguration('lookahead_preview_distance'),
            'curvature_change_threshold': LaunchConfiguration('curvature_change_threshold'),
            'curvature_sensitivity': LaunchConfiguration('curvature_sensitivity'),
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
        vmin_arg, vmax_arg, max_lateral_accel_arg, max_curvature_arg,
        ld_min_arg, ld_max_arg,
        lookahead_time_arg, lookahead_time_accel_arg, lookahead_time_decel_arg,
        lookahead_preview_distance_arg, curvature_change_threshold_arg,
        curvature_sensitivity_arg,
        ltimeout_arg, rate_arg, smooth_arg,
        curvature_exponent_arg, ref_velocity_arg,
        node
    ])
