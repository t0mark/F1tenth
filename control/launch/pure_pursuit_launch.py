#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 기본 인자(시작값) — 이후 노드가 자동으로 조정함
    lookahead_arg = DeclareLaunchArgument('lookahead_distance', default_value='1.2')
    speed_arg     = DeclareLaunchArgument('speed', default_value='2.0')
    wheelbase_arg = DeclareLaunchArgument('wheelbase', default_value='0.3302')
    maxsteer_arg  = DeclareLaunchArgument('max_steering_angle', default_value='0.418879')  # 24°

    path_topic_arg      = DeclareLaunchArgument('path_topic', default_value='/local_path')
    fallback_topic_arg  = DeclareLaunchArgument('fallback_path_topic', default_value='/global_path')
    odom_topic_arg      = DeclareLaunchArgument('odom_topic', default_value='/ego_racecar/odom')
    drive_topic_arg     = DeclareLaunchArgument('drive_topic', default_value='/drive')

    # Adaptive 관련 인자(원하면 CLI로 조정)
    vmin_arg   = DeclareLaunchArgument('v_min', default_value='1.3') # 1.0
    vmax_arg   = DeclareLaunchArgument('v_max', default_value='3.0') # 2.0
    aymax_arg  = DeclareLaunchArgument('a_y_max', default_value='4.4') # 3.0

    ld_straight_arg = DeclareLaunchArgument('ld_straight', default_value='1.9') #1.3
    ld_corner_arg   = DeclareLaunchArgument('ld_corner', default_value='0.8') # 0.9
    ld_min_arg      = DeclareLaunchArgument('ld_min', default_value='0.8') # 0.8
    ld_max_arg      = DeclareLaunchArgument('ld_max', default_value='2.1') # 1.6
    dthr_arg        = DeclareLaunchArgument('delta_corner_thresh', default_value='0.20944')  # 12°

    ltimeout_arg = DeclareLaunchArgument('local_path_timeout', default_value='0.5') # 1.0
    rate_arg     = DeclareLaunchArgument('control_rate_hz', default_value='50.0') # 50.0
    smooth_arg   = DeclareLaunchArgument('steer_smooth_alpha', default_value='0.25') # 0.25

# 빠르게 하기 위해선  가속 한계(a_y_max)↑, v_max↑, ld_corner↓, smoothing 낮춰 민첩↑, 제어주기↑
# Hz 높이면 제어 촘촘해짐, 하드웨어 인식 되는지 확인 해야함
# timeout 경로 저장 시간, 

    node = Node(
        package='control',
        executable='pure_pursuit',
        name='pure_pursuit_controller',
        output='screen',
        parameters=[{
            'lookahead_distance':    LaunchConfiguration('lookahead_distance'),
            'speed':                 LaunchConfiguration('speed'),
            'wheelbase':             LaunchConfiguration('wheelbase'),
            'max_steering_angle':    LaunchConfiguration('max_steering_angle'),
            'path_topic':            LaunchConfiguration('path_topic'),
            'fallback_path_topic':   LaunchConfiguration('fallback_path_topic'),
            'odom_topic':            LaunchConfiguration('odom_topic'),
            'drive_topic':           LaunchConfiguration('drive_topic'),
            'v_min':                 LaunchConfiguration('v_min'),
            'v_max':                 LaunchConfiguration('v_max'),
            'a_y_max':               LaunchConfiguration('a_y_max'),
            'ld_straight':           LaunchConfiguration('ld_straight'),
            'ld_corner':             LaunchConfiguration('ld_corner'),
            'ld_min':                LaunchConfiguration('ld_min'),
            'ld_max':                LaunchConfiguration('ld_max'),
            'delta_corner_thresh':   LaunchConfiguration('delta_corner_thresh'),
            'local_path_timeout':    LaunchConfiguration('local_path_timeout'),
            'control_rate_hz':       LaunchConfiguration('control_rate_hz'),
            'steer_smooth_alpha':    LaunchConfiguration('steer_smooth_alpha'),
        }]
    )

    return LaunchDescription([
        lookahead_arg, speed_arg, wheelbase_arg, maxsteer_arg,
        path_topic_arg, fallback_topic_arg, odom_topic_arg, drive_topic_arg,
        vmin_arg, vmax_arg, aymax_arg,
        ld_straight_arg, ld_corner_arg, ld_min_arg, ld_max_arg, dthr_arg,
        ltimeout_arg, rate_arg, smooth_arg,
        node
    ])
