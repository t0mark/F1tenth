#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    """
    F1TENTH Hardware Bringup Launch

    Launches the F1TENTH robot hardware including:
    - VESC motor controller
    - Sensors (LiDAR, IMU, etc.)
    - Base platform drivers
    """

    # F1TENTH stack bringup
    f1tenth_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('f1tenth_stack'),
                'launch',
                'bringup_launch.py'
            ])
        ])
    )

    neutral_ackermann = TimerAction(
        period=1.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2',
                    'topic',
                    'pub',
                    '--once',
                    '/ackermann_cmd',
                    'ackermann_msgs/msg/AckermannDriveStamped',
                    '{drive: {speed: 0.0, steering_angle: 0.0}}'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        f1tenth_bringup,
        neutral_ackermann
    ])
