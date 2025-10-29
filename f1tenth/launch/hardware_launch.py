#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
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

    # # Static TF: base_link -> camera_gyro_frame (same as laser position)
    # static_tf_camera = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_baselink_to_camera',
    #     arguments=['0.27', '0.0', '0.11', '0.0', '0.0', '0.0', 'base_link', 'camera_base_link'],
    #     output='screen'
    # )

    return LaunchDescription([
        f1tenth_bringup,
        # static_tf_camera
    ])
