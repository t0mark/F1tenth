#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    """
    RealSense Camera Launch

    Launches Intel RealSense camera with:
    - Gyroscope enabled
    - Accelerometer enabled
    - IMU data unified (gyro + accel combined)
    """

    # RealSense camera with IMU
    realsense_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '1',
            'base_frame_id': 'base_link'
        }.items()
    )

    # Static TF: base_link -> camera_base_link matching RealSense mounting
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_camera',
        arguments=['0.27', '0.0', '0.11', '0.0', '0.0', '0.0', 'base_link', 'camera_base_link'],
        output='screen'
    )

    return LaunchDescription([
        realsense_camera,
        static_tf_camera
    ])
