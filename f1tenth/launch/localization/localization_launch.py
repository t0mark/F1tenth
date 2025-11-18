import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('f1tenth')

    # 설정 파일 경로
    amcl_config = os.path.join(pkg_share, 'config', 'localization', 'amcl.yaml')
    ekf_launch_file = os.path.join(pkg_share, 'launch', 'localization', 'ekf_launch.py')

    return LaunchDescription([
        # 시뮬레이션 시간 사용 여부
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        # EKF 노드 런치를 별도 파일에서 포함
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ekf_launch_file),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
        ),

        # AMCL 노드 - map->odom 발행
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                amcl_config,
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),

        # TF to Odometry 노드 - map->base_link를 odometry로 발행
        Node(
            package='f1tenth',
            executable='tf_to_odom_node',
            name='tf_to_odom_node',
            output='screen',
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'source_frame': 'map',
                    'target_frame': 'base_link',
                    'odom_topic': '/odom',
                    'publish_rate': 30.0
                }
            ]
        ),
    ])
