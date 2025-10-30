import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('localization')
    
    # 설정 파일 경로
    amcl_config = os.path.join(pkg_share, 'config', 'amcl.yaml')
    map_file = os.path.join(pkg_share, 'maps', 'underground/underground_map.yaml')
    ekf_launch_file = os.path.join(pkg_share, 'launch', 'ekf_launch.py')
    
    return LaunchDescription([
        # 시뮬레이션 시간 사용 여부
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        # 맵 파일 경로
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to map file to load'
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
        
        # Map Server (맵 로드)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {
                    'yaml_filename': LaunchConfiguration('map'),
                    'use_sim_time': LaunchConfiguration('use_sim_time')
                }
            ]
        ),
        
        # Lifecycle Manager (Nav2 노드 관리)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'autostart': True},
                {'node_names': ['map_server', 'amcl']}
            ]
        ),
    ])
