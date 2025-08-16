# F1Tenth Gym 브리지 런치 파일
# F1Tenth 시뮬레이션 환경을 ROS2에서 실행하기 위한 모든 노드들을 시작

from launch import LaunchDescription
from launch_ros.actions import Node 
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    # 런치 설명 객체 생성
    ld = LaunchDescription()
    
    # 설정 파일
    ## 경로 구성
    config = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'),
        'config',
        'sim.yaml'
        )
    ## 파싱
    config_dict = yaml.safe_load(open(config, 'r'))

    # 상대 차량 존재 여부
    has_opp = config_dict['bridge']['ros__parameters']['num_agent'] > 1  
    # 키보드 조작 여부
    teleop = config_dict['bridge']['ros__parameters']['kb_teleop']

    # 시뮬레이션 브리지 노드 (F1Tenth gym과 ROS2 연결)
    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[config]
    )
    
    # RViz 시각화 노드
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'gym_bridge.rviz')]
    )
    
    # 지도 서버 노드 (정적 지도 제공)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{'yaml_filename': config_dict['bridge']['ros__parameters']['map_path'] + '.yaml'},
                    {'topic': 'map'},
                    {'frame_id': 'map'},
                    {'output': 'screen'},
                    {'use_sim_time': True}]
    )
    
    # Nav2 라이프사이클 관리자 (지도 서버 생명주기 관리)
    ## Nav2: ROS2용 자율주행 네비게이션 프레임워크 (경로계획, 장애물회피, 위치추정 등을 제공)
    ### 현재 패키지에서는 정적 지도 제공만 (map_server)
    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )
    
    # 자아 차량 로봇 상태 퍼블리셔 (URDF 모델 게시)
    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'ego_racecar.xacro')])}],
        remappings=[('/robot_description', 'ego_robot_description')]
    )
    
    # 상대 차량 로봇 상태 퍼블리셔 (멀티 에이전트용)
    opp_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='opp_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'opp_racecar.xacro')])}],
        remappings=[('/robot_description', 'opp_robot_description')]
    )

    # 노드들을 런치 설명에 추가
    ## 시각화
    ld.add_action(rviz_node)
    ## 시뮬레이션 브리지
    ld.add_action(bridge_node)
    ## 라이프사이클 관리
    ld.add_action(nav_lifecycle_node)
    ## 지도 서버
    ld.add_action(map_server_node)
    ## 자아 차량
    ld.add_action(ego_robot_publisher)
    
    ## 상대 차량이 있는 경우에만 추가
    if has_opp:
        ld.add_action(opp_robot_publisher)

    return ld