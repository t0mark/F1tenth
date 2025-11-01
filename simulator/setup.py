# F1TENTH Simulator 패키지 설정 파일
# ROS2에서 F1TENTH 시뮬레이션을 사용하기 위한 패키지 빌드 및 설치 설정

from setuptools import setup
import os
from glob import glob

# 패키지 이름
package_name = 'simulator'


# ament 리소스 마커 파일 경로 (resource 디렉토리에 실제 파일 존재)
ament_marker_file = 'resource/' + package_name

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Register package marker with the ament resource index
        ('share/ament_index/resource_index/packages', [ament_marker_file]),
        # 패키지 메타데이터 파일
        ('share/' + package_name, ['package.xml']),
        # 런치 파일들 (.py 파일)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # URDF/Xacro 파일들 (로봇 모델 정의)
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        # RViz 설정 파일들 (시각화 설정)
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        # 시뮬레이션 설정 파일들 (매개변수 등)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    # 설치 시 필요한 의존성
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Billy Zheng',
    maintainer_email='billyzheng.bz@gmail.com',
    description='F1TENTH Simulation Environment Bridge for ROS2',
    license='MIT',
    # 실행 가능한 스크립트 정의
    entry_points={
        'console_scripts': [
            # gym_bridge 명령어 정의
            'gym_bridge = simulator.gym_bridge:main'
        ],
    },
)
