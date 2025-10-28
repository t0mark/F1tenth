# F1TENTH Simulator 패키지 설정 파일
# ROS2에서 F1TENTH 시뮬레이션을 사용하기 위한 패키지 빌드 및 설치 설정

from setuptools import setup
import os
from glob import glob
from pathlib import Path
import atexit

# 패키지 이름
package_name = 'simulator'


def create_marker_file(name: str) -> str:
    """빌드 시점에만 존재하는 ament 리소스 인덱스 마커를 생성하고 상대 경로를 반환."""
    project_root = Path(__file__).resolve().parent
    generated_dir = project_root / '.generated_ament_index'
    generated_dir.mkdir(exist_ok=True)
    marker_path = generated_dir / name
    marker_path.write_text(f'{name}\n', encoding='utf-8')

    def _cleanup():
        try:
            marker_path.unlink()
        except FileNotFoundError:
            pass
        try:
            generated_dir.rmdir()
        except OSError:
            # 디렉터리가 비어있지 않으면 그대로 둔다.
            pass

    atexit.register(_cleanup)
    return str(marker_path.relative_to(project_root))


ament_marker_file = create_marker_file(package_name)

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
        (os.path.join('share', package_name, 'launch'), glob('launch/*.xacro')),
        # RViz 설정 파일들 (시각화 설정)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.rviz')),
        # 설정 파일들 (매개변수 등)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # 맵 파일들 (.yaml, .png 등)
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
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
