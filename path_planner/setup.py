# Path planner 패키지 설정 (ament_python)
from setuptools import setup
import os
from glob import glob

package_name = 'path_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, package_name + '.utils'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') if os.path.exists('config') else []),
        (os.path.join('share', package_name, 'data'), glob('data/*') if os.path.exists('data') else []),
    ],
    install_requires=[
        'setuptools',
        'numpy>=1.21,<1.24',
        'scikit-image>=0.18,<0.22',
        'filterpy',
    ],
    zip_safe=True,
    maintainer='tomark',
    maintainer_email='tomark@example.com',
    description='Global centerline and LiDAR local avoidance path planner for F1TENTH Gym ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'global_centerline_node = path_planner.global_centerline:main',
            'global_checkpoint_node = path_planner.global_checkpoint:main',
            'local_avoidance_node = path_planner.local_avoidance:main',
            'local_sampler_node = path_planner.local_sampler:main',
            'local_hybrid_astar_node = path_planner.local_hybrid_astar:main',
            'local_graph = path_planner.utils.local_graph:main',
            'extract_centerline = path_planner.utils.utils:extract_centerline_cli',
            'checkpoint_recorder_node = path_planner.utils.checkpoint_recorder:main',
            'centerline_logger_node = path_planner.centerline_logger:main',
            'opponent_detection_node = path_planner.opponent_detection:main',
            'opponent_tracking_node = path_planner.opponent_tracking:main',
            'spliner_node = path_planner.spliner:main',
            'state_machine_node = path_planner.state_machine:main',
            'controller_node = path_planner.controller:main',
        ],
    },
)
