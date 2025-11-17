from setuptools import setup
import os
from glob import glob

package_name = 'f1tenth'

setup(
    name=package_name,
    version='1.0.0',
    packages=[
        package_name,
        f'{package_name}.control',
        f'{package_name}.localization',
        f'{package_name}.path_planner',
        f'{package_name}.path_planner.utils',
        f'{package_name}.tools',
    ],
    data_files=[
        # Resource index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Package metadata
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Maps
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        # RViz configurations
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        # Config files
        (os.path.join('share', package_name, 'config', 'control'), glob('config/control/*.yaml')),
        (os.path.join('share', package_name, 'config', 'localization'), glob('config/localization/*.yaml')),
        (os.path.join('share', package_name, 'config', 'path_planner'), glob('config/path_planner/*.yaml')),
        (os.path.join('share', package_name, 'config', 'mapping'), glob('config/mapping/*.yaml')),
        # Data files
        (os.path.join('share', package_name, 'data'), glob('data/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='F1TENTH Team',
    maintainer_email='user@example.com',
    description='F1TENTH integrated autonomous racing system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Control
            'pure_pursuit_node = f1tenth.control.pure_pursuit:main',

            # Localization
            'tf_to_odom_node = f1tenth.localization.tf_to_odom:main',

            # Path Planner - Global
            'global_centerline_node = f1tenth.path_planner.global_centerline:main',
            'global_checkpoint_node = f1tenth.path_planner.global_checkpoint:main',

            # Path Planner - Local
            'local_avoidance_node = f1tenth.path_planner.local_avoidance:main',
            'local_hybrid_astar_node = f1tenth.path_planner.local_hybrid_astar:main',
            'local_sampler_node = f1tenth.path_planner.local_sampler:main',

            # Path Planner - Utils
            'checkpoint_recorder_node = f1tenth.path_planner.utils.checkpoint_recorder:main',

            # Tools
            'f1tenth-optimize-racing-line = f1tenth.tools.optimize_racing_line:main',
            'f1tenth-merge-widths = f1tenth.tools.merge_track_widths:main',
        ],
    },
)
