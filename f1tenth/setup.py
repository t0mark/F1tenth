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
        f'{package_name}.planning',
        f'{package_name}.planning.tools',
    ],
    data_files=[
        # Resource index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Package metadata
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'launch', 'localization'), glob('launch/localization/*.py')),
        (os.path.join('share', package_name, 'launch', 'planning'), glob('launch/planning/*.py')),
        (os.path.join('share', package_name, 'launch', 'control'), glob('launch/control/*.py')),
        (os.path.join('share', package_name, 'launch', 'tools'), glob('launch/tools/*.py')),
        # Maps
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        # RViz configurations
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        # Config files
        (os.path.join('share', package_name, 'config', 'control'), glob('config/control/*.yaml')),
        (os.path.join('share', package_name, 'config', 'localization'), glob('config/localization/*.yaml')),
        (os.path.join('share', package_name, 'config', 'planning'), glob('config/planning/*.yaml')),
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
            'control_pure_pursuit_node = f1tenth.control.pure_pursuit:main',

            # Localization
            'odom_broadcaster_node = f1tenth.localization.odom_broadcaster:main',

            # Planning - Global
            'global_centerline_node = f1tenth.planning.global_centerline:main',
            'global_checkpoint_node = f1tenth.planning.global_checkpoint:main',

            # Planning - Local
            'local_avoidance_node = f1tenth.planning.local_avoidance:main',
            'local_hybrid_astar_node = f1tenth.planning.local_hybrid_astar:main',
            'local_sampler_node = f1tenth.planning.local_sampler:main',

            # Planning - Tools
            'checkpoint_recorder_node = f1tenth.planning.tools.checkpoint_recorder:main',
        ],
    },
)
