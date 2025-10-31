from setuptools import setup
import os
from glob import glob

package_name = 'path_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=['path_planner'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('centerline.csv'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    author='Your Name',
    author_email='your_email@example.com',
    description='Path planner node that publishes local paths from global waypoints',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'path_sampler_node = path_planner.sampler:main',
        ],
    },
)