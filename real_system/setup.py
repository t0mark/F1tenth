from setuptools import setup
import os
from glob import glob

package_name = 'real_system'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        # Resource index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Package metadata
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='F1TENTH Team',
    maintainer_email='user@example.com',
    description='F1TENTH real robot hardware bringup package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
