import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   config = os.path.join(
      get_package_share_directory('f1tenth_icra_race'),
      'config',
      'r3.params.yaml'
      )

   return LaunchDescription([
      Node(
         package='f1tenth_icra_race',
         executable='controller.py',
         namespace='',
         name='controller',
         parameters=[config]
      )
   ])