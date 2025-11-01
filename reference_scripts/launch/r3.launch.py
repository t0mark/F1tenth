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
      ,Node(
         package='f1tenth_icra_race',
         executable='opponent_detection_node_cpp',
         namespace='',
         name='opponent_detection',
         parameters=[config]
      )
      ,Node(
         package='f1tenth_icra_race',
         executable='opponent_tracking.py',
         namespace='',
         name='opponent_tracking',
         parameters=[config]
      )
      ,Node(
         package='f1tenth_icra_race',
         executable='spliner.py',
         namespace='',
         name='spline_node',
         parameters=[config]
      )
      ,Node(
         package='f1tenth_icra_race',
         executable='state_machine.py',
         namespace='',
         name='state_machine',
         parameters=[config]
      )
   ])