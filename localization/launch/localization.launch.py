from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Get the launch directory
    pkg_dir = get_package_share_directory('localization')
    
    # Path to AMCL config file
    amcl_config_file = os.path.join(pkg_dir, 'config', 'amcl_config.yaml')
    
    # AMCL node with topic remapping for F1TENTH
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_config_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odom', '/ego_racecar/odom'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )
    
    # Lifecycle manager for AMCL
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['amcl']}
        ]
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add declarations
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add nodes
    ld.add_action(amcl_node)
    ld.add_action(lifecycle_manager_localization)
    
    return ld