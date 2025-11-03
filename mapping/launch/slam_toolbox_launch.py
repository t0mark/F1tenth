import os

try:
    from slam_toolbox.srv import SaveMap
except ImportError:
    SaveMap = None

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import logging as launch_logging
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


LOGGER = launch_logging.get_logger('mapping_launch')


def _save_slam_toolbox_map(map_stem: str):
    if not map_stem:
        LOGGER.warning('Map output stem is empty; skipping map save.')
        return []

    map_dir = os.path.dirname(map_stem) or '.'
    os.makedirs(map_dir, exist_ok=True)

    if SaveMap is None:
        LOGGER.error('Map save failed (slam_toolbox SaveMap service definition unavailable).')
        return []

    try:
        import rclpy
    except ImportError as exc:
        LOGGER.error('Map save failed (rclpy import error: %s).', exc)
        return []

    LOGGER.info('Saving slam_toolbox map via save_map service to %s.[yaml|pgm]', map_stem)
    context = rclpy.Context()
    node = None
    try:
        rclpy.init(context=context)
        node = rclpy.create_node('slam_toolbox_map_saver', context=context)

        client = node.create_client(SaveMap, '/slam_toolbox/save_map')
        if not client.wait_for_service(timeout_sec=5.0):
            LOGGER.error('Map save failed (service /slam_toolbox/save_map unavailable).')
            return []

        request = SaveMap.Request()
        request.name.data = map_stem

        future = client.call_async(request)
        try:
            rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)
        except Exception as exc:  # pylint: disable=broad-except
            LOGGER.error('Map save failed while waiting for response: %s', exc)
            return []

        if not future.done():
            LOGGER.error('Map save failed (service call timed out).')
            return []

        response = future.result()
        if response is None:
            LOGGER.error('Map save failed (no response received).')
            return []

        if response.result != response.RESULT_SUCCESS:
            LOGGER.error('Map save failed with result code %s.', response.result)
            return []

        LOGGER.info('Map saved successfully to %s.[yaml|pgm]', map_stem)
    except Exception as exc:  # pylint: disable=broad-except
        LOGGER.error('Map save failed: %s', exc)
    finally:
        if node is not None:
            node.destroy_node()
        if context.ok():
            rclpy.shutdown(context=context)

    return []


def generate_launch_description():
    slam_config = os.path.join(
        get_package_share_directory('mapping'), 'config', 'slam_toolbox.yaml')
    rviz_config = os.path.join(
        get_package_share_directory('mapping'), 'rviz', 'map.rviz')
    localization_launch = os.path.join(
        get_package_share_directory('localization'), 'launch', 'ekf_launch.py')

    default_maps_dir = os.path.join(get_package_share_directory('f1tenth'), 'maps')
    os.makedirs(default_maps_dir, exist_ok=True)
    default_map_stem = os.path.join(default_maps_dir, 'new')

    map_output_argument = DeclareLaunchArgument(
        'map_output_stem',
        default_value=default_map_stem,
        description='Absolute path (without extension) used when saving the map on shutdown.',
    )
    use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time',
    )

    def launch_setup(context, *args, **kwargs):
        use_sim_time = LaunchConfiguration('use_sim_time')
        map_stem = LaunchConfiguration('map_output_stem').perform(context)

        if not map_stem:
            LOGGER.warning('Map output stem is empty; map files will not be saved.')
        else:
            map_dir = os.path.dirname(map_stem) or '.'
            os.makedirs(map_dir, exist_ok=True)

        slam_node = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_config,
                {'use_sim_time': use_sim_time},
            ],
        )

        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='slam_toolbox_rviz',
            arguments=['-d', rviz_config],
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
            ],
        )

        ekf_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )

        save_map_handler = None
        if map_stem:
            save_map_handler = RegisterEventHandler(
                OnShutdown(
                    on_shutdown=[
                        OpaqueFunction(function=lambda ctx: _save_slam_toolbox_map(map_stem))
                    ],
                )
            )

        actions = [
            slam_node,
            rviz_node,
            ekf_launch,
        ]
        if save_map_handler is not None:
            actions.append(save_map_handler)

        return actions

    return LaunchDescription([
        map_output_argument,
        use_sim_time_argument,
        OpaqueFunction(function=launch_setup),
    ])
