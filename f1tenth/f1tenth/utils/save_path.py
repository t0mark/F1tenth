#!/usr/bin/env python3
"""
Subscribe to a Path topic and dump the latest trajectory in KITTI format.
"""

from __future__ import annotations

import math
import os
from typing import List, Optional

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory


def quaternion_to_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> List[List[float]]:
    """Convert quaternion to a 3x3 rotation matrix."""
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm == 0.0:
        return [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]

    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm

    r11 = 1 - 2 * (qy * qy + qz * qz)
    r12 = 2 * (qx * qy - qz * qw)
    r13 = 2 * (qx * qz + qy * qw)
    r21 = 2 * (qx * qy + qz * qw)
    r22 = 1 - 2 * (qx * qx + qz * qz)
    r23 = 2 * (qy * qz - qx * qw)
    r31 = 2 * (qx * qz - qy * qw)
    r32 = 2 * (qy * qz + qx * qw)
    r33 = 1 - 2 * (qx * qx + qy * qy)

    return [
        [r11, r12, r13],
        [r21, r22, r23],
        [r31, r32, r33],
    ]


def pose_to_kitti_row(pose: PoseStamped) -> List[float]:
    """
    Convert a PoseStamped to KITTI odometry format (flattened 3x4 matrix).
    """
    orientation = pose.pose.orientation
    translation = pose.pose.position

    rotation = quaternion_to_rotation_matrix(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    )

    return [
        rotation[0][0], rotation[0][1], rotation[0][2], translation.x,
        rotation[1][0], rotation[1][1], rotation[1][2], translation.y,
        rotation[2][0], rotation[2][1], rotation[2][2], translation.z,
    ]


def resolve_default_output_path(filename: str = 'trajectory.txt') -> str:
    """Resolve the data directory path regardless of install or source layout."""
    pkg_share = get_package_share_directory('f1tenth')
    parts = pkg_share.split(os.sep)
    if 'install' in parts:
        install_index = parts.index('install')
        workspace_root = os.sep.join(parts[:install_index])
        src_base = os.path.join(workspace_root, 'src', 'f1tenth')
    else:
        src_base = pkg_share
    return os.path.join(src_base, 'data', filename)


class PathSaver(Node):
    """ROS 2 node that latches onto a Path topic and saves it to disk on shutdown."""

    def __init__(self) -> None:
        super().__init__('path_saver')

        self.declare_parameter('path_topic', '/path')
        default_output = resolve_default_output_path()
        self.declare_parameter('output_file', default_output)

        path_topic_param = self.get_parameter('path_topic').get_parameter_value().string_value
        output_file_param = self.get_parameter('output_file').get_parameter_value().string_value

        self._path_topic = path_topic_param or '/path'
        self._output_file = os.path.expanduser(output_file_param or default_output)

        self._latest_path: Optional[Path] = None

        self.create_subscription(Path, self._path_topic, self._on_path, 10)

        self.get_logger().info(
            f'Path saver listening on {self._path_topic} (output: {self._output_file})'
        )

    def _on_path(self, msg: Path) -> None:
        self._latest_path = msg

    def save_latest_path(self) -> bool:
        """Write the most recent Path to disk in KITTI CSV format."""
        if not self._latest_path or not self._latest_path.poses:
            self.get_logger().warn('No Path data received; skipping save.')
            return False

        rows = [pose_to_kitti_row(pose) for pose in self._latest_path.poses]

        directory = os.path.dirname(self._output_file)
        if directory:
            os.makedirs(directory, exist_ok=True)

        with open(self._output_file, 'w') as outfile:
            for row in rows:
                outfile.write(' '.join(f'{value:.10f}' for value in row))
                outfile.write('\n')

        self.get_logger().info(
            f'Saved {len(rows)} poses to {self._output_file} in KITTI format.'
        )
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_latest_path()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
