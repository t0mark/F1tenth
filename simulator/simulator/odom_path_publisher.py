#!/usr/bin/env python3

"""Convert Odometry stream into a Path trajectory."""

from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomPathPublisher(Node):
    """Subscribe to Odometry and publish accumulated Path points."""

    def __init__(self) -> None:
        super().__init__('odom_path_publisher')

        # Parameters allow remapping topics and limiting the history length
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('path_frame_id', '')
        self.declare_parameter('max_points', 1000)
        self.declare_parameter('publish_rate', 0.0)

        self._odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self._path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        self._path_frame_id = self.get_parameter('path_frame_id').get_parameter_value().string_value
        self._max_points = max(1, int(self.get_parameter('max_points').value))
        publish_rate = float(self.get_parameter('publish_rate').value)
        self._publish_period = Duration(seconds=0.0) if publish_rate <= 0.0 else Duration(seconds=1.0 / publish_rate)

        self._path_msg = Path()
        self._path_msg.header.frame_id = self._path_frame_id

        self._last_publish_time: Optional[Time] = None

        self._poses: List[PoseStamped] = []

        self._odom_sub = self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)
        self._path_pub = self.create_publisher(Path, self._path_topic, 10)

        self.get_logger().info(
            f'Publishing odom trajectory from {self._odom_topic} to {self._path_topic} '
            f'(frame="{self._path_frame_id or "odom header"}", max_points={self._max_points})'
        )

    def _on_odom(self, msg: Odometry) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        frame_id = self._path_frame_id or pose.header.frame_id
        pose.header.frame_id = frame_id

        self._poses.append(pose)
        if len(self._poses) > self._max_points:
            self._poses.pop(0)

        self._path_msg.header.frame_id = frame_id
        self._path_msg.header.stamp = msg.header.stamp
        self._path_msg.poses = list(self._poses)

        now = self.get_clock().now()
        if self._publish_period.nanoseconds <= 0:
            self._path_pub.publish(self._path_msg)
            return

        if self._last_publish_time is None or (now - self._last_publish_time) >= self._publish_period:
            self._path_pub.publish(self._path_msg)
            self._last_publish_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
