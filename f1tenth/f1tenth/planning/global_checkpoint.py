#!/usr/bin/env python3
import os
import csv
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from builtin_interfaces.msg import Time
from ament_index_python.packages import get_package_share_directory


class GlobalCheckpointNode(Node):
    def __init__(self):
        super().__init__('global_checkpoint_node')

        # 파라미터
        self.declare_parameter('checkpoint_csv_path', '')
        self.declare_parameter('publish_topic', '/global_path')
        self.declare_parameter('initial_pose_topic', '/initialpose')
        # 얼마나 길게 만들지
        self.declare_parameter('repeat_count', 30)

        csv_filename = self.get_parameter('checkpoint_csv_path').get_parameter_value().string_value
        self.topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        self.initial_pose_topic = self.get_parameter('initial_pose_topic').get_parameter_value().string_value
        self.repeat_count = self.get_parameter('repeat_count').get_parameter_value().integer_value

        if not csv_filename:
            raise RuntimeError('checkpoint_csv_path parameter (filename) is required')

        pkg_share = get_package_share_directory('f1tenth')
        self.csv_path = os.path.join(pkg_share, 'data', csv_filename)

        # CSV에서 로드한 체크포인트를 그대로 사용
        self.base_waypoints = self._load_checkpoints(self.csv_path)
        self.get_logger().info(f'Loaded {len(self.base_waypoints)} checkpoints from {self.csv_path}')

        # 시작 인덱스
        self.start_index = 0
        self.frame_id = 'map'

        # 퍼블리셔
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(Path, self.topic, qos)

        # initialpose
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.initial_pose_topic,
            self._initial_pose_callback,
            10,
        )

        # 한 번 내보내고 가끔만 갱신
        self._publish_path()
        self.timer = self.create_timer(5.0, self._publish_path)

    # --------------------------------------------------------------
    def _load_checkpoints(self, csv_path):
        pts = []
        with open(csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    x = float(row['x'])
                    y = float(row['y'])
                    pts.append((x, y))
                except (ValueError, KeyError):
                    pass
        return pts

    # --------------------------------------------------------------
    def _initial_pose_callback(self, msg: PoseWithCovarianceStamped):
        pose_x = msg.pose.pose.position.x
        pose_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        pose_yaw = np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        pose_dir = np.array([np.cos(pose_yaw), np.sin(pose_yaw)])

        best_idx = 0
        best_score = -1e9

        for i, (wx, wy) in enumerate(self.base_waypoints):
            dist = np.hypot(wx - pose_x, wy - pose_y)

            next_i = (i + 1) % len(self.base_waypoints)
            nx, ny = self.base_waypoints[next_i]
            seg = np.array([nx - wx, ny - wy])
            seg_norm = np.linalg.norm(seg)
            if seg_norm > 1e-3:
                seg = seg / seg_norm
                alignment = np.dot(pose_dir, seg)
                score = alignment - 0.1 * dist
                if score > best_score:
                    best_score = score
                    best_idx = i

        self.start_index = best_idx
        self.get_logger().info(
            f"initialpose → start idx {self.start_index} (x={pose_x:.2f}, y={pose_y:.2f})"
        )
        self._publish_path()

    # --------------------------------------------------------------
    def _stamp_now(self) -> Time:
        return self.get_clock().now().to_msg()

    # --------------------------------------------------------------
    def _publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        path_msg.header.stamp = self._stamp_now()

        n = len(self.base_waypoints)
        if n == 0:
            self.pub.publish(path_msg)
            return

        # 재샘플된 폐곡선을 여러 번 이어붙여서 사실상 무한 경로처럼
        for r in range(self.repeat_count):
            for i in range(n):
                idx = (self.start_index + i) % n
                x, y = self.base_waypoints[idx]

                ps = PoseStamped()
                ps.header.frame_id = self.frame_id
                ps.header.stamp = path_msg.header.stamp
                ps.pose.position.x = float(x)
                ps.pose.position.y = float(y)
                ps.pose.orientation.w = 1.0
                path_msg.poses.append(ps)

        self.pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalCheckpointNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
