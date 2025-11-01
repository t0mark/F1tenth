#!/usr/bin/env python3
import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from builtin_interfaces.msg import Time
import csv


class GlobalCheckpointNode(Node):
    def __init__(self):
        super().__init__('global_checkpoint_node')

        # 파라미터 설정
        self.declare_parameter('checkpoint_csv_path', '')
        self.declare_parameter('publish_topic', '/global_path')
        self.declare_parameter('initial_pose_topic', '/initialpose')

        self.csv_path = self.get_parameter('checkpoint_csv_path').get_parameter_value().string_value
        self.topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        self.initial_pose_topic = self.get_parameter('initial_pose_topic').get_parameter_value().string_value

        if not self.csv_path:
            raise RuntimeError('checkpoint_csv_path parameter is required')

        # CSV에서 체크포인트를 불러옵니다.
        self.waypoints_xy = self._load_checkpoints(self.csv_path)
        self.get_logger().info(f'Loaded {len(self.waypoints_xy)} checkpoints from {self.csv_path}')

        # 시작 웨이포인트 인덱스(RViz의 2D Pose Estimate로 갱신됨)
        self.start_index = 0

        # Path 메시지를 준비합니다.
        self.frame_id = 'map'

        # 퍼블리셔 설정(TRANSIENT_LOCAL QoS로 래치와 유사한 동작을 제공합니다.)
        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Path, self.topic, qos)

        # RViz의 2D Pose Estimate(`/initialpose`)를 구독합니다.
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.initial_pose_topic,
            self._initial_pose_callback,
            10
        )

        # 타임스탬프 유지를 위해 주기적으로 퍼블리시합니다.
        self.timer = self.create_timer(1.0, self._publish_path)
        self._publish_path()

    def _load_checkpoints(self, csv_path):
        """CSV 파일에서 체크포인트(x, y 열)를 불러옵니다."""
        waypoints = []
        with open(csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    x = float(row['x'])
                    y = float(row['y'])
                    waypoints.append((x, y))
                except (ValueError, KeyError):
                    # 잘못된 행은 건너뜁니다.
                    pass
        return waypoints

    def _initial_pose_callback(self, msg: PoseWithCovarianceStamped):
        """
        RViz에서 들어오는 2D Pose Estimate 콜백입니다.
        현재 자세 방향과 가장 잘 맞는 가까운 웨이포인트를 찾습니다.
        """
        pose_x = msg.pose.pose.position.x
        pose_y = msg.pose.pose.position.y

        # 쿼터니언에서 요 각도를 추출합니다.
        q = msg.pose.pose.orientation
        pose_yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # 자세가 향하는 방향 벡터를 계산합니다.
        pose_dir = np.array([np.cos(pose_yaw), np.sin(pose_yaw)])

        # 방향까지 일치하는 가장 가까운 웨이포인트를 찾습니다.
        best_index = 0
        best_score = -float('inf')

        for i in range(len(self.waypoints_xy)):
            wx, wy = self.waypoints_xy[i]

            # 웨이포인트까지의 거리를 계산합니다.
            dist = np.sqrt((wx - pose_x)**2 + (wy - pose_y)**2)

            # 다음 웨이포인트로 향하는 방향을 계산합니다.
            next_i = (i + 1) % len(self.waypoints_xy)
            next_wx, next_wy = self.waypoints_xy[next_i]
            wp_dir = np.array([next_wx - wx, next_wy - wy])
            wp_dir_norm = np.linalg.norm(wp_dir)

            if wp_dir_norm > 0.001:
                wp_dir = wp_dir / wp_dir_norm

                # 내적을 통해 현재 자세 방향과의 정렬도를 구합니다.
                alignment = np.dot(pose_dir, wp_dir)

                # 점수는 가까우면서 정렬이 좋은 웨이포인트에 높은 값을 줍니다.
                # 정렬도가 높고 거리가 짧을수록 점수가 올라갑니다.
                score = alignment - 0.5 * dist

                if score > best_score:
                    best_score = score
                    best_index = i

        self.start_index = best_index
        self.get_logger().info(
            f'2D Pose Estimate received: ({pose_x:.2f}, {pose_y:.2f}, yaw={np.degrees(pose_yaw):.1f}°). '
            f'Starting from waypoint index {self.start_index}'
        )

        # 갱신된 경로를 즉시 퍼블리시합니다.
        self._publish_path()

    def _stamp_now(self) -> Time:
        return self.get_clock().now().to_msg()

    def _publish_path(self):
        """start_index부터 경로를 퍼블리시합니다."""
        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        path_msg.header.stamp = self._stamp_now()

        # start_index부터 순환 경로를 생성합니다.
        n = len(self.waypoints_xy)
        for i in range(n):
            idx = (self.start_index + i) % n
            x, y = self.waypoints_xy[idx]

            ps = PoseStamped()
            ps.header.frame_id = self.frame_id
            ps.header.stamp = path_msg.header.stamp
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)

        # 첫 번째 웨이포인트를 다시 추가해 루프를 닫습니다.
        if len(self.waypoints_xy) > 0:
            x, y = self.waypoints_xy[self.start_index]
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
