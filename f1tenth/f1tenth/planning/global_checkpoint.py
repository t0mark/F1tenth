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
        # 재샘플할 간격 (m)
        self.declare_parameter('resample_step', 0.25)

        csv_filename = self.get_parameter('checkpoint_csv_path').get_parameter_value().string_value
        self.topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        self.initial_pose_topic = self.get_parameter('initial_pose_topic').get_parameter_value().string_value
        self.repeat_count = self.get_parameter('repeat_count').get_parameter_value().integer_value
        self.resample_step = self.get_parameter('resample_step').get_parameter_value().double_value

        if not csv_filename:
            raise RuntimeError('checkpoint_csv_path parameter (filename) is required')

        pkg_share = get_package_share_directory('f1tenth')
        self.csv_path = os.path.join(pkg_share, 'data', csv_filename)

        # 원본 포인트
        raw_pts = self._load_checkpoints(self.csv_path)
        self.get_logger().info(f'Loaded {len(raw_pts)} raw checkpoints from {self.csv_path}')

        # ★ 재샘플해서 일정 간격으로 된 폐곡선 만들기
        self.base_waypoints = self._resample_closed_path(raw_pts, self.resample_step)
        self.get_logger().info(f'Resampled to {len(self.base_waypoints)} points (step={self.resample_step} m)')

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
    def _resample_closed_path(self, pts, step):
        """
        pts: [(x,y), ...] 폐곡선이라고 가정
        step: 원하는 점 간격
        -> 일정 간격으로 다시 찍힌 [(x,y), ...] 리턴
        """
        if len(pts) < 2:
            return pts

        # 끝과 시작 잇기 위해 한 점 더
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        xs.append(pts[0][0])
        ys.append(pts[0][1])

        # 각 세그먼트 길이
        seg_lengths = []
        total_len = 0.0
        for i in range(len(xs) - 1):
            dx = xs[i+1] - xs[i]
            dy = ys[i+1] - ys[i]
            d = (dx**2 + dy**2) ** 0.5
            seg_lengths.append(d)
            total_len += d

        # 총길이만큼 step으로 자르기
        num_out = max(2, int(total_len / step))
        out_pts = []
        cur_seg = 0
        cur_seg_pos = 0.0  # 현재 세그먼트 안에서 얼마나 왔는지
        cur_x = xs[0]
        cur_y = ys[0]
        out_pts.append((cur_x, cur_y))

        for k in range(1, num_out):
            target_dist = k * step
            # 현재 out_pts[-1]에서 target_dist만큼 앞으로 가야 함
            # 세그먼트 따라가면서 위치 찾기
            dist_left = target_dist - ( (k-1) * step )
            # 실제로는 step씩 늘어가니까 dist_left는 step이랑 같지만, 구조 그대로 두자
            move = step
            while move > 0 and cur_seg < len(seg_lengths):
                seg_len = seg_lengths[cur_seg]
                # 세그먼트 남은 길이
                seg_left = seg_len - cur_seg_pos
                if move <= seg_left + 1e-6:
                    # 이 세그먼트 안에서 해결
                    ratio = move / seg_len
                    cur_x = xs[cur_seg] + (xs[cur_seg+1] - xs[cur_seg]) * ((cur_seg_pos + move) / seg_len)
                    cur_y = ys[cur_seg] + (ys[cur_seg+1] - ys[cur_seg]) * ((cur_seg_pos + move) / seg_len)
                    cur_seg_pos += move
                    move = 0
                else:
                    # 이 세그먼트 끝까지 가고 다음 세그먼트로
                    move -= seg_left
                    cur_seg += 1
                    cur_seg_pos = 0.0
                    if cur_seg >= len(seg_lengths):
                        # 안전장치
                        cur_x = xs[-1]
                        cur_y = ys[-1]
                        break
            out_pts.append((cur_x, cur_y))

        return out_pts

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
