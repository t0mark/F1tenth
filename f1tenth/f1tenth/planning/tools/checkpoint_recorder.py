#!/usr/bin/env python3
import csv
import math
import os
import select
import sys
import threading
import termios
import tty
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

import numpy as np
import pandas as pd
from scipy.interpolate import CubicSpline
import matplotlib
matplotlib.use('Agg')  # GUI 없이 이미지 저장
import matplotlib.pyplot as plt


def _yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    """Compute yaw (rotation about Z) from quaternion components."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1 - 2.0 * (qy * qy + qz * qz))


def _optimize_racing_line(points, target_spacing, wheel_base, max_steering_angle):
    """
    차량의 기구학적 특성을 고려하여 최적 레이싱 라인 생성

    Parameters:
    -----------
    points : np.ndarray
        원본 체크포인트 (x, y)
    target_spacing : float
        목표 포인트 간격 (m)
    wheel_base : float
        차축 거리 (m)
    max_steering_angle : float
        최대 조향각 (rad)

    Returns:
    --------
    optimized_points : np.ndarray
        최적화된 레이싱 라인 포인트 (x, y)
    """
    # 1. 폐루프 완성
    if np.linalg.norm(points[0] - points[-1]) > 0.1:
        points = np.vstack([points, points[0:1]])

    # 2. 누적 거리 계산
    distances = np.zeros(len(points))
    for i in range(1, len(points)):
        distances[i] = distances[i-1] + np.linalg.norm(points[i] - points[i-1])
    total_distance = distances[-1]

    # 3. Cubic spline 생성 (폐루프)
    cs_x = CubicSpline(distances, points[:, 0], bc_type='periodic')
    cs_y = CubicSpline(distances, points[:, 1], bc_type='periodic')

    # 4. 균등 간격으로 포인트 재배치
    num_points = max(int(np.round(total_distance / target_spacing)), len(points))
    uniform_distances = np.linspace(0, total_distance, num_points, endpoint=False)

    optimized_x = cs_x(uniform_distances)
    optimized_y = cs_y(uniform_distances)
    optimized_points = np.column_stack((optimized_x, optimized_y))

    return optimized_points


class CheckpointRecorderNode(Node):
    """Node that records checkpoints from RViz clicks and saves them to CSV."""

    def __init__(self):
        super().__init__('checkpoint_recorder_node')

        # Parameters
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('output_csv_path', '')
        self.declare_parameter('auto_save_on_add', True)
        self.declare_parameter('publish_topic', '/checkpoint_path')
        self.declare_parameter('clicked_point_topic', '/clicked_point')
        self.declare_parameter('point_spacing', 0.25)

        # Vehicle parameters for racing line optimization
        self.declare_parameter('wheel_base', 0.33)
        self.declare_parameter('max_steering_angle', 0.42)

        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        output_param = self.get_parameter('output_csv_path').get_parameter_value().string_value
        if output_param:
            csv_path = output_param
        else:
            # 소스 디렉토리의 path_planner/data에 저장
            # 현재 파일: path_planner/path_planner/utils/checkpoint_recorder.py
            # 목표: path_planner/data/pre_checkpoints.csv
            current_file = os.path.abspath(__file__)
            package_root = os.path.dirname(os.path.dirname(os.path.dirname(current_file)))
            csv_path = os.path.join(package_root, 'data', 'pre_checkpoints.csv')
        self.output_csv_path = os.path.expanduser(csv_path)
        self.auto_save = bool(self.get_parameter('auto_save_on_add').value)
        self.publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        self.clicked_topic = self.get_parameter('clicked_point_topic').get_parameter_value().string_value
        self.point_spacing = float(self.get_parameter('point_spacing').value)

        # Vehicle parameters
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.max_steering_angle = float(self.get_parameter('max_steering_angle').value)

        self._checkpoints: List[Tuple[float, float]] = []
        self._lock = threading.Lock()

        # TF listener to retrieve current position on demand.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriptions / Services
        self.sub_clicked = self.create_subscription(PointStamped, self.clicked_topic, self._clicked_point_cb, 10)
        self.save_srv = self.create_service(Trigger, 'save_checkpoints', self._handle_save_checkpoints)
        self.clear_srv = self.create_service(Trigger, 'clear_checkpoints', self._handle_clear_checkpoints)

        # Publisher with transient local QoS so recorded path stays latched.
        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, self.publish_topic, qos)
        self._publish_path()

        # Background keyboard listener for removing checkpoints with 'y' and generating racing line with 'q'.
        self._keyboard_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._keyboard_thread.start()

        # 차량의 기구학적 한계 계산
        min_turning_radius = self.wheel_base / np.tan(self.max_steering_angle)
        max_kappa = 1.0 / min_turning_radius

        self.get_logger().info(
            f'Checkpoint recorder ready (map_frame={self.map_frame}, clicked_topic={self.clicked_topic}, '
            f'output="{self.output_csv_path}", point_spacing={self.point_spacing}m)'
        )
        self.get_logger().info('=' * 60)
        self.get_logger().info('차량 파라미터:')
        self.get_logger().info(f'  - 차축 거리: {self.wheel_base} m')
        self.get_logger().info(f'  - 최대 조향각: {self.max_steering_angle} rad ({np.degrees(self.max_steering_angle):.2f}°)')
        self.get_logger().info(f'  - 최소 회전 반경: {min_turning_radius:.3f} m')
        self.get_logger().info(f'  - 최대 곡률: {max_kappa:.3f} rad/m')
        self.get_logger().info('=' * 60)
        self.get_logger().info(
            '키보드 명령: [y] 마지막 체크포인트 삭제, [q] 레이싱 라인 생성 및 저장'
        )

    def _transform_point_to_map(self, msg: PointStamped) -> Tuple[float, float]:
        """Transform incoming point into map frame if required."""
        if not msg.header.frame_id or msg.header.frame_id == self.map_frame:
            return float(msg.point.x), float(msg.point.y)

        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, msg.header.frame_id, rclpy.time.Time())
        except Exception as exc:
            self.get_logger().warn(
                f'No transform from {msg.header.frame_id} to {self.map_frame}; skipping point: {exc}')
            return None

        trans = tf.transform.translation
        rot = tf.transform.rotation
        yaw = _yaw_from_quaternion(rot.x, rot.y, rot.z, rot.w)

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        x = msg.point.x
        y = msg.point.y
        # Rotate then translate
        map_x = trans.x + cos_yaw * x - sin_yaw * y
        map_y = trans.y + sin_yaw * x + cos_yaw * y
        return float(map_x), float(map_y)

    def _clicked_point_cb(self, msg: PointStamped):
        result = self._transform_point_to_map(msg)
        if result is None:
            return

        with self._lock:
            self._checkpoints.append(result)
            count = len(self._checkpoints)
        self.get_logger().info(
            f'Checkpoint #{count} recorded at x={result[0]:.2f}, y={result[1]:.2f}')
        self._publish_path()

        if self.auto_save:
            self._write_csv()

    def _handle_save_checkpoints(self, request, response):
        with self._lock:
            has_checkpoints = bool(self._checkpoints)
        if not has_checkpoints:
            response.success = False
            response.message = 'No checkpoints to save.'
            return response

        try:
            path = self._write_csv()
        except Exception as exc:
            response.success = False
            response.message = f'Failed to save CSV: {exc}'
            return response

        with self._lock:
            count = len(self._checkpoints)
        response.success = True
        response.message = f'Saved {count} checkpoints to {path}'
        return response

    def _handle_clear_checkpoints(self, request, response):
        with self._lock:
            count = len(self._checkpoints)
            self._checkpoints.clear()
        self._publish_path()
        # Rewrite CSV with header only.
        self._write_csv()
        response.success = True
        response.message = f'Cleared {count} checkpoints.'
        return response

    def _publish_path(self):
        with self._lock:
            checkpoints = list(self._checkpoints)
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        for idx, (x, y) in enumerate(checkpoints):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            yaw = self._estimate_yaw(checkpoints, idx)
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    def _estimate_yaw(self, checkpoints: List[Tuple[float, float]], index: int) -> float:
        if len(checkpoints) <= 1:
            return 0.0
        if index < len(checkpoints) - 1:
            next_x, next_y = checkpoints[index + 1]
            curr_x, curr_y = checkpoints[index]
        else:
            curr_x, curr_y = checkpoints[index]
            prev_x, prev_y = checkpoints[index - 1]
            next_x, next_y = curr_x, curr_y
            curr_x, curr_y = prev_x, prev_y
        dx = next_x - curr_x
        dy = next_y - curr_y
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return 0.0
        return math.atan2(dy, dx)

    def _write_csv(self) -> str:
        with self._lock:
            checkpoints = list(self._checkpoints)
        directory = os.path.dirname(self.output_csv_path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(self.output_csv_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['x', 'y'])
            writer.writerows(checkpoints)
        self.get_logger().info(f'Saved {len(checkpoints)} checkpoints to {self.output_csv_path}')
        return self.output_csv_path

    def _remove_last_checkpoint(self):
        with self._lock:
            if not self._checkpoints:
                count = 0
                removed = None
            else:
                removed = self._checkpoints.pop()
                count = len(self._checkpoints)
        if removed is None:
            self.get_logger().info('No checkpoints to remove.')
            return
        self.get_logger().info(
            f'Removed checkpoint #{count + 1} at x={removed[0]:.2f}, y={removed[1]:.2f}')
        self._publish_path()
        if self.auto_save:
            self._write_csv()

    def _generate_racing_line(self):
        """최적 레이싱 라인 생성 및 CSV와 시각화 저장"""
        with self._lock:
            checkpoints = list(self._checkpoints)

        if len(checkpoints) < 3:
            self.get_logger().warn('레이싱 라인 생성을 위해 최소 3개 이상의 체크포인트가 필요합니다.')
            return

        self.get_logger().info('최적 레이싱 라인 생성 중...')

        try:
            points = np.array(checkpoints)
            original_num_points = len(points)

            # 최적 레이싱 라인 생성
            optimized_points = _optimize_racing_line(
                points,
                target_spacing=self.point_spacing,
                wheel_base=self.wheel_base,
                max_steering_angle=self.max_steering_angle
            )

            # 출력 디렉토리 설정
            output_dir = os.path.dirname(self.output_csv_path)
            if output_dir:
                os.makedirs(output_dir, exist_ok=True)

            # checkpoints.csv 저장
            checkpoints_path = os.path.join(output_dir, 'checkpoints.csv')
            df = pd.DataFrame(optimized_points, columns=['x', 'y'])
            df.to_csv(checkpoints_path, index=False)
            self.get_logger().info(f'✓ 최적화된 레이싱 라인 저장: {checkpoints_path}')

            # 시각화 저장
            png_path = os.path.join(output_dir, 'checkpoints_visualization.png')
            self._save_racing_line_visualization(points, optimized_points, png_path)

            # 통계 출력
            total_distance = np.sum([np.linalg.norm(optimized_points[i] - optimized_points[i-1])
                                    for i in range(1, len(optimized_points))])
            self.get_logger().info('=' * 50)
            self.get_logger().info(f'원본 포인트 수: {original_num_points}개')
            self.get_logger().info(f'최적화된 포인트 수: {len(optimized_points)}개')
            self.get_logger().info(f'전체 경로 길이: {total_distance:.3f}m')
            self.get_logger().info(f'포인트 간 평균 간격: {total_distance/len(optimized_points):.4f}m')
            self.get_logger().info('=' * 50)

        except Exception as exc:
            self.get_logger().error(f'레이싱 라인 생성 실패: {exc}')

    def _save_racing_line_visualization(self, original_points, optimized_points, output_png):
        """원본 포인트와 최적화된 레이싱 라인 비교 시각화"""
        try:
            fig, axes = plt.subplots(1, 2, figsize=(16, 7))

            # (1) 원본 포인트 분포
            axes[0].plot(original_points[:, 0], original_points[:, 1], 'b.-',
                        label='원본 체크포인트', markersize=8, linewidth=2)
            axes[0].plot(original_points[0, 0], original_points[0, 1], 'go',
                        markersize=12, label='시작점', zorder=5)
            axes[0].set_xlabel('X (m)', fontsize=12)
            axes[0].set_ylabel('Y (m)', fontsize=12)
            axes[0].set_title(f'원본 체크포인트 ({len(original_points)}개)', fontsize=14)
            axes[0].legend(fontsize=10)
            axes[0].axis('equal')
            axes[0].grid(True, alpha=0.3)

            # (2) 최적화된 레이싱 라인
            axes[1].plot(optimized_points[:, 0], optimized_points[:, 1], 'r.-',
                        label='최적화된 레이싱 라인', markersize=3, linewidth=2)
            axes[1].plot(optimized_points[0, 0], optimized_points[0, 1], 'go',
                        markersize=12, label='시작점', zorder=5)
            # 폐루프 표시
            axes[1].plot([optimized_points[-1, 0], optimized_points[0, 0]],
                        [optimized_points[-1, 1], optimized_points[0, 1]],
                        'g--', alpha=0.5, linewidth=2, label='폐루프 연결')
            axes[1].set_xlabel('X (m)', fontsize=12)
            axes[1].set_ylabel('Y (m)', fontsize=12)
            axes[1].set_title(f'최적화된 레이싱 라인 ({len(optimized_points)}개)', fontsize=14)
            axes[1].legend(fontsize=10)
            axes[1].axis('equal')
            axes[1].grid(True, alpha=0.3)

            plt.tight_layout()
            plt.savefig(output_png, dpi=150, bbox_inches='tight')
            plt.close(fig)
            self.get_logger().info(f'✓ 시각화 저장: {output_png}')
        except Exception as exc:
            self.get_logger().error(f'시각화 저장 실패: {exc}')

    def _keyboard_loop(self):
        stream = sys.stdin if sys.stdin and sys.stdin.isatty() else None
        close_stream = False

        if stream is None:
            try:
                stream = open('/dev/tty')
                close_stream = True
                self.get_logger().info('키보드 입력: /dev/tty 사용')
            except OSError as exc:
                self.get_logger().warn(f"키보드 입력을 사용할 수 없습니다: /dev/tty 열기 실패 ({exc}).")
                return
        else:
            self.get_logger().info('키보드 입력: stdin 사용')

        fd = stream.fileno()
        try:
            old_settings = termios.tcgetattr(fd)
        except termios.error as exc:
            self.get_logger().warn(f'키보드 입력을 사용할 수 없습니다: {exc}')
            if close_stream:
                stream.close()
            return

        self.get_logger().info('키보드 리스너 시작됨 - [y] 삭제, [q] 레이싱 라인 생성')

        try:
            tty.setcbreak(fd)
            while rclpy.ok():
                readers, _, _ = select.select([stream], [], [], 0.1)
                if stream in readers:
                    ch = stream.read(1)
                    self.get_logger().info(f'키 입력 감지: {repr(ch)}')
                    if ch.lower() == 'y':
                        self.get_logger().info('y 키 감지 - 마지막 체크포인트 삭제')
                        self._remove_last_checkpoint()
                    elif ch.lower() == 'q':
                        self.get_logger().info('q 키 감지 - 레이싱 라인 생성 시작')
                        self._generate_racing_line()
        except Exception as exc:
            self.get_logger().warn(f'Keyboard listener stopped: {exc}')
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            if close_stream:
                stream.close()


def main(args=None):
    rclpy.init(args=args)
    node = CheckpointRecorderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
