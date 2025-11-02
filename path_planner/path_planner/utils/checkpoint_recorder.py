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
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

import numpy as np
import pandas as pd
from scipy.interpolate import CubicSpline
from scipy.ndimage import gaussian_filter1d
import matplotlib
matplotlib.use('Agg')  # GUI 없이 이미지 저장
import matplotlib.pyplot as plt


def _yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    """Compute yaw (rotation about Z) from quaternion components."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1 - 2.0 * (qy * qy + qz * qz))


def _calculate_smooth_speed_profile(kappas, max_speed=4.0, min_speed=1.0, sigma=5):
    """곡률 기반으로 속도 계산 후 가우시안 필터로 스무딩"""
    if len(kappas) == 0 or np.all(kappas == 0):
        return np.full(len(kappas), max_speed)

    max_kappa = np.max(np.abs(kappas))
    if max_kappa == 0:
        return np.full(len(kappas), max_speed)

    speeds = []
    for kappa in kappas:
        normalized_kappa = np.abs(kappa) / max_kappa
        speed = max_speed - (normalized_kappa * (max_speed - min_speed))
        speeds.append(max(min_speed, speed))

    smoothed_speeds = gaussian_filter1d(speeds, sigma=sigma)
    return np.array(smoothed_speeds)


def _fit_path(points):
    """(x, y) 점에 3차 스플라인을 맞추고, yaw, 곡률, 누적 거리, 속도를 계산"""
    x = points[:, 0]
    y = points[:, 1]
    t = np.linspace(0, 1, len(x))
    cs_x = CubicSpline(t, x, bc_type='clamped')
    cs_y = CubicSpline(t, y, bc_type='clamped')
    # 입력 포인트 개수 유지 (500개로 고정하지 않음)
    t_new = np.linspace(0, 1, len(x))
    x_new = cs_x(t_new)
    y_new = cs_y(t_new)

    dx = np.gradient(x_new)
    dy = np.gradient(y_new)
    yaw_new = np.arctan2(dy, dx)

    kappa = np.zeros_like(x_new)
    for i in range(1, len(x_new) - 1):
        dx = x_new[i + 1] - x_new[i - 1]
        dy = y_new[i + 1] - y_new[i - 1]
        d2x = (x_new[i + 1] - 2 * x_new[i] + x_new[i - 1])
        d2y = (y_new[i + 1] - 2 * y_new[i] + y_new[i - 1])
        denominator = (dx ** 2 + dy ** 2) ** (3 / 2)
        if denominator > 1e-6:
            kappa[i] = (dx * d2y - dy * d2x) / denominator
    kappa[0] = kappa[1]
    kappa[-1] = kappa[-2]

    s = np.zeros_like(x_new)
    for i in range(1, len(x_new)):
        s[i] = s[i - 1] + np.sqrt((x_new[i] - x_new[i - 1]) ** 2 + (y_new[i] - y_new[i - 1]) ** 2)

    vx = _calculate_smooth_speed_profile(kappa)

    return np.column_stack((x_new, y_new, yaw_new, s, kappa, vx))


class CheckpointRecorderNode(Node):
    """Node that records checkpoints from the current TF pose and saves them to CSV."""

    def __init__(self):
        super().__init__('checkpoint_recorder_node')

        # Parameters
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('output_csv_path', '')
        self.declare_parameter('auto_save_on_add', True)
        self.declare_parameter('publish_topic', '/checkpoint_path')
        self.declare_parameter('clicked_point_topic', '/clicked_point')
        self.declare_parameter('point_spacing', 0.25)

        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        output_param = self.get_parameter('output_csv_path').get_parameter_value().string_value
        if output_param:
            csv_path = output_param
        else:
            # 소스 디렉토리의 path_planner/data에 저장
            # 현재 파일: path_planner/path_planner/utils/checkpoint_recorder.py
            # 목표: path_planner/data/checkpoints.csv
            current_file = os.path.abspath(__file__)
            package_root = os.path.dirname(os.path.dirname(os.path.dirname(current_file)))
            csv_path = os.path.join(package_root, 'data', 'checkpoints.csv')
        self.output_csv_path = os.path.expanduser(csv_path)
        self.auto_save = bool(self.get_parameter('auto_save_on_add').value)
        self.publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        self.clicked_topic = self.get_parameter('clicked_point_topic').get_parameter_value().string_value
        self.point_spacing = float(self.get_parameter('point_spacing').value)

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

        # Background keyboard listener for removing checkpoints with 'y' and generating uniform distribution with 'q'.
        self._keyboard_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._keyboard_thread.start()

        self.get_logger().info(
            f'Checkpoint recorder ready (map_frame={self.map_frame}, clicked_topic={self.clicked_topic}, '
            f'output="{self.output_csv_path}", point_spacing={self.point_spacing}m)'
        )
        self.get_logger().info(
            '키보드 명령: [y] 마지막 체크포인트 삭제, [q] 균등 분포 생성 및 저장'
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

    def _generate_uniform_distribution(self):
        """체크포인트를 균등 분포로 재배치하고 CSV 및 시각화 저장"""
        with self._lock:
            checkpoints = list(self._checkpoints)

        if len(checkpoints) < 3:
            self.get_logger().warn('균등 분포 생성을 위해 최소 3개 이상의 체크포인트가 필요합니다.')
            return

        self.get_logger().info('균등 분포 포인트 생성 중...')

        try:
            # 1. 폐루프 완성
            points = np.array(checkpoints)
            original_num_points = len(points)
            dist_start_end = np.linalg.norm(points[0] - points[-1])

            if dist_start_end > 0.1:
                points = np.vstack([points, points[0:1]])
                self.get_logger().info(f'폐루프 완성: 시작점-끝점 연결 ({dist_start_end:.3f}m)')

            # 2. 누적 거리 계산
            distances = np.zeros(len(points))
            for i in range(1, len(points)):
                distances[i] = distances[i-1] + np.linalg.norm(points[i] - points[i-1])
            total_distance = distances[-1]

            # 3. Cubic spline 생성
            cs_x = CubicSpline(distances, points[:, 0], bc_type='periodic')
            cs_y = CubicSpline(distances, points[:, 1], bc_type='periodic')

            # 4. 균등 포인트 생성
            num_points = max(int(np.round(total_distance / self.point_spacing)), original_num_points)
            uniform_distances = np.linspace(0, total_distance, num_points, endpoint=True)
            uniform_points = np.array([cs_x(uniform_distances), cs_y(uniform_distances)]).T
            uniform_points = uniform_points[:-1]  # 마지막 중복 포인트 제거

            # 5. CSV 저장
            output_dir = os.path.dirname(self.output_csv_path)
            if output_dir:
                os.makedirs(output_dir, exist_ok=True)

            df = pd.DataFrame(uniform_points, columns=['x', 'y'])
            df.to_csv(self.output_csv_path, index=False)
            self.get_logger().info(f'✓ 균등 분포 포인트 저장: {self.output_csv_path}')

            # 6. 시각화 저장
            png_path = self.output_csv_path.replace('.csv', '_visualization.png')
            self._save_visualization(points, cs_x, cs_y, total_distance, original_num_points,
                                   uniform_points, png_path)

            # 7. 통계 출력
            self.get_logger().info('=' * 50)
            self.get_logger().info(f'원본 포인트 수: {original_num_points}개')
            self.get_logger().info(f'재분배된 포인트 수: {len(uniform_points)}개')
            self.get_logger().info(f'전체 경로 길이: {total_distance:.3f}m')
            self.get_logger().info(f'지정된 해상도: {self.point_spacing}m')
            self.get_logger().info(f'포인트 간 실제 간격: {total_distance/len(uniform_points):.4f}m')
            self.get_logger().info('=' * 50)

            # 8. fitted_waypoints.csv 생성
            self._generate_fitted_waypoints(uniform_points)

        except Exception as exc:
            self.get_logger().error(f'균등 분포 생성 실패: {exc}')

    def _save_visualization(self, points, cs_x, cs_y, total_distance, original_num_points,
                           uniform_points, output_png):
        """원본, 균등 분배(원본 수), 균등 분배(해상도 지정) 비교 시각화"""
        try:
            fig, axes = plt.subplots(1, 3, figsize=(15, 5))

            # (1) 원본 포인트 분포
            axes[0].plot(points[:, 0], points[:, 1], 'b.-', label='원본 (불균등)', markersize=6)
            axes[0].plot(points[0, 0], points[0, 1], 'go', markersize=10, label='시작점')
            axes[0].set_xlabel('X')
            axes[0].set_ylabel('Y')
            axes[0].set_title(f'원본 포인트 분포 ({original_num_points}개)')
            axes[0].legend()
            axes[0].axis('equal')
            axes[0].grid(True, alpha=0.3)

            # (2) 균등 분배 (원본과 동일 수)
            uniform_original_dist = np.linspace(0, total_distance, original_num_points, endpoint=False)
            uniform_original = np.array([cs_x(uniform_original_dist), cs_y(uniform_original_dist)]).T
            axes[1].plot(uniform_original[:, 0], uniform_original[:, 1], 'orange', marker='.',
                        linestyle='-', label='균등 분배 (원본 수)', markersize=6)
            axes[1].plot(uniform_original[0, 0], uniform_original[0, 1], 'go', markersize=10, label='시작점')
            axes[1].set_xlabel('X')
            axes[1].set_ylabel('Y')
            axes[1].set_title(f'균등 분배 ({original_num_points}개)')
            axes[1].legend()
            axes[1].axis('equal')
            axes[1].grid(True, alpha=0.3)

            # (3) 균등 분배 (해상도 지정)
            axes[2].plot(uniform_points[:, 0], uniform_points[:, 1], 'r.-',
                        label=f'균등 분배 ({self.point_spacing}m)', markersize=3)
            axes[2].plot(uniform_points[0, 0], uniform_points[0, 1], 'go', markersize=10, label='시작점')
            axes[2].plot([uniform_points[-1, 0], uniform_points[0, 0]],
                        [uniform_points[-1, 1], uniform_points[0, 1]], 'g--', alpha=0.5, linewidth=2)
            axes[2].set_xlabel('X')
            axes[2].set_ylabel('Y')
            axes[2].set_title(f'균등 분배 ({len(uniform_points)}개, 해상도: {self.point_spacing}m)')
            axes[2].legend()
            axes[2].axis('equal')
            axes[2].grid(True, alpha=0.3)

            plt.tight_layout()
            plt.savefig(output_png, dpi=150, bbox_inches='tight')
            plt.close(fig)
            self.get_logger().info(f'✓ 시각화 저장: {output_png}')
        except Exception as exc:
            self.get_logger().error(f'시각화 저장 실패: {exc}')

    def _generate_fitted_waypoints(self, uniform_points):
        """균등 분포 포인트로부터 fitted_waypoints.csv 생성"""
        try:
            self.get_logger().info('Fitted waypoints 생성 중...')

            # 폐루프 완성 (첫 번째 점을 끝에 추가)
            waypoints_closed = np.vstack([uniform_points, uniform_points[0:1]])

            # 곡률, 속도 등을 포함한 fitted waypoints 생성
            fitted_waypoints = _fit_path(waypoints_closed)

            output_dir = os.path.dirname(self.output_csv_path)

            # 1. fitted_waypoints.csv 저장 (쉼표 구분, checkpoints.csv와 동일 개수)
            fitted_csv_path = os.path.join(output_dir, 'fitted_waypoints.csv')
            header_comma = 'x_ref_m,y_ref_m,psi_racetraj_rad,s_racetraj_m,kappa_racetraj_radpm,vx_racetraj_mps'
            np.savetxt(fitted_csv_path, fitted_waypoints, delimiter=',',
                      header=header_comma, comments='', fmt='%.4f')
            self.get_logger().info(f'✓ Fitted waypoints 저장: {fitted_csv_path} ({len(fitted_waypoints)}개)')

            # 2. final_waypoints.csv 저장 (세미콜론 구분, 기존 노드 호환용)
            final_csv_path = os.path.join(output_dir, 'final_waypoints.csv')
            header_semicolon = 'x_ref_m;y_ref_m;psi_racetraj_rad;s_racetraj_m;kappa_racetraj_radpm;vx_racetraj_mps'
            np.savetxt(final_csv_path, fitted_waypoints, delimiter=';',
                      header=header_semicolon, comments='', fmt='%.4f')
            self.get_logger().info(f'✓ Final waypoints 저장: {final_csv_path} ({len(fitted_waypoints)}개)')

            # 3. 속도 히트맵 시각화 저장
            heatmap_png_path = os.path.join(output_dir, 'fitted_waypoints_speed_heatmap.png')
            self._save_speed_heatmap(fitted_waypoints, heatmap_png_path)

        except Exception as exc:
            self.get_logger().error(f'Fitted waypoints 생성 실패: {exc}')

    def _save_speed_heatmap(self, fitted_waypoints, output_png):
        """속도 히트맵 시각화 저장"""
        try:
            plt.figure(figsize=(10, 8))

            # 속도 값 추출 (vx_racetraj_mps 컬럼)
            speeds = fitted_waypoints[:, 5]
            min_speed = np.min(speeds)
            max_speed = np.max(speeds)

            # 속도를 [0, 1] 범위로 정규화
            norm_speed = (speeds - min_speed) / (max_speed - min_speed) if (max_speed - min_speed) > 0 else np.zeros_like(speeds)

            # 컬러맵 생성
            colormap = plt.get_cmap('viridis')

            # 각 세그먼트를 해당 색상으로 플로팅
            for i in range(len(fitted_waypoints) - 1):
                plt.plot(fitted_waypoints[i:i+2, 0], fitted_waypoints[i:i+2, 1],
                        color=colormap(norm_speed[i]), linewidth=2)

            # 웨이포인트를 산점도로 표시하고 컬러바 추가
            plt.scatter(fitted_waypoints[:, 0], fitted_waypoints[:, 1],
                       c=speeds, cmap='viridis', s=20, zorder=2)
            plt.colorbar(label='Speed (m/s)')
            plt.title('Fitted Waypoints Speed Heatmap')
            plt.xlabel('X (m)')
            plt.ylabel('Y (m)')
            plt.grid(True)
            plt.axis('equal')

            plt.savefig(output_png, dpi=150, bbox_inches='tight')
            plt.close()
            self.get_logger().info(f'✓ 속도 히트맵 저장: {output_png}')
        except Exception as exc:
            self.get_logger().error(f'속도 히트맵 저장 실패: {exc}')

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

        self.get_logger().info('키보드 리스너 시작됨 - [y] 삭제, [q] 균등분포 저장')

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
                        self.get_logger().info('q 키 감지 - 균등 분포 생성 시작')
                        self._generate_uniform_distribution()
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
