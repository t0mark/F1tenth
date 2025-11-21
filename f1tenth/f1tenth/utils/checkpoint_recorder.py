#!/usr/bin/env python3
import atexit
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
from tf2_ros import Buffer, TransformListener

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def _yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    """쿼터니언 구성 요소로부터 yaw(Z축 회전) 계산"""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1 - 2.0 * (qy * qy + qz * qz))


class CheckpointRecorderNode(Node):
    """RViz 클릭으로 체크포인트를 기록하고 CSV로 저장하는 노드"""

    def __init__(self):
        super().__init__('checkpoint_recorder_node')

        # 파라미터
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('output_csv_path', '')
        self.declare_parameter('auto_save_on_add', True)
        self.declare_parameter('publish_topic', '/checkpoint_path')
        self.declare_parameter('clicked_point_topic', '/clicked_point')

        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        csv_path = self.get_parameter('output_csv_path').get_parameter_value().string_value
        self.output_csv_path = os.path.expanduser(csv_path)
        self.auto_save = bool(self.get_parameter('auto_save_on_add').value)
        self.publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        self.clicked_topic = self.get_parameter('clicked_point_topic').get_parameter_value().string_value

        self._checkpoints: List[Tuple[float, float]] = []
        self._lock = threading.Lock()
        self._tty_fd = None
        self._tty_old_settings = None

        # 필요 시 현재 위치를 가져오기 위한 TF 리스너
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 구독
        self.sub_clicked = self.create_subscription(PointStamped, self.clicked_topic, self._clicked_point_cb, 10)

        # 기록된 경로가 유지되도록 Transient Local QoS로 퍼블리셔 생성
        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, self.publish_topic, qos)
        self._publish_path()

        # 'd' 키로 체크포인트 삭제, 's' 키로 시각화 저장하는 백그라운드 키보드 리스너
        self._keyboard_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._keyboard_thread.start()

        self.get_logger().info(
            f'체크포인트 레코더 준비 완료 (map_frame={self.map_frame}, clicked_topic={self.clicked_topic}, '
            f'output="{self.output_csv_path}")'
        )
        self.get_logger().info('=' * 60)
        self.get_logger().info('키보드 명령: [d] 마지막 체크포인트 삭제, [s] 시각화 저장')
        self.get_logger().info('=' * 60)

    def _transform_point_to_map(self, msg: PointStamped) -> Tuple[float, float]:
        """필요한 경우 들어오는 포인트를 맵 프레임으로 변환"""
        if not msg.header.frame_id or msg.header.frame_id == self.map_frame:
            return float(msg.point.x), float(msg.point.y)

        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, msg.header.frame_id, rclpy.time.Time())
        except Exception as exc:
            self.get_logger().warn(
                f'{msg.header.frame_id}에서 {self.map_frame}으로 변환 실패; 포인트 건너뜀: {exc}')
            return None

        trans = tf.transform.translation
        rot = tf.transform.rotation
        yaw = _yaw_from_quaternion(rot.x, rot.y, rot.z, rot.w)

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        x = msg.point.x
        y = msg.point.y
        # 회전 후 평행이동
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
            f'체크포인트 #{count} 기록됨 - x={result[0]:.2f}, y={result[1]:.2f}')
        self._publish_path()

        if self.auto_save:
            self._write_csv()

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
        self.get_logger().info(f'{len(checkpoints)}개 체크포인트 저장됨: {self.output_csv_path}')
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
            self.get_logger().info('삭제할 체크포인트가 없습니다.')
            return
        self.get_logger().info(
            f'체크포인트 #{count + 1} 삭제됨 - x={removed[0]:.2f}, y={removed[1]:.2f}')
        self._publish_path()
        if self.auto_save:
            self._write_csv()

    def _save_visualization(self):
        """체크포인트 시각화를 PNG 파일로 저장"""
        with self._lock:
            checkpoints = list(self._checkpoints)

        if len(checkpoints) < 2:
            self.get_logger().warn('시각화를 위해 최소 2개 이상의 체크포인트가 필요합니다.')
            return

        self.get_logger().info('체크포인트 시각화 저장 중...')

        try:
            points = np.array(checkpoints)

            # 출력 디렉토리 설정
            output_dir = os.path.dirname(self.output_csv_path)
            if output_dir:
                os.makedirs(output_dir, exist_ok=True)

            # 시각화 저장
            png_path = os.path.join(output_dir, 'checkpoints_visualization.png')
            self._save_checkpoint_visualization(points, png_path)

            # 통계
            total_distance = np.sum([np.linalg.norm(points[i] - points[i-1])
                                    for i in range(1, len(points))])
            self.get_logger().info('=' * 50)
            self.get_logger().info(f'체크포인트 개수: {len(points)}개')
            self.get_logger().info(f'전체 경로 길이: {total_distance:.3f}m')
            self.get_logger().info(f'평균 간격: {total_distance/(len(points)-1):.4f}m')
            self.get_logger().info('=' * 50)

        except Exception as exc:
            self.get_logger().error(f'시각화 저장 실패: {exc}')

    def _save_checkpoint_visualization(self, points, output_png):
        """기록된 체크포인트를 시각화하여 PNG로 저장"""
        try:
            fig, ax = plt.subplots(figsize=(10, 8))

            # 체크포인트 플롯
            ax.plot(points[:, 0], points[:, 1], 'b.-',
                   label='Checkpoints', markersize=8, linewidth=2)
            ax.plot(points[0, 0], points[0, 1], 'go',
                   markersize=12, label='Start Point', zorder=5)

            # 체크포인트 번호 추가
            for i, (x, y) in enumerate(points):
                ax.annotate(str(i+1), (x, y), textcoords="offset points",
                           xytext=(5, 5), fontsize=8, alpha=0.7)

            ax.set_xlabel('X (m)', fontsize=12)
            ax.set_ylabel('Y (m)', fontsize=12)
            ax.set_title(f'Recorded Checkpoints ({len(points)} points)', fontsize=14)
            ax.legend(fontsize=10)
            ax.axis('equal')
            ax.grid(True, alpha=0.3)

            plt.tight_layout()
            plt.savefig(output_png, dpi=150, bbox_inches='tight')
            plt.close(fig)
            self.get_logger().info(f'시각화 저장 완료: {output_png}')
        except Exception as exc:
            self.get_logger().error(f'시각화 저장 실패: {exc}')

    def _restore_terminal(self):
        """터미널 설정을 복원"""
        if self._tty_fd is not None and self._tty_old_settings is not None:
            try:
                termios.tcsetattr(self._tty_fd, termios.TCSADRAIN, self._tty_old_settings)
            except:
                pass

    def _keyboard_loop(self):
        stream = sys.stdin if sys.stdin and sys.stdin.isatty() else None
        close_stream = False

        if stream is None:
            try:
                stream = open('/dev/tty')
                close_stream = True
                self.get_logger().info('키보드 입력: /dev/tty 사용')
            except OSError as exc:
                self.get_logger().warn(f"키보드 입력 사용 불가: /dev/tty 열기 실패 ({exc}).")
                return
        else:
            self.get_logger().info('키보드 입력: stdin 사용')

        fd = stream.fileno()
        try:
            old_settings = termios.tcgetattr(fd)
            self._tty_fd = fd
            self._tty_old_settings = old_settings
            # 프로그램 종료 시 터미널 복원 등록
            atexit.register(self._restore_terminal)
        except termios.error as exc:
            self.get_logger().warn(f'키보드 입력 사용 불가: {exc}')
            if close_stream:
                stream.close()
            return

        self.get_logger().info('키보드 리스너 시작됨 - [d] 삭제, [s] 시각화 저장')

        try:
            tty.setcbreak(fd)
            while rclpy.ok():
                readers, _, _ = select.select([stream], [], [], 0.1)
                if stream in readers:
                    ch = stream.read(1)
                    self.get_logger().info(f'키 입력 감지: {repr(ch)}')
                    if ch.lower() == 'd':
                        self.get_logger().info('d 키 감지 - 마지막 체크포인트 삭제')
                        self._remove_last_checkpoint()
                    elif ch.lower() == 's':
                        self.get_logger().info('s 키 감지 - 시각화 저장')
                        self._save_visualization()
        except Exception as exc:
            self.get_logger().warn(f'키보드 리스너 중지됨: {exc}')
        finally:
            self._restore_terminal()
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
