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


def _yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    """Compute yaw (rotation about Z) from quaternion components."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1 - 2.0 * (qy * qy + qz * qz))


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

        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        output_param = self.get_parameter('output_csv_path').get_parameter_value().string_value
        if output_param:
            csv_path = output_param
        else:
            share_dir = get_package_share_directory('f1tenth_path_planner')
            csv_path = os.path.join(share_dir, 'data', 'checkpoints.csv')
        self.output_csv_path = os.path.expanduser(csv_path)
        self.auto_save = bool(self.get_parameter('auto_save_on_add').value)
        self.publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        self.clicked_topic = self.get_parameter('clicked_point_topic').get_parameter_value().string_value

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

        # Background keyboard listener for removing checkpoints with 'y'.
        self._keyboard_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._keyboard_thread.start()

        self.get_logger().info(
            f'Checkpoint recorder ready (map_frame={self.map_frame}, clicked_topic={self.clicked_topic}, '
            f'output="{self.output_csv_path}")'
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

    def _keyboard_loop(self):
        stream = sys.stdin if sys.stdin and sys.stdin.isatty() else None
        close_stream = False

        if stream is None:
            try:
                stream = open('/dev/tty')
                close_stream = True
            except OSError as exc:
                self.get_logger().warn(f"키보드 입력을 사용할 수 없습니다: /dev/tty 열기 실패 ({exc}).")
                return

        fd = stream.fileno()
        try:
            old_settings = termios.tcgetattr(fd)
        except termios.error as exc:
            self.get_logger().warn(f'키보드 입력을 사용할 수 없습니다: {exc}')
            if close_stream:
                stream.close()
            return

        try:
            tty.setcbreak(fd)
            while rclpy.ok():
                readers, _, _ = select.select([stream], [], [], 0.1)
                if stream in readers:
                    ch = stream.read(1)
                    if ch.lower() == 'y':
                        self._remove_last_checkpoint()
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
