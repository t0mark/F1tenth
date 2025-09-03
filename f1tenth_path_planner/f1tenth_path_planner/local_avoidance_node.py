#!/usr/bin/env python3
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener


def transform_point_map_from_base(x_b, y_b, tf):
    # tf: geometry_msgs/TransformStamped (map->base_link)
    tx = tf.transform.translation.x
    ty = tf.transform.translation.y
    q = tf.transform.rotation
    # yaw from quaternion
    # Small helper (tf_transformations not imported here for yaw to save deps)
    # Compute rotation matrix elements for yaw-only extraction
    import numpy as np
    qw, qx, qy, qz = q.w, q.x, q.y, q.z
    # yaw
    yaw = math.atan2(2.0*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
    # rotate and translate
    x_m = tx + (math.cos(yaw)*x_b - math.sin(yaw)*y_b)
    y_m = ty + (math.sin(yaw)*x_b + math.cos(yaw)*y_b)
    return x_m, y_m, yaw


class LocalAvoidanceNode(Node):
    def __init__(self):
        super().__init__('local_avoidance_node')

        # Parameters
        self.declare_parameter('global_path_topic', '/global_path')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('local_horizon', 8.0)  # meters
        self.declare_parameter('path_resolution', 0.2)  # meters
        self.declare_parameter('lateral_offsets', [0.0, 0.4, -0.4])  # meters
        self.declare_parameter('safety_radius', 0.4)  # meters
        self.declare_parameter('output_topic', '/local_path')
        self.declare_parameter('base_frame', 'ego_racecar/base_link')
        self.declare_parameter('map_frame', 'map')

        # Read params
        self.global_path_topic = self.get_parameter('global_path_topic').get_parameter_value().string_value
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.local_horizon = float(self.get_parameter('local_horizon').value)
        self.ds = float(self.get_parameter('path_resolution').value)
        self.lateral_offsets = [float(v) for v in self.get_parameter('lateral_offsets').value]
        self.safety_radius = float(self.get_parameter('safety_radius').value)
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.global_path: Path = None
        self.sub_path = self.create_subscription(Path, self.global_path_topic, self._path_cb, 10)
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_scan = self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, qos)

        # Publisher
        self.pub = self.create_publisher(Path, self.output_topic, 10)

        # State
        self.latest_scan: LaserScan = None

        # Timer to publish at fixed rate if data is available
        self.timer = self.create_timer(0.05, self._on_timer)  # 20 Hz

    def _path_cb(self, msg: Path):
        self.global_path = msg

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def _get_tf(self):
        try:
            return self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None

    def _closest_path_index(self, x, y) -> int:
        if self.global_path is None or not self.global_path.poses:
            return -1
        best_i = -1
        best_d2 = 1e18
        for i, ps in enumerate(self.global_path.poses):
            dx = ps.pose.position.x - x
            dy = ps.pose.position.y - y
            d2 = dx*dx + dy*dy
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
        return best_i

    def _is_point_safe(self, x_b, y_b, scan: LaserScan) -> bool:
        r = math.hypot(x_b, y_b)
        if r < 1e-3:
            return True
        theta = math.atan2(y_b, x_b)
        # clamp to scan angle range
        if theta < scan.angle_min or theta > scan.angle_max:
            return True  # outside FOV, treat as safe to avoid over-constraining
        idx = int(round((theta - scan.angle_min) / scan.angle_increment))
        if idx < 0 or idx >= len(scan.ranges):
            return True
        rng = scan.ranges[idx]
        if math.isnan(rng) or math.isinf(rng):
            return True
        return (rng - r) > self.safety_radius

    def _build_local_path(self, tf_map_base, scan: LaserScan) -> List[Tuple[float, float]]:
        local_pts = []  # in base_link frame
        steps = max(2, int(self.local_horizon / self.ds))
        x = 0.0
        for k in range(steps):
            x = k * self.ds
            # choose lateral offset with best clearance
            best_off = None
            for off in self.lateral_offsets:
                if self._is_point_safe(x, off, scan):
                    best_off = off
                    break
            if best_off is None:
                # if none safe, keep center but shorten horizon
                break
            local_pts.append((x, best_off))
        # transform to map frame
        out = []
        for (xb, yb) in local_pts:
            xm, ym, _ = transform_point_map_from_base(xb, yb, tf_map_base)
            out.append((xm, ym))
        return out

    def _on_timer(self):
        if self.global_path is None or self.latest_scan is None:
            return
        tfmb = self._get_tf()
        if tfmb is None:
            return

        # Find closest point to current pose to stamp and orient local path roughly along global path
        cx = tfmb.transform.translation.x
        cy = tfmb.transform.translation.y
        idx = self._closest_path_index(cx, cy)
        if idx < 0:
            return

        # Build and publish Path
        pts_map = self._build_local_path(tfmb, self.latest_scan)
        if not pts_map:
            return
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.map_frame
        # Estimate yaw from first to last point for pose orientation
        yaw = 0.0
        if len(pts_map) >= 2:
            dx = pts_map[-1][0] - pts_map[0][0]
            dy = pts_map[-1][1] - pts_map[0][1]
            yaw = math.atan2(dy, dx)
        qz = math.sin(yaw/2.0)
        qw = math.cos(yaw/2.0)
        for (x, y) in pts_map:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            path.poses.append(ps)
        self.pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = LocalAvoidanceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
