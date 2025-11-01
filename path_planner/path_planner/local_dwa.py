#!/usr/bin/env python3
"""
Local DWA Planner
=================
단일 노드에서 로컬 비용맵 생성과 Dynamic Window Approach 기반 속도 명령 및 로컬 경로를 계산한다.
"""

import math
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def transform_point_base_from_map(x_m: float, y_m: float, tf_map_base) -> Tuple[float, float]:
    tx = tf_map_base.transform.translation.x
    ty = tf_map_base.transform.translation.y
    q = tf_map_base.transform.rotation
    qw, qx, qy, qz = q.w, q.x, q.y, q.z
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))

    dx = x_m - tx
    dy = y_m - ty

    cos_yaw = math.cos(-yaw)
    sin_yaw = math.sin(-yaw)
    x_b = cos_yaw * dx - sin_yaw * dy
    y_b = sin_yaw * dx + cos_yaw * dy
    return x_b, y_b


def transform_point_map_from_base(x_b: float, y_b: float, tf_map_base) -> Tuple[float, float]:
    tx = tf_map_base.transform.translation.x
    ty = tf_map_base.transform.translation.y
    q = tf_map_base.transform.rotation
    qw, qx, qy, qz = q.w, q.x, q.y, q.z
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))

    x_m = tx + (math.cos(yaw) * x_b - math.sin(yaw) * y_b)
    y_m = ty + (math.sin(yaw) * x_b + math.cos(yaw) * y_b)
    return x_m, y_m


def apply_transform(x: float, y: float, transform) -> Tuple[float, float]:
    tx = transform.translation.x
    ty = transform.translation.y
    q = transform.rotation
    qw, qx, qy, qz = q.w, q.x, q.y, q.z
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    x_out = tx + (math.cos(yaw) * x - math.sin(yaw) * y)
    y_out = ty + (math.sin(yaw) * x + math.cos(yaw) * y)
    return x_out, y_out


class LocalDwaNode(Node):
    def __init__(self):
        super().__init__('local_dwa_node')

        # Topics / frames
        self.declare_parameter('global_path_topic', '/global_path')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('local_path_topic', '/local_path')
        self.declare_parameter('costmap_topic', '/local_costmap')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('publish_cmd_vel', False)
        self.declare_parameter('cmd_topic', '/cmd_vel')

        # Costmap parameters
        self.declare_parameter('costmap_size', 12.0)  # meters (square window)
        self.declare_parameter('costmap_resolution', 0.1)  # meters/cell
        self.declare_parameter('max_obstacle_range', 10.0)
        self.declare_parameter('inflation_radius', 0.5)

        # Robot footprint
        self.declare_parameter('robot_radius', 0.4)

        # DWA sampling parameters
        self.declare_parameter('sim_time', 2.0)
        self.declare_parameter('sim_dt', 0.1)
        self.declare_parameter('vx_samples', 6)
        self.declare_parameter('omega_samples', 11)

        # Dynamic window limits
        self.declare_parameter('max_speed', 4.0)
        self.declare_parameter('min_speed', 0.0)
        self.declare_parameter('max_accel', 4.0)
        self.declare_parameter('max_ang_speed', 4.0)
        self.declare_parameter('max_ang_accel', 6.0)

        # Cost weights
        self.declare_parameter('heading_weight', 1.0)
        self.declare_parameter('velocity_weight', 1.0)
        self.declare_parameter('clearance_weight', 1.0)

        # Goal tracking
        self.declare_parameter('goal_lookahead_dist', 3.0)

        # Read parameters
        self.global_path_topic = self.get_parameter('global_path_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.local_path_topic = self.get_parameter('local_path_topic').get_parameter_value().string_value
        self.costmap_topic = self.get_parameter('costmap_topic').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.publish_cmd_vel = bool(self.get_parameter('publish_cmd_vel').value)
        self.cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value

        self.costmap_size = float(self.get_parameter('costmap_size').value)
        self.resolution = float(self.get_parameter('costmap_resolution').value)
        self.max_obstacle_range = float(self.get_parameter('max_obstacle_range').value)
        self.inflation_radius = float(self.get_parameter('inflation_radius').value)
        self.robot_radius = float(self.get_parameter('robot_radius').value)

        self.sim_time = float(self.get_parameter('sim_time').value)
        self.sim_dt = float(self.get_parameter('sim_dt').value)
        self.vx_samples = int(self.get_parameter('vx_samples').value)
        self.omega_samples = int(self.get_parameter('omega_samples').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.max_accel = float(self.get_parameter('max_accel').value)
        self.max_ang_speed = float(self.get_parameter('max_ang_speed').value)
        self.max_ang_accel = float(self.get_parameter('max_ang_accel').value)
        self.heading_weight = float(self.get_parameter('heading_weight').value)
        self.velocity_weight = float(self.get_parameter('velocity_weight').value)
        self.clearance_weight = float(self.get_parameter('clearance_weight').value)
        self.goal_lookahead_dist = float(self.get_parameter('goal_lookahead_dist').value)

        # Derived costmap dimensions
        grid_dim = max(3, int(round(self.costmap_size / self.resolution)))
        if grid_dim % 2 == 0:
            grid_dim += 1  # keep odd to maintain symmetry about origin
        self.grid_size = grid_dim
        self.half_extent = (self.grid_size * self.resolution) / 2.0

        # Internal state buffers
        self.occupancy = np.zeros((self.grid_size, self.grid_size), dtype=np.int16)
        self.distance_map = np.ones((self.grid_size, self.grid_size), dtype=np.float32) * self.half_extent
        self.global_path: Optional[Path] = None
        self.latest_odom: Optional[Odometry] = None
        self.latest_scan: Optional[LaserScan] = None
        self.grid_origin_x = 0.0
        self.grid_origin_y = 0.0

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # Subscriptions
        qos_path = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Path, self.global_path_topic, self._path_cb, qos_path)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)
        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._scan_cb,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

        # Publishers
        self.cmd_pub = None
        if self.publish_cmd_vel:
            self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.path_pub = self.create_publisher(Path, self.local_path_topic, 10)
        self.costmap_pub = self.create_publisher(OccupancyGrid, self.costmap_topic, 1)

        # Timer
        self.timer = self.create_timer(self.sim_dt, self._on_timer)

        self._last_no_traj_warn = None

        self.get_logger().info(
            f'Local DWA ready (costmap_size={self.costmap_size}m, resolution={self.resolution}m, '
            f'vx_samples={self.vx_samples}, omega_samples={self.omega_samples})'
        )

    # ------------------------------------------------------------------ Callbacks
    def _path_cb(self, msg: Path):
        self.global_path = msg

    def _odom_cb(self, msg: Odometry):
        self.latest_odom = msg

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    # ------------------------------------------------------------------ Timer logic
    def _on_timer(self):
        if self.latest_scan is None or self.latest_odom is None or self.global_path is None:
            return
        if not self.global_path.poses:
            return

        try:
            tf_map_base = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05)
            )
        except Exception as exc:
            self.get_logger().warn(f'TF lookup failed: {exc}')
            return

        self._update_costmap(self.latest_scan, tf_map_base)
        self._publish_costmap()

        goal_point_map = self._select_goal_point(tf_map_base)
        if goal_point_map is None:
            self._warn_no_traj('Goal point unavailable for DWA planning.')
            self._publish_stop()
            return
        goal_point_base = transform_point_base_from_map(goal_point_map[0], goal_point_map[1], tf_map_base)

        best = self._evaluate_velocities(goal_point_base, tf_map_base)
        if best is None:
            self._warn_no_traj('No valid DWA trajectory found; issuing stop.')
            self._publish_stop()
            return

        traj_points, best_v, best_w = best
        self._publish_path(traj_points, tf_map_base)
        self._publish_cmd(best_v, best_w)

    # ------------------------------------------------------------------ Costmap construction
    def _update_costmap(self, scan: LaserScan, tf_map_base):
        self.occupancy.fill(-1)

        base_x = tf_map_base.transform.translation.x
        base_y = tf_map_base.transform.translation.y
        self.grid_origin_x = base_x - self.half_extent
        self.grid_origin_y = base_y - self.half_extent

        tf_base_scan = None
        if scan.header.frame_id and scan.header.frame_id != self.base_frame:
            try:
                tf_base_scan = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    scan.header.frame_id,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.05)
                )
            except Exception as exc:
                self.get_logger().warn(
                    f'Failed to lookup transform {self.base_frame} <- {scan.header.frame_id}: {exc}'
                )

        def to_base(x_local: float, y_local: float) -> Tuple[float, float]:
            if tf_base_scan is None:
                return x_local, y_local
            return apply_transform(x_local, y_local, tf_base_scan.transform)

        step = max(self.resolution * 0.5, 0.02)

        angle = scan.angle_min
        for rng in scan.ranges:
            if math.isinf(rng) or math.isnan(rng):
                angle += scan.angle_increment
                continue
            if rng <= 0.0:
                angle += scan.angle_increment
                continue

            limited_range = min(rng, self.max_obstacle_range, self.costmap_size)
            ray_dist = 0.0
            while ray_dist < limited_range:
                x_ray = ray_dist * math.cos(angle)
                y_ray = ray_dist * math.sin(angle)
                x_b_ray, y_b_ray = to_base(x_ray, y_ray)
                x_m_ray, y_m_ray = transform_point_map_from_base(x_b_ray, y_b_ray, tf_map_base)
                idx_ray = self._map_to_grid(x_m_ray, y_m_ray)
                if idx_ray is not None:
                    r_ray, c_ray = idx_ray
                    if self.occupancy[r_ray, c_ray] == -1:
                        self.occupancy[r_ray, c_ray] = 0
                ray_dist += step

            x = min(rng, self.max_obstacle_range) * math.cos(angle)
            y = min(rng, self.max_obstacle_range) * math.sin(angle)
            angle += scan.angle_increment

            x_b, y_b = to_base(x, y)
            x_m, y_m = transform_point_map_from_base(x_b, y_b, tf_map_base)
            idx = self._map_to_grid(x_m, y_m)
            if idx is None:
                continue
            row, col = idx
            self.occupancy[row, col] = 100

        obstacle_mask = (self.occupancy >= 100).astype(np.uint8)
        free_mask = np.ones_like(obstacle_mask, dtype=np.uint8)
        free_mask[obstacle_mask == 1] = 0

        if np.count_nonzero(obstacle_mask) > 0:
            distance = cv2.distanceTransform(free_mask, cv2.DIST_L2, 3)
            self.distance_map = distance * self.resolution
        else:
            self.distance_map.fill(self.half_extent)

    def _publish_costmap(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.resolution = self.resolution
        msg.info.width = self.grid_size
        msg.info.height = self.grid_size
        msg.info.origin.position.x = float(self.grid_origin_x)
        msg.info.origin.position.y = float(self.grid_origin_y)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = [int(v) for v in self.occupancy.flatten()]
        self.costmap_pub.publish(msg)

    # ------------------------------------------------------------------ Goal selection
    def _select_goal_point(self, tf_map_base) -> Optional[Tuple[float, float]]:
        base_x = tf_map_base.transform.translation.x
        base_y = tf_map_base.transform.translation.y

        poses = self.global_path.poses
        closest_idx = 0
        closest_dist = float('inf')
        for i, pose in enumerate(poses):
            dx = pose.pose.position.x - base_x
            dy = pose.pose.position.y - base_y
            dist = math.hypot(dx, dy)
            if dist < closest_dist:
                closest_dist = dist
                closest_idx = i

        lookahead = self.goal_lookahead_dist
        accum = 0.0
        prev = poses[closest_idx].pose.position
        for offset in range(1, len(poses)):
            idx = min(closest_idx + offset, len(poses) - 1)
            curr = poses[idx].pose.position
            ds = math.hypot(curr.x - prev.x, curr.y - prev.y)
            accum += ds
            prev = curr
            if accum >= lookahead or idx == len(poses) - 1:
                return (curr.x, curr.y)
        return None

    # ------------------------------------------------------------------ DWA evaluation
    def _evaluate_velocities(self, goal_point_base: Tuple[float, float], tf_map_base):
        current_v = self.latest_odom.twist.twist.linear.x
        current_w = self.latest_odom.twist.twist.angular.z

        v_min = max(self.min_speed, current_v - self.max_accel * self.sim_dt)
        v_max = min(self.max_speed, current_v + self.max_accel * self.sim_dt)
        w_min = max(-self.max_ang_speed, current_w - self.max_ang_accel * self.sim_dt)
        w_max = min(self.max_ang_speed, current_w + self.max_ang_accel * self.sim_dt)

        if self.vx_samples <= 1:
            velocities = [max(self.min_speed, min(self.max_speed, current_v))]
        else:
            velocities = np.linspace(v_min, v_max, self.vx_samples)

        if self.omega_samples <= 1:
            omegas = [max(-self.max_ang_speed, min(self.max_ang_speed, current_w))]
        else:
            omegas = np.linspace(w_min, w_max, self.omega_samples)

        best_cost = float('inf')
        best_solution = None

        for v in velocities:
            for w in omegas:
                traj = self._simulate_trajectory(v, w)
                if not traj:
                    continue
                heading_cost, velocity_cost, clearance_cost = self._score_trajectory(
                    traj, v, goal_point_base, tf_map_base
                )
                if math.isinf(heading_cost) or math.isinf(clearance_cost):
                    continue
                total_cost = (
                    self.heading_weight * heading_cost +
                    self.velocity_weight * velocity_cost +
                    self.clearance_weight * clearance_cost
                )
                if total_cost < best_cost:
                    best_cost = total_cost
                    best_solution = (traj, float(v), float(w))

        return best_solution

    def _simulate_trajectory(self, v: float, w: float) -> List[Tuple[float, float, float]]:
        x = 0.0
        y = 0.0
        yaw = 0.0
        traj: List[Tuple[float, float, float]] = []

        steps = max(1, int(self.sim_time / self.sim_dt))
        for _ in range(steps):
            x += v * math.cos(yaw) * self.sim_dt
            y += v * math.sin(yaw) * self.sim_dt
            yaw += w * self.sim_dt
            traj.append((x, y, yaw))
        return traj

    def _score_trajectory(
        self,
        traj: List[Tuple[float, float, float]],
        v: float,
        goal_point_base: Tuple[float, float],
        tf_map_base,
    ) -> Tuple[float, float, float]:
        min_clearance = float('inf')

        for (x, y, _) in traj:
            x_m, y_m = transform_point_map_from_base(x, y, tf_map_base)
            idx = self._map_to_grid(x_m, y_m)
            if idx is None:
                return float('inf'), float('inf'), float('inf')
            row, col = idx
            occ = self.occupancy[row, col]
            if occ >= 100:
                return float('inf'), float('inf'), float('inf')
            clearance = self.distance_map[row, col]
            min_clearance = min(min_clearance, clearance)

        if min_clearance < self.robot_radius:
            return float('inf'), float('inf'), float('inf')

        final_x, final_y, final_yaw = traj[-1]
        goal_angle = math.atan2(goal_point_base[1], goal_point_base[0])
        heading_error = abs(normalize_angle(goal_angle - final_yaw))

        velocity_cost = (self.max_speed - v) / max(self.max_speed, 1e-3)
        clearance_cost = 1.0 / max(min_clearance - self.robot_radius, 0.01)

        return heading_error, velocity_cost, clearance_cost

    # ------------------------------------------------------------------ Publications
    def _publish_cmd(self, v: float, w: float):
        if self.cmd_pub is None:
            return
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    def _publish_stop(self):
        self._publish_cmd(0.0, 0.0)

    def _publish_path(self, traj: List[Tuple[float, float, float]], tf_map_base):
        path_msg = Path()
        stamp = self.get_clock().now().to_msg()
        path_msg.header.stamp = stamp
        path_msg.header.frame_id = self.map_frame

        if not traj:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
            self.path_pub.publish(path_msg)
            return

        for i, (x_b, y_b, yaw_b) in enumerate(traj):
            pose = PoseStamped()
            pose.header = path_msg.header
            x_m, y_m = transform_point_map_from_base(x_b, y_b, tf_map_base)
            pose.pose.position.x = float(x_m)
            pose.pose.position.y = float(y_m)
            pose.pose.position.z = 0.0

            if i < len(traj) - 1:
                next_x, next_y, _ = traj[i + 1]
                yaw = math.atan2(next_y - y_b, next_x - x_b)
            else:
                yaw = yaw_b

            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    # ------------------------------------------------------------------ Helpers
    def _map_to_grid(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        rel_x = x - self.grid_origin_x
        rel_y = y - self.grid_origin_y
        if rel_x < 0.0 or rel_y < 0.0:
            return None
        if rel_x >= self.costmap_size or rel_y >= self.costmap_size:
            return None
        col = int(rel_x / self.resolution)
        row = int(rel_y / self.resolution)
        if 0 <= row < self.grid_size and 0 <= col < self.grid_size:
            return row, col
        return None

    def _warn_no_traj(self, message: str):
        now = self.get_clock().now()
        if (
            self._last_no_traj_warn is None
            or (now - self._last_no_traj_warn).nanoseconds > int(1e9)
        ):
            self.get_logger().warn(message)
            self._last_no_traj_warn = now


def main(args=None):
    rclpy.init(args=args)
    node = LocalDwaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Local DWA planner stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
