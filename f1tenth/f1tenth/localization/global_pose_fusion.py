#!/usr/bin/env python3
"""Global pose fusion node.

This node combines a slowly-updated global measurement (e.g. AMCL) with the
high-rate local odometry signal to provide a drift-aware global pose estimate.
The node is intentionally light-weight so that it can be swapped via YAML
configuration alongside other localization backends.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node


def _yaw_from_quaternion(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class GlobalPoseFusion(Node):
    """Fuse global and local localization sources."""

    def __init__(self) -> None:
        super().__init__('global_pose_fusion')

        self.declare_parameter('localization_source_topic', '/localization/local_odom')
        self.declare_parameter('global_measurement_topic', '/amcl_pose')
        self.declare_parameter('global_pose_topic', '/localization/global_pose')
        self.declare_parameter('global_odom_topic', '/localization/global_odom')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('covariance_scale', 1.0)
        self.declare_parameter('fallback_position_variance', 0.5)
        self.declare_parameter('fallback_orientation_variance', 0.2)
        self.declare_parameter('measurement_timeout_sec', 10.0)

        local_topic = self.get_parameter('localization_source_topic').value
        measurement_topic = self.get_parameter('global_measurement_topic').value
        self.global_pose_topic = self.get_parameter('global_pose_topic').value
        self.global_odom_topic = self.get_parameter('global_odom_topic').value
        self.map_frame_id = self.get_parameter('map_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        publish_period = 1.0 / float(self.get_parameter('publish_rate_hz').value)

        self.local_sub = self.create_subscription(Odometry, local_topic, self._on_local_odom, 10)
        self.measurement_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            measurement_topic,
            self._on_global_measurement,
            10,
        )
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.global_pose_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, self.global_odom_topic, 10)
        self.timer = self.create_timer(publish_period, self._publish_estimate)

        self._latest_local: Optional[Odometry] = None
        self._reference_local_pose: Optional[Pose] = None
        self._reference_measurement: Optional[PoseWithCovarianceStamped] = None
        self._reference_yaw: Optional[float] = None
        self._reference_measurement_yaw: Optional[float] = None
        self._last_measurement_time: Optional[float] = None

        self.get_logger().info(
            f'GlobalPoseFusion ready: local={local_topic}, measurement={measurement_topic}'
        )

    def _on_local_odom(self, msg: Odometry) -> None:
        self._latest_local = msg

    def _on_global_measurement(self, msg: PoseWithCovarianceStamped) -> None:
        self._reference_measurement = msg
        self._reference_measurement_yaw = _yaw_from_quaternion(msg.pose.pose.orientation)
        self._reference_local_pose = Pose()
        if self._latest_local is not None:
            self._reference_local_pose.position.x = self._latest_local.pose.pose.position.x
            self._reference_local_pose.position.y = self._latest_local.pose.pose.position.y
            self._reference_local_pose.position.z = self._latest_local.pose.pose.position.z
            self._reference_local_pose.orientation = self._latest_local.pose.pose.orientation
            self._reference_yaw = _yaw_from_quaternion(self._latest_local.pose.pose.orientation)
        else:
            self._reference_local_pose.position.x = 0.0
            self._reference_local_pose.position.y = 0.0
            self._reference_local_pose.position.z = 0.0
            self._reference_local_pose.orientation = _quaternion_from_yaw(0.0)
            self._reference_yaw = 0.0
        self._last_measurement_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info('Received new global measurement')

    def _publish_estimate(self) -> None:
        if self._latest_local is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        timeout = float(self.get_parameter('measurement_timeout_sec').value)
        measurement_valid = (
            self._reference_measurement is not None
            and self._reference_local_pose is not None
            and self._last_measurement_time is not None
            and (now - self._last_measurement_time) < timeout
        )

        if measurement_valid:
            pose, yaw = self._compose_global_pose()
            covariance = list(self._reference_measurement.pose.covariance)
            cov_scale = float(self.get_parameter('covariance_scale').value)
            covariance[0] += cov_scale
            covariance[7] += cov_scale
            covariance[14] += cov_scale
            covariance[35] += cov_scale
        else:
            pose = self._latest_local.pose.pose
            yaw = _yaw_from_quaternion(pose.orientation)
            covariance = [0.0] * 36
            covariance[0] = float(self.get_parameter('fallback_position_variance').value)
            covariance[7] = covariance[0]
            covariance[14] = covariance[0]
            covariance[35] = float(self.get_parameter('fallback_orientation_variance').value)

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = self.map_frame_id
        pose_msg.header.stamp = self._latest_local.header.stamp
        pose_msg.pose.pose = pose
        pose_msg.pose.covariance = covariance
        self.pose_pub.publish(pose_msg)

        odom_msg = Odometry()
        odom_msg.header.frame_id = self.map_frame_id
        odom_msg.child_frame_id = self.base_frame_id
        odom_msg.header.stamp = self._latest_local.header.stamp
        odom_msg.pose.pose = pose
        odom_msg.twist = self._latest_local.twist
        self.odom_pub.publish(odom_msg)

    def _compose_global_pose(self) -> Tuple[Pose, float]:
        assert self._reference_measurement is not None
        assert self._reference_local_pose is not None
        assert self._reference_measurement_yaw is not None
        assert self._reference_yaw is not None

        current_pose = self._latest_local.pose.pose  # type: ignore[union-attr]
        current_yaw = _yaw_from_quaternion(current_pose.orientation)
        ref_local = self._reference_local_pose
        measurement_pose = self._reference_measurement.pose.pose

        delta_x = current_pose.position.x - ref_local.position.x
        delta_y = current_pose.position.y - ref_local.position.y
        delta_yaw = _wrap_angle(current_yaw - self._reference_yaw)

        cos_m = math.cos(self._reference_measurement_yaw)
        sin_m = math.sin(self._reference_measurement_yaw)
        rotated_dx = cos_m * delta_x - sin_m * delta_y
        rotated_dy = sin_m * delta_x + cos_m * delta_y

        fused_pose = Pose()
        fused_pose.position.x = measurement_pose.position.x + rotated_dx
        fused_pose.position.y = measurement_pose.position.y + rotated_dy
        fused_pose.position.z = current_pose.position.z
        fused_yaw = _wrap_angle(self._reference_measurement_yaw + delta_yaw)
        fused_pose.orientation = _quaternion_from_yaw(fused_yaw)
        return fused_pose, fused_yaw


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GlobalPoseFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
