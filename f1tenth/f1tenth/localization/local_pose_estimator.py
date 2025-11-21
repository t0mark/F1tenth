#!/usr/bin/env python3
"""Local pose estimator for the F1TENTH stack.

This node performs lightweight sensor fusion between low-drift wheel odometry
and high-rate IMU orientation to provide a smooth odometry stream that is easy
to swap in and out via configuration.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


def _yaw_from_quaternion(q: Quaternion) -> float:
    """Convert quaternion to yaw (Z axis)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _quaternion_from_yaw(yaw: float) -> Quaternion:
    """Create a quaternion representing a yaw rotation."""
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _blend_angle(prev: float, measurement: float, alpha: float) -> float:
    """Blend two yaw angles while respecting wrap-around."""
    delta = math.atan2(math.sin(measurement - prev), math.cos(measurement - prev))
    return prev + alpha * delta


class LocalPoseEstimator(Node):
    """Simple complementary filter for local odometry."""

    def __init__(self) -> None:
        super().__init__('local_pose_estimator')

        # Topics and frames can be swapped via YAML.
        self.declare_parameter('odom_topic', '/wheel/odom')
        self.declare_parameter('imu_topic', '/camera/camera/imu')
        self.declare_parameter('output_topic', '/localization/local_odom')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')

        # Filter behavior.
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('position_smoothing_alpha', 0.4)
        self.declare_parameter('yaw_smoothing_alpha', 0.7)
        self.declare_parameter('velocity_smoothing_alpha', 0.5)
        self.declare_parameter('use_imu_orientation', True)

        # Covariance hints used for downstream fusion.
        self.declare_parameter('position_variance', 0.02)
        self.declare_parameter('orientation_variance', 0.01)
        self.declare_parameter('velocity_variance', 0.04)

        odom_topic = self.get_parameter('odom_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.publish_period = 1.0 / float(self.get_parameter('publish_rate_hz').value)

        # Subscriptions/publisher.
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.imu_sub = self.create_subscription(Imu, imu_topic, self._on_imu, 50)
        self.odom_pub = self.create_publisher(Odometry, self.output_topic, 10)
        self.timer = self.create_timer(self.publish_period, self._publish_estimate)

        # Internal state.
        self._latest_odom: Optional[Odometry] = None
        self._latest_imu: Optional[Imu] = None
        self._fused_position: Optional[Tuple[float, float, float]] = None
        self._fused_yaw: Optional[float] = None
        self._filtered_velocity: float = 0.0

        self.get_logger().info(f'LocalPoseEstimator ready: {odom_topic} -> {self.output_topic}')

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_odom = msg

    def _on_imu(self, msg: Imu) -> None:
        self._latest_imu = msg

    def _publish_estimate(self) -> None:
        if self._latest_odom is None:
            return

        odom = self._latest_odom
        measured_position = (
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z,
        )
        measured_yaw = _yaw_from_quaternion(odom.pose.pose.orientation)

        if self._fused_position is None:
            self._fused_position = measured_position
        if self._fused_yaw is None:
            self._fused_yaw = measured_yaw
            self._filtered_velocity = odom.twist.twist.linear.x

        # Update orientation using IMU when available.
        use_imu = bool(self.get_parameter('use_imu_orientation').value)
        if use_imu and self._latest_imu is not None:
            imu_yaw = _yaw_from_quaternion(self._latest_imu.orientation)
            yaw_measurement = imu_yaw
        else:
            yaw_measurement = measured_yaw

        yaw_alpha = float(self.get_parameter('yaw_smoothing_alpha').value)
        yaw_alpha = max(0.0, min(1.0, yaw_alpha))
        self._fused_yaw = _blend_angle(self._fused_yaw, yaw_measurement, yaw_alpha)

        # Blend position for smoother odometry.
        pos_alpha = float(self.get_parameter('position_smoothing_alpha').value)
        pos_alpha = max(0.0, min(1.0, pos_alpha))
        self._fused_position = tuple(
            (1.0 - pos_alpha) * prev + pos_alpha * meas
            for prev, meas in zip(self._fused_position, measured_position)
        )

        # Velocity smoothing.
        vel_alpha = float(self.get_parameter('velocity_smoothing_alpha').value)
        vel_alpha = max(0.0, min(1.0, vel_alpha))
        measured_velocity = odom.twist.twist.linear.x
        self._filtered_velocity = (1.0 - vel_alpha) * self._filtered_velocity + vel_alpha * measured_velocity

        fused_msg = Odometry()
        fused_msg.header.frame_id = self.odom_frame_id
        fused_msg.child_frame_id = self.base_frame_id
        fused_msg.header.stamp = self.get_clock().now().to_msg()
        fused_msg.pose.pose.position.x = self._fused_position[0]
        fused_msg.pose.pose.position.y = self._fused_position[1]
        fused_msg.pose.pose.position.z = self._fused_position[2]
        fused_msg.pose.pose.orientation = _quaternion_from_yaw(self._fused_yaw)
        fused_msg.twist.twist = odom.twist.twist
        fused_msg.twist.twist.linear.x = self._filtered_velocity

        # Inject simple covariance hints.
        pos_var = float(self.get_parameter('position_variance').value)
        ori_var = float(self.get_parameter('orientation_variance').value)
        vel_var = float(self.get_parameter('velocity_variance').value)
        fused_msg.pose.covariance[0] = pos_var
        fused_msg.pose.covariance[7] = pos_var
        fused_msg.pose.covariance[14] = pos_var
        fused_msg.pose.covariance[21] = ori_var
        fused_msg.pose.covariance[28] = ori_var
        fused_msg.pose.covariance[35] = ori_var
        fused_msg.twist.covariance[0] = vel_var
        fused_msg.twist.covariance[7] = vel_var
        fused_msg.twist.covariance[14] = vel_var

        self.odom_pub.publish(fused_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalPoseEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
