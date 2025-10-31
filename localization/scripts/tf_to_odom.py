#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import math


class TfToOdomNode(Node):
    def __init__(self):
        super().__init__('tf_to_odom_node')

        # 파라미터 선언
        self.declare_parameter('source_frame', 'map')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('publish_rate', 30.0)

        # 파라미터 가져오기
        self.source_frame = self.get_parameter('source_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        odom_topic = self.get_parameter('odom_topic').value
        publish_rate = self.get_parameter('publish_rate').value

        # TF2 설정
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher 설정
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)

        # 이전 상태 저장 (속도 계산용)
        self.last_transform = None
        self.last_time = None

        # 타이머 설정
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(f'TF to Odom node started')
        self.get_logger().info(f'Publishing {self.source_frame}->{self.target_frame} to {odom_topic}')

    def timer_callback(self):
        try:
            # TF 조회
            now = Time()
            transform = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.target_frame,
                now,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            # Odometry 메시지 생성
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = self.source_frame
            odom_msg.child_frame_id = self.target_frame

            # 위치 및 자세
            odom_msg.pose.pose.position.x = transform.transform.translation.x
            odom_msg.pose.pose.position.y = transform.transform.translation.y
            odom_msg.pose.pose.position.z = transform.transform.translation.z
            odom_msg.pose.pose.orientation = transform.transform.rotation

            # 속도 계산 (수치 미분)
            if self.last_transform is not None and self.last_time is not None:
                dt = (self.get_clock().now() - self.last_time).nanoseconds / 1e9

                if dt > 0:
                    # 선속도 계산
                    dx = transform.transform.translation.x - self.last_transform.transform.translation.x
                    dy = transform.transform.translation.y - self.last_transform.transform.translation.y
                    dz = transform.transform.translation.z - self.last_transform.transform.translation.z

                    odom_msg.twist.twist.linear.x = dx / dt
                    odom_msg.twist.twist.linear.y = dy / dt
                    odom_msg.twist.twist.linear.z = dz / dt

                    # 각속도 계산 (yaw만)
                    yaw_current = self.quaternion_to_yaw(transform.transform.rotation)
                    yaw_last = self.quaternion_to_yaw(self.last_transform.transform.rotation)
                    dyaw = self.normalize_angle(yaw_current - yaw_last)

                    odom_msg.twist.twist.angular.z = dyaw / dt

            # Covariance 설정 (고정값)
            # Pose covariance (6x6 matrix, row-major)
            odom_msg.pose.covariance = [
                0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.03
            ]

            # Twist covariance (6x6 matrix, row-major)
            odom_msg.twist.covariance = [
                0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.03
            ]

            # 발행
            self.odom_pub.publish(odom_msg)

            # 현재 상태 저장
            self.last_transform = transform
            self.last_time = self.get_clock().now()

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Could not get transform: {str(e)}', throttle_duration_sec=5.0)

    def quaternion_to_yaw(self, q):
        """쿼터니언에서 yaw 각도 추출"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        """각도를 -pi ~ pi 범위로 정규화"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = TfToOdomNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
