#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from numpy import linalg as LA
from os.path import expanduser
from time import gmtime, strftime
from sensor_msgs.msg import LaserScan

# ROS 2에서 tf_transformations를 사용하려면 다음 명령으로 설치되어 있는지 확인하세요:
#   sudo apt-get install ros-<your_distro>-tf-transformations
from tf_transformations import euler_from_quaternion


class WaypointsLogger(Node):
    def __init__(self):
        super().__init__('waypoints_logger')

        # 타임스탬프가 포함된 파일을 준비합니다.
        home = expanduser('~')
        filename = '/home/vaithak/Downloads/UPenn/F1Tenth/race3_map_edited/width_waypoints-prak.csv'
        self.file = open(filename, 'w')

        # 구독자를 생성합니다.
        self.subscription = self.create_subscription(
            Odometry,
            'ego_racecar/odom',    # 토픽 이름
            self.save_waypoint,
            10                 # QoS 큐 크기
        )
        self.subscription  # 사용되지 않는 변수 경고를 방지합니다.
        self.pose_msg = None

        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        self.get_logger().info('Waypoints Logger node has started. Logging to: {}'.format(filename))

    def save_waypoint(self, msg):
        self.pose_msg = msg

    def angle_to_index(self, angle, angle_min, angle_increment):
        """ 라디안 단위의 각도를 LiDAR data.ranges 배열의 인덱스로 변환합니다.
        """
        index = (angle - angle_min) / angle_increment
        return int(index)

    def scan_callback(self, msg):
        if self.pose_msg is None:
            self.get_logger().warn('Pose message not received yet. Skipping scan data.')
            return

        # 쿼터니언을 추출합니다.
        qx = self.pose_msg.pose.pose.orientation.x
        qy = self.pose_msg.pose.pose.orientation.y
        qz = self.pose_msg.pose.pose.orientation.z
        qw = self.pose_msg.pose.pose.orientation.w

        # 오일러 각으로 변환합니다.
        euler = euler_from_quaternion([qx, qy, qz, qw])
        yaw = euler[2]  # 롤 = euler[0], 피치 = euler[1], 요 = euler[2]

        # 선형 속도를 계산합니다.
        vx = self.pose_msg.twist.twist.linear.x
        vy = self.pose_msg.twist.twist.linear.y
        vz = self.pose_msg.twist.twist.linear.z
        speed = LA.norm([vx, vy, vz], 2)

        # 필요한 경우 콘솔에 출력합니다(예: vx > 0일 때만).
        if vx > 0.0:
            self.get_logger().info(f"Forward velocity: {vx}")

        # -90도와 +90도 방향의 라이다 스캔 데이터를 가져옵니다.
        index_1 = self.angle_to_index(-np.pi / 2, msg.angle_min, msg.angle_increment)
        index_2 = self.angle_to_index(np.pi / 2, msg.angle_min, msg.angle_increment)
        laser_data_1 = msg.ranges[index_1]
        laser_data_2 = msg.ranges[index_2]

        # 파일에 x, y, yaw, speed를 기록합니다.
        self.file.write('%f, %f, %f, %f\n' %
                        (self.pose_msg.pose.pose.position.x,
                         self.pose_msg.pose.pose.position.y,
                         laser_data_1,
                         laser_data_2))

    def destroy_node(self):
        # 종료 전에 파일을 닫습니다.
        self.file.close()
        self.get_logger().info('Waypoints Logger node is shutting down; file closed.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointsLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 모든 리소스가 깔끔하게 종료되도록 합니다.
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
