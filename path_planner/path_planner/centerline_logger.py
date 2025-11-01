#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from numpy import linalg as LA
import os
from os.path import expanduser
from time import gmtime, strftime
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt

# ROS 2에서 tf_transformations를 사용하려면 다음 명령으로 설치되어 있는지 확인하세요:
#   sudo apt-get install ros-humble-tf-transformations
from tf_transformations import euler_from_quaternion


class WaypointsLogger(Node):
    def __init__(self):
        super().__init__('waypoints_logger')

        # 출력 파일 경로 설정
        self.data_dir = 'src/path_planner/data'
        os.makedirs(self.data_dir, exist_ok=True)
        self.log_filename = os.path.join(self.data_dir, 'width_log.csv')
        self.png_filename = os.path.join(self.data_dir, 'width_log_visualization.png')
        
        self.file = open(self.log_filename, 'w')
        self.logged_data = [] # 시각화를 위해 데이터를 메모리에 저장

        # 구독자 생성
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',    # 토픽 이름
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

        self.get_logger().info('Waypoints Logger node has started. Logging to: {}'.format(self.log_filename))

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

        # 쿼터니언 추출
        qx = self.pose_msg.pose.pose.orientation.x
        qy = self.pose_msg.pose.pose.orientation.y
        qz = self.pose_msg.pose.pose.orientation.z
        qw = self.pose_msg.pose.pose.orientation.w

        # 오일러 각으로 변환
        euler = euler_from_quaternion([qx, qy, qz, qw])
        yaw = euler[2]  # 롤 = euler[0], 피치 = euler[1], 요 = euler[2]

        # -90도와 +90도 방향의 라이다 스캔 데이터 가져오기
        index_1 = self.angle_to_index(np.pi / 2, msg.angle_min, msg.angle_increment)
        index_2 = self.angle_to_index(-np.pi / 2, msg.angle_min, msg.angle_increment)
        left_dist = msg.ranges[index_1]
        right_dist = msg.ranges[index_2]

        x = self.pose_msg.pose.pose.position.x
        y = self.pose_msg.pose.pose.position.y

        # 파일에 기록
        self.file.write('%f, %f, %f, %f\n' % (x, y, left_dist, right_dist))
        
        # 시각화를 위해 데이터 저장
        self.logged_data.append([x, y, yaw, left_dist, right_dist])

    def visualize_and_save(self):
        self.get_logger().info(f"Generating visualization from {len(self.logged_data)} points...")
        data = np.array(self.logged_data)
        
        x = data[:, 0]
        y = data[:, 1]
        yaw = data[:, 2]
        left_dist = data[:, 3]
        right_dist = data[:, 4]

        # 좌우 경계점 계산
        left_x = x + left_dist * np.cos(yaw + np.pi/2)
        left_y = y + left_dist * np.sin(yaw + np.pi/2)
        right_x = x + right_dist * np.cos(yaw - np.pi/2)
        right_y = y + right_dist * np.sin(yaw - np.pi/2)

        plt.figure(figsize=(10, 8))
        plt.plot(x, y, label='Centerline Path', color='blue')
        plt.scatter(left_x, left_y, label='Left Boundary', color='green', s=1)
        plt.scatter(right_x, right_y, label='Right Boundary', color='red', s=1)
        
        plt.title('Logged Path with LiDAR Boundaries')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        
        plt.savefig(self.png_filename)
        self.get_logger().info(f"Visualization saved to: {self.png_filename}")
        plt.close()

    def destroy_node(self):
        # 종료 전에 파일을 닫고 시각화를 생성합니다.
        self.file.close()
        self.get_logger().info('Waypoints Logger node is shutting down; file closed.')
        if self.logged_data:
            self.visualize_and_save()
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
