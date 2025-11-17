#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Path as NavPath
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class PathSamplerNode(Node):
    def __init__(self):
        super().__init__('path_sampler_node')
        
        # 파라미터 선언
        self.declare_parameter('global_path_topic', '/global_path')
        self.declare_parameter('sampling_distance', 3.0)
        self.declare_parameter('update_hz', 100)
        self.declare_parameter('lookahead_distance', 20.0)
        
        # 파라미터 읽기
        self.global_path_topic = self.get_parameter('global_path_topic').value
        self.sampling_distance = self.get_parameter('sampling_distance').value
        update_hz = self.get_parameter('update_hz').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        
        # 퍼블리셔 설정
        self.path_pub = self.create_publisher(NavPath, '/local_path', 10)
        
        # Odometry 구독
        self.odom_position = None
        self.odom_yaw = None
        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        
        # 글로벌 경로 구독
        self.global_path_np = np.empty((0, 2))
        self._path_received = False
        self._last_path_warn_time = None
        self.create_subscription(NavPath, self.global_path_topic, self._global_path_callback, 10)
        
        self.get_logger().info(f"글로벌 경로 토픽 구독: {self.global_path_topic}")
        self.get_logger().info(f"샘플링 반경: {self.sampling_distance}m")
        self.get_logger().info(f"Lookahead 거리: {self.lookahead_distance}m")
        self.get_logger().info("/odom 토픽을 사용해 로봇 위치를 추적합니다.")
        
        # 타이머 설정(100Hz)
        timer_period = 1.0 / update_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self._last_odom_warn_time = None
    
    def _odom_callback(self, msg: Odometry):
        """/odom 토픽에서 최신 로봇 위치와 자세(yaw)를 추출"""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.odom_position = np.array([position.x, position.y], dtype=float)
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.odom_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def _global_path_callback(self, msg: NavPath):
        """글로벌 경로 메시지를 수신해 내부 경로 배열을 갱신"""
        if not msg.poses:
            self.global_path_np = np.empty((0, 2))
            return
        
        path_points = np.array(
            [[pose.pose.position.x, pose.pose.position.y] for pose in msg.poses],
            dtype=float
        )
        self.global_path_np = path_points
        self._last_path_warn_time = None
        if not getattr(self, "_path_received", False):
            self._path_received = True
            self.get_logger().info(f"글로벌 경로 포인트 수신: {len(self.global_path_np)}")
    
    def _find_nearest_path_index(self, robot_position):
        """로봇 현재 위치에서 가장 가까운 경로 포인트 인덱스 찾기"""
        if self.global_path_np.size == 0:
            return 0
        
        distances = np.linalg.norm(self.global_path_np - robot_position, axis=1)
        nearest_idx = int(np.argmin(distances))
        nearest_distance = float(distances[nearest_idx])
        
        self.get_logger().debug(
            f"로봇 위치 (map): ({robot_position[0]:.3f}, {robot_position[1]:.3f}), "
            f"가장 가까운 포인트: idx={nearest_idx}, 거리={nearest_distance:.3f}m"
        )
        
        return nearest_idx
    
    def _get_waypoints_within_radius(self, robot_position, robot_yaw):
        """글로벌 경로 순서를 유지하며 로봇 앞쪽에서 샘플링 반경과 Lookahead 조건을 만족하는 포인트 선택"""
        if self.global_path_np.size == 0:
            return []
        
        heading = np.array([math.cos(robot_yaw), math.sin(robot_yaw)], dtype=float)
        nearest_idx = self._find_nearest_path_index(robot_position)
        n = len(self.global_path_np)
        
        # 로봇 앞쪽에 있는 최초의 waypoint 인덱스 탐색
        start_idx = None
        for step in range(n):
            idx = (nearest_idx + step) % n
            vector = self.global_path_np[idx] - robot_position
            if float(np.dot(vector, heading)) > 0.0:
                start_idx = idx
                break
        
        if start_idx is None:
            # 모든 포인트가 뒤쪽이라면 가장 가까운 포인트만 사용
            return [self.global_path_np[nearest_idx]]
        
        lookahead_points = []
        accumulated_distance = 0.0
        previous_point = robot_position
        first_forward_point = None
        
        for step in range(n):
            idx = (start_idx + step) % n
            waypoint = self.global_path_np[idx]
            vector_from_robot = waypoint - robot_position
            dot = float(np.dot(vector_from_robot, heading))
            if dot <= 0.0:
                if lookahead_points:
                    break  # 더 이상 로봇 앞쪽이 아님
                continue  # 아직 포인트가 없으면 다음 후보 탐색
            
            distance_from_robot = float(np.linalg.norm(vector_from_robot))
            if first_forward_point is None:
                first_forward_point = waypoint
            if distance_from_robot > self.sampling_distance:
                if lookahead_points:
                    break
                continue
            
            segment_dist = float(np.linalg.norm(waypoint - previous_point))
            if lookahead_points:
                accumulated_distance += segment_dist
            else:
                accumulated_distance += distance_from_robot
            lookahead_points.append(waypoint)
            previous_point = waypoint
            
            if accumulated_distance >= self.lookahead_distance:
                break
        
        if not lookahead_points:
            if first_forward_point is not None:
                lookahead_points.append(first_forward_point)
            else:
                lookahead_points.append(self.global_path_np[nearest_idx])
        
        return lookahead_points
    
    def timer_callback(self):
        """주기적으로 로컬 경로 발행"""
        if self.global_path_np.size == 0:
            now = self.get_clock().now()
            if (
                self._last_path_warn_time is None
                or (now - self._last_path_warn_time).nanoseconds > int(1e9)
            ):
                self.get_logger().warn("글로벌 경로를 아직 수신하지 못했습니다. 경로를 발행하지 않습니다.")
                self._last_path_warn_time = now
            return
        
        # /odom 토픽에서 로봇 현재 위치 (map 프레임) 사용
        if self.odom_position is None or self.odom_yaw is None:
            now = self.get_clock().now()
            if (
                self._last_odom_warn_time is None
                or (now - self._last_odom_warn_time).nanoseconds > int(1e9)
            ):
                self.get_logger().warn("/odom 데이터를 아직 수신하지 못했습니다. 경로를 발행하지 않습니다.")
                self._last_odom_warn_time = now
            return
        
        robot_position = self.odom_position.copy()
        
        # 샘플링 반경 내의 포인트 추출
        lookahead_waypoints = self._get_waypoints_within_radius(robot_position, self.odom_yaw)
        
        # 로컬 경로 메시지 생성
        path_msg = NavPath()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for waypoint in lookahead_waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(waypoint[0])
            pose.pose.position.y = float(waypoint[1])
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)
        
        pose_count = len(path_msg.poses)
        if pose_count == 1:
            heading = np.array([math.cos(self.odom_yaw), math.sin(self.odom_yaw)], dtype=float)
            yaw = math.atan2(heading[1], heading[0])
            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)
            path_msg.poses[0].pose.orientation.z = qz
            path_msg.poses[0].pose.orientation.w = qw
        elif pose_count > 1:
            for i in range(pose_count):
                if i < pose_count - 1:
                    next_point = np.array([
                        path_msg.poses[i + 1].pose.position.x,
                        path_msg.poses[i + 1].pose.position.y
                    ])
                    current_point = np.array([
                        path_msg.poses[i].pose.position.x,
                        path_msg.poses[i].pose.position.y
                    ])
                    direction = next_point - current_point
                else:
                    prev_point = np.array([
                        path_msg.poses[i - 1].pose.position.x,
                        path_msg.poses[i - 1].pose.position.y
                    ])
                    current_point = np.array([
                        path_msg.poses[i].pose.position.x,
                        path_msg.poses[i].pose.position.y
                    ])
                    direction = current_point - prev_point
                if np.linalg.norm(direction) < 1e-6:
                    direction = np.array([math.cos(self.odom_yaw), math.sin(self.odom_yaw)], dtype=float)
                yaw = math.atan2(direction[1], direction[0])
                qz = math.sin(yaw / 2.0)
                qw = math.cos(yaw / 2.0)
                path_msg.poses[i].pose.orientation.z = qz
                path_msg.poses[i].pose.orientation.w = qw
        
        self.path_pub.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PathSamplerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('경로 샘플러 종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
