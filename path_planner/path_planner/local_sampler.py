#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
import numpy as np
import csv
from pathlib import Path
from nav_msgs.msg import Path as NavPath
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class PathSamplerNode(Node):
    def __init__(self):
        super().__init__('path_sampler_node')
        
        # 파라미터 선언
        self.declare_parameter('csv_path', './centerline.csv')
        self.declare_parameter('sampling_distance', 3.0)
        self.declare_parameter('update_hz', 100)
        self.declare_parameter('lookahead_distance', 20.0)
        
        # 파라미터 읽기
        self.csv_path = self.get_parameter('csv_path').value
        self.sampling_distance = self.get_parameter('sampling_distance').value
        update_hz = self.get_parameter('update_hz').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        
        # Publisher
        self.path_pub = self.create_publisher(NavPath, '/local_path', 10)
        
        # Odometry 구독
        self.odom_position = None
        self.odom_yaw = None
        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        
        # 경로 데이터 로드
        self.waypoints = self._load_path_from_csv()
        
        if len(self.waypoints) < 2:
            self.get_logger().error("경로 포인트가 2개 미만입니다.")
            rclpy.shutdown()
            return
        
        self.waypoints_np = np.array(self.waypoints) if self.waypoints else np.empty((0, 2))
        
        self.get_logger().info(f"로드된 경로 포인트: {len(self.waypoints)}")
        self.get_logger().info(f"샘플링 반경 내 포인트만 사용: {self.sampling_distance}m")
        self.get_logger().info(f"Lookahead 거리: {self.lookahead_distance}m")
        self.get_logger().info("/odom 토픽을 사용해 로봇 위치를 추적합니다.")
        
        # 타이머 (100Hz)
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
    
    def _load_path_from_csv(self):
        """CSV 파일에서 경로 포인트 로드"""
        waypoints = []
        try:
            csv_file = Path(self.csv_path)
            if not csv_file.exists():
                self.get_logger().error(f"CSV 파일을 찾을 수 없습니다: {self.csv_path}")
                return waypoints
            
            with open(csv_file, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    x = float(row['x'])
                    y = float(row['y'])
                    waypoints.append(np.array([x, y]))
            
            self.get_logger().info(f"CSV에서 {len(waypoints)}개의 포인트 로드 완료")
            return waypoints
        
        except Exception as e:
            self.get_logger().error(f"CSV 로드 중 오류: {e}")
            return waypoints
    
    def _find_nearest_waypoint_index(self, robot_position):
        """로봇 현재 위치에서 가장 가까운 경로 포인트 인덱스 찾기"""
        if self.waypoints_np.size == 0:
            return 0
        
        distances = np.linalg.norm(self.waypoints_np - robot_position, axis=1)
        nearest_idx = int(np.argmin(distances))
        nearest_distance = float(distances[nearest_idx])
        
        self.get_logger().debug(
            f"로봇 위치 (map): ({robot_position[0]:.3f}, {robot_position[1]:.3f}), "
            f"가장 가까운 포인트: idx={nearest_idx}, 거리={nearest_distance:.3f}m"
        )
        
        return nearest_idx
    
    def _order_selected_indices(self, nearest_idx, selected_indices):
        """선택된 인덱스를 시작 인덱스 기준으로 순차 정렬"""
        if len(selected_indices) == 0:
            return []
        
        ordered = []
        visited = set()
        selected_set = set(int(i) for i in selected_indices)
        n = len(self.waypoints_np)
        
        idx = nearest_idx
        for _ in range(n):
            if idx in selected_set and idx not in visited:
                ordered.append(idx)
                visited.add(idx)
                if len(ordered) == len(selected_set):
                    break
            idx = (idx + 1) % n
        
        return ordered
    
    def _get_waypoints_within_radius(self, robot_position, robot_yaw):
        """CSV 순서를 유지하며 로봇 앞쪽에서 샘플링 반경과 Lookahead 조건을 만족하는 포인트 선택"""
        if self.waypoints_np.size == 0:
            return []
        
        heading = np.array([math.cos(robot_yaw), math.sin(robot_yaw)], dtype=float)
        nearest_idx = self._find_nearest_waypoint_index(robot_position)
        n = len(self.waypoints_np)
        
        # 로봇 앞쪽에 있는 최초의 waypoint 인덱스 탐색
        start_idx = None
        for step in range(n):
            idx = (nearest_idx + step) % n
            vector = self.waypoints_np[idx] - robot_position
            if float(np.dot(vector, heading)) > 0.0:
                start_idx = idx
                break
        
        if start_idx is None:
            # 모든 포인트가 뒤쪽이라면 가장 가까운 포인트만 사용
            return [self.waypoints_np[nearest_idx]]
        
        lookahead_points = []
        accumulated_distance = 0.0
        previous_point = robot_position
        
        for step in range(n):
            idx = (start_idx + step) % n
            waypoint = self.waypoints_np[idx]
            vector_from_robot = waypoint - robot_position
            dot = float(np.dot(vector_from_robot, heading))
            if dot <= 0.0:
                break  # 더 이상 로봇 앞쪽이 아님
            
            distance_from_robot = float(np.linalg.norm(vector_from_robot))
            if distance_from_robot > self.sampling_distance:
                if lookahead_points:
                    break
                continue
            
            segment_dist = float(np.linalg.norm(waypoint - previous_point))
            if lookahead_points:
                accumulated_distance += segment_dist
            lookahead_points.append(waypoint)
            previous_point = waypoint
            
            if accumulated_distance >= self.lookahead_distance:
                break
        
        if not lookahead_points:
            lookahead_points.append(self.waypoints_np[nearest_idx])
        
        return lookahead_points
    
    def timer_callback(self):
        """주기적으로 로컬 경로 발행"""
        if not self.waypoints:
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
            pose.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose)
        
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
