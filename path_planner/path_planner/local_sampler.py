#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import csv
from pathlib import Path
from nav_msgs.msg import Path as NavPath
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
from tf2_ros.transform_listener import TransformListener

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
        
        # TF2 Buffer와 Listener 설정
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # TF 프레임 설정
        self.source_frame = 'map'      # 경로 기준 프레임
        self.target_frame = 'base_link'  # 로봇 기준 프레임
        
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
        self.get_logger().info(f"TF 변환: {self.source_frame} -> {self.target_frame}")
        
        # 타이머 (100Hz)
        timer_period = 1.0 / update_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def _get_robot_position_from_tf(self):
        """TF2를 통해 map 프레임에서의 로봇 현재 위치 추출"""
        try:
            # map 프레임에서의 base_link 위치 쿼리
            transform = self.tf_buffer.lookup_transform(
                self.source_frame,  # 목표 프레임 (map)
                self.target_frame,  # 원본 프레임 (base_link)
                rclpy.time.Time()   # 최신 변환 사용
            )
            
            # 위치 추출
            position = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y
            ])
            
            return position, True
        
        except Exception as e:
            self.get_logger().warn(f"TF 변환 실패: {e}")
            return np.array([0.0, 0.0]), False
    
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
    
    def _get_waypoints_within_radius(self, robot_position):
        """샘플링 반경 내의 체크포인트를 로컬 경로로 사용"""
        if self.waypoints_np.size == 0:
            return []
        
        distances = np.linalg.norm(self.waypoints_np - robot_position, axis=1)
        within_indices = np.where(distances <= self.sampling_distance)[0]
        
        nearest_idx = self._find_nearest_waypoint_index(robot_position)
        
        if within_indices.size == 0:
            return [self.waypoints_np[nearest_idx]]
        
        ordered_indices = self._order_selected_indices(nearest_idx, within_indices)
        if not ordered_indices:
            ordered_indices = [nearest_idx]
        
        lookahead_points = []
        accumulated_distance = 0.0
        previous_point = robot_position
        
        for idx in ordered_indices:
            waypoint = self.waypoints_np[idx]
            segment_dist = np.linalg.norm(waypoint - previous_point)
            accumulated_distance += segment_dist
            
            if accumulated_distance > self.lookahead_distance and len(lookahead_points) > 0:
                break
            
            lookahead_points.append(waypoint)
            previous_point = waypoint
        
        return lookahead_points
    
    def timer_callback(self):
        """주기적으로 로컬 경로 발행"""
        if not self.waypoints:
            return
        
        # TF2를 통해 로봇 현재 위치 (map 프레임) 추출
        robot_position, tf_success = self._get_robot_position_from_tf()
        
        if not tf_success:
            self.get_logger().warn_throttle(
                1.0,  # 1초마다 한 번만 출력
                "TF 변환을 가져올 수 없습니다. 경로를 발행하지 않습니다."
            )
            return
        
        # 샘플링 반경 내의 포인트 추출
        lookahead_waypoints = self._get_waypoints_within_radius(robot_position)
        
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
