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
import math

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
        
        # 샘플링된 경로 생성
        self.sampled_waypoints = self._resample_path()
        
        self.get_logger().info(f"원본 경로 포인트: {len(self.waypoints)}")
        self.get_logger().info(f"샘플링된 경로 포인트: {len(self.sampled_waypoints)}")
        self.get_logger().info(f"샘플링 거리: {self.sampling_distance}m")
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
    
    def _resample_path(self):
        """절대 거리 기준으로 경로를 리샘플링 (폐곡선)"""
        if len(self.waypoints) < 2:
            return self.waypoints
        
        sampled = []
        sampled.append(self.waypoints[0])
        
        total_distance = 0.0
        
        for i in range(len(self.waypoints)):
            p1 = self.waypoints[i]
            p2 = self.waypoints[(i + 1) % len(self.waypoints)]
            
            segment_dist = np.linalg.norm(p2 - p1)
            
            if segment_dist < 1e-6:
                continue
            
            direction = (p2 - p1) / segment_dist
            
            while True:
                next_sample_dist = (len(sampled)) * self.sampling_distance
                distance_along_segment = next_sample_dist - total_distance
                
                if distance_along_segment > segment_dist:
                    break
                
                sample_point = p1 + direction * distance_along_segment
                sampled.append(sample_point)
            
            total_distance += segment_dist
        
        self.get_logger().info(f"경로 재샘플링 완료: {len(sampled)} 포인트")
        return sampled
    
    def _find_nearest_waypoint_index(self, robot_position):
        """로봇 현재 위치에서 가장 가까운 경로 포인트 인덱스 찾기"""
        if len(self.sampled_waypoints) == 0:
            return 0
        
        distances = []
        for waypoint in self.sampled_waypoints:
            dist = np.linalg.norm(waypoint - robot_position)
            distances.append(dist)
        
        nearest_idx = np.argmin(distances)
        nearest_distance = distances[nearest_idx]
        
        self.get_logger().debug(
            f"로봇 위치 (map): ({robot_position[0]:.3f}, {robot_position[1]:.3f}), "
            f"가장 가까운 포인트: idx={nearest_idx}, 거리={nearest_distance:.3f}m"
        )
        
        return nearest_idx
    
    def _get_lookahead_waypoints(self, start_idx):
        """현재 위치에서 시작하여 lookahead 거리만큼의 포인트들 추출"""
        if len(self.sampled_waypoints) == 0:
            return []
        
        lookahead_points = []
        accumulated_distance = 0.0
        idx = start_idx
        
        for _ in range(len(self.sampled_waypoints)):
            lookahead_points.append(self.sampled_waypoints[idx])
            
            if accumulated_distance >= self.lookahead_distance:
                break
            
            next_idx = (idx + 1) % len(self.sampled_waypoints)
            segment_dist = np.linalg.norm(
                self.sampled_waypoints[next_idx] - self.sampled_waypoints[idx]
            )
            accumulated_distance += segment_dist
            idx = next_idx
        
        return lookahead_points
    
    def timer_callback(self):
        """주기적으로 로컬 경로 발행"""
        if not self.sampled_waypoints:
            return
        
        # TF2를 통해 로봇 현재 위치 (map 프레임) 추출
        robot_position, tf_success = self._get_robot_position_from_tf()
        
        if not tf_success:
            self.get_logger().warn_throttle(
                1.0,  # 1초마다 한 번만 출력
                "TF 변환을 가져올 수 없습니다. 경로를 발행하지 않습니다."
            )
            return
        
        # 현재 로봇 위치에서 가장 가까운 포인트 찾기
        nearest_idx = self._find_nearest_waypoint_index(robot_position)
        
        # lookahead 거리만큼 포인트 추출
        lookahead_waypoints = self._get_lookahead_waypoints(nearest_idx)
        
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
