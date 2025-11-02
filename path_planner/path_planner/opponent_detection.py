#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor

import time
from .modules.frenet_conversion import FrenetConverter
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from bisect import bisect_left
from nav_msgs.msg import Odometry
import math
import numpy as np
from tf_transformations import quaternion_from_euler
from tf_transformations import euler_from_quaternion
from typing import List, Tuple
from .modules import utils
from f1tenth_icra_race_msgs.msg import ObstacleArray, ObstacleMsg

from visualization_msgs.msg import Marker, MarkerArray

import os

Point2D = Tuple[float, float]

# ROS1의 "latching"과 동일한 동작을 제공합니다.
latching_qos = QoSProfile(
    depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST)


def normalize_s(x,track_length):
    x = x % (track_length)
    if x > track_length/2:
        x -= track_length
    return x


class Obstacle:
    """
    장애물의 속성을 표현하는 클래스입니다.
    """
    def __init__(self, x, y, size, theta) -> None:
        self.center_x = x
        self.center_y = y
        self.size = size
        self.id = None
        self.theta = theta

class OpponentDetection(Node):
    """
    트랙 위의 장애물을 감지하는 ROS 노드입니다.

    구독 토픽:
        - `/scan`: 라이다 스캔
        - '/odom' : 자차의 오도메트리 (시뮬레이션 동일)

    발행 토픽:
        - `/breakpoints_markers`: 장애물 경계 마커
        - `/raw_obstacles`: 감지된 장애물 정보
        - `/obstacles_markers_new`: 장애물 시각화 마커
    """

    def __init__(self) -> None:
        """
        노드를 초기화하고, 필요한 구독자와 퍼블리셔를 생성합니다.
        """
        super().__init__('opponent_detection')

        qos = rclpy.qos.QoSProfile(history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
                                   depth=1,
                                   reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
                                   durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)
        
        # 시뮬레이션 실행 여부를 확인하는 파라미터를 선언합니다.

        # 플롯 및 콘솔 디버깅을 위한 파라미터를 생성합니다.
        self.declare_parameter('plot_debug', True)
        self.plot_debug = self.get_parameter('plot_debug').value
        self.declare_parameter('print_debug', False)
        self.print_debug = self.get_parameter('print_debug').value

        # 포즈 토픽 구독자를 생성합니다.
        msgs_cb_group = ReentrantCallbackGroup()
        self.pose_sub = self.create_subscription(
                Odometry,
                '/odom',
                self.pose_callback,
                qos,
                callback_group=msgs_cb_group)
            


        # 라이다 스캔 토픽 구독자를 생성합니다.
        scan_topic = '/scan'
        self.laser_frame = 'laser'
        self.laser_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.laser_callback,
            qos,
            callback_group=msgs_cb_group)

        # CSV 파일에서 웨이포인트를 읽어옵니다.
        waypoint_file = os.path.join('src/path_planner/data', 'final_waypoints.csv')
        self.declare_parameter('waypoint_file', waypoint_file)
        waypoint_file = self.get_parameter('waypoint_file').value
        self.waypoints = np.genfromtxt(waypoint_file, delimiter='; ', skip_header=1)
        waypoint_cols_dict = utils.column_numbers_for_waypoints()
        yaws = self.waypoints[:, waypoint_cols_dict['psi_racetraj_rad']]
        yaws = np.array(utils.convert_psi(yaws))
        self.d_right_array = self.waypoints[:, waypoint_cols_dict['width_right_m']]
        self.d_left_array = self.waypoints[:, waypoint_cols_dict['width_left_m']]
        self.s_array = self.waypoints[:, waypoint_cols_dict['s_racetraj_m']]
        self.smallest_d = min(min(self.d_right_array), min(self.d_left_array))
        self.biggest_d = max(self.d_right_array+self.d_left_array)
        self.track_length = self.waypoints[-1, waypoint_cols_dict['s_racetraj_m']]

        # FrenetConverter 객체를 초기화합니다.
        self.frenet_converter = FrenetConverter(
            self.waypoints[:, 0], self.waypoints[:, 1], yaws)
        self.get_logger().info(
            "[Opponent Detection]: initialized FrenetConverter object")

        # --- 노드 속성 ---

        # --- 퍼블리셔 ---
        if self.plot_debug:
            self.breakpoints_markers_pub = self.create_publisher(
                MarkerArray, '/perception/breakpoints_markers', 5)
            self.obstacles_marker_pub = self.create_publisher(
                MarkerArray, '/perception/obstacles_markers_new', 5)

        # Raw obstacles를 tracking 노드로 전달
        self.obstacles_msg_pub = self.create_publisher(
            ObstacleArray, '/perception/detection/raw_obstacles', 5)            

        self.declare_parameter("rate", 40, descriptor=ParameterDescriptor(
            description="rate at which the node is running"))
        self.declare_parameter("lambda", 10, descriptor=ParameterDescriptor(
            description="minimum reliables detection angle in degrees"))
        self.declare_parameter("sigma", 0.03, descriptor=ParameterDescriptor(
            description="standard deviation of the noise of the lidar ranges in m"))
        self.declare_parameter("min_2_points_dist", 0.01, descriptor=ParameterDescriptor(
            description="minimum distance between two points"))

        # --- 조정 가능한 파라미터 ---
        self.rate = self.get_parameter(
            "rate").get_parameter_value().integer_value
        self.lambda_angle = self.get_parameter(
            "lambda").get_parameter_value().integer_value * math.pi/180
        self.sigma = self.get_parameter(
            "sigma").get_parameter_value().double_value
        self.min_2_points_dist = self.get_parameter(
            "min_2_points_dist").get_parameter_value().double_value

        # 위 파라미터를 동적이 아닌 조정 가능한 형태로 유지합니다.
        # 장애물에 포함될 최소 포인트 수 (단위: 포인트)
        self.declare_parameter('min_obs_size', 10, descriptor=ParameterDescriptor(
            description="minimum number of points in an obstacle")
        )
        # 장애물의 최대 크기 (단위: 미터)
        self.declare_parameter('max_obs_size', 0.6, descriptor=ParameterDescriptor(
            description="maximum size of an obstacle in m"),
        )
        # 라이다의 최대 시야 거리 (단위: 미터)
        self.declare_parameter('max_viewing_distance', 5.0, descriptor=ParameterDescriptor(
            description="maximum viewing distance of the lidar in m"),
        )
        # 트랙 경계 안쪽으로 벽으로 간주할 마진 (단위: 미터)
        self.declare_parameter('track_boundary_margin', 0.25, descriptor=ParameterDescriptor(
            description="margin inside track boundary to consider as wall in m"),
        )
        # 중심선으로부터 장애물을 사전 필터링할 거리 (단위: 미터)
        self.declare_parameter('centerline_filter_distance', 0.5, descriptor=ParameterDescriptor(
            description="distance from centerline to pre-filter obstacles in m"),
        )
        
        self.min_obs_size = self.get_parameter(
            'min_obs_size').get_parameter_value().integer_value
        self.max_obs_size = self.get_parameter(
            'max_obs_size').get_parameter_value().double_value
        self.max_viewing_distance = self.get_parameter(
            'max_viewing_distance').get_parameter_value().double_value
        self.track_boundary_margin = self.get_parameter(
            'track_boundary_margin').get_parameter_value().double_value
        self.centerline_filter_distance = self.get_parameter(
            'centerline_filter_distance').get_parameter_value().double_value
        

        # --- 변수 ---
        # 자차의 s 좌표
        self.car_s = None
        self.car_global_x = 0
        self.car_global_y = 0
        self.car_global_yaw = 0

        # 라이다 원시 스캔 데이터
        self.laser_scans = None
        self.angle_increment = 0
        self.angle_min = 0
        self.front_view_start_index = 0
        self.front_view_end_index = 0
        self.angles = None

        self.tracked_obstacles = []

        # main_timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.main_timer = self.create_timer(1/self.rate, self.loop, callback_group=msgs_cb_group)

    # --- 콜백 ---

    def pose_callback(self, pose_msg):
        # 차량의 현재 x, y 위치를 가져옵니다.
        pose = pose_msg.pose.pose
        self.car_global_x = pose.position.x
        self.car_global_y = pose.position.y
        self.car_global_yaw = euler_from_quaternion([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])[2]
        if self.print_debug:
            self.get_logger().info(f'Pose: {self.car_global_x}, {self.car_global_y}, {self.car_global_yaw}')

        # 전역 좌표를 프레네 좌표로 변환합니다.
        s, _ = self.frenet_converter.get_frenet(np.array([self.car_global_x]), np.array([self.car_global_y]))
        self.car_s = normalize_s(s[0], self.track_length)


    def laser_callback(self, msg: LaserScan):
        self.laser_scans = msg
        self.angle_increment = msg.angle_increment
        self.angle_min = msg.angle_min
        self.front_view_start_index = self.angle_to_index(-np.pi/2)
        self.front_view_end_index = self.angle_to_index(np.pi/2)
        if self.angles is None:
            self.angles = np.arange(-np.pi/2, np.pi/2, self.angle_increment)

    # --- 함수 모음 ---

    def angle_to_index(self, angle):
        """ 라디안 각도를 LiDAR ranges 배열의 인덱스로 변환합니다.
        """
        index = (angle - self.angle_min) / self.angle_increment
        return int(index)

    def clearmarkers(self) -> MarkerArray:
        marker_array = MarkerArray()
        marker = Marker()
        marker.action = 3
        marker_array.markers = [marker]
        return marker_array

    def laserPointOnTrack(self, s, d, car_s) -> bool:
        if normalize_s(s-car_s, self.track_length) > self.max_viewing_distance:
            return False
        if abs(d) >= self.biggest_d:
            return False
        if abs(d) <= self.smallest_d:
            return True
        idx = bisect_left(self.s_array, s)
        if idx:
            idx -= 1
        if (d <= -self.d_right_array[idx]) or (d >= self.d_left_array[idx]):
            return False
        return True

    def scans2ObsPointCloud(self, car_s: Float32, scans: LaserScan, car_x: Float32, car_y: Float32, car_yaw: Float32) -> List[List[Point2D]]:
        """
        라이다 스캔을 2D 포인트 클라우드로 변환하고 객체 단위로 분할합니다.
        """
        # --- 보조 파라미터 초기화 ---
        l = self.lambda_angle
        d_phi = scans.angle_increment
        sigma = self.sigma

        # --- 스캔 범위를 포인트 클라우드로 변환 ---
        # -90도에서 90도 범위의 각도만 고려합니다.
        ranges = np.array(scans.ranges[self.front_view_start_index:self.front_view_end_index+1])
        x_laser_frame = (ranges * np.cos(self.angles)).flatten()
        y_laser_frame = (ranges * np.sin(self.angles)).flatten()
        z_laser_frame = np.zeros(len(ranges))
        # 4xN 행렬
        xyz_laser_frame = np.vstack((x_laser_frame, y_laser_frame, z_laser_frame, np.ones(len(ranges))))

        # 차량 자세를 이용해 변환 행렬을 구성합니다.
        H_l2m = np.eye(4)
        H_l2m[0, 3] = car_x
        H_l2m[1, 3] = car_y
        H_l2m[2, 3] = 0.0
        H_l2m[0, 0] = np.cos(car_yaw)
        H_l2m[0, 1] = -np.sin(car_yaw)
        H_l2m[1, 0] = np.sin(car_yaw)
        H_l2m[1, 1] = np.cos(car_yaw)

        xyz_map = H_l2m @ xyz_laser_frame

        cloudPoints_list = np.transpose(xyz_map[:2, :]).tolist()

        # --------------------------------------------------
        # 적응형 방법으로 포인트 클라우드를 잠재적 객체 단위로
        # 분할합니다.
        # --------------------------------------------------

        first_point: Point2D = (cloudPoints_list[0][0], cloudPoints_list[0][1])
        objects_pointcloud_list: List[List[Point2D]] = [[first_point]]

        div_const = np.sin(d_phi) / np.sin(l - d_phi)

        """
        for i in range(1, len(cloudPoints_list)):
            curr_range = scans.ranges[i]
            d_max = curr_range * div_const + 3 * sigma
 
            # 점 사이의 거리는 맵 프레임과 라이다 프레임 모두에서 동일합니다.
            dist_to_next_point = np.linalg.norm(xyz_laser_frame[:2, i] - xyz_laser_frame[:2, i - 1])
            
            # 이후에는 맵 프레임의 점을 사용합니다.
            curr_point = (cloudPoints_list[i][0], cloudPoints_list[i][1])
            if dist_to_next_point < d_max:
                objects_pointcloud_list[-1].append(curr_point)
            else:
                objects_pointcloud_list.append([curr_point])

        위 로직은 아래의 벡터화된 코드로 대체했습니다.
        """

        d_maxs = ranges * div_const + 3 * sigma

        # 라이다 프레임에서 연속 점 사이의 거리를 계산합니다.
        dists = np.linalg.norm(np.diff(xyz_laser_frame[:2, :], axis=1), axis=0)

        # 새로운 객체가 시작되는 인덱스를 찾습니다.
        new_object_indices = np.where(dists >= d_maxs[:-1])[0] + 1

        # new_object_indices를 기준으로 포인트 클라우드를 나눕니다.
        split_indices = np.split(np.arange(len(cloudPoints_list)), new_object_indices)
        objects_pointcloud_list = [np.array(cloudPoints_list)[indices].tolist() for indices in split_indices]

        # ------------------------------------------------
        # 1단계: 중심선 근접도, 최소 크기, 최대 시야 거리로 후보군을 필터링합니다.
        # 이 단계에서는 트랙의 실제 폭 대신, 중심선 기준의 고정된 거리(0.5m)를 사용합니다.
        x_points = []
        y_points = []
        for obs in objects_pointcloud_list:
            mean_x_pos = np.mean([point[0] for point in obs])
            mean_y_pos = np.mean([point[1] for point in obs])
            x_points.append(mean_x_pos)
            y_points.append(mean_y_pos)
        s_points, d_points = self.frenet_converter.get_frenet(np.array(x_points), np.array(y_points))

        filtered_indices = []
        for idx, obj in enumerate(objects_pointcloud_list):
            s = s_points[idx]
            d = d_points[idx]
            # 전방 시야 거리, 최소 포인트 수, 중심선 근접도 조건을 확인합니다.
            if (len(obj) >= self.min_obs_size and
                    normalize_s(s - car_s, self.track_length) <= self.max_viewing_distance and
                    abs(d) <= self.centerline_filter_distance):
                filtered_indices.append(idx)

        objects_pointcloud_list = [objects_pointcloud_list[i] for i in filtered_indices]

        if self.plot_debug:
            markers_array = []
            for idx, object in enumerate(objects_pointcloud_list):
                # 첫 번째 포인트
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.id = idx*10
                marker.type = Marker.SPHERE
                marker.scale.x = 0.25
                marker.scale.y = 0.25
                marker.scale.z = 0.25
                marker.color.a = 0.5
                marker.color.g = 1.
                marker.color.r = 0.
                marker.color.b = idx/len(objects_pointcloud_list)
                marker.pose.position.x = object[0][0]
                marker.pose.position.y = object[0][1]
                marker.pose.orientation.w = 1.
                markers_array.append(marker)

                # 마지막 포인트
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.id = idx*10+2
                marker.type = Marker.SPHERE
                marker.scale.x = 0.25
                marker.scale.y = 0.25
                marker.scale.z = 0.25
                marker.color.a = 0.5
                marker.color.g = 1.
                marker.color.r = 0.
                marker.color.b = idx/len(objects_pointcloud_list)
                marker.pose.position.x = object[-1][0]
                marker.pose.position.y = object[-1][1]
                marker.pose.orientation.w = 1.
                markers_array.append(marker)
            # RViz에서 마커가 깜박일 수 있지만 알고리즘에는 큰 영향을 주지 않습니다.
            self.breakpoints_markers_pub.publish(self.clearmarkers())
            if len(markers_array) > 0:
                self.breakpoints_markers_pub.publish(MarkerArray(markers=markers_array))

        return objects_pointcloud_list

    def obsPointClouds2obsArray(self, objects_pointcloud_list):
        current_obstacle_array = []
        min_dist = self.min_2_points_dist
        for obstacle in objects_pointcloud_list:
            # --- 데이터 포인트에 사각형을 맞춥니다. ---
            theta = np.linspace(0, np.pi/2-np.pi/180, 90)
            cos_theta = np.cos(theta)
            sin_theta = np.sin(theta)
            distance1 = np.dot(obstacle, [cos_theta, sin_theta])
            distance2 = np.dot(obstacle, [-sin_theta, cos_theta])
            D10 = -distance1 + np.amax(distance1, axis=0)
            D11 = distance1 - np.amin(distance1, axis=0)
            D20 = -distance2 + np.amax(distance2, axis=0)
            D21 = distance2 - np.amin(distance2, axis=0)
            min_array = np.argmin(
                [np.linalg.norm(D10, axis=0), np.linalg.norm(D11, axis=0)], axis=0)
            D10 = np.transpose(D10)
            D11 = np.transpose(D11)
            D10[min_array == 1] = D11[min_array == 1]
            D10 = np.transpose(D10)
            min_array = np.argmin(
                [np.linalg.norm(D20, axis=0), np.linalg.norm(D21, axis=0)], axis=0)
            D20 = np.transpose(D20)
            D21 = np.transpose(D21)
            D20[min_array == 1] = D21[min_array == 1]
            D20 = np.transpose(D20)
            D = np.minimum(D10, D20)
            D[D < min_dist] = min_dist

            # --------------------------------------------
            # 장애물이 정사각형이라고 가정하고 중심을 계산합니다.
            # --------------------------------------------

            theta_opt = np.argmax(np.sum(np.reciprocal(D), axis=0))*np.pi/180
            distances1 = np.dot(
                obstacle, [np.cos(theta_opt), np.sin(theta_opt)])
            distances2 = np.dot(
                obstacle, [-np.sin(theta_opt), np.cos(theta_opt)])
            max_dist1 = np.max(distances1)
            min_dist1 = np.min(distances1)
            max_dist2 = np.max(distances2)
            min_dist2 = np.min(distances2)

            # 모서리들은 시계 반대 방향으로 검출됩니다.
            corner1 = None
            corner2 = None
            # 장애물 감지가 세로 방향으로 더 많을 때
            if (np.var(distances2) > np.var(distances1)):
                if (np.linalg.norm(-distances1+max_dist1) < np.linalg.norm(distances1-min_dist1)):
                    # 감지 지점이 오른쪽 경계에 더 가깝습니다.
                    # 오른쪽 아래 모서리
                    corner1 = np.array([np.cos(theta_opt)*max_dist1-np.sin(theta_opt)*min_dist2,
                                        np.sin(theta_opt)*max_dist1+np.cos(theta_opt)*min_dist2])
                    # 오른쪽 위 모서리
                    corner2 = np.array([np.cos(theta_opt)*max_dist1-np.sin(theta_opt)*max_dist2,
                                        np.sin(theta_opt)*max_dist1+np.cos(theta_opt)*max_dist2])
                else:
                    # 감지 지점이 왼쪽 경계에 더 가깝습니다.
                    # 왼쪽 위 모서리
                    corner1 = np.array([np.cos(theta_opt)*min_dist1-np.sin(theta_opt)*max_dist2,
                                        np.sin(theta_opt)*min_dist1+np.cos(theta_opt)*max_dist2])
                    # 왼쪽 아래 모서리
                    corner2 = np.array([np.cos(theta_opt)*min_dist1-np.sin(theta_opt)*min_dist2,
                                        np.sin(theta_opt)*min_dist1+np.cos(theta_opt)*min_dist2])
            else:  # 장애물 감지가 가로 방향으로 더 많을 때
                if (np.linalg.norm(-distances2+max_dist2) < np.linalg.norm(distances2-min_dist2)):
                    # 감지 지점이 상단 경계에 더 가깝습니다.
                    # 오른쪽 위 모서리
                    corner1 = np.array([np.cos(theta_opt)*max_dist1-np.sin(theta_opt)*max_dist2,
                                        np.sin(theta_opt)*max_dist1+np.cos(theta_opt)*max_dist2])
                    # 왼쪽 위 모서리
                    corner2 = np.array([np.cos(theta_opt)*min_dist1-np.sin(theta_opt)*max_dist2,
                                        np.sin(theta_opt)*min_dist1+np.cos(theta_opt)*max_dist2])
                else:
                    # 감지 지점이 하단 경계에 더 가깝습니다.
                    # 왼쪽 아래 모서리
                    corner1 = np.array([np.cos(theta_opt)*min_dist1-np.sin(theta_opt)*min_dist2,
                                        np.sin(theta_opt)*min_dist1+np.cos(theta_opt)*min_dist2])
                    # 오른쪽 아래 모서리
                    corner2 = np.array([np.cos(theta_opt)*max_dist1-np.sin(theta_opt)*min_dist2,
                                        np.sin(theta_opt)*max_dist1+np.cos(theta_opt)*min_dist2])
            # corner1에서 corner2로 가는 벡터
            colVec = np.array([corner2[0]-corner1[0], corner2[1]-corner1[1]])
            # 위 벡터에 수직인 벡터
            orthVec = np.array([-colVec[1], colVec[0]])
            # 중심 위치
            center = corner1 + 0.5*colVec + 0.5*orthVec

            current_obstacle_array.append(
                Obstacle(center[0], center[1], np.linalg.norm(colVec), theta_opt))

        self.get_logger().debug(
            f"[Opponent Detection] detected {len(current_obstacle_array)} raw obstacles.")

        return current_obstacle_array

    def checkObstacles(self, current_obstacles):
        """
        너무 크거나 트랙을 벗어난 장애물을 제거합니다.
        """
        self.tracked_obstacles.clear()
        if not current_obstacles:
            return

        remove_list = []
        
        # 모든 장애물의 s, d 좌표를 한 번에 계산합니다.
        x_centers = [obs.center_x for obs in current_obstacles]
        y_centers = [obs.center_y for obs in current_obstacles]
        frenet_coords = self.frenet_converter.get_frenet(np.array(x_centers), np.array(y_centers))
        
        # Frenet 변환 결과를 정규화합니다.
        if frenet_coords.ndim == 1:
            s_points = np.array([frenet_coords[0]])
            d_points = np.array([frenet_coords[1]])
        else:
            s_points = frenet_coords[0]
            d_points = frenet_coords[1]

        for i, obs in enumerate(current_obstacles):
            # 1. 크기 검증
            if obs.size > self.max_obs_size:
                if obs not in remove_list:
                    remove_list.append(obs)
                continue

            # 2. 트랙 경계 검증
            s = s_points[i]
            d = d_points[i]
            
            # s 값에 해당하는 트랙 폭 인덱스를 찾습니다.
            idx = bisect_left(self.s_array, s)
            if idx: # if idx is not 0
                idx -= 1
            
            track_width_left = self.d_left_array[idx]
            track_width_right = self.d_right_array[idx]

            # 안전 마진을 적용하여 유효 트랙 폭을 계산합니다.
            effective_track_width_left = track_width_left - self.track_boundary_margin
            effective_track_width_right = -track_width_right + self.track_boundary_margin

            # 장애물의 좌/우 경계를 프레네 좌표계에서 계산합니다.
            obs_d_left = d + obs.size / 2
            obs_d_right = d - obs.size / 2

            is_off_track = obs_d_left >= effective_track_width_left or obs_d_right <= effective_track_width_right

#             # 디버깅 로그 추가
#             self.get_logger().info(f"""[Obstacle Check]
#     - Obstacle ID: {i}
#     - Center (x, y): ({obs.center_x:.2f}, {obs.center_y:.2f})
#     - Frenet (s, d): ({s:.2f}, {d:.2f})
#     - Size: {obs.size:.2f}
#     - Track Width (L, R) at s={s:.2f}: ({track_width_left:.2f}, {track_width_right:.2f})
#     - Margin: {self.track_boundary_margin:.2f}
#     - Effective Boundary (L, R): ({effective_track_width_left:.2f}, {effective_track_width_right:.2f})
#     - Obstacle Edge (L, R): ({obs_d_left:.2f}, {obs_d_right:.2f})
#     - Is Off Track?: {is_off_track}
# """)

            # 장애물이 유효 트랙 경계를 벗어났는지 확인합니다.
            if is_off_track:
                if obs not in remove_list:
                    remove_list.append(obs)
                continue
        
        # 유효한 장애물만 필터링합니다.
        valid_obstacles = [obs for obs in current_obstacles if obs not in remove_list]
        
        self.get_logger().debug(
            f"[Opponent Detection] removed {len(remove_list)} obstacles as they are too big or off-track.")

        for idx, curr_obs in enumerate(valid_obstacles):
            curr_obs.id = idx
            self.tracked_obstacles.append(curr_obs)

        self.get_logger().debug(
            f"[Opponent Detection] tracking {len(self.tracked_obstacles)} obstacles.")

    def publishObstaclesMessage(self, tracked_obstacles):
        obstacles_array_message = ObstacleArray()
        obstacles_array_message.header.stamp = self.get_clock().now().to_msg()
        obstacles_array_message.header.frame_id = "map"

        if not tracked_obstacles:
            self.obstacles_msg_pub.publish(obstacles_array_message)
            return

        x_center = []
        y_center = []
        for obstacle in tracked_obstacles:
            x_center.append(obstacle.center_x)
            y_center.append(obstacle.center_y)

        frenet_result = self.frenet_converter.get_frenet(
            np.array(x_center), np.array(y_center))

        # Frenet 변환 결과 검증
        if frenet_result.shape[0] != 2:
            self.get_logger().warn("Frenet conversion returned invalid shape")
            self.obstacles_msg_pub.publish(obstacles_array_message)
            return

        # 결과를 1D 배열로 정규화
        if frenet_result.ndim == 1:
            # 단일 점: shape (2,) -> (2, 1)
            s_points = np.array([frenet_result[0]])
            d_points = np.array([frenet_result[1]])
        else:
            # 다중 점: shape (2, n)
            s_points = frenet_result[0]
            d_points = frenet_result[1]

        # 배열 크기 검증
        if len(s_points) != len(tracked_obstacles):
            self.get_logger().warn(
                f"Size mismatch: obstacles={len(tracked_obstacles)}, frenet_points={len(s_points)}")
            self.obstacles_msg_pub.publish(obstacles_array_message)
            return

        for idx, obstacle in enumerate(tracked_obstacles):
            # numpy scalar을 python float로 명시적 변환
            s = float(s_points[idx])
            d = float(d_points[idx])

            obsMsg = ObstacleMsg()
            obsMsg.id = obstacle.id
            obsMsg.s_start = float(s - obstacle.size/2)
            obsMsg.s_end = float(s + obstacle.size/2)
            obsMsg.d_left = float(d + obstacle.size/2)
            obsMsg.d_right = float(d - obstacle.size/2)
            obsMsg.s_center = s
            obsMsg.d_center = d
            obsMsg.size = float(obstacle.size)

            obstacles_array_message.obstacles.append(obsMsg)

        self.obstacles_msg_pub.publish(obstacles_array_message)

    def publishObstaclesMarkers(self, tracked_obstacles):
        markers_array = []
        for obs in tracked_obstacles:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = obs.id
            marker.type = Marker.CUBE
            marker.scale.x = obs.size
            marker.scale.y = obs.size
            marker.scale.z = obs.size
            marker.color.a = 0.5
            marker.color.g = 1.
            marker.color.r = 0.
            marker.color.b = 1.
            marker.pose.position.x = obs.center_x
            marker.pose.position.y = obs.center_y
            q = quaternion_from_euler(0, 0, obs.theta)
            marker.pose.orientation.x = q[0]
            marker.pose.orientation.y = q[1]
            marker.pose.orientation.z = q[2]
            marker.pose.orientation.w = q[3]
            markers_array.append(marker)

        # RViz에서 마커가 깜박일 수 있지만 알고리즘에는 큰 영향을 주지 않습니다.
        self.obstacles_marker_pub.publish(self.clearmarkers())
        if len(markers_array) > 0:
            self.obstacles_marker_pub.publish(MarkerArray(markers=markers_array))


    def loop(self):
        """
        노드의 메인 루프입니다.
        """
        if self.laser_scans is None or self.car_s is None:
            return

        scans = self.laser_scans
        car_x = self.car_global_x
        car_y = self.car_global_y
        car_yaw = self.car_global_yaw
        car_s = self.car_s

        # --- 장애물 감지 ---
        if self.print_debug:
            start_time = time.perf_counter()
        
        objects_pointcloud_list = self.scans2ObsPointCloud(
            car_s=car_s, scans=scans, car_x=car_x, car_y=car_y, car_yaw=car_yaw)
        current_obstacles = self.obsPointClouds2obsArray(objects_pointcloud_list)
        self.checkObstacles(current_obstacles)

        obstacles = list(self.tracked_obstacles)

        self.publishObstaclesMessage(obstacles)
        if self.plot_debug:
            self.publishObstaclesMarkers(obstacles)

        if self.print_debug:
            end_time = time.perf_counter()
            latency = end_time - start_time
            self.get_logger().info(f"Latency checkObstacles: {latency:.4f} seconds")

def main():
    rclpy.init()
    detect = OpponentDetection()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(detect)
    try:
        executor.spin()
    except KeyboardInterrupt:
        detect.get_logger().info("KeyboardInterrupt")


    detect.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
