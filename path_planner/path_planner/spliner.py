#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    ReliabilityPolicy,
    QoSHistoryPolicy,
)
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor, SetParametersResult, ParameterType
from rclpy.parameter import Parameter
from f1tenth_icra_race_msgs.msg import ObstacleArray, ObstacleMsg, OTWpntArray, WpntArray, Wpnt
from geometry_msgs.msg import PointStamped
from tf_transformations import quaternion_from_euler
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np
import time
from typing import List, Any, Tuple
from scipy.interpolate import InterpolatedUnivariateSpline as Spline
from modules.frenet_conversion import FrenetConverter
from modules import utils
import os


def normalize_s(x, track_length):
    x = x % (track_length)
    return x

class SplineNode(Node):
    def __init__(self):
        super().__init__("spline_node")

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # 플롯 및 콘솔 디버깅을 위한 파라미터를 생성합니다.
        self.declare_parameter('plot_debug', False)
        self.plot_debug = self.get_parameter('plot_debug').value
        self.declare_parameter('print_debug', False)
        self.print_debug = self.get_parameter('print_debug').value

        # 시각화를 위한 퍼블리셔를 생성합니다.
        if self.plot_debug:
            self.closest_obs_pub = self.create_publisher(
                Marker, '/planner/avoidance/considered_OBS', qos)
            self.pub_propagated = self.create_publisher(
                Marker, '/planner/avoidance/propagated_OBS', qos)
            self.mrks_pub = self.create_publisher(
                MarkerArray, '/planner/avoidance/markers', qos)

        self.evasion_pub = self.create_publisher(
            OTWpntArray, '/planner/avoidance/otwpnts', qos)

        # 파라미터에서 웨이포인트 파일 경로를 읽어옵니다.
        self.declare_parameter("waypoint_file", os.path.join('src/path_planner/data', 'final_waypoints.csv'))
        waypoint_file = self.get_parameter("waypoint_file").get_parameter_value().string_value

        self.waypoints = np.genfromtxt(waypoint_file, delimiter=';', skip_header=1)
        waypoint_cols = utils.column_numbers_for_waypoints()

        self.waypoints_x = self.waypoints[:, waypoint_cols['x_ref_m']]
        self.waypoints_y = self.waypoints[:, waypoint_cols['y_ref_m']]
        self.waypoints_v = self.waypoints[:, waypoint_cols['vx_racetraj_mps']]
        self.gb_vmax = np.max(self.waypoints_v)
        waypoints_psi = self.waypoints[:, waypoint_cols['psi_racetraj_rad']]
        # waypoints_psi = np.array(utils.convert_psi(waypoints_psi))
        self.wpnts_d_right_array = self.waypoints[:, waypoint_cols['width_right_m']]
        self.wpnts_d_left_array = self.waypoints[:, waypoint_cols['width_left_m']]
        self.wpnts_s_array = self.waypoints[:, waypoint_cols['s_racetraj_m']]
        self.gb_max_idx = len(self.wpnts_s_array)
        self.kappa_array = self.waypoints[:, waypoint_cols['kappa_racetraj_radpm']]
        
        self.track_length = float(self.wpnts_s_array[-1])

        # 프레네 변환기를 초기화합니다.
        self.converter = FrenetConverter(self.waypoints_x, self.waypoints_y, waypoints_psi)

        self.declare_parameter("publish_rate", 40)
        self.publish_rate = self.get_parameter("publish_rate").get_parameter_value().integer_value
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.create_subscription(
            ObstacleArray, '/perception/obstacles', self.obs_cb, QoSProfile(depth=10))
        
        self.pose_sub = self.create_subscription(
                Odometry,
                '/odom',
                self.pose_callback,
                qos)
            
            
        self.car_global_x = None
        self.car_global_y = None
        self.car_global_yaw = None
        self.car_s = None
        self.car_d = None
        self.car_vs = None
        self.car_vd = None
        self.obstacles = None

            
        self.pre_apex_0 = -4.0
        self.pre_apex_1 = -3.0
        self.pre_apex_2 = -1.5
        self.post_apex_0 = 2.0
        self.post_apex_1 = 3.0
        self.post_apex_2 = 4.0
        self.evasion_dist = 0.5
        self.obs_traj_tresh = 0.35
        self.spline_bound_mindist = 0.0
        self.fixed_pred_time = 0.15
        self.kd_obs_pred = 1.0
        self.lookahead = 7.0

        # 파라미터를 선언하고 값을 읽어옵니다.
        self.declare_parameter("pre_apex_0", self.pre_apex_0)
        self.declare_parameter("pre_apex_1", self.pre_apex_1)
        self.declare_parameter("pre_apex_2", self.pre_apex_2)
        self.declare_parameter("post_apex_0", self.post_apex_0)
        self.declare_parameter("post_apex_1", self.post_apex_1)
        self.declare_parameter("post_apex_2", self.post_apex_2)
        self.declare_parameter("evasion_dist", self.evasion_dist)
        self.declare_parameter("obs_traj_tresh", self.obs_traj_tresh)
        self.declare_parameter("spline_bound_mindist", self.spline_bound_mindist)
        self.declare_parameter("fixed_pred_time", self.fixed_pred_time)
        self.declare_parameter("kd_obs_pred", self.kd_obs_pred)
        self.declare_parameter("lookahead", self.lookahead)
        self.pre_apex_0 = self.get_parameter("pre_apex_0").get_parameter_value().double_value
        self.pre_apex_1 = self.get_parameter("pre_apex_1").get_parameter_value().double_value
        self.pre_apex_2 = self.get_parameter("pre_apex_2").get_parameter_value().double_value
        self.post_apex_0 = self.get_parameter("post_apex_0").get_parameter_value().double_value
        self.post_apex_1 = self.get_parameter("post_apex_1").get_parameter_value().double_value
        self.post_apex_2 = self.get_parameter("post_apex_2").get_parameter_value().double_value
        self.evasion_dist = self.get_parameter("evasion_dist").get_parameter_value().double_value
        self.obs_traj_tresh = self.get_parameter("obs_traj_tresh").get_parameter_value().double_value
        self.spline_bound_mindist = self.get_parameter("spline_bound_mindist").get_parameter_value().double_value
        self.fixed_pred_time = self.get_parameter("fixed_pred_time").get_parameter_value().double_value
        self.kd_obs_pred = self.get_parameter("kd_obs_pred").get_parameter_value().double_value
        self.lookahead = self.get_parameter("lookahead").get_parameter_value().double_value

        # 마지막 전환 시각과 이동 방향을 기록합니다.
        self.last_switch_time = self.get_clock().now().to_msg()
        self.last_ot_side = ""


    def obs_cb(self, msg: ObstacleArray):
        self.obstacles = msg

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
        s, d = self.converter.get_frenet(np.array([self.car_global_x]), np.array([self.car_global_y]))
        self.car_s = normalize_s(s[0], self.track_length)
        self.car_d = d[0]
        vs, vd = self.converter.get_frenet_velocities(np.array([pose_msg.twist.twist.linear.x]), np.array([pose_msg.twist.twist.linear.y]), self.car_global_yaw)

        self.car_vs = vs[0]
        self.car_vd = vd[0]

    def timer_callback(self):
        # 차량 상태가 유효한지 확인합니다.
        if self.car_global_x is None or self.car_global_y is None:
            return
        if self.obstacles is None:
            return

        # 데이터를 샘플링합니다.
        wpnts = OTWpntArray()
        mrks = MarkerArray()

        # 장애물이 있으면 이를 우회하도록 스플라인을 계산합니다.
        if len(self.obstacles.obstacles) > 0:
            wpnts, mrks = self.do_spline(self.obstacles)

        # 마커 배열에 삭제 마커를 추가합니다.
        del_mrk = Marker()
        del_mrk.header.stamp = self.get_clock().now().to_msg()
        del_mrk.action = Marker.DELETEALL
        mrks.markers.insert(0, del_mrk)

        # 웨이포인트와 마커를 퍼블리시합니다.
        self.evasion_pub.publish(wpnts)
        self.mrks_pub.publish(mrks)


    #################### 유틸리티 함수 ####################
    def _predict_obs_movement(self, obs: ObstacleMsg, mode: str = "constant") -> ObstacleMsg:
        """
        현재 상태와 모드에 따라 장애물의 이동을 예측합니다.

        TODO: 모듈화 강화를 위해 상대 예측을 별도 컴포넌트로 분리해야 합니다.

        Args:
            obs (Obstacle): 이동을 예측할 장애물
            mode (str, optional): 예측 모드, 기본값은 "constant"

        Returns:
            Obstacle: 이동이 반영된 장애물 객체
        """
        # 거리 기반 시간으로 상대를 전방으로 예측 이동시킵니다.
        dist_in_front = normalize_s(obs.s_center - self.car_s, self.track_length)
        if dist_in_front < self.lookahead:  # TODO 파라미터화
            # s 좌표계에서의 거리
            rel_speed = np.clip(self.car_vs - obs.vs, 0.1, 10)
            ot_time_distance = np.clip(dist_in_front / rel_speed, 0, 5) * 0.5

            delta_s = ot_time_distance * obs.vs
            delta_d = ot_time_distance * obs.vd
            # delta_d = -(obs.d_center + delta_d) * np.exp(-np.abs(self.kd_obs_pred * obs.d_center))

            # 장애물 위치를 업데이트합니다.
            obs.s_start += delta_s
            obs.s_center += delta_s
            obs.s_end += delta_s
            obs.s_start = normalize_s(obs.s_start, self.track_length)
            obs.s_center = normalize_s(obs.s_center, self.track_length)
            obs.s_end = normalize_s(obs.s_end, self.track_length)

            obs.d_left += delta_d
            obs.d_center += delta_d
            obs.d_right += delta_d

            if self.plot_debug:
                resp = self.converter.get_cartesian([obs.s_center], [obs.d_center])
                marker = self.xy_to_point(resp[0], resp[1], opponent=True)
                self.pub_propagated.publish(marker)

        return obs


    def _more_space(self, obstacle: ObstacleMsg, opp_wpnt_idx: int) -> Tuple[str, float]:
        width_left = self.wpnts_d_left_array[opp_wpnt_idx]
        width_right = self.wpnts_d_right_array[opp_wpnt_idx]
        left_gap = abs(width_left - obstacle.d_left)
        right_gap = abs(width_right + obstacle.d_right)
        min_space = self.evasion_dist + self.spline_bound_mindist

        if right_gap > min_space and left_gap < min_space:
            # 상대 차량 기준 오른쪽으로의 에이펙스 거리를 계산합니다.
            d_apex_right = obstacle.d_right - self.evasion_dist
            # 오른쪽에서 추월하지만 에이펙스가 중심선의 왼쪽이면 0으로 조정합니다.
            if d_apex_right > 0:
                d_apex_right = 0
            return "right", d_apex_right

        elif left_gap > min_space and right_gap < min_space:
            # 상대 차량 기준 왼쪽으로의 에이펙스 거리를 계산합니다.
            d_apex_left = obstacle.d_left + self.evasion_dist
            # 왼쪽에서 추월하지만 에이펙스가 중심선의 오른쪽이면 0으로 조정합니다.
            if d_apex_left < 0:
                d_apex_left = 0
            return "left", d_apex_left
        else:
            candidate_d_apex_left = obstacle.d_left + self.evasion_dist
            candidate_d_apex_right = obstacle.d_right - self.evasion_dist

            if abs(candidate_d_apex_left) < abs(candidate_d_apex_right):
                # 왼쪽 추월인데 에이펙스가 중심선 오른쪽이면 0으로 조정합니다.
                if candidate_d_apex_left < 0:
                    candidate_d_apex_left = 0
                return "left", candidate_d_apex_left
            else:
                # 오른쪽 추월인데 에이펙스가 중심선 왼쪽이면 0으로 조정합니다.
                if candidate_d_apex_right > 0:
                    candidate_d_apex_right = 0
                return "right", candidate_d_apex_right



    def do_spline(self, obstacles: ObstacleArray) -> Tuple[WpntArray, MarkerArray]:
        """
        가장 가까운 장애물을 기준으로 프리·포스트 에이펙스 점을 연결하는 스플라인 회피 경로를 생성합니다.

        레퍼런스 레이싱 라인을 이루는 전역 웨이포인트와 회피 대상 장애물을 입력으로 받아,
        레이싱 라인 근처에 있는 장애물만 선별한 뒤 각 장애물에 대한 회피 경로를 작성합니다.
        회피 경로는 장애물 전후에 떨어진 에이펙스 지점을 스플라인으로 연결해 만들며,
        공간·속도 성분은 `Spline` 클래스를 사용해 계산합니다.

        Args:
        - obstacles (ObstacleArray): 회피 대상 장애물 배열
        - gb_wpnts (WpntArray): 기준 레이싱 라인을 이루는 전역 웨이포인트 목록
        - state (Odometry): 차량의 현재 상태

        Returns:
        - wpnts (WpntArray): 가장 가까운 장애물에 대한 회피 경로 웨이포인트
        - mrks (MarkerArray): 시각화용 마커 배열

        """
        # 회피 경로와 마커를 생성합니다.
        mrks = MarkerArray()
        wpnts = OTWpntArray()

        # 레이싱 라인과 일정 거리 이내의 장애물만 고려합니다.
        close_obs = self._obs_filtering(obstacles=obstacles)

        # 전방 범위 안에 장애물이 있으면 가장 가까운 것을 기준으로 회피 경로를 생성합니다.
        if len(close_obs) > 0:
            # 랩어라운드를 고려해 가장 가까운 장애물을 찾습니다.
            closest_obs = min(
                close_obs, key=lambda obs: normalize_s(obs.s_center - self.car_s, self.track_length)
            )

            # 트랙 경계에서 충분히 떨어진 회피 에이펙스를 계산합니다.
            if closest_obs.s_end < closest_obs.s_start:
                s_apex = (closest_obs.s_end + self.track_length +
                          closest_obs.s_start) / 2
                s_apex = normalize_s(s_apex, self.track_length)
            else:
                s_apex = (closest_obs.s_end + closest_obs.s_start) / 2
            # 전방 약 20개의 전역 웨이포인트(약 2m)를 확인해 코너 외측 방향을 판단합니다.
            obstacle_idx = utils.find_closest_index(self.wpnts_s_array, s_apex)
            gb_idxs = [(obstacle_idx + i) % self.gb_max_idx for i in range(20)]
            kappas = np.array([self.kappa_array[gb_idx] for gb_idx in gb_idxs])
            outside = "left" if np.sum(kappas) < 0 else "right"
            # 적절한 측면을 선택하고 장애물 기준 에이펙스 거리를 계산합니다.
            more_space, d_apex = self._more_space(closest_obs, obstacle_idx)

            # 스플라인 기준점(에이펙스)을 퍼블리시합니다.
            if self.plot_debug:
                mrk = self.xy_to_point(
                    x=self.waypoints_x[obstacle_idx], y=self.waypoints_y[obstacle_idx], opponent=False)
                self.closest_obs_pub.publish(mrk)

            # 전역 경로에서 속도를 포함한 회피 웨이포인트를 선택합니다.
            evasion_points = []
            spline_params = [
                self.pre_apex_0,
                self.pre_apex_1,
                self.pre_apex_2,
                0,
                self.post_apex_0,
                self.post_apex_1,
                self.post_apex_2,
            ]
            for i, dst in enumerate(spline_params):
                # 속도를 최대 속도로 정규화해 1~1.5 배로 선형 스케일링합니다.
                dst = dst * np.clip(1.0 + self.car_vs / self.gb_vmax, 1, 1.5)
                # 외측 추월 시 스플라인을 완만하게 만듭니다.
                if outside == more_space:
                    si = s_apex + dst * 1.75  # TODO make parameter
                else:
                    si = s_apex + dst
                di = d_apex if dst == 0 else 0
                evasion_points.append([si, di])
            # NumPy 배열로 변환합니다.
            evasion_points = np.array(evasion_points)

            # s를 기반으로 d에 대한 공간 스플라인을 계산합니다.
            spline_resolution = 0.25
            spatial_spline = Spline(
                x=evasion_points[:, 0], y=evasion_points[:, 1])
            evasion_s = np.arange(
                evasion_points[0, 0], evasion_points[-1, 0], spline_resolution)
            # d 값을 에이펙스 거리로 제한합니다.
            if d_apex < 0:
                evasion_d = np.clip(spatial_spline(evasion_s), d_apex, 0)
            else:
                evasion_d = np.clip(spatial_spline(evasion_s), 0, d_apex)

            # s 값의 래핑을 처리합니다.
            evasion_s = normalize_s(evasion_s, self.track_length)

            # 스플라인을 프레네 좌표로 변환하고 마커와 웨이포인트를 생성합니다.
            danger_flag = False
            resp = self.converter.get_cartesian(evasion_s, evasion_d)

            # 측면 전환이 가능한지 확인합니다.
            if not self._check_ot_side_possible(more_space):
                danger_flag = True

            for i in range(evasion_s.shape[0]):
                gb_wpnt_i = utils.find_closest_index(self.wpnts_s_array, evasion_s[i])
                # 스플라인이 레이싱 라인 밖으로 벗어났을 때 트랙 경계와의 거리를 확인합니다.
                if abs(evasion_d[i]) > spline_resolution:
                    tb_dist = self.wpnts_d_left_array[gb_wpnt_i] if more_space == "left" else self.wpnts_d_right_array[gb_wpnt_i]
                    # 스플라인이 트랙 경계에 과도하게 근접했는지 검사합니다.
                    if abs(evasion_d[i]) > abs(tb_dist) - self.spline_bound_mindist:
                        self.get_logger().info(
                            "Evasion trajectory too close to TRACKBOUNDS, aborting evasion"
                        )
                        danger_flag = True
                        break
                # 전역 웨이포인트에서 속도를 가져오고 인사이드로 진입하면 감속합니다.
                # TODO: 속도 스케일링을 ROS 파라미터로 노출합니다.
                vi = self.waypoints_v[gb_wpnt_i] if outside == more_space else self.waypoints_v[gb_wpnt_i] * 0.8
                wpnts.wpnts.append(
                    self.xyv_to_wpnts(
                        x=resp[0, i], y=resp[1, i], s=evasion_s[i], d=evasion_d[i], v=vi, wpnts=wpnts)
                )
                mrks.markers.append(self.xyv_to_markers(
                    x=resp[0, i], y=resp[1, i], v=vi, mrks=mrks))

            # 나머지 OT 웨이포인트를 채웁니다.
            wpnts.header.stamp = self.get_clock().now().to_msg()
            wpnts.header.frame_id = "map"
            if not danger_flag:
                wpnts.ot_side = more_space
                wpnts.ot_line = outside
                wpnts.side_switch = True if self.last_ot_side != more_space else False
                wpnts.last_switch_time = self.last_switch_time

                # 마지막 전환 시각과 방향을 갱신합니다.
                if self.last_ot_side != more_space:
                    self.last_switch_time = self.get_clock().now().to_msg()
                self.last_ot_side = more_space
            else:
                wpnts.wpnts = []
                mrks.markers = []
                # 상태 머신의 급격한 전환을 완화하기 위한 조치입니다.
                wpnts.side_switch = True
                self.last_switch_time = self.get_clock().now().to_msg()
                self.last_ot_side = more_space
        return wpnts, mrks


    def _obs_filtering(self, obstacles: ObstacleArray) -> List[ObstacleMsg]:
        # 레이싱 라인과 일정 거리 이내의 장애물만 사용합니다.
        obs_on_traj = [obs for obs in obstacles.obstacles if abs(
            obs.d_center) < self.obs_traj_tresh]

        # 차량 전방 self.lookahead 범위 내 장애물만 고려합니다.
        close_obs = []
        for obs in obs_on_traj:
            obs = self._predict_obs_movement(obs)
            # 래핑을 처리합니다.
            dist_in_front = normalize_s(obs.s_center - self.car_s, self.track_length)
            # dist_in_back = abs(dist_in_front % (-self.track_length)) # 후방 장애물까지의 거리
            if dist_in_front < self.lookahead:
                close_obs.append(obs)
        return close_obs
    
    def _check_ot_side_possible(self, more_space) -> bool:
        # TODO: cur_d 임계값을 ROS 파라미터로 노출합니다.
        if abs(self.car_d) > 4.9 and more_space != self.last_ot_side:
            self.get_logger().info("Can't switch sides, because we are not on the raceline")
            return False
        return True

    #################### 시각화 함수 ####################
    def xyv_to_markers(self, x: float, y: float, v: float, mrks: MarkerArray) -> Marker:
        mrk = Marker()
        mrk.header.frame_id = "map"
        mrk.header.stamp = self.get_clock().now().to_msg()
        mrk.type = mrk.CYLINDER
        mrk.scale.x = 0.1
        mrk.scale.y = 0.1
        mrk.scale.z = float(v / self.gb_vmax)
        mrk.color.a = 1.0
        mrk.color.b = 0.75
        mrk.color.r = 0.75

        mrk.id = len(mrks.markers)
        mrk.pose.position.x = float(x)
        mrk.pose.position.y = float(y)
        mrk.pose.position.z = 0.0
        mrk.pose.orientation.w = 1.0

        return mrk

    def xy_to_point(self, x: float, y: float, opponent=True) -> Marker:
        mrk = Marker()
        mrk.header.frame_id = "map"
        mrk.header.stamp = self.get_clock().now().to_msg()
        mrk.type = mrk.SPHERE
        mrk.scale.x = 0.5
        mrk.scale.y = 0.5
        mrk.scale.z = 0.5
        mrk.color.a = 0.8
        mrk.color.b = 0.65
        mrk.color.r = 1.0 if opponent else 0.0
        mrk.color.g = 0.65

        mrk.pose.position.x = float(x)
        mrk.pose.position.y = float(y)
        mrk.pose.position.z = 0.0
        mrk.pose.orientation.w = 1.0

        return mrk

    def xyv_to_wpnts(self, s: float, d: float, x: float, y: float, v: float, wpnts: OTWpntArray) -> Wpnt:
        wpnt = Wpnt()
        wpnt.id = len(wpnts.wpnts)
        wpnt.x_m = float(x)
        wpnt.y_m = float(y)
        wpnt.s_m = float(s)
        wpnt.d_m = float(d)
        wpnt.vx_mps = float(v)
        return wpnt
    

def main(args=None):
    rclpy.init(args=args)
    node = SplineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    





            
        

        



        
