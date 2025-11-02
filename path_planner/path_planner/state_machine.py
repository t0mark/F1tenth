#!/usr/bin/env python3

import rclpy
import numpy as np
from typing import Tuple
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    ReliabilityPolicy,
    QoSHistoryPolicy,
)
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from f1tenth_icra_race_msgs.msg import ObstacleArray, ObstacleMsg, OTWpntArray, WpntArray, Wpnt
from visualization_msgs.msg import Marker, MarkerArray
from .modules import utils, frenet_conversion, state_helpers
from tf_transformations import euler_from_quaternion
import os

def normalize_s(s, track_length):
    return s % track_length

def time_to_float(time_instant: Time):
    return time_instant.sec + time_instant.nanosec * 1e-9

def create_wpnts_from_np_array(wpnts_x, wpnts_y, wpnts_v, wpnts_s, wpnts_d):
    wpnts = []
    n = len(wpnts_x)
    for i in range(n):
        wpnt = Wpnt()
        wpnt.id = i
        wpnt.x_m = float(wpnts_x[i])
        wpnt.y_m = float(wpnts_y[i])
        wpnt.vx_mps = float(wpnts_v[i])
        wpnt.s_m = float(wpnts_s[i])
        wpnt.d_m = float(wpnts_d[i])
        wpnts.append(wpnt)
    return wpnts


class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')

        # 구독 설정
        # 파라미터에서 웨이포인트 파일을 읽어옵니다.
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
        self.track_length = self.waypoints[-1, waypoint_cols['s_racetraj_m']]
        self.glb_wpnts = create_wpnts_from_np_array(
            self.waypoints_x, self.waypoints_y, self.waypoints_v,
            self.wpnts_s_array, self.wpnts_d_right_array
        )
        self.num_glb_wpnts = len(self.glb_wpnts)

        self.converter = frenet_conversion.FrenetConverter(self.waypoints_x, self.waypoints_y, waypoints_psi)

        # 플롯 및 콘솔 디버깅을 위한 파라미터를 선언합니다.
        self.declare_parameter('plot_debug', True)
        self.plot_debug = self.get_parameter('plot_debug').value
        self.declare_parameter('print_debug', False)
        self.print_debug = self.get_parameter('print_debug').value
        self.declare_parameter("rate_hz", 50)
        self.rate_hz = self.get_parameter("rate_hz").get_parameter_value().integer_value
        self.timer = self.create_timer(1.0 / self.rate_hz, self.main_loop_callback)

        # 기타 파라미터를 선언합니다.
        self.declare_parameter("gb_ego_width_m", 0.15)
        self.gb_ego_width_m = self.get_parameter("gb_ego_width_m").get_parameter_value().double_value
        self.declare_parameter("lateral_width_m", 0.3)
        self.lateral_width_m = self.get_parameter("lateral_width_m").get_parameter_value().double_value
        self.declare_parameter("overtaking_horizon_m", 6.0)
        self.overtaking_horizon_m = self.get_parameter("overtaking_horizon_m").get_parameter_value().double_value
        self.declare_parameter("spline_hyst_timer_sec", 0.3)
        self.spline_hyst_timer_sec = self.get_parameter("spline_hyst_timer_sec").get_parameter_value().double_value
        self.declare_parameter("n_loc_wpnts", 50)
        self.n_loc_wpnts = self.get_parameter("n_loc_wpnts").get_parameter_value().integer_value
        self.declare_parameter("prediction_horizon_sec", 0.5)
        self.prediction_horizon = self.get_parameter("prediction_horizon_sec").get_parameter_value().double_value

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.pose_sub = self.create_subscription(
                Odometry,
                '/odom',
                self.pose_callback,
                qos)
            
            
        self.create_subscription(
            ObstacleArray, '/perception/obstacles', self.obstacle_callback, qos
        )
        self.create_subscription(OTWpntArray, '/planner/avoidance/otwpnts', self.avoidance_cb, qos)

        # 퍼블리셔 설정
        self.state_pub = self.create_publisher(String, '/state_machine/state', qos)
        self.loc_wpnt_pub = self.create_publisher(WpntArray, '/state_machine/local_waypoints', qos)
        if self.plot_debug:
            self.vis_loc_wpnt_pub = self.create_publisher(MarkerArray, '/state_machine/local_waypoints/markers', qos)
            self.state_marker_pub = self.create_publisher(Marker, '/state_machine/state_marker', qos)

        self.car_global_x = None
        self.car_global_y = None
        self.car_global_yaw = None
        self.car_s = None
        self.car_d = None
        self.first_visualization = True

        self.obstacles = []
        self.predicted_obstacles = []  # 예측된 장애물 정보
        self.spline_ttl = 1.0 # 최악의 경우 1초 동안 스플라인을 유지합니다.
        self.spline_ttl_counter = int(self.spline_ttl * self.rate_hz)
        self.avoidance_wpnts = None
        self.last_valid_avoidance_wpnts = None
        self.local_waypoints = WpntArray()

        # 상태 머신 관련 파라미터를 선언합니다.
        self.state_logic = state_helpers.DefaultStateLogic
        
        self.declare_parameter("mode", "head_to_head")
        self.mode = self.get_parameter("mode").get_parameter_value().string_value
        if self.mode == "head_to_head":
            self.state_transition = state_helpers.head_to_head_transition
        elif self.mode == "timetrials":
            self.state_transition = state_helpers.timetrials_transition
        else:
            self.state_transition = state_helpers.dummy_transition
        self.declare_parameter("force_state", "None")
        self.force_state = self.get_parameter("force_state").get_parameter_value().string_value
        self.force_state = state_helpers.string_to_state_type(self.force_state)
        if self.force_state is None:
            self.state = state_helpers.StateType.GB_TRACK
        else:
            self.state = self.force_state


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
        # 차량의 프레네 속도 (vs: 종방향, vd: 횡방향)를 계산합니다.
        vs, vd = self.converter.get_frenet_velocities(
            np.array([pose_msg.twist.twist.linear.x]),
            np.array([pose_msg.twist.twist.linear.y]),
            self.car_global_yaw
        )
        self.car_vs = vs[0]
        self.car_vd = vd[0]


    def obstacle_callback(self, msg):
        if len(msg.obstacles) == 0:
            self.obstacles = []
            self.predicted_obstacles = []
        else:
            # opponent_tracking에서 이미 is_static이 설정되어 있으므로
            # 그 값을 신뢰하고 예측만 수행합니다
            predicted_obstacles = []

            for obs in msg.obstacles:
                # 동적 장애물 경로 예측 (등속 운동 가정)
                if not obs.is_static:
                    dt = self.prediction_horizon
                    s_future = normalize_s(obs.s_center + obs.vs * dt, self.track_length)
                    d_future = obs.d_center + obs.vd * dt

                    # 예측 정보를 딕셔너리로 저장
                    predicted_obs = {
                        'id': obs.id,
                        's_current': obs.s_center,
                        'd_current': obs.d_center,
                        's_predicted': s_future,
                        'd_predicted': d_future,
                        'vs': obs.vs,
                        'vd': obs.vd,
                        'prediction_time': dt
                    }
                    predicted_obstacles.append(predicted_obs)

                if self.print_debug:
                    velocity_magnitude = np.sqrt(obs.vs**2 + obs.vd**2)
                    pred_info = ""
                    if not obs.is_static and len(predicted_obstacles) > 0:
                        pred = predicted_obstacles[-1]
                        pred_info = f" -> pred(s={pred['s_predicted']:.2f}, d={pred['d_predicted']:.2f})"

                    self.get_logger().info(
                        f"Obstacle {obs.id}: vel={velocity_magnitude:.2f} m/s, "
                        f"static={obs.is_static}, s={obs.s_center:.2f}, d={obs.d_center:.2f}{pred_info}"
                    )

            self.obstacles = list(msg.obstacles)  # 이미 분류된 장애물 리스트
            self.predicted_obstacles = predicted_obstacles

    
    def avoidance_cb(self, msg):
        if len(msg.wpnts) > 0:
            self.avoidance_wpnts = msg
            self.spline_ttl_counter = int(self.spline_ttl * self.rate_hz)

    # TODO: Add sectors
    @property
    def _check_ot_sector(self) -> bool:
        # for sector in self.ot_sectors:
        #     if sector['ot_flag']:
        #         if (sector['start'] <= self.car_s / self.waypoints_dist <= (sector['end']+1)):
        #             return True
        # return False
        return True

    @property
    def _check_close_to_raceline(self) -> bool:
        return np.abs(self.car_d) < self.gb_ego_width_m  # [m]
    
    @property
    def _check_ofree(self) -> bool:
        """
        회피 경로가 장애물과 충돌하지 않는지 확인합니다.
        동적 장애물의 경우 예측 위치도 함께 고려합니다.
        """
        o_free = True

        if self.last_valid_avoidance_wpnts is not None:
            horizon = self.overtaking_horizon_m  # Horizon in front of car_s [m]

            for obs in self.obstacles:
                obs_s = obs.s_center
                obs_d = obs.d_center

                # 현재 위치 확인
                dist_to_obj = (obs_s - self.car_s) % self.track_length
                if dist_to_obj < horizon and len(self.last_valid_avoidance_wpnts):
                    # Get d wrt to mincurv from the overtaking line
                    avoid_wpnt_idx = np.argmin(
                        np.array([abs(avoid_s.s_m - obs_s) for avoid_s in self.last_valid_avoidance_wpnts])
                    )
                    ot_d = self.last_valid_avoidance_wpnts[avoid_wpnt_idx].d_m
                    ot_obs_dist = ot_d - obs_d
                    if abs(ot_obs_dist) < self.lateral_width_m:
                        o_free = False
                        break

                # 동적 장애물의 경우 예측 위치도 확인
                if not obs.is_static:
                    # 예측된 장애물 정보 찾기
                    pred_obs = next((p for p in self.predicted_obstacles if p['id'] == obs.id), None)
                    if pred_obs:
                        s_pred = pred_obs['s_predicted']
                        d_pred = pred_obs['d_predicted']

                        dist_to_pred = (s_pred - self.car_s) % self.track_length
                        if dist_to_pred < horizon:
                            # 예측 위치에서의 회피 경로 확인
                            avoid_wpnt_idx_pred = np.argmin(
                                np.array([abs(avoid_s.s_m - s_pred) for avoid_s in self.last_valid_avoidance_wpnts])
                            )
                            ot_d_pred = self.last_valid_avoidance_wpnts[avoid_wpnt_idx_pred].d_m
                            ot_obs_dist_pred = ot_d_pred - d_pred
                            if abs(ot_obs_dist_pred) < self.lateral_width_m:
                                o_free = False
                                break
        else:
            o_free = True
        return o_free    
    
    @property
    def _check_gbfree(self) -> bool:
        gb_free = True
        horizon = self.overtaking_horizon_m  # Horizon in front of car_s [m]

        for obs in self.obstacles:
            obs_s = obs.s_center
            gap = (obs_s - self.car_s) % self.track_length
            if gap < horizon:
                obs_d = obs.d_center
                # Get d wrt to mincurv from the overtaking line
                if abs(obs_d) < self.lateral_width_m:
                    gb_free = False
                    #self.get_logger().info(f"GB_FREE False, obs dist to ot lane: {obs_d} m")
                    break

        return gb_free

    @property
    def _check_static_obstacle_ahead(self) -> bool:
        """
        전방에 정적 장애물이 있는지 확인합니다.
        """
        horizon = self.overtaking_horizon_m

        for obs in self.obstacles:
            if obs.is_static:
                obs_s = obs.s_center
                gap = (obs_s - self.car_s) % self.track_length
                if gap < horizon:
                    obs_d = obs.d_center
                    if abs(obs_d) < self.lateral_width_m:
                        return True
        return False

    def _calculate_overtake_area(self, obstacle: ObstacleMsg, car_s: float, horizon: float) -> Tuple[float, float]:
        """
        장애물 전방의 경로를 따라 좌/우 가용 공간의 총합(면적)을 계산합니다.
        """
        area_left = 0.0
        area_right = 0.0

        # 장애물 s_center를 기준으로 전방 horizon까지의 웨이포인트를 탐색합니다.
        # car_s를 기준으로 하는 것이 아니라, 장애물 s_center를 기준으로 해야 합니다.
        # 장애물 s_center에서 시작하여 horizon만큼 앞까지
        start_s_for_area = obstacle.s_center
        end_s_for_area = normalize_s(start_s_for_area + horizon, self.track_length)

        # 웨이포인트 인덱스 범위 계산
        start_idx = utils.find_closest_index(self.wpnts_s_array, start_s_for_area)
        end_idx = utils.find_closest_index(self.wpnts_s_array, end_s_for_area)

        # 랩어라운드 처리
        if start_idx <= end_idx:
            indices = np.arange(start_idx, end_idx + 1)
        else:
            indices = np.concatenate((np.arange(start_idx, self.gb_max_idx), np.arange(0, end_idx + 1)))

        # 각 웨이포인트에서 가용 공간 계산 및 합산
        for idx in indices:
            wpnt_s = self.wpnts_s_array[idx]
            wpnt_d_left = self.wpnts_d_left_array[idx]
            wpnt_d_right = self.wpnts_d_right_array[idx]

            # 장애물의 d_center를 기준으로 가용 공간 계산
            # 장애물이 차지하는 d 범위: [obstacle.d_center - obstacle.size / 2, obstacle.d_center + obstacle.size / 2]
            # 단순화를 위해 obstacle.d_left, obstacle.d_right 사용

            # 왼쪽 가용 공간: 트랙 왼쪽 경계 - 장애물 왼쪽 경계
            # 장애물이 트랙 왼쪽에 걸쳐있지 않다면, 트랙 왼쪽 경계까지 모두 가용 공간
            available_left = wpnt_d_left - (obstacle.d_center + obstacle.size / 2)
            if available_left > 0:
                area_left += available_left

            # 오른쪽 가용 공간: 장애물 오른쪽 경계 - 트랙 오른쪽 경계
            # 장애물이 트랙 오른쪽에 걸쳐있지 않다면, 트랙 오른쪽 경계까지 모두 가용 공간
            available_right = (obstacle.d_center - obstacle.size / 2) - (-wpnt_d_right)
            if available_right > 0:
                area_right += available_right
        
        return area_left, area_right


    def _more_space(self, obstacle: ObstacleMsg, opp_wpnt_idx: int) -> Tuple[str, float]:
        """
        장애물 위치에서의 좌우 여백(width_left_m, width_right_m)을 직접 비교하여 최적의 회피/추월 방향을 결정합니다.
        """
        # final_waypoints.csv의 width_left_m, width_right_m 값을 직접 사용
        available_left = self.wpnts_d_left_array[opp_wpnt_idx]
        available_right = self.wpnts_d_right_array[opp_wpnt_idx]

        d_apex_left = obstacle.d_center + self.lateral_width_m
        d_apex_right = obstacle.d_center - self.lateral_width_m

        d_apex_left = min(d_apex_left, self.wpnts_d_left_array[opp_wpnt_idx] - self.gb_ego_width_m)
        d_apex_right = max(d_apex_right, -self.wpnts_d_right_array[opp_wpnt_idx] + self.gb_ego_width_m)

        # 단순히 좌우 여백을 비교하여 더 넓은 쪽 선택
        if available_left < available_right:
            return "left", d_apex_left
        else:
            return "right", d_apex_right

    def _get_closest_dynamic_obstacle(self):
        """
        전방의 가장 가까운 동적 장애물을 반환합니다.
        """
        horizon = self.overtaking_horizon_m
        closest_obs = None
        min_gap = float('inf')

        for obs in self.obstacles:
            if not obs.is_static:
                obs_s = obs.s_center
                gap = (obs_s - self.car_s) % self.track_length
                if gap < horizon and gap < min_gap:
                    obs_d = obs.d_center
                    if abs(obs_d) < self.lateral_width_m:
                        closest_obs = obs
                        min_gap = gap

        return closest_obs, min_gap

    @property
    def _check_need_overtake(self) -> bool:
        """
        추월이 필요한지 판단합니다.
        - 정적 장애물: 즉시 True
        - 동적 장애물: 상대 속도가 음수이거나 목표 속도보다 현저히 낮으면 True
        """
        horizon = self.overtaking_horizon_m

        for obs in self.obstacles:
            obs_s = obs.s_center
            gap = (obs_s - self.car_s) % self.track_length

            if gap < horizon:
                obs_d = obs.d_center
                if abs(obs_d) < self.lateral_width_m:
                    # 정적 장애물: 즉시 추월 필요
                    if obs.is_static:
                        return True

                    # 동적 장애물: 상대 속도 확인
                    # 상대 차량의 종방향 속도가 현저히 느리면 추월 필요
                    target_velocity = self.gb_vmax * 0.7  # 목표 속도의 70%
                    if obs.vs < target_velocity:
                        return True

        return False

    @property
    def _check_availability_spline_wpts(self) -> bool:
        if self.avoidance_wpnts is None:
            return False
        elif len(self.avoidance_wpnts.wpnts) == 0:
            return False
        # Say no to the ot line if the last switch was less than 0.75 seconds ago
        elif (
            abs(time_to_float(self.avoidance_wpnts.header.stamp) - time_to_float(self.avoidance_wpnts.last_switch_time))
            < self.spline_hyst_timer_sec
        ):
            return False
        else:
            # If the splines are valid update the last valid ones
            self.last_valid_avoidance_wpnts = self.avoidance_wpnts.wpnts.copy()
            return True
        
    def get_spline_wpts(self) -> WpntArray:
        """
        Obtain the waypoints by fusing those obtained by spliner with the
        global ones.
        """
        spline_glob = self.glb_wpnts.copy()

        # Handle wrapping
        if self.last_valid_avoidance_wpnts is not None and len(self.last_valid_avoidance_wpnts) > 0:
            s_start_idx = utils.find_closest_index(
                self.wpnts_s_array,
                self.last_valid_avoidance_wpnts[0].s_m,
            )
            s_end_idx = utils.find_closest_index(
                self.wpnts_s_array,
                self.last_valid_avoidance_wpnts[-1].s_m,
            )
            if self.last_valid_avoidance_wpnts[-1].s_m > self.last_valid_avoidance_wpnts[0].s_m:
                spline_idxs = [s for s in range(s_start_idx, s_end_idx + 1)]
            else:
                # Wrap around the track
                spline_idxs = [s for s in range(s_start_idx, self.gb_max_idx)] +\
                                [s for s in range(0, s_end_idx + 1)]

            # Get the spline waypoints
            # s 좌표 기반으로 가장 가까운 회피 waypoint를 글로벌 경로에 매핑
            # 글로벌 waypoint(~0.03m 간격)가 회피 waypoint(~0.25m 간격)보다 8배 촘촘하므로
            # 각 글로벌 위치에 가장 가까운 회피 waypoint를 할당
            for idx in spline_idxs:
                # 현재 글로벌 waypoint의 s 좌표
                current_s = self.wpnts_s_array[idx]

                # 가장 가까운 회피 waypoint 찾기 (s 좌표 기반 nearest neighbor)
                closest_avoid_idx = np.argmin(
                    np.abs([wpnt.s_m - current_s for wpnt in self.last_valid_avoidance_wpnts])
                )

                # 해당 회피 waypoint로 교체
                spline_glob[idx] = self.last_valid_avoidance_wpnts[closest_avoid_idx]

        # If the last valid points have been reset, then we just pass the global waypoints
        else:
            self.get_logger().warn(f"No valid avoidance waypoints, passing global waypoints")
            pass

        return spline_glob


    def visualize_state(self, state: state_helpers.StateType):
        """
        Function that visualizes the state of the car by displaying a colored cube in RVIZ.

        Parameters
        ----------
        action
            Current state of the car to be displayed
        """
        if self.first_visualization:
            self.first_visualization = False
            x0 = self.glb_wpnts[0].x_m
            y0 = self.glb_wpnts[0].y_m
            x1 = self.glb_wpnts[1].x_m
            y1 = self.glb_wpnts[1].y_m
            # compute normal vector of 125% length of trackboundary but to the left of the trajectory
            xy_norm = (
                -np.array([y1 - y0, x0 - x1]) / np.linalg.norm([y1 - y0, x0 - x1]) * 1.25 * self.glb_wpnts[0].d_left
            )

            self.x_viz = x0 + xy_norm[0]
            self.y_viz = y0 + xy_norm[1]

        mrk = Marker()
        mrk.type = mrk.SPHERE
        mrk.id = int(1)
        mrk.header.frame_id = "map"
        mrk.header.stamp = self.get_clock().now().to_msg()
        mrk.color.a = 1.0
        mrk.color.g = 1.0
        mrk.pose.position.x = float(self.x_viz)
        mrk.pose.position.y = float(self.y_viz)
        mrk.pose.position.z = 0.0
        mrk.pose.orientation.w = 1.0
        mrk.scale.x = 1.0
        mrk.scale.y = 1.0
        mrk.scale.z = 1.0

        # Set color and log info based on the state of the car
        if state == state_helpers.StateType.GB_TRACK:
            mrk.color.g = 1.0
        elif state == state_helpers.StateType.OVERTAKE:
            mrk.color.r = 1.0
            mrk.color.g = 1.0
            mrk.color.b = 1.0
        elif state == state_helpers.StateType.TRAILING:
            mrk.color.r = 0.0
            mrk.color.g = 0.0
            mrk.color.b = 1.0
        self.state_marker_pub.publish(mrk)


    def publish_local_waypoints(self, local_wpnts: WpntArray):
        loc_markers = MarkerArray()
        loc_wpnts = local_wpnts

        # set stamp to now         
        loc_wpnts.header.stamp = self.get_clock().now().to_msg()
        loc_wpnts.header.frame_id = "map"

        # Publish the local waypoints
        if len(loc_wpnts.wpnts) == 0:
            self.get_logger().warn("No local waypoints published...")
        else:
            self.loc_wpnt_pub.publish(loc_wpnts)

        if self.plot_debug:
            for i, wpnt in enumerate(loc_wpnts.wpnts):
                mrk = Marker()
                mrk.header.frame_id = "map"
                mrk.type = mrk.SPHERE
                mrk.scale.x = 0.15
                mrk.scale.y = 0.15
                mrk.scale.z = 0.15
                mrk.color.a = 1.0
                mrk.color.g = 1.0

                mrk.id = i
                mrk.pose.position.x = wpnt.x_m
                mrk.pose.position.y = wpnt.y_m
                mrk.pose.position.z = 0.0
                mrk.pose.orientation.w = 1.0
                loc_markers.markers.append(mrk)

            self.vis_loc_wpnt_pub.publish(loc_markers)


    def main_loop_callback(self):
        if self.car_global_x is None or self.car_global_y is None:
            self.get_logger().warn("No pose received yet")
            return

        # transition logic
        if self.force_state:
            self.state = self.force_state
        else:
            self.state = self.state_transition(self)

        msg = String()
        msg.data = str(self.state)
        self.state_pub.publish(msg)
        if self.plot_debug:
            self.visualize_state(state=self.state)

        self.local_waypoints.wpnts = self.state_logic(self)
        self.publish_local_waypoints(self.local_waypoints)

        if self.mode == "head_to_head":
            self.spline_ttl_counter -= 1
            # Once ttl has reached 0 we overwrite the avoidance waypoints with the empty waypoints
            if self.spline_ttl_counter <= 0:
                self.last_valid_avoidance_wpnts = None
                self.avoidance_wpnts = WpntArray()
                self.spline_ttl_counter = -1


def main(args=None):
    rclpy.init(args=args)
    print("State Machine Initialized")
    node = StateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
