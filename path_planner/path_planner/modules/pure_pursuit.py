#!/usr/bin/env python3

import logging
import numpy as np
from .utils import find_closest_index
from .state_helpers import StateType

class PP_Controller:
    """자율주행을 위한 순수 추종(Pure Pursuit) 컨트롤러를 구현합니다.
    입출력 토픽은 컨트롤러 매니저에서 관리합니다.
    """

    def __init__(self, 
                t_clip_min,
                t_clip_max,
                m_l1,
                q_l1,
                speed_lookahead,
                lat_err_coeff,
                speed_lookahead_for_steer,

                trailing_gap,
                trailing_p_gain,
                trailing_d_gain,

                wheelbase = 0.15875 + 0.17145, # F1Tenth 기준
                
                logger_info = logging.info,
                logger_warn = logging.warn
            ):
        
        # 매니저에서 전달되는 파라미터
        self.t_clip_min = t_clip_min
        self.t_clip_max = t_clip_max
        self.m_l1 = m_l1
        self.q_l1 = q_l1
        self.speed_lookahead = speed_lookahead
        self.lat_err_coeff = lat_err_coeff
        self.speed_lookahead_for_steer = speed_lookahead_for_steer

        self.trailing_gap = trailing_gap
        self.trailing_p_gain = trailing_p_gain
        self.trailing_d_gain = trailing_d_gain

        # lwb = lf + lr, lf는 전륜, lr은 후륜입니다.
        # F1Tenth의 경우 lf = 0.15875, lr = 0.17145입니다.
        self.wheelbase = wheelbase

        # 컨트롤러 내부 상태 파라미터
        self.curr_steering_angle = 0
        self.idx_nearest_waypoint = None  # 차량에 가장 가까운 웨이포인트 인덱스
        self.track_length = None

        self.gap = None
        self.gap_should = None
        self.gap_error = None
        self.gap_actual = None
        self.v_diff = None
        self.trailing_command = 2
        self.speed_command = None
        self.d_vs = np.zeros(10)
        self.acceleration_command = 0
                
        self.logger_info = logger_info
        self.logger_warn = logger_warn

    # 메인 루프
    def main_loop(self, state, position_in_map, waypoint_array_in_map, speed_now, opponent, position_in_map_frenet, track_length):
        # 매니저로부터 전달된 값을 갱신합니다.
        self.state = state
        self.position_in_map = position_in_map
        self.waypoint_array_in_map = waypoint_array_in_map
        self.speed_now = speed_now
        self.opponent = opponent
        self.position_in_map_frenet = position_in_map_frenet
        self.track_length = track_length
        ## 전처리 ##
        # 속도 벡터
        yaw = self.position_in_map[2]
        v = [np.cos(yaw)*self.speed_now, np.sin(yaw)*self.speed_now] 

        lat_e_norm, lateral_error = self.calc_lateral_error_norm()

        ### 종방향 제어 ###
        self.speed_command = self.calc_speed_command(v, lat_e_norm)
        speed = self.speed_command
    
        ### 횡방향 제어 ###
        steering_angle = None
        L1_point, L1_distance = self.calc_L1_point(lateral_error)
        
        if L1_point.any() is not None: 
            steering_angle = self.calc_steering_angle(L1_point, L1_distance, yaw, lat_e_norm, v)
        else: 
            raise Exception("L1_point is None")
        
        return speed, steering_angle, L1_point, L1_distance, self.idx_nearest_waypoint
    
    def calc_steering_angle(self, L1_point, L1_distance, yaw, lat_e_norm, v):
        """
        L1 포인트, 목표 횡가속도, 속도를 바탕으로 조향각을 계산합니다.

        Inputs:
            L1_point: 차량 전방 L1 거리 지점
            L1_distance: 차량과 L1 포인트 사이 거리
            yaw: 차량의 요 각도
            lat_e_norm: 정규화된 횡방향 오차
            v : 속도 벡터

        Returns:
            steering_angle: 계산된 조향각
        """
        # 조향 지연을 보정하기 위한 루크어헤드 처리
        if self.state == StateType.TRAILING and (self.opponent is not None):
            speed_la_for_lu = self.speed_now
        else:
            adv_ts_st = self.speed_lookahead_for_steer
            la_position_steer = [self.position_in_map[0] + v[0]*adv_ts_st, self.position_in_map[1] + v[1]*adv_ts_st]
            idx_la_steer = self.nearest_waypoint(la_position_steer, self.waypoint_array_in_map[:, :2])
            speed_la_for_lu = self.waypoint_array_in_map[idx_la_steer, 2]
        speed_for_lu = self.speed_adjust_lat_err(speed_la_for_lu, lat_e_norm)

        L1_vector = np.array([L1_point[0] - self.position_in_map[0], L1_point[1] - self.position_in_map[1]])
        if np.linalg.norm(L1_vector) == 0:
            self.logger_warn("[Controller] norm of L1 vector was 0, eta is set to 0")
            eta = 0
        else:
            eta = np.arcsin(np.dot([-np.sin(yaw), np.cos(yaw)], L1_vector)/np.linalg.norm(L1_vector))

        steering_angle = np.arctan(2*self.wheelbase*np.sin(eta)/L1_distance)

        self.curr_steering_angle = steering_angle
        return steering_angle

    def calc_L1_point(self, lateral_error):
        """
        L1 포인트와 거리를 계산합니다.
        
        Inputs:
            lateral_error: 차량 위치에서 최근접 웨이포인트까지의 프레네 d 거리
        Returns:
            L1_point: 차량 전방 L1 거리 지점
            L1_distance: L1 포인트까지의 거리
        """
        
        self.idx_nearest_waypoint = self.nearest_waypoint(self.position_in_map[:2], self.waypoint_array_in_map[:, :2]) 
        
        if np.isnan(self.idx_nearest_waypoint): 
            self.idx_nearest_waypoint = 0

        # L1 가이던스를 계산합니다.
        L1_distance = self.q_l1 + self.speed_now * self.m_l1
        if self.state == StateType.OVERTAKE and (self.opponent is not None):
            # 선두 차량이 앞에 있으면 상대와의 거리를 사용합니다.
            L1_distance = np.clip(self.opponent[0] - self.position_in_map_frenet[0] - 0.5, 1.0, L1_distance)

        lower_bound = self.t_clip_min
        L1_distance = np.clip(L1_distance, lower_bound, self.t_clip_max)

        L1_point = self.waypoint_at_distance_before_car(L1_distance, self.waypoint_array_in_map, self.idx_nearest_waypoint)
        return L1_point, L1_distance
    
    def calc_speed_command(self, v, lat_e_norm):
        """
        속도 계산을 메인 루프에서 분리해 처리합니다.
        
        Inputs:
            v: 속도 벡터
            lat_e_norm: 정규화된 횡방향 오차
        Returns:
            speed_command: mux로 보낼 수 있는 조정된 속도 명령
        """

        # 속도 지연을 보상하기 위한 루크어헤드 계산
        adv_ts_sp = self.speed_lookahead
        la_position = [self.position_in_map[0] + v[0]*adv_ts_sp, self.position_in_map[1] + v[1]*adv_ts_sp]
        idx_la_position = self.nearest_waypoint(la_position, self.waypoint_array_in_map[:, :2])
        
        # 웨이포인트에서 직접 목표 속도를 가져옵니다.
        global_speed = self.waypoint_array_in_map[idx_la_position, 2]

        if(self.state == StateType.TRAILING and (self.opponent is not None)):  # 추종 제어기
            speed_command = self.trailing_controller(global_speed)
        else:
            speed_command = global_speed

        speed_command = self.speed_adjust_lat_err(speed_command, lat_e_norm)

        return speed_command
    
    def trailing_controller(self, global_speed):
        """
        선두 차량을 일정 거리에서 추종하도록 속도를 조정합니다.
        Inputs:
            speed_command: 전역 레이싱 라인의 속도
            self.opponent: 상대 차량의 프레네 s 위치와 속도
            self.position_in_map_frenet: 자차의 프레네 s 위치와 속도
        Returns:
            trailing_command: 추종용 기준 속도
        """

        self.gap = (self.opponent[0] - self.position_in_map_frenet[0])%self.track_length  # 상대 차량과의 거리
        self.gap_actual = self.gap
        self.gap_should = self.trailing_gap
        self.gap_error = self.gap_should - self.gap_actual
        self.v_diff =  self.position_in_map_frenet[2] - self.opponent[2]
    
        p_value = self.gap_error * self.trailing_p_gain
        d_value = self.v_diff * self.trailing_d_gain

        
        self.trailing_command = np.clip(self.opponent[2] - p_value - d_value, 0, global_speed) 

        # 상대 차량이 보이지 않으면 전역 속도를 사용합니다.
        if not self.opponent[3] and self.gap_actual > self.gap_should:
            self.trailing_command = global_speed

        return self.trailing_command

    def calc_lateral_error_norm(self):
        """
        횡방향 오차를 계산합니다.

        Returns:
            lat_e_norm: 정규화된 횡방향 오차
            lateral_error: 차량 위치와 최근접 웨이포인트 사이의 거리
        """
        lateral_error = abs(self.position_in_map_frenet[1])  # 프레네 좌표 d

        max_lat_e = 0.5
        min_lat_e = 0.
        lat_e_clip = np.clip(lateral_error, a_min=min_lat_e, a_max=max_lat_e)
        lat_e_norm = ((lat_e_clip - min_lat_e) / (max_lat_e - min_lat_e))
        return lat_e_norm, lateral_error

    def speed_adjust_lat_err(self, global_speed, lat_e_norm):
        """
        횡방향 오차와 곡률을 고려해 전역 속도를 줄입니다.
        lat_e_coeff 값으로 감속 비중을 조절합니다.
        lat_e_coeff = 0: 횡방향 오차를 무시
        lat_e_coaff = 1: 횡방향 오차를 최대 반영

        Returns:
            global_speed: 목표 속도
        """
        # 횡방향 오차와 곡률 기반 감속
        # TODO: 추가 튜닝 여지
        if self.speed_now < 1.0:
            return global_speed
        lat_e_coeff = self.lat_err_coeff  # [0, 1] 범위여야 합니다.
        global_speed *= (1 - lat_e_coeff + lat_e_coeff*np.exp(-lat_e_norm))
        return global_speed
    
    def speed_adjust_heading(self, speed_command):
        """
        헤딩 오차를 기준으로 속도를 줄입니다.
        지도 헤딩과 실제 헤딩 차이가 20도 이상이면 최대 0.5배까지 선형으로 감속합니다.
        
        Returns:
            global_speed: 목표 속도
        """

        heading = self.position_in_map[0,2]
        map_heading = self.waypoint_array_in_map[self.idx_nearest_waypoint, 4]
        if abs(heading - map_heading) > np.pi:  # 래핑 문제를 해결합니다.
            heading_error = 2*np.pi - abs(heading- map_heading)
        else:
            heading_error = abs(heading - map_heading)

        if heading_error < np.pi/9:  # 약 20도까지는 허용합니다.
            return speed_command
        elif heading_error < np.pi/2: 
            scaler = 1 - 0.5* heading_error/(np.pi/2)  # 0.5배까지 선형 감속
        else:
            scaler = 0.5
        self.logger_info(f"[MAP Controller] heading error decreasing velocity by {scaler}")
        return speed_command * scaler
        
    def nearest_waypoint(self, position, waypoints):
        """
        차량에 가장 가까운 웨이포인트 인덱스를 계산합니다.

        Returns:
            차량에 가장 가까운 웨이포인트 인덱스
        """
        distances = np.linalg.norm(waypoints - position, axis=1)
        idx_nearest_waypoint = np.argmin(distances)
        return idx_nearest_waypoint

    def waypoint_at_distance_before_car(self, distance, waypoints, idx_waypoint_behind_car):
        """
        차량 전방 특정 프레네 거리만큼 떨어진 웨이포인트를 계산합니다.

        Returns:
            해당 거리 앞에 위치한 웨이포인트(np.array)
        """
        if distance is None:
            distance = self.t_clip_min
        curr_s = waypoints[idx_waypoint_behind_car, 3]
        target_s = (curr_s + distance) % self.track_length
        idx_waypoint_ahead_car = self.search_closest_arr(waypoints[:, 3], target_s)

        return np.array(waypoints[idx_waypoint_ahead_car, :2])
    
    def search_closest_arr(self, arr, val):
        """
        정렬되지 않은 배열에서도 가장 가까운 값의 인덱스를 찾습니다.
        Args:
            arr: 탐색할 배열
            val: 찾고자 하는 값
        Returns:
            가장 가까운 값의 인덱스
        """
        idx = 0
        for i in range(len(arr)):
            if abs(arr[i] - val) < abs(arr[idx] - val):
                idx = i
        return idx

