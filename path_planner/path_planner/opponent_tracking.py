#!/usr/bin/env python3

# ForzaETH ROS 1 추적 코드 대비 주요 변경 사항:
# 1. ROS 1(rospy)에서 ROS 2(rclpy)로 이관하면서 QoS와 파라미터를 재정의했습니다.
# 2. 감지 노드가 프레네 기반 ObstacleArray를 제공한다고 가정하고 `/perception/detection/raw_obstacles`만 구독합니다.
# 3. `/scan`, `/odom` 등 사용하지 않는 구독을 제거해 추적 로직을 단순화했습니다.
# 4. F1TENTH 기본 시나리오에 맞춰 첫 번째 동적 장애물(단일 상대 차량)만 추적합니다.
# 5. `OpponentState`를 재구성하고 EKF를 OpponentTracking 내부에 통합했습니다.
# 6. ROS 2 파라미터 서버를 활용해 노이즈 및 모델 파라미터를 손쉽게 튜닝합니다.
# 7. 감지·맵핑 스택과 일관되도록 `MarkerArray`로 시각화를 제공합니다.
# 8. 예측-갱신-퍼블리시 단계가 명확한 모듈형 구조로 하드웨어·시뮬레이션 모두에 적합합니다.
# -------------------------------------------

"""
상대 추적 노드
==============

이 ROS 2 노드는 인지 스택에서 감지된 단일 상대 차량을 추적합니다.
참조 중심선을 따라가는 아크 길이 ``s``와 횡방향 오프셋 ``d``로 구성된
**프레네 좌표**를 사용해 폐곡선 트랙에서 상대 위치를 표현합니다.
**확장 칼만 필터(EKF)** 는 다음 상태 벡터를 추정합니다.

    x = [s, v_s, d, v_d]^T

여기서
    * ``s``  - 트랙을 따라 진행한 종방향 위치 (m)
    * ``v_s`` - 종방향 속도 (m/s)
    * ``d``  - 중심선으로부터의 횡방향 오프셋 (m)
    * ``v_d`` - 횡방향 속도 (m/s)

예측 단계에서는 두 방향 모두 등속을 가정하고,
관측이 일시적으로 누락되더라도 발산하지 않도록
횡방향 축에 단순 비례 감쇠를 적용합니다.

관측 값은 ``/perception/detection/raw_obstacles``(``ObstacleArray``)로 유입되며,
1 대 1 레이스라는 가정하에 배열의 첫 번째 장애물만 처리합니다.

추정된 상태는 ``/perception/obstacles``(``ObstacleArray``)로 재발행되어
후속 플래너가 사용하며, 시각화용 **Marker**는
``/perception/static_dynamic_marker_pub`` 토픽으로 방송됩니다.
"""

import os

# === 표준 라이브러리 ===
import math
import time
from typing import List
import modules.utils as utils

# === 서드파티 수치 라이브러리 ===
import numpy as np

# === ROS 2 핵심 ===
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    ReliabilityPolicy,
    QoSHistoryPolicy,
)

# === ROS 2 메시지 타입 ===
from nav_msgs.msg import Odometry  # (향후 확장을 위해 유지)
from std_msgs.msg import Float32   # (미사용 – 자리 표시)
from sensor_msgs.msg import LaserScan  # (미사용 – 자리 표시)
from visualization_msgs.msg import Marker, MarkerArray
from f1tenth_icra_race_msgs.msg import ObstacleArray, ObstacleMsg

# === 추정 도구 ===
from filterpy.kalman import ExtendedKalmanFilter as EKF
from filterpy.common import Q_discrete_white_noise
from scipy.linalg import block_diag

# === 로컬 헬퍼 ===
from modules.frenet_conversion import FrenetConverter

# ---------------------------------------------------------------------------
# 유틸리티 함수
# ---------------------------------------------------------------------------

def normalize_s(x,track_length):
    x = x % (track_length)
    if x > track_length/2:
        x -= track_length
    return x
# ---------------------------------------------------------------------------
# 확장 칼만 필터 기반 상대 차량 모델
# ---------------------------------------------------------------------------

class OpponentState:
    """단일 상대 차량에 대한 EKF 인스턴스와 보조 버퍼를 보관합니다."""
    track_length: float

    def __init__(
        self,
        rate: int,
        process_var_vs: float,
        process_var_vd: float,
        meas_var_s: float,
        meas_var_d: float,
        meas_var_vs: float,
        meas_var_vd: float,
        P_vs: float,
        P_d: float,
        P_vd: float,
    ) -> None:
        # --- 모델 상수 ------------------------------------------------
        self.rate = rate
        self.dt = 1.0 / rate  # 샘플링 주기(s)

        # 예측 단계에서 횡방향 감쇠를 위한 단순 P 제어 이득
        self.P_vs = P_vs
        self.P_d = P_d
        self.P_vd = P_vd
        self.size = 0.0
        self.is_visible = False
        self.is_visible_ttl = 5

        # --- 칼만 필터 --------------------------------------------------
        self.kf = EKF(dim_x=4, dim_z=4)

        # 상태 전이 야코비안(두 축 모두 거의 등속을 가정하므로 선형)
        self.kf.F = np.array(
            [
                [1.0, self.dt, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, self.dt],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

        # 공정 잡음 공분산 – 평면상의 독립된 운동
        q1 = Q_discrete_white_noise(dim=2, dt=self.dt, var=process_var_vs)
        q2 = Q_discrete_white_noise(dim=2, dt=self.dt, var=process_var_vd)
        self.kf.Q = block_diag(q1, q2)

        # 측정 공분산 (``s, v_s, d, v_d``)
        self.kf.R = np.diag([meas_var_s, meas_var_vs, meas_var_d, meas_var_vd])

        # 관측 행렬(모든 상태를 직접 관측)
        self.kf.H = np.identity(4)

        # 초기 공분산(정보가 거의 없도록 완만하게 설정)
        self.kf.P *= 5.0

        # --- 런타임 보조 변수 -----------------------------------------------
        self.initialized: bool = False  # 첫 관측 이후 True가 됩니다.
        self.vs_filt: List[float] = [0.0] * 5  # 종방향 이동 평균 버퍼
        self.vd_filt: List[float] = [0.0] * 5  # 횡방향 이동 평균 버퍼

    # ------------------------- 예측 / 업데이트 -------------------------

    def residual_h(a, b):
        y = a-b
        y[0] = normalize_s(y[0], OpponentState.track_length)
        return y
    
    def Hjac(x):
        return np.identity(4)

    def hx(x):
        return np.array([normalize_s(x[0],
                         OpponentState.track_length),x[1], x[2], x[3]])

    def predict(self) -> None:
        """EKF 예측 단계에서 *d*와 *v_d*에 횡방향 감쇠를 적용합니다."""
        # 제어 입력 벡터 ``u`` 는 횡방향 상태에 비례 감쇠를 적용해
        # 몇 프레임 동안 관측이 없어도 필터가 발산하지 않도록 합니다.
        u = np.array([0, 0, -self.P_d * self.kf.x[2], -self.P_vd * self.kf.x[3]])
        self.kf.predict(u=u)
        # 예측 후 ``s`` 가 트랙 길이 범위를 벗어나지 않도록 보정합니다.
        self.kf.x[0] = normalize_s(self.kf.x[0], OpponentState.track_length)

    def update(self, s: float, d: float, vs: float, vd: float) -> None:
        """최신 노이즈 관측을 사용한 EKF 보정 단계입니다."""
        z = np.array([s, vs, d, vd])
        self.kf.update(
            z=z,
            HJacobian=OpponentState.Hjac,
            Hx=OpponentState.hx,
            residual=OpponentState.residual_h,
        )
        self.kf.x[0] = normalize_s(self.kf.x[0], OpponentState.track_length)

        # 속도 출력을 부드럽게 하기 위한 간단한 이동 평균 업데이트
        self.vs_filt.pop(0)
        self.vs_filt.append(self.kf.x[1])
        self.vd_filt.pop(0)
        self.vd_filt.append(self.kf.x[3])

# ---------------------------------------------------------------------------
# ROS 2 노드 구현
# ---------------------------------------------------------------------------

class OpponentTracking(Node):
    """ROS 2 node that wraps :class:`OpponentState` and handles I/O."""

    def __init__(self) -> None:
        super().__init__("opponent_tracking")

        waypoint_file = os.path.join('src/path_planner/data', 'final_waypoints.csv')
        self.declare_parameter("waypoint_file", waypoint_file)
        waypoint_file = self.get_parameter("waypoint_file").get_parameter_value().string_value

        waypoints = np.genfromtxt(waypoint_file, delimiter='; ', skip_header=1)
        waypoint_cols = utils.column_numbers_for_waypoints()

        waypoints_x = waypoints[:, waypoint_cols['x_ref_m']]
        waypoints_y = waypoints[:, waypoint_cols['y_ref_m']]
        waypoints_psi = waypoints[:, waypoint_cols['psi_racetraj_rad']]

        self.track_length = float(waypoints[-1, waypoint_cols['s_racetraj_m']])
        self.converter = FrenetConverter(waypoints_x, waypoints_y, waypoints_psi)



        # ------------------------------------------------------------------
        # QoS 설정 – 최신 메시지만 유지하고 신뢰성을 보장합니다.
        # ------------------------------------------------------------------
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # ----------------------- 파라미터 인터페이스 ----------------------
        # (모두 합리적인 기본값을 가지며 런치 파일에서 재정의할 수 있습니다.) --------
        self.declare_parameter("rate", 40)
        self.rate = (
            self.get_parameter("rate").get_parameter_value().integer_value
        )

        # 노이즈 파라미터 --------------------------------------------------
        self.declare_parameter("process_var_vs", 0.6)
        self.declare_parameter("process_var_vd", 0.6)
        self.declare_parameter("meas_var_s", 0.1)
        self.declare_parameter("meas_var_d", 0.1)
        self.declare_parameter("meas_var_vs", 0.1)
        self.declare_parameter("meas_var_vd", 0.1)
        self.declare_parameter("P_vs", 1.0)
        self.declare_parameter("P_d", 1.0)
        self.declare_parameter("P_vd", 1.0)

    

        # ------------------------------------------------------------------
        # 상대 상태 추정기를 초기화합니다.
        # ------------------------------------------------------------------
        OpponentState.track_length = self.track_length
        self.state = OpponentState(
            rate=self.rate,
            process_var_vs=self.get_parameter("process_var_vs").get_parameter_value().double_value,
            process_var_vd=self.get_parameter("process_var_vd").get_parameter_value().double_value,
            meas_var_s=self.get_parameter("meas_var_s").get_parameter_value().double_value,
            meas_var_d=self.get_parameter("meas_var_d").get_parameter_value().double_value,
            meas_var_vs=self.get_parameter("meas_var_vs").get_parameter_value().double_value,
            meas_var_vd=self.get_parameter("meas_var_vd").get_parameter_value().double_value,
            P_vs=self.get_parameter("P_vs").get_parameter_value().double_value,
            P_d=self.get_parameter("P_d").get_parameter_value().double_value,
            P_vd=self.get_parameter("P_vd").get_parameter_value().double_value,
        )

        # 차분 기반 속도 계산을 위한 이전 ``s`` 관측값
        self.prev_obs_s: float | None = None
        self.prev_obs_d: float | None = None

        # --------------------------- 구독 설정 -------------------------
        self.subscription = self.create_subscription(
            ObstacleArray,
            "/perception/detection/raw_obstacles",
            self.obstacle_callback,
            qos,
        )

        # ---------------------------- 퍼블리셔 ---------------------------
        self.marker_pub = self.create_publisher(
            MarkerArray, "/perception/static_dynamic_marker_pub", qos
        )
        self.obstacles_pub = self.create_publisher(
            ObstacleArray, "/perception/obstacles", qos
        )

        # ------------------------------ 타이머 ------------------------------
        self.timer = self.create_timer(1.0 / self.rate, self.loop)

    # ------------------------------------------------------------------
    # 원시 인지 데이터 콜백
    # ------------------------------------------------------------------
    def obstacle_callback(self, msg: ObstacleArray) -> None:
        """원시 장애물 메시지를 관측값으로 변환해 EKF에 제공합니다."""
        if not msg.obstacles:
            if self.state.is_visible:
                self.state.is_visible_ttl -= 1
                if self.state.is_visible_ttl <= 0:
                    self.state.is_visible = False
                    self.state_is_visible_ttl = 5
            return  # 이번 프레임에는 감지 결과가 없습니다.
        
        self.state.is_visible = True
        self.state.is_visible_ttl = 5

        # 단순화를 위해 중심선에 가장 가까운 장애물만 추적합니다(d_center 사용).
        obs = msg.obstacles[0]        
        for curr_obs in msg.obstacles:
            if abs(curr_obs.d_center) < abs(obs.d_center):
                obs = curr_obs
        s = obs.s_center
        d = obs.d_center

        # 연속 *s* 관측값을 차분해 종방향 속도를 근사합니다.
        # (노이즈가 크지만 EKF가 평활화합니다.)
        vs = 0.0 if self.prev_obs_s is None else (s - self.prev_obs_s) * self.rate
        self.prev_obs_s = s
        if vs < -1 or vs > 8:
            vs = 0.0

        vd = 0.0 if self.prev_obs_d is None else (d - self.prev_obs_d) * self.rate
        self.prev_obs_d = d
        if vd < -2 or vd > 2:
            vd = 0.0

        # 최초 관측이 들어오면 EKF를 초기화합니다.
        if not self.state.initialized:
            self.state.kf.x = np.array([s, vs, d, vd])
            self.state.initialized = True
            self.state.size = obs.size
        else:
            self.state.update(s, d, vs, vd)

    # ------------------------------------------------------------------
    # 메인 루프 – 예측 및 퍼블리시
    # ------------------------------------------------------------------
    def loop(self) -> None:
        if not self.state.initialized:
            return  # 첫 관측을 기다립니다.

        # 1. EKF 예측 단계
        if self.state.is_visible:
            self.state.predict()
        else:
            # 속도를 0으로 설정한 상태로 재초기화합니다.
            self.state.kf.x = np.zeros(4)
            self.state.initialized = False
            self.state.vs_filt = [0.0] * 5
            self.state.vd_filt = [0.0] * 5            

        # 2. 현재 추정치를 사용해 *ObstacleArray* 메시지를 구성합니다.
        obstacle_msg = ObstacleArray()
        obstacle_msg.header.stamp = self.get_clock().now().to_msg()
        obstacle_msg.header.frame_id = "map"

        # 공분산 행렬의 트레이스를 계산합니다.
        trace = np.trace(self.state.kf.P)
        if trace < 0.5 and self.state.is_visible:
            obs = ObstacleMsg()
            obs.id = 1  # 단일 상대 차량 ID

            obs.s_center = self.state.kf.x[0]
            if self.state.kf.x[0] < 0:
                obs.s_center += self.track_length
            obs.d_center = self.state.kf.x[2]
            obs.vs = float(np.mean(self.state.vs_filt))
            obs.vd = float(np.mean(self.state.vd_filt))
            obs.size = self.state.size
            obs.is_static = False
            obs.s_start = (obs.s_center - obs.size / 2.0) % self.track_length
            obs.s_end = (obs.s_center + obs.size / 2.0) % self.track_length
            obs.d_right = obs.d_center - obs.size / 2.0
            obs.d_left = obs.d_center + obs.size / 2.0
            obs.is_visible = self.state.is_visible

            # 불확실성이 필요한 소비자 노드를 위한 공분산 대각 성분
            # obs.s_var = float(self.state.kf.P[0, 0])
            # obs.vs_var = float(self.state.kf.P[1, 1])
            # obs.d_var = float(self.state.kf.P[2, 2])
            # obs.vd_var = float(self.state.kf.P[3, 3])

            obstacle_msg.obstacles.append(obs)
            self.obstacles_pub.publish(obstacle_msg)

            # 3. 시각적 디버깅을 위해 RViz 마커를 발행합니다.
            self.publish_marker(obs)
        else:
            # 마커를 초기화하고 빈 장애물 배열을 발행합니다.
            self.marker_pub.publish(self.clearmarkers())
            obstacle_msg.obstacles = []
            self.obstacles_pub.publish(obstacle_msg)


    def clearmarkers(self) -> MarkerArray:
        marker_array = MarkerArray()
        marker = Marker()
        marker.action = 3
        marker_array.markers = [marker]
        return marker_array

    # ------------------------------------------------------------------
    # 시각화 보조 함수
    # ------------------------------------------------------------------
    def publish_marker(self, obs: ObstacleMsg) -> None:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = obs.id
        marker.type = Marker.SPHERE

        # RViz 표시를 위해 프레네 좌표를 xy로 변환합니다.
        if obs.s_center < 0:
            obs.s_center += self.track_length
        x, y = self.converter.get_cartesian(obs.s_center, obs.d_center)
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.orientation.w = 1.0  # 회전 없음

        marker.scale.x = marker.scale.y = marker.scale.z = obs.size

        # 반투명 빨간색
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8

        self.marker_pub.publish(MarkerArray(markers=[marker]))

# ---------------------------------------------------------------------------
# 진입점
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = OpponentTracking()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
