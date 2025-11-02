#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from nav_msgs.msg import Odometry
from f1tenth_icra_race_msgs.msg import WpntArray, ObstacleArray
from std_msgs.msg import String
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from ackermann_msgs.msg import AckermannDriveStamped

import numpy as np
from .modules.pure_pursuit import PP_Controller
from .modules import utils
from .modules.frenet_conversion import FrenetConverter
from tf_transformations import euler_from_quaternion
import os


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller')
        waypoint_file = os.path.join('src/path_planner/data', 'final_waypoints.csv')
        self.declare_parameter("waypoint_file", waypoint_file)

        # 기준 트랙 참고용 로컬 웨이포인트 CSV 파일을 불러옵니다.
        waypoint_file = self.get_parameter("waypoint_file").get_parameter_value().string_value
        waypoints = np.genfromtxt(waypoint_file, delimiter=';', skip_header=1)
        waypoint_cols = utils.column_numbers_for_waypoints()
        self.waypoints_x = waypoints[:, waypoint_cols['x_ref_m']]
        self.waypoints_y = waypoints[:, waypoint_cols['y_ref_m']]
        self.waypoints_psi = waypoints[:, waypoint_cols['psi_racetraj_rad']]
        # self.waypoints_psi = utils.convert_psi(self.waypoints_psi)  # 필요 시 psi를 변환합니다.
        self.track_length = float(waypoints[-1, waypoint_cols['s_racetraj_m']])
        self.converter = FrenetConverter(self.waypoints_x, self.waypoints_y, self.waypoints_psi)
        
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
        
        self.create_subscription(WpntArray, '/state_machine/local_waypoints', self.wpnts_callback, qos)
        self.create_subscription(String, '/state_machine/state', self.state_callback, qos)
        self.create_subscription(ObstacleArray, '/perception/obstacles', self.obstacles_callback, qos)

        # 주행 명령을 퍼블리시할 퍼블리셔를 설정합니다.
        self.declare_parameter('drive_topic', '/drive')
        drive_topic = self.get_parameter('drive_topic').value
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, qos)

        # 플롯 및 콘솔 디버깅을 위한 파라미터를 선언합니다.
        self.declare_parameter('plot_debug', True)
        self.plot_debug = self.get_parameter('plot_debug').value
        self.declare_parameter('print_debug', False)
        self.print_debug = self.get_parameter('print_debug').value

        # 시각화용 퍼블리셔를 생성합니다.
        self.lookahead_pub = self.create_publisher(PointStamped, '/controller/lookahead_point', qos)
        self.l1_pub = self.create_publisher(Point, 'l1_distance', qos)

        # 내부 상태 변수
        self.current_pose = None  # [x, y, yaw]
        self.current_pose_frenet = None  # [s, d, vs, vd]
        self.current_speed = 0.0
        self.local_waypoints = []  # (x, y, v, s) 리스트
        self.opponent = None  # ObstacleMsg
        self.state = None  # StateType

        # 타이머 루프를 설정합니다.
        self.declare_parameter('rate_hz', 50.0)
        self.rate_hz = self.get_parameter('rate_hz').value
        self.timer = self.create_timer(1.0 / self.rate_hz, self.control_loop)

        # --- 순수 추종(Pure Pursuit) 컨트롤러 상세 파라미터 --- #

        # L1 Distance (조향 Lookahead) 계산 파라미터: L1 = (m_l1 * 현재속도) + q_l1
        # m_l1: 속도 비례 계수 [초]. 속도가 높을수록 Lookahead 거리를 더 멀리 설정합니다.
        self.declare_parameter('m_l1', 0.65)
        # q_l1: 고정 상수항 [미터]. 속도와 관계없이 Lookahead 거리에 추가되는 최소 거리입니다.
        self.declare_parameter('q_l1', -0.65)
        # t_clip_min: L1 Distance의 최소 한계값 [미터]. 저속에서 Lookahead 거리가 너무 짧아지는 것을 방지합니다.
        self.declare_parameter('t_clip_min', 1.0)
        # t_clip_max: L1 Distance의 최대 한계값 [미터]. 고속에서 Lookahead 거리가 너무 길어지는 것을 방지합니다.
        self.declare_parameter('t_clip_max', 7.0)

        # 속도 결정을 위한 Lookahead 파라미터
        # speed_lookahead: 목표 속도를 가져올 미래 예측 시간 [초]. 이 시간만큼 뒤의 웨이포인트 속도를 현재 목표 속도로 설정합니다.
        self.declare_parameter('speed_lookahead', 0.1)

        # 조향 계산 보조를 위한 Lookahead 파라미터
        # speed_lookahead_for_steer: 조향각 계산 시 참조할 미래 예측 시간 [초]. 이 시간만큼 뒤의 웨이포인트 속도를 참조하여 조향을 보정합니다.
        self.declare_parameter('speed_lookahead_for_steer', 0.1)

        # 횡방향 오차(Lateral Error) 기반 속도 감속 파라미터
        # lat_err_coeff: 횡방향 오차에 대한 민감도 [무차원, 0~1]. 값이 클수록 경로에서 벗어났을 때 속도를 더 공격적으로 줄입니다.
        self.declare_parameter('lat_err_coeff', 0.25)

        # 기타 파라미터
        # start_scale_speed: 출발 시 초기 속도 스케일링 값 [무차원].
        self.declare_parameter('start_scale_speed', 0.5)

        # 상대 차량 추종(Trailing) 제어 파라미터
        # trailing_gap: 추종 시 유지할 목표 거리 [미터].
        self.declare_parameter('trailing_gap', 1.0)
        # trailing_p_gain: 추종 거리 오차에 대한 비례(P) 이득 [무차원].
        self.declare_parameter('trailing_p_gain', 0.5)
        # trailing_d_gain: 추종 속도 차이에 대한 미분(D) 이득 [무차원].
        self.declare_parameter('trailing_d_gain', 0.5)
        
        self.pure_pursuit_control = PP_Controller(
            self.get_parameter('t_clip_min').value,
            self.get_parameter('t_clip_max').value,
            self.get_parameter('m_l1').value,
            self.get_parameter('q_l1').value,
            self.get_parameter('speed_lookahead').value,
            self.get_parameter('lat_err_coeff').value,
            self.get_parameter('start_scale_speed').value,
            self.get_parameter('speed_lookahead_for_steer').value,
            self.get_parameter('trailing_gap').value,
            self.get_parameter('trailing_p_gain').value,
            self.get_parameter('trailing_d_gain').value
        )


    def pose_callback(self, pose_msg):
        # 차량의 현재 x, y 위치를 가져옵니다.
        pose = pose_msg.pose.pose
        yaw = euler_from_quaternion([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])[2]
        if self.print_debug:
            self.get_logger().info(f'Pose: {self.car_global_x}, {self.car_global_y}, {self.car_global_yaw}')

        # 현재 속도를 설정합니다.
        self.current_speed = pose_msg.twist.twist.linear.x

        # 현재 자세를 기록합니다.
        self.current_pose = [pose.position.x, pose.position.y, yaw]

        # 전역 좌표를 프레네 좌표로 변환합니다.
        s, d = self.converter.get_frenet(np.array([pose.position.x]), np.array([pose.position.y]))
        vs, vd = self.converter.get_frenet_velocities(np.array([pose_msg.twist.twist.linear.x]), np.array([pose_msg.twist.twist.linear.y]), yaw)
        self.current_pose_frenet = [s[0], d[0], vs[0], vd[0]]

    def wpnts_callback(self, msg: WpntArray):
        if self.local_waypoints is None or len(msg.wpnts) > 0:
            self.local_waypoints = []
        for waypoint in msg.wpnts:
            self.local_waypoints.append([
                waypoint.x_m,
                waypoint.y_m,
                waypoint.vx_mps,
                waypoint.s_m, waypoint.psi_rad, waypoint.ax_mps2])
        self.local_waypoints = np.array(self.local_waypoints)

    def state_callback(self, msg: String):
        self.state = msg.data
        if self.print_debug:
            self.get_logger().info(f'State: {self.state}')

    def obstacles_callback(self, msg: ObstacleArray):
        if msg.obstacles and len(msg.obstacles) > 0:
            obstacle = msg.obstacles[0]
            self.opponent = [obstacle.s_center, obstacle.d_center, obstacle.vs, obstacle.is_visible]
        else:
            self.opponent = None

    def pure_pursuit_control_and_visualize(self):
        speed, steering_angle, L1_point, _, _ = self.pure_pursuit_control.main_loop(
            self.state,
            self.current_pose,
            self.local_waypoints,
            self.current_speed,
            self.opponent,
            self.current_pose_frenet,
            self.track_length
        )

        if self.plot_debug:
            self.set_lookahead_marker(L1_point, 100)
        return speed, steering_angle


    def control_loop(self):
        if self.current_pose is None or self.local_waypoints is None:
            return
        if len(self.local_waypoints) == 0:
            return

        speed, steering_angle = self.pure_pursuit_control_and_visualize()

        # 주행 명령을 퍼블리시합니다.
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_angle
        self.drive_pub.publish(drive_msg)

    ### 시각화 관련 메서드
    def set_lookahead_marker(self, lookahead_point, id):
        """
        Lookahead 포인트를 PointStamped 메시지로 발행
        RViz에서 크기 조절 가능하도록 PointStamped 사용
        (Display Properties -> Size (m) 에서 크기 조절 가능)
        """
        lookahead_msg = PointStamped()
        lookahead_msg.header.frame_id = "map"
        lookahead_msg.header.stamp = self.get_clock().now().to_msg()
        lookahead_msg.point.x = lookahead_point[0]
        lookahead_msg.point.y = lookahead_point[1]
        lookahead_msg.point.z = 0.0
        self.lookahead_pub.publish(lookahead_msg)


def main(args=None):
    rclpy.init(args=args)
    print("Controller Node Initialized")
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
