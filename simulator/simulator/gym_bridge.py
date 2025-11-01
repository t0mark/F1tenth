import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import PointStamped
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformListener
from tf2_ros.buffer import Buffer
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

from pathlib import Path
import gymnasium as gym
import numpy as np
from transforms3d import euler
import math
import xacro

# Import for collision override
from f110_gym.envs.base_classes import Simulator, RaceCar, Integrator

from visualization_msgs.msg import Marker, MarkerArray
from tf2_geometry_msgs import do_transform_point
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

class NoCollisionSimulator(Simulator):
    """
    Simulator 클래스를 상속받아 충돌 감지를 비활성화하는 클래스
    """
    
    def __init__(self, params, num_agents, seed, time_step=0.01, ego_idx=0, integrator=None, lidar_dist=0.0):
        """
        기본 Simulator 초기화 후 agents를 NoCollisionRaceCar로 교체
        """
        super().__init__(params, num_agents, seed, time_step, ego_idx, integrator, lidar_dist)
        
        # 기존 agents를 NoCollisionRaceCar로 교체
        self.agents = []
        for i in range(self.num_agents):
            if i == ego_idx:
                ego_car = NoCollisionRaceCar(params, self.seed, is_ego=True, time_step=self.time_step, integrator=integrator, lidar_dist=lidar_dist)
                self.agents.append(ego_car)
            else:
                agent = NoCollisionRaceCar(params, self.seed, is_ego=False, time_step=self.time_step, integrator=integrator, lidar_dist=lidar_dist)
                self.agents.append(agent)
    
    def check_collision(self):
        """
        충돌 감지를 비활성화 - 항상 충돌 없음으로 설정
        """
        self.collisions = np.zeros((self.num_agents, ))
        self.collision_idx = -1 * np.ones((self.num_agents, ))

class NoCollisionRaceCar(RaceCar):
    """
    RaceCar 클래스를 상속받아 환경 충돌 감지를 비활성화하는 클래스
    """
    
    def check_ttc(self, current_scan):
        """
        TTC 기반 충돌 감지를 비활성화 - 항상 충돌 없음으로 설정
        """
        self.in_collision = False
        return False

class GymBridge(Node):
    """
    F1TENTH Gym 시뮬레이터와 ROS2를 연결하는 브리지 노드
    - 시뮬레이터에서 센서 데이터를 받아 ROS 토픽으로 퍼블리시
    - ROS 토픽에서 제어 명령을 받아 시뮬레이터에 전달
    - 단일 또는 다중 에이전트 시뮬레이션 지원
    """
    
    def __init__(self):
        super().__init__('gym_bridge')
        
        package_root = Path(__file__).resolve().parents[1]
        try:
            maps_share_dir = Path(get_package_share_directory('f1tenth')) / 'maps'
        except PackageNotFoundError:
            maps_share_dir = package_root.parent / 'f1tenth' / 'maps'
        default_map_prefix = str(maps_share_dir / 'Spielberg_map')

        default_params = {
            'ego_namespace': 'ego_racecar',
            'ego_odom_topic': 'odom',
            'ego_opp_odom_topic': 'opp_odom',
            'ego_scan_topic': 'scan',
            'ego_drive_topic': 'drive',
            'opp_namespace': 'opp_racecar',
            'opp_odom_topic': 'odom',
            'opp_ego_odom_topic': 'opp_odom',
            'opp_scan_topic': 'opp_scan',
            'opp_drive_topic': 'opp_drive',
            'opp_teleop_topic': 'teleop',
            'scan_distance_to_base_link': 0.0,
            'scan_fov': 4.7,
            'scan_beams': 1080,
            'map_path': default_map_prefix,
            'map_img_ext': '.png',
            'num_agent': 1,
            'sx': 0.0,
            'sy': 0.0,
            'stheta': 0.0,
            'sx1': 2.0,
            'sy1': 0.5,
            'stheta1': 0.0,
            'kb_teleop': True,
            'obstacle_length': 0.12,
            'obstacle_width': 0.1,
            'obstacle_height': 0.2,
        }

        for name, default in default_params.items():
            self.declare_parameter(name, default)

        self.num_agents = self.get_parameter('num_agent').value

        # 유효성 검사
        if self.num_agents < 1 or self.num_agents > 2:
            raise ValueError('num_agents should be either 1 or 2.')
        elif type(self.num_agents) != int:
            raise ValueError('num_agents should be an int.')

        # 라이다 센서 설정
        scan_fov = self.get_parameter('scan_fov').value
        scan_beams = self.get_parameter('scan_beams').value
        self.angle_min = -scan_fov / 2.
        self.angle_max = scan_fov / 2.
        self.angle_inc = scan_fov / scan_beams
        self.scan_distance_to_base_link = self.get_parameter('scan_distance_to_base_link').value
        
        # 메인 차량(ego) 설정
        sx = self.get_parameter('sx').value
        sy = self.get_parameter('sy').value
        stheta = self.get_parameter('stheta').value
        self.ego_namespace = self.get_parameter('ego_namespace').value
        self.ego_pose = [sx, sy, stheta]
        self.ego_speed = [0.0, 0.0, 0.0]
        self.ego_requested_speed = 0.0
        self.ego_steer = 0.0
        self.ego_collision = False
        self.ego_drive_published = False
        # 장애물 설정 (라이다 업데이트에서 참조하므로 환경 리셋 전에 초기화)
        self.obstacle_length = float(self.get_parameter('obstacle_length').value)
        self.obstacle_width = float(self.get_parameter('obstacle_width').value)
        self.obstacle_height = float(self.get_parameter('obstacle_height').value)
        self.obstacles = []
        self.obstacle_seq = 0
        self.scan_range_max = 30.0

        simulator_share_dir = Path(get_package_share_directory('simulator'))
        obstacle_xacro = simulator_share_dir / 'urdf' / 'obstacle.xacro'
        try:
            processed = xacro.process_file(
                str(obstacle_xacro),
                mappings={
                    'obstacle_name': 'dynamic_obstacle',
                    'length': str(self.obstacle_length),
                    'width': str(self.obstacle_width),
                    'height': str(self.obstacle_height),
                }
            )
            self.obstacle_urdf = processed.toxml()
        except Exception as exc:
            self.get_logger().warn(f'failed to process obstacle xacro: {exc}')
            self.obstacle_urdf = ''

        # 상대방 차량(opponent) 설정
        if self.num_agents == 2:
            self.has_opp = True
            sx1 = self.get_parameter('sx1').value
            sy1 = self.get_parameter('sy1').value
            stheta1 = self.get_parameter('stheta1').value
            self.opp_namespace = self.get_parameter('opp_namespace').value
            self.opp_pose = [sx1, sy1, stheta1]
            self.opp_speed = [0.0, 0.0, 0.0]
            self.opp_requested_speed = 0.0
            self.opp_steer = 0.0
            self.opp_collision = False
            self.opp_drive_published = False
        else:
            self.has_opp = False
        
        # TF 브로드캐스터 초기화 및 타이머 설정
        self.env = gym.make(
            'f110_gym:f110-v0',
            map=self.get_parameter('map_path').value,
            map_ext=self.get_parameter('map_img_ext').value,
            num_agents=self.num_agents)
        
        # 충돌 감지 비활성화를 위해 Simulator 교체
        # Gymnasium wrapper를 unwrap하여 실제 환경에 접근
        unwrapped_env = self.env.unwrapped
        original_sim = unwrapped_env.sim
        integrator_type = original_sim.agents[0].integrator if original_sim.agents else Integrator.RK4
        lidar_dist = original_sim.agents[0].lidar_dist if original_sim.agents else 0.0
        
        # 새로운 시뮬레이터 생성 후 기존 맵 정보 복사
        new_sim = NoCollisionSimulator(
            original_sim.params, 
            original_sim.num_agents, 
            original_sim.seed, 
            time_step=original_sim.time_step, 
            ego_idx=original_sim.ego_idx,
            integrator=integrator_type,
            lidar_dist=lidar_dist
        )
        
        # 기존 시뮬레이터에서 이미 로드된 맵 정보를 새 시뮬레이터로 복사
        for i, agent in enumerate(new_sim.agents):
            if i < len(original_sim.agents):
                agent.scan_simulator = original_sim.agents[i].scan_simulator
        
        unwrapped_env.sim = new_sim

        if self.num_agents == 2:
            initial_poses = np.array([[sx, sy, stheta], [sx1, sy1, stheta1]], dtype=float)
        else:
            initial_poses = np.array([[sx, sy, stheta]], dtype=float)

        self._reset_environment(initial_poses)

        # TF 브로드캐스터
        self.br = TransformBroadcaster(self)

        # TF 버퍼 (RViz Publish Point에서 map 프레임으로 좌표 변환)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        marker_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.obstacle_marker_pub = self.create_publisher(MarkerArray, 'obstacle', marker_qos)

        # 토픽 이름 설정
        ego_scan_topic = self.get_parameter('ego_scan_topic').value
        ego_drive_topic = self.get_parameter('ego_drive_topic').value
        ego_odom_topic = self.get_parameter('ego_odom_topic').value

        if self.has_opp:
            opp_scan_topic = self.get_parameter('opp_scan_topic').value
            opp_odom_topic = self.opp_namespace + '/' + self.get_parameter('opp_odom_topic').value
            ego_opp_odom_topic = self.ego_namespace + '/' + self.get_parameter('ego_opp_odom_topic').value
            opp_ego_odom_topic = self.opp_namespace + '/' + self.get_parameter('opp_ego_odom_topic').value

        # 퍼블리셔
        self.ego_scan_pub = self.create_publisher(LaserScan, ego_scan_topic, 10)
        self.ego_odom_pub = self.create_publisher(Odometry, ego_odom_topic, 10)
        
        if self.has_opp:
            self.opp_scan_pub = self.create_publisher(LaserScan, opp_scan_topic, 10)
            self.opp_odom_pub = self.create_publisher(Odometry, opp_odom_topic, 10)
            self.ego_opp_odom_pub = self.create_publisher(Odometry, ego_opp_odom_topic, 10)
            self.opp_ego_odom_pub = self.create_publisher(Odometry, opp_ego_odom_topic, 10)

        # 서브스크라이버
        self.ego_drive_sub = self.create_subscription(
            AckermannDriveStamped, ego_drive_topic, self.drive_callback, 10)
        self.ego_reset_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.ego_reset_callback, 10)
        
        if self.has_opp:
            opp_drive_topic = self.get_parameter('opp_drive_topic').value
            self.opp_drive_sub = self.create_subscription(
                AckermannDriveStamped, opp_drive_topic, self.opp_drive_callback, 10)
            self.opp_reset_sub = self.create_subscription(
                PoseStamped, '/goal_pose', self.opp_reset_callback, 10)
            opp_teleop_topic = self.get_parameter('opp_teleop_topic').value
            self.opp_teleop_sub = self.create_subscription(
                AckermannDriveStamped, opp_teleop_topic, self.opp_teleop_callback, 10)
        
        self.clicked_point_sub = self.create_subscription(
            PointStamped, '/clicked_point', self.clicked_point_callback, 10)

        if self.get_parameter('kb_teleop').value:
            self.teleop_sub = self.create_subscription(
                Twist, '/cmd_vel', self.teleop_callback, 10)

        # 타이머: 시뮬레이션 스텝(100Hz) / 퍼블리시 루프(250Hz)
        self.drive_timer = self.create_timer(0.01, self.drive_timer_callback)
        self.timer = self.create_timer(0.004, self.timer_callback)
    
    def drive_callback(self, drive_msg):
        """
        ego 차량 제어 명령 콜백
        - 목적: ego 차량의 목표 속도/조향각을 갱신
        - 입력: AckermannDriveStamped (speed [m/s], steering_angle [rad])
        """
        self.ego_requested_speed = drive_msg.drive.speed
        self.ego_steer = drive_msg.drive.steering_angle
        self.ego_drive_published = True

    def opp_drive_callback(self, drive_msg):
        """
        opp 차량 제어 명령 콜백 (다중 에이전트일 때만)
        - 목적: opp 차량의 목표 속도/조향각을 갱신
        - 입력: AckermannDriveStamped (speed [m/s], steering_angle [rad])
        """
        self.opp_requested_speed = drive_msg.drive.speed
        self.opp_steer = drive_msg.drive.steering_angle
        self.opp_drive_published = True

    def opp_teleop_callback(self, drive_msg):
        """
        조이스틱 텔레옵 콜백
        - 목적: 텔레옵 토픽을 통해 상대 차량 제어
        """
        self.opp_drive_callback(drive_msg)

    def teleop_callback(self, twist_msg):
        """
        키보드 텔레오퍼레이션 콜백 (옵션)
        - 목적: /cmd_vel을 통해 간단한 전/후진 및 좌/우 조향을 ego에 반영
        - 입력: geometry_msgs/Twist (linear.x [m/s], angular.z [rad])
        - 텔레오프 사용 시에도 정식 Ackermann 명령(조향 모델 제어)과 동일하게 drive_timer에서 스텝이 진행됨
        """
        if not self.ego_drive_published:
            self.ego_drive_published = True

        self.ego_requested_speed = twist_msg.linear.x

        if twist_msg.angular.z > 0.0:
            self.ego_steer = 0.3
        elif twist_msg.angular.z < 0.0:
            self.ego_steer = -0.3
        else:
            self.ego_steer = 0.0

    def _reset_environment(self, poses):
        """
        Gymnasium 환경을 지정 포즈로 리셋하고 내부 상태를 초기화.
        """
        self.obs, _ = self.env.reset(options={'poses': np.asarray(poses, dtype=float)})
        self.terminated = False
        self.truncated = False
        self._update_sim_state()

    def ego_reset_callback(self, pose_msg):
        """
        RViz 등에서 /initialpose를 수신해 ego 차량 위치를 리셋.
        """
        rx = pose_msg.pose.pose.position.x
        ry = pose_msg.pose.pose.position.y
        quat = pose_msg.pose.pose.orientation
        _, _, rtheta = euler.quat2euler(
            [quat.w, quat.x, quat.y, quat.z],
            axes='sxyz'
        )

        if self.has_opp:
            opp_pose = [
                self.obs['poses_x'][1],
                self.obs['poses_y'][1],
                self.obs['poses_theta'][1],
            ]
            poses = np.array([[rx, ry, rtheta], opp_pose], dtype=float)
        else:
            poses = np.array([[rx, ry, rtheta]], dtype=float)
        self._reset_environment(poses)

    def opp_reset_callback(self, pose_msg):
        """
        /goal_pose를 이용해 상대 차량(opp) 위치를 리셋.
        """
        if not self.has_opp:
            return

        rx = pose_msg.pose.position.x
        ry = pose_msg.pose.position.y
        quat = pose_msg.pose.orientation
        _, _, rtheta = euler.quat2euler(
            [quat.w, quat.x, quat.y, quat.z],
            axes='sxyz'
        )
        poses = np.array([list(self.ego_pose), [rx, ry, rtheta]], dtype=float)
        self._reset_environment(poses)

    def drive_timer_callback(self):
        """
        물리 시뮬레이션 스텝 실행 (주기 타이머)
        - 목적: 현재 명령(ego/opp)을 바탕으로 env.step을 1스텝 진행
        - 주기: 0.01s (100 Hz)
        - (ego_drive_published 플래그) 최초 명령 수신 전에는 스텝을 진행 X
        """
        if not self.has_opp:
            if self.ego_drive_published:
                step_action = np.array([[self.ego_steer, self.ego_requested_speed]], dtype=np.float32)
                self.obs, _, self.terminated, self.truncated, _ = self.env.step(step_action)
        else:
            if self.ego_drive_published or self.opp_drive_published:
                ego_speed = self.ego_requested_speed if self.ego_drive_published else 0.0
                ego_steer = self.ego_steer if self.ego_drive_published else 0.0
                opp_speed = self.opp_requested_speed if self.opp_drive_published else 0.0
                opp_steer = self.opp_steer if self.opp_drive_published else 0.0
                step_action = np.array([[ego_steer, ego_speed],
                                        [opp_steer, opp_speed]], dtype=np.float32)
                self.obs, _, self.terminated, self.truncated, _ = self.env.step(step_action)
        self._update_sim_state()

    def timer_callback(self):
        """
        LiDAR/Odom/TF 퍼블리싱 루프 (주기 타이머)
        - 목적: 현재 시뮬레이션 상태를 ROS 토픽/TF로 퍼블리시합니다.
        - 주기: 0.004s (≈250 Hz)
        - 퍼블리시:
          * LaserScan: ego(+opp)
          * Odometry: ego(+opp), 상호 추정 odom(ego_opp/opp_ego)
          * TF: map→base_link, base_link→laser, 휠 힌지→휠(조향각)
        - 타임스탬프: 노드 clock 기준 now()
        """
        ts = self.get_clock().now().to_msg()

        self._publish_laser_scan(ts)
        self._publish_odom(ts)
        self._publish_transforms(ts)
        self._publish_laser_transforms(ts)
        self._publish_wheel_transforms(ts)

    
    def _update_sim_state(self):
        """
        시뮬레이터 관측값을 내부 상태로 반영
        - 목적: env.step/reset 결과 -> self.obs (gym 관측 dict) -> 포즈/속도/스캔 버퍼
        - 이 함수는 순수 상태 동기화만 수행
        """
        self.ego_scan = self.obs['scans'][0].astype(float).tolist()
        self.ego_pose[0] = float(self.obs['poses_x'][0])
        self.ego_pose[1] = float(self.obs['poses_y'][0])
        self.ego_pose[2] = float(self.obs['poses_theta'][0])
        self.ego_speed[0] = float(self.obs['linear_vels_x'][0])
        self.ego_speed[1] = float(self.obs['linear_vels_y'][0])
        self.ego_speed[2] = float(self.obs['ang_vels_z'][0])
        
        if self.has_opp:
            self.opp_scan = self.obs['scans'][1].astype(float).tolist()
            self.opp_pose[0] = float(self.obs['poses_x'][1])
            self.opp_pose[1] = float(self.obs['poses_y'][1])
            self.opp_pose[2] = float(self.obs['poses_theta'][1])
            self.opp_speed[0] = float(self.obs['linear_vels_x'][1])
            self.opp_speed[1] = float(self.obs['linear_vels_y'][1])
            self.opp_speed[2] = float(self.obs['ang_vels_z'][1])

        if self.obstacles:
            self.ego_scan = self._apply_obstacles_to_scan(self.ego_pose, self.ego_scan)
            if self.has_opp:
                self.opp_scan = self._apply_obstacles_to_scan(self.opp_pose, self.opp_scan)

    def _publish_laser_scan(self, ts):
        """
        라이다 스캔 데이터 퍼블리시
        - 목적: 최신 스캔 버퍼를 LaserScan 메시지로 송신
        - 프레임: {namespace}/laser
        """
        scan = LaserScan()
        scan.header.stamp = ts
        scan.header.frame_id = 'laser'
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_inc
        scan.range_min = 0.
        scan.range_max = 30.
        scan.ranges = self.ego_scan
        self.ego_scan_pub.publish(scan)

        if self.has_opp:
            opp_scan = LaserScan()
            opp_scan.header.stamp = ts
            opp_scan.header.frame_id = self.opp_namespace + '/laser'
            opp_scan.angle_min = self.angle_min
            opp_scan.angle_max = self.angle_max
            opp_scan.angle_increment = self.angle_inc
            opp_scan.range_min = 0.
            opp_scan.range_max = 30.
            opp_scan.ranges = self.opp_scan
            self.opp_scan_pub.publish(opp_scan)

    def _publish_odom(self, ts):
        """
        오도메트리 데이터 퍼블리시
        - 목적: 현재 포즈/속도를 nav_msgs/Odometry로 송신
        - 교차 퍼블리시: ego_opp_odom_pub / opp_ego_odom_pub (상대 차량에 대한 odom 제공)
        - covariance는 설정하지 않으므로 다운스트림에서 필요 시 주의 요망
        """
        ego_odom = Odometry()
        ego_odom.header.stamp = ts
        ego_odom.header.frame_id = 'map'
        ego_odom.child_frame_id = 'base_link'
        ego_odom.pose.pose.position.x = self.ego_pose[0]
        ego_odom.pose.pose.position.y = self.ego_pose[1]
        ego_quat = euler.euler2quat(0., 0., self.ego_pose[2], axes='sxyz')
        ego_odom.pose.pose.orientation.x = ego_quat[1]
        ego_odom.pose.pose.orientation.y = ego_quat[2]
        ego_odom.pose.pose.orientation.z = ego_quat[3]
        ego_odom.pose.pose.orientation.w = ego_quat[0]
        ego_odom.twist.twist.linear.x = self.ego_speed[0]
        ego_odom.twist.twist.linear.y = self.ego_speed[1]
        ego_odom.twist.twist.angular.z = self.ego_speed[2]
        self.ego_odom_pub.publish(ego_odom)

        if self.has_opp:
            opp_odom = Odometry()
            opp_odom.header.stamp = ts
            opp_odom.header.frame_id = 'map'
            opp_odom.child_frame_id = self.opp_namespace + '/base_link'
            opp_odom.pose.pose.position.x = self.opp_pose[0]
            opp_odom.pose.pose.position.y = self.opp_pose[1]
            opp_quat = euler.euler2quat(0., 0., self.opp_pose[2], axes='sxyz')
            opp_odom.pose.pose.orientation.x = opp_quat[1]
            opp_odom.pose.pose.orientation.y = opp_quat[2]
            opp_odom.pose.pose.orientation.z = opp_quat[3]
            opp_odom.pose.pose.orientation.w = opp_quat[0]
            opp_odom.twist.twist.linear.x = self.opp_speed[0]
            opp_odom.twist.twist.linear.y = self.opp_speed[1]
            opp_odom.twist.twist.angular.z = self.opp_speed[2]
            self.opp_odom_pub.publish(opp_odom)
            self.opp_ego_odom_pub.publish(ego_odom)
            self.ego_opp_odom_pub.publish(opp_odom)

    def _publish_transforms(self, ts):
        """
        차량 베이스 링크 TF 퍼블리시
        - 목적: map → {namespace}/base_link 변환을 TF로 브로드캐스트
        - 레이저/휠 등 하위 링크는 별도 함수에서 송신
        """
        ego_t = Transform()
        ego_t.translation.x = self.ego_pose[0]
        ego_t.translation.y = self.ego_pose[1]
        ego_t.translation.z = 0.0
        ego_quat = euler.euler2quat(0.0, 0.0, self.ego_pose[2], axes='sxyz')
        ego_t.rotation.x = ego_quat[1]
        ego_t.rotation.y = ego_quat[2]
        ego_t.rotation.z = ego_quat[3]
        ego_t.rotation.w = ego_quat[0]

        ego_ts = TransformStamped()
        ego_ts.transform = ego_t
        ego_ts.header.stamp = ts
        ego_ts.header.frame_id = 'map'
        ego_ts.child_frame_id = 'base_link'
        self.br.sendTransform(ego_ts)

        if self.has_opp:
            opp_t = Transform()
            opp_t.translation.x = self.opp_pose[0]
            opp_t.translation.y = self.opp_pose[1]
            opp_t.translation.z = 0.0
            opp_quat = euler.euler2quat(0.0, 0.0, self.opp_pose[2], axes='sxyz')
            opp_t.rotation.x = opp_quat[1]
            opp_t.rotation.y = opp_quat[2]
            opp_t.rotation.z = opp_quat[3]
            opp_t.rotation.w = opp_quat[0]

            opp_ts = TransformStamped()
            opp_ts.transform = opp_t
            opp_ts.header.stamp = ts
            opp_ts.header.frame_id = 'map'
            opp_ts.child_frame_id = self.opp_namespace + '/base_link'
            self.br.sendTransform(opp_ts)

    def _publish_wheel_transforms(self, ts):
        """
        바퀴 조향각 TF 퍼블리시
        - 목적: 조향각을 반영하여 힌지→휠 링크 회전 TF를 브로드캐스트
        - 순수 회전만 갱신(translation 없음)
        """
        ego_wheel_ts = TransformStamped()
        ego_wheel_quat = euler.euler2quat(0., 0., self.ego_steer, axes='sxyz')
        ego_wheel_ts.transform.rotation.x = ego_wheel_quat[1]
        ego_wheel_ts.transform.rotation.y = ego_wheel_quat[2]
        ego_wheel_ts.transform.rotation.z = ego_wheel_quat[3]
        ego_wheel_ts.transform.rotation.w = ego_wheel_quat[0]
        ego_wheel_ts.header.stamp = ts
        ego_wheel_ts.header.frame_id = 'front_left_hinge'
        ego_wheel_ts.child_frame_id = 'front_left_wheel'
        self.br.sendTransform(ego_wheel_ts)
        ego_wheel_ts.header.frame_id = 'front_right_hinge'
        ego_wheel_ts.child_frame_id = 'front_right_wheel'
        self.br.sendTransform(ego_wheel_ts)

        if self.has_opp:
            opp_wheel_ts = TransformStamped()
            opp_wheel_quat = euler.euler2quat(0., 0., self.opp_steer, axes='sxyz')
            opp_wheel_ts.transform.rotation.x = opp_wheel_quat[1]
            opp_wheel_ts.transform.rotation.y = opp_wheel_quat[2]
            opp_wheel_ts.transform.rotation.z = opp_wheel_quat[3]
            opp_wheel_ts.transform.rotation.w = opp_wheel_quat[0]
            opp_wheel_ts.header.stamp = ts
            opp_wheel_ts.header.frame_id = self.opp_namespace + '/front_left_hinge'
            opp_wheel_ts.child_frame_id = self.opp_namespace + '/front_left_wheel'
            self.br.sendTransform(opp_wheel_ts)
            opp_wheel_ts.header.frame_id = self.opp_namespace + '/front_right_hinge'
            opp_wheel_ts.child_frame_id = self.opp_namespace + '/front_right_wheel'
            self.br.sendTransform(opp_wheel_ts)

    def clicked_point_callback(self, point_msg: PointStamped):
        """
        RViz Publish Point를 이용해 전달된 좌표를 장애물로 추가.
        """
        target_frame = 'map'
        point_in_map = point_msg

        if point_msg.header.frame_id and point_msg.header.frame_id != target_frame:
            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    point_msg.header.frame_id,
                    Time())
                point_in_map = do_transform_point(point_msg, transform)
            except Exception as exc:
                self.get_logger().warn(f'clicked point transform failed: {exc}')
                return

        ox = float(point_in_map.point.x)
        oy = float(point_in_map.point.y)
        self._add_obstacle(ox, oy)

    def _add_obstacle(self, x: float, y: float):
        half_width = self.obstacle_width / 2.0
        half_length = self.obstacle_length / 2.0
        bounds = (x - half_width, x + half_width, y - half_length, y + half_length)
        obstacle = {
            'id': self.obstacle_seq,
            'center': (x, y),
            'bounds': bounds,
            'height': self.obstacle_height,
        }
        self.obstacle_seq += 1
        self.obstacles.append(obstacle)
        self._publish_obstacle_markers()
        self.get_logger().info(f'Added obstacle #{obstacle["id"]} at ({x:.2f}, {y:.2f})')

    def _publish_obstacle_markers(self):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        if not self.obstacles:
            marker = Marker()
            marker.action = Marker.DELETEALL
            marker.header.frame_id = 'map'
            marker.header.stamp = stamp
            markers.markers.append(marker)
            self.obstacle_marker_pub.publish(markers)
            return

        for obstacle in self.obstacles:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = stamp
            marker.ns = 'dynamic_obstacles'
            marker.id = obstacle['id']
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = obstacle['center'][0]
            marker.pose.position.y = obstacle['center'][1]
            marker.pose.position.z = obstacle['height'] / 2.0
            marker.scale.x = self.obstacle_width
            marker.scale.y = self.obstacle_length
            marker.scale.z = self.obstacle_height
            marker.color.r = 0.9
            marker.color.g = 0.2
            marker.color.b = 0.2
            marker.color.a = 0.8
            markers.markers.append(marker)

        self.obstacle_marker_pub.publish(markers)

    def _apply_obstacles_to_scan(self, pose, scan):
        if not self.obstacles:
            return scan

        updated_scan = list(scan)
        px, py, ptheta = pose
        origin_x = px + math.cos(ptheta) * self.scan_distance_to_base_link
        origin_y = py + math.sin(ptheta) * self.scan_distance_to_base_link

        for idx, base_range in enumerate(scan):
            beam_angle = ptheta + self.angle_min + idx * self.angle_inc
            direction = (math.cos(beam_angle), math.sin(beam_angle))
            current_range = base_range
            if current_range <= 0.0 or math.isinf(current_range) or math.isnan(current_range):
                current_range = self.scan_range_max
            current_range = min(current_range, self.scan_range_max)

            for obstacle in self.obstacles:
                distance = self._ray_box_intersection(
                    (origin_x, origin_y),
                    direction,
                    obstacle['bounds']
                )
                if distance is not None and distance < current_range:
                    current_range = max(distance, 0.0)

            updated_scan[idx] = float(current_range)

        return updated_scan

    @staticmethod
    def _ray_box_intersection(origin, direction, bounds):
        """
        2D 축 정렬 박스(Ray vs AABB) 교차 거리 계산.
        origin: (x, y), direction: (dx, dy), bounds: (min_x, max_x, min_y, max_y)
        반환: 교차 거리 (없으면 None)
        """
        ox, oy = origin
        dx, dy = direction
        min_x, max_x, min_y, max_y = bounds

        epsilon = 1e-9

        if abs(dx) < epsilon:
            if ox < min_x or ox > max_x:
                return None
            tx_min = -math.inf
            tx_max = math.inf
        else:
            tx1 = (min_x - ox) / dx
            tx2 = (max_x - ox) / dx
            tx_min = min(tx1, tx2)
            tx_max = max(tx1, tx2)

        if abs(dy) < epsilon:
            if oy < min_y or oy > max_y:
                return None
            ty_min = -math.inf
            ty_max = math.inf
        else:
            ty1 = (min_y - oy) / dy
            ty2 = (max_y - oy) / dy
            ty_min = min(ty1, ty2)
            ty_max = max(ty1, ty2)

        t_enter = max(tx_min, ty_min)
        t_exit = min(tx_max, ty_max)

        if t_exit < 0 or t_enter > t_exit:
            return None

        if t_enter < 0.0:
            return 0.0

        return t_enter

    def _publish_laser_transforms(self, ts):
        """
        라이다 센서 TF 퍼블리시
        - 목적: base_link → laser 고정 변환을 송신
        """
        ego_scan_ts = TransformStamped()
        ego_scan_ts.transform.translation.x = self.scan_distance_to_base_link
        ego_scan_ts.transform.rotation.w = 1.
        ego_scan_ts.header.stamp = ts
        ego_scan_ts.header.frame_id = 'base_link'
        ego_scan_ts.child_frame_id = 'laser'
        self.br.sendTransform(ego_scan_ts)

        if self.has_opp:
            opp_scan_ts = TransformStamped()
            opp_scan_ts.transform.translation.x = self.scan_distance_to_base_link
            opp_scan_ts.transform.rotation.w = 1.
            opp_scan_ts.header.stamp = ts
            opp_scan_ts.header.frame_id = self.opp_namespace + '/base_link'
            opp_scan_ts.child_frame_id = self.opp_namespace + '/laser'
            self.br.sendTransform(opp_scan_ts)


def main(args=None):
    """
    메인 함수 - 노드 초기화 및 실행
    - 목적: rclpy 초기화, GymBridge 노드 생성, 스핀 진입
    - 종료: Ctrl+C 등으로 스핀 종료 시 rclpy.shutdown()은 런타임이 정리
    - 외부에서 launch 파일로 여러 노드를 구동할 경우, 파라미터를 launch에서 전달
    """
    rclpy.init(args=args)
    gym_bridge = GymBridge()
    rclpy.spin(gym_bridge)

if __name__ == '__main__':
    main()
