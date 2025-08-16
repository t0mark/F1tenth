# F1TENTH Gym ROS2 Simulation Bridge

F1TENTH gym 환경을 ROS2 시뮬레이션으로 변환하는 통신 브릿지입니다.

## 요구 사항

- Ubuntu 20.04 LTS
- ROS 2 Foxy

## 설치

**Python 패키지**
```bash
pip3 install setuptools==59.6.0 \
  testresources wheel numpy matplotlib pyyaml \
  gymnasium pybullet-utils transforms3d
```

**시스템 패키지**
```bash
sudo apt-get update
sudo apt-get install python3-dev build-essential
```

**ROS 2 패키지**
```bash
sudo apt update
sudo apt install ros-foxy-joint-state-publisher \
  ros-foxy-joint-state-publisher-gui \
  ros-foxy-robot-state-publisher ros-foxy-xacro \
  ros-foxy-navigation2 ros-foxy-nav2-bringup \
  ros-foxy-rviz2 \
  ros-foxy-tf2-tools ros-foxy-tf2-ros-py
```

## 빌드

```bash
git clone https://github.com/t0mark/F1tenth .
cd f1tenth_gym && pip3 install -e .

mkdir -p ~/sim_ws/src
cd ~/sim_ws/src
git clone https://github.com/f1tenth/f1tenth_gym_ros

# 파일 수정 f1tenth_gym_ros/config/sim.yaml
# 수정 전: 
map_path '/sim_ws/src/f1tenth_gym_ros/maps/levine'
# 수정 후: 
map_path: '{Home 디렉토리}/sim_ws/src/f1tenth_gym_ros/maps/levine'

source /opt/ros/foxy/setup.bash
cd ~/sim_ws
rosdep update
rosdep install -i --from-path src --rosdistro foxy -y
colcon build

echo "source ~/sim_ws/install/local_setup.bash" >> ~/.bashrc
echo "source ~/sim_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 실행

### 기본 시뮬레이션

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

### 키보드 텔레오프

- 키보드 텔레오프 활성화: `config/sim.yaml`에서 `kb_teleop: True` 설정

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

| 동작 | 전진 | 전진+좌회전 | 전진+우회전 | 후진 | 후진+좌회전 | 후진+우회전 | 정지 |
|------|------|-------------|-------------|------|-------------|-------------|------|
| 키   | `i`  | `u`         | `o`         | `,`  | `m`         | `.`         | `k`  |

#### RViz 초기 위치 설정
1. RViz에서 "2D Pose Estimate" 도구 선택
2. 맵상 차량 위치 클릭 후 드래그로 방향 설정

## 시뮬레이션 구성

구성 파일: `f1tenth_gym_ros/config/sim.yaml`

- `map_path`: 맵 파일 경로 (전체 경로 필요)
- `num_agent`: 에이전트 수 (1 또는 2)
- `kb_teleop`: 키보드 텔레오프 활성화

구성 변경 후 `colcon build` 재실행 필요

## 토픽 구조

### 발행 토픽 (Published)

**센서 데이터**
| 토픽명 | 메시지 타입 | 주파수 | 설명 |
|--------|-------------|---------|------|
| `/scan` | `sensor_msgs/LaserScan` | 250Hz | 주 차량 라이다 스캔 데이터 |
| `/ego_racecar/odom` | `nav_msgs/Odometry` | 250Hz | 주 차량 오도메트리 정보 |
| `/opp_scan` | `sensor_msgs/LaserScan` | 250Hz | 상대방 라이다 (2 에이전트) |
| `/opp_racecar/odom` | `nav_msgs/Odometry` | 250Hz | 상대방 오도메트리 (2 에이전트) |
| `/ego_racecar/opp_odom` | `nav_msgs/Odometry` | 250Hz | 주 차량 네임스페이스의 상대방 정보 |
| `/opp_racecar/ego_odom` | `nav_msgs/Odometry` | 250Hz | 상대방 네임스페이스의 주 차량 정보 |

**환경 정보**
| 토픽명 | 메시지 타입 | 설명 |
|--------|-------------|------|
| `/map` | `nav_msgs/OccupancyGrid` | 트랙 맵 |
| `/tf` | `tf2_msgs/TFMessage` | 좌표 변환 |
| `/tf_static` | `tf2_msgs/TFMessage` | 정적 좌표 변환 |
| `/joint_states` | `sensor_msgs/JointState` | 관절 상태 |

**Localization**
| 토픽명 | 메시지 타입 | 설명 |
|--------|-------------|------|
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 추정 위치 |
| `/particle_cloud` | `geometry_msgs/PoseArray` | 파티클 구름 |

### 구독 토픽 (Subscribed)

**제어 명령**
| 토픽명 | 메시지 타입 | 처리 주파수 | 설명 |
|--------|-------------|-------------|------|
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | 100Hz | 주 차량 조향/가속 명령 |
| `/cmd_vel` | `geometry_msgs/Twist` | 실시간 | 키보드 텔레오프 속도 명령 (kb_teleop 활성화 시) |
| `/opp_drive` | `ackermann_msgs/AckermannDriveStamped` | 100Hz | 상대방 드라이브 명령 (2 에이전트) |

**RViz 상호작용**
| 토픽명 | 메시지 타입 | 기능 | 설명 |
|--------|-------------|------|------|
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | ego 리셋 | 주 차량 위치 초기화 (RViz "2D Pose Estimate") |
| `/goal_pose` | `geometry_msgs/PoseStamped` | opp 리셋 | 상대방 차량 위치 설정 (RViz "2D Nav Goal") |

> ⚠️ **주의**: RViz 도구용 토픽은 직접 발행하지 마세요. RViz GUI 도구를 사용하세요.

## 토픽 세부 정보

### 라이다 스캔 (`sensor_msgs/LaserScan`)
- **각도 범위**: FOV 파라미터에 따라 설정 (기본값 확인 필요)
- **빔 개수**: scan_beams 파라미터에 따라 설정
- **최대 거리**: laser_max_range (기본 100.0m)
- **base_link로부터 거리**: scan_distance_to_base_link 파라미터

### 제어 명령 처리
- **시뮬레이션 스텝**: 100Hz (0.01초 간격)
- **센서 데이터 발행**: 250Hz (0.004초 간격)
- **Ackermann 드라이브**: speed (m/s), steering_angle (rad)
- **키보드 텔레오프**: linear.x (전진/후진), angular.z (좌/우 조향 ±0.3rad)

## 에이전트 개발

자체 에이전트 개발 방법:

1. **워크스페이스 내 패키지**: `~/sim_ws`에서 새 패키지 생성 후 별도 터미널에서 실행
2. **Docker 컨테이너**: 에이전트용 별도 컨테이너 생성하여 통신

## Localization 설정

**주요 AMCL 파라미터** (`localization/config/amcl_config.yaml`):
- `max_particles`: 2000 (파티클 최대 개수)
- `min_particles`: 500 (파티클 최소 개수) 
- `laser_max_range`: 100.0 (라이다 최대 범위)
- `base_frame_id`: "ego_racecar/base_link"