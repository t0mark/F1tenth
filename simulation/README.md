# 🏎️ Simulator Package

F1TENTH Gym ↔ ROS2 브리지 패키지

## 📦 의존성

**ROS2**: `rclpy`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `ackermann_msgs`, `tf2_ros`, `nav2_map_server`, `nav2_lifecycle_manager`, `teleop_twist_keyboard`
**Python**: `gymnasium`, `numpy`, `transforms3d`, `PyYAML`

## 🏗️ 빌드

```bash
cd ~/f1_ws
colcon build --packages-select simulator
source install/setup.bash
```

## 🚀 사용법

### 노드: `gym_bridge`
시뮬레이터 ↔ ROS2 핵심 브리지

**주요 파라미터**:
- `num_agent` (1): 차량 수 (1 또는 2)
- `map_path`: 맵 파일 경로
- `sx`, `sy`, `stheta`: 초기 위치
- `scan_fov` (4.7 rad), `scan_beams` (1080): LiDAR 설정
- `kb_teleop` (true): 키보드 제어 활성화

**실행**:
```bash
# 기본 맵
ros2 launch simulator gym_bridge_launch.py

# 특정 맵
ros2 launch simulator gym_bridge_launch.py \
  map_path:=$(ros2 pkg prefix f1tenth)/share/f1tenth/maps/Spielberg_map.yaml
```

### 토픽

**발행** (250 Hz):
- `/scan` (LaserScan): LiDAR 데이터
- `/ego_racecar/odom` (Odometry): 차량 오도메트리
- `/map` (OccupancyGrid): 맵 데이터
- `/tf`, `/tf_static`: 좌표 변환

**구독**:
- `/ego_racecar/drive` (AckermannDriveStamped): 차량 제어
- `/cmd_vel` (Twist): 키보드 제어
- `/initialpose` (PoseWithCovarianceStamped): RViz 초기 위치

### 키보드 제어

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**키**:
- `i`: 전진, `o`: 전진+우회전, `u`: 전진+좌회전
- `,`: 후진, `j`: 좌회전, `l`: 우회전, `k`: 정지

### TF 구조

```
map
└── ego_racecar/base_link
    ├── ego_racecar/laser
    ├── ego_racecar/front_left_hinge
    │   └── ego_racecar/front_left_wheel
    └── ego_racecar/front_right_hinge
        └── ego_racecar/front_right_wheel
```

## 📂 구조

- `config/sim.yaml`: 기본 파라미터
- *(맵 파일은 `f1tenth` 패키지의 `maps/` 디렉터리에 위치)*
- `urdf/`: 차량 모델 (Xacro)
- `rviz/`: RViz 설정
