# 🏁 F1TENTH ROS2 Workspace

F1TENTH 자율주행 레이싱 시뮬레이션 환경

## 📋 시스템 요구사항

- **OS**: Ubuntu 22.04 LTS
- **ROS**: ROS2 Humble
- **Python**: 3.10+

## 📦 패키지 구조

### 1. [simulation](simulation/) - 시뮬레이션 환경
F1TENTH Gym ↔ ROS2 브리지, 센서 데이터 발행, TF 관리

### 2. [f1tenth](f1tenth/) - 통합 자율주행 시스템
- **control**: Pure Pursuit 기반 경로 추종 및 조향 제어
- **localization**: EKF 및 AMCL 기반 위치 추정
- **path_planner**: 전역/지역 경로 계획
  - Global: 중심선/체크포인트 기반 전역 경로
  - Local: LiDAR 기반 실시간 장애물 회피
- **tools**: CLI 도구 (경로 최적화, 데이터 처리)

### 3. [real_system](real_system/) - 실제 로봇 하드웨어 제어
- F1TENTH 로봇 하드웨어 드라이버 (VESC, LiDAR, IMU)
- RealSense 카메라 및 IMU 센서

## 🛠️ 설치

### 1. F1TENTH Gym 설치
```bash
git clone https://github.com/t0mark/f1tenth.git -b gym gym
cd gym && pip install -e .
```

### 의존성
```bash
pip install numpy==1.26 scikit-image opencv-python PyYAML gymnasium transforms3d

sudo apt install ros-humble-librealsense2* \
  ros-humble-realsense2-* \
  ros-humble-cartographer ros-humble-cartographer-ros \
  ros-humble-nav2-map-server \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-ackermann-msgs
```

### 4. ROS2 패키지 빌드
```bash
mkdir -p ~/f1_ws
cd ~/f1_ws

git clone https://github.com/t0mark/f1tenth.git src

source /opt/ros/humble/setup.bash
colcon build --symlink-install
echo source ~/f1_ws/install/setup.bash >> ~/.bashrc

# 하드웨어 사용 시
mkdir -p ~/hw_ws
cd ~/hw_ws
git clone https://github.com/t0mark/f1tenth.git -b hardware src
colcon build --symlink-install
## 순서대로 실행
source ~/f1_ws/install/setup.bash
source ~/hw_ws/install/setup.bash


# 의존성 문제 발생 시
rosdep update
rosdep install -i --from-path src --rosdistro humble -y
```

> **참고:** `colcon build` 실행 시 `vesc_ackermann`, `vesc_driver` 관련 경고가 나타날 수 있습니다. 이는 패키지 개발자를 위한 권장 사항으로, 사용자에게는 영향을 주지 않으므로 무시해도 괜찮습니다.

## 🚀 빠른 시작

### 실제 로봇 연결
```bash
# F1TENTH 로봇 연결
ros2 launch real_system hardware_launch.py

# RealSense 카메라 실행
ros2 launch real_system camera_launch.py
```

### 시뮬레이션 - 통합 시스템 실행
```bash
# 시뮬레이터
ros2 launch f1tenth full_system_launch.py
```

### 체크포인트 기록
```bash
# RViz에서 "Publish Point"로 경로 기록
ros2 launch f1tenth checkpoint_recorder_launch.py
```

### CLI 도구 사용
```bash
# 레이싱 라인 최적화
f1tenth-optimize-racing-line

# 트랙 폭 데이터 병합
f1tenth-merge-widths
```

## 🔗 주요 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/scan` | LaserScan | LiDAR 데이터 |
| `/odom` | Odometry | 차량 위치 |
| `/global_path` | Path | 전역 경로 |
| `/local_path` | Path | 지역 경로 |
| `/drive` | AckermannDriveStamped | 차량 제어 명령 |

## 📂 디렉토리 구조
