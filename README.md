<div style="text-align: center;">
  <div style="margin-bottom: 10px;">
    <img src="img/avoidance.png" width="100%">
    <p>회피 경로</p>
  </div>
</div>

# 🏁 F1TENTH ROS2 Workspace

F1TENTH 자율주행 레이싱 시뮬레이션 환경

## 📋 시스템 요구사항

- **OS**: Ubuntu 22.04 LTS
- **ROS**: ROS2 Humble
- **Python**: 3.10+

## 📦 패키지 구조

### 1. [simulator](simulator/) - 시뮬레이션 환경
F1TENTH Gym ↔ ROS2 브리지, 센서 데이터 발행, TF 관리

### 2. [path_planner](path_planner/) - 경로 계획
- **Global**: 중심선/체크포인트 기반 전역 경로
- **Local**: LiDAR 기반 실시간 장애물 회피

### 3. [control](control/) - 차량 제어
Pure Pursuit 기반 경로 추종 및 조향 제어

## 🛠️ 설치

### 1. F1TENTH Gym 설치
```bash
git clone https://github.com/t0mark/f1tenth.git -b gym gym
cd gym && pip install -e .
```

### 의존성
```bash
pip install numpy scikit-image opencv-python PyYAML gymnasium transforms3d

sudo apt install ros-humble-librealsense2*
sudo apt install ros-humble-realsense2-*
sudo apt install ros-humble-cartographer ros-humble-cartographer-ros
```

### 4. ROS2 패키지 빌드
```bash
mkdir -p ~/f1_ws
cd ~/f1_ws

git clone https://github.com/t0mark/f1tenth.git src
# 하드웨어 사용 시
git clone https://github.com/t0mark/f1tenth.git -b hardware src/hardware

source /opt/ros/humble/setup.bash
rosdep update
rosdep install -i --from-path src --rosdistro humble -y
colcon build
source install/setup.bash
```

> **참고:** `colcon build` 실행 시 `vesc_ackermann`, `vesc_driver` 관련 경고가 나타날 수 있습니다. 이는 패키지 개발자를 위한 권장 사항으로, 사용자에게는 영향을 주지 않으므로 무시해도 괜찮습니다.

## 🚀 빠른 시작

### 실제 로봇 연결
```bash
# F1TENTH 로봇 연결 (VESC, 센서 등)
ros2 launch f1tenth hardware_launch.py

# RealSense 카메라 실행
ros2 launch f1tenth camera_launch.py
```

### 시뮬레이션 - 통합 시스템 실행
```bash
# 중심선 기반 경로 + 장애물 회피 + 제어
ros2 launch path_planner path_planner_launch.py
```

### 시뮬레이션 - 개별 패키지 실행
```bash
# 1. 시뮬레이터
ros2 launch simulator gym_bridge_launch.py
```

### 체크포인트 기록
```bash
# RViz에서 "Publish Point"로 경로 기록
ros2 launch path_planner checkpoint_recorder_launch.py
```

### 키보드 제어
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 🔗 주요 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/scan` | LaserScan | LiDAR 데이터 |
| `/ego_racecar/odom` | Odometry | 차량 위치 |
| `/global_path` | Path | 전역 경로 |
| `/local_path` | Path | 지역 경로 |
| `/drive` | AckermannDriveStamped | 차량 제어 명령 |

## 📂 디렉토리 구조

```
src/
├── simulator/         # 시뮬레이터 브리지
│   ├── config/        # 맵, 파라미터
│   └── urdf/          # 차량 모델
├── path_planner/      # 경로 계획
│   ├── config/        # 플래너 설정
│   └── data/          # 체크포인트
└── control/           # 차량 제어
    └── config/        # 제어 파라미터
```
