# 🚗 Control Package

Pure Pursuit 기반 차량 제어 패키지

## 📦 의존성

**ROS2**: `rclpy`, `geometry_msgs`, `nav_msgs`, `ackermann_msgs`, `tf2_ros`
**Python**: `setuptools`

## 🏗️ 빌드

```bash
cd ~/f1_ws
colcon build --packages-select control
source install/setup.bash
```

## 🚀 사용법

### 노드: `pure_pursuit_controller`
`local_path` 우선 사용, 유효하지 않으면 `global_path` 사용

**주요 파라미터**:
- `lookahead_distance` (2.5m): 전방 주시 거리
- `speed` (0.3 m/s): 고정 속도
- `wheelbase` (0.3302m): 차량 휠베이스
- `max_steering_angle` (0.4189 rad): 최대 조향각

**실행**:
```bash
# 기본 설정
ros2 launch control pure_pursuit_launch.py

# 파라미터 변경
ros2 launch control pure_pursuit_launch.py lookahead_distance:=3.0 speed:=1.5
```

### 토픽

**발행**: `/drive` (AckermannDriveStamped)

**구독**:
- `/local_path` (Path): 지역 경로 (우선)
- `/global_path` (Path): 전역 경로 (폴백)
- `/ego_racecar/odom` (Odometry): 차량 위치

### Pure Pursuit 알고리즘

1. 경로 선택 (local → global)
2. 목표점 탐색 (lookahead_distance 기반)
3. 조향각 계산: `atan2(2 * L * sin(α), ld)`
4. 조향각 제한 및 명령 발행

**안전 메커니즘**:
- 경로 유효성 검사
- 목표점 탐색 실패 시 정지
- 조향각 물리적 제한
