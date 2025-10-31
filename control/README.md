# 🚗 Control Package

Pure Pursuit 기반 차량 제어 패키지

## 📦 의존성

**ROS2**: `rclpy`, `geometry_msgs`, `nav_msgs`, `ackermann_msgs`, `visualization_msgs`
**Python**: `setuptools`

## 🏗️ 빌드

```bash
cd ~/f110_ws
colcon build --packages-select control
source install/setup.bash
```

## 🚀 사용법

### 노드: `pure_pursuit_controller`
`local_path` 우선 사용, 유효하지 않으면 `global_path` 사용

**주요 파라미터**:

차량 파라미터:
- `wheelbase` (0.3302m): 차량 휠베이스
- `max_steering_angle` (0.4189 rad): 최대 조향각

속도 제어:
- `v_min` (1.3 m/s): 최소 속도 (급커브)
- `v_max` (5.0 m/s): 최대 속도 (직선)
- `max_curvature` (1.0): 곡률 정규화 기준값

Lookahead 거리:
- `ld_min` (0.8m): 최소 전방 주시 거리
- `ld_max` (2.1m): 최대 전방 주시 거리

제어 파라미터:
- `local_path_timeout` (1.0초): 로컬 경로 유효 시간
- `control_rate_hz` (50.0 Hz): 제어 루프 주파수
- `steer_smooth_alpha` (0.3): 조향 스무딩 계수 (0-1, 높을수록 반응 빠름)

**실행**:
```bash
# 기본 설정
ros2 launch control pure_pursuit_launch.py

# 파라미터 변경 예시
ros2 launch control pure_pursuit_launch.py v_max:=7.0 ld_max:=3.0 steer_smooth_alpha:=0.5
```

### 토픽

**발행**:
- `/drive` (AckermannDriveStamped): 주행 명령
- `/lookahead_point` (PointStamped): 목표점 시각화
- `/speed_marker` (Marker): 현재 속도 표시
- `/steering_marker` (Marker): 조향각 표시

**구독**:
- `/local_path` (Path): 지역 경로 (우선)
- `/global_path` (Path): 전역 경로 (폴백)
- `/ego_racecar/odom` (Odometry): 차량 위치

### Pure Pursuit 알고리즘

1. **경로 선택**: local path 우선, 타임아웃 시 global path 사용
2. **Adaptive Lookahead**: 현재 속도에 비례하여 lookahead 거리 조정
3. **목표점 탐색**: 전방 주시 중 lookahead 거리에 위치한 점 찾기
4. **곡률 계산**: 목표점 주변 3점을 이용한 Menger curvature 계산
5. **Adaptive Speed**: 곡률이 높을수록 속도 감소 (v_min ~ v_max)
6. **조향각 계산**: `atan2(2 * L * sin(α), ld)`
7. **Steering Smoothing**: 지수 이동 평균으로 조향 안정화
8. **명령 발행**: 속도 및 조향각 명령 발행

**안전 메커니즘**:
- 경로 유효성 검사 (타임아웃 체크)
- 목표점 탐색 실패 시 정지
- 조향각 물리적 제한
- 전방 주시 기반 가장 가까운 점 탐색
