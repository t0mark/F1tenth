# 🧭 Path Planner Package

전역/지역 경로 계획 패키지

## 📦 의존성

**ROS2**: `rclpy`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `tf2_ros`, `std_srvs`
**Python**: `numpy`, `scikit-image`, `opencv-python`, `PyYAML`

## 🏗️ 빌드

```bash
cd ~/f1_ws
colcon build --packages-select path_planner
source install/setup.bash
```

## 🚀 노드

### 1. `global_centerline_node`
맵 이미지에서 중심선 추출 → 전역 경로 생성

**주요 파라미터**:
- `map_path`: 맵 이미지 경로
- `sample_step_m` (0.2m): 웨이포인트 간격

**발행**: `/global_path`

### 2. `global_checkpoint_node`
CSV 파일의 체크포인트 → 전역 경로 생성

**주요 파라미터**:
- `checkpoint_csv_path`: 체크포인트 파일 경로

**발행**: `/global_path`
**구독**: `/initialpose` (RViz에서 시작점 변경)

### 3. `local_avoidance_node`
LiDAR 기반 실시간 장애물 회피 경로 생성

**주요 파라미터**:
- `local_horizon` (8.0m): 지역 경로 유효 거리
- `lateral_offsets` ([0.0, 0.4, -0.4]m): 횡방향 오프셋 후보
- `safety_radius` (0.4m): 안전 반경

**발행**: `/local_path`
**구독**: `/global_path`, `/scan`

### 4. `local_dwa_node`
로봇 기준 로컬 비용맵을 구성하고 DWA로 `/cmd_vel`과 로컬 경로를 동시에 생성

**주요 파라미터**:
- `costmap_size` (12m), `costmap_resolution` (0.1m)
- `sim_time` (2.0s), `vx_samples`/`omega_samples`
- `heading/velocity/clearance_weight`: 비용 가중치
- `publish_cmd_vel`: true일 때만 `/cmd_vel` 발행

**발행**: `/local_costmap` (`OccupancyGrid`), `/local_path`, `/cmd_vel`
**구독**: `/global_path`, `/odom`, `/scan`

### 5. `checkpoint_recorder_node`
RViz에서 클릭하여 체크포인트 기록

**서비스**:
- `/save_checkpoints`: CSV 저장
- `/clear_checkpoints`: 체크포인트 초기화

**발행**: `/checkpoint_path`
**구독**: `/clicked_point` (RViz "Publish Point")

## 🎯 런치 파일

```bash
# 중심선 기반 경로 계획
ros2 launch path_planner centerline_avoidance_launch.py

# 체크포인트 기반 경로 계획
ros2 launch path_planner checkpoint_avoidance_launch.py

# 체크포인트 기록
ros2 launch path_planner checkpoint_recorder_launch.py

# 통합 런치 (설정 파일 선택 가능)
ros2 launch path_planner path_planner_launch.py \
  global_config:=global_centerline.yaml \
  local_config:=local_avoidance.yaml \
  is_integrated:=false
```

## 📂 구조

- `config/`: 노드 파라미터 설정 파일
- `data/`: 체크포인트 CSV 파일
- `utils.py`: 중심선 추출, 좌표 변환 유틸리티
