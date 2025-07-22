# Localization Package

F1TENTH 시뮬레이터와 함께 사용하는 AMCL(Adaptive Monte Carlo Localization) 패키지입니다.

## 개요

이 패키지는 f1tenth_gym_ros 시뮬레이터에서 실제 로컬라이제이션을 제공합니다:

- **Ground truth에서 실제 로컬라이제이션으로**: 시뮬레이터의 완벽한 위치 정보 대신 라이다 센서 데이터를 사용한 확률적 로컬라이제이션
- **AMCL 알고리즘**: 파티클 필터 기반 로컬라이제이션
- **표준 ROS2 구조**: `map` → `odom` → `base_link` transform 구조 구현

## 패키지 구성

```
localization/
├── config/
│   └── amcl_config.yaml          # AMCL 설정 파라미터
├── launch/
│   └── localization.launch.py    # AMCL 론치 파일
├── localization/
│   └── transform_remap.py        # Transform 구조 재매핑 노드
└── README.md
```

## 기능

### Transform 재매핑 노드 (transform_remap)
- `/ego_racecar/odom` (frame_id: 'map') → `/odom` (frame_id: 'odom')로 변환
- `odom` → `ego_racecar/base_link` transform 발행
- AMCL 호환 transform 구조 생성

### AMCL 로컬라이제이션
- 라이다 스캔 데이터를 사용한 위치 추정
- 파티클 필터 기반 확률적 로컬라이제이션
- `/amcl_pose` 토픽으로 추정 위치 발행
- `map` → `odom` transform 발행

## 사용법

### 1. 빌드
```bash
cd ~/sim_ws
colcon build --packages-select localization
source install/setup.bash
```

### 2. F1TENTH 시뮬레이터 실행
```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

### 3. Localization 실행 (새 터미널)
```bash
source ~/sim_ws/install/setup.bash
ros2 launch localization localization.launch.py
```

### 4. RViz에서 초기 위치 설정
- RViz에서 "2D Pose Estimate" 도구 사용
- 차량의 실제 위치에 화살표를 그어서 초기 위치 설정

## 토픽

### 발행 토픽
- `/odom` (nav_msgs/Odometry): 재매핑된 오도메트리
- `/amcl_pose` (geometry_msgs/PoseWithCovarianceStamped): AMCL 추정 위치
- `/particle_cloud` (nav2_msgs/ParticleCloud): 파티클 클라우드

### 구독 토픽  
- `/ego_racecar/odom` (nav_msgs/Odometry): 원본 오도메트리
- `/scan` (sensor_msgs/LaserScan): 라이다 스캔 데이터
- `/map` (nav_msgs/OccupancyGrid): 맵 데이터
- `/initialpose` (geometry_msgs/PoseWithCovarianceStamped): 초기 위치

## Transform 구조

### 기존 (Ground Truth)
```
map
└── ego_racecar/base_link
    └── ego_racecar/laser
```

### AMCL 적용 후
```
map
└── odom (AMCL이 발행)
    └── ego_racecar/base_link (transform_remap이 발행)
        └── ego_racecar/laser
```

## 설정

`config/amcl_config.yaml`에서 AMCL 파라미터 조정 가능:
- 파티클 수 (max_particles, min_particles)
- 모션 모델 노이즈 (alpha1-5)
- 센서 모델 파라미터 (laser_sigma_hit, laser_z_hit 등)
- 업데이트 임계값 (update_min_d, update_min_a)

## 주의사항

- AMCL 실행 전에 반드시 f1tenth_gym_ros가 실행되어 있어야 합니다
- RViz에서 초기 위치 설정이 필요합니다 (2D Pose Estimate)
- 로컬라이제이션 품질은 맵의 품질과 라이다 데이터에 의존합니다