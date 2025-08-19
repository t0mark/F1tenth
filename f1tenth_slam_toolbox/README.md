# 🗺️ F1tenth SLAM Toolbox

<div align="center">
  <img src="https://raw.githubusercontent.com/SteveMacenski/slam_toolbox/ros2/docs/logo.png" alt="SLAM Toolbox Logo" width="300">
</div>

F1tenth Racing Simulation을 위한 SLAM Toolbox 설정 패키지입니다. 기존 slam_toolbox와의 TF 충돌을 방지하고 F1tenth 시뮬레이션 환경에 최적화된 설정을 제공합니다.

## ⚠️ 해결된 문제점들

<div align="center">
  <img src="https://docs.ros.org/en/humble/_images/tf2_frames_example.png" alt="TF Frames" width="400">
</div>

### 1. 🔗 TF 프레임 및 토픽 이름 문제

**문제**: SLAM toolbox가 `ego_racecar/laser` 프레임을 인식하지 못함
- F1tenth gym이 `/scan` 토픽을 `ego_racecar/laser` frame_id로 발행
- SLAM toolbox가 `ego_racecar/base_link`와의 TF 관계를 찾지 못해 메시지 드롭

**해결방안**:
- `scan_header_fix.py`에서 frame_id를 `ego_racecar/base_link`로 변경
- `/scan` → `/scan_fixed` 토픽 리매핑으로 수정된 스캔 데이터 사용

### 2. 📡 LiDAR 스캔 개수 문제

<div align="center">
  <img src="https://classic.gazebosim.org/assets/tutorials/ros_gzplugins/hokuyo_laser.png" alt="LiDAR Scan" width="300">
</div>

**문제**: 스캔 데이터 개수 불일치
- F1tenth gym: 1080개 beam 데이터 생성
- SLAM toolbox: 1081개 beam 예상 (angle_min/max/increment 불일치)

**해결방안**:
- `scan_header_fix.py`에서 `angle_max` 값을 실제 데이터 개수에 맞게 재계산
- `angle_max = angle_min + angle_increment * (num_readings - 1)`

### 3. ⏰ 시간 동기화 문제

<div align="center">
  <img src="https://docs.ros.org/en/humble/_images/time_sync.png" alt="Time Synchronization" width="350">
</div>

**문제**: 패키지 간 시간 설정 불일치
- F1tenth gym: `use_sim_time: True` (하드코딩)
- SLAM toolbox: 초기 설정 누락
- F1tenth gym이 `/clock` 토픽을 발행하지 않아 시뮬레이션 시간 동기화 불가

**해결방안**:
- 모든 패키지를 `use_sim_time: false`로 통일
- 실시간 시뮬레이션이므로 실제 시계 사용이 적합

## 📁 패키지 구조

```
f1tenth_slam_toolbox/
├── config/
│   └── f1tenth_mapper_params.yaml    # SLAM toolbox 매개변수
├── launch/
│   ├── f1tenth_slam_launch.py        # 메인 런치 파일
│   └── scan_header_fix.py            # 스캔 헤더 수정 노드
├── CMakeLists.txt
├── package.xml
└── README.md
```

## ⚙️ 설정 파일

### f1tenth_mapper_params.yaml
- `use_sim_time: false` - 실제 시간 사용
- `publish_tf: false` - TF 발행 비활성화 (F1tenth가 제공)
- `odom_frame: map` - F1tenth의 직접 map→base_link 변환 사용
- `base_frame: ego_racecar/base_link` - 시뮬레이터와 일치

### f1tenth_slam_launch.py
- map↔odom 항등 TF 발행
- 스캔 헤더 수정 노드 실행
- SLAM toolbox 노드 실행 (`/scan_fixed` 사용)

## 🔄 입출력 토픽

### 입력
- `/scan` - F1tenth gym에서 발행하는 원본 LiDAR 데이터

### 출력
- `/scan_fixed` - 헤더 수정된 LiDAR 데이터
- `/slam_map` - SLAM이 생성한 지도 (`/map`에서 리매핑)

## 🚀 사용법

<div align="center">
  <img src="https://navigation.ros.org/images/slam_toolbox_demo.gif" alt="SLAM Demo" width="500">
</div>

```bash
# 1단계: F1tenth 시뮬레이션 실행
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

# 2단계: 새 터미널에서 SLAM toolbox 실행  
ros2 launch f1tenth_slam_toolbox f1tenth_slam_launch.py

# 3단계: 차량을 움직여서 지도 생성
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 📊 결과 확인
- **RViz**: 실시간 지도 생성 과정 시각화
- **토픽**: `/slam_map`에서 생성된 지도 확인
- **TF**: 로봇 위치 추정 결과 확인