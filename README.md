# F1TENTH gym environment ROS2 통신 브릿지
F1TENTH gym 환경을 ROS2 시뮬레이션으로 변환하는 ROS 통신 브릿지입니다.

# 요구 사항
- Ubuntu 20.04 LTS
- ROS 2 Foxy

# 설치
## 의존성 설정
**Python 패키지 의존성**
  ```bash
  pip3 install setuptools==59.6.0 \
    testresources wheel numpy matplotlib pyyaml \
    gymnasium pybullet-utils transforms3d
  ```

**시스템 의존성**
   ```bash
   sudo apt-get update
   sudo apt-get install python3-dev build-essential
   ```

**F1TENTH Gym**
  ```bash
  git clone https://github.com/f1tenth/f1tenth_gym
  cd f1tenth_gym && pip3 install -e .
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

## 시뮬레이션 설치
```bash
mkdir -p ~/sim_ws/src
cd ~/sim_ws/src
git clone https://github.com/f1tenth/f1tenth_gym_ros

# 파일 수정 f1tenth_gym_ros/config/sim.yaml
## 수정 전
45 | map_path: '/sim_ws/src/f1tenth_gym_ros/maps/levine'
## 수정 후
45 | map_path: '{Home 디렉토리}/sim_ws/src/f1tenth_gym_ros/maps/levine'

source /opt/ros/foxy/setup.bash
cd ~/sim_ws
rosdep update
rosdep install -i --from-path src --rosdistro foxy -y
colcon build

echo "source ~/sim_ws/install/local_setup.bash" >> ~/.bashrc
```

# 실행

## 시뮬레이션 실행
```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

## 키보드 텔레오프
- 키보드 텔레오프를 활성화하려면 config/sim.yaml - `kb_teleop: True` 설정
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
| 동작           | 전진 | 전진+좌회전 | 전진+우회전 | 후진 | 후진+좌회전 | 후진+우회전 | 정지 |
|----------------|------|-------------|-------------|------|-------------|-------------|------|
| 키(Key) 입력   | `i`  | `u`         | `o`         | `,`  | `m`         | `.`         | `k`  |

# 시뮬레이션 구성
- 시뮬레이션 구성 파일은 `f1tenth_gym_ros/config/sim.yaml`에 있습니다.
- 토픽 이름과 네임스페이스는 구성할 수 있지만 그대로 두는 것을 권장합니다.
- `map_path` 매개변수를 통해 맵을 변경할 수 있습니다. 맵 파일의 전체 경로를 사용해야 합니다. 맵은 ROS 규칙을 따르며, 이미지 파일과 맵의 `yaml` 파일이 같은 디렉토리에 같은 이름으로 있다고 가정합니다.
- `num_agent` 매개변수는 단일 또는 두 에이전트 레이싱을 위해 1 또는 2로 변경할 수 있습니다.
- ego와 상대방의 시작 위치도 매개변수를 통해 변경할 수 있으며, 이는 글로벌 맵 좌표계에 있습니다.

구성을 변경한 후에는 변경사항이 반영되도록 워크스페이스에서 `colcon build`를 다시 실행하세요.

# 토픽 구성

## 발행 토픽 (Published Topics)

### 센서 데이터
| 토픽명 | 메시지 타입 | 설명 |
|--------|-------------|------|
| `/scan` | `sensor_msgs/LaserScan` | Ego 에이전트의 라이다 스캔 데이터 |
| `/ego_racecar/odom` | `nav_msgs/Odometry` | Ego 에이전트의 위치/속도 정보 |
| `/opp_scan` | `sensor_msgs/LaserScan` | 상대 에이전트의 라이다 스캔 (2 에이전트 모드) |
| `/opp_racecar/odom` | `nav_msgs/Odometry` | 상대 에이전트의 오도메트리 (2 에이전트 모드) |

### 환경 정보  
| 토픽명 | 메시지 타입 | 설명 |
|--------|-------------|------|
| `/map` | `nav_msgs/OccupancyGrid` | 트랙 맵 데이터 |
| `/tf` | `tf2_msgs/TFMessage` | 좌표 변환 정보 |
| `/tf_static` | `tf2_msgs/TFMessage` | 정적 좌표 변환 정보 |
| `/joint_states` | `sensor_msgs/JointState` | 로봇 관절 상태 정보 |

### 기타
| 토픽명 | 메시지 타입 | 설명 |
|--------|-------------|------|
| `/clock` | `rosgraph_msgs/Clock` | 시뮬레이션 시간 |
| `/ego_robot_description` | `std_msgs/String` | Ego 로봇 URDF 설명 |

## 구독 토픽 (Subscribed Topics)

### 제어 명령
| 토픽명 | 메시지 타입 | 설명 |
|--------|-------------|------|
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | Ego 에이전트 조향/가속 명령 |
| `/cmd_vel` | `geometry_msgs/Twist` | 키보드 텔레오프용 속도 명령 |
| `/opp_drive` | `ackermann_msgs/AckermannDriveStamped` | 상대 에이전트 드라이브 명령 (2 에이전트 모드) |

### RViz 인터랙션
| 토픽명 | 메시지 타입 | 설명 |
|--------|-------------|------|
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | Ego 에이전트 초기 위치 설정 (RViz 2D Pose Estimate) |
| `/goal_pose` | `geometry_msgs/PoseStamped` | 상대 에이전트 목표 위치 설정 (RViz 2D Goal Pose) |
| `/clicked_point` | `geometry_msgs/PointStamped` | RViz에서 클릭한 점 정보 |

> ⚠️ **주의**: `/initialpose`와 `/goal_pose` 토픽은 RViz 도구와 함께 사용하도록 설계되었습니다. 직접 발행하지 마세요.

# ROS 2에서 자체 에이전트 개발 및 생성

차량을 제어하는 자체 에이전트를 시작하는 여러 가지 방법이 있습니다.
- 첫 번째는 `/sim_ws` 워크스페이스에서 에이전트를 위한 새 패키지를 만드는 것입니다. 시뮬레이션을 실행한 후 시뮬레이션이 실행되는 동안 다른 bash 세션에서 에이전트 노드를 실행합니다.
- 두 번째는 에이전트 노드를 위한 새 ROS 2 컨테이너를 만드는 것입니다. 그런 다음 내부에 자체 패키지와 노드를 만듭니다. 시뮬레이션 컨테이너와 에이전트 컨테이너를 모두 실행합니다. `docker`의 기본 네트워킹 구성을 사용하면 두 컨테이너가 같은 네트워크에 배치되어 서로 다른 토픽에서 서로를 발견하고 통신할 수 있습니다.
