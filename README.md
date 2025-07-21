# F1TENTH gym environment ROS2 통신 브릿지
F1TENTH gym 환경을 ROS2 시뮬레이션으로 변환하는 ROS 통신 브릿지입니다.

# 설치

## Ubuntu 20.04 네이티브 설치

**다음 의존성을 설치하세요:**

1. **ROS 2** [여기](https://docs.ros.org/en/foxy/Installation.html)의 지침을 따라 ROS 2 Foxy를 설치합니다.

2. **Python 패키지 의존성**
   ```bash
   pip3 install setuptools==59.6.0
   pip3 install testresources
   pip3 install wheel numpy matplotlib pyyaml
   pip3 install gymnasium pybullet-utils transforms3d
   ```

3. **시스템 의존성**
   ```bash
   sudo apt-get update
   sudo apt-get install python3-dev build-essential
   ```

4. **F1TENTH Gym**
   ```bash
   git clone https://github.com/f1tenth/f1tenth_gym
   cd f1tenth_gym && pip3 install -e .
   ```

5. **ROS 2 패키지**
   ```bash
   sudo apt update
   sudo apt install ros-foxy-joint-state-publisher ros-foxy-joint-state-publisher-gui
   sudo apt install ros-foxy-robot-state-publisher ros-foxy-xacro
   sudo apt install ros-foxy-navigation2 ros-foxy-nav2-bringup
   sudo apt install ros-foxy-rviz2
   sudo apt install ros-foxy-tf2-tools ros-foxy-tf2-ros-py
   ```

**시뮬레이션 설치:**
- 워크스페이스 생성: ```cd ~ && mkdir -p sim_ws/src```
- 워크스페이스에 저장소 복제:
  ```bash
  cd ~/sim_ws/src
  git clone https://github.com/f1tenth/f1tenth_gym_ros
  ```
- 맵 파일 경로 매개변수 업데이트:
  복제된 저장소의 `sim.yaml` [https://github.com/f1tenth/f1tenth_gym_ros/blob/main/config/sim.yaml](https://github.com/f1tenth/f1tenth_gym_ros/blob/main/config/sim.yaml)로 이동하여 `map_path` 매개변수를 올바른 위치로 변경합니다. `'~/sim_ws/src/f1tenth_gym_ros/maps/levine'`로 설정해야 합니다.
- rosdep으로 추가 의존성 설치:
  ```bash
  source /opt/ros/foxy/setup.bash
  cd ..
  rosdep update
  rosdep install -i --from-path src --rosdistro foxy -y
  ```
- 워크스페이스 빌드: ```colcon build```
- 로컬 설정 소스: ```source install/local_setup.bash```

# 시뮬레이션 실행

시뮬레이션을 실행하려면 ROS2 설정 스크립트와 로컬 워크스페이스 설정 스크립트를 모두 소스해야 합니다:
```bash
$ source /opt/ros/foxy/setup.bash
$ source install/local_setup.bash
$ ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
rviz 창이 나타나 시뮬레이션을 표시합니다.

# 시뮬레이션 구성
- 시뮬레이션 구성 파일은 `f1tenth_gym_ros/config/sim.yaml`에 있습니다.
- 토픽 이름과 네임스페이스는 구성할 수 있지만 그대로 두는 것을 권장합니다.
- `map_path` 매개변수를 통해 맵을 변경할 수 있습니다. 맵 파일의 전체 경로를 사용해야 합니다. 맵은 ROS 규칙을 따르며, 이미지 파일과 맵의 `yaml` 파일이 같은 디렉토리에 같은 이름으로 있다고 가정합니다.
- `num_agent` 매개변수는 단일 또는 두 에이전트 레이싱을 위해 1 또는 2로 변경할 수 있습니다.
- ego와 상대방의 시작 위치도 매개변수를 통해 변경할 수 있으며, 이는 글로벌 맵 좌표계에 있습니다.

구성을 변경한 후에는 변경사항이 반영되도록 워크스페이스에서 `colcon build`를 다시 실행하세요.

# 시뮬레이션에서 발행하는 토픽

**단일** 에이전트:

`/scan`: ego 에이전트의 레이저 스캔

`/ego_racecar/odom`: ego 에이전트의 오도메트리

`/map`: 환경의 맵

`tf` 트리도 유지됩니다.

**두** 에이전트:

단일 에이전트 시나리오에서 사용 가능한 토픽 외에도 다음 토픽들도 사용 가능합니다:

`/opp_scan`: 상대 에이전트의 레이저 스캔

`/ego_racecar/opp_odom`: ego 에이전트의 계획자를 위한 상대 에이전트의 오도메트리

`/opp_racecar/odom`: 상대 에이전트의 오도메트리

`/opp_racecar/opp_odom`: 상대 에이전트의 계획자를 위한 ego 에이전트의 오도메트리

# 시뮬레이션에서 구독하는 토픽

**단일** 에이전트:

`/drive`: `AckermannDriveStamped` 메시지를 통한 ego 에이전트의 드라이브 명령

`/initalpose`: RViz의 2D Pose Estimate 도구를 통한 ego 위치 재설정을 위한 토픽입니다. 무엇을 하는지 모르면 이 토픽에 직접 발행하지 **마세요**.

**두** 에이전트:

단일 에이전트 시나리오의 모든 토픽 외에도 다음 토픽들도 사용 가능합니다:

`/opp_drive`: `AckermannDriveStamped` 메시지를 통한 상대 에이전트의 드라이브 명령. 2개의 에이전트를 사용할 때 차량이 움직이려면 ego의 드라이브 토픽과 상대방의 드라이브 토픽 **모두**에 발행해야 합니다.

`/goal_pose`: RViz의 2D Goal Pose 도구를 통한 상대 에이전트 위치 재설정을 위한 토픽입니다. 무엇을 하는지 모르면 이 토픽에 직접 발행하지 **마세요**.

# 키보드 텔레오프

`teleop_twist_keyboard`의 키보드 텔레오프 노드도 시뮬레이션의 의존성의 일부로 설치됩니다. 키보드 텔레오프를 활성화하려면 `sim.yaml`에서 `kb_teleop`를 `True`로 설정하세요. 시뮬레이션을 시작한 후 다른 터미널에서 실행하세요:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
그런 다음 텔레오프 노드를 실행하는 터미널 창에서 `i`를 눌러 전진, `u`와 `o`를 눌러 전진하며 회전, `,`를 눌러 후진, `m`과 `.`를 눌러 후진하며 회전, `k`를 눌러 정지할 수 있습니다.

# ROS 2에서 자체 에이전트 개발 및 생성

차량을 제어하는 자체 에이전트를 시작하는 여러 가지 방법이 있습니다.

- 첫 번째는 `/sim_ws` 워크스페이스에서 에이전트를 위한 새 패키지를 만드는 것입니다. 시뮬레이션을 실행한 후 시뮬레이션이 실행되는 동안 다른 bash 세션에서 에이전트 노드를 실행합니다.
- 두 번째는 에이전트 노드를 위한 새 ROS 2 컨테이너를 만드는 것입니다. 그런 다음 내부에 자체 패키지와 노드를 만듭니다. 시뮬레이션 컨테이너와 에이전트 컨테이너를 모두 실행합니다. `docker`의 기본 네트워킹 구성을 사용하면 두 컨테이너가 같은 네트워크에 배치되어 서로 다른 토픽에서 서로를 발견하고 통신할 수 있습니다.
