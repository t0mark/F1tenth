# 센터라인 추출 실행 순서

1. 손으로 Centerline 추출
```
ros2 launch path_planner checkpoint_recorder_launch.py
```


2. fernet 좌표계 형식으로 변환 및 가우시안 필터링으로 속도값 추가
```
python3 src/path_planner/scripts/traj_file_generator.py
```

<div style="text-align: center;">
  <div style="margin-bottom: 10px;">
    <img src="src/path_planner/data/fitted_waypoints_speed_heatmap.png" width="100%">
    <p>거리 정보 저장</p>
  </div>
</div>

3. 시뮬레이션을 통해 centerline 기준 트랙 폭 저장

```
# 터미널 1
ros2 launch f1tenth simulation_full_system_launch.py
#터미널 2
ros2 run path_planner centerline_logger_node
```
<div style="text-align: center;">
  <div style="margin-bottom: 10px;">
    <img src="src/path_planner/data/width_log_visualization.png" width="100%">
    <p>거리 정보 저장</p>
  </div>
</div>


4. 위 두 csv 파일 통합
```
python3 src/path_planner/scripts/width_left_right_creator.py
```