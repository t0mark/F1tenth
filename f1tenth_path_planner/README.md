# F1TENTH Path Planner (ROS2)

Global centerline-based waypoint generator and LiDAR local avoidance path for F1TENTH Gym ROS2.

## Nodes

- `global_centerline_node`
  - Reads the track image, extracts a skeletonized centerline, converts to map coordinates, and publishes `nav_msgs/Path` on `/global_path`.
  - Parameters:
    - `map_path` (string): Root path to map without extension. If empty, auto-loads from `f1tenth_gym_ros/config/sim.yaml`.
    - `map_img_ext` (string): Image extension (default `.png`).
    - `map_yaml_path` (string): Full YAML path; defaults to `map_path + '.yaml'`.
    - `sample_step_m` (double): Waypoint spacing in meters (default 0.2).
    - `publish_topic` (string): Output topic, default `/global_path`.
    - `save_centerline_overlay` (bool): Save `<map>_centerline.png` overlay.

- `local_avoidance_node`
  - Subscribes to `/global_path` and `/scan` and produces a short-horizon collision-free local `nav_msgs/Path` on `/local_path` by choosing a lateral offset with best LiDAR clearance.
  - Parameters:
    - `global_path_topic` (string): Default `/global_path`.
    - `scan_topic` (string): Default `/scan` (use `/scan_fixed` if running SLAM fix).
    - `local_horizon` (double): Horizon length in meters (default 8.0).
    - `path_resolution` (double): Point spacing in meters (default 0.2).
    - `lateral_offsets` (list): Candidate lateral offsets in meters (default [0.0, 0.4, -0.4]).
    - `safety_radius` (double): Safety margin in meters (default 0.4).
    - `base_frame` (string): Default `ego_racecar/base_link`.
    - `map_frame` (string): Default `map`.

## Launch

```
ros2 launch f1tenth_path_planner path_planner_launch.py
```
- Auto-reads `map_path` from `f1tenth_gym_ros/config/sim.yaml` if available.

## Topics

- Published:
  - `/global_path` (`nav_msgs/Path`, frame `map`)
  - `/local_path` (`nav_msgs/Path`, frame `map`)
- Subscribed:
  - `/scan` (`sensor_msgs/LaserScan`) or `/scan_fixed`

## Dependencies

- ROS2: `rclpy`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `tf2_ros`
- Python: `opencv-python`, `scikit-image`, `numpy`, `PyYAML`

Example install (system-wide or venv):
```
pip3 install opencv-python scikit-image numpy PyYAML
```

## Notes

- Only path generation is implemented. No velocity/steering control.
- Centerline overlay saved at `<map_path>_centerline.png`.
- Local path planner is a lightweight LiDAR-based lateral offset selector intended as a baseline.

