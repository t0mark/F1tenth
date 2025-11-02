#!/usr/bin/env python3

"""Estimate drive command scaling factors from real vehicle data.

Pipeline:
1. Record real vehicle data (joystick commands + sensors) in a rosbag
2. Replay bag + run localization (map_server + AMCL/EKF + tf_to_odom)
3. Collect synchronized command and odometry data
4. Estimate speed_scale and steer_scale via regression
5. Apply these scales to gym_bridge for simulation calibration
"""

from __future__ import annotations

import argparse
import signal
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
import yaml

try:
    import rosbag2_py  # type: ignore
    from rosidl_runtime_py.utilities import get_message  # type: ignore
    from rclpy.serialization import deserialize_message  # type: ignore
except ImportError:
    rosbag2_py = None  # type: ignore
    get_message = None  # type: ignore
    deserialize_message = None  # type: ignore

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


# ============================================================================
# Data Structures
# ============================================================================

@dataclass
class CommandSample:
    """Single command sample with timestamp."""
    time: float
    speed: float
    steering: float


@dataclass
class OdomSample:
    """Single odometry sample with timestamp."""
    time: float
    x: float
    y: float
    yaw: float


@dataclass
class ScaleResult:
    """Result of scale calibration."""
    speed_scale: float
    steer_scale: float
    speed_samples: int
    steer_samples: int
    speed_rmse: float
    steer_rmse: float
    speed_r2: float
    steer_r2: float


# ============================================================================
# Utility Functions
# ============================================================================

def _require_rosbag() -> None:
    """Check if rosbag2_py is available."""
    if rosbag2_py is None or get_message is None or deserialize_message is None:
        raise RuntimeError(
            'rosbag2_py and ROS 2 Python utilities are required. '
            'Please source your ROS 2 workspace before running this script.'
        )


def _quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Convert quaternion to yaw angle."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return np.arctan2(siny_cosp, cosy_cosp)


def _stamp_to_float(stamp) -> float:
    """Convert ROS timestamp to float seconds."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _unwrap_angles(angles: np.ndarray) -> np.ndarray:
    """Unwrap angle array to avoid discontinuities."""
    unwrapped = np.zeros_like(angles)
    unwrapped[0] = angles[0]

    for i in range(1, len(angles)):
        diff = angles[i] - angles[i-1]
        # Normalize difference to [-pi, pi]
        diff = np.arctan2(np.sin(diff), np.cos(diff))
        unwrapped[i] = unwrapped[i-1] + diff

    return unwrapped


# ============================================================================
# Bag Data Checker
# ============================================================================

def check_bag_has_odom(bag_path: Path, odom_topic: str) -> bool:
    """Check if bag contains odometry data."""
    _require_rosbag()

    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_names = {meta.name for meta in reader.get_all_topics_and_types()}
    return odom_topic in topic_names


def get_initial_pose_from_bag(bag_path: Path) -> Optional[tuple[float, float, float]]:
    """
    Extract initial pose (x, y, yaw) from bag.

    Tries to find pose from:
    1. /odom topic (if exists)
    2. /amcl_pose topic (if exists)
    3. Returns None if no pose found

    Returns:
        (x, y, yaw) or None
    """
    _require_rosbag()

    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_map = {meta.name: meta.type for meta in reader.get_all_topics_and_types()}

    # Try to find pose topics
    pose_topics = ['/wheel/odom', '/odom', '/amcl_pose', '/pf/pose/odom']
    target_topic = None
    for topic in pose_topics:
        if topic in topic_map:
            target_topic = topic
            break

    if target_topic is None:
        return None

    msg_type = get_message(topic_map[target_topic])

    # Read first message from pose topic
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic == target_topic:
            msg = deserialize_message(data, msg_type)

            # Extract pose
            if hasattr(msg, 'pose'):
                if hasattr(msg.pose, 'pose'):
                    pose = msg.pose.pose  # Odometry or PoseWithCovarianceStamped
                else:
                    pose = msg.pose  # PoseStamped
            else:
                continue

            x = float(pose.position.x)
            y = float(pose.position.y)
            yaw = _quaternion_to_yaw(
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w)
            )

            return (x, y, yaw)

    return None


# ============================================================================
# Live Data Collector (for bag replay with localization)
# ============================================================================

class InitialPosePublisher(Node):
    """Publish initial pose to AMCL for localization initialization."""

    def __init__(self, x: float, y: float, yaw: float) -> None:
        super().__init__('initial_pose_publisher')

        # Use sim time for bag replay
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        # QoS profile for /initialpose (needs to be transient local for AMCL)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        self._pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            qos
        )

        # Store pose
        self._x = x
        self._y = y
        self._yaw = yaw

        # Timer to publish periodically (helps with AMCL initialization)
        # Publish at 2Hz throughout the entire bag playback
        self._timer = self.create_timer(0.5, self._publish_pose)

        self.get_logger().info(f'Publishing initial pose: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')

    def _publish_pose(self) -> None:
        """Publish initial pose continuously."""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = np.sin(self._yaw / 2.0)
        msg.pose.pose.orientation.w = np.cos(self._yaw / 2.0)

        # Set covariance (diagonal matrix) - larger variance to allow AMCL to adjust
        msg.pose.covariance = [
            0.5, 0.0, 0.0, 0.0, 0.0, 0.0,  # x variance (0.5m)
            0.0, 0.5, 0.0, 0.0, 0.0, 0.0,  # y variance (0.5m)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # z variance
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # roll variance
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # pitch variance
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1,  # yaw variance (~18 degrees)
        ]

        self._pub.publish(msg)


class LiveDataCollector(Node):
    """Collect command and odometry data during bag replay with localization."""

    def __init__(self, cmd_topic: str, odom_topic: str) -> None:
        super().__init__('scale_calibration_collector')

        # Use sim time for bag replay
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self._lock = threading.Lock()
        self._cmd_samples: list[CommandSample] = []
        self._odom_samples: list[OdomSample] = []

        # Subscribe to command and odometry topics
        self.create_subscription(
            AckermannDriveStamped,
            cmd_topic,
            self._on_cmd,
            100
        )

        self.create_subscription(
            Odometry,
            odom_topic,
            self._on_odom,
            100
        )

        self.get_logger().info(f'Collecting data from {cmd_topic} and {odom_topic}')

    def _on_cmd(self, msg: AckermannDriveStamped) -> None:
        """Callback for command messages."""
        with self._lock:
            sample = CommandSample(
                time=_stamp_to_float(msg.header.stamp),
                speed=float(msg.drive.speed),
                steering=float(msg.drive.steering_angle)
            )
            self._cmd_samples.append(sample)

    def _on_odom(self, msg: Odometry) -> None:
        """Callback for odometry messages."""
        with self._lock:
            pose = msg.pose.pose
            sample = OdomSample(
                time=_stamp_to_float(msg.header.stamp),
                x=float(pose.position.x),
                y=float(pose.position.y),
                yaw=_quaternion_to_yaw(
                    float(pose.orientation.x),
                    float(pose.orientation.y),
                    float(pose.orientation.z),
                    float(pose.orientation.w)
                )
            )
            self._odom_samples.append(sample)

    def get_data(self) -> tuple[list[CommandSample], list[OdomSample]]:
        """Get collected data (thread-safe)."""
        with self._lock:
            return self._cmd_samples.copy(), self._odom_samples.copy()


# ============================================================================
# Process Management
# ============================================================================

class ProcessManager:
    """Manage lifecycle of multiple subprocesses."""

    def __init__(self) -> None:
        self._processes: list[subprocess.Popen] = []

    def spawn(self, cmd: list[str], name: str = "") -> subprocess.Popen:
        """Spawn a subprocess and track it."""
        proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self._processes.append(proc)
        if name:
            print(f"Started {name} (PID: {proc.pid})")
        return proc

    def terminate_all(self, timeout: float = 10.0) -> None:
        """Terminate all tracked processes."""
        for proc in self._processes:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)

        # Wait for graceful shutdown
        deadline = time.time() + timeout
        for proc in self._processes:
            remaining = max(0, deadline - time.time())
            try:
                proc.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()

        self._processes.clear()


# ============================================================================
# Localization Pipeline
# ============================================================================

def run_localization_pipeline(
    bag_path: Path,
    map_yaml: Path,
    localization_launch: Path,
    cmd_topic: str,
    odom_topic: str,
    startup_wait: float,
    post_wait: float
) -> tuple[list[CommandSample], list[OdomSample]]:
    """
    Run complete localization pipeline:
    1. Start map_server with the map
    2. Start localization (AMCL/EKF)
    3. Start tf_to_odom converter
    4. Play bag
    5. Collect synchronized command and odom data
    """

    if not map_yaml.exists():
        raise RuntimeError(f'Map YAML not found: {map_yaml}')
    if not localization_launch.exists():
        raise RuntimeError(f'Localization launch file not found: {localization_launch}')

    print(f"\n{'='*60}")
    print("Starting Localization Pipeline")
    print(f"{'='*60}")
    print(f"Bag: {bag_path}")
    print(f"Map: {map_yaml}")
    print(f"Localization: {localization_launch}")

    # Try to get initial pose from bag
    print("\nExtracting initial pose from bag...")
    initial_pose = get_initial_pose_from_bag(bag_path)
    if initial_pose is not None:
        x, y, yaw = initial_pose
        print(f"Found initial pose: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad ({np.rad2deg(yaw):.1f}°)")
    else:
        print("Warning: Could not extract initial pose from bag. AMCL may not initialize properly.")
        print("Using default pose: x=0, y=0, yaw=0")
        initial_pose = (0.0, 0.0, 0.0)

    # Initialize ROS
    rclpy.init()
    collector = LiveDataCollector(cmd_topic, odom_topic)
    executor = MultiThreadedExecutor()
    executor.add_node(collector)

    # Add initial pose publisher if we have a pose
    pose_publisher = None
    if initial_pose is not None:
        x, y, yaw = initial_pose
        pose_publisher = InitialPosePublisher(x, y, yaw)
        executor.add_node(pose_publisher)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    proc_mgr = ProcessManager()

    try:
        # 1. Start map_server
        map_server_cmd = [
            'ros2', 'launch',
            'f1tenth', 'map_server_launch.py',
            f'map:={str(map_yaml)}',
            'use_sim_time:=true'
        ]
        proc_mgr.spawn(map_server_cmd, "map_server")

        # 2. Start lifecycle manager for map_server
        lifecycle_cmd = [
            'ros2', 'run',
            'nav2_lifecycle_manager', 'lifecycle_manager',
            '--ros-args',
            '-p', 'use_sim_time:=true',
            '-p', 'autostart:=true',
            '-p', 'bond_timeout:=0.0',
            '-p', 'node_names:=[map_server]'
        ]
        proc_mgr.spawn(lifecycle_cmd, "lifecycle_manager")

        # 3. Start localization (AMCL/EKF + tf_to_odom)
        # Note: amcl_launch.py already includes tf_to_odom node
        localization_cmd = [
            'ros2', 'launch',
            str(localization_launch),
            'use_sim_time:=true'
        ]
        proc_mgr.spawn(localization_cmd, "localization")
        print("  (localization launch includes AMCL, EKF, and tf_to_odom)")

        # Wait for all nodes to initialize
        print(f"\nWaiting {startup_wait}s for localization to initialize...")
        time.sleep(startup_wait)

        # 5. Play bag
        print("\nPlaying bag...")
        print("Note: AMCL needs /scan messages to localize. Make sure your bag contains LiDAR data.")
        print(f"Listening for odometry on: {odom_topic}")
        bag_cmd = [
            'ros2', 'bag', 'play',
            str(bag_path),
            '--clock',  # Publish /clock for sim time
            '--rate', '1.0'
        ]
        bag_proc = proc_mgr.spawn(bag_cmd, "bag_player")

        # Wait for bag to finish
        bag_proc.wait()
        print(f"\nBag finished, waiting {post_wait}s for trailing messages...")
        time.sleep(post_wait)

        # Check intermediate collection status
        cmd_samples_temp, odom_samples_temp = collector.get_data()
        print(f"Intermediate check: {len(cmd_samples_temp)} commands, {len(odom_samples_temp)} odom samples")

    finally:
        print("\nShutting down pipeline...")
        proc_mgr.terminate_all()

        executor.shutdown()
        collector.destroy_node()
        if pose_publisher is not None:
            pose_publisher.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)

    # Get collected data
    cmd_samples, odom_samples = collector.get_data()

    print(f"\n{'='*60}")
    print(f"Collected {len(cmd_samples)} command samples")
    print(f"Collected {len(odom_samples)} odometry samples")
    print(f"{'='*60}\n")

    if not cmd_samples:
        raise RuntimeError('No command messages collected during bag replay')
    if len(odom_samples) < 10:
        raise RuntimeError(f'Insufficient odometry samples: {len(odom_samples)} (need at least 10)')

    return cmd_samples, odom_samples


# ============================================================================
# Scale Estimation
# ============================================================================

def _compute_velocity_from_odom(odom_samples: list[OdomSample]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Compute linear and angular velocities from odometry.

    Returns:
        times: Mid-point timestamps for each velocity sample
        linear_speeds: Linear speed (m/s)
        angular_speeds: Angular speed (rad/s)
    """
    if len(odom_samples) < 2:
        raise RuntimeError('Need at least 2 odometry samples to compute velocity')

    # Sort by time
    odom_samples = sorted(odom_samples, key=lambda s: s.time)

    times = []
    linear_speeds = []
    angular_speeds = []

    for i in range(len(odom_samples) - 1):
        s1, s2 = odom_samples[i], odom_samples[i + 1]
        dt = s2.time - s1.time

        if dt <= 0:
            continue

        # Linear velocity
        dx = s2.x - s1.x
        dy = s2.y - s1.y
        linear_speed = np.sqrt(dx**2 + dy**2) / dt

        # Angular velocity (handle wraparound)
        dyaw = s2.yaw - s1.yaw
        dyaw = np.arctan2(np.sin(dyaw), np.cos(dyaw))  # Normalize to [-pi, pi]
        angular_speed = dyaw / dt

        # Mid-point time
        mid_time = s1.time + dt / 2.0

        times.append(mid_time)
        linear_speeds.append(linear_speed)
        angular_speeds.append(angular_speed)

    return (
        np.array(times),
        np.array(linear_speeds),
        np.array(angular_speeds)
    )


def _interpolate_commands(
    cmd_samples: list[CommandSample],
    target_times: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """
    Interpolate command values at target times.

    Returns:
        speeds: Interpolated speed commands
        steerings: Interpolated steering commands
    """
    if not cmd_samples:
        return np.full_like(target_times, np.nan), np.full_like(target_times, np.nan)

    # Sort by time
    cmd_samples = sorted(cmd_samples, key=lambda s: s.time)

    cmd_times = np.array([s.time for s in cmd_samples])
    cmd_speeds = np.array([s.speed for s in cmd_samples])
    cmd_steerings = np.array([s.steering for s in cmd_samples])

    # Interpolate
    interp_speeds = np.interp(target_times, cmd_times, cmd_speeds, left=np.nan, right=np.nan)
    interp_steerings = np.interp(target_times, cmd_times, cmd_steerings, left=np.nan, right=np.nan)

    return interp_speeds, interp_steerings


def estimate_scales(
    cmd_samples: list[CommandSample],
    odom_samples: list[OdomSample],
    wheelbase: float,
    min_speed: float,
    min_steer: float,
    min_samples: int
) -> ScaleResult:
    """
    Estimate speed and steering scale factors via regression.

    The regression model is:
        actual_speed = speed_scale * commanded_speed
        actual_steering = steer_scale * commanded_steering

    For steering, we use the bicycle model:
        angular_velocity = (linear_speed / wheelbase) * tan(steering_angle)

    For small angles: tan(θ) ≈ θ, so:
        angular_velocity ≈ (linear_speed / wheelbase) * steering_angle
        => steering_angle ≈ (angular_velocity * wheelbase) / linear_speed
    """

    print(f"\n{'='*60}")
    print("Starting Scale Estimation")
    print(f"{'='*60}")

    # 1. Compute velocities from odometry
    odom_times, linear_speeds, angular_speeds = _compute_velocity_from_odom(odom_samples)
    print(f"Computed velocities from {len(odom_samples)} odometry samples")
    print(f"  -> {len(odom_times)} velocity samples")

    # 2. Interpolate commands at velocity timestamps
    cmd_speeds, cmd_steerings = _interpolate_commands(cmd_samples, odom_times)
    print(f"Interpolated commands at velocity timestamps")

    # 3. Estimate speed scale
    # Filter valid samples
    valid_speed = (
        np.isfinite(cmd_speeds) &
        np.isfinite(linear_speeds) &
        (np.abs(cmd_speeds) >= min_speed) &
        (np.abs(linear_speeds) >= min_speed)
    )

    n_speed_samples = np.sum(valid_speed)
    print(f"\nSpeed regression:")
    print(f"  Valid samples: {n_speed_samples} (min required: {min_samples})")

    if n_speed_samples < min_samples:
        raise RuntimeError(
            f'Insufficient speed samples: {n_speed_samples} (need at least {min_samples}). '
            f'Try reducing --min-speed or recording more data.'
        )

    # Linear regression: actual = scale * commanded
    cmd_speed_valid = cmd_speeds[valid_speed]
    actual_speed_valid = linear_speeds[valid_speed]

    # Least squares: scale = (cmd · actual) / (cmd · cmd)
    speed_scale = np.dot(cmd_speed_valid, actual_speed_valid) / np.dot(cmd_speed_valid, cmd_speed_valid)

    # Compute RMSE and R²
    speed_pred = speed_scale * cmd_speed_valid
    speed_residuals = actual_speed_valid - speed_pred
    speed_rmse = np.sqrt(np.mean(speed_residuals**2))

    ss_res = np.sum(speed_residuals**2)
    ss_tot = np.sum((actual_speed_valid - np.mean(actual_speed_valid))**2)
    speed_r2 = 1 - (ss_res / ss_tot) if ss_tot > 0 else 0.0

    print(f"  Speed scale: {speed_scale:.5f}")
    print(f"  RMSE: {speed_rmse:.4f} m/s")
    print(f"  R²: {speed_r2:.4f}")

    # 4. Estimate steering scale
    # From bicycle model: steering = (angular * wheelbase) / linear
    with np.errstate(divide='ignore', invalid='ignore'):
        actual_steering = (angular_speeds * wheelbase) / linear_speeds

    # Filter valid samples
    valid_steer = (
        np.isfinite(cmd_steerings) &
        np.isfinite(actual_steering) &
        np.isfinite(linear_speeds) &
        (np.abs(cmd_steerings) >= min_steer) &
        (np.abs(linear_speeds) >= min_speed) &  # Need sufficient speed for steering estimation
        (np.abs(angular_speeds) > 1e-3)  # Need some turning
    )

    n_steer_samples = np.sum(valid_steer)
    print(f"\nSteering regression:")
    print(f"  Valid samples: {n_steer_samples} (min required: {min_samples})")

    if n_steer_samples < min_samples:
        raise RuntimeError(
            f'Insufficient steering samples: {n_steer_samples} (need at least {min_samples}). '
            f'Try reducing --min-steer, recording more turning maneuvers, or reducing --min-samples.'
        )

    cmd_steer_valid = cmd_steerings[valid_steer]
    actual_steer_valid = actual_steering[valid_steer]

    # Least squares
    steer_scale = np.dot(cmd_steer_valid, actual_steer_valid) / np.dot(cmd_steer_valid, cmd_steer_valid)

    # Compute RMSE and R²
    steer_pred = steer_scale * cmd_steer_valid
    steer_residuals = actual_steer_valid - steer_pred
    steer_rmse = np.sqrt(np.mean(steer_residuals**2))

    ss_res = np.sum(steer_residuals**2)
    ss_tot = np.sum((actual_steer_valid - np.mean(actual_steer_valid))**2)
    steer_r2 = 1 - (ss_res / ss_tot) if ss_tot > 0 else 0.0

    print(f"  Steer scale: {steer_scale:.5f}")
    print(f"  RMSE: {steer_rmse:.4f} rad")
    print(f"  R²: {steer_r2:.4f}")

    return ScaleResult(
        speed_scale=speed_scale,
        steer_scale=steer_scale,
        speed_samples=n_speed_samples,
        steer_samples=n_steer_samples,
        speed_rmse=speed_rmse,
        steer_rmse=steer_rmse,
        speed_r2=speed_r2,
        steer_r2=steer_r2
    )


# ============================================================================
# Configuration Helpers
# ============================================================================

def _find_default_localization_launch() -> Optional[Path]:
    """Find default localization launch file."""
    try:
        share_dir = Path(get_package_share_directory('localization'))
        candidate = share_dir / 'launch' / 'amcl_launch.py'
        return candidate if candidate.exists() else None
    except PackageNotFoundError:
        return None


def _find_default_map_yaml() -> Optional[Path]:
    """Find default map YAML from simulator config."""
    try:
        sim_share = Path(get_package_share_directory('simulator'))
        f1tenth_share = Path(get_package_share_directory('f1tenth'))

        sim_config = sim_share / 'config' / 'sim.yaml'
        if not sim_config.exists():
            return None

        with sim_config.open() as f:
            config = yaml.safe_load(f)

        map_path = config.get('bridge', {}).get('ros__parameters', {}).get('map_path')
        if not map_path:
            return None

        map_yaml = f1tenth_share / 'maps' / f'{map_path}.yaml'
        return map_yaml if map_yaml.exists() else None

    except (PackageNotFoundError, yaml.YAMLError, KeyError):
        return None


# ============================================================================
# Main
# ============================================================================

def load_data_from_bag(
    bag_path: Path,
    cmd_topic: str,
    odom_topic: str
) -> tuple[list[CommandSample], list[OdomSample]]:
    """
    Load command and odometry data directly from bag file.

    This is simpler and more reliable than running localization.
    """
    _require_rosbag()

    print(f"\n{'='*60}")
    print("Loading data from bag")
    print(f"{'='*60}")
    print(f"Bag: {bag_path}")
    print(f"Command topic: {cmd_topic}")
    print(f"Odometry topic: {odom_topic}")

    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_map = {meta.name: meta.type for meta in reader.get_all_topics_and_types()}

    if cmd_topic not in topic_map:
        raise RuntimeError(f'Command topic "{cmd_topic}" not found in bag')
    if odom_topic not in topic_map:
        raise RuntimeError(f'Odometry topic "{odom_topic}" not found in bag')

    cmd_msg_type = get_message(topic_map[cmd_topic])
    odom_msg_type = get_message(topic_map[odom_topic])

    cmd_samples: list[CommandSample] = []
    odom_samples: list[OdomSample] = []

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        stamp_sec = timestamp * 1e-9

        if topic == cmd_topic:
            msg = deserialize_message(data, cmd_msg_type)
            sample = CommandSample(
                time=stamp_sec,
                speed=float(msg.drive.speed),
                steering=float(msg.drive.steering_angle)
            )
            cmd_samples.append(sample)

        elif topic == odom_topic:
            msg = deserialize_message(data, odom_msg_type)
            pose = msg.pose.pose
            sample = OdomSample(
                time=stamp_sec,
                x=float(pose.position.x),
                y=float(pose.position.y),
                y=float(pose.position.y),
                yaw=_quaternion_to_yaw(
                    float(pose.orientation.x),
                    float(pose.orientation.y),
                    float(pose.orientation.z),
                    float(pose.orientation.w)
                )
            )
            odom_samples.append(sample)

    print(f"\nLoaded {len(cmd_samples)} command samples")
    print(f"Loaded {len(odom_samples)} odometry samples")
    print(f"{'='*60}\n")

    if not cmd_samples:
        raise RuntimeError(f'No command messages found on topic "{cmd_topic}"')
    if len(odom_samples) < 10:
        raise RuntimeError(f'Insufficient odometry samples: {len(odom_samples)} (need at least 10)')

    return cmd_samples, odom_samples


def main(argv: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(
        description='Estimate Ackermann drive scale factors from real vehicle data.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example usage:
  # Basic usage (uses /wheel/odom from bag):
  python3 scale_calibrator.py my_recording

  # Specify custom topics:
  python3 scale_calibrator.py my_recording --cmd-topic /teleop --odom-topic /wheel/odom

  # Run with localization (if bag doesn't have odometry):
  python3 scale_calibrator.py my_recording --use-localization \\
      --localization-launch ~/f1_ws/install/localization/share/localization/launch/amcl_launch.py \\
      --map ~/f1_ws/install/f1tenth/share/f1tenth/maps/track.yaml

The script will output speed_scale and steer_scale values to apply in gym_bridge.
"""
    )

    # Required arguments
    parser.add_argument(
        'bag',
        type=Path,
        help='Path to rosbag2 directory with recorded vehicle data'
    )

    # Topic configuration
    parser.add_argument(
        '--cmd-topic',
        default='/teleop',
        help='AckermannDriveStamped command topic (default: /teleop)'
    )
    parser.add_argument(
        '--odom-topic',
        default='/wheel/odom',
        help='Odometry topic (default: /wheel/odom from bag)'
    )

    # Vehicle parameters
    parser.add_argument(
        '--wheelbase',
        type=float,
        default=0.33,
        help='Vehicle wheelbase in meters (default: 0.33)'
    )

    # Filtering parameters
    parser.add_argument(
        '--min-speed',
        type=float,
        default=0.1,
        help='Minimum speed magnitude to include in regression (m/s, default: 0.1)'
    )
    parser.add_argument(
        '--min-steer',
        type=float,
        default=0.02,
        help='Minimum steering magnitude to include in regression (rad, default: 0.02)'
    )
    parser.add_argument(
        '--min-samples',
        type=int,
        default=50,
        help='Minimum number of samples required for each regression (default: 50)'
    )

    # Localization pipeline (optional, advanced users)
    parser.add_argument(
        '--use-localization',
        action='store_true',
        help='Run localization pipeline (only needed if bag lacks odometry)'
    )
    parser.add_argument(
        '--localization-launch',
        type=Path,
        help='Localization launch file (required if --use-localization)'
    )
    parser.add_argument(
        '--map',
        type=Path,
        help='Map YAML file (required if --use-localization)'
    )
    parser.add_argument(
        '--startup-wait',
        type=float,
        default=8.0,
        help='Seconds to wait after starting localization before playing bag (default: 8.0)'
    )
    parser.add_argument(
        '--post-wait',
        type=float,
        default=3.0,
        help='Seconds to wait after bag finishes to collect trailing messages (default: 3.0)'
    )

    args = parser.parse_args(argv)

    # Validate inputs
    args.bag = args.bag.expanduser().resolve()
    if not args.bag.exists():
        parser.error(f'Bag directory not found: {args.bag}')

    # Check if we need localization
    if args.use_localization:
        # Validate localization arguments
        if not args.localization_launch:
            parser.error('--localization-launch is required when using --use-localization')
        if not args.map:
            parser.error('--map is required when using --use-localization')

        args.localization_launch = args.localization_launch.expanduser().resolve()
        if not args.localization_launch.exists():
            parser.error(f'Localization launch file not found: {args.localization_launch}')

        args.map = args.map.expanduser().resolve()
        if not args.map.exists():
            parser.error(f'Map YAML not found: {args.map}')
    else:
        # Verify bag has required topics
        if not check_bag_has_odom(args.bag, args.odom_topic):
            parser.error(
                f'Bag does not contain {args.odom_topic}. '
                f'Either record odometry data or use --use-localization.'
            )

    # Run pipeline
    try:
        if args.use_localization:
            # Run full localization pipeline
            cmd_samples, odom_samples = run_localization_pipeline(
                bag_path=args.bag,
                map_yaml=args.map,
                localization_launch=args.localization_launch,
                cmd_topic=args.cmd_topic,
                odom_topic=args.odom_topic,
                startup_wait=args.startup_wait,
                post_wait=args.post_wait
            )
        else:
            # Load data directly from bag (simple and fast!)
            cmd_samples, odom_samples = load_data_from_bag(
                bag_path=args.bag,
                cmd_topic=args.cmd_topic,
                odom_topic=args.odom_topic
            )

        # Estimate scales
        result = estimate_scales(
            cmd_samples=cmd_samples,
            odom_samples=odom_samples,
            wheelbase=args.wheelbase,
            min_speed=args.min_speed,
            min_steer=args.min_steer,
            min_samples=args.min_samples
        )

    except Exception as e:
        parser.exit(status=1, message=f'\nError: {e}\n')

    # Print results
    print(f"\n{'='*60}")
    print("CALIBRATION RESULTS")
    print(f"{'='*60}")
    print(f"Bag: {args.bag}")
    if args.use_localization:
        print(f"Map: {args.map}")
        print(f"Localization: {args.localization_launch}")
    print(f"Command topic: {args.cmd_topic}")
    print(f"Odom topic: {args.odom_topic}")
    print(f"Wheelbase: {args.wheelbase} m")
    print()
    print(f"Speed Scale: {result.speed_scale:.6f}")
    print(f"  Samples: {result.speed_samples}")
    print(f"  RMSE: {result.speed_rmse:.4f} m/s")
    print(f"  R²: {result.speed_r2:.4f}")
    print()
    print(f"Steer Scale: {result.steer_scale:.6f}")
    print(f"  Samples: {result.steer_samples}")
    print(f"  RMSE: {result.steer_rmse:.4f} rad ({np.rad2deg(result.steer_rmse):.2f}°)")
    print(f"  R²: {result.steer_r2:.4f}")
    print(f"{'='*60}")
    print("\nApply these scales to your gym_bridge configuration:")
    print(f"  speed_scale: {result.speed_scale:.6f}")
    print(f"  steer_scale: {result.steer_scale:.6f}")
    print(f"{'='*60}\n")


if __name__ == '__main__':
    main()
