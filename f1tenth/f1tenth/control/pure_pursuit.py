#!/usr/bin/env python3
"""
Simple Pure Pursuit Controller for F1TENTH
- Monotonic lookahead index (never goes backward)
- Speed-adaptive lookahead distance (long in straights, short in turns)
- Speed range: 1.5 ~ 9.0 m/s
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry, Path
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped


class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')

        # Parameters
        self.declare_parameter('wheelbase', 0.3302)
        self.declare_parameter('max_steering_angle', 0.4189)
        self.declare_parameter('path_topic', '/local_path')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('drive_topic', '/drive')

        # Speed parameters
        self.declare_parameter('v_min', 1.5)
        self.declare_parameter('v_max', 9.0)
        self.declare_parameter('max_lateral_accel', 4.0)  # m/s²

        # Lookahead parameters
        self.declare_parameter('ld_min', 1.0)  # Minimum lookahead (tight turns)
        self.declare_parameter('ld_max', 4.0)  # Maximum lookahead (straights)
        self.declare_parameter('ld_speed_gain', 0.4)  # Lookahead = ld_min + speed * gain

        # Control parameters
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('steer_smooth_alpha', 0.3)

        # Get parameters
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.v_min = self.get_parameter('v_min').value
        self.v_max = self.get_parameter('v_max').value
        self.max_lateral_accel = self.get_parameter('max_lateral_accel').value
        self.ld_min = self.get_parameter('ld_min').value
        self.ld_max = self.get_parameter('ld_max').value
        self.ld_speed_gain = self.get_parameter('ld_speed_gain').value
        self.control_rate_hz = self.get_parameter('control_rate_hz').value
        self.steer_smooth_alpha = self.get_parameter('steer_smooth_alpha').value

        # State
        self.current_pose = None
        self.current_path = None
        self.current_speed = self.v_min
        self.previous_steering = 0.0
        self.last_lookahead_index = 0  # Monotonic index tracking

        # Speed ramping parameters
        self.max_accel = 2.0  # m/s² - maximum acceleration (slower for corner exits)
        self.max_decel = 6.0  # m/s² - maximum deceleration (brake harder for corners)

        # QoS
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_callback,
            qos
        )

        self.path_sub = self.create_subscription(
            Path,
            self.get_parameter('path_topic').value,
            self.path_callback,
            10
        )

        # Publishers
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            self.get_parameter('drive_topic').value,
            10
        )

        self.lookahead_pub = self.create_publisher(
            PointStamped,
            '/lookahead_point',
            10
        )

        # Control timer
        control_period = 1.0 / self.control_rate_hz
        self.control_timer = self.create_timer(control_period, self.control_loop)

        self.get_logger().info('Pure Pursuit Controller initialized')
        self.get_logger().info(f'Speed: {self.v_min} ~ {self.v_max} m/s')
        self.get_logger().info(f'Lookahead: {self.ld_min} ~ {self.ld_max} m')
        self.get_logger().info(f'Lookahead gain: {self.ld_speed_gain}')

    def odom_callback(self, msg):
        """Update current vehicle pose"""
        self.current_pose = msg.pose.pose

    def path_callback(self, msg):
        """Update current path and reset lookahead index if path changes"""
        if len(msg.poses) > 0:
            # Reset lookahead index when receiving a new path
            if self.current_path is None or len(self.current_path.poses) != len(msg.poses):
                self.last_lookahead_index = 0
            self.current_path = msg

    def calculate_curvature_at_index(self, path, index):
        """Calculate path curvature at given index using 3 points"""
        if len(path.poses) < 3 or index < 1 or index >= len(path.poses) - 1:
            return 0.0

        p1 = path.poses[index - 1].pose.position
        p2 = path.poses[index].pose.position
        p3 = path.poses[index + 1].pose.position

        # Menger curvature
        area = 0.5 * abs(p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y))

        d12 = math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)
        d23 = math.sqrt((p2.x - p3.x)**2 + (p2.y - p3.y)**2)
        d31 = math.sqrt((p3.x - p1.x)**2 + (p3.y - p1.y)**2)

        denominator = d12 * d23 * d31
        if denominator < 1e-6:
            return 0.0

        return 4.0 * area / denominator

    def calculate_lookahead_curvature(self, path, start_idx, lookahead_distance):
        """Calculate average curvature over lookahead distance"""
        if not path or len(path.poses) < 3 or start_idx >= len(path.poses) - 1:
            return 0.0

        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        curvatures = []

        # Collect curvatures within lookahead distance
        for i in range(start_idx, min(len(path.poses), start_idx + 50)):
            px = path.poses[i].pose.position.x
            py = path.poses[i].pose.position.y
            dist = math.sqrt((px - current_x)**2 + (py - current_y)**2)

            if dist > lookahead_distance:
                break

            # Calculate curvature at this point
            curv = self.calculate_curvature_at_index(path, i)
            if curv > 1e-6:  # Only consider significant curvature
                curvatures.append(curv)

        # Return average curvature, or 0 if straight
        if len(curvatures) > 0:
            return sum(curvatures) / len(curvatures)
        return 0.0

    def calculate_speed_from_curvature(self, curvature):
        """
        Calculate safe speed based on lateral acceleration limit
        Formula from F1TENTH benchmark: v = sqrt(a_lat_max / curvature)
        """
        if abs(curvature) < 1e-4:
            # Straight: max speed
            return self.v_max

        # F1TENTH standard: v = sqrt(a_lat / curvature)
        # a_lat = 1.5 * g = 14.715 m/s^2 (from benchmark paper)
        v_safe = math.sqrt(self.max_lateral_accel / abs(curvature))
        return max(self.v_min, min(self.v_max, v_safe))

    def calculate_lookahead_distance(self, speed):
        """Calculate speed-adaptive lookahead distance"""
        # Lookahead increases with speed
        ld = self.ld_min + speed * self.ld_speed_gain
        return max(self.ld_min, min(self.ld_max, ld))

    def find_closest_point(self, path):
        """Find closest point on path (forward-looking only)"""
        if not path or not self.current_pose:
            return None

        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        # Get vehicle heading
        q = self.current_pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        min_dist = float('inf')
        closest_idx = 0

        for i, pose_stamped in enumerate(path.poses):
            dx = pose_stamped.pose.position.x - current_x
            dy = pose_stamped.pose.position.y - current_y
            dist = math.sqrt(dx * dx + dy * dy)

            # Check if point is in front
            to_point_angle = math.atan2(dy, dx)
            angle_diff = abs(to_point_angle - yaw)
            angle_diff = min(angle_diff, 2 * math.pi - angle_diff)

            if angle_diff < math.pi / 2 and dist < min_dist:
                min_dist = dist
                closest_idx = i

        return closest_idx

    def find_lookahead_point(self, path, lookahead_distance):
        """
        Find lookahead point with MONOTONIC index constraint.
        Lookahead index never goes backward from last iteration.
        """
        if not path or not self.current_pose:
            return None, None

        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        # Find closest point
        closest_idx = self.find_closest_point(path)
        if closest_idx is None:
            return None, None

        # Ensure lookahead search starts from max(closest, last_lookahead)
        # This prevents index from going backward
        start_idx = max(closest_idx, self.last_lookahead_index)

        # Search forward from start_idx for lookahead distance
        for i in range(start_idx, len(path.poses)):
            px = path.poses[i].pose.position.x
            py = path.poses[i].pose.position.y
            dist = math.sqrt((px - current_x)**2 + (py - current_y)**2)

            if dist >= lookahead_distance:
                # Update monotonic index
                self.last_lookahead_index = i
                return (px, py), i

        # If no point at lookahead distance, use last point
        if len(path.poses) > 0:
            last_idx = len(path.poses) - 1
            last_pose = path.poses[last_idx].pose
            self.last_lookahead_index = last_idx
            return (last_pose.position.x, last_pose.position.y), last_idx

        return None, None

    def pure_pursuit_steering(self, target_point):
        """Calculate steering angle using pure pursuit"""
        if not target_point or not self.current_pose:
            return 0.0

        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        # Get vehicle heading
        q = self.current_pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        # Transform target to vehicle frame
        dx = target_point[0] - current_x
        dy = target_point[1] - current_y

        target_x_vehicle = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        target_y_vehicle = math.sin(-yaw) * dx + math.cos(-yaw) * dy

        # Lookahead distance
        ld_actual = math.sqrt(target_x_vehicle**2 + target_y_vehicle**2)

        if ld_actual < 0.1:
            return 0.0

        # Pure pursuit formula
        alpha = math.atan2(target_y_vehicle, target_x_vehicle)
        steering = math.atan2(2 * self.wheelbase * math.sin(alpha), ld_actual)

        # Limit steering
        return max(-self.max_steering_angle, min(self.max_steering_angle, steering))

    def calculate_speed_from_steering(self, steering_angle):
        """
        Calculate safe speed from steering angle
        F1TENTH benchmark formula: v = sqrt(k * g * L / tan(|δ|))
        where L is wheelbase, δ is steering angle, k is safety factor
        """
        if abs(steering_angle) < 0.01:  # Nearly straight
            return self.v_max

        # Modified F1TENTH formula with reduced safety factor for slower turns
        # k = 1.0 (reduced from 1.5) for more conservative turning speeds
        safety_factor = 1.0
        tan_delta = math.tan(abs(steering_angle))
        if tan_delta < 1e-6:
            return self.v_max

        v_safe = math.sqrt(safety_factor * 9.81 * self.wheelbase / tan_delta)
        return max(self.v_min, min(self.v_max, v_safe))

    def control_loop(self):
        """Main control loop at 50Hz"""
        if not self.current_pose or not self.current_path or len(self.current_path.poses) < 2:
            # Stop if no path or pose
            self.publish_drive(0.0, 0.0)
            return

        # 1. Find closest point
        closest_idx = self.find_closest_point(self.current_path)
        if closest_idx is None:
            self.publish_drive(0.0, 0.0)
            return

        # 2. Calculate speed-adaptive lookahead distance (using current speed)
        lookahead_distance = self.calculate_lookahead_distance(self.current_speed)

        # 3. Find lookahead point (with monotonic index)
        target_point, lookahead_idx = self.find_lookahead_point(
            self.current_path,
            lookahead_distance
        )

        if not target_point:
            self.publish_drive(0.0, 0.0)
            return

        # Publish lookahead point for visualization
        lookahead_msg = PointStamped()
        lookahead_msg.header.frame_id = 'map'
        lookahead_msg.header.stamp = self.get_clock().now().to_msg()
        lookahead_msg.point.x = target_point[0]
        lookahead_msg.point.y = target_point[1]
        lookahead_msg.point.z = 0.0
        self.lookahead_pub.publish(lookahead_msg)

        # 4. Calculate steering angle
        raw_steering = self.pure_pursuit_steering(target_point)

        # 5. Apply steering smoothing
        steering = (self.steer_smooth_alpha * raw_steering +
                   (1.0 - self.steer_smooth_alpha) * self.previous_steering)
        self.previous_steering = steering

        # 6. Calculate target speed based on steering angle (F1TENTH method)
        target_speed = self.calculate_speed_from_steering(steering)

        # 7. Apply acceleration/deceleration limits for smooth speed changes
        dt = 1.0 / self.control_rate_hz  # Time step
        speed_diff = target_speed - self.current_speed

        if speed_diff > 0:
            # Accelerating - limit to max_accel
            max_speed_increase = self.max_accel * dt
            actual_speed = self.current_speed + min(speed_diff, max_speed_increase)
        else:
            # Decelerating - limit to max_decel
            max_speed_decrease = self.max_decel * dt
            actual_speed = self.current_speed + max(speed_diff, -max_speed_decrease)

        # 8. Update current speed for next iteration
        self.current_speed = actual_speed

        # 9. Publish drive command
        self.publish_drive(actual_speed, steering)

    def publish_drive(self, speed, steering):
        """Publish Ackermann drive command"""
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Pure Pursuit Controller')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
