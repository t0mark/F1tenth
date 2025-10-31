#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry, Path
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker


class PurePursuitController(Node):
    """
    Pure Pursuit Controller for F1TENTH
    - Subscribes to global/local path and odometry
    - Publishes Ackermann drive commands
    - Uses local path (priority) and global path (fallback) for steering control
    - Adaptive speed control based on path curvature
    - Adaptive lookahead distance based on current speed
    - Steering angle smoothing for stable control
    """

    def __init__(self):
        super().__init__('pure_pursuit_controller')
        
        # Parameters
        self.declare_parameter('wheelbase', 0.3302)
        self.declare_parameter('max_steering_angle', 0.4189)
        self.declare_parameter('path_topic', '/local_path')
        self.declare_parameter('fallback_path_topic', '/global_path')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('v_min', 1.3)
        self.declare_parameter('v_max', 5.0)
        self.declare_parameter('ld_min', 0.8)
        self.declare_parameter('ld_max', 2.1)
        self.declare_parameter('max_curvature', 1.0)
        self.declare_parameter('local_path_timeout', 1.0)
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('steer_smooth_alpha', 0.3)
        self.declare_parameter('curvature_exponent', 1.5)
        self.declare_parameter('ref_velocity', self.get_parameter('v_max').get_parameter_value().double_value)

        # Get parameters
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.v_min = self.get_parameter('v_min').value
        self.v_max = self.get_parameter('v_max').value
        self.ld_min = self.get_parameter('ld_min').value
        self.ld_max = self.get_parameter('ld_max').value
        self.max_curvature = self.get_parameter('max_curvature').value
        self.local_path_timeout = self.get_parameter('local_path_timeout').value
        self.control_rate_hz = self.get_parameter('control_rate_hz').value
        self.steer_smooth_alpha = self.get_parameter('steer_smooth_alpha').value
        self.curvature_exponent = self.get_parameter('curvature_exponent').value
        self.ref_velocity = self.get_parameter('ref_velocity').value

        # State variables
        self.current_pose = None
        self.global_path = None
        self.local_path = None
        self.local_path_timestamp = None
        self.current_speed = self.v_min
        self.previous_steering_angle = 0.0

        # QoS profiles
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
        
        self.fallback_path_sub = self.create_subscription(
            Path,
            self.get_parameter('fallback_path_topic').value,
            self.fallback_path_callback,
            10
        )
        
        # Publisher
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            self.get_parameter('drive_topic').value,
            10
        )
        self.lookahead_point_pub = self.create_publisher(
            PointStamped, '/lookahead_point', 10)
        self.speed_marker_pub = self.create_publisher(
            Marker, '/speed_marker', 10)
        self.steering_marker_pub = self.create_publisher(
            Marker, '/steering_marker', 10)

        # Control timer
        control_period = 1.0 / self.control_rate_hz
        self.control_timer = self.create_timer(control_period, self.control_loop)
        
        self.get_logger().info('Pure Pursuit Controller initialized')
        self.get_logger().info(f'Adaptive speed: min={self.v_min} m/s, max={self.v_max} m/s')
        self.get_logger().info(f'Adaptive lookahead: min={self.ld_min} m, max={self.ld_max} m')
        self.get_logger().info(f'Control rate: {self.control_rate_hz} Hz')
        self.get_logger().info(f'Steering smoothing: alpha={self.steer_smooth_alpha}')

    def odom_callback(self, msg):
        """Update current vehicle pose from global odometry (map frame)"""
        # Directly use pose from global odometry (already in map frame)
        self.current_pose = msg.pose.pose

    def path_callback(self, msg):
        """Local path callback"""
        if len(msg.poses) > 0:
            self.local_path = msg
            self.local_path_timestamp = self.get_clock().now()

    def fallback_path_callback(self, msg):
        """Global path callback"""
        if len(msg.poses) > 0:
            self.global_path = msg

    def get_current_path(self):
        """
        Get current path with priority: local path (if recent) > global path
        """
        current_time = self.get_clock().now()
        
        # Check if local path is recent and available
        if (self.local_path is not None and 
            self.local_path_timestamp is not None and 
            len(self.local_path.poses) > 0):
            
            time_diff = (current_time - self.local_path_timestamp).nanoseconds / 1e9
            if time_diff < self.local_path_timeout:
                return self.local_path
        
        # Use global path as fallback
        if self.global_path is not None and len(self.global_path.poses) > 0:
            return self.global_path
            
        return None

    def calculate_average_curvature(self, path):
        """Calculate the average curvature of the given path."""
        if len(path.poses) < 3:
            return 0.0

        total_curvature = 0.0
        num_points = 0
        for i in range(1, len(path.poses) - 1):
            total_curvature += self.calculate_curvature_at_point(path, i)
            num_points += 1
        
        if num_points == 0:
            return 0.0
            
        return total_curvature / num_points

    def calculate_lookahead_from_curvature_and_speed(self, avg_curvature, current_speed):
        """Calculate adaptive lookahead based on path curvature."""
        # 1. Calculate base lookahead from curvature (inverse non-linear relationship)
        curvature_factor = min(1.0, abs(avg_curvature) / self.max_curvature)
        base_ld = self.ld_max - pow(curvature_factor, self.curvature_exponent) * (self.ld_max - self.ld_min)

        # 2. The lookahead is determined solely by curvature, no longer adjusted by current speed.
        # The 'current_speed' parameter is kept for compatibility but not used for adjustment.

        # 3. Clamp the final lookahead distance
        final_ld = max(self.ld_min, min(self.ld_max, base_ld))
        return final_ld

    def calculate_speed_from_lookahead(self, lookahead_distance):
        """Calculate adaptive speed based on lookahead distance (linear relationship)."""
        ld_ratio = (lookahead_distance - self.ld_min) / (self.ld_max - self.ld_min)
        speed = self.v_min + ld_ratio * (self.v_max - self.v_min)
        return max(self.v_min, min(self.v_max, speed))

    def find_target_point(self, lookahead_distance):
        """
        Find target point on path using lookahead distance
        Returns target point coordinates (target_x, target_y) and its index, or (None, None)
        """
        path = self.get_current_path()
        if not path or not self.current_pose:
            return None, None
            
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        # Get vehicle heading for forward-looking search
        q = self.current_pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        
        # Find closest point on path (forward-looking)
        min_dist = float('inf')
        closest_idx = 0
        
        for i, pose_stamped in enumerate(path.poses):
            dx = pose_stamped.pose.position.x - current_x
            dy = pose_stamped.pose.position.y - current_y
            dist = math.sqrt(dx*dx + dy*dy)
            
            # Check if point is roughly in front of vehicle
            to_point_angle = math.atan2(dy, dx)
            angle_diff = abs(to_point_angle - yaw)
            angle_diff = min(angle_diff, 2*math.pi - angle_diff)  # wrap to [0, pi]
            
            # Prefer points that are in front (within 90 degrees)
            if angle_diff < math.pi/2 and dist < min_dist:
                min_dist = dist
                closest_idx = i

        # Search forward from closest point for lookahead distance
        for i in range(closest_idx, len(path.poses)):
            target_x = path.poses[i].pose.position.x
            target_y = path.poses[i].pose.position.y
            
            dx = target_x - current_x
            dy = target_y - current_y
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist >= lookahead_distance:
                return (target_x, target_y), i
        
        # If no point found at lookahead distance, use last point
        if len(path.poses) > 0:
            self.get_logger().warn('No target point found, using last point')
            last_pose = path.poses[-1].pose
            return (last_pose.position.x, last_pose.position.y), len(path.poses) - 1
            
        return None, None

    def pure_pursuit_control(self, target_point):
        """
        Calculate steering angle using pure pursuit algorithm
        """
        if not target_point or not self.current_pose:
            return 0.0
            
        # Current vehicle state
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        # Get vehicle heading from quaternion
        q = self.current_pose.orientation
        # Convert quaternion to yaw
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        
        # Transform target point to vehicle frame
        target_x, target_y = target_point
        
        # Vector from vehicle to target
        dx = target_x - current_x
        dy = target_y - current_y
        
        # Rotate to vehicle frame
        target_x_vehicle = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        target_y_vehicle = math.sin(-yaw) * dx + math.cos(-yaw) * dy
        
        # Calculate lookahead distance (actual distance to target)
        lookahead_actual = math.sqrt(target_x_vehicle**2 + target_y_vehicle**2)
        
        if lookahead_actual < 0.1:  # Too close, no steering needed
            return 0.0
        
        # Pure pursuit steering calculation
        # steering_angle = atan2(2 * L * sin(alpha), lookahead_distance)
        # where alpha is the angle between vehicle heading and target direction
        alpha = math.atan2(target_y_vehicle, target_x_vehicle)
        
        steering_angle = math.atan2(2 * self.wheelbase * math.sin(alpha), lookahead_actual)
        
        # Limit steering angle
        steering_angle = max(-self.max_steering_angle, 
                           min(self.max_steering_angle, steering_angle))
        
        return steering_angle

    def calculate_curvature_at_point(self, path, index):
        """Calculate curvature at a specific point on the path using Menger curvature."""
        if len(path.poses) < 3:
            return 0.0

        # Use 3 consecutive points to calculate curvature
        p1_idx = max(0, index - 1)
        p2_idx = index
        p3_idx = min(len(path.poses) - 1, index + 1)

        p1 = path.poses[p1_idx].pose.position
        p2 = path.poses[p2_idx].pose.position
        p3 = path.poses[p3_idx].pose.position

        # Triangle area using cross product
        area = 0.5 * abs(p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y))

        # Side lengths
        d12 = math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)
        d23 = math.sqrt((p2.x - p3.x)**2 + (p2.y - p3.y)**2)
        d31 = math.sqrt((p3.x - p1.x)**2 + (p3.y - p1.y)**2)

        # Menger curvature formula: K = 4 * Area / (d1*d2*d3)
        denominator = d12 * d23 * d31
        if denominator < 1e-6:
            return 0.0
        
        return 4.0 * area / denominator

    def calculate_speed_from_curvature(self, curvature):
        """Calculate adaptive speed based on path curvature."""
        # Normalize curvature
        curvature_factor = min(1.0, abs(curvature) / self.max_curvature)

        # Higher curvature -> lower speed
        speed = self.v_max - curvature_factor * (self.v_max - self.v_min)
        
        return max(self.v_min, min(self.v_max, speed))

    def control_loop(self):
        """Main control loop - runs at 50Hz"""
        current_path = self.get_current_path()
        if not self.current_pose or not current_path:
            # Stop if no path or pose
            self.publish_drive_command(0.0, 0.0)
            return

        # 1. Calculate average curvature of the path
        avg_curvature = self.calculate_average_curvature(current_path)

        # 2. Calculate adaptive lookahead based on curvature and current speed
        adaptive_lookahead = self.calculate_lookahead_from_curvature_and_speed(
            avg_curvature, self.current_speed
        )

        # 3. Calculate adaptive speed from the new lookahead distance
        adaptive_speed = self.calculate_speed_from_lookahead(adaptive_lookahead)

        # 4. Find target point using this lookahead
        target_point, _ = self.find_target_point(adaptive_lookahead)
        if not target_point:
            self.get_logger().warn('No target point found on path, stopping')
            self.publish_drive_command(0.0, 0.0)
            return

        # Publish lookahead point for visualization
        lookahead_point_msg = PointStamped()
        lookahead_point_msg.header.frame_id = 'map'
        lookahead_point_msg.header.stamp = self.get_clock().now().to_msg()
        lookahead_point_msg.point.x = target_point[0]
        lookahead_point_msg.point.y = target_point[1]
        lookahead_point_msg.point.z = 0.0
        self.lookahead_point_pub.publish(lookahead_point_msg)
        
        # 5. Calculate steering angle
        raw_steering_angle = self.pure_pursuit_control(target_point)

        # 6. Apply steering smoothing (exponential moving average)
        steering_angle = (self.steer_smooth_alpha * raw_steering_angle +
                         (1.0 - self.steer_smooth_alpha) * self.previous_steering_angle)
        self.previous_steering_angle = steering_angle

        # Publish speed as a text marker for visualization
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'speed'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.0  # Position above the car
        marker.scale.z = 0.5  # Text size
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.text = f'{adaptive_speed:.2f} m/s'
        self.speed_marker_pub.publish(marker)

        # Publish steering angle as an arrow marker for visualization
        steer_marker = Marker()
        steer_marker.header.frame_id = 'base_link'
        steer_marker.header.stamp = self.get_clock().now().to_msg()
        steer_marker.ns = 'steering'
        steer_marker.id = 0
        steer_marker.type = Marker.ARROW
        steer_marker.action = Marker.ADD
        
        # Position the arrow at the front of the car
        steer_marker.pose.position.x = self.wheelbase 
        
        # Orientation from steering angle
        yaw = steering_angle
        q_w = math.cos(yaw * 0.5)
        q_z = math.sin(yaw * 0.5)
        steer_marker.pose.orientation.w = q_w
        steer_marker.pose.orientation.z = q_z

        # Scale of the arrow
        steer_marker.scale.x = 0.5  # Length
        steer_marker.scale.y = 0.1  # Width
        steer_marker.scale.z = 0.1  # Height

        # Color of the arrow (e.g., blue)
        steer_marker.color.a = 1.0
        steer_marker.color.r = 0.0
        steer_marker.color.g = 0.0
        steer_marker.color.b = 1.0

        self.steering_marker_pub.publish(steer_marker)

        # 7. Update current_speed for the next iteration
        self.current_speed = adaptive_speed

        # 8. Publish command
        self.publish_drive_command(adaptive_speed, steering_angle)

    def publish_drive_command(self, speed, steering_angle):
        """Publish Ackermann drive command"""
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_angle
        
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