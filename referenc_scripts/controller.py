#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from nav_msgs.msg import Odometry
from f1tenth_icra_race_msgs.msg import WpntArray, ObstacleArray
from std_msgs.msg import String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from ackermann_msgs.msg import AckermannDriveStamped

import numpy as np
from modules.pure_pursuit import PP_Controller
from modules import utils
from modules.frenet_conversion import FrenetConverter
from tf_transformations import euler_from_quaternion


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller')
        self.declare_parameter("waypoint_file", "/home/vaithak/Downloads/UPenn/F1Tenth/sim_ws/src/f1tenth_icra_race/waypoints/levine-practise-lane-optimal.csv")

        # Load local waypoint CSV file for reference track
        waypoint_file = self.get_parameter("waypoint_file").get_parameter_value().string_value
        waypoints = np.genfromtxt(waypoint_file, delimiter=';', skip_header=1)
        waypoint_cols = utils.column_numbers_for_waypoints()
        self.waypoints_x = waypoints[:, waypoint_cols['x_ref_m']]
        self.waypoints_y = waypoints[:, waypoint_cols['y_ref_m']]
        self.waypoints_psi = waypoints[:, waypoint_cols['psi_racetraj_rad']]
        # self.waypoints_psi = utils.convert_psi(self.waypoints_psi)
        self.track_length = float(waypoints[-1, waypoint_cols['s_racetraj_m']])
        self.converter = FrenetConverter(self.waypoints_x, self.waypoints_y, self.waypoints_psi)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # Subscriptions
        self.declare_parameter("is_sim", True)
        self.is_sim = self.get_parameter("is_sim").get_parameter_value().bool_value
        if self.is_sim:
            self.pose_sub = self.create_subscription(
                Odometry,
                '/ego_racecar/odom',
                self.pose_callback,
                qos)
        else:
            self.pose_sub = self.create_subscription(
                Odometry,
                '/pf/pose/odom',
                self.pose_callback,
                qos)
        self.create_subscription(WpntArray, '/state_machine/local_waypoints', self.wpnts_callback, qos)
        self.create_subscription(String, '/state_machine/state', self.state_callback, qos)
        self.create_subscription(ObstacleArray, '/perception/obstacles', self.obstacles_callback, qos)

        # Publisher for commands
        self.declare_parameter('drive_topic', '/drive')
        drive_topic = self.get_parameter('drive_topic').value
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, qos)

        # Create parameters for plot and print debugging
        self.declare_parameter('plot_debug', False)
        self.plot_debug = self.get_parameter('plot_debug').value
        self.declare_parameter('print_debug', False)
        self.print_debug = self.get_parameter('print_debug').value

        # Publisher for visualization
        self.lookahead_pub = self.create_publisher(Marker, '/controller/lookahead_point', qos)
        self.l1_pub = self.create_publisher(Point, 'l1_distance', qos)

        # Internal state
        self.current_pose = None  # [x, y, yaw]
        self.current_pose_frenet = None  # [s, d, vs, vd]
        self.current_speed = 0.0
        self.local_waypoints = []  # List of (x, y, v, s)
        self.opponent = None  # ObstacleMsg
        self.state = None  # StateType

        # Timer loop
        self.declare_parameter('rate_hz', 50.0)
        self.rate_hz = self.get_parameter('rate_hz').value
        self.timer = self.create_timer(1.0 / self.rate_hz, self.control_loop)

        # Pure Pursuit Controller
        self.declare_parameter('t_clip_min', 1.0)
        self.declare_parameter('t_clip_max', 7.0)
        self.declare_parameter('q_l1', -0.65)
        self.declare_parameter('m_l1', 0.65)
        self.declare_parameter('speed_lookahead', 0.1)
        self.declare_parameter('lat_err_coeff', 0.25)
        self.declare_parameter('start_scale_speed', 0.5)
        self.declare_parameter('speed_lookahead_for_steer', 0.1)
        self.declare_parameter('trailing_gap', 1.0)
        self.declare_parameter('trailing_p_gain', 0.5)
        self.declare_parameter('trailing_d_gain', 0.5)
        self.pure_pursuit_control = PP_Controller(
            self.get_parameter('t_clip_min').value,
            self.get_parameter('t_clip_max').value,
            self.get_parameter('m_l1').value,
            self.get_parameter('q_l1').value,
            self.get_parameter('speed_lookahead').value,
            self.get_parameter('lat_err_coeff').value,
            self.get_parameter('start_scale_speed').value,
            self.get_parameter('speed_lookahead_for_steer').value,
            self.get_parameter('trailing_gap').value,
            self.get_parameter('trailing_p_gain').value,
            self.get_parameter('trailing_d_gain').value
        )


    def pose_callback(self, pose_msg):
        # Get the current x, y position of the vehicle
        pose = pose_msg.pose.pose
        yaw = euler_from_quaternion([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])[2]
        if self.print_debug:
            self.get_logger().info(f'Pose: {self.car_global_x}, {self.car_global_y}, {self.car_global_yaw}')

        # Set the current speed
        self.current_speed = pose_msg.twist.twist.linear.x

        # Set the current pose
        self.current_pose = [pose.position.x, pose.position.y, yaw]

        # Convert the global coordinates to Frenet coordinates
        s, d = self.converter.get_frenet(np.array([pose.position.x]), np.array([pose.position.y]))
        vs, vd = self.converter.get_frenet_velocities(np.array([pose_msg.twist.twist.linear.x]), np.array([pose_msg.twist.twist.linear.y]), yaw)
        self.current_pose_frenet = [s[0], d[0], vs[0], vd[0]]

    def wpnts_callback(self, msg: WpntArray):
        if self.local_waypoints is None or len(msg.wpnts) > 0:
            self.local_waypoints = []
        for waypoint in msg.wpnts:
            self.local_waypoints.append([
                waypoint.x_m,
                waypoint.y_m,
                waypoint.vx_mps,
                waypoint.s_m, waypoint.psi_rad, waypoint.ax_mps2])
        self.local_waypoints = np.array(self.local_waypoints)

    def state_callback(self, msg: String):
        self.state = msg.data
        if self.print_debug:
            self.get_logger().info(f'State: {self.state}')

    def obstacles_callback(self, msg: ObstacleArray):
        if msg.obstacles and len(msg.obstacles) > 0:
            obstacle = msg.obstacles[0]
            self.opponent = [obstacle.s_center, obstacle.d_center, obstacle.vs, obstacle.is_visible]
        else:
            self.opponent = None

    def pure_pursuit_control_and_visualize(self):
        speed, steering_angle, L1_point, _, _ = self.pure_pursuit_control.main_loop(
            self.state,
            self.current_pose,
            self.local_waypoints,
            self.current_speed,
            self.opponent,
            self.current_pose_frenet,
            self.track_length
        )

        if self.plot_debug:
            self.set_lookahead_marker(L1_point, 100)
        return speed, steering_angle


    def control_loop(self):
        if self.current_pose is None or self.local_waypoints is None:
            return
        if len(self.local_waypoints) == 0:
            return

        speed, steering_angle = self.pure_pursuit_control_and_visualize()

        # Publish the drive command
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_angle
        self.drive_pub.publish(drive_msg)

    ### Visualization methods
    def set_lookahead_marker(self, lookahead_point, id):
        lookahead_marker = Marker()
        lookahead_marker.header.frame_id = "map"
        lookahead_marker.header.stamp = self.get_clock().now().to_msg()
        lookahead_marker.type = 2
        lookahead_marker.id = id
        lookahead_marker.scale.x = 0.15
        lookahead_marker.scale.y = 0.15
        lookahead_marker.scale.z = 0.15
        lookahead_marker.color.r = 1.0
        lookahead_marker.color.g = 0.0
        lookahead_marker.color.b = 0.0
        lookahead_marker.color.a = 1.0
        lookahead_marker.pose.position.x = lookahead_point[0]
        lookahead_marker.pose.position.y = lookahead_point[1]
        lookahead_marker.pose.position.z = 0.0
        lookahead_marker.pose.orientation.x = 0.0
        lookahead_marker.pose.orientation.y = 0.0
        lookahead_marker.pose.orientation.z = 0.0
        lookahead_marker.pose.orientation.w = 1.0
        self.lookahead_pub.publish(lookahead_marker)


def main(args=None):
    rclpy.init(args=args)
    print("Controller Node Initialized")
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
