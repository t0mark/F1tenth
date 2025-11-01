#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor

import time
from modules.frenet_conversion import FrenetConverter
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from bisect import bisect_left
from nav_msgs.msg import Odometry
import math
import numpy as np
from tf_transformations import quaternion_from_euler
from tf_transformations import euler_from_quaternion
from typing import List, Tuple
import modules.utils as utils
from f1tenth_icra_race_msgs.msg import ObstacleArray, ObstacleMsg

from visualization_msgs.msg import Marker, MarkerArray

Point2D = Tuple[float, float]

# This is the equivalent of "latching" in ROS1
latching_qos = QoSProfile(
    depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST)


def normalize_s(x,track_length):
    x = x % (track_length)
    if x > track_length/2:
        x -= track_length
    return x


class Obstacle:
    """
    This class implements the properties of the obstacles
    """
    def __init__(self, x, y, size, theta) -> None:
        self.center_x = x
        self.center_y = y
        self.size = size
        self.id = None
        self.theta = theta

class OpponentDetection(Node):
    """
    This class implements a ROS node that detects obstacles on the track

    It subscribes to the following topics:
        - `/scan`: Publishes the lidar scans
        - '/ego_racecar/odom' or '/pf/pose/odom': Publishes the odometry of the ego car.

    The node publishes the following topics:
        - `/breakpoints_markers`: Publishes the breakpoint markers of the obstacles
        - `/raw_obstacles`: Publishes the detected obstacles
        - `/obstacles_markers_new`: Publishes the markers of the detected obstacles
    """

    def __init__(self) -> None:
        """
        Initialize the node, subscribe to topics, and create publishers and service proxies
        """
        super().__init__('opponent_detection')

        qos = rclpy.qos.QoSProfile(history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
                                   depth=1,
                                   reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
                                   durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)
        
        # Declare a parameter to check if the node is running in simulation
        self.declare_parameter('is_sim', True)
        self.is_sim = self.get_parameter('is_sim').value

        # Create parameters for plot and print debugging
        self.declare_parameter('plot_debug', False)
        self.plot_debug = self.get_parameter('plot_debug').value
        self.declare_parameter('print_debug', False)
        self.print_debug = self.get_parameter('print_debug').value

        # Create a subscriber to the pose topic
        msgs_cb_group = ReentrantCallbackGroup()
        if self.is_sim:
            self.pose_sub = self.create_subscription(
                Odometry,
                '/ego_racecar/odom',
                self.pose_callback,
                qos,
                callback_group=msgs_cb_group)
        else:
            self.pose_sub = self.create_subscription(
                Odometry,
                '/pf/pose/odom',
                self.pose_callback,
                qos,
                callback_group=msgs_cb_group)

        # Create a subscriber to the laser scan topic
        scan_topic = '/scan'
        self.laser_frame = 'ego_racecar/laser' if self.is_sim else 'laser'
        self.laser_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.laser_callback,
            qos,
            callback_group=msgs_cb_group)

        # Read the waypoints from the CSV file
        self.declare_parameter('waypoint_file', '/home/vaithak/Downloads/UPenn/F1Tenth/sim_ws/src/f1tenth_icra_race/waypoints/levine-practise-lane-optimal.csv')
        waypoint_file = self.get_parameter('waypoint_file').value
        self.waypoints = np.genfromtxt(waypoint_file, delimiter='; ', skip_header=1)
        waypoint_cols_dict = utils.column_numbers_for_waypoints()
        yaws = self.waypoints[:, waypoint_cols_dict['psi_racetraj_rad']]
        yaws = np.array(utils.convert_psi(yaws))
        self.d_right_array = self.waypoints[:, waypoint_cols_dict['width_right_m']]
        self.d_left_array = self.waypoints[:, waypoint_cols_dict['width_left_m']]
        self.s_array = self.waypoints[:, waypoint_cols_dict['s_racetraj_m']]
        self.smallest_d = min(min(self.d_right_array), min(self.d_left_array))
        self.biggest_d = max(self.d_right_array+self.d_left_array)
        self.track_length = self.waypoints[-1, waypoint_cols_dict['s_racetraj_m']]

        # Initialize the FrenetConverter object
        self.frenet_converter = FrenetConverter(
            self.waypoints[:, 0], self.waypoints[:, 1], yaws)
        self.get_logger().info(
            "[Opponent Detection]: initialized FrenetConverter object")

        # --- Node properties ---

        # --- Publisher ---
        if self.plot_debug:
            self.breakpoints_markers_pub = self.create_publisher(
                MarkerArray, '/perception/breakpoints_markers', 5)
            self.obstacles_marker_pub = self.create_publisher(
                MarkerArray, '/perception/obstacles_markers_new', 5)
    
        self.obstacles_msg_pub = self.create_publisher(
            ObstacleArray, '/perception/detection/raw_obstacles', 5)            

        self.declare_parameter("rate", 40, descriptor=ParameterDescriptor(
            description="rate at which the node is running"))
        self.declare_parameter("lambda", 10, descriptor=ParameterDescriptor(
            description="minimum reliables detection angle in degrees"))
        self.declare_parameter("sigma", 0.03, descriptor=ParameterDescriptor(
            description="standard deviation of the noise of the lidar ranges in m"))
        self.declare_parameter("min_2_points_dist", 0.01, descriptor=ParameterDescriptor(
            description="minimum distance between two points"))

        # --- Tunable params ---
        self.rate = self.get_parameter(
            "rate").get_parameter_value().integer_value
        self.lambda_angle = self.get_parameter(
            "lambda").get_parameter_value().integer_value * math.pi/180
        self.sigma = self.get_parameter(
            "sigma").get_parameter_value().double_value
        self.min_2_points_dist = self.get_parameter(
            "min_2_points_dist").get_parameter_value().double_value

        # Make the above params tunable instead of dynamic
        self.declare_parameter('min_obs_size', 10, descriptor=ParameterDescriptor(
            description="minimum number of points in an obstacle")
        )
        self.declare_parameter('max_obs_size', 0.5, descriptor=ParameterDescriptor(
            description="maximum size of an obstacle in m"),
        )
        self.declare_parameter('max_viewing_distance', 9.0, descriptor=ParameterDescriptor(
            description="maximum viewing distance of the lidar in m"),
        )
        self.min_obs_size = self.get_parameter(
            'min_obs_size').get_parameter_value().integer_value
        self.max_obs_size = self.get_parameter(
            'max_obs_size').get_parameter_value().double_value
        self.max_viewing_distance = self.get_parameter(
            'max_viewing_distance').get_parameter_value().double_value

        # --- variables ---
        # ego car s position
        self.car_s = None
        self.car_global_x = 0
        self.car_global_y = 0
        self.car_global_yaw = 0

        # raw scans from the lidar
        self.laser_scans = None
        self.angle_increment = 0
        self.angle_min = 0
        self.front_view_start_index = 0
        self.front_view_end_index = 0
        self.angles = None

        self.tracked_obstacles = []

        # main_timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.main_timer = self.create_timer(1/self.rate, self.loop, callback_group=msgs_cb_group)

    # --- Callbacks ---

    def pose_callback(self, pose_msg):
        # Get the current x, y position of the vehicle
        pose = pose_msg.pose.pose
        self.car_global_x = pose.position.x
        self.car_global_y = pose.position.y
        self.car_global_yaw = euler_from_quaternion([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])[2]
        if self.print_debug:
            self.get_logger().info(f'Pose: {self.car_global_x}, {self.car_global_y}, {self.car_global_yaw}')

        # Convert the global coordinates to Frenet coordinates
        s, _ = self.frenet_converter.get_frenet(np.array([self.car_global_x]), np.array([self.car_global_y]))
        self.car_s = normalize_s(s[0], self.track_length)


    def laser_callback(self, msg: LaserScan):
        self.laser_scans = msg
        self.angle_increment = msg.angle_increment
        self.angle_min = msg.angle_min
        self.front_view_start_index = self.angle_to_index(-np.pi/2)
        self.front_view_end_index = self.angle_to_index(np.pi/2)
        if self.angles is None:
            self.angles = np.arange(-np.pi/2, np.pi/2, self.angle_increment)

    # --- Functions ---

    def angle_to_index(self, angle):
        """ Convert a given angle in radians to an index in the LiDAR data.ranges array
        """
        index = (angle - self.angle_min) / self.angle_increment
        return int(index)

    def clearmarkers(self) -> MarkerArray:
        marker_array = MarkerArray()
        marker = Marker()
        marker.action = 3
        marker_array.markers = [marker]
        return marker_array

    def laserPointOnTrack(self, s, d, car_s) -> bool:
        if normalize_s(s-car_s, self.track_length) > self.max_viewing_distance:
            return False
        if abs(d) >= self.biggest_d:
            return False
        if abs(d) <= self.smallest_d:
            return True
        idx = bisect_left(self.s_array, s)
        if idx:
            idx -= 1
        if (d <= -self.d_right_array[idx]) or (d >= self.d_left_array[idx]):
            return False
        return True

    def scans2ObsPointCloud(self, car_s: Float32, scans: LaserScan, car_x: Float32, car_y: Float32, car_yaw: Float32) -> List[List[Point2D]]:
        """
        Converts the lidar scans to a 2D PointCloud and segments them into objects
        """
        # --- initialisation of some utility parameters ---
        l = self.lambda_angle
        d_phi = scans.angle_increment
        sigma = self.sigma

        # --- transform the scan ranges to a cloud point ---
        # Only consider angles from -90 to 90 degrees
        ranges = np.array(scans.ranges[self.front_view_start_index:self.front_view_end_index+1])
        x_laser_frame = (ranges * np.cos(self.angles)).flatten()
        y_laser_frame = (ranges * np.sin(self.angles)).flatten()
        z_laser_frame = np.zeros(len(ranges))
        # 4xN matrix
        xyz_laser_frame = np.vstack((x_laser_frame, y_laser_frame, z_laser_frame, np.ones(len(ranges))))

        # Form the transformation matrix using the car pose
        H_l2m = np.eye(4)
        H_l2m[0, 3] = car_x
        H_l2m[1, 3] = car_y
        H_l2m[2, 3] = 0.0
        H_l2m[0, 0] = np.cos(car_yaw)
        H_l2m[0, 1] = -np.sin(car_yaw)
        H_l2m[1, 0] = np.sin(car_yaw)
        H_l2m[1, 1] = np.cos(car_yaw)

        xyz_map = H_l2m @ xyz_laser_frame

        cloudPoints_list = np.transpose(xyz_map[:2, :]).tolist()

        # --------------------------------------------------
        # segment the cloud point into smaller point clouds
        # that represent potential object using the adaptive
        # method
        # --------------------------------------------------

        first_point: Point2D = (cloudPoints_list[0][0], cloudPoints_list[0][1])
        objects_pointcloud_list: List[List[Point2D]] = [[first_point]]

        div_const = np.sin(d_phi) / np.sin(l - d_phi)

        """
        for i in range(1, len(cloudPoints_list)):
            curr_range = scans.ranges[i]
            d_max = curr_range * div_const + 3 * sigma
 
            # Distance between points does not change in map frame or laser frame.
            dist_to_next_point = np.linalg.norm(xyz_laser_frame[:2, i] - xyz_laser_frame[:2, i - 1])
            
            # But from now onward, we deal with points in map frame.
            curr_point = (cloudPoints_list[i][0], cloudPoints_list[i][1])
            if dist_to_next_point < d_max:
                objects_pointcloud_list[-1].append(curr_point)
            else:
                objects_pointcloud_list.append([curr_point])

        The above logic is replaced with the following vectorized code
        """

        d_maxs = ranges * div_const + 3 * sigma

        # Calculate distances between consecutive points in the laser frame
        dists = np.linalg.norm(np.diff(xyz_laser_frame[:2, :], axis=1), axis=0)

        # Identify indices where a new object starts
        new_object_indices = np.where(dists >= d_maxs[:-1])[0] + 1

        # Split the cloudPoints_list into objects based on new_object_indices
        split_indices = np.split(np.arange(len(cloudPoints_list)), new_object_indices)
        objects_pointcloud_list = [np.array(cloudPoints_list)[indices].tolist() for indices in split_indices]

        # ------------------------------------------------
        # removing point clouds that are too small or too
        # big or that have their center point not on the
        # track
        # ------------------------------------------------

        x_points = []
        y_points = []
        for obs in objects_pointcloud_list:
            mean_x_pos = np.mean([point[0] for point in obs])
            mean_y_pos = np.mean([point[1] for point in obs])
            x_points.append(mean_x_pos)
            y_points.append(mean_y_pos)
        s_points, d_points = self.frenet_converter.get_frenet(np.array(x_points), np.array(y_points))

        # Use list comprehension to filter objects efficiently
        objects_pointcloud_list = [
            obj for idx, obj in enumerate(objects_pointcloud_list)
            if len(obj) >= self.min_obs_size and self.laserPointOnTrack(s_points[idx], d_points[idx], car_s)
        ]

        if self.plot_debug:
            markers_array = []
            for idx, object in enumerate(objects_pointcloud_list):
                # first element
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.id = idx*10
                marker.type = Marker.SPHERE
                marker.scale.x = 0.25
                marker.scale.y = 0.25
                marker.scale.z = 0.25
                marker.color.a = 0.5
                marker.color.g = 1.
                marker.color.r = 0.
                marker.color.b = idx/len(objects_pointcloud_list)
                marker.pose.position.x = object[0][0]
                marker.pose.position.y = object[0][1]
                marker.pose.orientation.w = 1.
                markers_array.append(marker)

                # last element
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.id = idx*10+2
                marker.type = Marker.SPHERE
                marker.scale.x = 0.25
                marker.scale.y = 0.25
                marker.scale.z = 0.25
                marker.color.a = 0.5
                marker.color.g = 1.
                marker.color.r = 0.
                marker.color.b = idx/len(objects_pointcloud_list)
                marker.pose.position.x = object[-1][0]
                marker.pose.position.y = object[-1][1]
                marker.pose.orientation.w = 1.
                markers_array.append(marker)
            # This causes the markers to flicker in RViz, but likely doesn't affect the underlying algo.
            self.breakpoints_markers_pub.publish(self.clearmarkers())
            if len(markers_array) > 0:
                self.breakpoints_markers_pub.publish(MarkerArray(markers=markers_array))

        return objects_pointcloud_list

    def obsPointClouds2obsArray(self, objects_pointcloud_list):
        current_obstacle_array = []
        min_dist = self.min_2_points_dist
        for obstacle in objects_pointcloud_list:
            # --- fit a rectangle to the data points ---
            theta = np.linspace(0, np.pi/2-np.pi/180, 90)
            cos_theta = np.cos(theta)
            sin_theta = np.sin(theta)
            distance1 = np.dot(obstacle, [cos_theta, sin_theta])
            distance2 = np.dot(obstacle, [-sin_theta, cos_theta])
            D10 = -distance1 + np.amax(distance1, axis=0)
            D11 = distance1 - np.amin(distance1, axis=0)
            D20 = -distance2 + np.amax(distance2, axis=0)
            D21 = distance2 - np.amin(distance2, axis=0)
            min_array = np.argmin(
                [np.linalg.norm(D10, axis=0), np.linalg.norm(D11, axis=0)], axis=0)
            D10 = np.transpose(D10)
            D11 = np.transpose(D11)
            D10[min_array == 1] = D11[min_array == 1]
            D10 = np.transpose(D10)
            min_array = np.argmin(
                [np.linalg.norm(D20, axis=0), np.linalg.norm(D21, axis=0)], axis=0)
            D20 = np.transpose(D20)
            D21 = np.transpose(D21)
            D20[min_array == 1] = D21[min_array == 1]
            D20 = np.transpose(D20)
            D = np.minimum(D10, D20)
            D[D < min_dist] = min_dist

            # --------------------------------------------
            # extract the center of the obstacle assuming
            # that it is actually a square obstacle
            # --------------------------------------------

            theta_opt = np.argmax(np.sum(np.reciprocal(D), axis=0))*np.pi/180
            distances1 = np.dot(
                obstacle, [np.cos(theta_opt), np.sin(theta_opt)])
            distances2 = np.dot(
                obstacle, [-np.sin(theta_opt), np.cos(theta_opt)])
            max_dist1 = np.max(distances1)
            min_dist1 = np.min(distances1)
            max_dist2 = np.max(distances2)
            min_dist2 = np.min(distances2)

            # corners are detected in a anti_clockwise manner
            corner1 = None
            corner2 = None
            # the obstacle has more detection in the verticle direction
            if (np.var(distances2) > np.var(distances1)):
                if (np.linalg.norm(-distances1+max_dist1) < np.linalg.norm(distances1-min_dist1)):
                    # the detections are nearer to the right edge
                    # lower_right_corner
                    corner1 = np.array([np.cos(theta_opt)*max_dist1-np.sin(theta_opt)*min_dist2,
                                        np.sin(theta_opt)*max_dist1+np.cos(theta_opt)*min_dist2])
                    # upper_right_corner
                    corner2 = np.array([np.cos(theta_opt)*max_dist1-np.sin(theta_opt)*max_dist2,
                                        np.sin(theta_opt)*max_dist1+np.cos(theta_opt)*max_dist2])
                else:
                    # the detections are nearer to the left edge
                    # upper_left_corner
                    corner1 = np.array([np.cos(theta_opt)*min_dist1-np.sin(theta_opt)*max_dist2,
                                        np.sin(theta_opt)*min_dist1+np.cos(theta_opt)*max_dist2])
                    # lower_left_corner
                    corner2 = np.array([np.cos(theta_opt)*min_dist1-np.sin(theta_opt)*min_dist2,
                                        np.sin(theta_opt)*min_dist1+np.cos(theta_opt)*min_dist2])
            else:  # the obstacle has more detection in the horizontal direction
                if (np.linalg.norm(-distances2+max_dist2) < np.linalg.norm(distances2-min_dist2)):
                    # the detections are nearer to the top edge
                    # upper_right_corner
                    corner1 = np.array([np.cos(theta_opt)*max_dist1-np.sin(theta_opt)*max_dist2,
                                        np.sin(theta_opt)*max_dist1+np.cos(theta_opt)*max_dist2])
                    # upper_left_corner
                    corner2 = np.array([np.cos(theta_opt)*min_dist1-np.sin(theta_opt)*max_dist2,
                                        np.sin(theta_opt)*min_dist1+np.cos(theta_opt)*max_dist2])
                else:
                    # the detections are nearer to the bottom edge
                    # lower_left_corner
                    corner1 = np.array([np.cos(theta_opt)*min_dist1-np.sin(theta_opt)*min_dist2,
                                        np.sin(theta_opt)*min_dist1+np.cos(theta_opt)*min_dist2])
                    # lower_right_corner
                    corner2 = np.array([np.cos(theta_opt)*max_dist1-np.sin(theta_opt)*min_dist2,
                                        np.sin(theta_opt)*max_dist1+np.cos(theta_opt)*min_dist2])
            # vector that goes from corner1 to corner2
            colVec = np.array([corner2[0]-corner1[0], corner2[1]-corner1[1]])
            # orthogonal vector to the one that goes from corner1 to corner2
            orthVec = np.array([-colVec[1], colVec[0]])
            # center position
            center = corner1 + 0.5*colVec + 0.5*orthVec

            current_obstacle_array.append(
                Obstacle(center[0], center[1], np.linalg.norm(colVec), theta_opt))

        self.get_logger().debug(
            f"[Opponent Detection] detected {len(current_obstacle_array)} raw obstacles.")

        return current_obstacle_array

    def checkObstacles(self, current_obstacles):
        """
        Delete obstacles that are too big
        """

        remove_list = []
        self.tracked_obstacles.clear()
        for obs in current_obstacles:
            if (obs.size > self.max_obs_size):
                remove_list.append(obs)

        self.get_logger().debug(
            f"[Opponent Detection] removed {len(remove_list)} obstacles as they are too big.")

        for obs in remove_list:
            current_obstacles.remove(obs)

        for idx, curr_obs in enumerate(current_obstacles):
            curr_obs.id = idx
            self.tracked_obstacles.append(curr_obs)

        self.get_logger().debug(
            f"[Opponent Detection] tracking {len(self.tracked_obstacles)} obstacles.")

    def publishObstaclesMessage(self):
        obstacles_array_message = ObstacleArray()
        obstacles_array_message.header.stamp = self.get_clock().now().to_msg()
        obstacles_array_message.header.frame_id = "map"

        x_center = []
        y_center = []
        for obstacle in self.tracked_obstacles:
            x_center.append(obstacle.center_x)
            y_center.append(obstacle.center_y)

        s_points, d_points = self.frenet_converter.get_frenet(
            np.array(x_center), np.array(y_center))

        for idx, obstacle in enumerate(self.tracked_obstacles):
            s = s_points[idx]
            d = d_points[idx]

            obsMsg = ObstacleMsg()
            obsMsg.id = obstacle.id
            obsMsg.s_start = s-obstacle.size/2
            obsMsg.s_end = s+obstacle.size/2
            obsMsg.d_left = d+obstacle.size/2
            obsMsg.d_right = d-obstacle.size/2
            obsMsg.s_center = s
            obsMsg.d_center = d
            obsMsg.size = obstacle.size

            obstacles_array_message.obstacles.append(obsMsg)

        self.obstacles_msg_pub.publish(obstacles_array_message)

    def publishObstaclesMarkers(self):
        markers_array = []
        for obs in self.tracked_obstacles:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = obs.id
            marker.type = Marker.CUBE
            marker.scale.x = obs.size
            marker.scale.y = obs.size
            marker.scale.z = obs.size
            marker.color.a = 0.5
            marker.color.g = 1.
            marker.color.r = 0.
            marker.color.b = 1.
            marker.pose.position.x = obs.center_x
            marker.pose.position.y = obs.center_y
            q = quaternion_from_euler(0, 0, obs.theta)
            marker.pose.orientation.x = q[0]
            marker.pose.orientation.y = q[1]
            marker.pose.orientation.z = q[2]
            marker.pose.orientation.w = q[3]
            markers_array.append(marker)

        # This causes the markers to flicker in RViz, but likely doesn't affect the underlying algo.
        self.obstacles_marker_pub.publish(self.clearmarkers())
        if len(markers_array) > 0:
            self.obstacles_marker_pub.publish(MarkerArray(markers=markers_array))


    def loop(self):
        """
        Main loop of the node
        """
        if self.laser_scans is None or self.car_s is None:
            return

        scans = self.laser_scans
        car_x = self.car_global_x
        car_y = self.car_global_y
        car_yaw = self.car_global_yaw
        car_s = self.car_s

        # --- obstacle detection ---
        if self.print_debug:
            start_time = time.perf_counter()
        
        objects_pointcloud_list = self.scans2ObsPointCloud(
            car_s=car_s, scans=scans, car_x=car_x, car_y=car_y, car_yaw=car_yaw)
        current_obstacles = self.obsPointClouds2obsArray(objects_pointcloud_list)
        self.checkObstacles(current_obstacles)

        self.publishObstaclesMessage()
        if self.plot_debug:
            self.publishObstaclesMarkers()

        if self.print_debug:
            end_time = time.perf_counter()
            latency = end_time - start_time
            self.get_logger().info(f"Latency checkObstacles: {latency:.4f} seconds")

def main():
    rclpy.init()
    detect = OpponentDetection()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(detect)
    try:
        executor.spin()
    except KeyboardInterrupt:
        detect.get_logger().info("KeyboardInterrupt")


    detect.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()