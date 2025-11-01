#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from numpy import linalg as LA
from visualization_msgs.msg import MarkerArray, Marker

MODE = 2

def prune_points(points, distance_threshold=0.15):
    """
    Prune points to ensure that only points with a distance of `distance_threshold` are kept.
    """
    pruned_points = [points[0]]
    for i in range(1, len(points)):
        dist = np.linalg.norm(points[i] - pruned_points[-1])
        if dist >= distance_threshold:
            pruned_points.append(points[i])
    return np.array(pruned_points)

def FitPath(points):
    """
    Points are in the form of (x, y, yaw).
    """
    if MODE == 1:
        return points
    elif MODE == 2:
        # Fit a cubic spline to the (x, y) points
        from scipy.interpolate import CubicSpline
        # repetition_points = 2
        # x = points[:-(repetition_points - 1), 0]
        # y = points[:-(repetition_points - 1), 1]
        x = points[:, 0]
        y = points[:, 1]
        t = np.linspace(0, 1, len(x))
        cs_x = CubicSpline(t, x, bc_type='clamped')
        cs_y = CubicSpline(t, y, bc_type='clamped')
        t_new = np.linspace(0, 1, 500)
        x_new = cs_x(t_new)
        y_new = cs_y(t_new)

        # Prune points to ensure that only points with a distance of 0.2 are kept
        # x_new, y_new = prune_points(np.column_stack((x_new, y_new)), distance_threshold=0.2).T
        # print("Number of points after pruning: ", len(x_new))

        # Yaw will be dy/dx = y'(t) / x'(t)
        dx = np.gradient(x_new)
        dy = np.gradient(y_new)
        yaw_new = np.arctan2(dy, dx)
        # yaw_new = np.unwrap(yaw_new)  # Unwrap the angles to avoid discontinuities

        # Calculate the curvature (kappa) at each point
        kappa = np.zeros_like(x_new)
        for i in range(1, len(x_new) - 1):
            dx = x_new[i + 1] - x_new[i - 1]
            dy = y_new[i + 1] - y_new[i - 1]
            d2x = (x_new[i + 1] - 2 * x_new[i] + x_new[i - 1])
            d2y = (y_new[i + 1] - 2 * y_new[i] + y_new[i - 1])
            kappa[i] = (dx * d2y - dy * d2x) / ((dx ** 2 + dy ** 2) ** (3 / 2))
        kappa[0] = kappa[1]
        kappa[-1] = kappa[-2]

        # Calculate the `s` for each point
        s = np.zeros_like(x_new)
        for i in range(1, len(x_new)):
            s[i] = s[i - 1] + np.sqrt((x_new[i] - x_new[i - 1]) ** 2 + (y_new[i] - y_new[i - 1]) ** 2)

        vx = np.zeros_like(x_new)
        vx += 2.5 # m/s for now, TODO: fix this

        return np.column_stack((x_new, y_new, yaw_new, s, kappa, vx))

class PathPlotter(Node):
    """
    Given a parameter that specifies the path to a CSV file containing waypoints,
    this node reads the waypoints and publishes them as a MarkerArray for visualization.
    The CSV file should have the following format:
    x1, y1, yaw1, (speed1)? - first row, speed may or may not be present, just extract the first 3 values

    For each waypoint, a Marker is create with postion (x, y) and orientation (yaw) as a quaternion.
    """
    def __init__(self):
        super().__init__('path_plotter')

        # Create publisher
        self.publisher = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)

        # Create a ros2 parameter to specify the waypoint file
        self.declare_parameter('waypoint_file', '/home/vaithak/Downloads/UPenn/F1Tenth/sim_ws/src/f1tenth_icra_race/waypoints/icra_race_2_interactive.csv')
        waypoint_file = self.get_parameter('waypoint_file').value
        self.waypoints = np.genfromtxt(waypoint_file, delimiter=',')
        self.waypoints = self.waypoints
        # Append the first point to the end of the array
        self.waypoints = np.append(self.waypoints, [self.waypoints[0]], axis=0)

        # Fit the waypoints
        self.waypoints = FitPath(self.waypoints)

        # Create a MarkerArray
        self.marker_array = MarkerArray()
        self.marker_array.markers = []

        print("Number of waypoints: ", len(self.waypoints))

        # Create a Marker for each waypoint
        for i, waypoint in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            if len(self.waypoints) < 50:
                marker.type = marker.ARROW
            else:
                marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.pose.position.x = waypoint[0]
            marker.pose.position.y = waypoint[1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0 #np.sin(waypoint[2] / 2)
            marker.pose.orientation.w = 1.0 #np.cos(waypoint[2] / 2)
            if len(self.waypoints) < 50:
                marker.scale.x = 1.0
            else:
                marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            if MODE == 2:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            elif MODE == 1:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            self.marker_array.markers.append(marker)

        # Publish the MarkerArray
        self.publisher.publish(self.marker_array)

        # Save the points to a CSV file
        if MODE == 2:
            save_path = '/home/vaithak/Downloads/UPenn/F1Tenth/sim_ws/src/f1tenth_icra_race/waypoints/icra_race_2_interactive_fitted.csv'
            # Add a header to the CSV file
            header = 'x_ref_m,y_ref_m,psi_racetraj_rad,s_racetraj_m,kappa_racetraj_radpm,vx_racetraj_mps'
            np.savetxt(save_path, self.waypoints, delimiter=',', header=header, comments='', fmt='%.4f')

def main(args=None):
    rclpy.init(args=args)
    path_plotter = PathPlotter()
    rclpy.spin(path_plotter)
    path_plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



