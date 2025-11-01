#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from numpy import linalg as LA
from os.path import expanduser
from time import gmtime, strftime
from sensor_msgs.msg import LaserScan

# If using tf_transformations in ROS 2, ensure it is installed:
#   sudo apt-get install ros-<your_distro>-tf-transformations
from tf_transformations import euler_from_quaternion


class WaypointsLogger(Node):
    def __init__(self):
        super().__init__('waypoints_logger')

        # Prepare a timestamped file
        home = expanduser('~')
        filename = '/home/vaithak/Downloads/UPenn/F1Tenth/race3_map_edited/width_waypoints-prak.csv'
        self.file = open(filename, 'w')

        # Create subscription
        self.subscription = self.create_subscription(
            Odometry,
            'ego_racecar/odom',    # Topic name
            self.save_waypoint,
            10                 # QoS queue size
        )
        self.subscription  # prevent unused variable warning
        self.pose_msg = None

        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        self.get_logger().info('Waypoints Logger node has started. Logging to: {}'.format(filename))

    def save_waypoint(self, msg):
        self.pose_msg = msg

    def angle_to_index(self, angle, angle_min, angle_increment):
        """ Convert a given angle in radians to an index in the LiDAR data.ranges array
        """
        index = (angle - angle_min) / angle_increment
        return int(index)

    def scan_callback(self, msg):
        if self.pose_msg is None:
            self.get_logger().warn('Pose message not received yet. Skipping scan data.')
            return

        # Extract quaternion
        qx = self.pose_msg.pose.pose.orientation.x
        qy = self.pose_msg.pose.pose.orientation.y
        qz = self.pose_msg.pose.pose.orientation.z
        qw = self.pose_msg.pose.pose.orientation.w

        # Convert to Euler angles
        euler = euler_from_quaternion([qx, qy, qz, qw])
        yaw = euler[2]  # roll = euler[0], pitch = euler[1], yaw = euler[2]

        # Compute linear speed
        vx = self.pose_msg.twist.twist.linear.x
        vy = self.pose_msg.twist.twist.linear.y
        vz = self.pose_msg.twist.twist.linear.z
        speed = LA.norm([vx, vy, vz], 2)

        # Print to console if needed (example: only if vx > 0)
        if vx > 0.0:
            self.get_logger().info(f"Forward velocity: {vx}")

        # take the -90 degree and +90 degree laser scan data
        index_1 = self.angle_to_index(-np.pi / 2, msg.angle_min, msg.angle_increment)
        index_2 = self.angle_to_index(np.pi / 2, msg.angle_min, msg.angle_increment)
        laser_data_1 = msg.ranges[index_1]
        laser_data_2 = msg.ranges[index_2]

        # Write to file: x, y, yaw, speed
        self.file.write('%f, %f, %f, %f\n' %
                        (self.pose_msg.pose.pose.position.x,
                         self.pose_msg.pose.pose.position.y,
                         laser_data_1,
                         laser_data_2))

    def destroy_node(self):
        # Close the file before shutting down
        self.file.close()
        self.get_logger().info('Waypoints Logger node is shutting down; file closed.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointsLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure everything shuts down cleanly
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()