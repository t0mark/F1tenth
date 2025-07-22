#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_ros import TransformBroadcaster


class TransformRemapNode(Node):
    """
    Node to remap transform structure for AMCL compatibility.
    
    This node:
    1. Subscribes to /ego_racecar/odom (with frame_id='map')
    2. Republishes odometry with frame_id='odom' 
    3. Publishes odom->base_link transform based on odometry
    4. The original map->base_link transform from gym_bridge remains for ground truth
    """
    
    def __init__(self):
        super().__init__('transform_remap')
        
        # Publishers
        self.odom_pub = self.create_publisher(
            Odometry, 
            '/odom', 
            10
        )
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )
        
        self.get_logger().info('Transform remap node started')
        self.get_logger().info('Subscribing to: /ego_racecar/odom')
        self.get_logger().info('Publishing to: /odom')
        self.get_logger().info('Broadcasting: odom -> ego_racecar/base_link transform')
    
    def odom_callback(self, msg):
        """
        Process incoming odometry message.
        
        Args:
            msg (Odometry): Original odometry message with frame_id='map'
        """
        # Create new odometry message with odom frame
        odom_msg = Odometry()
        odom_msg.header.stamp = msg.header.stamp
        odom_msg.header.frame_id = 'odom'  # Change from 'map' to 'odom'
        odom_msg.child_frame_id = msg.child_frame_id  # Keep 'ego_racecar/base_link'
        
        # Copy pose and twist data
        odom_msg.pose = msg.pose
        odom_msg.twist = msg.twist
        
        # Publish remapped odometry
        self.odom_pub.publish(odom_msg)
        
        # Create and publish odom -> base_link transform
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = msg.child_frame_id
        
        # Copy translation and rotation from odometry
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    node = TransformRemapNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Transform remap node shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()