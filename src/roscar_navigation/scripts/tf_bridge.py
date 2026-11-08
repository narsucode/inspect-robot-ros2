#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdomTfBridge(Node):
    def __init__(self):
        super().__init__('odom_tf_bridge')
        self.sub = self.create_subscription(Odometry, 'odom', self.cb, 10)
        self.br = TransformBroadcaster(self)
        self.get_logger().info('odom->base_footprint TF bridge running')

    def cb(self, msg):
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)

def main():
    rclpy.init()
    rclpy.spin(OdomTfBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
