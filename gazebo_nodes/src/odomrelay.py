#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdomToMap(Node):
    def __init__(self):
        super().__init__('odom_to_map')
        self.sub = self.create_subscription(Odometry, '/prius/odom', self.cb, 10)
        self.br = TransformBroadcaster(self)

    def cb(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'prius/odom'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = OdomToMap()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
