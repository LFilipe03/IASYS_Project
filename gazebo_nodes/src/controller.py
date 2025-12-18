#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math

velocity = 5.0  # m/s

class Controller(Node):
    def __init__(self):
        super().__init__('straight_imu_controller')

        self.cmd_pub = self.create_publisher(Twist, '/prius/cmd_vel', 10)
        self.imu_sub = self.create_subscription(Imu, '/prius/imu', self.imu_cb, 10)

        self.target_yaw = None
        self.current_yaw = 0.0

        self.kp = 2.5          # steering gain
        self.speed = 35.0       # forward speed (m/s)

        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

    def imu_cb(self, msg):
        q = msg.orientation

        # quaternion → yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        if self.target_yaw is None:
            self.target_yaw = self.current_yaw
            self.get_logger().info(f'Locked heading at {self.target_yaw:.3f} rad')

    def control_loop(self):
        if self.target_yaw is None:
            return

        error = self.target_yaw - self.current_yaw

        # normalize error to [-pi, pi]
        error = math.atan2(math.sin(error), math.cos(error))

        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = self.kp * error

        self.cmd_pub.publish(twist)

def main():
    rclpy.init()
    node = Controller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
