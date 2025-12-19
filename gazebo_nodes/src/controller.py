#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math


class Controller(Node):
    DRIVE = 0
    STOPPED = 1

    def __init__(self):
        super().__init__('controller')

        # Publishers
        self.cmd_pub = self.create_publisher(
            Twist, '/prius/cmd_vel', 10)

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/prius/imu', self.imu_cb, 10)

        self.perception_sub = self.create_subscription(
            String, '/atc/perception', self.perception_cb, 10)

        # State
        self.state = self.DRIVE

        # IMU control
        self.target_yaw = None
        self.current_yaw = 0.0

        self.kp = 2.5
        self.speed = 10.0

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Controller node started.')

    # ---------- Perception callback ----------
    def perception_cb(self, msg):
        cmd = msg.data.strip().upper()

        if cmd == 'STOP':
            if self.state != self.STOPPED:
                self.get_logger().warn('STOP received from perception!')
            self.state = self.STOPPED
            self.publish_stop()

    # ---------- IMU callback ----------
    def imu_cb(self, msg):
        if self.state == self.STOPPED:
            return

        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        if self.target_yaw is None:
            self.target_yaw = self.current_yaw
            self.get_logger().info(f'Locked heading: {self.target_yaw:.3f} rad')

    # ---------- Control ----------
    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def control_loop(self):
        if self.state == self.STOPPED:
            self.publish_stop()
            return

        if self.target_yaw is None:
            return

        error = self.target_yaw - self.current_yaw
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
