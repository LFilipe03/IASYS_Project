#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rclpy.time import Time
from etsi_its_msgs.msg import (
    DENM,
    ItsPduHeader,
    ManagementContainer,
    ActionID,
    ReferencePosition,
    RelevanceDistance,
    RelevanceTrafficDirection,
    StationType
)

import math


class Controller(Node):
    DRIVE = 0
    ROADWORK_DRIVE = 1
    STOPPED = 2
    GO_RIGHT = 3
    GO_LEFT = 4


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

        self.warning_sub = self.create_subscription(
            DENM,
            '/infra/roadwork/beacon',
            self.warning_cb,
            10)


        # State
        self.state = self.DRIVE

        # IMU control
        self.target_yaw = None
        self.current_yaw = 0.0

        self.kp = 2.5
        self.speed = 14.0  # m/s

        self.timer = self.create_timer(0.05, self.control_loop)

        self.lane_change_duration = 5  # seconds
        self.lane_change_start_time = None
        self.phase1_time = 4.5

        self.left_yaw_offset = 0.12    # ~20°
        self.right_yaw_offset = -0.01   # counter-steer

        self.initial_yaw = None

        self.get_logger().info('Controller node started.')

    # ---------- Perception callback ----------
    def perception_cb(self, msg):
        cmd = msg.data.strip().upper()

        if cmd == 'STOP':
            if self.state != self.STOPPED:
                self.get_logger().warn('STOP received from perception!')
            self.state = self.STOPPED
            self.publish_stop()

        if cmd == 'GO_LEFT' and self.state == self.GO_LEFT: 
            now = self.get_clock().now()
            elapsed = (now - self.lane_change_start_time).nanoseconds * 1e-9
            self.get_logger().info(f'Doing GO_LEFT Maneuver (elapsed: {elapsed:.2f} seconds)')
            return
        
        if cmd == 'GO_LEFT':
            self.get_logger().info('GO_LEFT')
            self.state = self.GO_LEFT
            self.lane_change_start_time = self.get_clock().now()
            self.initial_yaw = self.current_yaw

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
    
    # ---------- DENM callback ----------
    def warning_cb(self, msg: DENM):
        # Ignore if stopped or mid-maneuver
        if self.state in [self.STOPPED, self.GO_LEFT, self.GO_RIGHT]:
            return

        # Basic sanity check: is this really a DENM?
        if msg.its_header.message_id != ItsPduHeader.MESSAGE_ID_DENM:
            self.get_logger().warn(
                'NOT A DEMN'
            )
            return

        if self.state != self.ROADWORK_DRIVE:
            self.get_logger().warn(
                'ROADWORK DENM received → slowing down to 8 m/s'
            )
            self.state = self.ROADWORK_DRIVE

    # ---------- Control ----------
    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def control_loop(self):
        if self.state == self.STOPPED:
            self.publish_stop()
            return

        if self.target_yaw is None:
            return

        #Handle lane change timing
        if self.state == self.GO_LEFT:
            now = self.get_clock().now()
            elapsed = (now - self.lane_change_start_time).nanoseconds * 1e-9
            self.speed = 10.0

            # Phase 1: steer left
            if elapsed < self.phase1_time:
                self.target_yaw = self.initial_yaw + self.left_yaw_offset

            # Phase 2: steer right to straighten
            elif elapsed < self.lane_change_duration:
                self.target_yaw = self.initial_yaw + self.right_yaw_offset

            # Finish maneuver
            else:
                self.get_logger().info('Lane change completed, car straightened')
                self.state = self.DRIVE
                self.target_yaw = self.initial_yaw
                return

        if self.state == self.ROADWORK_DRIVE:
            self.speed = 5.0

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
