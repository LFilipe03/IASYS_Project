#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from ultralytics import YOLO

class GesturePublisherNode(Node):

    def __init__(self):
        super().__init__('gesture_publisher_node')

        # --- 1. YOLO LOAD (CRITICAL: MUST BE IN __INIT__) ---
        self.get_logger().info('Loading YOLO model... (this takes a moment)')
        # This loads the model into memory ONCE. 
        # If this line is inside a callback, your PC will crash.
        self.model = YOLO('yolov8n-pose.pt') 
        
        self.conf_threshold = 0.5
        self.trigger_margin = 40
        self.stop_margin = 50

        # --- 2. ROS SETUP ---
        self.bridge = CvBridge()
        self.latest_frame = None

        # Publisher (Output)
        self.command_publisher_ = self.create_publisher(
            String,
            '/atc/perception', 
            10
        )

        # Subscriber (Input)
        # We subscribe to the camera. When a frame arrives, we save it.
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Timer (Processing Loop)
        # Process data at 10Hz (Safety Limit to prevent CPU overload)
        self.timer = self.create_timer(0.1, self.process_latest_frame)

        self.get_logger().info('Gesture Publisher Node Ready.')

    def image_callback(self, msg):
        """
        Just saves the latest image. Does NO processing.
        This keeps the camera stream smooth.
        """
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Frame conversion error: {e}')

    def process_latest_frame(self):
        """
        Runs every 0.1s. Performs inference and publishes result.
        """
        if self.latest_frame is None:
            return

        # 1. Prepare Frame
        frame = self.latest_frame.copy()
        
        # NOTE: If your camera is mirrored, keep this. If not, comment it out.
        # frame = cv2.flip(frame, 1)

        # 2. Run Inference
        # verbose=False prevents the terminal from spamming "Ultralytics..."
        results = self.model(frame, verbose=False)
        
        detected_command = "WAITING"

        # 3. Analyze Keypoints
        if results[0].keypoints is not None and results[0].keypoints.data.shape[0] > 0:
            kpts = results[0].keypoints.data.cpu().numpy()[0]

            if len(kpts) >= 17:
                # Keypoints Map: 5=L_Shoulder, 6=R_Shoulder, 9=L_Wrist, 10=R_Wrist
                l_shldr = kpts[5]
                r_shldr = kpts[6]
                l_wrist = kpts[9]
                r_wrist = kpts[10]

                stop_detected = False

                # LOGIC: Check STOP
                is_left_stop = (l_wrist[2] > self.conf_threshold and l_shldr[2] > self.conf_threshold and 
                                l_wrist[1] < (l_shldr[1] - self.stop_margin))
                is_right_stop = (r_wrist[2] > self.conf_threshold and r_shldr[2] > self.conf_threshold and 
                                 r_wrist[1] < (r_shldr[1] - self.stop_margin))

                if is_left_stop or is_right_stop:
                    detected_command = "STOP"
                    stop_detected = True

                # LOGIC: Check Direction (Only if not stopped)
                if not stop_detected:
                    # Left
                    if l_wrist[2] > self.conf_threshold and l_shldr[2] > self.conf_threshold:
                        dist = l_shldr[0] - l_wrist[0]
                        if dist > self.trigger_margin:
                            detected_command = "GO LEFT"

                    # Right
                    if r_wrist[2] > self.conf_threshold and r_shldr[2] > self.conf_threshold:
                        dist = r_wrist[0] - r_shldr[0]
                        if dist > self.trigger_margin:
                            detected_command = "GO RIGHT"

        # 4. Publish
        msg = String()
        msg.data = detected_command
        self.command_publisher_.publish(msg)

        # Log change for debugging
        if detected_command != "WAITING":
            self.get_logger().info(f'Command: {detected_command}')

def main(args=None):
    rclpy.init(args=args)
    node = GesturePublisherNode()

    try:
        # Standard spin is safer for CV nodes than MultiThreaded
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Strict cleanup order prevents "Handle Destroyed" error
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()