#!/usr/bin/env python3

import rclpy, os, signal, cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraSensor(Node):
    def __init__(self):
        super().__init__('camera_sensor')

        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Não foi possível aceder a câmera!')
            return

        self.cap.set(3, 640)  
        self.cap.set(4, 480)  

        self.bridge = CvBridge()

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().error('Falha ao capturar imagem')
            return

        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)
            self.get_logger().info('Publicando uma nova imagem')
        except Exception as e:
            self.get_logger().error(f'Erro ao converter imagem: {e}')

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraSensor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        os.kill(os.getpid(), signal.SIGTERM)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
