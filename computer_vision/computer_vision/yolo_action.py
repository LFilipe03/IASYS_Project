#!/usr/bin/env python3
 
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import yolo_messages
from yolo_messages.action import YoloAction
import cv2
import numpy as np
from ultralytics import YOLO
import time
 
   
class GestureActionServer(Node):
 
    def __init__(self):
        super().__init__('gesture_action_server')
 
        # 1. Configurações do YOLO e Lógica
        self.model = YOLO('yolov8n-pose.pt')
        self.conf_threshold = 0.5
        self.trigger_margin = 40
        self.stop_margin = 50
        # 2. Configurações ROS
        self.bridge = CvBridge()
        self.latest_frame = None
 
        # Callback Group permite que a subscrição de imagem e a action corram em paralelo
        self.cb_group = ReentrantCallbackGroup()
 
        # Subscrição da Imagem
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10,
            callback_group=self.cb_group
        )
 
        # Servidor de Ação
        self._action_server = ActionServer(
            self,
            YoloAction, # Trocar pelo tipo da tua Action real
            'yolo_action',
            self.execute_callback,
            callback_group=self.cb_group
        )
 
        self.get_logger().info('Gesture Action Server pronto e à espera de goals...')
 
    def image_callback(self, msg):
        """Guarda sempre o frame mais recente"""
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Erro ao converter imagem: {e}')
 
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executando Action...')
        feedback_msg = YoloAction.Feedback()
        result = YoloAction.Result()
 
        # Loop principal da Action
        while rclpy.ok():
            # Verifica se o cliente cancelou
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Action cancelada')
                return result
 
            # Verifica se temos imagem
            if self.latest_frame is None:
                self.get_logger().warning('À espera de imagens...')
                time.sleep(0.5)
                continue
 
            # Copia o frame para processar (thread safety básico)
            frame = self.latest_frame.copy()
            # --- LÓGICA YOLO (Adaptada do teu script) ---
            # Nota: O flip depende da tua câmara. Se a imagem vier invertida do sensor, mantém.
            # Se a imagem do sensor já estiver correta, remove o cv2.flip.
            frame = cv2.flip(frame, 1)
 
            results = self.model(frame, verbose=False)
            status_text = "WAITING..."
            detected_command = None # None, "STOP", "GO LEFT", "GO RIGHT"
 
            # Check if at least one person is detected
            if results[0].keypoints is not None and results[0].keypoints.data.shape[0] > 0:
                kpts = results[0].keypoints.data.cpu().numpy()[0]
 
                if len(kpts) >= 17:
                    # Extract points
                    l_shldr = kpts[5]
                    r_shldr = kpts[6]
                    l_wrist = kpts[9]
                    r_wrist = kpts[10]
 
                    stop_detected = False
 
                    # 1. CHECK STOP (Feedback)
                    is_left_stop = (l_wrist[2] > self.conf_threshold and l_shldr[2] > self.conf_threshold and 
                                    l_wrist[1] < (l_shldr[1] - self.stop_margin))
                    is_right_stop = (r_wrist[2] > self.conf_threshold and r_shldr[2] > self.conf_threshold and 
                                     r_wrist[1] < (r_shldr[1] - self.stop_margin))
 
                    if is_left_stop or is_right_stop:
                        status_text = "!!! STOP !!!"
                        stop_detected = True
                        detected_command = "STOP"
 
                    # 2. IF NOT STOP, CHECK DIRECTION (Result)
                    if not stop_detected:
                        # Check Left (User's Left Arm pointing to their Left -> Robot's Right logic?)
                        # Pela tua lógica: "GO LEFT"
                        if l_wrist[2] > self.conf_threshold and l_shldr[2] > self.conf_threshold:
                            dist = l_shldr[0] - l_wrist[0]
                            if dist > self.trigger_margin:
                                status_text = "<<< GO LEFT"
                                detected_command = "GO LEFT"
 
                        # Check Right
                        if r_wrist[2] > self.conf_threshold and r_shldr[2] > self.conf_threshold:
                            dist = r_wrist[0] - r_shldr[0]
                            if dist > self.trigger_margin:
                                status_text = "GO RIGHT >>>"
                                detected_command = "GO RIGHT"
 
            # --- DECISÃO DA ACTION ---
 
            self.get_logger().info(f'Status Atual: {status_text}')
 
            if detected_command == "GO LEFT" or detected_command == "GO RIGHT":
                # CONDIÇÃO DE SUCESSO: Encontrou uma direção válida
                result.command = detected_command
                goal_handle.succeed()
                self.get_logger().info(f'Action concluída com sucesso: {detected_command}')
                return result
            else:
                # CONDIÇÃO DE FEEDBACK: Waiting ou Stop
                # Envia feedback e continua o loop
                feedback_msg.status = status_text
                goal_handle.publish_feedback(feedback_msg)
                # Pequena pausa para não saturar o CPU
                time.sleep(0.1)
 
def main(args=None):
    rclpy.init(args=args)
 
    try:
        node = GestureActionServer()
        # Usamos MultiThreadedExecutor para permitir que a Action corra (blocking loop)
        # e a subscrição da imagem continue a atualizar em background.
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()
 
if __name__ == '__main__':
    main()