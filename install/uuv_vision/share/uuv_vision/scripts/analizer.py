#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import cv2
import os
import threading
from cv_bridge import CvBridge
from ultralytics import YOLO
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Int16
from tf_transformations import euler_from_quaternion
import math

class YoloSegmenterNode(Node):
    def __init__(self):
        super().__init__('analizer')

        # Usar el grupo para que las suscripciones y el timer no se bloqueen entre sí
        self.group = ReentrantCallbackGroup()

        self.create_subscription(Image, 'image_left_to_analyze', self.picture_left_callback, 10, callback_group=self.group)
        self.create_subscription(Image, 'image_right_to_analyze', self.picture_right_callback, 10, callback_group=self.group)
        self.create_subscription(Pose, 'pose', self.pose_callback, 10)

        self.alineado_pub = self.create_publisher(Bool, 'alineado', 10) # Bandera para mission handler para determinar cuando el sub esta alineado con el obstaculo a ir
        self.servoing_complete_pub = self.create_publisher(Bool, 'servoing_complete', 10) # Bandera para mission handler para determinar cuando el sub llego al objetivo
        self.direccion_pub = self.create_publisher(Int16, 'direccion', 10)
        self.pic_analized_pub = self.create_publisher(Int16, 'picture_analized', 10) # Topico para indicar al mission handler que se esta analizando una foto 0 es sin foto, 1 es analizando y 2 es foto analizada

        self.bridge = CvBridge()
        self.queue_left = []
        self.queue_right = []

        self.known_obstacles = [] # Lista de diccionarios: {'name': str, 'x': float, 'y': float}
        self.min_dist_threshold = 2.0 # Distancia mínima en metros para considerar un objeto "nuevo"
        self.pose_actual = None

        # Cargar modelo con seguridad
        try:
            model_path = os.path.join(get_package_share_directory('uuv_vision'), 'models', 'best.pt')
            self.model = YOLO(model_path)
        except Exception as e:
            self.get_logger().error(f"No se pudo cargar el modelo: {e}")
            self.model = YOLO("yolov8n-seg.pt") # Modelo de emergencia

        self.db_path = os.path.join(get_package_share_directory('uuv_visualization'), 'data')
        if not os.path.exists(self.db_path):
            os.makedirs(self.db_path)
        self.db_path = os.path.join(self.db_path, 'obstacles.db')

        self.processing_lock = threading.Lock()
        self.is_processing = False

        # --- Parámetros ---
        self.focal_lenght = 672.0
        self.focal_pixels = 672.2
        self.baseline = 0.1
        self.img_width = 1920
        self.img_height = 1080
        self.img_center_x = 960.0 # Ajusta esto a la mitad de tu resolución real
        self.img_center_y = 540.0

        self.camera_matrix = np.array([
            [self.focal_pixels, 0, self.img_width/2],
            [0, self.focal_pixels, self.img_height/2],
            [0, 0, 1]
        ], dtype=np.float32)

        self.dist_coeffs = np.array([-0.15, 0.01, 0, 0, 0], dtype=np.float32)

        self.distancia_limite = 20.0
        self.count = 0
        self.target_names = ['slalom', 'tagging', 'octagon', 'mesa', 'path']

        self.timer = self.create_timer(0.5, self.inference_loop, callback_group=self.group)
        self.get_logger().info("🔍 Analizador YOLO iniciado y esperando imágenes...")

    def pose_callback(self, msg: Pose):
        # Si recibes geometry_msgs/Pose, no lleva el campo .pose intermedio
        try:
            quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
            _, _, yaw = euler_from_quaternion(quat)
            self.pose_actual = {
                'x': msg.position.x,
                'y': msg.position.y,
                'z': msg.position.z,
                'yaw': yaw
            }
        except Exception:
            # Por si acaso tu tópico sí es PoseStamped
            quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
            _, _, yaw = euler_from_quaternion(quat)
            self.pose_actual = {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z,
                'yaw': yaw
            }


    def picture_left_callback(self, msg: Image):
        self.get_logger().info("📸 Llegó imagen a la cola IZQUIERDA")
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self.processing_lock:
            self.queue_left.append(cv_img)

        self.pic_analized_pub.publish(Int16(data = 1))

        

    def picture_right_callback(self, msg: Image):
        self.get_logger().info("📸 Llegó imagen a la cola DERECHA")
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self.processing_lock:
            self.queue_right.append(cv_img)

    
    def get_filtered_detections(self, results):
        detections = {}
        if results.boxes is None:
            return detections

        for i, box in enumerate(results.boxes):
            label = self.model.names[int(box.cls[0])]
            conf = float(box.conf[0])
            
            if conf < 0.5: # Umbral de confianza
                continue

            # Obtener Bounding Box [x1, y1, x2, y2]
            xyxy = box.xyxy[0].cpu().numpy()
            x1, y1, x2, y2 = xyxy
            w, h = x2 - x1, y2 - y1
            cx, cy = int(x1 + w/2), int(y1 + h/2)

            det_data = {
                'bbox': [int(x1), int(y1), int(w), int(h)],
                'center': (cx, cy),
                'conf': conf,
            }

            # Si hay máscaras de segmentación, guardamos el polígono
            if results.masks is not None:
                det_data['polygon'] = results.masks.xy[i]

            if label not in detections:
                detections[label] = []
            detections[label].append(det_data)
        
        return detections


    def inference_loop(self):
        # Log de latido para saber que el timer está vivo
        if not self.queue_left or not self.queue_right:
            self.get_logger().info("Esperando que ambas colas tengan imágenes...", once=True)
            self.pic_analized_pub.publish(Int16(data = 0))
            return

        if self.is_processing:
            return

        try:
            self.pic_analized_pub.publish(Int16(data = 1)) # Publica que se esta analizando una foto

            self.is_processing = True
            with self.processing_lock:
                img_l = self.queue_left.pop(0)
                img_r = self.queue_right.pop(0)
                # Limpiar colas para procesar siempre lo más nuevo
                self.queue_left.clear()
                self.queue_right.clear()

            self.get_logger().info("Iniciando analisis")


            # 1. Inferencia
            res_l = self.model(img_l, verbose=False)[0]
            res_r = self.model(img_r, verbose=False)[0]

            filtered_l = self.get_filtered_detections(res_l)
            target_found = False

            debug_img = img_l.copy()

            if not filtered_l:
                self.get_logger().info("⚠️ No se detectó NADA en esta frame.")
            
            else:
                for label, items in filtered_l.items():
                    self.get_logger().info(f"🔎 Detectado: {label} (x{len(items)})")

                    for item in items:
                        # Dibujar en la imagen de debug
                        color = (0, 255, 0) if label == 'tagging' else (255, 120, 0)
                        self.draw_debug_info(debug_img, item, f"{label} {item['conf']:.2f}", color)

            if 'tagging' in filtered_l:
                target_found = True
                best_l = max(filtered_l['tagging'], key=lambda x: x['conf'])
                cx, cy = best_l['center']
                _, _, _, h = best_l['bbox']


                error_x = cx - self.img_center_x
                msg_dir = Int16()
                self.pic_analized_pub.publish(Int16(data = 2))

                self.get_logger().info(f"Error_x del tagging: {error_x}", once=True)

                if abs(error_x) < 50:
                    msg_dir.data = 0
                    self.direccion_pub.publish(msg_dir)

                    self.alineado_pub.publish(Bool(data = True))
                    self.get_logger().info("🎯 ¡OBJETIVO ALINEADO!")

                else: # (x - in_min)*(out_max - out_min) / (in_max - in_min) + out_min Map function de arduino
                    msg_dir.data = int(-1*((error_x - 0) * (30 - 0) / (800 - 0) + 0))
                    # msg_dir.data = 1 if error_x > 0 else -1
                    self.direccion_pub.publish(msg_dir)
                    self.get_logger().info(f"Angulo del tagging: {msg_dir.data}", once=True)
                    self.alineado_pub.publish(Bool(data = False))

                if h >= 850:
                    self.servoing_complete_pub.publish(Bool(data = True))
                    self.get_logger().info("✅ LLEGAMOS AL OBJETIVO")

                else:
                    self.servoing_complete_pub.publish(Bool(data = False))

            else:
                target_gound = False
                self.get_logger().warn("Target perdido")
                self.pic_analized_pub.publish(Int16(data = 3))

            if not target_found:
                self.get_logger().warn("🔭 Target 'tagging' perdido. Enviando comando de búsqueda (Giro 10°)")
                self.alineado_pub.publish(Bool(data=False))
                self.servoing_complete_pub.publish(Bool(data=False))
                
                msg_search = Int16()
                msg_search.data = 10 # Fuerza un giro constante para encontrar el objeto
                self.direccion_pub.publish(msg_search)
                self.pic_analized_pub.publish(Int16(data = 0))

            # 5. GUARDAR LA FOTO ANOTADA
            save_path = os.path.expanduser(f"~/Desktop/Fotos_sub/debug_yolo_{self.count}.jpg")
            cv2.imwrite(save_path, debug_img)
            self.count += 1

        except Exception as e:
            self.get_logger().error(f"🚨 Error en loop de inferencia: {e}")
        finally:
            self.is_processing = False


    def draw_debug_info(self, img, detection, label_text, color):
        x_tl, y_tl, w, h = detection['bbox']
        cx, cy = detection['center']
        polygon = detection['polygon'].astype(np.int32)

        # 1. Dibujar el polígono de la máscara (transparente)
        overlay = img.copy()
        cv2.fillPoly(overlay, [polygon], color)
        cv2.addWeighted(overlay, 0.3, img, 0.7, 0, img)

        # 2. Dibujar el contorno y el centroide
        cv2.polylines(img, [polygon], True, color, 2)
        cv2.circle(img, (cx, cy), 5, (255, 255, 255), -1) # Punto blanco en el centroide

        # 3. Texto informativo
        cv2.putText(img, label_text, (x_tl, y_tl - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

def main():
    rclpy.init()
    node = YoloSegmenterNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()