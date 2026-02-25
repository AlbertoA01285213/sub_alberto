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
from std_msgs.msg import Bool, Int16, String
from tf_transformations import euler_from_quaternion
import math

class YoloSegmenterNode(Node):
    def __init__(self):
        super().__init__('tagging_analizer')

        self.group = ReentrantCallbackGroup()

        self.create_subscription(Image, 'image_left_to_analyze', self.picture_left_callback, 10, callback_group=self.group)    # Topico que recibe la imagen de la camara izquierda
        self.create_subscription(Image, 'image_right_to_analyze', self.picture_right_callback, 10, callback_group=self.group)  # Topico que recibe la imagen de la camara derecha
        self.create_subscription(Pose, 'pose', self.pose_callback, 10)                                                         # Topico que recibe la pose del submarino
        self.create_subscription(Int16, 'tagging_analyzer', self.tagging_analyzer_callback, 10)                                # Topico que recibe la activacion del nodo

        self.alineado_pub = self.create_publisher(Bool, 'alineado', 10)                                                        # Indica que el submarino ya esta alineado para continuar
        self.servoing_complete_pub = self.create_publisher(Bool, 'servoing_complete', 10)                                      # Indica que el submarino ya esta listo para hacer la accion
        self.direccion_pub = self.create_publisher(Int16, 'direccion', 10)                                                     # Indica la manera en la que el sub se va a mover
        self.pic_analized_pub = self.create_publisher(Int16, 'picture_analized', 10)                                           # Indica si el submarino ya analizo la foto
        
        try:
            model_path = os.path.join(get_package_share_directory('uuv_vision'), 'models', 'tagging_model.pt')
            self.model = YOLO(model_path)

        except Exception as e:
            self.get_logger().error(f"No se pudo cargar el modelo: {e}")

        self.bridge = CvBridge()
        self.queue_left = []
        self.queue_right = []
        self.is_processing = False
        self.processing_lock = threading.Lock()
        self.activate = 0 
        self.objective = "tagging" # O inicializar vacío

        self.create_timer(0.5, self.inference_loop, callback_group=self.group)

    
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
        if self.activate == 1:
            self.get_logger().info("📸 Llegó imagen a la cola IZQUIERDA")
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.processing_lock:
                self.queue_left.append(cv_img)

            self.pic_analized_pub.publish(Int16(data = 1))

        

    def picture_right_callback(self, msg: Image):
        if self.activate == 1:
            self.get_logger().info("📸 Llegó imagen a la cola DERECHA")
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.processing_lock:
                self.queue_right.append(cv_img)

    def objective_callback(self, msg: String):
        self.objective = msg.data

    def tagging_analyzer_callback(self, msg: Int16):
        self.activate = msg.data

    def get_filtered_detections(self, results):
        detections = {}
        if results.boxes is None:
            return detections
        
        for i, box in enumerate(results.boxes):
            label = self.model.names[int(box.cls[0])]
            conf = float(box.conf[0])

            if conf < 0.5:
                continue

            xyxy = box.xyxy[0].cpu().numpy()
            x1, y1, x2, y2 = xyxy
            w, h = x2 - x1, y2 - y1
            cx, cy = int(x1 + w/2), int(y1 + h/2)

            det_data = {
                'bbox': [int(x1), int(y1), int(w), int(h)],
                'center': (cx, cy),
                'conf': conf, 
            }

            if results.masks is not None:
                perimeter = cv2.arcLength(results.masks.xy[i], True)
                epsilon = 0.02 * perimeter

                polygon = cv2.approxPolyDP(results.masks.xy[i], epsilon, True)
                det_data['polygon'] = polygon

            if label not in detections:
                detections[label] = []

            detections[label].append(det_data)

        return detections

    def inference_loop(self):
        if self.activate == 1:
            if not self.queue_left or not self.queue_right:
                self.get_logger().info("Esperando que ambas colas tengan imágenes...", once=True)
                self.pic_analized_pub.publish(Int16(data = 0))
                return

            if self.is_processing:
                return
            
            try:
                self.pic_analized_pub.publish(Int16(data = 1))

                self.is_processing = True
                with self.processing_lock:
                    img_l = self.queue_left.pop(0)
                    img_r = self.queue_right.pop(0)
                    # Limpiar colas para procesar siempre lo más nuevo
                    self.queue_left.clear()
                    self.queue_right.clear()

                self.get_logger().info("Iniciando analisis")

                res_l = self.model(img_l, verbose=False)[0]
                res_r = self.model(img_r, verbose=False)[0]

                filtered_l = self.get_filtered_Detections(res_l)
                target_found = False

                debug_img = img_l.copy()

                if not filtered_l:
                    self.get_logger().info("⚠️ No se detectó NADA en esta frame")

                else:
                    for label, items in filtered_l.items():
                        self.get_logger().info(f"🔎 Detectado: {label} (x{len(items)})")

                        if self.objective in filtered_l:
                            target_found = True
                            # Tomamos la detección con mejor confianza
                            best_det = max(filtered_l[self.objective], key=lambda x: x['conf'])
                            
                            if 'polygon' in best_det:
                                # 1. Obtener puntos del polígono y ordenarlos
                                points = best_det['polygon'].reshape(-1, 2)
                                
                                # Buscamos las dos esquinas superiores (menor valor de y)
                                # Ordenamos por y ascendente y tomamos los dos primeros
                                top_points = points[points[:, 1].argsort()][:2]
                                
                                # De esos dos, identificamos cuál es izquierda y cuál derecha por su x
                                if top_points[0][0] < top_points[1][0]:
                                    tl, tr = top_points[0], top_points[1] # top_left, top_right
                                else:
                                    tr, tl = top_points[0], top_points[1]

                                # 2. CALCULO DE ERRORES
                                # Error de Perpendicularidad: diferencia de altura entre esquinas
                                # En imagen, menor 'y' es más arriba. 
                                # Si error_perpendicular > 0, la derecha está más ABAJO que la izquierda.
                                error_perpendicular = tr[1] - tl[1]
                                
                                # Error de Centrado: qué tan lejos está el centro del objeto del centro de la imagen
                                cx, cy = best_det['center']
                                error_x = cx - (img_l.shape[1] // 2)

                                self.get_logger().info(f"Error Perp: {error_perpendicular} | Error X: {error_x}")

                                # 3. LÓGICA DE CONTROL (DIRECCIÓN)
                                msg_dir = Int16()
                                
                                # Definimos umbrales
                                umbral_perp = 10 # pixeles de diferencia aceptables
                                umbral_x = 30   # pixeles de centrado

                                if abs(error_perpendicular) > umbral_perp:
                                    # Si no estamos perpendiculares, priorizamos rotar/moverse lateralmente
                                    # Mapeo: si la derecha está más alta (error_perp < 0), girar a un lado
                                    msg_dir.data = int(np.clip(error_perpendicular * 0.5, -30, 30))
                                    self.alineado_pub.publish(Bool(data=False))
                                    self.get_logger().info("🔄 Corrigiendo perpendicularidad...")
                                
                                elif abs(error_x) > umbral_x:
                                    # Si ya estamos planos pero no centrados, nos centramos
                                    msg_dir.data = int(np.clip(error_x * 0.1, -20, 20))
                                    self.alineado_pub.publish(Bool(data=False))
                                    self.get_logger().info("🎯 Centrando objetivo...")
                                
                                else:
                                    # Estamos alineados y centrados
                                    msg_dir.data = 0
                                    self.alineado_pub.publish(Bool(data=True))
                                    self.get_logger().info("✅ ¡POSICIÓN PERFECTA!")

                                self.direccion_pub.publish(msg_dir)

                                # Dibujar para debug
                                cv2.circle(debug_img, tuple(tl), 5, (0, 0, 255), -1) # Rojo: Izquierda
                                cv2.circle(debug_img, tuple(tr), 5, (0, 255, 0), -1) # Verde: Derecha
            except Exception as e:
                self.pic_analized_pub.publish(Int16(data = 3))
                self.get_logger().error(f"Error en la inferencia: {e}")


        else:
            self.get_logger().info("Nodo desactivado")

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


