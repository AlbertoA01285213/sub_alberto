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

class YoloSegmenterNode(Node):
    def __init__(self):
        super().__init__('tagging_analizer')

        self.group = ReentrantCallbackGroup()

        # --- Directorio para guardar imágenes ---
        self.save_path = os.path.expanduser(f"~/Desktop/Fotos_sub")
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)

        self.create_subscription(Image, 'image_left_to_analyze', self.picture_left_callback, 10, callback_group=self.group)
        self.create_subscription(Image, 'image_right_to_analyze', self.picture_right_callback, 10, callback_group=self.group)
        self.create_subscription(Pose, 'pose', self.pose_callback, 10)
        self.create_subscription(Int16, 'tagging_analyzer', self.tagging_analyzer_callback, 10)

        self.alineado_pub = self.create_publisher(Bool, 'alineado', 10)
        self.servoing_complete_pub = self.create_publisher(Bool, 'servoing_complete', 10)
        self.direccion_pub = self.create_publisher(Int16, 'direccion', 10)
        self.pic_analized_pub = self.create_publisher(Int16, 'picture_analized', 10)
        
        try:
            model_path = os.path.join(get_package_share_directory('uuv_vision'), 'models', 'tagging.pt')
            self.model = YOLO(model_path)
        except Exception as e:
            self.get_logger().error(f"No se pudo cargar el modelo: {e}")

        self.bridge = CvBridge()
        self.queue_left = []
        self.queue_right = []
        self.is_processing = False
        self.processing_lock = threading.Lock()
        self.activate = 0 
        self.objective = "tagging"
        self.count = 0

        # self.get_logger().info("Nodo tagging_analizer iniciado")
        self.create_timer(0.5, self.inference_loop, callback_group=self.group)


    def pose_callback(self, msg: Pose):
        try:
            quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
            _, _, yaw = euler_from_quaternion(quat)
            self.pose_actual = {'x': msg.position.x, 'y': msg.position.y, 'z': msg.position.z, 'yaw': yaw}
        except Exception:
            quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
            _, _, yaw = euler_from_quaternion(quat)
            self.pose_actual = {'x': msg.pose.position.x, 'y': msg.pose.position.y, 'z': msg.pose.position.z, 'yaw': yaw}

    def picture_left_callback(self, msg: Image):
        if self.activate == 1:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.processing_lock:
                self.queue_left.append(cv_img)
            self.pic_analized_pub.publish(Int16(data = 1))

    def picture_right_callback(self, msg: Image):
        if self.activate == 1:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.processing_lock:
                self.queue_right.append(cv_img)

    def tagging_analyzer_callback(self, msg: Int16):
        self.activate = msg.data

    def get_filtered_detections(self, results):
        detections = {}
        if results.boxes is None: return detections
        for i, box in enumerate(results.boxes):
            label = self.model.names[int(box.cls[0])]
            conf = float(box.conf[0])
            if conf < 0.5: continue
            xyxy = box.xyxy[0].cpu().numpy()
            x1, y1, x2, y2 = xyxy
            cx, cy = int(x1 + (x2-x1)/2), int(y1 + (y2-y1)/2)
            det_data = {'bbox': [int(x1), int(y1), int(x2-x1), int(y2-y1)], 'center': (cx, cy), 'conf': conf}
            if results.masks is not None:
                points = results.masks.xy[i].astype(np.int32)
                perimeter = cv2.arcLength(points, True)
                epsilon = 0.02 * perimeter
                polygon = cv2.approxPolyDP(points, epsilon, True)
                det_data['polygon'] = polygon
                det_data['full_mask'] = points # Para dibujo suave
            if label not in detections: detections[label] = []
            detections[label].append(det_data)
        return detections

    def inference_loop(self):
        if self.activate != 1 or not self.queue_left or not self.queue_right:
            return
        
        if self.is_processing: return
        
        try:
            self.get_logger().info("Nodo tagging_analizer iniciado")
            self.is_processing = True
            with self.processing_lock:
                img_l = self.queue_left.pop(0)
                self.queue_left.clear()
                self.queue_right.clear()

            res_l = self.model(img_l, verbose=False)[0]
            # Corregido el typo: era get_filtered_detections (minúscula)
            filtered_l = self.get_filtered_detections(res_l)
            
            debug_img = img_l.copy()

            if self.objective in filtered_l:
                best_det = max(filtered_l[self.objective], key=lambda x: x['conf'])
                
                if 'polygon' in best_det:
                    # --- LÓGICA DE ESQUINAS ---
                    points = best_det['polygon'].reshape(-1, 2)
                    top_points = points[points[:, 1].argsort()][:2]
                    
                    if top_points[0][0] < top_points[1][0]:
                        tl, tr = top_points[0], top_points[1]
                    else:
                        tr, tl = top_points[0], top_points[1]

                    error_perp = int(tr[1] - tl[1])
                    cx, cy = best_det['center']
                    error_x = int(cx - (img_l.shape[1] // 2))

                    # --- DIBUJO DE DEBUG ---
                    # 1. Dibujar el polígono de la máscara (Azul)
                    cv2.polylines(debug_img, [best_det['full_mask']], True, (255, 0, 0), 2)
                    
                    # 2. Dibujar las esquinas superiores encontradas
                    cv2.circle(debug_img, tuple(tl), 8, (0, 0, 255), -1) # Rojo: Top Left
                    cv2.circle(debug_img, tuple(tr), 8, (0, 255, 0), -1) # Verde: Top Right
                    
                    # 3. Poner texto con los errores
                    info_text = f"Err Perp: {error_perp} | Err X: {error_x}"
                    cv2.putText(debug_img, info_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                    
                    # 4. Guardar la imagen
                    save_name = os.path.expanduser(f"~/Desktop/Fotos_sub/fotos_tagging/tagging_yolo_{self.count}.jpg")
                    cv2.imwrite(save_name, debug_img)
                    self.count += 1

                    # --- CONTROL ---
                    msg_dir = Int16()
                    if abs(error_perp) > 10:
                        msg_dir.data = int(np.clip(error_perp * 0.5, -30, 30))
                        self.alineado_pub.publish(Bool(data=False))
                    elif abs(error_x) > 30:
                        msg_dir.data = int(np.clip(error_x * 0.1, -20, 20))
                        self.alineado_pub.publish(Bool(data=False))
                    else:
                        msg_dir.data = 0
                        self.alineado_pub.publish(Bool(data=True))
                    
                    self.direccion_pub.publish(msg_dir)

            self.is_processing = False

        except Exception as e:
            self.get_logger().error(f"Error en la inferencia: {e}")
            self.is_processing = False

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