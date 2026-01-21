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

class YoloSegmenterNode(Node):
    def __init__(self):
        super().__init__('analizer')

        # Usar el grupo para que las suscripciones y el timer no se bloqueen entre sí
        self.group = ReentrantCallbackGroup()

        self.create_subscription(Image, 'image_left_to_analyze', self.picture_left_callback, 10, callback_group=self.group)
        self.create_subscription(Image, 'image_right_to_analyze', self.picture_right_callback, 10, callback_group=self.group)

        self.bridge = CvBridge()
        self.queue_left = []
        self.queue_right = []

        # Cargar modelo con seguridad
        try:
            model_path = os.path.join(get_package_share_directory('uuv_vision'), 'models', 'best.pt')
            self.model = YOLO(model_path)
        except Exception as e:
            self.get_logger().error(f"No se pudo cargar el modelo: {e}")
            self.model = YOLO("yolov8n-seg.pt") # Modelo de emergencia

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

    def picture_left_callback(self, msg: Image):
        self.get_logger().info("📸 Llegó imagen a la cola IZQUIERDA")
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self.processing_lock:
            self.queue_left.append(cv_img)

    def picture_right_callback(self, msg: Image):
        self.get_logger().info("📸 Llegó imagen a la cola DERECHA")
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self.processing_lock:
            self.queue_right.append(cv_img)

    def inference_loop(self):
        # Log de latido para saber que el timer está vivo
        if not self.queue_left or not self.queue_right:
            self.get_logger().info("Esperando que ambas colas tengan imágenes...", once=True)
            return

        if self.is_processing:
            return

        try:
            self.is_processing = True
            with self.processing_lock:
                img_l = self.queue_left.pop(0)
                img_r = self.queue_right.pop(0)
                # Limpiar colas para procesar siempre lo más nuevo
                self.queue_left.clear()
                self.queue_right.clear()

            self.get_logger().info("Iniciando analisis")

            # img_l = self.undistort_image(img_l)
            # img_r = self.undistort_image(img_r)

            # 1. Inferencia
            res_l = self.model(img_l, verbose=False)[0]
            res_r = self.model(img_r, verbose=False)[0]

            filtered_l = self.get_filtered_detections(res_l)
            filtered_r = self.get_filtered_detections(res_r)

            n_raw_l = len(res_l.boxes) if res_l.boxes is not None else 0
            self.get_logger().info(f"YOLO encontró {n_raw_l} objetos en total (sin filtrar).")

            # Imagen para dibujar
            debug_img = img_l.copy()
            detecciones_reales = 0

            # 2. Match Estéreo
            for label, items_l in filtered_l.items():
                # Siempre intentamos buscar el par en la derecha
                best_l = max(items_l, key=lambda x: x['conf'])
                coords_3d = None
                
                if label in filtered_r:
                    items_r = filtered_r[label]
                    best_r = min(items_r, key=lambda x: abs(x['center'][1] - best_l['center'][1]))
                    coords_3d = self.calculate_3d_position(best_l, best_r)
                    
                    if coords_3d:
                        x_r, y_r, z_r = coords_3d
                        if z_r <= self.distancia_limite:
                            # CASO A: Cerca y con posición
                            self.draw_debug_info(debug_img, best_l, f"{label} {z_r:.1f}m", (0, 255, 0))
                            self.get_logger().info(f"✅ {label} detectado a {z_r:.2f}m")
                        else:
                            # CASO B: Detectado pero muy lejos (>20m)
                            self.draw_debug_info(debug_img, best_l, f"{label} FAR", (0, 165, 255))
                            self.get_logger().info(f"🔭 {label} detectado pero muy lejos ({z_r:.2f}m)")
                    else:
                        # CASO C: Solo se ve en una cámara o disparidad nula
                        self.draw_debug_info(debug_img, best_l, f"{label} ?", (0, 0, 255))
                        self.get_logger().warn(f"⚠️ {label} sin datos de profundidad.")

                detecciones_reales += 1

            # 3. Guardado solo si hay detecciones (para no llenar disco)
            if detecciones_reales > 0:
                folder_path = os.path.join(os.path.expanduser('~'), 'Desktop', 'Fotos_sub')
                if not os.path.exists(folder_path):
                    os.makedirs(folder_path)

                filename = f"analisis_{self.count:04d}.jpg"
                # save_path = os.path.join(folder_path, filename)
                save_path = os.path.join(get_package_share_directory('uuv_vision'), 'images', filename)
                cv2.imwrite(save_path, debug_img)
                self.get_logger().info(f"✅ Imagen guardada: {filename} con {detecciones_reales} objetos.")
                self.count += 1

        except Exception as e:
            self.get_logger().error(f"🚨 Error en loop: {e}")
        finally:
            self.is_processing = False

    
    def undistort_image(self, raw_img):
        h, w = raw_img.shape[:2]
        # Obtenemos una nueva matriz de cámara óptima para no perder bordes
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h)
        )
        undistorted = cv2.undistort(raw_img, self.camera_matrix, self.dist_coeffs, None, new_camera_matrix)
        
        # Opcional: Recortar la imagen si quedan bordes negros (roi)
        # x, y, w_roi, h_roi = roi
        # undistorted = undistorted[y:y+h_roi, x:x+w_roi]
        
        return undistorted
    
    def get_filtered_detections(self, result):
        filtered = {}
        if result.masks is None: 
            return filtered
            
        # Recorremos cajas y máscaras al mismo tiempo
        for box, mask in zip(result.boxes, result.masks):
            label = self.model.names[int(box.cls[0])]
            
            if label in self.target_names:
                if label not in filtered: 
                    filtered[label] = []
                
                # Obtenemos los puntos del polígono de la máscara
                polygon = mask.xy[0] # Lista de puntos (x, y)
                
                # Calculamos el centroide usando Momentos de OpenCV
                M = cv2.moments(polygon)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                else:
                    # Fallback al centro de la caja si la máscara falla
                    cx, cy = int(box.xywh[0][0]), int(box.xywh[0][1])

                pts = polygon.astype(np.int32).reshape((-1, 1, 2))
                hull = cv2.convexHull(pts)
                hull = hull.reshape(-1, 2)
                ext_left  = tuple(hull[hull[:, 0].argmin()])
                ext_right = tuple(hull[hull[:, 0].argmax()])
                ext_top   = tuple(hull[hull[:, 1].argmin()])
                ext_bot   = tuple(hull[hull[:, 1].argmax()])

                x, y, w, h = box.xywh[0].tolist()
                filtered[label].append({
                    'center': (cx, cy), # <-- ¡Ahora es el centroide!
                    'conf': box.conf[0].item(),
                    'bbox': (int(x - w/2), int(y - h/2), int(w), int(h)),
                    'polygon': polygon, # Guardamos el polígono para dibujar
                    'extremes': (ext_left, ext_right, ext_top, ext_bot)
                })
        return filtered

    def calculate_3d_position(self, det_l, det_r):
        x_left = det_l['center'][0]
        x_right = det_r['center'][0]
        disparity = abs(x_left - x_right)
        
        if disparity <= 0.5: 
            return None
        
        z = (self.focal_lenght * self.baseline) / disparity
        x_real = (x_left - self.img_center_x) * z / self.focal_lenght
        y_real = (det_l['center'][1] - self.img_center_y) * z / self.focal_lenght
        return (x_real, y_real, z)

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

'''
Coincidencia estereo: consiste en estimar un modelo 3D a partir de 2 imagenes.
Disparidad: diferencia en pixeles de la posicion del objeto entre camaras
f: distancia focal de la camara en pixeles
B: baseline la distancia entre camaras en metros

Z = f*B/d

field of view = pixeles ancho/(2*tan(fov/2))
*con fov horizontal

hay que aplicar stereoRectify()
initUndistortRectifyMap

extraer 
extremos
convex hull
puntos equidistantes

vision:
orb
akaze
sift

yolo-pose
openpose
hrnet

promedio vs ransac 
'''
