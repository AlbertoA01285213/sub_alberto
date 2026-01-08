#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
import threading
import cv2
import os # Necesario para rutas de archivos
from cv_bridge import CvBridge
from fastapi import FastAPI, WebSocket
from fastapi.responses import StreamingResponse, FileResponse # Agregamos FileResponse
import uvicorn
import asyncio

app = FastAPI()
bridge = CvBridge()

# Obtener la ruta donde está este script para buscar el index.html
# Asumiremos que index.html está en la misma carpeta o en una subcarpeta 'www'
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))

latest_data = {
    "pose": {"x": 0.0, "y": 0.0, "z": 0.0},
    "waypoint": {"x": 0.0, "y": 0.0},
}
last_image = None

class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_node')
        self.create_subscription(PoseStamped, '/pose', self.pose_cb, 10)
        self.create_subscription(Image, '/camera', self.image_cb, 10)

    def pose_cb(self, msg):
        print(f"Recibiendo Pose: {msg.pose.position.x}")
        latest_data["pose"] = {
            "x": round(msg.pose.position.x, 2),
            "y": round(msg.pose.position.y, 2),
            "z": round(msg.pose.position.z, 2)
        }

    def image_cb(self, msg):
        global last_image
        print("¡Recibiendo Imagen!")
        try:
            cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")
            _, buffer = cv2.imencode('.jpg', cv_img)
            last_image = buffer.tobytes()
        except Exception as e:
            self.get_logger().error(f"Error en cámara: {e}")

# --- RUTAS DE FASTAPI ---

# ESTA ES LA RUTA QUE TE FALTABA (Evita el 404)
@app.get("/")
async def get_index():
    # Cambia 'index.html' por la ruta real de tu archivo
    # Si está en una carpeta llamada 'www', usa os.path.join(SCRIPT_DIR, 'www', 'index.html')
    html_path = os.path.join(SCRIPT_DIR, 'index.html')
    return FileResponse(html_path)

    # html_path = os.path.join(current_dir, 'index.html')
        
    # if not os.path.exists(html_path):
    #     return {"error": f"No encontré el HTML en: {html_path}"}
        
    # return FileResponse(html_path)

@app.get("/video_feed")
async def video_feed():
    def generate():
        while True:
            if last_image:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + last_image + b'\r\n')
            import time
            time.sleep(0.04) # Limita a ~25 FPS para no saturar la CPU
    return StreamingResponse(generate(), media_type="multipart/x-mixed-replace; boundary=frame")

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            await websocket.send_json(latest_data)
            await asyncio.sleep(0.1)
    except Exception:
        # Esto maneja cuando cierras la pestaña del navegador
        await websocket.close()

def run_ros():
    node = DashboardNode()
    # Usamos un ejecutor que permite manejar varios callbacks a la vez
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init()
    thread = threading.Thread(target=run_ros, daemon=True)
    thread.start()
    uvicorn.run(app, host="0.0.0.0", port=8000)

if __name__ == '__main__':
    main()