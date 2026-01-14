#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import yaml
import cv2
import sqlite3
from cv_bridge import CvBridge
from ultralytics import YOLO
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Bool, String, Float32, Int16
from sensor_msgs.msg import Image
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import time
import os
from ament_index_python.packages import get_package_share_directory

class Analizer(Node):
    def __init__(self):
        super().__init__('analizer')

        self.create_subscription(Image, 'image_left_to_analyze', self.picture_left_callback, 10)
        self.create_subscription(Image, 'image_right_to_analyze', self.picture_right_callback, 10)

        self.bridge = CvBridge()

        self.queue_left = []
        self.queue_right = []

        self.model_path = ""
        self.model = YOLO(self.model_path)

        self.focal_lenght = 400.0
        self.baseline = 0.12

        self.timer = self.create_timer(0.5, self.run)


    def picture_left_callback(self, msg: Image):
        self.image_left = msg.data
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.queue_left.append(cv_img)

    def picture_right_callback(self, msg: Image):
        self.image_right = msg.data
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.queue_right.append(cv_img)

    def run(self):
            if not self.queue:
                return
                
            img = self.queue.pop(0)
            results = self.model(img)
            
            for r in results:
                for box in r.boxes:
                    # 1. Obtener centro del objeto en la imagen IZQUIERDA
                    x_left = box.xywh[0][0].item()
                    y_left = box.xywh[0][1].item()
                    label = self.model.names[int(box.cls[0])]

                    # 2. Aquí viene el truco: 
                    # Debes buscar el MISMO objeto en la imagen derecha para tener x_right.
                    # Como simplificación, si el objeto está centrado:
                    disparity = 20.0 # Esto debe calcularse buscando el objeto en la foto derecha
                    
                    if disparity > 0:
                        z = (self.focal_length * self.baseline) / disparity
                        x = (x_left - (msg.width/2)) * z / self.focal_length
                        y = (y_left - (msg.height/2)) * z / self.focal_length
                        
                        # 3. Guardar en SQLITE
                        self.save_to_db(label, x, y, z)





def main():
    rclpy.init()
    rclpy.spin(Analizer())
    rclpy.shutdown()

if __name__ == "__main__":
    main()